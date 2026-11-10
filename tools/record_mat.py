#!/usr/bin/env python3
"""Record the radio environment to a MATLAB .mat file of interleaved I/Q samples.

    record_mat.py --preset single --pi raha-pi-lan --connect 192.168.77.2:5555 --out single.mat
    record_mat.py --preset multi  --pi raha-pi-lan --connect 192.168.77.2:5555 --out multi.mat
    record_mat.py --center-mhz 2402.5 --rate 1e6 --connect 192.168.77.2:5555 --out tuned.mat

The presets are the lab's receiver configurations:

    single   2402 MHz, 2 MS/s     one BLE advertising channel (37)
    multi    2411 MHz, 20 MS/s    BLE channel 37 and the data channels above it

The file holds `iq`, a row vector of int16 [i0, q0, i1, q1, ...] ("complex short"), and
`config`, a struct describing the recording. The ADC's full scale is 16384 in these units,
leaving headroom for the resampling filter. In MATLAB:

    load single.mat
    x = double(iq(1:2:end)) + 1j * double(iq(2:2:end));
    t = (0:numel(x) - 1) / config.sample_rate_hz;

The ESP32 records bursts, not a continuous stream: about 1 ms at a time, separated by
roughly 28 ms while the burst is sent. By default the file keeps true time, with each
burst at the moment it was captured and zeros in between, so it spans the full duration.
`config.burst_start` and `config.burst_length` say which samples are real. With
`--gaps join` the bursts follow one another instead, and the file is much shorter.

The ESP32 samples at 16, 40 or 80 MS/s and tunes to whole MHz, so the recording is made at
a native rate and tuning, then shifted and resampled to the requested center and rate.
Run this on the PC. With --pi it starts pi_receiver.py on the Pi over SSH, and otherwise
it prints the command to run there.
"""

import argparse
import datetime
import math
import pathlib
import shlex
import socket
import subprocess
import sys
import time

import numpy as np
from scipy import io, signal

import pi_receiver

# Gains are table indices, not dB, so the slides' USRP gains don't carry over. At 2411 MHz,
# strong Wi-Fi saturated the ADC in 4% of bursts at gain 48 and never at 40.
PRESETS = {
    "single": {"center_mhz": 2402.0, "rate_hz": 2_000_000, "gain": 48},
    "multi": {"center_mhz": 2411.0, "rate_hz": 20_000_000, "gain": 40},
}
DEFAULT_GAIN = 48
DEFAULT_DURATION_S = 5.0

HARDWARE_RATES_HZ = (16_000_000, 40_000_000, 80_000_000)
ANALOG_HALF_BANDWIDTH_MHZ = 10.0  # Close to flat inside this, from the band sweeps
USABLE_FRACTION = 0.9  # Of the hardware rate's Nyquist band, leaving room for filtering
DC_CLEARANCE_MHZ = 0.5  # Keep the tuned frequency this far outside the recorded band
DC_STEP_MHZ = 2  # How far to move the tuning to keep its DC offset out of the band

ADC_FULL_SCALE = 512  # Signed 10-bit samples
# 10 bits to 16 with 6 dB of headroom: the ADC's full scale becomes 16384, since resampling
# can overshoot it, by up to 685 of 512 counts in testing.
INT16_SCALE = 32
MAT_VARIABLE_LIMIT = 2 ** 31 - 1  # Bytes per variable in a version 5 .mat file
TIMESTAMP_WRAP = 1 << 32
CONNECT_TIMEOUT_S = 20
PI_EXTRA_S = 4  # How much longer than the recording the Pi streams for


def usable_half_band_mhz(hardware_rate_hz):
    return min(hardware_rate_hz / 2e6 * USABLE_FRACTION, ANALOG_HALF_BANDWIDTH_MHZ)


def plan(center_mhz, rate_hz):
    """Choose a native sample rate and whole-MHz tuning that cover the requested band.

    The tuned frequency carries the receiver's DC offset and LO leakage, so if there is
    room it is moved outside the band, the way --ble does."""
    half_band = rate_hz / 2e6
    for hardware_rate in HARDWARE_RATES_HZ:
        if hardware_rate < rate_hz:
            continue
        usable = usable_half_band_mhz(hardware_rate)
        tuned = round(center_mhz) + DC_STEP_MHZ
        offset = abs(tuned - center_mhz)
        if offset - half_band >= DC_CLEARANCE_MHZ and offset + half_band <= usable:
            return tuned, hardware_rate
        tuned = round(center_mhz)
        if abs(tuned - center_mhz) + half_band <= usable:
            return tuned, hardware_rate
    sys.exit(f"{rate_hz / 1e6:g} MS/s around {center_mhz:g} MHz doesn't fit the receiver's "
             f"+/- {ANALOG_HALF_BANDWIDTH_MHZ:g} MHz analog bandwidth")


def pi_command(script, tuned_mhz, hardware_rate_hz, gain, duration_s, address):
    host, _, port = address.rpartition(":")
    return (f"python3 {shlex.quote(script)} --freq {tuned_mhz} --rate {hardware_rate_hz // 1_000_000} "
            f"--gain {gain} --samples {pi_receiver.MAX_SAMPLES} --bursts 0 "
            f"--duration {math.ceil(duration_s + PI_EXTRA_S)} --tcp-port {port} --tcp-bind {host}")


def wait_for_server(address, process):
    host, _, port = address.rpartition(":")
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while time.monotonic() < deadline:
        if process and process.poll() is not None:
            sys.exit(f"pi_receiver.py exited on the Pi:\n{process.stdout.read().decode(errors='replace')}")
        try:
            socket.create_connection((host, int(port)), timeout=1).close()
            return
        except OSError:
            time.sleep(0.3)
    sys.exit(f"nothing is serving on {address}")


class Converter:
    """Shifts the requested center to 0 Hz and resamples a burst to the output rate."""

    def __init__(self, center_hz, tuned_hz, input_rate, output_rate):
        self.offset_hz = center_hz - tuned_hz
        self.input_rate = input_rate
        divisor = math.gcd(int(input_rate), int(output_rate))
        self.up = int(output_rate) // divisor
        self.down = int(input_rate) // divisor
        # resample_poly assumes zeros beyond the burst, so its first and last few outputs
        # ramp in and out. Those are dropped.
        self.edge = math.ceil(10 * max(self.up, self.down) / self.down) + 1

    def convert(self, words):
        i, q, gain = pi_receiver.unpack_iq(words)
        x = (i + 1j * q).astype(np.complex128)
        n = np.arange(len(x))
        x *= np.exp(-2j * np.pi * self.offset_hz * n / self.input_rate)
        y = signal.resample_poly(x, self.up, self.down)
        return y[self.edge:len(y) - self.edge], gain


def encode(samples, sample_format):
    """Interleave I and Q. Returns the array and how many values were clipped."""
    interleaved = np.empty(2 * len(samples))
    interleaved[0::2] = samples.real
    interleaved[1::2] = samples.imag
    if sample_format == "float32":
        return (interleaved / ADC_FULL_SCALE).astype(np.float32), 0
    scaled = np.round(interleaved * INT16_SCALE)
    clipped = int(np.sum(np.abs(scaled) > 32767))
    return np.clip(scaled, -32768, 32767).astype(np.int16), clipped


def record(args, bursts, metadata):
    tuned_hz = metadata["center_mhz"] * 1e6
    input_rate = metadata["sample_rate_hz"]
    center_hz = args.center_mhz * 1e6
    output_rate = args.rate
    converter = Converter(center_hz, tuned_hz, input_rate, output_rate)

    half_band = output_rate / 2
    if abs(center_hz - tuned_hz) + half_band > usable_half_band_mhz(input_rate) * 1e6:
        sys.exit(f"The Pi is tuned to {tuned_hz / 1e6:g} MHz at {input_rate / 1e6:g} MS/s, which "
                 f"doesn't cover {output_rate / 1e6:g} MS/s around {args.center_mhz:g} MHz")
    if abs(center_hz - tuned_hz) < half_band:
        print(f"note: the receiver's DC offset at {tuned_hz / 1e6:g} MHz is inside the recorded band, "
              f"at {(tuned_hz - center_hz) / 1e6:+g} MHz", file=sys.stderr)

    total = int(round(args.duration * output_rate))
    dtype = np.float32 if args.format == "float32" else np.int16
    if args.gaps == "zeros":
        if 2 * total * np.dtype(dtype).itemsize > MAT_VARIABLE_LIMIT:
            sys.exit("That recording would exceed the .mat format's 2 GB per variable. "
                     "Shorten it, lower the rate, or use int16.")
        iq = np.zeros(2 * total, dtype=dtype)
    else:
        pieces = []

    starts, lengths, times, numbers = [], [], [], []
    gain_low, gain_high = 127, 0
    clipped = 0
    first_time = None
    position = 0
    previous_number = None
    missing = 0

    for burst in bursts:
        if first_time is None:
            first_time = burst.timestamp_us
        elapsed_us = (burst.timestamp_us - first_time) % TIMESTAMP_WRAP
        if elapsed_us >= args.duration * 1e6:
            break
        if previous_number is not None and burst.number != previous_number + 1:
            missing += burst.number - previous_number - 1
        previous_number = burst.number

        samples, gain = converter.convert(burst.words)
        values, clip = encode(samples, args.format)
        clipped += clip
        gain_low, gain_high = min(gain_low, int(gain.min())), max(gain_high, int(gain.max()))

        if args.gaps == "zeros":
            # The timestamp marks the same point in every burst, so the spacing is exact.
            start = max(int(round(elapsed_us * output_rate / 1e6)) + converter.edge, position)
            count = min(len(samples), total - start)
            if count <= 0:
                break
            iq[2 * start:2 * (start + count)] = values[:2 * count]
        else:
            start, count = position, len(samples)
            pieces.append(values)
        position = start + count

        starts.append(start + 1)  # 1-based, for MATLAB
        lengths.append(count)
        times.append(start / output_rate)
        numbers.append(burst.number)

    if first_time is None:
        sys.exit("FAIL: no bursts arrived")
    if args.gaps == "join":
        iq = np.concatenate(pieces)
        total = len(iq) // 2

    captured = int(np.sum(lengths))
    config = {
        "center_frequency_hz": center_hz,
        "sample_rate_hz": float(output_rate),
        "data_format": "complex short: int16, interleaved [I0 Q0 I1 Q1 ...]" if args.format == "int16"
                       else "complex float: float32, interleaved [I0 Q0 I1 Q1 ...]",
        "adc_full_scale": float(ADC_FULL_SCALE * INT16_SCALE if args.format == "int16" else 1.0),
        "gain_index": args.gain,
        "gain_note": "ESP32 RX gain table index, not dB; noise rises about 0.9 dB per index near the top",
        "measured_gain_index_range": np.array([gain_low, gain_high], dtype=float),
        "duration_s": total / output_rate,
        "num_samples": float(total),
        "gaps": args.gaps,
        "burst_start": np.array(starts, dtype=float),
        "burst_length": np.array(lengths, dtype=float),
        "burst_time_s": np.array(times),
        "burst_number": np.array(numbers, dtype=float),
        "captured_fraction": captured / total,
        "missing_bursts": float(missing),
        "clipped_values": float(clipped),
        "tuned_frequency_hz": tuned_hz,
        "hardware_sample_rate_hz": float(input_rate),
        "receiver": "ESP32 modem I/Q capture, 10-bit samples",
        "recorded_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
        "notes": ("Bursts are real samples; with gaps = 'zeros' everything between them is filled "
                  "with zeros. burst_start is a 1-based index into the complex samples."),
    }
    io.savemat(args.out, {args.variable: iq.reshape(1, -1), "config": config},
               do_compression=args.compress, oned_as="row")

    print(f"recorded {len(starts)} bursts: {captured} real samples of {total} "
          f"({captured / total * 100:.1f}% of {total / output_rate:.2f} s) at {output_rate / 1e6:g} MS/s "
          f"centered on {args.center_mhz:g} MHz")
    print(f"tuned to {tuned_hz / 1e6:g} MHz at {input_rate / 1e6:g} MS/s, gain index {args.gain}, "
          f"clipped values: {clipped}, missing bursts: {missing}")
    print(f"saved {args.out} ({args.out.stat().st_size / 1e6:.1f} MB), variable '{args.variable}' "
          f"with {2 * total} {args.format} values")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connect", required=True, metavar="HOST:PORT",
                        help="the Pi's TCP stream, such as 192.168.77.2:5555")
    parser.add_argument("--out", type=pathlib.Path, required=True, help="the .mat file to write")
    parser.add_argument("--preset", choices=PRESETS, help="a configuration from the lab's slides")
    parser.add_argument("--center-mhz", type=float, help="center frequency of the recording")
    parser.add_argument("--rate", type=float, help="sample rate of the recording, in S/s")
    parser.add_argument("--gain", type=int,
                        help=f"RX gain table index, 0-81 (default: the preset's, or {DEFAULT_GAIN})")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S,
                        help="seconds to record (default: %(default)s)")
    parser.add_argument("--format", choices=("int16", "float32"), default="int16",
                        help="int16 is complex short, float32 is complex float (default: %(default)s)")
    parser.add_argument("--gaps", choices=("zeros", "join"), default="zeros",
                        help="keep true time with zeros between bursts, or join the bursts "
                             "(default: %(default)s)")
    parser.add_argument("--variable", default="iq", help="name of the samples in the file (default: %(default)s)")
    parser.add_argument("--compress", action="store_true",
                        help="compress the file. The gaps' zeros compress very well, but then the "
                             "file size no longer shows how much the data format takes")
    parser.add_argument("--pi", metavar="SSH_HOST", help="start pi_receiver.py on the Pi over SSH")
    parser.add_argument("--pi-script", default="pi_receiver.py",
                        help="path of pi_receiver.py on the Pi (default: %(default)s)")
    args = parser.parse_args()

    if args.preset:
        settings = PRESETS[args.preset]
        args.center_mhz = args.center_mhz or settings["center_mhz"]
        args.rate = args.rate or settings["rate_hz"]
        args.gain = settings["gain"] if args.gain is None else args.gain
    if args.gain is None:
        args.gain = DEFAULT_GAIN
    if not (args.center_mhz and args.rate):
        parser.error("give --preset, or --center-mhz and --rate")
    if not 0 <= args.gain <= 81:
        parser.error("--gain must be 0-81")
    if args.out.suffix != ".mat":
        parser.error("--out must end in .mat")

    tuned_mhz, hardware_rate = plan(args.center_mhz, args.rate)
    command = pi_command(args.pi_script, tuned_mhz, hardware_rate, args.gain, args.duration, args.connect)

    process = None
    if args.pi:
        print(f"starting on {args.pi}: {command}")
        process = subprocess.Popen(["ssh", "-o", "BatchMode=yes", args.pi, command],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    else:
        print(f"run this on the Pi first, then this recorder:\n  {command}")
    wait_for_server(args.connect, process)

    stats = pi_receiver.Stats()
    bursts = pi_receiver.pull_bursts(args.connect, stats)
    metadata = next(bursts)
    if metadata["channels"][0] or metadata["center_mhz"] != tuned_mhz or \
            metadata["sample_rate_hz"] != hardware_rate:
        print(f"note: the Pi is streaming {metadata['center_mhz']} MHz at "
              f"{metadata['sample_rate_hz'] / 1e6:g} MS/s, not the planned {tuned_mhz} MHz at "
              f"{hardware_rate / 1e6:g} MS/s. Converting from what it sends.", file=sys.stderr)
    if metadata["gain_requested"] != args.gain:
        args.gain = metadata["gain_requested"]
        print(f"note: the Pi is using gain {args.gain}", file=sys.stderr)
    if len(metadata["channels"]) > 1:
        sys.exit("The Pi is running a sweep, which can't be recorded as one frequency")

    try:
        record(args, bursts, metadata)
    finally:
        if process:
            try:
                process.wait(timeout=args.duration + PI_EXTRA_S + 10)
            except subprocess.TimeoutExpired:
                process.terminate()
    if stats.crc_errors:
        sys.exit(f"FAIL: {stats.crc_errors} bursts arrived corrupted and were left out")


if __name__ == "__main__":
    main()
