#!/usr/bin/env python3
"""Bridge the Pi's I/Q stream to standard SDR tools, as a raw stream or an rtl_tcp server.

    sdr_source.py --connect 192.168.77.2:5555 --out - --format cf32 | your_tool
    sdr_source.py --connect 192.168.77.2:5555 --out /tmp/iq.fifo --mkfifo --rate 2048000
    sdr_source.py --connect 192.168.77.2:5555 --out capture.cf32 --count 200
    sdr_source.py --connect 192.168.77.2:5555 --rtl-tcp 1234

Those tools expect a continuous stream, but the ESP32 captures bursts: about 1 ms in every
28 ms. There are two ways to present that, chosen with --gaps:

    join    Bursts directly after one another. A spectrum looks right, but time is compressed
            and the signal jumps between bursts, so demodulators won't work across them.
    zeros   Silence in the gaps, using the capture timestamps. Time is true, so the data rate
            matches the nominal sample rate, but the power reads far lower than it really is.

The Pi's tuning and gain are set when it starts streaming, so this can't change them. An
rtl_tcp client can only retune inside the captured band, which is done digitally here, and
its gain commands are ignored.
"""

import argparse
import math
import os
import pathlib
import socket
import stat
import struct
import sys
import threading

import numpy as np
from scipy import signal

import pi_receiver

FULL_SCALE = 512  # Signed 10-bit samples
FORMATS = {"cf32": "<f4", "cs16": "<i2", "cu8": "u1"}

# rtl_tcp: the server sends a 12-byte header, then unsigned 8-bit I/Q. The client sends
# commands of 1 byte plus a big-endian 32-bit value.
RTL_MAGIC = b"RTL0"
RTL_TUNER_R820T = 5
RTL_GAIN_COUNT = 29
RTL_SET_FREQUENCY = 0x01
RTL_SET_SAMPLE_RATE = 0x02
RTL_DEFAULT_RATE = 2_048_000
RTL_COMMAND = struct.Struct(">BI")

TIMESTAMP_WRAP = 1 << 32


def log(message):
    print(message, file=sys.stderr, flush=True)


def to_complex(burst):
    """Convert raw words to complex samples, with +/- 1.0 as full scale."""
    i, q, _ = pi_receiver.unpack_iq(burst.words)
    return (i + 1j * q) / FULL_SCALE


def retune(iq, input_rate, output_rate, offset_hz):
    """Move `offset_hz` to the center, then change sample rate with a polyphase filter."""
    if offset_hz:
        n = np.arange(len(iq))
        iq = iq * np.exp(-2j * np.pi * offset_hz * n / input_rate)
    if output_rate == input_rate:
        return iq
    divisor = math.gcd(int(input_rate), int(output_rate))
    return signal.resample_poly(iq, int(output_rate) // divisor, int(input_rate) // divisor)


def encode(iq, sample_format, gain):
    """Interleave I and Q in the output format. Returns the bytes and the clipped fraction."""
    scaled = iq * gain
    interleaved = np.empty(2 * len(scaled), dtype=np.float32)
    interleaved[0::2] = scaled.real
    interleaved[1::2] = scaled.imag
    clipped = np.mean(np.abs(interleaved) > 1.0) if len(interleaved) else 0.0
    interleaved = np.clip(interleaved, -1.0, 1.0)

    if sample_format == "cf32":
        return interleaved.astype("<f4").tobytes(), clipped
    if sample_format == "cs16":
        return np.round(interleaved * 32767).astype("<i2").tobytes(), clipped
    return np.round(interleaved * 127.5 + 127.5).astype("u1").tobytes(), clipped


def silence(sample_count, sample_format):
    """Bytes for samples of zero. For unsigned 8-bit, zero is the midpoint of the range."""
    if sample_format == "cu8":
        return b"\x80" * (2 * sample_count)
    return bytes(2 * sample_count * np.dtype(FORMATS[sample_format]).itemsize)


class GapFiller:
    """Works out how many zero samples belong before each burst, from its timestamp."""

    def __init__(self):
        self.next_time_us = None

    def samples_before(self, burst, input_rate, output_rate, output_samples):
        duration_us = len(burst.words) / input_rate * 1e6
        gap = 0
        if self.next_time_us is not None:
            elapsed_us = (burst.timestamp_us - self.next_time_us) % TIMESTAMP_WRAP
            gap = int(elapsed_us * output_rate / 1e6)
        self.next_time_us = (burst.timestamp_us + duration_us) % TIMESTAMP_WRAP
        return gap


def connect(args):
    stats = pi_receiver.Stats()
    bursts = pi_receiver.pull_bursts(args.connect, stats)
    metadata = next(bursts)
    if len(metadata["channels"]) > 1:
        sys.exit("That stream is a sweep, whose frequency changes on every burst. Tools "
                 "expecting one frequency can't use it. Start the Pi on one channel or --freq.")
    log(f"connected: {metadata['center_mhz']} MHz, {metadata['sample_rate_hz'] / 1e6:g} MS/s, "
        f"{metadata['burst_samples']} samples per burst, gain {metadata['gain_requested']}")
    return bursts, metadata


def open_output(args):
    if str(args.out) == "-":
        return sys.stdout.buffer
    if args.mkfifo:
        if not args.out.exists():
            os.mkfifo(args.out)
        elif not stat.S_ISFIFO(args.out.stat().st_mode):
            sys.exit(f"{args.out} exists and isn't a FIFO")
        log(f"waiting for a reader on {args.out}")
    return open(args.out, "wb")


def run_stream(args):
    bursts, metadata = connect(args)
    input_rate = metadata["sample_rate_hz"]
    output_rate = args.rate or input_rate
    gain = 10 ** (args.digital_gain_db / 20)
    center_hz = metadata["center_mhz"] * 1e6
    log(f"writing {args.format} at a nominal {output_rate / 1e6:g} MS/s, centered on "
        f"{center_hz / 1e6:g} MHz, with --gaps {args.gaps}")

    gaps = GapFiller()
    written = 0
    count = 0
    clipped_total = 0.0
    output = open_output(args)
    try:
        for burst in bursts:
            samples = retune(to_complex(burst), input_rate, output_rate, 0)
            if args.gaps == "zeros":
                output.write(silence(gaps.samples_before(burst, input_rate, output_rate, len(samples)),
                                     args.format))
            data, clipped = encode(samples, args.format, gain)
            output.write(data)
            output.flush()
            written += len(samples)
            count += 1
            clipped_total += clipped
            if args.count and count >= args.count:
                break
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    finally:
        try:
            if output is not sys.stdout.buffer:
                output.close()
        except BrokenPipeError:
            pass
    log(f"bursts: {count}, samples: {written}, clipped: {clipped_total / max(count, 1) * 100:.2f}%")


class RtlClient:
    """One connected rtl_tcp client, and what it has asked for."""

    def __init__(self, connection, center_hz, input_rate):
        self.connection = connection
        self.frequency_hz = center_hz
        self.rate = RTL_DEFAULT_RATE
        self.center_hz = center_hz
        self.input_rate = input_rate
        self.gaps = GapFiller()
        self.alive = True
        connection.sendall(RTL_MAGIC + struct.pack(">II", RTL_TUNER_R820T, RTL_GAIN_COUNT))
        threading.Thread(target=self._read_commands, daemon=True).start()

    def _read_commands(self):
        buffer = b""
        try:
            while True:
                chunk = self.connection.recv(RTL_COMMAND.size)
                if not chunk:
                    break
                buffer += chunk
                while len(buffer) >= RTL_COMMAND.size:
                    command, value = RTL_COMMAND.unpack_from(buffer)
                    buffer = buffer[RTL_COMMAND.size:]
                    self._handle(command, value)
        except OSError:
            pass
        self.alive = False

    def _handle(self, command, value):
        if command == RTL_SET_FREQUENCY:
            limit = self.input_rate / 2
            if abs(value - self.center_hz) >= limit:
                log(f"client asked for {value / 1e6:g} MHz, which is outside the captured "
                    f"{(self.center_hz - limit) / 1e6:g}-{(self.center_hz + limit) / 1e6:g} MHz. "
                    "Restart the Pi on another frequency.")
                return
            self.frequency_hz = value
            log(f"client tuned to {value / 1e6:g} MHz")
        elif command == RTL_SET_SAMPLE_RATE:
            if not 0 < value <= self.input_rate:
                log(f"client asked for {value} S/s, above the captured rate. Ignoring it.")
                return
            self.rate = value
            log(f"client set {value / 1e6:g} MS/s")

    def send(self, burst, digital_gain, use_gaps):
        samples = retune(to_complex(burst), self.input_rate, self.rate,
                         self.frequency_hz - self.center_hz)
        if use_gaps:
            gap = self.gaps.samples_before(burst, self.input_rate, self.rate, len(samples))
            self.connection.sendall(silence(gap, "cu8"))
        data, _ = encode(samples, "cu8", digital_gain)
        self.connection.sendall(data)


def run_rtl_tcp(args):
    bursts, metadata = connect(args)
    center_hz = metadata["center_mhz"] * 1e6
    input_rate = metadata["sample_rate_hz"]
    gain = 10 ** (args.digital_gain_db / 20)

    listener = socket.create_server((args.bind, args.rtl_tcp))
    listener.setblocking(False)
    log(f"rtl_tcp server on {args.bind}:{args.rtl_tcp}, with --gaps {args.gaps}")

    client = None
    try:
        for burst in bursts:
            try:
                connection, address = listener.accept()
            except BlockingIOError:
                connection = None
            if connection:
                connection.setblocking(True)
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                if client:
                    client.connection.close()
                client = RtlClient(connection, center_hz, input_rate)
                log(f"client connected: {address[0]}")

            if client:
                try:
                    if not client.alive:
                        raise OSError
                    client.send(burst, gain, args.gaps == "zeros")
                except OSError:
                    log("client disconnected")
                    client.connection.close()
                    client = None
    except KeyboardInterrupt:
        pass
    finally:
        listener.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connect", required=True, metavar="HOST:PORT", help="the Pi's TCP stream")
    parser.add_argument("--out", type=pathlib.Path, help="output file or FIFO, or - for stdout")
    parser.add_argument("--format", choices=FORMATS, default="cf32",
                        help="sample format for --out (default: %(default)s)")
    parser.add_argument("--mkfifo", action="store_true", help="create --out as a named pipe")
    parser.add_argument("--rate", type=int, help="resample --out to this rate, in S/s")
    parser.add_argument("--count", type=int, default=0, help="stop after this many bursts")
    parser.add_argument("--rtl-tcp", type=int, metavar="PORT", help="serve rtl_tcp on this port")
    parser.add_argument("--bind", default="127.0.0.1", help="rtl_tcp address (default: %(default)s)")
    parser.add_argument("--gaps", choices=("join", "zeros"), default="join",
                        help="how to present the time between bursts (default: %(default)s)")
    parser.add_argument("--digital-gain-db", type=float, default=0.0,
                        help="scale the samples up, which 8-bit output often needs because of "
                             "its small range (default: %(default)s)")
    args = parser.parse_args()

    if bool(args.out) == bool(args.rtl_tcp):
        parser.error("give either --out or --rtl-tcp")
    if args.mkfifo and (not args.out or str(args.out) == "-"):
        parser.error("--mkfifo needs an --out path")
    if args.rtl_tcp:
        run_rtl_tcp(args)
    else:
        run_stream(args)


if __name__ == "__main__":
    main()
