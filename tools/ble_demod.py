#!/usr/bin/env python3
"""Demodulate BLE advertising packets from I/Q samples, using only NumPy and SciPy.

The physical layer, stage by stage:

    I/Q burst -> channel filter and decimation -> FM discriminator -> Access Address
    correlation -> symbol timing and bit slicing -> de-whitening -> CRC-24 -> PDU parser

Usage:
    ble_demod.py --self-test
    ble_demod.py capture.npy
    ble_demod.py --connect 192.168.77.2:5555

Capture with `pi_receiver.py --ble 37 --gain 48`. The ESP32 only captures about 1 ms in every
28 ms, so this samples the channel: it decodes packets that land fully inside a burst, and
misses the rest.
"""

import argparse
import json
import pathlib
import sys

import numpy as np
from scipy import signal

import pi_receiver

SYMBOL_RATE_HZ = 1_000_000  # BLE 1M PHY
SAMPLES_PER_SYMBOL = 4
DEVIATION_HZ = 250_000  # Modulation index h = 0.5, so +/- 250 kHz
GAUSSIAN_BT = 0.5
CHANNEL_FILTER_CUTOFF_HZ = 800_000
CHANNEL_FILTER_TAPS_PER_DECIMATION = 16

ACCESS_ADDRESS = 0x8E89BED6  # Sent least significant bit first, so 0xD6 goes first
ACCESS_ADDRESS_BITS = 32
MAX_ACCESS_ADDRESS_ERRORS = 2
SYNC_THRESHOLD = 0.6  # Normalized correlation that makes a sample worth checking
TIMING_LOOP_GAIN = 0.08

CRC_POLYNOMIAL = 0x00065B  # x^24 + x^10 + x^9 + x^6 + x^4 + x^3 + x + 1, without x^24
CRC_INIT = 0x555555
HEADER_BITS = 16
CRC_BITS = 24
MAX_PDU_PAYLOAD = 255

FULL_SCALE = 512  # Signed 10-bit samples


# --- Stage 1: channel selection ------------------------------------------------------

def decimation_factor(sample_rate_hz):
    factor = sample_rate_hz / (SAMPLES_PER_SYMBOL * SYMBOL_RATE_HZ)
    if factor != int(factor) or factor < 1:
        sys.exit(f"{sample_rate_hz / 1e6:g} MS/s isn't a whole multiple of "
                 f"{SAMPLES_PER_SYMBOL} MS/s")
    return int(factor)


def select_channel(iq, sample_rate_hz, offset_hz):
    """Move the BLE channel to 0 Hz, low-pass filter it, and decimate.

    Mixing:      y[n] = x[n] * exp(-j 2 pi offset n / fs)
    Filtering:   z[n] = sum_k h[k] y[n-k], a Hamming-windowed sinc low-pass
    Decimation:  w[m] = z[m D], which is safe because z has no energy above fs / (2 D)
    """
    n = np.arange(len(iq))
    baseband = iq * np.exp(-2j * np.pi * offset_hz * n / sample_rate_hz)

    decimation = decimation_factor(sample_rate_hz)
    if decimation == 1:
        return baseband
    taps = signal.firwin(CHANNEL_FILTER_TAPS_PER_DECIMATION * decimation + 1,
                         CHANNEL_FILTER_CUTOFF_HZ, fs=sample_rate_hz)
    return signal.fftconvolve(baseband, taps, mode="same")[::decimation]


# --- Stage 2: FM discriminator -------------------------------------------------------

def quadrature_demodulate(x, sample_rate_hz):
    """Instantaneous frequency from the phase step between samples.

    delta_theta[n] = angle(x[n] * conj(x[n-1]))
    f[n] = delta_theta[n] * fs / (2 pi)

    The output doesn't depend on the signal's amplitude, so this also acts as a limiter.
    """
    delta_theta = np.angle(x[1:] * np.conj(x[:-1]))
    return np.concatenate(([0.0], delta_theta * sample_rate_hz / (2 * np.pi)))


# --- The transmitter's waveform, used for sync, and as the self-test's modulator ------

def gaussian_pulse(samples_per_symbol):
    """Frequency pulse of GFSK: a Gaussian filter with bandwidth-time product BT.

    g[n] = exp(-(2 pi BT n / sps)^2 / (2 ln 2))    truncated to +/- 2 symbols, sum g = 1
    """
    n = np.arange(-2 * samples_per_symbol, 2 * samples_per_symbol + 1)
    pulse = np.exp(-(2 * np.pi * GAUSSIAN_BT * n / samples_per_symbol) ** 2 / (2 * np.log(2)))
    return pulse / pulse.sum()


def gfsk_frequency(bits, samples_per_symbol):
    """Ideal frequency against time, in Hz: +deviation for a 1, -deviation for a 0."""
    nrz = np.repeat(2 * np.asarray(bits, dtype=float) - 1, samples_per_symbol)
    return DEVIATION_HZ * np.convolve(nrz, gaussian_pulse(samples_per_symbol), mode="same")


def bits_from_int(value, count):
    """Bits of an integer, least significant first, which is BLE's order on the air."""
    return [(value >> i) & 1 for i in range(count)]


def bits_to_bytes(bits):
    return bytes(sum(bit << i for i, bit in enumerate(bits[start:start + 8]))
                 for start in range(0, len(bits) - 7, 8))


# --- Stage 3: Access Address correlation ---------------------------------------------

def access_address_waveform():
    """Bits of the Access Address, and its ideal frequency against time, in Hz."""
    bits = bits_from_int(ACCESS_ADDRESS, ACCESS_ADDRESS_BITS)
    return np.array(bits), gfsk_frequency(bits, SAMPLES_PER_SYMBOL)


def correlate_access_address(freq):
    """Sliding normalized correlation of the frequency against the expected waveform.

    rho[n] = sum_k (f[n+k] - mean_n) t[k] / (norm(f - mean_n) * norm(t))

    The template has zero mean, and the sliding mean makes this insensitive to a frequency
    offset, since an offset only moves f up or down. rho = 1 is a perfect match.
    """
    _, waveform = access_address_waveform()
    template = waveform - waveform.mean()
    length = len(template)
    if len(freq) < length:
        return np.zeros(0)

    numerator = signal.fftconvolve(freq, template[::-1], mode="valid")
    cumulative = np.cumsum(np.concatenate(([0.0], freq)))
    cumulative_sq = np.cumsum(np.concatenate(([0.0], freq ** 2)))
    window_sum = cumulative[length:] - cumulative[:-length]
    window_sum_sq = cumulative_sq[length:] - cumulative_sq[:-length]
    energy = np.maximum(window_sum_sq - window_sum ** 2 / length, 1e-9)
    return numerator / np.sqrt(energy * np.sum(template ** 2))


def find_sync_candidates(rho):
    """Sample positions where rho peaks above the threshold, strongest first."""
    distance = ACCESS_ADDRESS_BITS * SAMPLES_PER_SYMBOL
    peaks, _ = signal.find_peaks(rho, height=SYNC_THRESHOLD, distance=distance)
    return peaks[np.argsort(rho[peaks])[::-1]]


def estimate_offset_and_deviation(freq, start):
    """Least-squares fit of f[n] = offset + deviation * s[n] over the Access Address, with s
    its ideal waveform scaled to +/- 1. The offset is the carrier error between the radios.

    The Access Address has 18 ones and 14 zeros, so s has a mean of its own. Treating the
    mean of f as the offset would read about 31 kHz too high.
    """
    _, waveform = access_address_waveform()
    window = freq[start:start + len(waveform)]
    shape = waveform / DEVIATION_HZ
    centered_shape = shape - shape.mean()
    deviation = np.dot(window - window.mean(), centered_shape) / np.dot(centered_shape, centered_shape)
    return window.mean() - deviation * shape.mean(), deviation


# --- Stage 4: symbol timing and bit slicing ------------------------------------------

def slice_bits(freq, first_center, count):
    """Sample one bit per symbol, steering the clock with the signal's zero crossings.

    Between two different bits the frequency crosses zero, ideally midway between their
    sampling instants, at t - T/2. If it crosses at t_c, the timing error is
    e = t_c - (t - T/2), and the next sampling instant is

    t[k+1] = t[k] + T + gain * e

    Decisions are by the sign of the frequency. Samples between instants are interpolated
    linearly. Returns the bits and the final timing correction, in samples.
    """
    bits = []
    t = float(first_center)
    previous_t = t - SAMPLES_PER_SYMBOL
    correction = 0.0
    last = len(freq) - 1

    for _ in range(count):
        if t >= last:
            break
        index = int(t)
        value = freq[index] + (t - index) * (freq[index + 1] - freq[index])
        bit = int(value > 0)

        if bits and bit != bits[-1]:
            low = max(int(previous_t), 0)
            high = min(int(np.ceil(t)), last)
            segment = freq[low:high + 1]
            crossings = np.where(np.signbit(segment[:-1]) != np.signbit(segment[1:]))[0]
            if len(crossings):
                k = crossings[0]
                crossing = low + k + segment[k] / (segment[k] - segment[k + 1])
                error = crossing - (t - SAMPLES_PER_SYMBOL / 2)
                correction += TIMING_LOOP_GAIN * error
                previous_t = t
                t += SAMPLES_PER_SYMBOL + TIMING_LOOP_GAIN * error
                bits.append(bit)
                continue

        previous_t = t
        t += SAMPLES_PER_SYMBOL
        bits.append(bit)
    return bits, correction


# --- Stage 5: de-whitening -----------------------------------------------------------

def whitening_sequence(ble_channel, count):
    """Output of the whitening LFSR, x^7 + x^4 + 1, seeded from the channel index.

    In the Core Specification's diagram, position 0 is set to 1 and positions 1 to 6 hold
    the channel index, most significant bit first. Numbering the register the other way
    round gives the compact form used here: seed = channel | 0x40, which is 0x65 for
    channel 37. The output is bit 0, and each step is a right shift that feeds the output
    back into bits 6 and 2: state = (state >> 1) ^ (output ? 0x44 : 0).
    """
    state = ble_channel | 0x40
    sequence = []
    for _ in range(count):
        output = state & 1
        sequence.append(output)
        state >>= 1
        if output:
            state ^= 0x44
    return sequence


def dewhiten(bits, ble_channel):
    """Whitening is an XOR with the LFSR's output, so undoing it is the same operation."""
    return [bit ^ w for bit, w in zip(bits, whitening_sequence(ble_channel, len(bits)))]


# --- Stage 6: CRC-24 -----------------------------------------------------------------

def crc24(bits, init=CRC_INIT):
    """CRC-24 of bits in air order. Each input bit is XORed with bit 23, the register
    shifts up, and if that gave 1 the polynomial is XORed in. Bit 23 is sent first."""
    crc = init
    for bit in bits:
        feedback = ((crc >> 23) & 1) ^ bit
        crc = (crc << 1) & 0xFFFFFF
        if feedback:
            crc ^= CRC_POLYNOMIAL
    return crc


def crc_from_air_bits(bits):
    value = 0
    for bit in bits:
        value = (value << 1) | bit
    return value


# --- Stage 7: PDU parser -------------------------------------------------------------

PDU_TYPES = {0: "ADV_IND", 1: "ADV_DIRECT_IND", 2: "ADV_NONCONN_IND", 3: "SCAN_REQ",
             4: "SCAN_RSP", 5: "CONNECT_IND", 6: "ADV_SCAN_IND", 7: "ADV_EXT_IND"}
COMPANY_IDS = {0x0006: "Microsoft", 0x004C: "Apple", 0x0059: "Nordic Semiconductor",
               0x0075: "Samsung", 0x00E0: "Google", 0x0157: "Anhui Huami", 0x038F: "Xiaomi"}
AD_TYPES = {0x01: "Flags", 0x02: "Incomplete 16-bit UUIDs", 0x03: "Complete 16-bit UUIDs",
            0x06: "Incomplete 128-bit UUIDs", 0x07: "Complete 128-bit UUIDs",
            0x08: "Short name", 0x09: "Complete name", 0x0A: "TX power", 0x16: "Service data",
            0x19: "Appearance", 0xFF: "Manufacturer data"}
FLAG_NAMES = ["LE Limited Discoverable", "LE General Discoverable", "BR/EDR Not Supported",
              "LE+BR/EDR Controller", "LE+BR/EDR Host"]


def format_address(raw, is_random):
    text = ":".join(f"{byte:02X}" for byte in reversed(raw))
    if not is_random:
        return f"{text} (public)"
    kind = {0b11: "static", 0b01: "resolvable private", 0b00: "non-resolvable private"}.get(
        raw[5] >> 6, "reserved")
    return f"{text} (random, {kind})"


def describe_ad_value(ad_type, value):
    if ad_type == 0x01 and value:
        names = [name for i, name in enumerate(FLAG_NAMES) if value[0] >> i & 1]
        return ", ".join(names) or "none"
    if ad_type in (0x08, 0x09):
        return repr(value.decode("utf-8", errors="replace"))
    if ad_type == 0x0A and value:
        return f"{int.from_bytes(value[:1], 'little', signed=True)} dBm"
    if ad_type in (0x02, 0x03):
        return ", ".join(f"0x{int.from_bytes(value[i:i + 2], 'little'):04X}"
                         for i in range(0, len(value) - 1, 2))
    if ad_type == 0xFF and len(value) >= 2:
        company = int.from_bytes(value[:2], "little")
        name = COMPANY_IDS.get(company, "unknown company")
        return f"{name} (0x{company:04X}), {value[2:].hex()}"
    if ad_type == 0x16 and len(value) >= 2:
        return f"UUID 0x{int.from_bytes(value[:2], 'little'):04X}, {value[2:].hex()}"
    return value.hex()


def parse_ad_structures(data):
    """AdvData is a list of [length, type, value...], where length counts type and value."""
    structures = []
    position = 0
    while position < len(data):
        length = data[position]
        if length == 0 or position + 1 + length > len(data):
            break
        ad_type = data[position + 1]
        value = data[position + 2:position + 1 + length]
        structures.append((AD_TYPES.get(ad_type, f"Type 0x{ad_type:02X}"),
                           describe_ad_value(ad_type, value)))
        position += 1 + length
    return structures


def parse_pdu(pdu):
    """Decode the 2-byte header and the payload's addresses and advertising data."""
    header, length = pdu[0], pdu[1]
    pdu_type = header & 0x0F
    tx_random = bool(header >> 6 & 1)
    rx_random = bool(header >> 7 & 1)
    payload = pdu[2:2 + length]
    fields = {"type": PDU_TYPES.get(pdu_type, f"Reserved 0x{pdu_type:X}"), "length": length,
              "payload": payload}

    if pdu_type in (0, 2, 6, 4) and length >= 6:
        fields["AdvA"] = format_address(payload[:6], tx_random)
        fields["adv_data"] = payload[6:]
        fields["ad_structures"] = parse_ad_structures(payload[6:])
    elif pdu_type == 1 and length >= 12:
        fields["AdvA"] = format_address(payload[:6], tx_random)
        fields["TargetA"] = format_address(payload[6:12], rx_random)
    elif pdu_type == 3 and length >= 12:
        fields["ScanA"] = format_address(payload[:6], tx_random)
        fields["AdvA"] = format_address(payload[6:12], rx_random)
    elif pdu_type == 5 and length >= 12:
        fields["InitA"] = format_address(payload[:6], tx_random)
        fields["AdvA"] = format_address(payload[6:12], rx_random)
    return fields


# --- The whole receiver --------------------------------------------------------------

class Packet:
    def __init__(self, **fields):
        self.__dict__.update(fields)


def demodulate(iq, sample_rate_hz, offset_hz, ble_channel):
    """Run every stage on one burst, returning a Packet per Access Address found."""
    x = select_channel(iq, sample_rate_hz, offset_hz)
    rate = SAMPLES_PER_SYMBOL * SYMBOL_RATE_HZ
    freq = quadrature_demodulate(x, rate)
    rho = correlate_access_address(freq)
    aa_bits, _ = access_address_waveform()

    packets = []
    claimed_until = -1
    for start in sorted(int(k) for k in find_sync_candidates(rho)):
        if start < claimed_until:
            continue
        carrier_offset, deviation = estimate_offset_and_deviation(freq, start)
        centered = freq - carrier_offset

        first_center = start + SAMPLES_PER_SYMBOL // 2
        received_aa, _ = slice_bits(centered, first_center, ACCESS_ADDRESS_BITS)
        if len(received_aa) < ACCESS_ADDRESS_BITS:
            continue
        aa_errors = int(np.sum(np.array(received_aa) != aa_bits))
        if aa_errors > MAX_ACCESS_ADDRESS_ERRORS:
            continue

        # The header gives the length, so slice it first, then the rest of the packet.
        after_aa = first_center + ACCESS_ADDRESS_BITS * SAMPLES_PER_SYMBOL
        bits, _ = slice_bits(centered, after_aa, HEADER_BITS)
        if len(bits) < HEADER_BITS:
            packets.append(Packet(status="truncated", start=start, rho=rho[start],
                                  carrier_offset=carrier_offset))
            continue
        header = dewhiten(bits, ble_channel)
        length = bits_to_bytes(header[8:16])[0]
        total = HEADER_BITS + length * 8 + CRC_BITS

        bits, correction = slice_bits(centered, after_aa, total)
        duration = (ACCESS_ADDRESS_BITS + total) * SAMPLES_PER_SYMBOL
        power = np.mean(np.abs(x[start:start + duration]) ** 2)
        packet = Packet(status="truncated" if len(bits) < total else "crc_fail", start=start,
                        rho=float(rho[start]), carrier_offset=float(carrier_offset),
                        deviation=float(deviation), aa_errors=aa_errors, length=length,
                        timing_correction=correction, power_dbfs=10 * np.log10(power / FULL_SCALE ** 2 + 1e-20),
                        time_us=start / SAMPLES_PER_SYMBOL)
        packets.append(packet)
        if len(bits) < total:
            continue

        clear = dewhiten(bits, ble_channel)
        pdu_bits = clear[:HEADER_BITS + length * 8]
        computed = crc24(pdu_bits)
        received = crc_from_air_bits(clear[-CRC_BITS:])
        packet.computed_crc, packet.received_crc = computed, received
        packet.pdu = bits_to_bytes(pdu_bits)
        if computed == received:
            packet.status = "ok"
            packet.fields = parse_pdu(packet.pdu)
            claimed_until = start + duration
    return packets


# --- Output --------------------------------------------------------------------------

def print_packet(packet, ble_channel, burst_label):
    fields = packet.fields
    print(f"[{burst_label} +{packet.time_us:.0f} us] BLE ch {ble_channel} {fields['type']}, "
          f"{fields['length']} bytes, CRC OK 0x{packet.received_crc:06X}")
    print(f"  signal: {packet.power_dbfs:.1f} dBFS, carrier offset {packet.carrier_offset / 1e3:+.1f} kHz, "
          f"deviation {abs(packet.deviation) / 1e3:.0f} kHz, Access Address errors {packet.aa_errors}, "
          f"timing correction {packet.timing_correction:+.2f} samples")
    for name in ("AdvA", "TargetA", "ScanA", "InitA"):
        if name in fields:
            print(f"  {name}: {fields[name]}")
    if "adv_data" in fields:
        print(f"  AdvData: {fields['adv_data'].hex() or '(empty)'}")
        for name, value in fields["ad_structures"]:
            print(f"    {name}: {value}")
    elif fields["payload"]:
        print(f"  Payload: {fields['payload'].hex()}")


class Totals:
    def __init__(self):
        self.bursts = 0
        self.ok = 0
        self.crc_fail = 0
        self.truncated = 0
        self.addresses = {}

    def add(self, packet):
        if packet.status == "ok":
            self.ok += 1
            address = packet.fields.get("AdvA")
            if address:
                self.addresses[address] = self.addresses.get(address, 0) + 1
        elif packet.status == "crc_fail":
            self.crc_fail += 1
        else:
            self.truncated += 1

    def report(self):
        print(f"bursts: {self.bursts}, packets with valid CRC: {self.ok}, CRC failures: "
              f"{self.crc_fail}, cut off by the burst's end: {self.truncated}")
        for address, count in sorted(self.addresses.items(), key=lambda item: -item[1]):
            print(f"  {count:4d} x {address}")


def process_burst(iq, metadata, ble_channel, totals, label, verbose):
    offset_hz = (pi_receiver.BLE_CHANNEL_MHZ[ble_channel] - metadata["center_mhz"]) * 1e6
    for packet in demodulate(iq, metadata["sample_rate_hz"], offset_hz, ble_channel):
        totals.add(packet)
        if packet.status == "ok":
            print_packet(packet, ble_channel, label)
        elif verbose and packet.status == "crc_fail":
            print(f"[{label} +{packet.time_us:.0f} us] CRC failed: {packet.length} bytes, "
                  f"computed 0x{packet.computed_crc:06X}, received 0x{packet.received_crc:06X}, "
                  f"AA errors {packet.aa_errors}, {packet.pdu.hex()}")


def resolve_channel(requested, metadata):
    channel = requested or metadata.get("ble_channel")
    if channel not in pi_receiver.BLE_CHANNEL_MHZ:
        sys.exit("Which BLE channel is this? Capture with pi_receiver.py --ble, or pass --ble-channel.")
    return channel


def run_file(args):
    iq = np.load(args.capture).astype(np.complex128)
    sidecar = args.capture.with_suffix(".json")
    if not sidecar.exists():
        sys.exit(f"{sidecar} is missing, so the capture's tuning and sample rate are unknown")
    metadata = json.loads(sidecar.read_text())
    ble_channel = resolve_channel(args.ble_channel, metadata)

    totals = Totals()
    samples = metadata["burst_samples"]
    for number, start in zip(metadata["burst_numbers"], metadata["burst_starts"]):
        totals.bursts += 1
        process_burst(iq[start:start + samples], metadata, ble_channel, totals,
                      f"burst {number}", args.verbose)
    totals.report()


def run_live(args):
    stats = pi_receiver.Stats()
    bursts = pi_receiver.pull_bursts(args.connect, stats)
    metadata = next(bursts)
    ble_channel = resolve_channel(args.ble_channel, metadata)
    print(f"connected: BLE channel {ble_channel}, tuned to {metadata['center_mhz']} MHz at "
          f"{metadata['sample_rate_hz'] / 1e6:g} MS/s")

    totals = Totals()
    try:
        for burst in bursts:
            totals.bursts += 1
            i, q, _ = pi_receiver.unpack_iq(burst.words)
            process_burst((i + 1j * q).astype(np.complex128), metadata, ble_channel, totals,
                          f"burst {burst.number}", args.verbose)
            if args.count and totals.bursts >= args.count:
                break
    except KeyboardInterrupt:
        pass
    totals.report()


# --- Self-test: a transmitter and a channel model, so the receiver can run without radio -

def build_air_bits(pdu, ble_channel):
    """Preamble, Access Address, then the whitened PDU and CRC, in air order."""
    aa = bits_from_int(ACCESS_ADDRESS, ACCESS_ADDRESS_BITS)
    preamble = [(aa[0] + 1 + i) % 2 for i in range(8)]  # Alternates, ending opposite to aa[0]
    pdu_bits = [bit for byte in pdu for bit in bits_from_int(byte, 8)]
    crc = crc24(pdu_bits)
    crc_bits = [(crc >> (23 - i)) & 1 for i in range(24)]
    return preamble + aa + dewhiten(pdu_bits + crc_bits, ble_channel)


def modulate(bits, sample_rate_hz, clock_error=0.0):
    """Continuous-phase FM: x[n] = exp(j 2 pi sum_{k<=n} f[k] / fs)."""
    oversampling = 16
    freq = gfsk_frequency(bits, oversampling)
    samples_per_symbol = sample_rate_hz / SYMBOL_RATE_HZ * (1 + clock_error)
    positions = np.arange(0, (len(freq) - 1) * samples_per_symbol / oversampling) * \
        oversampling / samples_per_symbol
    resampled = np.interp(positions, np.arange(len(freq)), freq)
    return np.exp(2j * np.pi * np.cumsum(resampled) / sample_rate_hz)


def make_test_burst(pdu, ble_channel, sample_rate_hz, tuning_offset_hz, snr_db, rng,
                    carrier_offset_hz=0.0, clock_error=0.0, amplitude=200.0, burst_samples=16380,
                    position=3000, flip_air_bits=()):
    bits = build_air_bits(pdu, ble_channel)
    for index in flip_air_bits:
        bits[index] ^= 1
    tx = amplitude * modulate(bits, sample_rate_hz, clock_error)
    burst = np.zeros(burst_samples, dtype=np.complex128)
    end = min(position + len(tx), burst_samples)
    burst[position:end] = tx[:end - position]
    n = np.arange(burst_samples)
    burst *= np.exp(2j * np.pi * (tuning_offset_hz + carrier_offset_hz) * n / sample_rate_hz)

    # SNR is measured in the receiver's channel filter, since that is the noise it sees.
    noise_power = amplitude ** 2 / 10 ** (snr_db / 10) * sample_rate_hz / (2 * CHANNEL_FILTER_CUTOFF_HZ)
    noise = np.sqrt(noise_power / 2) * (rng.standard_normal(burst_samples) +
                                        1j * rng.standard_normal(burst_samples))
    return burst + noise


def make_test_pdu(rng=None):
    adv_address = bytes.fromhex("112233445566")[::-1]
    adv_data = bytes([2, 0x01, 0x06, 5, 0x09]) + b"Test" + bytes([3, 0x03, 0x0F, 0x18])
    payload = adv_address + adv_data
    return bytes([0x40, len(payload)]) + payload  # ADV_IND with a random TxAdd


def self_test():
    rng = np.random.default_rng(1)
    failures = []

    def check(name, passed, detail=""):
        print(f"  {'pass' if passed else 'FAIL'}  {name}{': ' + detail if detail else ''}")
        if not passed:
            failures.append(name)

    print("Bit-level stages")
    check("LFSR seed for channel 37 is 0x65", 37 | 0x40 == 0x65)
    period = whitening_sequence(37, 254)
    check("LFSR has period 127, as a maximal 7-bit LFSR must", period[:127] == period[127:])
    pdu_bits = [bit for byte in make_test_pdu() for bit in bits_from_int(byte, 8)]
    check("whitening undoes itself", dewhiten(dewhiten(pdu_bits, 38), 38) == pdu_bits)
    crc_bits = [(crc24(pdu_bits) >> (23 - i)) & 1 for i in range(24)]
    check("CRC of a message followed by its CRC is 0", crc24(pdu_bits + crc_bits) == 0)
    flipped = list(pdu_bits)
    flipped[10] ^= 1
    check("CRC changes if a bit flips", crc24(flipped) != crc24(pdu_bits))
    fields = parse_pdu(make_test_pdu())
    check("parser reads the address and name",
          "66:55:44:33:22:11" not in fields["AdvA"] and "11:22:33:44:55:66" in fields["AdvA"] and
          ("Complete name", "'Test'") in fields["ad_structures"], fields["AdvA"])

    print("End to end, through the modulator and a noisy channel")
    fs = 16_000_000
    offset = -2_000_000
    pdu = make_test_pdu()

    def received_ok(packets):
        return any(p.status == "ok" and p.pdu == pdu for p in packets)

    def run(**kwargs):
        burst = make_test_burst(pdu, 37, fs, offset, rng=rng, **kwargs)
        return demodulate(burst, fs, offset, 37)

    packets = run(snr_db=30)
    check("clean packet decodes", received_ok(packets))
    good = next((p for p in packets if p.status == "ok"), None)
    if good:
        check("position is right", abs(good.time_us - (3000 / 16 + 8)) < 1.5,
              f"{good.time_us:.1f} us, expected {3000 / 16 + 8:.1f}")
    packets = run(snr_db=30, carrier_offset_hz=120_000)
    good = next((p for p in packets if p.status == "ok"), None)
    check("carrier offset of +120 kHz decodes and is measured",
          bool(good) and abs(good.carrier_offset - 120_000) < 10_000,
          f"{good.carrier_offset / 1e3:+.1f} kHz" if good else "")
    packets = run(snr_db=30, carrier_offset_hz=-150_000)
    check("carrier offset of -150 kHz decodes", received_ok(packets))
    for error in (500e-6, -500e-6, 1500e-6, -1500e-6):
        check(f"clock error of {error * 1e6:+.0f} ppm decodes",
              received_ok(run(snr_db=30, clock_error=error)))
    # The Access Address occupies air bits 8 to 39, after the preamble.
    check("Access Address with 2 bit errors decodes", received_ok(run(snr_db=30, flip_air_bits=(12, 30))))
    packets = run(snr_db=30, flip_air_bits=(12, 22, 32))
    check("Access Address with 3 bit errors is rejected", not packets)

    packets = demodulate(make_test_burst(pdu, 37, fs, offset, 30, rng, flip_air_bits=(70,)), fs, offset, 37)
    check("a flipped payload bit fails the CRC",
          any(p.status == "crc_fail" for p in packets) and not received_ok(packets))
    packets = demodulate(make_test_burst(pdu, 37, fs, offset, 30, rng, position=16380 - 800), fs, offset, 37)
    check("a packet cut off by the burst's end is reported as truncated",
          any(p.status == "truncated" for p in packets) and not received_ok(packets))
    packets = demodulate(make_test_burst(pdu, 38, fs, offset, 30, rng), fs, offset, 37)
    check("the wrong channel's whitening fails", not received_ok(packets))
    noise_only = make_test_burst(pdu, 37, fs, offset, 30, rng, amplitude=1e-3, position=16000)
    check("noise alone gives no packets", not any(p.status == "ok" for p in demodulate(noise_only, fs, offset, 37)))

    print("Packets decoded, of 100, against SNR in the channel filter")
    for snr in (6, 8, 10, 12, 15, 20):
        decoded = sum(received_ok(run(snr_db=snr, carrier_offset_hz=rng.uniform(-100e3, 100e3),
                                      position=int(rng.integers(0, 4000))))
                      for _ in range(100))
        print(f"  {snr:3d} dB: {decoded:3d}")
        if snr == 15:
            check("at least 95 decode at 15 dB", decoded >= 95)

    print("Sample rates")
    for rate in (16_000_000, 40_000_000, 80_000_000):
        burst = make_test_burst(pdu, 37, rate, offset, 30, rng, burst_samples=int(rate * 600e-6),
                                position=int(rate * 100e-6))
        check(f"{rate // 1_000_000} MS/s decodes", received_ok(demodulate(burst, rate, offset, 37)))

    print("PASS" if not failures else f"FAIL: {', '.join(failures)}")
    return not failures


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("capture", type=pathlib.Path, nargs="?", help=".npy saved by pi_receiver.py --ble")
    parser.add_argument("--connect", metavar="HOST:PORT", help="demodulate live from a running Pi")
    parser.add_argument("--self-test", action="store_true", help="test the receiver with no radio")
    parser.add_argument("--ble-channel", type=int, choices=pi_receiver.BLE_CHANNEL_MHZ,
                        help="BLE channel, if the capture's metadata doesn't say")
    parser.add_argument("--count", type=int, default=0, help="stop after this many bursts")
    parser.add_argument("--verbose", action="store_true", help="also print packets that fail the CRC")
    args = parser.parse_args()

    if args.self_test:
        sys.exit(0 if self_test() else 1)
    if bool(args.capture) == bool(args.connect):
        parser.error("give a capture file, --connect, or --self-test")
    if args.capture:
        run_file(args)
    else:
        run_live(args)


if __name__ == "__main__":
    main()
