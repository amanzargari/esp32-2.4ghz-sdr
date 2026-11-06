#!/usr/bin/env python3
"""Stream I/Q bursts from the ESP32 over SPI, or pull them from a Pi over TCP.

On the Raspberry Pi 5:
    pi_receiver.py --channel 6 --gain 64 --bursts 100 --out capture.npy
    pi_receiver.py --channel 6 --gain 64 --bursts 0 --tcp-port 5555
    pi_receiver.py --sweep 1-13 --gain 48 --bursts 20 --out sweep.npy
    pi_receiver.py --ble 37 --gain 48 --bursts 0 --tcp-port 5555

On the laptop:
    pi_receiver.py --connect 192.168.77.2:5555 --count 100 --out capture.npy
    pi_receiver.py --connect 192.168.77.2:5555 --live

The ESP32's dump engine takes snapshots, so bursts are separated by gaps. A file holds
the bursts one after another, and its .json sidecar records where each one starts.
"""

import argparse
import glob
import json
import pathlib
import queue
import signal
import socket
import struct
import sys
import threading
import time
import zlib

import numpy as np

# Mirrors main/protocol.h.
FRAME_SIZE = 4096
HEADER = struct.Struct("<IBBBBIIHHHBBII")
CRC_OFFSET = 28
FRAME_MAGIC = 0x46524453
COMMAND_MAGIC = 0x43524453
FRAME_IDLE, FRAME_STATUS, FRAME_DATA, FRAME_END = 0, 1, 2, 3
COMMAND_START, COMMAND_STOP = 1, 2
COMMAND = struct.Struct("<IBBBBBHHH")
GAIN_AGC = 0xFF
RATES_MSPS = {80: 0, 40: 1, 16: 2}
STATUS_TEXT = {1: "bad arguments", 2: "dump engine timed out",
               3: "dump engine wrote the wrong length", 4: "a stream is already running"}
TUNE_MIN_MHZ = 2400
TUNE_MAX_MHZ = 2500
MIN_SAMPLES = 256
MAX_SAMPLES = 16380
FULL_SCALE = 512  # Signed 10-bit samples

# ESP-IDF documents 10 MHz as the ESP32's limit. 25 MHz is beyond that, but ran without
# errors when measured on this wiring. The usable clocks there were 8-12.5 and 20-25 MHz.
DEFAULT_SPI_HZ = 25_000_000
SYNC_SPI_HZ = 10_000_000
SPI_MODE = 0  # Must match SPI_LINK_MODE in main/spi_link.c
HANDSHAKE_BCM = 25
MISO_DELAYS = (0, 1, 2)  # Candidate MISO delays, in clock cycles
MISO_CHECK_FRAMES = 20
EXTRA_READ = 4  # Clocks in the bits a MISO delay pushes past the end of the frame
IDLE_REARM_S = 0.001
SPIDEV_BUFSIZ_PATH = "/sys/module/spidev/parameters/bufsiz"
READY_TIMEOUT_S = 5.0
HANDSHAKE_POLL_S = 0.0005
SYNC_POLL_S = 0.1
SYNC_MAX_FRAMES = 64

TCP_MAGIC = 0x42524453  # "SDRB" on the wire
TCP_BURST = struct.Struct("<IIIIIB")  # magic, number, samples, timestamp_us, crc32, channel
TCP_CLIENT_QUEUE = 16


class Stats:
    def __init__(self):
        self.frames = 0
        self.crc_errors = 0
        self.lost_frames = 0
        self.bad_bursts = 0


def channel_mhz(channel):
    return 2484 if channel == 14 else 2407 + 5 * channel


# BLE advertising channels: frequency, and the tuning for them. The radio is tuned off the
# channel because its own DC offset and LO leakage sit at the tuned frequency.
BLE_CHANNEL_MHZ = {37: 2402, 38: 2426, 39: 2480}
BLE_TUNING_OFFSET_MHZ = 2
BLE_RATE_MSPS = 16  # The longest burst, 1024 us, which fits any advertising packet


def make_command(command, first_channel=0, last_channel=0, rate=0, gain=0, samples=0, passes=0,
                 frequency_mhz=0):
    body = COMMAND.pack(COMMAND_MAGIC, command, first_channel, last_channel, rate, gain,
                        samples, passes, frequency_mhz)
    return body + struct.pack("<I", zlib.crc32(body))


def parse_frame(frame):
    """Return the header fields if the magic and CRC are valid, otherwise None."""
    fields = HEADER.unpack_from(frame)
    crc = zlib.crc32(frame[HEADER.size:], zlib.crc32(frame[:CRC_OFFSET]))
    return fields if fields[0] == FRAME_MAGIC and fields[-1] == crc else None


class SpiLink:
    """One SPI device plus the handshake line that paces it."""

    def __init__(self, bus, device, hz, stats):
        import gpiod
        import spidev

        self._check_buffer_size()
        self.stats = stats
        self.miso_delay = 0
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.mode = SPI_MODE
        self.spi.max_speed_hz = hz
        self.active = gpiod.line.Value.ACTIVE
        # Edges only wake the wait below. They can't be counted: CS switching on the
        # neighbouring pin adds false ones, and real ones can be missed.
        try:
            self.gpio = gpiod.request_lines(
                self._find_gpio_chip(gpiod),
                consumer="pi_receiver",
                config={HANDSHAKE_BCM: gpiod.LineSettings(
                    direction=gpiod.line.Direction.INPUT,
                    bias=gpiod.line.Bias.PULL_DOWN,
                    edge_detection=gpiod.line.Edge.RISING)})
        except OSError as error:
            sys.exit(f"FAIL: can't use GPIO {HANDSHAKE_BCM}: {error.strerror}. Is another "
                     "pi_receiver.py already running?")

    @staticmethod
    def _check_buffer_size():
        # The kernel's default of 4096 is too small for a frame plus the MISO delay's extra
        # bytes, and a longer transfer would be split, which ends the frame early.
        needed = FRAME_SIZE + EXTRA_READ
        size = int(pathlib.Path(SPIDEV_BUFSIZ_PATH).read_text())
        if size < needed:
            sys.exit(f"FAIL: the spidev buffer is {size} bytes, and this needs {needed}. Add "
                     "'options spidev bufsiz=65536' to /etc/modprobe.d/spidev.conf and reboot.")

    @staticmethod
    def _find_gpio_chip(gpiod):
        # The RP1 chip's number has changed between Pi 5 kernels, but not its label.
        for path in sorted(glob.glob("/dev/gpiochip*")):
            with gpiod.Chip(path) as chip:
                if chip.get_info().label == "pinctrl-rp1":
                    return path
        sys.exit("FAIL: no pinctrl-rp1 GPIO chip found. This must run on a Raspberry Pi 5.")

    def close(self):
        self.gpio.release()
        self.spi.close()

    def exchange(self, tx=None):
        """Clock one frame. `tx` is sent at the start of it, followed by zeros."""
        size = FRAME_SIZE + EXTRA_READ if self.miso_delay else FRAME_SIZE
        raw = bytes(self.spi.xfer3((tx or b"").ljust(size, b"\0")))
        if not self.miso_delay:
            return raw
        # MISO arrives whole clock cycles late, so every bit reads that many places late.
        shifted = (int.from_bytes(raw, "big") << self.miso_delay) & ((1 << (size * 8)) - 1)
        return shifted.to_bytes(size, "big")[:FRAME_SIZE]

    def set_speed(self, hz):
        self.spi.max_speed_hz = hz

    def calibrate_miso_delay(self, hz):
        """Find the MISO delay at this clock, using the idle frames the ESP32 has armed."""
        self.set_speed(hz)
        for delay in MISO_DELAYS:
            self.miso_delay = delay
            for _ in range(MISO_CHECK_FRAMES):
                time.sleep(IDLE_REARM_S)
                fields = parse_frame(self.exchange())
                if not fields or fields[1] != FRAME_IDLE:
                    break
            else:
                return delay
        sys.exit(f"FAIL: no usable MISO timing at {hz / 1e6:g} MHz. Clocks that worked "
                 "here are 8-12.5 and 20-25 MHz. Check the wiring.")

    def wait_ready(self, timeout):
        """Wait for the handshake to be high. The ESP32 drops it as soon as a frame ends,
        before the transfer call returns, so this can't see the previous frame's level."""
        deadline = time.monotonic() + timeout
        while self.gpio.get_value(HANDSHAKE_BCM) != self.active:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            if self.gpio.wait_edge_events(min(remaining, HANDSHAKE_POLL_S)):
                self.gpio.read_edge_events()
        return True


class Burst:
    def __init__(self, number, channel, timestamp_us, words):
        self.number = number
        self.channel = channel
        self.timestamp_us = timestamp_us
        self.words = words


def synchronize(link):
    """Bring the ESP32 to idle from any state, such as a stream left by a crashed run."""
    tx = make_command(COMMAND_STOP)
    settled = False
    for _ in range(SYNC_MAX_FRAMES):
        fields = parse_frame(link.exchange(tx))
        if fields and fields[1] in (FRAME_IDLE, FRAME_STATUS, FRAME_END):
            settled = True
            tx = None
        ready = link.wait_ready(SYNC_POLL_S)
        if settled and not ready:
            return
    sys.exit("FAIL: no valid frames from the ESP32. Check the wiring, including ground, "
             "the firmware, and --spi-hz.")


def start_stream(link, args):
    request = make_command(COMMAND_START, args.channels[0], args.channels[-1],
                           RATES_MSPS[args.rate], args.gain, args.samples, args.bursts,
                           args.freq or 0)
    time.sleep(IDLE_REARM_S)  # Idle frames raise no handshake, so allow time to arm one
    link.exchange(request)
    if not link.wait_ready(READY_TIMEOUT_S):
        sys.exit("FAIL: no reply to the start command")
    fields = parse_frame(link.exchange())
    if not fields or fields[1] != FRAME_STATUS:
        sys.exit("FAIL: invalid reply to the start command")
    if fields[10]:
        sys.exit(f"FAIL: {STATUS_TEXT.get(fields[10], fields[10])}")


def receive_bursts(link, stop, stats):
    """Yield each burst whose frames all arrived intact, until the ESP32 ends the stream."""
    expected_sequence = None
    burst_number = None
    buffer = None
    filled = 0
    timestamp = 0
    burst_channel = 0
    tx_stop = make_command(COMMAND_STOP)

    while True:
        if not link.wait_ready(READY_TIMEOUT_S):
            sys.exit("FAIL: timed out waiting for the handshake. Did the ESP32 reset? "
                     "Opening its serial port from the laptop does that.")
        frame = link.exchange(tx_stop if stop.is_set() else None)
        stats.frames += 1

        fields = parse_frame(frame)
        if fields is None:
            stats.crc_errors += 1
            continue
        _, kind, channel, _, _, sequence, burst, offset, count, burst_samples, status, _, ts, _ = fields

        if kind not in (FRAME_DATA, FRAME_END):
            continue
        if expected_sequence is not None and sequence == (expected_sequence - 1) & 0xFFFFFFFF:
            continue  # The ESP32 saw a short transfer and resent a frame that did arrive
        if expected_sequence is not None and sequence != expected_sequence:
            stats.lost_frames += (sequence - expected_sequence) & 0xFFFFFFFF
        expected_sequence = (sequence + 1) & 0xFFFFFFFF

        if kind == FRAME_END:
            if status:
                sys.exit(f"FAIL: capture error: {STATUS_TEXT.get(status, status)}")
            return

        if offset == 0:
            if burst_number is not None:
                stats.bad_bursts += 1  # The previous burst never finished
            burst_number, buffer, filled, timestamp = burst, bytearray(burst_samples * 4), 0, ts
            burst_channel = channel
        if burst != burst_number or offset != filled:
            stats.bad_bursts += burst_number is not None
            burst_number = None
            continue

        buffer[offset * 4:(offset + count) * 4] = frame[HEADER.size:HEADER.size + count * 4]
        filled += count
        if filled == burst_samples:
            yield Burst(burst_number, burst_channel, timestamp, np.frombuffer(buffer, dtype="<u4"))
            burst_number = None


def unpack_iq(words):
    """Split raw words into signed 10-bit I and Q, and the gain index."""
    i = (((words >> 10) & 0x3FF) ^ 0x200).astype(np.int32) - 0x200
    q = ((words & 0x3FF) ^ 0x200).astype(np.int32) - 0x200
    return i, q, (words >> 20) & 0x7F


class FileSink:
    """Appends bursts to a contiguous .npy of complex64, or a file of raw 32-bit words."""

    NPY_HEADER_SIZE = 128

    def __init__(self, path, metadata, remove_dc):
        self.path = path
        self.metadata = metadata
        self.remove_dc = remove_dc
        self.is_npy = path.suffix == ".npy"
        self.file = open(path, "wb")
        self.samples = 0
        self.burst_starts = []
        self.burst_numbers = []
        self.burst_channels = []
        self.timestamps = []
        self.gain_range = [127, 0]
        if self.is_npy:
            self.file.write(b"\0" * self.NPY_HEADER_SIZE)

    def write(self, burst):
        i, q, gain = unpack_iq(burst.words)
        if self.is_npy:
            iq = i + 1j * q
            if self.remove_dc:
                iq = iq - iq.mean()
            self.file.write(iq.astype("<c8").tobytes())
        else:
            self.file.write(burst.words.tobytes())

        self.burst_starts.append(self.samples)
        self.burst_numbers.append(burst.number)
        self.burst_channels.append(burst.channel)
        self.timestamps.append(burst.timestamp_us)
        self.samples += len(burst.words)
        self.gain_range = [min(self.gain_range[0], int(gain.min())),
                           max(self.gain_range[1], int(gain.max()))]

    def close(self):
        if self.is_npy:
            header = f"{{'descr': '<c8', 'fortran_order': False, 'shape': ({self.samples},), }}"
            prefix = b"\x93NUMPY\x01\x00" + struct.pack("<H", self.NPY_HEADER_SIZE - 10)
            body = header.ljust(self.NPY_HEADER_SIZE - len(prefix) - 1) + "\n"
            self.file.seek(0)
            self.file.write(prefix + body.encode("latin1"))
        self.file.close()

        sidecar = dict(self.metadata)
        sidecar.update(samples=self.samples, bursts=len(self.burst_starts),
                       burst_starts=self.burst_starts, burst_numbers=self.burst_numbers,
                       burst_channels=self.burst_channels,
                       burst_timestamps_us=self.timestamps, gain_min=self.gain_range[0],
                       gain_max=self.gain_range[1], dc_removed=self.remove_dc and self.is_npy)
        self.path.with_suffix(".json").write_text(json.dumps(sidecar) + "\n")


class TcpServer:
    """Read-only stream of bursts. A slow client loses bursts rather than stalling capture."""

    def __init__(self, bind, port, metadata):
        self.hello = (json.dumps(metadata) + "\n").encode()
        self.clients = []
        self.lock = threading.Lock()
        self.listener = socket.create_server((bind, port), reuse_port=False)
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            connection, address = self.listener.accept()
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"client connected: {address[0]}")
            outbox = queue.Queue(TCP_CLIENT_QUEUE)
            with self.lock:
                self.clients.append(outbox)
            threading.Thread(target=self._serve, args=(connection, outbox), daemon=True).start()

    def _serve(self, connection, outbox):
        try:
            connection.sendall(self.hello)
            while True:
                burst = outbox.get()
                if burst is None:
                    break
                payload = burst.words.tobytes()
                connection.sendall(TCP_BURST.pack(TCP_MAGIC, burst.number, len(burst.words),
                                                  burst.timestamp_us, zlib.crc32(payload),
                                                  burst.channel))
                connection.sendall(payload)
        except OSError:
            pass
        finally:
            with self.lock:
                if outbox in self.clients:
                    self.clients.remove(outbox)
            connection.close()

    def publish(self, burst):
        with self.lock:
            clients = list(self.clients)
        for outbox in clients:
            try:
                outbox.put_nowait(burst)
            except queue.Full:
                pass  # The client sees the gap in the burst numbers

    def close(self):
        self.listener.close()
        with self.lock:
            clients = list(self.clients)
        for outbox in clients:
            try:
                outbox.put(None, timeout=2)
            except queue.Full:
                pass
        # Give client threads a moment to flush what is already queued.
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            with self.lock:
                if not self.clients:
                    break
            time.sleep(0.05)


def run_capture(args):
    stats = Stats()
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.set())

    metadata = {
        "channels": args.channels,
        "sample_rate_hz": args.rate * 1_000_000,
        "center_mhz": args.freq or channel_mhz(args.channels[0]),
        "ble_channel": args.ble,
        "burst_samples": args.samples,
        "full_scale": FULL_SCALE,
        "gain_requested": "agc" if args.gain == GAIN_AGC else args.gain,
    }
    link = SpiLink(args.spi_bus, args.spi_device, args.spi_hz, stats)
    sink = FileSink(args.out, metadata, args.remove_dc) if args.out else None
    server = TcpServer(args.tcp_bind, args.tcp_port, metadata) if args.tcp_port else None
    if server:
        print(f"streaming on {args.tcp_bind}:{args.tcp_port}")

    count = 0
    start = None
    try:
        link.set_speed(SYNC_SPI_HZ)
        synchronize(link)
        delay = link.calibrate_miso_delay(args.spi_hz)
        print(f"SPI {args.spi_hz / 1e6:g} MHz, MISO delay {delay} clock(s)")
        start = time.perf_counter()
        start_stream(link, args)
        for burst in receive_bursts(link, stop, stats):
            count += 1
            if sink:
                sink.write(burst)
            if server:
                server.publish(burst)
            if args.duration and time.perf_counter() - start >= args.duration:
                stop.set()
    finally:
        elapsed = time.perf_counter() - start if start else 0
        synchronize(link)
        link.close()
        if sink:
            sink.close()
        if server:
            server.close()

    sample_rate = args.rate * 1e6
    payload = count * args.samples * 4
    if elapsed and len(args.channels) > 1:
        print(f"sweep rate:    {count / len(args.channels) / elapsed:.2f} passes/s, "
              f"{elapsed / max(count, 1) * 1e3:.1f} ms per hop")
    print(f"bursts:        {count}")
    print(f"samples:       {count * args.samples}")
    print(f"frames:        {stats.frames}")
    print(f"CRC errors:    {stats.crc_errors}")
    print(f"lost frames:   {stats.lost_frames}")
    print(f"bad bursts:    {stats.bad_bursts}")
    if elapsed:
        print(f"throughput:    {payload / elapsed / 1e6:.2f} MB/s of samples over {elapsed:.2f} s")
        print(f"radio time:    {count * args.samples / sample_rate / elapsed * 100:.2f}% of real time")
    if args.out:
        print(f"saved {args.out} and {args.out.with_suffix('.json')}")
    if stats.crc_errors or stats.lost_frames or stats.bad_bursts:
        sys.exit("FAIL: some data was corrupted or lost")
    print("PASS")


def read_exactly(connection, size):
    data = bytearray()
    while len(data) < size:
        chunk = connection.recv(min(size - len(data), 1 << 16))
        if not chunk:
            return None
        data += chunk
    return bytes(data)


class LiveSpectrum:
    def __init__(self, metadata):
        import matplotlib.pyplot as plt

        sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
        import plot_capture

        self.plt = plt
        self.tools = plot_capture
        self.rate = metadata["sample_rate_hz"]
        self.metadata = metadata
        self.figure, self.axes = plt.subplots(figsize=(10, 5), facecolor=plot_capture.SURFACE)
        plot_capture.style_axes(self.axes)
        self.line = None
        plt.ion()
        plt.show()

    def update(self, burst):
        i, q, _ = unpack_iq(burst.words)
        result = self.tools.analyze((i + 1j * q).astype(np.complex128), self.rate, 0, False)
        bins = max(1, len(i) // 128)
        trace = self.tools.dbfs(self.tools.smooth(result["power"], bins))
        trace[result["hidden"]] = np.nan
        if self.line is None:
            freqs_mhz = result["freqs"] / 1e6
            self.line, = self.axes.plot(freqs_mhz, trace, color=self.tools.SERIES_I, linewidth=1.5)
            self.axes.set_xlim(freqs_mhz[0], freqs_mhz[-1])
            self.axes.set_ylim(-110, 0)
            self.axes.set_xlabel("Offset from tuned frequency (MHz)")
            self.axes.set_ylabel("dBFS")
        else:
            self.line.set_ydata(trace)
        self.axes.set_title(f"Channel {burst.channel} · burst {burst.number}", loc="left")
        self.plt.pause(0.001)
        return bool(self.plt.get_fignums())


def describe_channels(metadata):
    channels = metadata["channels"]
    if not channels[0]:
        return f"{metadata['center_mhz']} MHz"
    if len(channels) == 1:
        return f"channel {channels[0]}"
    return f"channels {channels[0]}-{channels[-1]}"


def pull_bursts(address, stats):
    """Connect to a running Pi. Yield its metadata, then each intact burst until it stops."""
    host, _, port = address.rpartition(":")
    if not host or not port.isdigit():
        sys.exit("--connect needs HOST:PORT")

    connection = socket.create_connection((host, int(port)), timeout=30)
    with connection, connection.makefile("rb") as reader:
        yield json.loads(reader.readline())
        while True:
            header = reader.read(TCP_BURST.size)
            if len(header) < TCP_BURST.size:
                print("server closed the stream", file=sys.stderr)
                return
            magic, number, samples, timestamp, crc, channel = TCP_BURST.unpack(header)
            if magic != TCP_MAGIC:
                sys.exit("FAIL: lost framing on the TCP stream")
            payload = reader.read(samples * 4)
            if len(payload) < samples * 4:
                print("server closed the stream", file=sys.stderr)
                return
            if zlib.crc32(payload) != crc:
                stats.crc_errors += 1
                continue
            yield Burst(number, channel, timestamp, np.frombuffer(payload, dtype="<u4"))


def run_client(args):
    stats = Stats()
    bursts = pull_bursts(args.connect, stats)
    metadata = next(bursts)
    print(f"connected: {describe_channels(metadata)}, {metadata['burst_samples']} "
          f"samples per burst at {metadata['sample_rate_hz'] / 1e6:g} MS/s")
    sink = FileSink(args.out, metadata, args.remove_dc) if args.out else None
    live = LiveSpectrum(metadata) if args.live else None

    count = gaps = 0
    previous = None
    try:
        for burst in bursts:
            if previous is not None and burst.number != previous + 1:
                gaps += 1
            previous = burst.number
            count += 1
            if sink:
                sink.write(burst)
            if live and not live.update(burst):
                break
            if args.count and count >= args.count:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if sink:
            sink.close()

    print(f"bursts: {count}, gaps in burst numbers: {gaps}, CRC errors: {stats.crc_errors}")
    if args.out:
        print(f"saved {args.out} and {args.out.with_suffix('.json')}")
    if stats.crc_errors:
        sys.exit("FAIL: some data was corrupted")


def parse_sweep(text):
    first, _, last = text.partition("-")
    if not (first.isdigit() and last.isdigit() and 1 <= int(first) <= int(last) <= 14):
        raise argparse.ArgumentTypeError("expected a range of channels, such as 1-13")
    return list(range(int(first), int(last) + 1))


def parse_gain(text):
    if text == "agc":
        return GAIN_AGC
    gain = int(text)
    if not 0 <= gain < GAIN_AGC:
        raise argparse.ArgumentTypeError(f"{text} is out of range")
    return gain


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=pathlib.Path,
                        help="save bursts: .npy as complex64, anything else as raw 32-bit words")
    parser.add_argument("--remove-dc", action="store_true",
                        help="subtract each burst's mean I and Q before saving (.npy only)")

    capture = parser.add_argument_group("capture, on the Pi")
    capture.add_argument("--channel", type=int, default=1, help="Wi-Fi channel, 1-14")
    capture.add_argument("--sweep", type=parse_sweep, metavar="FIRST-LAST",
                         help="hop through these channels instead, needing a manual --gain. "
                              "--bursts then counts passes")
    capture.add_argument("--freq", type=int, metavar="MHZ",
                         help=f"tune to any whole MHz, {TUNE_MIN_MHZ}-{TUNE_MAX_MHZ}, instead of a channel")
    capture.add_argument("--ble", type=int, choices=BLE_CHANNEL_MHZ,
                         help="capture a BLE advertising channel for ble_demod.py. Sets --freq "
                              f"and --rate {BLE_RATE_MSPS}")
    capture.add_argument("--rate", type=int, default=80, choices=RATES_MSPS, help="sample rate, MS/s")
    capture.add_argument("--samples", type=int, default=MAX_SAMPLES,
                         help=f"samples per burst, {MIN_SAMPLES}-{MAX_SAMPLES} (default: %(default)s)")
    capture.add_argument("--gain", type=parse_gain, default=GAIN_AGC, metavar="INDEX|agc",
                         help="RX gain table index (not dB), or agc (default)")
    capture.add_argument("--bursts", type=int, default=1,
                         help="bursts to capture, or 0 to run until Ctrl+C (default: %(default)s)")
    capture.add_argument("--duration", type=float, help="stop after this many seconds")
    capture.add_argument("--spi-hz", type=int, default=DEFAULT_SPI_HZ,
                         help="SPI clock (default: %(default)s, above the 10 MHz ESP-IDF documents)")
    capture.add_argument("--spi-bus", type=int, default=0)
    capture.add_argument("--spi-device", type=int, default=0)
    capture.add_argument("--tcp-port", type=int, help="also serve bursts over TCP on this port")
    capture.add_argument("--tcp-bind", default="0.0.0.0",
                         help="address to serve on. There is no authentication, so prefer "
                              "the direct cable's address (default: %(default)s)")

    client = parser.add_argument_group("client, on the laptop")
    client.add_argument("--connect", metavar="HOST:PORT", help="pull bursts from a running Pi")
    client.add_argument("--count", type=int, default=0, help="stop after this many bursts")
    client.add_argument("--live", action="store_true", help="show a live spectrum")

    args = parser.parse_args()

    if args.remove_dc and (args.out is None or args.out.suffix != ".npy"):
        parser.error("--remove-dc needs an .npy --out, since raw words are saved unmodified")
    if args.connect:
        run_client(args)
        return

    if args.sweep and args.gain == GAIN_AGC:
        parser.error("--sweep needs a manual --gain, since AGC would scale each channel "
                     "differently and the results couldn't be stitched")
    if sum(map(bool, (args.sweep, args.freq, args.ble))) > 1:
        parser.error("--sweep, --freq and --ble are alternatives")
    if args.ble:
        args.freq = BLE_CHANNEL_MHZ[args.ble] + BLE_TUNING_OFFSET_MHZ
        args.rate = BLE_RATE_MSPS
    if args.freq and not TUNE_MIN_MHZ <= args.freq <= TUNE_MAX_MHZ:
        parser.error(f"--freq must be {TUNE_MIN_MHZ}-{TUNE_MAX_MHZ}")
    args.channels = [0] if args.freq else (args.sweep or [args.channel])
    if not MIN_SAMPLES <= args.samples <= MAX_SAMPLES:
        parser.error(f"--samples must be {MIN_SAMPLES}-{MAX_SAMPLES}")
    if not 0 <= args.bursts <= 0xFFFF:
        parser.error("--bursts must be 0-65535")
    run_capture(args)


if __name__ == "__main__":
    main()
