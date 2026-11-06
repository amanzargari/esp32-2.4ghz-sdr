#!/usr/bin/env python3
"""Ping the ESP32 over UART and verify every heartbeat reply. Linux only.

This only checks link health: captures go over SPI, through pi_receiver.py.

PC-side line errors can't be read here: a classic CP2102 passes garbled bytes
through without flagging them, and the kernel's TIOCGICOUNT counters stay at 0.
Corruption is caught by the packet's magic byte, checksum and counter instead.
ESP32-side receive errors are reported in the heartbeat itself.
"""

import argparse
import array
import fcntl
import statistics
import struct
import sys
import time

import serial

CMD_PING = b"\xaa"
HEARTBEAT_MAGIC = 0x55
# magic, timestamp_us, counter, rx_errors, checksum. Mirrors heartbeat_t in main/main.c.
HEARTBEAT = struct.Struct("<BQIHB")

TCGETS2 = 0x802C542A


def open_port(port, baud):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.5
    # NodeMCU wires DTR/RTS to EN/GPIO0, so keep them idle to avoid a reset.
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def driver_baud(ser):
    """Return the rate the driver applied. It clamps without raising an error.

    This can't catch everything: a classic CP2102 reports 1000000 but runs at 921600.
    """
    buf = array.array("i", [0] * 64)
    fcntl.ioctl(ser.fd, TCGETS2, buf)
    return buf[10]


def check_packet(raw, prev):
    """Return (parsed fields, problem string or None)."""
    if len(raw) != HEARTBEAT.size:
        return None, f"short read: {len(raw)} bytes"

    fields = HEARTBEAT.unpack(raw)
    magic, timestamp, counter, _, checksum = fields
    if magic != HEARTBEAT_MAGIC:
        return None, f"bad magic: {raw.hex()}"

    if checksum != xor_of(raw[:-1]):
        return None, f"bad checksum: {raw.hex()}"

    if prev is not None:
        if counter != prev[2] + 1:
            return fields, f"counter jumped {prev[2]} -> {counter}"
        if timestamp <= prev[1]:
            return fields, f"timestamp went backwards {prev[1]} -> {timestamp}"
    return fields, None


def xor_of(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921_600)
    parser.add_argument("--count", type=int, default=1000, help="number of ping rounds")
    parser.add_argument("--burst", type=int, default=1, help="pings sent per round")
    args = parser.parse_args()

    ser = open_port(args.port, args.baud)

    actual = driver_baud(ser)
    if actual != args.baud:
        sys.exit(f"FAIL: requested {args.baud} baud but the driver set {actual}. "
                 "The USB bridge does not support this rate.")

    # Opening the port may have reset the board, and the ROM bootloader always
    # prints at 115200. Let that finish, then discard it.
    time.sleep(1.0)
    ser.reset_input_buffer()

    prev = None
    first = None
    bad = 0
    round_trips = []
    start = time.perf_counter()

    for _ in range(args.count):
        sent = time.perf_counter()
        ser.write(CMD_PING * args.burst)
        for _ in range(args.burst):
            fields, problem = check_packet(ser.read(HEARTBEAT.size), prev)
            if problem:
                bad += 1
                if bad <= 5:
                    print(f"  {problem}")
            if fields:
                prev = fields
                first = first or fields
        if first is None:
            sys.exit("FAIL: no valid reply. Check that the firmware and --baud match "
                     "and that the bridge really supports this rate.")
        round_trips.append(time.perf_counter() - sent)

    elapsed = time.perf_counter() - start
    device_errors = prev[3] - first[3] if prev else 0
    total = args.count * args.burst
    ser.close()

    print(f"baud:              {actual}")
    print(f"packets:           {total - bad}/{total} good")
    print(f"ESP32 rx errors:   {device_errors}")
    print(f"round trip (ms):   median={statistics.median(round_trips) * 1e3:.2f} "
          f"max={max(round_trips) * 1e3:.2f}")
    print(f"throughput:        {(total - bad) * HEARTBEAT.size / elapsed / 1024:.1f} KiB/s")

    if bad or device_errors:
        sys.exit("FAIL")
    print("PASS")


if __name__ == "__main__":
    main()
