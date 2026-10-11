#!/usr/bin/env python3
"""Decode the ESP32 sensor telemetry coming from the USB-to-RS485 adapter."""

import argparse
import sys
import time

import serial


FRAME_LEN = 15


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default="/dev/cu.usbmodem5A990493261",
        help="USB-to-RS485 serial port",
    )
    parser.add_argument("--baud", type=int, default=115200)
    return parser.parse_args()


def main():
    args = parse_args()
    print(f"LIVE RS485 FEED on {args.port} @ {args.baud} baud")
    print("Using RS485 adapter only. ESP32 USB/programming port is not opened.")
    print("Frame: AA 55 0A 01 [tof0] [tof1] [tof2] [tof3] [ultra] [checksum]")
    print("Press Ctrl-C to stop.\n")
    sys.stdout.flush()

    buffer = bytearray()
    count = 0
    valid = 0
    started = time.time()
    last_summary = started

    with serial.Serial(args.port, args.baud, timeout=0.1) as stream:
        while True:
            data = stream.read(4096)
            if data:
                buffer.extend(data)

            while len(buffer) >= FRAME_LEN:
                if buffer[0] != 0xAA or buffer[1] != 0x55:
                    del buffer[0]
                    continue

                frame = bytes(buffer[:FRAME_LEN])
                del buffer[:FRAME_LEN]

                count += 1
                checksum_ok = (sum(frame[2:14]) & 0xFF) == frame[14]
                if checksum_ok:
                    valid += 1

                tof0 = (frame[4] << 8) | frame[5]
                tof1 = (frame[6] << 8) | frame[7]
                tof2 = (frame[8] << 8) | frame[9]
                tof3 = (frame[10] << 8) | frame[11]
                ultra = (frame[12] << 8) | frame[13]
                hex_frame = " ".join(f"{byte:02X}" for byte in frame)
                status = "OK" if checksum_ok else "BAD"

                print(
                    f"{time.time() - started:8.2f}s #{count:06d} {status} | "
                    f"ToF0={tof0:5d} ToF1={tof1:5d} ToF2={tof2:5d} "
                    f"ToF3={tof3:5d} Ultra={ultra:5d} | {hex_frame}"
                )
                sys.stdout.flush()

            now = time.time()
            if now - last_summary >= 5:
                print(
                    f"-- summary: frames={count}, valid={valid}, "
                    f"invalid={count - valid}, buffered={len(buffer)} --"
                )
                sys.stdout.flush()
                last_summary = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
