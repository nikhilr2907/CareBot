#!/usr/bin/env python3
"""Live STM32 STLink VCP monitor with auto-reconnect."""

import argparse
import sys
import time

import serial
from serial.serialutil import SerialException


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/cu.usbmodem2103")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--retry-delay", type=float, default=0.75)
    return parser.parse_args()


def main():
    args = parse_args()
    print(f"LIVE STM32 STLink VCP FEED on {args.port} @ {args.baud} baud")
    print("STM32 receives ESP32 sensor-node frames over RS485 UARTs and prints SENSOR lines here.")
    print("USART2/STLink is only the live/debug terminal; boot motor movement test is disabled.")
    print("Auto-reconnect enabled. Press Ctrl-C to stop.\n")
    sys.stdout.flush()

    while True:
        try:
            with serial.Serial(args.port, args.baud, timeout=0.2) as stream:
                try:
                    stream.dtr = False
                    stream.rts = False
                except Exception:
                    pass

                print(f"-- connected to {args.port} --")
                sys.stdout.flush()

                while True:
                    data = stream.read(4096)
                    if data:
                        sys.stdout.write(data.decode("utf-8", errors="replace"))
                        sys.stdout.flush()
        except KeyboardInterrupt:
            print("\nStopped.")
            return
        except (OSError, SerialException) as exc:
            print(f"\n-- serial disconnected/unavailable: {exc} --")
            print(f"-- retrying {args.port} in {args.retry_delay:.2f}s --")
            sys.stdout.flush()
            time.sleep(args.retry_delay)


if __name__ == "__main__":
    main()
