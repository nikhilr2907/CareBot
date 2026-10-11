#!/usr/bin/env python3
"""Interactive live monitor for the STM32's four HX711 load-cell channels."""

from __future__ import annotations

import glob
import os
import re
import select
import signal
import sys
import termios
import time


BAUD = 115200
RAW_RE = re.compile(r"LC raw=\[([^]]+)\] delta=\[([^]]+)\]")


def find_port() -> str:
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        raise SystemExit("No STM32 STLink serial port found.")
    return ports[0]


def open_serial(port: str) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def send(fd: int, command: str) -> None:
    os.write(fd, (command + "\n").encode("ascii"))


def main() -> None:
    port = find_port()
    fd = open_serial(port)
    serial_buffer = b""
    latest: tuple[int, int, int, int] | None = None
    last_print = 0.0
    running = True

    def stop(*_args: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    send(fd, "S")
    send(fd, "LC STREAM 0")
    time.sleep(0.3)
    send(fd, "LC TARE")
    send(fd, "LC RESET")
    time.sleep(0.7)
    send(fd, "LC STREAM 1")

    print(f"Load-cell monitor: {port} @ {BAUD}")
    print("Channels: CH0 CH1 CH2 CH3")
    print("Keys: t = re-zero, s = status, q = stop")
    print("Motors are stopped.\n")

    try:
        while running:
            readable, _, _ = select.select([fd, sys.stdin], [], [], 0.1)

            if sys.stdin in readable:
                key = sys.stdin.readline().strip().lower()
                if key == "q":
                    break
                if key == "t":
                    send(fd, "LC STREAM 0")
                    send(fd, "LC TARE")
                    send(fd, "LC RESET")
                    time.sleep(0.5)
                    send(fd, "LC STREAM 1")
                    print("Re-zeroed all channels.")
                elif key == "s":
                    send(fd, "LC STATUS")

            if fd in readable:
                data = os.read(fd, 4096)
                if data:
                    serial_buffer += data
                while b"\n" in serial_buffer:
                    line, serial_buffer = serial_buffer.split(b"\n", 1)
                    text = line.decode("utf-8", errors="replace").strip()
                    match = RAW_RE.search(text)
                    if match:
                        values = tuple(int(value) for value in match.group(2).split())
                        if len(values) == 4:
                            latest = values  # type: ignore[assignment]
                    elif text.startswith("LC ") and "LIVE" not in text and "span=" not in text:
                        print(f"BOARD: {text}")

            now = time.monotonic()
            if latest is not None and now - last_print >= 0.5:
                values = " ".join(f"{value:>9}" for value in latest)
                print(f"DELTA {values}", flush=True)
                last_print = now
    finally:
        try:
            send(fd, "LC STREAM 0")
            send(fd, "S")
        finally:
            os.close(fd)
        print("Monitoring stopped. Motors stopped.")


if __name__ == "__main__":
    main()
