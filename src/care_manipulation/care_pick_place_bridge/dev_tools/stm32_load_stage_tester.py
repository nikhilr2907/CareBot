#!/usr/bin/env python3
"""Staged STM32 stepper load tester.

This script uses the STM32 USART2/STLink terminal at 115200 baud.
It assumes ENA is not driven by firmware and the DM542-style drivers are left
enabled by their hardware/default state.
"""

from __future__ import annotations

import glob
import os
import select
import signal
import sys
import termios
import threading
import time
from dataclasses import dataclass


BAUD = 115200


@dataclass(frozen=True)
class MotorDef:
    number: int
    name: str
    pul: str
    direction: str


MOTORS: dict[int, MotorDef] = {
    1: MotorDef(1, "conveyor x axis left", "PA6", "PB12"),
    2: MotorDef(2, "y axis right", "PA7", "PB14"),
    3: MotorDef(3, "conveyor belt right", "PC8", "PB15"),
    4: MotorDef(4, "conveyor belt left", "PB1", "PB2"),
    5: MotorDef(5, "y axis left", "PA11", "PC4"),
    6: MotorDef(6, "conveyor x axis right", "PB8", "PC5"),
    7: MotorDef(7, "x axis angle motor", "PB9", "PC1"),
}

PAIRS: dict[str, list[int]] = {
    "y axis pair": [5, 2],
    "conveyor x axis pair": [1, 6],
    "conveyor belt pair": [4, 3],
}

GROUPS: dict[str, list[int]] = {
    "y axis pair": [5, 2],
    "conveyor x axis pair": [1, 6],
    "conveyor belt pair": [4, 3],
    "conveyor group": [1, 6, 4, 3],
    "all except angle": [1, 2, 3, 4, 5, 6],
    "all motors": [1, 2, 3, 4, 5, 6, 7],
}


def find_ports() -> list[str]:
    patterns = [
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
        "/dev/cu.SLAB_USBtoUART*",
        "/dev/cu.wchusbserial*",
        "/dev/cu.usb*",
    ]
    ports: list[str] = []
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    return sorted(set(ports))


def baud_constant(baud: int) -> int:
    name = f"B{baud}"
    if not hasattr(termios, name):
        raise ValueError(f"Unsupported baud rate: {baud}")
    return getattr(termios, name)


def open_serial(port: str, baud: int) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    if hasattr(termios, "CRTSCTS"):
        attrs[2] &= ~termios.CRTSCTS
    baud_const = baud_constant(baud)
    attrs[4] = baud_const
    attrs[5] = baud_const
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def write_line(fd: int, line: str) -> None:
    os.write(fd, (line + "\n").encode("ascii", errors="ignore"))


def choose_port() -> str:
    ports = find_ports()
    if not ports:
        raise SystemExit("No STM serial port found. Plug in the STM microUSB cable.")
    preferred = "/dev/cu.usbmodem103"
    if preferred in ports:
        return preferred
    if len(ports) == 1:
        return ports[0]

    print("Serial ports found:")
    for idx, port in enumerate(ports, 1):
        print(f"  {idx}) {port}")
    while True:
        text = input(f"Select port [1]: ").strip()
        if not text:
            return ports[0]
        try:
            choice = int(text)
        except ValueError:
            continue
        if 1 <= choice <= len(ports):
            return ports[choice - 1]


def ask_int(prompt: str, default: int, low: int, high: int) -> int:
    while True:
        text = input(f"{prompt} [{default}]: ").strip()
        if not text:
            return default
        try:
            value = int(text)
        except ValueError:
            print("Please enter a number.")
            continue
        if low <= value <= high:
            return value
        print(f"Please enter a number from {low} to {high}.")


def ask_direction(default: str = "F") -> str:
    while True:
        text = input(f"Direction F/R [{default}]: ").strip().upper()
        if not text:
            return default
        if text in ("F", "R"):
            return text
        print("Please type F or R.")


def reader(fd: int, stop: threading.Event) -> None:
    buf = b""
    while not stop.is_set():
        try:
            readable, _, _ = select.select([fd], [], [], 0.05)
            if not readable:
                continue
            data = os.read(fd, 4096)
        except OSError:
            return
        if not data:
            continue
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("utf-8", errors="replace").strip()
            if text:
                print(f"BOARD: {text}")


def describe(numbers: list[int]) -> str:
    return ", ".join(f"M{n} {MOTORS[n].name}" for n in numbers)


def motor_list_arg(numbers: list[int]) -> str:
    return ",".join(str(n) for n in numbers)


def stop_all(fd: int) -> None:
    write_line(fd, "S")


def run_set(fd: int, numbers: list[int], hz: int, direction: str, seconds: int) -> None:
    print()
    print("Ready to run:")
    print(f"  Motors:    {describe(numbers)}")
    print(f"  Command:   G {motor_list_arg(numbers)} {hz} {direction}")
    print(f"  Duration:  {seconds} second(s)")
    print()
    print("Lift/secure the mechanism. Keep fingers clear.")
    ready = input("Type GO then Enter to start, or anything else to cancel: ").strip().upper()
    if ready != "GO":
        stop_all(fd)
        print("Cancelled. STOP sent.")
        return

    write_line(fd, f"G {motor_list_arg(numbers)} {hz} {direction}")
    started = time.monotonic()
    try:
        while True:
            remaining = seconds - int(time.monotonic() - started)
            if remaining <= 0:
                break
            print(f"Running... {remaining}s remaining. Press Ctrl+C for STOP.", end="\r")
            time.sleep(0.2)
    finally:
        stop_all(fd)
        print("\nSTOP sent.")


def print_definitions() -> None:
    print()
    print("Stored motor definitions")
    print("------------------------")
    for n in sorted(MOTORS):
        motor = MOTORS[n]
        print(f"M{n}: {motor.name:<24} PUL={motor.pul:<4} DIR={motor.direction}")
    print()
    print("Pairs")
    print("-----")
    for name, numbers in PAIRS.items():
        print(f"{name}: {motor_list_arg(numbers)}")


def choose_from_table(title: str, items: list[tuple[str, list[int]]]) -> list[int] | None:
    print()
    print(title)
    print("-" * len(title))
    for idx, (name, numbers) in enumerate(items, 1):
        print(f"  {idx}) {name:<24} {motor_list_arg(numbers)}")
    text = input("Select number, or blank to cancel: ").strip()
    if not text:
        return None
    try:
        choice = int(text)
    except ValueError:
        return None
    if 1 <= choice <= len(items):
        return items[choice - 1][1]
    return None


def custom_group() -> list[int] | None:
    text = input("Type motor numbers, e.g. 1,2,7: ").strip()
    if not text:
        return None
    numbers: list[int] = []
    for part in text.replace("+", ",").replace(" ", ",").split(","):
        if not part:
            continue
        try:
            number = int(part)
        except ValueError:
            print(f"Bad motor number: {part}")
            return None
        if number not in MOTORS:
            print(f"Motor must be 1..7: {number}")
            return None
        if number not in numbers:
            numbers.append(number)
    return numbers or None


def main() -> int:
    print("STM32 staged stepper load tester")
    print("Press Ctrl+C at any time to send STOP ALL.")
    print_definitions()

    port = choose_port()
    fd = open_serial(port, BAUD)
    stop_reader = threading.Event()
    thread = threading.Thread(target=reader, args=(fd, stop_reader), daemon=True)
    thread.start()

    def stop_on_signal(_signum: int, _frame: object) -> None:
        stop_all(fd)
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_on_signal)

    try:
        write_line(fd, "LOG 0")
        time.sleep(0.1)
        stop_all(fd)

        while True:
            print()
            print("Test menu")
            print("---------")
            print("  1) One motor at a time")
            print("  2) Motor pairs")
            print("  3) Groups of motors")
            print("  4) All motors at once")
            print("  5) Custom motor list")
            print("  0) STOP ALL")
            print("  q) Quit")
            choice = input("Choose: ").strip().lower()

            if choice == "q":
                break
            if choice == "0":
                stop_all(fd)
                print("STOP sent.")
                continue

            numbers: list[int] | None = None
            if choice == "1":
                items = [(f"M{n} {MOTORS[n].name}", [n]) for n in sorted(MOTORS)]
                numbers = choose_from_table("Single motors", items)
            elif choice == "2":
                numbers = choose_from_table("Motor pairs", list(PAIRS.items()))
            elif choice == "3":
                numbers = choose_from_table("Motor groups", list(GROUPS.items()))
            elif choice == "4":
                numbers = [1, 2, 3, 4, 5, 6, 7]
            elif choice == "5":
                numbers = custom_group()
            else:
                print("Unknown choice.")
                continue

            if not numbers:
                print("No motors selected.")
                continue

            hz = ask_int("Speed in step pulses/second", 80, 1, 5000)
            direction = ask_direction("F")
            seconds = ask_int("Run duration in seconds", 3, 1, 60)
            run_set(fd, numbers, hz, direction, seconds)

    except KeyboardInterrupt:
        print("\nEmergency stop requested.")
    finally:
        try:
            stop_all(fd)
            time.sleep(0.2)
        finally:
            stop_reader.set()
            os.close(fd)
            print("All motors stopped. Serial port closed.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
