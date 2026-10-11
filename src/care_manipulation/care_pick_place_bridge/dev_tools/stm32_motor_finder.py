#!/usr/bin/env python3
"""Keyboard-driven STM32 stepper motor finder.

This talks to the STM32 USART2/STLink terminal at 115200 baud. The firmware
understands:
  M <motor> <hz> <F|R>     continuous pulses
  J <motor> <steps> <F|R> <pulse_us>  exact jog pulses
  S                         stop all steppers

The tool starts by sending S and LOG 0. It never drives the motor-driver ENA
line; the known-working board setup leaves ENA floating.
"""

from __future__ import annotations

import argparse
import glob
import os
import select
import signal
import sys
import termios
import time
from dataclasses import dataclass, field
from typing import Iterable


BAUD = 115200
MOTOR_COUNT = 7

MOTOR_MAP = {
    1: ("PA6", "PB12"),
    2: ("PA7", "PB14"),
    3: ("PC8", "PB15"),
    4: ("PB1", "PB2"),
    5: ("PA11", "PC4"),
    6: ("PB8", "PC5"),
    7: ("PB9", "PC1"),
}


def find_ports() -> list[str]:
    patterns = [
        "/dev/cu.usbmodem*",
        "/dev/cu.usbserial*",
        "/dev/cu.SLAB_USBtoUART*",
        "/dev/cu.wchusbserial*",
        "/dev/cu.usb*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
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


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


@dataclass
class UiState:
    port: str
    baud: int
    motor: int = 2
    mode: str = "continuous"
    running: bool = False
    speed_hz: int = 100
    speed_step_hz: int = 25
    direction: str = "F"
    jog_steps: int = 20
    jog_step_delta: int = 5
    jog_pulse_us: int = 500
    board_log_enabled: bool = False
    last_command: str = "S"
    status: str = "Ready. ENA untouched. Pick 1-7; Space runs, Enter jogs."
    board_lines: list[str] = field(default_factory=list)


class RawTerminal:
    def __init__(self) -> None:
        self.fd = sys.stdin.fileno()
        self.old_attrs: list[int | list[bytes]] | None = None

    def __enter__(self) -> "RawTerminal":
        self.old_attrs = termios.tcgetattr(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[3] &= ~(termios.ECHO | termios.ICANON | termios.IEXTEN | termios.ISIG)
        attrs[0] &= ~(termios.IXON | termios.ICRNL | termios.BRKINT | termios.INPCK | termios.ISTRIP)
        attrs[1] &= ~(termios.OPOST)
        attrs[2] |= termios.CS8
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        return self

    def __exit__(self, *_exc: object) -> None:
        self.restore()

    def restore(self) -> None:
        if self.old_attrs is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.old_attrs)

    def raw_again(self) -> None:
        self.__enter__()


def prompt_int(raw: RawTerminal, prompt: str, default: int, low: int, high: int) -> int:
    raw.restore()
    try:
        print()
        text = input(f"{prompt} [{default}]: ").strip()
        if not text:
            return default
        try:
            value = int(text)
        except ValueError:
            print(f"Not a number; keeping {default}. Press any key.")
            sys.stdin.read(1)
            return default
        return clamp(value, low, high)
    finally:
        raw.raw_again()


def clear_screen() -> None:
    sys.stdout.write("\x1b[2J\x1b[H")


def render(state: UiState) -> None:
    pul, direction_pin = MOTOR_MAP[state.motor]
    mode_text = "CONTINUOUS" if state.mode == "continuous" else "JOG / BY STEP"
    run_text = "RUNNING" if state.running else "stopped"
    log_text = "ON" if state.board_log_enabled else "quiet"
    clear_screen()
    print("STM32 Stepper Motor Finder")
    print("Lift the robot. Keep fingers clear. Press 0 for STOP ALL.")
    print()
    print(f"Port:              {state.port} @ {state.baud} baud")
    print(f"Selected motor:    M{state.motor}     PUL={pul}  DIR={direction_pin}")
    print(f"Mode:              {mode_text}     Continuous is {run_text}")
    print(f"Speed:             {state.speed_hz} Hz     +/- changes by {state.speed_step_hz} Hz")
    print(f"Direction:         {state.direction}        F = forward, R = reverse")
    print(f"Jog size:          {state.jog_steps} pulse(s)   +/- in jog changes by {state.jog_step_delta}")
    print(f"Jog pulse width:   {state.jog_pulse_us} us high + {state.jog_pulse_us} us low")
    print(f"Board logging:     {log_text}       l toggles noisy SENSOR/IMU lines")
    print("Stepper enable:    not driven by firmware/tester")
    print()
    print("Keys:")
    print("  1-7        select motor, stopping the old one first")
    print("  c / j      continuous mode / jog-by-step mode")
    print("  SPACE      start or stop continuous mode")
    print("  ENTER      jog selected motor by the shown pulse count")
    print("  + or Up    speed up in continuous; bigger jog in jog mode")
    print("  - or Down  speed down in continuous; smaller jog in jog mode")
    print("  [ or ]     smaller / bigger continuous speed-change amount")
    print("  , or .     shorter / longer jog pulse width")
    print("  f or r     set direction")
    print("  v          type exact continuous speed")
    print("  s          type exact jog pulse count")
    print("  l          toggle board logs")
    print("  h          ask STM32 for command help")
    print("  0 or x     STOP ALL motors")
    print("  q          stop all and quit")
    print()
    print(f"Last command: {state.last_command}")
    print(f"Status:       {state.status}")
    print()
    print("Board messages:")
    for line in state.board_lines[-9:]:
        print(f"  {line}")
    sys.stdout.flush()


def record_board_line(state: UiState, line: str) -> None:
    line = line.rstrip("\r")
    if not line.strip():
        return
    state.board_lines.append(line)
    if len(state.board_lines) > 80:
        del state.board_lines[:-80]


def send_stop(fd: int, state: UiState, status: str = "STOP ALL sent.") -> None:
    write_line(fd, "S")
    state.running = False
    state.last_command = "S"
    state.status = status


def send_continuous(fd: int, state: UiState) -> None:
    cmd = f"M {state.motor} {state.speed_hz} {state.direction}"
    write_line(fd, cmd)
    state.running = True
    state.last_command = cmd
    state.status = f"M{state.motor} running at {state.speed_hz} Hz {state.direction}."


def send_jog(fd: int, state: UiState) -> None:
    state.running = False
    cmd = f"J {state.motor} {state.jog_steps} {state.direction} {state.jog_pulse_us}"
    write_line(fd, cmd)
    state.last_command = cmd
    state.status = f"M{state.motor} jogged {state.jog_steps} pulse(s)."


def handle_key(fd: int, state: UiState, raw: RawTerminal, key: bytes) -> bool:
    if key in (b"\x1b[A", b"+", b"="):
        if state.mode == "continuous":
            state.speed_hz = clamp(state.speed_hz + state.speed_step_hz, 1, 100000)
            state.status = f"Speed is now {state.speed_hz} Hz."
            if state.running:
                send_continuous(fd, state)
        else:
            state.jog_steps = clamp(state.jog_steps + state.jog_step_delta, 1, 10000)
            state.status = f"Jog size is now {state.jog_steps} pulse(s)."
        return True

    if key in (b"\x1b[B", b"-", b"_"):
        if state.mode == "continuous":
            state.speed_hz = clamp(state.speed_hz - state.speed_step_hz, 1, 100000)
            state.status = f"Speed is now {state.speed_hz} Hz."
            if state.running:
                send_continuous(fd, state)
        else:
            state.jog_steps = clamp(state.jog_steps - state.jog_step_delta, 1, 10000)
            state.status = f"Jog size is now {state.jog_steps} pulse(s)."
        return True

    if key == b"[":
        state.speed_step_hz = clamp(max(1, state.speed_step_hz // 2), 1, 100000)
        state.status = f"Continuous speed-change amount is now {state.speed_step_hz} Hz."
        return True

    if key == b"]":
        state.speed_step_hz = clamp(state.speed_step_hz * 2, 1, 100000)
        state.status = f"Continuous speed-change amount is now {state.speed_step_hz} Hz."
        return True

    if key == b",":
        state.jog_pulse_us = clamp(max(2, state.jog_pulse_us // 2), 2, 100000)
        state.status = f"Jog pulse width is now {state.jog_pulse_us} us."
        return True

    if key == b".":
        state.jog_pulse_us = clamp(state.jog_pulse_us * 2, 2, 100000)
        state.status = f"Jog pulse width is now {state.jog_pulse_us} us."
        return True

    if key in (b"\r", b"\n"):
        send_jog(fd, state)
        return True

    if key == b" ":
        if state.running:
            send_stop(fd, state, "Continuous run stopped.")
        else:
            send_continuous(fd, state)
        return True

    ch = key.decode("ascii", errors="ignore").lower()
    if ch in "1234567":
        send_stop(fd, state, f"Selected M{ch}. Press Space to run or Enter to jog.")
        state.motor = int(ch)
        return True

    if ch == "c":
        state.mode = "continuous"
        state.status = "Continuous mode selected. Space starts/stops."
        return True

    if ch == "j":
        if state.running:
            send_stop(fd, state, "Stopped before switching to jog mode.")
        state.mode = "jog"
        state.status = "Jog mode selected. Enter sends the shown pulse count."
        return True

    if ch == "f":
        state.direction = "F"
        state.status = "Direction set to forward."
        if state.running:
            send_continuous(fd, state)
        return True

    if ch == "r":
        state.direction = "R"
        state.status = "Direction set to reverse."
        if state.running:
            send_continuous(fd, state)
        return True

    if ch == "v":
        state.speed_hz = prompt_int(raw, "Exact continuous speed in Hz", state.speed_hz, 1, 100000)
        state.status = f"Speed is now {state.speed_hz} Hz."
        if state.running:
            send_continuous(fd, state)
        return True

    if ch == "s":
        state.jog_steps = prompt_int(raw, "Exact jog pulse count", state.jog_steps, 1, 10000)
        state.status = f"Jog size is now {state.jog_steps} pulse(s)."
        return True

    if ch == "l":
        state.board_log_enabled = not state.board_log_enabled
        cmd = "LOG 1" if state.board_log_enabled else "LOG 0"
        write_line(fd, cmd)
        state.last_command = cmd
        state.status = "Board logs enabled." if state.board_log_enabled else "Board logs quieted."
        return True

    if ch == "h":
        write_line(fd, "H")
        state.last_command = "H"
        state.status = "Asked STM32 to print help."
        return True

    if ch in ("0", "x"):
        send_stop(fd, state)
        return True

    if ch == "q":
        send_stop(fd, state, "Quitting.")
        return False

    state.status = f"Unknown key {key!r}. Press h for STM32 help."
    return True


def choose_port(cli_port: str | None, ports: Iterable[str]) -> str:
    if cli_port:
        return cli_port
    ports = list(ports)
    if not ports:
        raise SystemExit("No STM32 serial port found. Connect the STM32 Micro-USB cable and run again.")
    usbmodems = [p for p in ports if "usbmodem" in p]
    return usbmodems[0] if usbmodems else ports[0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem2103")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--motor", type=int, default=2, choices=range(1, MOTOR_COUNT + 1))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    port = choose_port(args.port, find_ports())
    state = UiState(port=port, baud=args.baud, motor=args.motor)
    fd = open_serial(port, args.baud)
    serial_buf = b""

    def stop_on_signal(_signum: int, _frame: object) -> None:
        try:
            write_line(fd, "S")
        finally:
            raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop_on_signal)

    with RawTerminal() as raw:
        try:
            write_line(fd, "LOG 0")
            time.sleep(0.05)
            write_line(fd, "S")
            time.sleep(0.05)
            render(state)

            alive = True
            last_render = time.monotonic()
            while alive:
                readable, _, _ = select.select([fd, sys.stdin.fileno()], [], [], 0.05)

                if fd in readable:
                    try:
                        data = os.read(fd, 4096)
                    except OSError as exc:
                        state.status = f"Serial read failed: {exc}"
                        data = b""
                    if data:
                        serial_buf += data
                        while b"\n" in serial_buf:
                            line, serial_buf = serial_buf.split(b"\n", 1)
                            record_board_line(state, line.decode("utf-8", errors="replace"))

                if sys.stdin.fileno() in readable:
                    key = os.read(sys.stdin.fileno(), 16)
                    if key:
                        alive = handle_key(fd, state, raw, key)
                        render(state)

                now = time.monotonic()
                if now - last_render > 0.5:
                    render(state)
                    last_render = now
        except KeyboardInterrupt:
            send_stop(fd, state, "Interrupted; STOP ALL sent.")
            render(state)
            time.sleep(0.2)
        finally:
            try:
                write_line(fd, "S")
                time.sleep(0.1)
            except OSError:
                pass
            os.close(fd)
    print("\nAll stepper motors stopped. Serial port closed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
