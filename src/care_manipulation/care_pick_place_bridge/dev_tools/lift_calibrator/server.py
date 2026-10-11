#!/usr/bin/env python3
"""Local STM32 lift calibration console.

The server owns the STLink VCP serial port, serves a localhost-only web UI,
and records commands plus human feedback as JSON Lines. It deliberately uses
only the Python standard library so it can run on a clean Mac installation.
"""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import webbrowser
import termios
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse
import uuid


BAUD = 115200
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
MIN_BASE_HZ = 300
MAX_BASE_HZ = 4000
MIN_DURATION_MS = 100
MAX_DURATION_MS = 3000
MAX_TRIM_PERCENT = 30.0
TRIM_STEP_PERCENT = 2.5


def utc_timestamp() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def motor_speeds(base_hz: int, trim_percent: float) -> tuple[int, int]:
    """Return M2-right and M5-left rates without raising either above base.

    Positive trim favours the right side by slowing M5. Negative trim favours
    the left side by slowing M2. Keeping the faster side at the base rate
    avoids reducing the stalled side's available torque by increasing speed.
    """

    trim = clamp(trim_percent, -MAX_TRIM_PERCENT, MAX_TRIM_PERCENT) / 100.0
    if trim >= 0:
        return base_hz, max(100, round(base_hz * (1.0 - trim)))
    return max(100, round(base_hz * (1.0 + trim))), base_hz


class SessionLog:
    def __init__(self, data_dir: Path) -> None:
        data_dir.mkdir(parents=True, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = data_dir / f"lift_session_{stamp}.jsonl"
        self.policy_path = data_dir / "lift_policy.json"
        self._lock = threading.Lock()

    def append(self, record: dict[str, Any]) -> None:
        row = {"timestamp": utc_timestamp(), **record}
        with self._lock:
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(row, separators=(",", ":")) + "\n")

    def load_policy(self) -> dict[str, dict[str, Any]]:
        try:
            data = json.loads(self.policy_path.read_text(encoding="utf-8"))
            return data if isinstance(data, dict) else {}
        except (OSError, json.JSONDecodeError):
            return {}

    def save_policy(self, policy: dict[str, dict[str, Any]]) -> None:
        temporary = self.policy_path.with_suffix(".tmp")
        with self._lock:
            temporary.write_text(json.dumps(policy, indent=2) + "\n", encoding="utf-8")
            temporary.replace(self.policy_path)


class EventStore:
    def __init__(self, session_log: SessionLog) -> None:
        self._events: deque[dict[str, Any]] = deque(maxlen=2000)
        self._next_id = 1
        self._lock = threading.Lock()
        self._session_log = session_log

    def emit(self, kind: str, message: str, **data: Any) -> dict[str, Any]:
        with self._lock:
            event = {
                "id": self._next_id,
                "time": time.time(),
                "kind": kind,
                "message": message,
                "data": data,
            }
            self._next_id += 1
            self._events.append(event)
        self._session_log.append({"type": "event", **event})
        return event

    def after(self, event_id: int) -> list[dict[str, Any]]:
        with self._lock:
            return [event for event in self._events if event["id"] > event_id]

    def latest_id(self) -> int:
        with self._lock:
            return self._next_id - 1


class SerialController:
    def __init__(self, events: EventStore, requested_port: str | None, simulate: bool) -> None:
        self.events = events
        self.requested_port = requested_port
        self.simulate = simulate
        self.fd: int | None = None
        self.port: str | None = "SIMULATED" if simulate else None
        self.connected = simulate
        self._write_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._buffer = b""

    def start(self) -> None:
        if self.simulate:
            self.events.emit("system", "Simulation serial connection ready")
            return
        self._thread = threading.Thread(target=self._run, name="stm32-serial", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        try:
            self.send("S")
        except RuntimeError:
            pass
        self._disconnect()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _discover_port(self) -> str | None:
        if self.requested_port:
            return self.requested_port if Path(self.requested_port).exists() else None
        ports = sorted(glob.glob("/dev/cu.usbmodem*"))
        return ports[0] if ports else None

    @staticmethod
    def _open_serial(port: str) -> int:
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

    def _connect(self) -> bool:
        port = self._discover_port()
        if not port:
            return False
        try:
            fd = self._open_serial(port)
        except OSError as exc:
            self.events.emit("error", f"Could not open {port}: {exc}")
            return False
        self.fd = fd
        self.port = port
        self.connected = True
        self._buffer = b""
        self.events.emit("system", f"Connected to {port} at {BAUD} baud")
        try:
            self.send("S")
        except RuntimeError:
            return False
        return True

    def _disconnect(self) -> None:
        fd = self.fd
        self.fd = None
        was_connected = self.connected
        self.connected = self.simulate
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
        if was_connected and not self.simulate:
            self.events.emit("system", "STM32 serial connection closed")

    def _run(self) -> None:
        last_probe = 0.0
        while not self._stop.is_set():
            if self.fd is None:
                if self._connect():
                    continue
                now = time.monotonic()
                if now - last_probe >= 5.0:
                    last_probe = now
                    try:
                        subprocess.run(
                            ["st-info", "--probe"],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            timeout=2,
                        )
                    except (OSError, subprocess.SubprocessError):
                        pass
                self._stop.wait(0.5)
                continue

            try:
                readable, _, _ = select.select([self.fd], [], [], 0.1)
                if not readable:
                    continue
                data = os.read(self.fd, 4096)
                if not data:
                    raise OSError("serial device returned no data")
                self._buffer += data
                while b"\n" in self._buffer:
                    raw, self._buffer = self._buffer.split(b"\n", 1)
                    text = raw.decode("utf-8", errors="replace").rstrip("\r")
                    if text.strip():
                        self.events.emit("board", text)
            except (OSError, ValueError) as exc:
                self.events.emit("error", f"Serial disconnected: {exc}")
                self._disconnect()

    def send(self, command: str) -> None:
        clean = command.strip()
        if not clean or "\n" in clean or "\r" in clean:
            raise RuntimeError("Invalid serial command")
        if self.simulate:
            self.events.emit("command", clean)
            self.events.emit("board", f"SIM OK: {clean}")
            if clean.startswith("LIFT "):
                try:
                    duration_ms = int(clean.split()[-1])
                except (ValueError, IndexError):
                    duration_ms = 500
                timer = threading.Timer(
                    duration_ms / 1000.0,
                    lambda: self.events.emit("board", "SIM OK: LIFT complete -> M2/M5 stopped"),
                )
                timer.daemon = True
                timer.start()
            return
        with self._write_lock:
            if self.fd is None or not self.connected:
                raise RuntimeError("STM32 is not connected")
            try:
                os.write(self.fd, (clean + "\n").encode("ascii"))
            except OSError as exc:
                self._disconnect()
                raise RuntimeError(f"Serial write failed: {exc}") from exc
        self.events.emit("command", clean)


class LiftCalibration:
    LABELS = {"smooth", "left_stall", "right_stall", "both_stall", "vibration"}

    def __init__(self, serial: SerialController, events: EventStore, session_log: SessionLog) -> None:
        self.serial = serial
        self.events = events
        self.session_log = session_log
        self.policy = session_log.load_policy()
        self.armed = False
        self.active_trial: dict[str, Any] | None = None
        self.last_trial: dict[str, Any] | None = None
        self.feedback_counts = {label: 0 for label in self.LABELS}
        self._lock = threading.Lock()

    @staticmethod
    def context_key(direction: str, height_percent: float) -> str:
        height = clamp(height_percent, 0.0, 100.0)
        band_start = min(80, int(height // 20) * 20)
        return f"{direction.upper()}:{band_start:02d}-{band_start + 20:02d}"

    def recommendation(self, direction: str, height_percent: float) -> dict[str, Any]:
        key = self.context_key(direction, height_percent)
        entry = self.policy.get(key, {})
        return {
            "context": key,
            "base_hz": int(entry.get("base_hz", 1000)),
            "trim_percent": float(entry.get("trim_percent", 0.0)),
            "trials": int(entry.get("trials", 0)),
            "smooth": int(entry.get("smooth", 0)),
        }

    def set_armed(self, armed: bool) -> None:
        if armed and not self.serial.connected:
            raise RuntimeError("Connect the STM32 before arming controls")
        if not armed:
            try:
                self.serial.send("S")
            except RuntimeError:
                pass
            with self._lock:
                self.active_trial = None
        self.armed = armed
        self.events.emit("safety", "Controls armed" if armed else "Controls disarmed; STOP sent")

    def stop(self, reason: str = "operator") -> None:
        self.serial.send("S")
        with self._lock:
            if self.active_trial is not None:
                self.active_trial["stopped_reason"] = reason
                self.active_trial["stopped_at"] = time.time()
                self.last_trial = dict(self.active_trial)
                self.active_trial = None
        self.events.emit("safety", f"STOP sent ({reason})")

    def _validate_motion(self, direction: str, base_hz: int, duration_ms: int) -> str:
        direction = direction.upper()
        if direction not in {"UP", "DOWN"}:
            raise RuntimeError("Direction must be UP or DOWN")
        if not MIN_BASE_HZ <= base_hz <= MAX_BASE_HZ:
            raise RuntimeError(f"Base speed must be {MIN_BASE_HZ}..{MAX_BASE_HZ} Hz")
        if not MIN_DURATION_MS <= duration_ms <= MAX_DURATION_MS:
            raise RuntimeError(
                f"Duration must be {MIN_DURATION_MS}..{MAX_DURATION_MS} ms"
            )
        if not self.armed:
            raise RuntimeError("Arm controls before starting motion")
        if not self.serial.connected:
            raise RuntimeError("STM32 is not connected")
        return direction

    def start_trial(
        self,
        direction: str,
        base_hz: int,
        trim_percent: float,
        duration_ms: int,
        height_percent: float,
    ) -> dict[str, Any]:
        direction = self._validate_motion(direction, base_hz, duration_ms)
        trim = clamp(trim_percent, -MAX_TRIM_PERCENT, MAX_TRIM_PERCENT)
        height = clamp(height_percent, 0.0, 100.0)
        m2_hz, m5_hz = motor_speeds(base_hz, trim)
        trial = {
            "id": uuid.uuid4().hex[:10],
            "started_at": time.time(),
            "direction": direction,
            "height_percent": height,
            "context": self.context_key(direction, height),
            "base_hz": base_hz,
            "trim_percent": trim,
            "m2_hz": m2_hz,
            "m5_hz": m5_hz,
            "duration_ms": duration_ms,
            "kind": "pair",
        }
        command = f"LIFT {direction} {m2_hz} {m5_hz} {duration_ms}"
        with self._lock:
            self.active_trial = trial
            self.last_trial = dict(trial)
        try:
            self.serial.send(command)
        except RuntimeError:
            with self._lock:
                self.active_trial = None
            raise
        self.session_log.append({"type": "trial", **trial, "command": command})
        self.events.emit(
            "trial",
            f"{direction} trial: M2 {m2_hz} Hz, M5 {m5_hz} Hz for {duration_ms} ms",
            trial=trial,
        )
        return trial

    def jog(self, side: str, direction: str, hz: int, duration_ms: int) -> dict[str, Any]:
        direction = self._validate_motion(direction, hz, duration_ms)
        if side not in {"left", "right"}:
            raise RuntimeError("Jog side must be left or right")
        m2_hz = hz if side == "right" else 0
        m5_hz = hz if side == "left" else 0
        trial = {
            "id": uuid.uuid4().hex[:10],
            "started_at": time.time(),
            "direction": direction,
            "base_hz": hz,
            "trim_percent": 0.0,
            "m2_hz": m2_hz,
            "m5_hz": m5_hz,
            "duration_ms": duration_ms,
            "kind": f"{side}_jog",
        }
        command = f"LIFT {direction} {m2_hz} {m5_hz} {duration_ms}"
        with self._lock:
            self.active_trial = trial
            self.last_trial = dict(trial)
        self.serial.send(command)
        self.session_log.append({"type": "trial", **trial, "command": command})
        self.events.emit("trial", f"{side.title()} jog {direction} for {duration_ms} ms", trial=trial)
        return trial

    def feedback(self, label: str, note: str = "") -> dict[str, Any]:
        if label not in self.LABELS:
            raise RuntimeError("Unknown feedback label")
        with self._lock:
            trial = dict(self.active_trial or self.last_trial or {})
            self.feedback_counts[label] += 1
        if not trial:
            raise RuntimeError("Run a trial before recording feedback")

        if label != "smooth":
            try:
                self.serial.send("S")
            except RuntimeError as exc:
                self.events.emit("error", f"STOP could not be sent: {exc}")
            finally:
                with self._lock:
                    if self.active_trial is not None:
                        self.last_trial = dict(self.active_trial)
                        self.active_trial = None

        latency_ms = round((time.time() - float(trial["started_at"])) * 1000)
        context = trial.get("context")
        recommendation: dict[str, Any] | None = None
        if context:
            entry = self.policy.setdefault(
                context,
                {
                    "base_hz": int(trial["base_hz"]),
                    "trim_percent": float(trial["trim_percent"]),
                    "trials": 0,
                    "smooth": 0,
                },
            )
            entry["trials"] = int(entry.get("trials", 0)) + 1
            entry["last_label"] = label
            entry["updated_at"] = utc_timestamp()

            trim = float(entry.get("trim_percent", trial["trim_percent"]))
            base_hz = int(entry.get("base_hz", trial["base_hz"]))
            if label == "right_stall":
                trim += TRIM_STEP_PERCENT
            elif label == "left_stall":
                trim -= TRIM_STEP_PERCENT
            elif label in {"both_stall", "vibration"}:
                base_hz = max(MIN_BASE_HZ, base_hz - 200)
            elif label == "smooth":
                entry["smooth"] = int(entry.get("smooth", 0)) + 1

            entry["trim_percent"] = round(
                clamp(trim, -MAX_TRIM_PERCENT, MAX_TRIM_PERCENT), 2
            )
            entry["base_hz"] = base_hz
            self.session_log.save_policy(self.policy)
            recommendation = self.recommendation(
                str(trial["direction"]), float(trial.get("height_percent", 0.0))
            )

        record = {
            "type": "feedback",
            "label": label,
            "note": note.strip()[:300],
            "latency_ms": latency_ms,
            "trial": trial,
            "recommendation": recommendation,
        }
        self.session_log.append(record)
        self.events.emit(
            "feedback",
            label.replace("_", " ").title(),
            trial_id=trial["id"],
            latency_ms=latency_ms,
            recommendation=recommendation,
        )
        return record

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            active = dict(self.active_trial) if self.active_trial else None
            last = dict(self.last_trial) if self.last_trial else None
            counts = dict(self.feedback_counts)
        if active and time.time() >= active["started_at"] + active["duration_ms"] / 1000.0 + 0.4:
            with self._lock:
                self.last_trial = dict(active)
                self.active_trial = None
            active = None
        return {
            "connected": self.serial.connected,
            "serial_port": self.serial.port,
            "armed": self.armed,
            "active_trial": active,
            "last_trial": last,
            "feedback_counts": counts,
            "log_path": str(self.session_log.path),
            "event_id": self.events.latest_id(),
        }


class CalibrationHandler(SimpleHTTPRequestHandler):
    server_version = "LiftCalibration/1.0"

    @property
    def app(self) -> "CalibrationServer":
        return self.server  # type: ignore[return-value]

    def log_message(self, format_string: str, *args: Any) -> None:
        return

    def _json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > 64_000:
            return {}
        try:
            payload = json.loads(self.rfile.read(length))
        except json.JSONDecodeError as exc:
            raise RuntimeError("Invalid JSON request") from exc
        if not isinstance(payload, dict):
            raise RuntimeError("JSON body must be an object")
        return payload

    def _send_json(self, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/state":
            self._send_json(self.app.calibration.snapshot())
            return
        if parsed.path == "/api/events":
            params = parse_qs(parsed.query)
            try:
                after = int(params.get("after", ["0"])[0])
            except ValueError:
                after = 0
            self._send_json({"events": self.app.events.after(after)})
            return
        if parsed.path == "/api/recommendation":
            params = parse_qs(parsed.query)
            direction = params.get("direction", ["UP"])[0]
            try:
                height = float(params.get("height", ["0"])[0])
            except ValueError:
                height = 0.0
            self._send_json(self.app.calibration.recommendation(direction, height))
            return
        if parsed.path == "/":
            self.path = "/index.html"
        super().do_GET()

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        try:
            data = self._json_body()
            if parsed.path == "/api/arm":
                self.app.calibration.set_armed(bool(data.get("armed")))
                result: Any = self.app.calibration.snapshot()
            elif parsed.path == "/api/stop":
                self.app.calibration.stop(str(data.get("reason", "operator")))
                result = self.app.calibration.snapshot()
            elif parsed.path == "/api/trial":
                result = self.app.calibration.start_trial(
                    str(data.get("direction", "UP")),
                    int(data.get("base_hz", 1000)),
                    float(data.get("trim_percent", 0.0)),
                    int(data.get("duration_ms", 1000)),
                    float(data.get("height_percent", 0.0)),
                )
            elif parsed.path == "/api/jog":
                result = self.app.calibration.jog(
                    str(data.get("side", "left")),
                    str(data.get("direction", "UP")),
                    int(data.get("hz", 500)),
                    int(data.get("duration_ms", 250)),
                )
            elif parsed.path == "/api/feedback":
                result = self.app.calibration.feedback(
                    str(data.get("label", "")), str(data.get("note", ""))
                )
            else:
                self._send_json({"error": "Unknown API endpoint"}, HTTPStatus.NOT_FOUND)
                return
            self._send_json({"ok": True, "result": result})
        except (RuntimeError, ValueError, TypeError) as exc:
            self.app.events.emit("error", str(exc))
            self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)


class CalibrationServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        address: tuple[str, int],
        handler: type[CalibrationHandler],
        directory: Path,
        serial: SerialController,
        events: EventStore,
        calibration: LiftCalibration,
    ) -> None:
        self.static_directory = directory
        self.serial = serial
        self.events = events
        self.calibration = calibration

        def factory(*args: Any, **kwargs: Any) -> CalibrationHandler:
            return handler(*args, directory=str(directory), **kwargs)

        super().__init__(address, factory)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--serial", dest="serial_port", help="STM32 VCP device path")
    parser.add_argument("--data-dir", type=Path, help="Session log and learned-policy folder")
    parser.add_argument("--simulate", action="store_true", help="Run without hardware")
    parser.add_argument("--no-browser", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = Path(__file__).resolve().parent
    data_dir = args.data_dir or root / "data"
    session_log = SessionLog(data_dir)
    events = EventStore(session_log)
    serial_controller = SerialController(events, args.serial_port, args.simulate)
    calibration = LiftCalibration(serial_controller, events, session_log)
    server = CalibrationServer(
        (args.host, args.port),
        CalibrationHandler,
        root / "static",
        serial_controller,
        events,
        calibration,
    )

    stopping = threading.Event()

    def shutdown_handler(signum: int, frame: Any) -> None:
        if stopping.is_set():
            return
        stopping.set()
        try:
            serial_controller.send("S")
        except RuntimeError:
            pass
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    serial_controller.start()
    url = f"http://{args.host}:{args.port}"
    print(f"Lift calibration console: {url}")
    print(f"Session log: {session_log.path}")

    if not args.no_browser:
        try:
            webbrowser.open(url)
        except OSError:
            pass

    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        serial_controller.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
