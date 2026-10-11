#!/usr/bin/env python3
"""Local conveyor calibration and named-action console.

The server owns the Nucleo ST-Link serial port, serves a localhost-only UI,
and records every command, motion segment, ToF reading, and operator note as
JSON Lines. It uses only the Python standard library.
"""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
import math
import os
from pathlib import Path
import re
import select
import signal
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
DEFAULT_PORT = 8768
MIN_HZ = 100
MAX_HZ = 4000
MAX_STEPS = 10_000_000
# The flashed Nucleo `C` command accepts no more than 100,000 steps for any
# selected motor. An action may be much larger; coordinated plans are split
# into bounded commands before they are sent to the board.
MAX_BOARD_SEGMENT_STEPS = 100_000
RAMP_LEVELS = (0.2, 0.4, 0.6, 0.8, 1.0)
MAX_CLAMP_PROFILE_POINTS = 24
MAX_CLAMP_OFFSET_STEPS = 100_000
MAX_COMPOSITE_SLICES = 60
MIN_LAC_SPEED_PERCENT_S = 2
MAX_LAC_SPEED_PERCENT_S = 100
LAC_UPDATE_PERIOD_S = 0.05
DEFAULT_LAC_SUPPORT_STEPS = 300
DEFAULT_LAC_SUPPORT_HZ = 500
DEFAULT_LAC_SUPPORT_RAMP_PERCENT = 100
LAC_SUPPORT_COMMAND_INTERVAL_S = 0.05
MAX_LAC_SUPPORT_SEGMENTS = 400

GROUPS: dict[str, dict[str, Any]] = {
    "mechanism": {
        "label": "Side mechanisms (M1 right + M6 left)",
        "motors": (1, 6),
        "presets": {
            "clamp": ("R", "R"),
            "unclamp": ("F", "F"),
            "shift_left": ("R", "F"),
            "shift_right": ("F", "R"),
        },
    },
    "belts": {
        "label": "Conveyor belts (M3 + M4)",
        "motors": (3, 4),
        "presets": {
            "box_out": ("R", "F"),
            "box_in": ("F", "R"),
        },
    },
    "m1": {
        "label": "M1 physical-right mechanism",
        "motors": (1,),
        "presets": {"forward": ("F",), "reverse": ("R",)},
    },
    "m6": {
        "label": "M6 physical-left mechanism",
        "motors": (6,),
        "presets": {"forward": ("F",), "reverse": ("R",)},
    },
    "m3": {
        "label": "M3 conveyor belt",
        "motors": (3,),
        "presets": {"forward": ("F",), "reverse": ("R",)},
    },
    "m4": {
        "label": "M4 conveyor belt",
        "motors": (4,),
        "presets": {"forward": ("F",), "reverse": ("R",)},
    },
    "m7": {
        "label": "M7 geared X-axis angle motor",
        "motors": (7,),
        "presets": {"clockwise": ("F",), "counterclockwise": ("R",)},
    },
}

DEFAULT_ACTIONS: tuple[dict[str, Any], ...] = (
    {"id": "clamp-box", "name": "Clamp box", "kind": "stepper", "group": "mechanism",
     "directions": ["R", "R"], "order": 10, "pair_id": "release-box"},
    {"id": "release-box", "name": "Release box", "kind": "stepper", "group": "mechanism",
     "directions": ["F", "F"], "order": 20, "pair_id": "clamp-box"},
    {"id": "move-left", "name": "System: move left", "kind": "stepper", "group": "mechanism",
     "directions": ["R", "F"], "order": 30, "pair_id": "move-right"},
    {"id": "move-right", "name": "System: move right", "kind": "stepper", "group": "mechanism",
     "directions": ["F", "R"], "order": 40, "pair_id": "move-left"},
    {"id": "box-out", "name": "Conveyor: box out", "kind": "stepper", "group": "belts",
     "directions": ["R", "F"], "order": 50, "pair_id": "box-in"},
    {"id": "box-in", "name": "Conveyor: box in", "kind": "stepper", "group": "belts",
     "directions": ["F", "R"], "order": 60, "pair_id": "box-out"},
    {"id": "actuator-box-out", "name": "Linear actuator: box out",
     "kind": "lac", "group": "lac", "order": 70, "pair_id": None},
    {"id": "actuator-box-in", "name": "Linear actuator: box in",
     "kind": "lac", "group": "lac", "order": 80, "pair_id": None},
)
DEFAULT_ACTION_BY_ID = {action["id"]: action for action in DEFAULT_ACTIONS}
ACTION_PAIRS = {
    action["id"]: action["pair_id"] for action in DEFAULT_ACTIONS
    if action.get("pair_id")
}

# Open-loop alignment trim requested during the transfer study.  The value is
# relative to the action's editable base step count, so changing 225,000 later
# still preserves the 300-pulse correction on the intended physical side.
SHIFT_MOTOR_STEP_OFFSETS: dict[str, dict[str, int]] = {
    "move-left": {"1": 300, "6": 0},   # physical-right M1 travels farther
    "move-right": {"1": 0, "6": 300},  # physical-left M6 travels farther
}

TOF_RE = re.compile(
    r"TOF PHYSICAL_RIGHT (?P<distance>\d+) mm raw=(?P<raw>\d+) mm "
    r"avg_n=(?P<avg>\d+) status=(?P<status>\d+) samples=(?P<samples>\d+) "
    r"errors=(?P<errors>\d+) age=(?P<age>\d+)ms"
)
LAC_RE = re.compile(r"LAC commanded: 1=(?P<left>\d+)%.*2=(?P<right>\d+)%")


def utc_timestamp() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")


def allocate(total: int, buckets: int) -> list[int]:
    if buckets <= 0:
        return []
    base, remainder = divmod(total, buckets)
    return [base + (1 if index < remainder else 0) for index in range(buckets)]


def build_ramp_plan(steps: int, max_hz: int, ramp_percent: int) -> list[dict[str, Any]]:
    """Build a symmetric stepped ramp whose planned pulses sum to steps."""
    if not 1 <= steps <= MAX_STEPS:
        raise RuntimeError(f"Steps must be 1..{MAX_STEPS}")
    if not MIN_HZ <= max_hz <= MAX_HZ:
        raise RuntimeError(f"Speed must be {MIN_HZ}..{MAX_HZ} Hz")
    if not 0 <= ramp_percent <= 45:
        raise RuntimeError("Ramp must be 0..45%")

    def bounded(segments: list[dict[str, Any]]) -> list[dict[str, Any]]:
        result: list[dict[str, Any]] = []
        for segment in segments:
            remaining = int(segment["steps"])
            hz = int(segment["hz"])
            while remaining:
                chunk = min(remaining, MAX_BOARD_SEGMENT_STEPS)
                result.append({"hz": hz, "steps": chunk, "duration_s": chunk / hz})
                remaining -= chunk
        return result

    if ramp_percent == 0 or steps < 10:
        return bounded(
            [{"hz": max_hz, "steps": steps, "duration_s": steps / max_hz}]
        )

    ramp_each = min(steps // 2, round(steps * ramp_percent / 100.0))
    if ramp_each == 0:
        return bounded(
            [{"hz": max_hz, "steps": steps, "duration_s": steps / max_hz}]
        )

    ascending_steps = allocate(ramp_each, len(RAMP_LEVELS))
    descending_steps = list(reversed(ascending_steps))
    hold_steps = steps - (2 * ramp_each)
    segments: list[dict[str, Any]] = []

    def add_segment(fraction: float, segment_steps: int) -> None:
        if segment_steps <= 0:
            return
        hz = max(MIN_HZ, min(max_hz, round(max_hz * fraction)))
        if segments and segments[-1]["hz"] == hz:
            segments[-1]["steps"] += segment_steps
            segments[-1]["duration_s"] = segments[-1]["steps"] / hz
        else:
            segments.append(
                {"hz": hz, "steps": segment_steps, "duration_s": segment_steps / hz}
            )

    for fraction, segment_steps in zip(RAMP_LEVELS, ascending_steps):
        add_segment(fraction, segment_steps)
    add_segment(1.0, hold_steps)
    for fraction, segment_steps in zip(reversed(RAMP_LEVELS), descending_steps):
        add_segment(fraction, segment_steps)

    return bounded(segments)


def build_full_duration_support_plan(
    steps: int, max_hz: int, motion_duration_s: float
) -> tuple[list[dict[str, Any]], float]:
    """Spread monotonic clamp pulses across the complete action timeline.

    Unlike the motor speed ramp, this never decelerates or releases the clamp.
    Its cumulative clamp position rises from 0 to ``steps`` throughout 100% of
    the action. Short pulse bursts are evenly scheduled when the required
    average rate is below the Nucleo's 100 Hz minimum command frequency.
    """
    if not 1 <= steps <= MAX_STEPS:
        raise RuntimeError(f"Steps must be 1..{MAX_STEPS}")
    if not MIN_HZ <= max_hz <= MAX_HZ:
        raise RuntimeError(f"Speed must be {MIN_HZ}..{MAX_HZ} Hz")
    if motion_duration_s < 0:
        raise RuntimeError("Motion duration cannot be negative")

    coverage_duration_s = max(float(motion_duration_s), steps / max_hz)
    desired_segments = max(
        1, round(coverage_duration_s / LAC_SUPPORT_COMMAND_INTERVAL_S)
    )
    segment_count = min(steps, MAX_LAC_SUPPORT_SEGMENTS, desired_segments)
    chunks = allocate(steps, segment_count)
    nominal_slot_s = coverage_duration_s / segment_count
    pulse_hz = max(
        MIN_HZ,
        min(max_hz, math.ceil(max(chunks) / nominal_slot_s)),
    )
    durations = [chunk / pulse_hz for chunk in chunks]
    if segment_count == 1:
        starts = [0.0]
    else:
        start_span = max(0.0, coverage_duration_s - durations[-1])
        starts = [
            start_span * index / (segment_count - 1)
            for index in range(segment_count)
        ]
    plan = [
        {
            "hz": pulse_hz,
            "steps": chunk,
            "duration_s": duration,
            "start_offset_s": start,
        }
        for chunk, duration, start in zip(chunks, durations, starts)
    ]
    return plan, coverage_duration_s


def apply_motor_step_offsets(
    primary_plan: list[dict[str, Any]],
    motors: tuple[int, ...],
    offsets_by_motor: dict[str, int],
) -> list[dict[str, Any]]:
    """Spread small per-motor pulse corrections across the same ramp.

    Every coordinated slice still starts and finishes all selected motors
    together.  The motor with extra pulses therefore runs fractionally faster
    within each slice instead of performing a separate visible nudge at the end.
    """
    if len(motors) < 2:
        raise RuntimeError("Per-motor step correction requires a paired motor group")
    total_primary = sum(int(segment["steps"]) for segment in primary_plan)
    offsets = [int(offsets_by_motor.get(str(motor), 0)) for motor in motors]
    totals = [total_primary + offset for offset in offsets]
    if any(total < 1 or total > MAX_STEPS for total in totals):
        raise RuntimeError(f"Corrected motor steps must be 1..{MAX_STEPS}")
    if not any(offsets):
        return primary_plan

    result: list[dict[str, Any]] = []
    primary_cursor = 0
    allocated = [0 for _ in motors]
    for index, segment in enumerate(primary_plan):
        primary_steps = int(segment["steps"])
        primary_cursor += primary_steps
        cumulative = [
            total if index == len(primary_plan) - 1
            else round(primary_cursor * total / total_primary)
            for total in totals
        ]
        motor_steps = [
            target - previous for target, previous in zip(cumulative, allocated)
        ]
        allocated = cumulative
        largest = max(motor_steps)
        requested_duration = float(segment["duration_s"])
        duration_floor = max(requested_duration, largest / MAX_HZ)
        slot_hz = max(1, min(MAX_HZ, round(largest / duration_floor)))
        # Offsets are applied after the base ramp is split, so a 100,000-step
        # base slice can become slightly larger than the board's per-motor
        # limit. Re-split the corrected slice while preserving each motor's
        # proportional pulse count and the common synchronized duration.
        piece_count = math.ceil(largest / MAX_BOARD_SEGMENT_STEPS)
        largest_pieces = allocate(largest, piece_count)
        piece_cursor = 0
        motor_piece_allocated = [0 for _ in motors]
        primary_piece_allocated = 0
        for piece_index, largest_piece in enumerate(largest_pieces):
            piece_cursor += largest_piece
            cumulative_motor_steps = [
                total if piece_index == piece_count - 1
                else round(piece_cursor * total / largest)
                for total in motor_steps
            ]
            piece_motor_steps = [
                target - previous
                for target, previous in zip(
                    cumulative_motor_steps, motor_piece_allocated
                )
            ]
            motor_piece_allocated = cumulative_motor_steps
            cumulative_primary = (
                primary_steps
                if piece_index == piece_count - 1
                else round(piece_cursor * primary_steps / largest)
            )
            piece_primary_steps = cumulative_primary - primary_piece_allocated
            primary_piece_allocated = cumulative_primary
            result.append(
                {
                    **segment,
                    "motors": list(motors),
                    "motor_steps": piece_motor_steps,
                    "steps": largest_piece,
                    "primary_steps": piece_primary_steps,
                    "hz": slot_hz,
                    "duration_s": largest_piece / slot_hz,
                    "coordinated": True,
                }
            )
    return result


def normalize_motor_step_offsets(
    raw_offsets: Any,
    motors: tuple[int, ...],
    group: str,
) -> dict[str, int]:
    """Validate editable extra pulses for a synchronized motor pair."""
    if raw_offsets in (None, ""):
        raw_offsets = {}
    if not isinstance(raw_offsets, dict):
        raise RuntimeError("Motor step adjustments must be a motor/value map")
    offsets: dict[str, int] = {}
    for motor in motors:
        try:
            value = int(raw_offsets.get(str(motor), 0))
        except (TypeError, ValueError):
            raise RuntimeError(f"M{motor} extra steps must be a whole number")
        if not 0 <= value <= MAX_STEPS:
            raise RuntimeError(f"M{motor} extra steps must be 0..{MAX_STEPS}")
        offsets[str(motor)] = value
    if any(offsets.values()) and (group != "mechanism" or len(motors) != 2):
        raise RuntimeError(
            "Per-motor extra steps are only available for the paired side mechanisms"
        )
    return offsets


def normalize_clamp_profile(
    raw_profile: Any, total_steps: int
) -> list[dict[str, int]]:
    """Validate cumulative clamp offsets against conveyor progress.

    Positive offset means M1/M6 move in the verified clamp direction (R/R),
    while a negative change means release (F/F). The curve always begins at
    zero because it is relative to the mechanism position at action start.
    """
    if not isinstance(raw_profile, list):
        raise RuntimeError("Clamp profile must be a list of points")
    if not 2 <= len(raw_profile) <= MAX_CLAMP_PROFILE_POINTS:
        raise RuntimeError(
            f"Clamp profile must contain 2..{MAX_CLAMP_PROFILE_POINTS} points"
        )
    points: list[dict[str, int]] = []
    for raw_point in raw_profile:
        if not isinstance(raw_point, dict):
            raise RuntimeError("Each clamp profile point must contain step and offset")
        try:
            at_step = int(raw_point.get("at_step"))
            offset_steps = int(raw_point.get("offset_steps"))
        except (TypeError, ValueError) as exc:
            raise RuntimeError("Clamp profile values must be whole steps") from exc
        if not 0 <= at_step <= total_steps:
            raise RuntimeError(
                f"Clamp profile positions must be within 0..{total_steps} conveyor steps"
            )
        if abs(offset_steps) > MAX_CLAMP_OFFSET_STEPS:
            raise RuntimeError(
                f"Clamp offsets must be within ±{MAX_CLAMP_OFFSET_STEPS} steps"
            )
        points.append({"at_step": at_step, "offset_steps": offset_steps})
    if points[0] != {"at_step": 0, "offset_steps": 0}:
        raise RuntimeError("Clamp profile must start at conveyor step 0 and clamp offset 0")
    if points[-1]["at_step"] != total_steps:
        raise RuntimeError(f"Clamp profile must end at conveyor step {total_steps}")
    if any(
        second["at_step"] <= first["at_step"]
        for first, second in zip(points, points[1:])
    ):
        raise RuntimeError("Clamp profile conveyor positions must increase")
    return points


def clamp_profile_value(points: list[dict[str, int]], at_step: int) -> int:
    at_step = max(points[0]["at_step"], min(points[-1]["at_step"], at_step))
    for first, second in zip(points, points[1:]):
        if at_step > second["at_step"]:
            continue
        width = second["at_step"] - first["at_step"]
        fraction = (at_step - first["at_step"]) / width
        return round(
            first["offset_steps"]
            + fraction * (second["offset_steps"] - first["offset_steps"])
        )
    return points[-1]["offset_steps"]


def reverse_clamp_profile(
    points: list[dict[str, int]], total_steps: int
) -> list[dict[str, int]]:
    """Return the exact time-reversed curve relative to its new start."""
    end_offset = points[-1]["offset_steps"]
    return [
        {
            "at_step": total_steps - point["at_step"],
            "offset_steps": point["offset_steps"] - end_offset,
        }
        for point in reversed(points)
    ]


def rescale_clamp_profile(
    points: list[dict[str, int]], old_total: int, new_total: int
) -> list[dict[str, int]]:
    """Preserve a curve's relative timing when paired conveyor length changes."""
    if old_total <= 0:
        raise RuntimeError("Previous conveyor step total must be positive")
    scaled: list[dict[str, int]] = []
    for index, point in enumerate(points):
        at_step = (
            0 if index == 0 else new_total
            if index == len(points) - 1 else
            round(point["at_step"] / old_total * new_total)
        )
        if scaled and at_step <= scaled[-1]["at_step"]:
            # Very small new totals can collapse interior points. The endpoint
            # still preserves the final offset; collapsed interior points are
            # omitted rather than creating an invalid zero-width range.
            if index != len(points) - 1:
                continue
            while len(scaled) > 1 and scaled[-1]["at_step"] >= new_total:
                scaled.pop()
        scaled.append(
            {"at_step": at_step, "offset_steps": point["offset_steps"]}
        )
    if len(scaled) < 2:
        scaled = [
            {"at_step": 0, "offset_steps": 0},
            {"at_step": new_total, "offset_steps": points[-1]["offset_steps"]},
        ]
    return normalize_clamp_profile(scaled, new_total)


def slice_clamp_profile(
    points: list[dict[str, int]], start: int, target: int,
    actual_start_offset: int | None = None,
) -> list[dict[str, int]]:
    """Extract a curve for an editor demo moving from start to target."""
    distance = abs(target - start)
    if distance == 0:
        return [{"at_step": 0, "offset_steps": 0},
                {"at_step": 1, "offset_steps": 0}]
    actual = (
        clamp_profile_value(points, start)
        if actual_start_offset is None else int(actual_start_offset)
    )
    between = [
        point for point in points
        if min(start, target) < point["at_step"] < max(start, target)
    ]
    if target < start:
        between.reverse()
    traversal = [{"at_step": 0, "offset_steps": 0}]
    for point in between:
        traversal.append(
            {
                "at_step": abs(point["at_step"] - start),
                "offset_steps": point["offset_steps"] - actual,
            }
        )
    traversal.append(
        {
            "at_step": distance,
            "offset_steps": clamp_profile_value(points, target) - actual,
        }
    )
    return traversal


def build_composite_plan(
    primary_plan: list[dict[str, Any]],
    belt_directions: list[str],
    clamp_profile: list[dict[str, int]],
) -> list[dict[str, Any]]:
    """Run belt travel and M1/M6 clamp pulses over the same time windows.

    The Nucleo's coordinated pulse distributor gives every selected motor an
    exact pulse total while starting and finishing the slice together. A steep
    clamp curve may therefore slow the belts below their requested top speed;
    clamp motion never exceeds MAX_HZ and the conveyor does not pause for it.
    """
    total_steps = sum(int(segment["steps"]) for segment in primary_plan)
    plan_ranges: list[tuple[int, int, int]] = []
    cursor = 0
    for segment in primary_plan:
        end = cursor + int(segment["steps"])
        plan_ranges.append((cursor, end, int(segment["hz"])))
        cursor = end

    boundaries = {0, total_steps}
    boundaries.update(point["at_step"] for point in clamp_profile)
    boundaries.update(end for _, end, _ in plan_ranges)
    base_boundaries = sorted(boundaries)
    preferred_slice_size = max(
        100, (total_steps + MAX_COMPOSITE_SLICES - 1) // MAX_COMPOSITE_SLICES
    )
    slice_size = min(MAX_BOARD_SEGMENT_STEPS, preferred_slice_size)
    sliced = {0, total_steps}
    for first, second in zip(base_boundaries, base_boundaries[1:]):
        sliced.add(first)
        width = second - first
        clamp_change = abs(
            clamp_profile_value(clamp_profile, second)
            - clamp_profile_value(clamp_profile, first)
        )
        count = max(
            1,
            (width + slice_size - 1) // slice_size,
            (clamp_change + MAX_BOARD_SEGMENT_STEPS - 1)
            // MAX_BOARD_SEGMENT_STEPS,
        )
        # Integer conveyor steps are the smallest possible synchronization
        # unit. The validation below gives a clear error for an impossibly
        # steep clamp curve rather than sending an oversized board command.
        count = min(width, count)
        for index in range(1, count):
            sliced.add(first + round(width * index / count))
        sliced.add(second)
    all_boundaries = sorted(sliced)
    result: list[dict[str, Any]] = []

    def rate_at(step: int) -> int:
        for first, end, hz in plan_ranges:
            if first <= step < end or (step == total_steps and end == total_steps):
                return hz
        return int(primary_plan[-1]["hz"])

    for first, second in zip(all_boundaries, all_boundaries[1:]):
        belt_steps = second - first
        if belt_steps <= 0:
            continue
        hz = rate_at(first)
        clamp_delta = (
            clamp_profile_value(clamp_profile, second)
            - clamp_profile_value(clamp_profile, first)
        )
        clamp_steps = abs(clamp_delta)
        clamp_sign = 1 if clamp_delta > 0 else -1 if clamp_delta < 0 else 0
        clamp_dirs = ["R", "R"] if clamp_sign > 0 else ["F", "F"]
        motors = [3, 4]
        directions = list(belt_directions)
        motor_steps = [belt_steps, belt_steps]
        if clamp_steps:
            motors = [1, 3, 4, 6]
            directions = [
                clamp_dirs[0], belt_directions[0], belt_directions[1], clamp_dirs[1]
            ]
            motor_steps = [clamp_steps, belt_steps, belt_steps, clamp_steps]

        if max(motor_steps) > MAX_BOARD_SEGMENT_STEPS:
            raise RuntimeError(
                "Clamp profile changes too steeply: no coordinated board "
                f"command may exceed {MAX_BOARD_SEGMENT_STEPS:,} steps per motor"
            )

        largest_motor_steps = max(motor_steps)
        desired_duration = belt_steps / hz
        minimum_clamp_duration = clamp_steps / MAX_HZ if clamp_steps else 0.0
        duration_floor = max(desired_duration, minimum_clamp_duration)
        slot_hz = max(
            1,
            min(MAX_HZ, round(largest_motor_steps / duration_floor)),
        )
        duration_s = largest_motor_steps / slot_hz
        result.append(
            {
                "motors": motors,
                "directions": directions,
                "motor_steps": motor_steps,
                "hz": slot_hz,
                "steps": largest_motor_steps,
                "duration_s": duration_s,
                "primary_steps": belt_steps,
                "clamp_steps": clamp_steps,
                "clamp_sign": clamp_sign,
                "coordinated": True,
                "belt_target_hz": hz,
                "belt_effective_hz": belt_steps / duration_s,
                "clamp_effective_hz": (
                    clamp_steps / duration_s if clamp_steps else 0.0
                ),
            }
        )

    return result


class SessionLog:
    def __init__(self, data_dir: Path) -> None:
        data_dir.mkdir(parents=True, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = data_dir / f"robot_session_{stamp}.jsonl"
        self.actions_path = data_dir / "saved_actions.json"
        self.runtime_path = data_dir / "runtime_state.json"
        self._lock = threading.Lock()

    def append(self, record: dict[str, Any]) -> None:
        row = {"timestamp": utc_timestamp(), **record}
        with self._lock:
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(row, separators=(",", ":")) + "\n")

    def load_actions(self) -> dict[str, dict[str, Any]]:
        try:
            data = json.loads(self.actions_path.read_text(encoding="utf-8"))
            return data if isinstance(data, dict) else {}
        except (OSError, json.JSONDecodeError):
            return {}

    def save_actions(self, actions: dict[str, dict[str, Any]]) -> None:
        temporary = self.actions_path.with_suffix(".tmp")
        with self._lock:
            temporary.write_text(json.dumps(actions, indent=2) + "\n", encoding="utf-8")
            temporary.replace(self.actions_path)

    def load_runtime(self) -> dict[str, Any]:
        try:
            data = json.loads(self.runtime_path.read_text(encoding="utf-8"))
            return data if isinstance(data, dict) else {}
        except (OSError, json.JSONDecodeError):
            return {}

    def save_runtime(self, state: dict[str, Any]) -> None:
        temporary = self.runtime_path.with_suffix(".tmp")
        with self._lock:
            temporary.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
            temporary.replace(self.runtime_path)


class EventStore:
    def __init__(self, session_log: SessionLog) -> None:
        self._events: deque[dict[str, Any]] = deque(maxlen=3000)
        self._next_id = 1
        self._lock = threading.Lock()
        self.session_log = session_log

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
        self.session_log.append({"type": "event", **event})
        return event

    def after(self, event_id: int) -> list[dict[str, Any]]:
        with self._lock:
            return [event for event in self._events if event["id"] > event_id]

    def latest_id(self) -> int:
        with self._lock:
            return self._next_id - 1


class SerialController:
    def __init__(
        self,
        events: EventStore,
        requested_port: str | None,
        simulate: bool,
        line_callback: Any = None,
    ) -> None:
        self.events = events
        self.requested_port = requested_port
        self.simulate = simulate
        self.line_callback = line_callback
        self.fd: int | None = None
        self.port: str | None = "SIMULATED" if simulate else None
        self.connected = simulate
        self._write_lock = threading.Lock()
        self._transaction_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._buffer = b""
        self._sim_lac = {"1": 0, "2": 0}
        self._reply_condition = threading.Condition()
        self._reply_sequence = 0
        self._reply_lines: deque[tuple[int, str]] = deque(maxlen=256)

    def start(self) -> None:
        if self.simulate:
            self.events.emit("system", "Simulation serial connection ready")
            return
        self._thread = threading.Thread(target=self._run, name="nucleo-serial", daemon=True)
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
        termios.tcflush(fd, termios.TCIFLUSH)
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
        self.events.emit("system", f"Connected to Nucleo on {port} at {BAUD} baud")
        try:
            self.send("S")
            self.send("TOF STATUS")
            self.send("LAC STATUS")
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
            self.events.emit("system", "Nucleo serial connection closed")

    def _accept_line(self, line: str) -> None:
        with self._reply_condition:
            self._reply_sequence += 1
            self._reply_lines.append((self._reply_sequence, line))
            self._reply_condition.notify_all()
        if self.line_callback is not None:
            self.line_callback(line)
        self.events.emit("board", line)

    def _run(self) -> None:
        while not self._stop.is_set():
            if self.fd is None:
                if self._connect():
                    continue
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
                    line = raw.decode("utf-8", errors="replace").rstrip("\r")
                    if line.strip():
                        self._accept_line(line)
            except (OSError, ValueError) as exc:
                self.events.emit("error", f"Serial disconnected: {exc}")
                self._disconnect()

    def send(self, command: str) -> None:
        clean = command.strip()
        if not clean or "\n" in clean or "\r" in clean:
            raise RuntimeError("Invalid serial command")
        if self.simulate:
            self.events.emit("command", clean)
            if clean == "TOF STATUS":
                self._accept_line(
                    "TOF PHYSICAL_RIGHT 620 mm raw=618 mm avg_n=6 status=0 "
                    "samples=120 errors=0 age=30ms pin=D13/PA5"
                )
            elif clean == "LAC STATUS":
                self._accept_line(
                    f"LAC commanded: 1={self._sim_lac['1']}% LEFT on A0/PA0, "
                    f"2={self._sim_lac['2']}% RIGHT on A1/PA1"
                )
            elif clean == "LAC HOME":
                self._sim_lac = {"1": 0, "2": 0}
                self._accept_line("SIM OK: LAC HOME")
            elif match := re.fullmatch(r"LAC BOTH (\d+)", clean):
                percent = int(match.group(1))
                self._sim_lac = {"1": percent, "2": percent}
                self._accept_line(f"SIM OK: {clean}")
            elif match := re.fullmatch(r"LAC ([12]) (\d+)", clean):
                self._sim_lac[match.group(1)] = int(match.group(2))
                self._accept_line(f"SIM OK: {clean}")
            else:
                self._accept_line(f"SIM OK: {clean}")
            return
        with self._write_lock:
            if self.fd is None or not self.connected:
                raise RuntimeError("Nucleo is not connected")
            try:
                os.write(self.fd, (clean + "\n").encode("ascii"))
            except OSError as exc:
                self._disconnect()
                raise RuntimeError(f"Serial write failed: {exc}") from exc
        self.events.emit("command", clean)

    def send_and_wait(
        self,
        command: str,
        success_prefixes: tuple[str, ...],
        timeout: float = 1.5,
    ) -> str:
        # The Nucleo CLI has one pending-line slot. Keep the write and its
        # acknowledgement as one transaction so simultaneous LAC and stepper
        # workers cannot overwrite one another's command.
        with self._transaction_lock:
            return self._send_and_wait_locked(
                command, success_prefixes, timeout
            )

    def _send_and_wait_locked(
        self,
        command: str,
        success_prefixes: tuple[str, ...],
        timeout: float = 1.5,
    ) -> str:
        """Send one line and wait until the board confirms processing it.

        USART2 firmware currently has one pending-line slot, so this also
        serializes commands and prevents a later line from being discarded
        while the main loop is still handling the previous one.
        """
        if self.simulate:
            self.send(command)
            return f"SIM ACK: {command}"
        with self._reply_condition:
            after_sequence = self._reply_sequence
        self.send(command)
        deadline = time.monotonic() + timeout
        with self._reply_condition:
            while True:
                for sequence, line in self._reply_lines:
                    if sequence <= after_sequence:
                        continue
                    if line.startswith(success_prefixes):
                        return line
                    if line.startswith(("ERR:", "LOCKED:")):
                        raise RuntimeError(
                            f"Nucleo rejected {command!r}: {line}"
                        )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(
                        f"Nucleo did not acknowledge {command!r} within {timeout:.1f}s"
                    )
                self._reply_condition.wait(remaining)


class RobotConsole:
    def __init__(self, events: EventStore, session_log: SessionLog) -> None:
        self.events = events
        self.session_log = session_log
        self.serial = SerialController(events, None, False, self._handle_board_line)
        self.controls_armed = False
        self.active_motion: dict[str, Any] | None = None
        self.last_motion: dict[str, Any] | None = None
        self.paused_motion: dict[str, Any] | None = None
        self.motor_steps = {str(motor): 0 for motor in (1, 3, 4, 6, 7)}
        self.lac = {"1": 0, "2": 0}
        self.tof: dict[str, Any] = {
            "state": "unknown",
            "valid": False,
            "distance_mm": None,
            "raw_mm": None,
            "status": None,
            "avg_n": 0,
            "samples": 0,
            "errors": 0,
            "updated_at": None,
            "pin": "D13/PA5",
            "physical_side": "right",
        }
        self.actions = session_log.load_actions()
        defaults_added = False
        for default in DEFAULT_ACTIONS:
            if default["id"] in self.actions:
                existing = self.actions[default["id"]]
                defaults = {
                    "kind": default.get("kind", "stepper"),
                    "pair_id": default.get("pair_id"),
                    "tof_mode": (
                        "range" if existing.get("tof_min_mm") is not None
                        and existing.get("tof_max_mm") is not None
                        else "unspecified"
                    ),
                    "tof_min_mm": None,
                    "tof_ideal_mm": existing.get("tof_reference_mm"),
                    "tof_max_mm": None,
                }
                if default.get("kind") == "lac":
                    defaults["lac_speed_percent_s"] = 25
                    defaults["support_clamp_steps"] = DEFAULT_LAC_SUPPORT_STEPS
                    defaults["support_max_hz"] = DEFAULT_LAC_SUPPORT_HZ
                    defaults["support_ramp_percent"] = (
                        DEFAULT_LAC_SUPPORT_RAMP_PERCENT
                    )
                if default["id"] in SHIFT_MOTOR_STEP_OFFSETS:
                    defaults["motor_step_offsets"] = dict(
                        SHIFT_MOTOR_STEP_OFFSETS[default["id"]]
                    )
                if default.get("group") == "belts":
                    action_steps = int(existing.get("steps", 500))
                    defaults.update(
                        {
                            "clamp_profile_enabled": False,
                            "clamp_profile": [
                                {"at_step": 0, "offset_steps": 0},
                                {"at_step": action_steps, "offset_steps": 0},
                            ],
                        }
                    )
                for field, value in defaults.items():
                    if field not in existing:
                        existing[field] = value
                        defaults_added = True
                if (
                    default.get("kind") == "lac"
                    and existing.get("support_ramp_percent") != 100
                ):
                    # This field used to mean a short symmetric speed ramp.
                    # It now records full-action clamp coverage, per the
                    # operator's clarified anti-slip requirement.
                    existing["support_ramp_percent"] = 100
                    defaults_added = True
                if default["id"] in SHIFT_MOTOR_STEP_OFFSETS:
                    action_steps = int(existing.get("steps", 500))
                    corrected_plan = apply_motor_step_offsets(
                        build_ramp_plan(
                            action_steps,
                            int(existing.get("max_hz", 500)),
                            int(existing.get("ramp_percent", 20)),
                        ),
                        GROUPS["mechanism"]["motors"],
                        existing["motor_step_offsets"],
                    )
                    corrected_duration = sum(
                        item["duration_s"] for item in corrected_plan
                    )
                    if existing.get("estimated_duration_s") != corrected_duration:
                        existing["estimated_duration_s"] = corrected_duration
                        defaults_added = True
                    corrected_totals = {
                        motor: action_steps + int(offset)
                        for motor, offset in existing["motor_step_offsets"].items()
                    }
                    if existing.get("motor_step_totals") != corrected_totals:
                        existing["motor_step_totals"] = corrected_totals
                        defaults_added = True
                if not existing.get("configured") and existing.get("name") != default["name"]:
                    existing["name"] = default["name"]
                    defaults_added = True
                continue
            record = {
                **default,
                "created_at": utc_timestamp(),
                "instructions": "",
                "tof_reference_mm": None,
                "tof_mode": "unspecified",
                "tof_min_mm": None,
                "tof_ideal_mm": None,
                "tof_max_mm": None,
                "configured": False,
                "built_in": True,
            }
            if default.get("kind") == "lac":
                record.update(
                    {
                        "lac_target": "both",
                        "lac_percent": 0,
                        "lac_speed_percent_s": 25,
                        "support_clamp_steps": DEFAULT_LAC_SUPPORT_STEPS,
                        "support_max_hz": DEFAULT_LAC_SUPPORT_HZ,
                        "support_ramp_percent": DEFAULT_LAC_SUPPORT_RAMP_PERCENT,
                        "motors": [],
                        "directions": [],
                        "estimated_duration_s": None,
                    }
                )
            else:
                action_offsets = dict(
                    SHIFT_MOTOR_STEP_OFFSETS.get(default["id"], {})
                )
                initial_plan = build_ramp_plan(500, 500, 20)
                if action_offsets:
                    initial_plan = apply_motor_step_offsets(
                        initial_plan,
                        GROUPS[default["group"]]["motors"],
                        action_offsets,
                    )
                record.update(
                    {
                        "motors": list(GROUPS[default["group"]]["motors"]),
                        "motor_step_offsets": action_offsets,
                        "motor_step_totals": {
                            motor: 500 + int(offset)
                            for motor, offset in action_offsets.items()
                        },
                        "steps": 500,
                        "max_hz": 500,
                        "ramp_percent": 20,
                        "estimated_duration_s": sum(
                            item["duration_s"] for item in initial_plan
                        ),
                    }
                )
                if default.get("group") == "belts":
                    record.update(
                        {
                            "clamp_profile_enabled": False,
                            "clamp_profile": [
                                {"at_step": 0, "offset_steps": 0},
                                {"at_step": 500, "offset_steps": 0},
                            ],
                        }
                    )
            self.actions[default["id"]] = record
            defaults_added = True
        if defaults_added:
            session_log.save_actions(self.actions)
        runtime = session_log.load_runtime()
        saved_motor_steps = runtime.get("motor_steps", {})
        if isinstance(saved_motor_steps, dict):
            for motor in self.motor_steps:
                try:
                    self.motor_steps[motor] = int(saved_motor_steps.get(motor, 0))
                except (TypeError, ValueError):
                    self.motor_steps[motor] = 0
        self.action_demo_positions = {
            str(key): int(value)
            for key, value in runtime.get("action_demo_positions", {}).items()
            if isinstance(value, (int, float))
        }
        self.action_demo_clamp_offsets = {
            str(key): int(value)
            for key, value in runtime.get("action_demo_clamp_offsets", {}).items()
            if isinstance(value, (int, float))
        }
        raw_contexts = runtime.get("action_demo_contexts", {})
        self.action_demo_contexts = {
            str(key): dict(value)
            for key, value in raw_contexts.items()
            if isinstance(value, dict)
        } if isinstance(raw_contexts, dict) else {}
        for key in self.action_demo_positions:
            if key in self.action_demo_contexts:
                continue
            action = self.actions.get(key, {})
            if action.get("kind", "stepper") == "stepper" and action.get("group") in GROUPS:
                self.action_demo_contexts[key] = {
                    "group": action["group"],
                    "directions": list(action.get("directions", [])),
                    "clamp_profile_enabled": bool(action.get("clamp_profile_enabled")),
                    "clamp_profile": list(action.get("clamp_profile", [])),
                }
        raw_history = runtime.get("action_demo_history", [])
        self.action_demo_history = (
            [dict(item) for item in raw_history[-100:] if isinstance(item, dict)]
            if isinstance(raw_history, list) else []
        )
        raw_flags = runtime.get("step_flags", [])
        self.step_flags = (
            [dict(item) for item in raw_flags if isinstance(item, dict)]
            if isinstance(raw_flags, list) else []
        )
        try:
            self.counter_epoch = max(1, int(runtime.get("counter_epoch", 1)))
        except (TypeError, ValueError):
            self.counter_epoch = 1
        raw_paused = runtime.get("paused_motion")
        try:
            paused_step_valid = (
                raw_paused.get("group") in GROUPS
                and isinstance(raw_paused.get("directions"), list)
                and 0 <= int(raw_paused.get("completed_steps", -1))
                < int(raw_paused.get("steps", 0))
            ) if isinstance(raw_paused, dict) else False
        except (TypeError, ValueError):
            paused_step_valid = False
        paused_lac_valid = (
            isinstance(raw_paused, dict)
            and raw_paused.get("kind") == "lac"
            and isinstance(raw_paused.get("start_values"), dict)
            and isinstance(raw_paused.get("target_values"), dict)
            and isinstance(raw_paused.get("selected_actuators"), list)
        )
        if isinstance(raw_paused, dict) and (paused_step_valid or paused_lac_valid):
            self.paused_motion = dict(raw_paused)
        self._lock = threading.Lock()
        self._motion_stop = threading.Event()
        self._motion_thread: threading.Thread | None = None
        self._sensor_stop = threading.Event()
        self._sensor_thread: threading.Thread | None = None

    def _runtime_snapshot_locked(self) -> dict[str, Any]:
        return {
            "motor_steps": dict(self.motor_steps),
            "action_demo_positions": dict(self.action_demo_positions),
            "action_demo_clamp_offsets": dict(self.action_demo_clamp_offsets),
            "action_demo_contexts": dict(self.action_demo_contexts),
            "action_demo_history": list(self.action_demo_history),
            "step_flags": list(self.step_flags),
            "counter_epoch": self.counter_epoch,
            "paused_motion": dict(self.paused_motion) if self.paused_motion else None,
        }

    def _persist_runtime_state(self) -> None:
        with self._lock:
            state = self._runtime_snapshot_locked()
        self.session_log.save_runtime(state)

    def configure_serial(self, requested_port: str | None, simulate: bool) -> None:
        self.serial.requested_port = requested_port
        self.serial.simulate = simulate
        self.serial.port = "SIMULATED" if simulate else None
        self.serial.connected = simulate

    def start(self) -> None:
        self.serial.start()
        self._sensor_thread = threading.Thread(
            target=self._sensor_loop, name="sensor-poll", daemon=True
        )
        self._sensor_thread.start()

    def close(self) -> None:
        self._sensor_stop.set()
        self.stop("server shutdown")
        self.serial.close()
        if self._sensor_thread is not None:
            self._sensor_thread.join(timeout=1.0)

    def _sensor_loop(self) -> None:
        lac_counter = 0
        while not self._sensor_stop.wait(1.0):
            if not self.serial.connected:
                continue
            with self._lock:
                moving = self.active_motion is not None
            if moving:
                continue
            try:
                self.serial.send("TOF STATUS")
                lac_counter += 1
                if lac_counter >= 5:
                    lac_counter = 0
                    self.serial.send("LAC STATUS")
            except RuntimeError:
                continue

    def _handle_board_line(self, line: str) -> None:
        match = TOF_RE.search(line)
        if match:
            data = {key: int(value) for key, value in match.groupdict().items()}
            valid = data["status"] == 0 and data["distance"] != 65535
            with self._lock:
                self.tof = {
                    "state": "valid" if valid else "invalid",
                    "valid": valid,
                    "distance_mm": data["distance"] if valid else None,
                    "raw_mm": data["raw"],
                    "status": data["status"],
                    "avg_n": data["avg"],
                    "samples": data["samples"],
                    "errors": data["errors"],
                    "sensor_age_ms": data["age"],
                    "updated_at": time.time(),
                    "pin": "D13/PA5",
                    "physical_side": "right",
                }
            self.session_log.append({"type": "tof", **self.tof})
            return
        if line.startswith("TOF PHYSICAL_RIGHT OFF"):
            with self._lock:
                self.tof.update(
                    {"state": "offline", "valid": False, "distance_mm": None,
                     "updated_at": time.time()}
                )
            return
        if line.startswith("TOF PHYSICAL_RIGHT WAIT"):
            with self._lock:
                self.tof.update(
                    {"state": "waiting", "valid": False, "distance_mm": None,
                     "updated_at": time.time()}
                )
            return
        lac_match = LAC_RE.search(line)
        if lac_match:
            with self._lock:
                self.lac = {
                    "1": int(lac_match.group("left")),
                    "2": int(lac_match.group("right")),
                }

    def set_controls_armed(self, armed: bool) -> None:
        if armed:
            if not self.serial.connected:
                raise RuntimeError("Connect the Nucleo before arming")
            self.serial.send_and_wait("ARM", ("OK: ARMED",))
            self.serial.send_and_wait("LAC STATUS", ("LAC commanded:",))
        else:
            self.stop("UI controls locked")
        self.controls_armed = armed
        self.events.emit(
            "safety",
            "Controls armed; existing LAC targets preserved"
            if armed else "UI controls locked; STOP sent",
        )

    def stop(self, reason: str = "operator") -> None:
        self._motion_stop.set()
        if self.serial.connected:
            try:
                self.serial.send("S")
            except RuntimeError as exc:
                self.events.emit("error", f"STOP failed: {exc}")
        self.events.emit("safety", f"STOP sent ({reason})")

    def read_tof(self) -> None:
        self.serial.send("TOF STATUS")

    def _fresh_tof_snapshot(self, timeout_s: float = 0.35) -> dict[str, Any]:
        """Request the working physical-right sensor and return the new reply if seen."""
        with self._lock:
            previous_update = self.tof.get("updated_at")
        try:
            self.serial.send("TOF STATUS")
        except RuntimeError:
            with self._lock:
                snapshot = dict(self.tof)
            snapshot["fresh"] = False
            return snapshot
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                snapshot = dict(self.tof)
            if snapshot.get("updated_at") != previous_update:
                snapshot["fresh"] = True
                return snapshot
            time.sleep(0.01)
        with self._lock:
            snapshot = dict(self.tof)
        snapshot["fresh"] = False
        return snapshot

    @staticmethod
    def _optional_tof_value(data: dict[str, Any], field: str) -> int | None:
        value = data.get(field)
        if value in (None, ""):
            return None
        try:
            parsed = int(value)
        except (TypeError, ValueError) as exc:
            raise RuntimeError("ToF values must be whole millimetres") from exc
        if not 20 <= parsed <= 4000:
            raise RuntimeError("ToF values must be 20..4000 mm")
        return parsed

    @classmethod
    def _tof_range_from_data(cls, data: dict[str, Any]) -> dict[str, Any]:
        requested_mode = str(data.get("tof_mode", "")).strip().lower()
        minimum = cls._optional_tof_value(data, "tof_min_mm")
        ideal = cls._optional_tof_value(data, "tof_ideal_mm")
        maximum = cls._optional_tof_value(data, "tof_max_mm")
        # Accept the original single reference as an ideal when loading older clients.
        if ideal is None:
            ideal = cls._optional_tof_value(data, "tof_reference_mm")
        legacy_ideal = not requested_mode and ideal is not None
        mode = requested_mode or (
            "range" if minimum is not None or maximum is not None else "unspecified"
        )
        if mode not in {"range", "unspecified", "not_applicable"}:
            raise RuntimeError("Unknown ToF setting")
        if mode != "range":
            return {
                "mode": mode, "min_mm": None,
                "ideal_mm": ideal if legacy_ideal else None, "max_mm": None
            }
        if minimum is None and maximum is None:
            raise RuntimeError("Set the minimum and maximum ToF values")
        if minimum is None or maximum is None:
            raise RuntimeError("Set both the minimum and maximum ToF values")
        if minimum > maximum:
            raise RuntimeError("Minimum ToF cannot be greater than maximum ToF")
        if ideal is None:
            ideal = round((minimum + maximum) / 2)
        if not minimum <= ideal <= maximum:
            raise RuntimeError("Ideal ToF must sit inside the acceptable range")
        return {
            "mode": "range", "min_mm": minimum,
            "ideal_mm": ideal, "max_mm": maximum,
        }

    @staticmethod
    def _classify_tof(
        snapshot: dict[str, Any], target_range: dict[str, Any]
    ) -> dict[str, Any]:
        distance = snapshot.get("distance_mm") if snapshot.get("valid") else None
        mode = target_range.get("mode", "unspecified")
        if mode == "not_applicable":
            reading = "no valid reading" if distance is None else f"reading {distance} mm"
            return {"code": "not_applicable", "label": f"ToF not applicable · {reading}"}
        minimum = target_range.get("min_mm")
        ideal = target_range.get("ideal_mm")
        maximum = target_range.get("max_mm")
        if distance is None:
            return {"code": "unavailable", "label": "No valid ToF target"}
        if mode != "range" or minimum is None or maximum is None:
            return {"code": "recorded", "label": f"ToF unspecified · recorded {distance} mm"}
        if distance < minimum:
            return {
                "code": "below_range",
                "label": f"Below range by {minimum - distance} mm",
            }
        if distance > maximum:
            return {
                "code": "above_range",
                "label": f"Above range by {distance - maximum} mm",
            }
        tolerance = max(2, round((maximum - minimum) * 0.15))
        if ideal is not None and abs(distance - ideal) <= tolerance:
            return {
                "code": "near_ideal",
                "label": f"In range · near ideal ({distance} mm)",
            }
        ideal_suffix = "" if ideal is None else f" · {distance - ideal:+d} mm from ideal"
        return {
            "code": "within_range",
            "label": f"In range ({distance} mm){ideal_suffix}",
        }

    def _send_lac_values(
        self, selected: list[str], values: dict[str, int]
    ) -> None:
        """Apply one LAC signal update and wait for the Nucleo to accept it."""
        if selected == ["1", "2"] and values["1"] == values["2"]:
            commands = [
                (
                    f"LAC BOTH {values['1']}",
                    ("OK: both LAC actuators commanded",),
                    {"1": values["1"], "2": values["2"]},
                )
            ]
        else:
            commands = [
                (
                    f"LAC {actuator} {values[actuator]}",
                    (f"OK: LAC {actuator} commanded",),
                    {actuator: values[actuator]},
                )
                for actuator in selected
            ]
        for command, acknowledgement, update in commands:
            board_ack = self.serial.send_and_wait(
                command, acknowledgement, timeout=1.5
            )
            with self._lock:
                self.lac.update(update)
            self.session_log.append(
                {
                    "type": "lac_signal_update",
                    "command": command,
                    "board_ack": board_ack,
                    "values": update,
                }
            )

    def set_lac(self, target: str, percent: int) -> None:
        """Apply an immediate LAC command; retained for setup and diagnostics."""
        if not self.controls_armed:
            raise RuntimeError("Arm controls before changing a LAC target")
        with self._lock:
            if self.active_motion is not None:
                raise RuntimeError("Stop the active movement before changing a LAC target")
        if not 0 <= percent <= 100:
            raise RuntimeError("LAC percentage must be 0..100")
        selected = ["1", "2"] if target == "both" else [target]
        if any(actuator not in {"1", "2"} for actuator in selected):
            raise RuntimeError("LAC target must be 1, 2, or both")
        values = {actuator: percent for actuator in selected}
        self._send_lac_values(selected, values)
        self.session_log.append(
            {"type": "lac_action", "target": target, "percent": percent,
             "speed_percent_s": None, "state": "immediate"}
        )

    def _run_lac_clamp_support(
        self,
        spec: dict[str, Any],
        result: dict[str, Any],
        timeline_started: float,
    ) -> None:
        """Run M1/M6 clamp pulses while the LAC signal ramp is active."""
        completed = 0
        direction = str(spec.get("support_direction", "R"))
        interrupted = False
        failure: str | None = None
        try:
            for index, segment in enumerate(spec.get("support_plan", [])):
                if self._motion_stop.is_set():
                    interrupted = True
                    break
                scheduled_offset_s = float(segment.get("start_offset_s", 0))
                wait_s = timeline_started + scheduled_offset_s - time.monotonic()
                if wait_s > 0 and self._motion_stop.wait(wait_s):
                    interrupted = True
                    break
                hz = int(segment["hz"])
                planned = int(segment["steps"])
                duration_s = float(segment["duration_s"])
                command = (
                    f"C {hz} 1 {planned} {direction} 6 {planned} {direction}"
                )
                board_ack = self.serial.send_and_wait(
                    command, ("OK: C started",), timeout=1.5
                )
                segment_started = time.monotonic()
                stopped = self._motion_stop.wait(duration_s + 0.01)
                elapsed = min(duration_s, time.monotonic() - segment_started)
                delivered = (
                    planned
                    if not stopped
                    else min(planned, round(planned * elapsed / duration_s))
                )
                completed += delivered
                self.session_log.append(
                    {
                        "type": "lac_support_segment",
                        "motion_id": spec["id"],
                        "index": index,
                        "command": command,
                        "board_ack": board_ack,
                        "hz": hz,
                        "planned_steps": planned,
                        "estimated_delivered_steps": delivered,
                        "planned_duration_s": duration_s,
                        "scheduled_offset_s": scheduled_offset_s,
                        "actual_offset_s": segment_started - timeline_started,
                        "elapsed_s": elapsed,
                        "interrupted": stopped,
                    }
                )
                if stopped:
                    interrupted = True
                    break
        except RuntimeError as exc:
            interrupted = True
            failure = str(exc)
            self._motion_stop.set()
            self.events.emit(
                "error", f"Actuator clamp support {spec['id']} failed: {exc}"
            )
        finally:
            try:
                self.serial.send_and_wait(
                    "S", ("OK: all steppers stopped",), timeout=1.5
                )
            except RuntimeError as exc:
                failure = failure or str(exc)
            sign = 1 if direction == "F" else -1
            with self._lock:
                self.motor_steps["1"] += sign * completed
                self.motor_steps["6"] += sign * completed
            result.update(
                {
                    "completed_steps": completed,
                    "direction": direction,
                    "state": "stopped" if interrupted else "complete",
                    "error": failure,
                }
            )
            self.session_log.append(
                {
                    "type": "lac_support_finished",
                    "motion_id": spec["id"],
                    **result,
                }
            )

    def start_lac_motion(
        self,
        target: str,
        percent: int,
        speed_percent_s: int,
        name: str = "Direct linear actuator positioning",
        *,
        target_values: dict[str, int] | None = None,
        demo_context: dict[str, Any] | None = None,
        recovery_mode: str | None = None,
        support_clamp_steps: int = 0,
        support_max_hz: int = DEFAULT_LAC_SUPPORT_HZ,
        support_ramp_percent: int = DEFAULT_LAC_SUPPORT_RAMP_PERCENT,
        support_direction: str = "R",
    ) -> dict[str, Any]:
        if not self.controls_armed:
            raise RuntimeError("Arm controls before moving a linear actuator")
        if not self.serial.connected:
            raise RuntimeError("Nucleo is not connected")
        if not MIN_LAC_SPEED_PERCENT_S <= speed_percent_s <= MAX_LAC_SPEED_PERCENT_S:
            raise RuntimeError(
                f"Actuator signal speed must be {MIN_LAC_SPEED_PERCENT_S}.."
                f"{MAX_LAC_SPEED_PERCENT_S}% per second"
            )
        selected = ["1", "2"] if target == "both" else [target]
        if any(actuator not in {"1", "2"} for actuator in selected):
            raise RuntimeError("LAC target must be 1, 2, or both")
        if not 0 <= percent <= 100:
            raise RuntimeError("LAC percentage must be 0..100")
        support_clamp_steps = int(support_clamp_steps)
        if not 0 <= support_clamp_steps <= MAX_STEPS:
            raise RuntimeError(f"Actuator clamp steps must be 0..{MAX_STEPS}")
        support_direction = str(support_direction).upper()
        if support_direction not in {"F", "R"}:
            raise RuntimeError("Actuator clamp direction must be F or R")
        # Clamp support is a monotonic position accumulation across the full
        # action, not a symmetric motor-speed ramp. Retain this field in saved
        # data for compatibility, but its only valid meaning is 100% coverage.
        support_ramp_percent = 100
        with self._lock:
            if self.active_motion is not None:
                raise RuntimeError("A movement is already running; stop it first")
            start_values = dict(self.lac)
        desired = {
            actuator: int(
                target_values.get(actuator, percent)
                if target_values is not None else percent
            )
            for actuator in selected
        }
        if any(not 0 <= value <= 100 for value in desired.values()):
            raise RuntimeError("LAC percentage must be 0..100")
        max_change = max(
            abs(desired[actuator] - start_values[actuator])
            for actuator in selected
        )
        lac_duration_s = max_change / speed_percent_s
        if support_clamp_steps:
            support_plan, support_duration_s = build_full_duration_support_plan(
                support_clamp_steps,
                int(support_max_hz),
                lac_duration_s,
            )
            duration_s = support_duration_s
        else:
            support_plan = []
            support_duration_s = 0.0
            duration_s = lac_duration_s
        spec: dict[str, Any] = {
            "id": uuid.uuid4().hex[:10],
            "name": str(name)[:100],
            "kind": "lac",
            "lac_target": target,
            "selected_actuators": selected,
            "start_values": start_values,
            "target_values": desired,
            "speed_percent_s": speed_percent_s,
            "lac_duration_s": lac_duration_s,
            "support_clamp_steps": support_clamp_steps,
            "support_max_hz": int(support_max_hz),
            "support_ramp_percent": int(support_ramp_percent),
            "support_direction": support_direction,
            "support_plan": support_plan,
            "support_duration_s": support_duration_s,
            "estimated_duration_s": duration_s,
            "started_at": time.time(),
            "state": "running",
        }
        if demo_context:
            spec.update(demo_context)
        if recovery_mode:
            spec["recovery_mode"] = recovery_mode
        self._motion_stop.clear()
        with self._lock:
            self.paused_motion = None
            self.active_motion = dict(spec)
        self._persist_runtime_state()
        self.session_log.append({"type": "lac_motion_started", **spec})
        self.events.emit(
            "motion",
            f"Started {spec['name']}: {speed_percent_s}%/s signal ramp to "
            + ", ".join(f"LAC{item} {desired[item]}%" for item in selected)
            + (
                f" with {support_clamp_steps} clamp steps spread across 100% "
                "of the action"
                if support_clamp_steps else ""
            ),
            motion=spec,
        )
        self._motion_thread = threading.Thread(
            target=self._run_lac_motion,
            args=(spec,),
            name="lac-signal-ramp",
            daemon=True,
        )
        self._motion_thread.start()
        return spec

    def _run_lac_motion(self, spec: dict[str, Any]) -> None:
        selected = list(spec["selected_actuators"])
        starts = dict(spec["start_values"])
        targets = dict(spec["target_values"])
        speed = int(spec["speed_percent_s"])
        ramp_started = time.monotonic()
        last_sent = {actuator: starts[actuator] for actuator in selected}
        interrupted = False
        failure: str | None = None
        support_result: dict[str, Any] = {
            "completed_steps": 0,
            "direction": str(spec.get("support_direction", "R")),
            "state": "not_requested",
            "error": None,
        }
        support_thread: threading.Thread | None = None
        if spec.get("support_plan"):
            support_thread = threading.Thread(
                target=self._run_lac_clamp_support,
                args=(spec, support_result, ramp_started),
                name="lac-clamp-support",
                daemon=True,
            )
            support_thread.start()
        try:
            while True:
                if self._motion_stop.is_set():
                    interrupted = True
                    break
                elapsed_s = time.monotonic() - ramp_started
                timeline_duration_s = float(
                    spec.get("estimated_duration_s", spec["lac_duration_s"])
                )
                progress = (
                    1.0
                    if timeline_duration_s <= 0
                    else min(1.0, elapsed_s / timeline_duration_s)
                )
                next_values: dict[str, int] = {}
                complete = True
                for actuator in selected:
                    start = starts[actuator]
                    target = targets[actuator]
                    delta = target - start
                    value = round(start + delta * progress)
                    next_values[actuator] = value
                    complete = complete and value == target
                if next_values != last_sent:
                    self._send_lac_values(selected, next_values)
                    last_sent = next_values
                if complete:
                    break
                if self._motion_stop.wait(LAC_UPDATE_PERIOD_S):
                    interrupted = True
                    break
        except RuntimeError as exc:
            interrupted = True
            failure = str(exc)
            self._motion_stop.set()
            self.events.emit("error", f"Actuator ramp {spec['id']} failed: {exc}")
        finally:
            if support_thread is not None:
                support_thread.join(
                    timeout=max(2.0, float(spec.get("support_duration_s", 0)) + 2.0)
                )
                if support_thread.is_alive():
                    interrupted = True
                    failure = failure or "Clamp support did not stop in time"
                    self._motion_stop.set()
            if support_result.get("error"):
                interrupted = True
                failure = failure or str(support_result["error"])
            with self._lock:
                current = dict(self.lac)
                finished = dict(spec)
                finished.update(
                    {
                        "actual_lac": current,
                        "support_completed_steps": int(
                            support_result.get("completed_steps", 0)
                        ),
                        "support_state": support_result.get("state"),
                        "finished_at": time.time(),
                        "state": "stopped" if interrupted else "complete",
                    }
                )
                if failure:
                    finished["error"] = failure
                self.last_motion = finished
                self.active_motion = None
                incomplete = any(
                    current[actuator] != targets[actuator]
                    for actuator in selected
                )
                support_incomplete = int(
                    support_result.get("completed_steps", 0)
                ) < int(spec.get("support_clamp_steps", 0))
                if (
                    interrupted
                    and self._motion_stop.is_set()
                    and (incomplete or support_incomplete)
                ):
                    self.paused_motion = {
                        "original_motion_id": spec["id"],
                        "name": spec["name"],
                        "kind": "lac",
                        "lac_target": spec["lac_target"],
                        "selected_actuators": selected,
                        "start_values": starts,
                        "current_values": current,
                        "target_values": targets,
                        "speed_percent_s": speed,
                        "support_clamp_steps": int(
                            spec.get("support_clamp_steps", 0)
                        ),
                        "support_completed_steps": int(
                            support_result.get("completed_steps", 0)
                        ),
                        "support_max_hz": int(
                            spec.get("support_max_hz", DEFAULT_LAC_SUPPORT_HZ)
                        ),
                        "support_ramp_percent": int(
                            spec.get(
                                "support_ramp_percent",
                                DEFAULT_LAC_SUPPORT_RAMP_PERCENT,
                            )
                        ),
                        "support_direction": str(
                            spec.get("support_direction", "R")
                        ),
                        "stopped_at": finished["finished_at"],
                        **{
                            field: spec[field]
                            for field in (
                                "demo_key", "demo_tof_before", "demo_tof_range"
                            ) if field in spec
                        },
                    }
                else:
                    self.paused_motion = None
            self._persist_runtime_state()
            after_tof = self._fresh_tof_snapshot() if spec.get("demo_key") else None
            if spec.get("demo_key"):
                tof_result = self._classify_tof(
                    after_tof or {}, spec.get("demo_tof_range", {})
                )
                demo_record = {
                    "action_id": str(spec["demo_key"]),
                    "action_name": spec["name"],
                    "kind": "lac",
                    "from_lac": starts,
                    "target_percent": list(targets.values())[0],
                    "speed_percent_s": speed,
                    "support_clamp_steps": int(
                        spec.get("support_clamp_steps", 0)
                    ),
                    "support_completed_steps": int(
                        support_result.get("completed_steps", 0)
                    ),
                    "support_direction": str(
                        support_result.get("direction", "R")
                    ),
                    "actual_lac": current,
                    "tof_before": spec.get("demo_tof_before"),
                    "tof_after": after_tof,
                    "tof_range": spec.get("demo_tof_range", {}),
                    "tof_result": tof_result,
                    "finished_at": finished["finished_at"],
                    "state": finished["state"],
                }
                with self._lock:
                    self.action_demo_history.append(demo_record)
                    self.action_demo_history = self.action_demo_history[-100:]
                self._persist_runtime_state()
                self.session_log.append({"type": "action_demo_result", **demo_record})
            self.session_log.append({"type": "lac_motion_finished", **finished})
            self.events.emit(
                "motion",
                f"{finished['name']} {finished['state']} at "
                + ", ".join(f"LAC{item} {current[item]}%" for item in selected)
                + (
                    f"; clamp {support_result.get('direction', 'R')} "
                    f"{int(support_result.get('completed_steps', 0))}/"
                    f"{int(spec.get('support_clamp_steps', 0))} steps"
                    if spec.get("support_clamp_steps") else ""
                )
                + "; signal ramp stopped at its current command",
                motion=finished,
            )

    @staticmethod
    def _motion_spec(data: dict[str, Any], reverse: bool = False) -> dict[str, Any]:
        group = str(data.get("group", "mechanism"))
        if group not in GROUPS:
            raise RuntimeError("Unknown motor group")
        motors = GROUPS[group]["motors"]
        directions = [str(data.get("direction_a", "F")).upper()]
        if len(motors) > 1:
            directions.append(str(data.get("direction_b", "F")).upper())
        if any(direction not in {"F", "R"} for direction in directions):
            raise RuntimeError("Directions must be F or R")
        if reverse:
            directions = ["R" if direction == "F" else "F" for direction in directions]
        steps = int(data.get("steps", 500))
        max_hz = int(data.get("max_hz", 500))
        ramp_percent = int(data.get("ramp_percent", 0))
        plan = build_ramp_plan(steps, max_hz, ramp_percent)
        motor_step_offsets = normalize_motor_step_offsets(
            data.get("motor_step_offsets"), motors, group
        )
        if any(motor_step_offsets.values()):
            plan = apply_motor_step_offsets(plan, motors, motor_step_offsets)
        motor_step_totals = {
            str(motor): steps + motor_step_offsets.get(str(motor), 0)
            for motor in motors
        }
        profile_enabled = bool(data.get("clamp_profile_enabled", False))
        clamp_profile: list[dict[str, int]] = []
        if profile_enabled:
            if group != "belts":
                raise RuntimeError(
                    "A clamp profile can only be attached to a conveyor action"
                )
            clamp_profile = normalize_clamp_profile(
                data.get("clamp_profile"), steps
            )
            if reverse and not bool(data.get("clamp_profile_is_traversal", False)):
                clamp_profile = reverse_clamp_profile(clamp_profile, steps)
            plan = build_composite_plan(plan, directions, clamp_profile)
        duration_s = sum(segment["duration_s"] for segment in plan)
        return {
            "group": group,
            "group_label": GROUPS[group]["label"],
            "motors": list(motors),
            "directions": directions,
            "steps": steps,
            "motor_step_offsets": motor_step_offsets,
            "motor_step_totals": motor_step_totals,
            "max_hz": max_hz,
            "ramp_percent": ramp_percent,
            "clamp_profile_enabled": profile_enabled,
            "clamp_profile": clamp_profile,
            "instructions": str(data.get("instructions", ""))[:1000],
            "plan": plan,
            "estimated_duration_s": duration_s,
            "reverse": reverse,
        }

    def start_motion(
        self,
        data: dict[str, Any],
        reverse: bool = False,
        demo_context: dict[str, Any] | None = None,
        recovery_mode: str | None = None,
    ) -> dict[str, Any]:
        if not self.controls_armed:
            raise RuntimeError("Arm controls before running motion")
        if not self.serial.connected:
            raise RuntimeError("Nucleo is not connected")
        with self._lock:
            if self.active_motion is not None:
                raise RuntimeError("A motion is already running; stop it first")
        spec = self._motion_spec(data, reverse)
        spec.update(
            {
                "id": uuid.uuid4().hex[:10],
                "name": str(data.get("name", "Unsaved motion"))[:100],
                "started_at": time.time(),
                "completed_steps": 0,
                "state": "running",
            }
        )
        if demo_context:
            spec.update(demo_context)
        if recovery_mode:
            spec["recovery_mode"] = recovery_mode
        self._motion_stop.clear()
        with self._lock:
            # Starting any new movement supersedes the previous recovery choice.
            # If this movement is stopped, _run_motion installs a fresh one.
            self.paused_motion = None
            self.active_motion = dict(spec)
        self._persist_runtime_state()
        self.session_log.append({"type": "motion_started", **spec})
        motor_description = " + ".join(
            f"M{motor} {direction}"
            for motor, direction in zip(spec["motors"], spec["directions"])
        )
        self.events.emit(
            "motion",
            f"Started {spec['name']}: {motor_description}, {spec['steps']} steps, "
            f"{spec['max_hz']} Hz, ramp {spec['ramp_percent']}%",
            motion=spec,
        )
        self._motion_thread = threading.Thread(
            target=self._run_motion, args=(spec,), name="paired-motion", daemon=True
        )
        self._motion_thread.start()
        return spec

    def preview_motion(self, data: dict[str, Any]) -> dict[str, Any]:
        """Return the exact ramp-aware estimate without arming or moving."""
        spec = self._motion_spec(data)
        return {
            "steps": spec["steps"],
            "motor_step_offsets": spec["motor_step_offsets"],
            "motor_step_totals": spec["motor_step_totals"],
            "max_hz": spec["max_hz"],
            "ramp_percent": spec["ramp_percent"],
            "clamp_profile_enabled": spec["clamp_profile_enabled"],
            "estimated_duration_s": spec["estimated_duration_s"],
            "segment_count": len(spec["plan"]),
        }

    def _run_motion(self, spec: dict[str, Any]) -> None:
        primary_motors = spec["motors"]
        primary_directions = spec["directions"]
        completed_steps = 0
        signed_steps_by_motor = {
            str(motor): 0 for motor in set((1, 3, 4, 6)) | set(primary_motors)
        }
        clamp_offset_delivered = 0
        used_motors = set(primary_motors)
        interrupted = False
        try:
            self.serial.send_and_wait(
                "TOF STATUS", ("TOF PHYSICAL_RIGHT",), timeout=2.0
            )
            self.serial.send_and_wait(
                "LAC STATUS", ("LAC commanded:",), timeout=1.5
            )
            for index, segment in enumerate(spec["plan"]):
                if self._motion_stop.is_set():
                    interrupted = True
                    break
                hz = int(segment["hz"])
                planned_steps = int(segment["steps"])
                duration_s = float(segment["duration_s"])
                motors = list(segment.get("motors", primary_motors))
                directions = list(segment.get("directions", primary_directions))
                motor_steps = list(
                    segment.get("motor_steps", [planned_steps] * len(motors))
                )
                used_motors.update(motors)
                # Every paired action uses the board's exact, self-stopping
                # scheduler. Single-motor direct positioning still uses its
                # continuous command because firmware C requires 2+ motors.
                coordinated = bool(
                    segment.get("coordinated", len(motors) > 1)
                )
                if coordinated:
                    command = "C " + str(hz) + " " + " ".join(
                        f"{motor} {steps_for_motor} {direction}"
                        for motor, steps_for_motor, direction in zip(
                            motors, motor_steps, directions
                        )
                    )
                elif len(motors) == 1:
                    command = f"M {motors[0]} {hz} {directions[0]}"
                else:
                    command = "D " + " ".join(
                        f"{motor} {hz} {direction}"
                        for motor, direction in zip(motors, directions)
                    )
                if coordinated:
                    acknowledgement = ("OK: C started",)
                elif len(motors) == 1:
                    acknowledgement = (f"OK: M{motors[0]} continuous",)
                else:
                    acknowledgement = ("OK: D started",)
                board_ack = self.serial.send_and_wait(
                    command, acknowledgement, timeout=1.5
                )
                segment_started = time.monotonic()
                # Coordinated commands stop themselves after exact pulse counts.
                # A short allowance covers serial parsing before the board starts.
                stopped = self._motion_stop.wait(
                    duration_s + (0.03 if coordinated else 0.0)
                )
                elapsed = min(duration_s, time.monotonic() - segment_started)
                fraction = 1.0 if not stopped else min(1.0, elapsed / duration_s)
                delivered_by_motor = [
                    planned if not stopped else min(planned, round(planned * fraction))
                    for planned in motor_steps
                ]
                delivered = max(delivered_by_motor, default=0)
                primary_planned = int(segment.get("primary_steps", planned_steps))
                delivered_primary = min(
                    primary_planned, round(primary_planned * fraction)
                )
                completed_steps += delivered_primary
                for motor, direction, motor_delivered in zip(
                    motors, directions, delivered_by_motor
                ):
                    sign = 1 if direction == "F" else -1
                    signed_steps_by_motor[str(motor)] += sign * motor_delivered
                clamp_sign = int(segment.get("clamp_sign", 0))
                clamp_planned = int(segment.get("clamp_steps", planned_steps))
                clamp_delivered = min(
                    clamp_planned, round(clamp_planned * fraction)
                )
                clamp_offset_delivered += clamp_sign * clamp_delivered
                self.session_log.append(
                    {
                        "type": "motion_segment",
                        "motion_id": spec["id"],
                        "index": index,
                        "command": command,
                        "board_ack": board_ack,
                        "hz": hz,
                        "planned_steps": planned_steps,
                        "estimated_delivered_steps": delivered,
                        "planned_steps_by_motor": dict(zip(motors, motor_steps)),
                        "estimated_delivered_steps_by_motor": dict(
                            zip(motors, delivered_by_motor)
                        ),
                        "primary_planned_steps": primary_planned,
                        "estimated_primary_delivered_steps": delivered_primary,
                        "motors": motors,
                        "directions": directions,
                        "clamp_sign": clamp_sign,
                        "coordinated": coordinated,
                        "planned_duration_s": duration_s,
                        "elapsed_s": elapsed,
                        "interrupted": stopped,
                    }
                )
                if stopped:
                    interrupted = True
                    break
        except RuntimeError as exc:
            interrupted = True
            self.events.emit("error", f"Motion {spec['id']} failed: {exc}")
        finally:
            try:
                self.serial.send_and_wait(
                    "S", ("OK: all steppers stopped",), timeout=1.5
                )
            except RuntimeError:
                pass
            demo_after_tof = (
                self._fresh_tof_snapshot() if spec.get("demo_key") else None
            )
            with self._lock:
                for motor, signed_steps in signed_steps_by_motor.items():
                    self.motor_steps[motor] += signed_steps
                if spec.get("demo_key"):
                    demo_position = int(spec["demo_start_steps"]) + (
                        int(spec["demo_delta_sign"]) * completed_steps
                    )
                    demo_key = str(spec["demo_key"])
                    demo_clamp_offset = int(
                        spec.get("demo_start_clamp_offset", 0)
                    ) + clamp_offset_delivered
                    self.action_demo_positions[demo_key] = demo_position
                    self.action_demo_clamp_offsets[demo_key] = demo_clamp_offset
                    if demo_position == 0 and demo_clamp_offset == 0:
                        self.action_demo_contexts.pop(demo_key, None)
                else:
                    demo_position = None
                    demo_clamp_offset = None
                finished = dict(spec)
                # These are open-loop requested pulses. The current robot has
                # no stepper encoder, so never describe them as confirmed
                # physical travel.
                finished["completed_steps"] = completed_steps
                finished["commanded_steps"] = completed_steps
                finished["commanded_steps_by_motor"] = signed_steps_by_motor
                finished["clamp_offset_delta_steps"] = clamp_offset_delivered
                finished["physical_motion_confirmed"] = False
                finished["finished_at"] = time.time()
                finished["state"] = "stopped" if interrupted else "complete"
                if demo_position is not None:
                    finished["demo_position_steps"] = demo_position
                    finished["demo_clamp_offset_steps"] = demo_clamp_offset
                    tof_result = self._classify_tof(
                        demo_after_tof or {}, spec.get("demo_tof_range", {})
                    )
                    finished["demo_tof_before"] = spec.get("demo_tof_before")
                    finished["demo_tof_after"] = demo_after_tof
                    finished["demo_tof_result"] = tof_result
                    demo_record = {
                        "action_id": str(spec["demo_key"]),
                        "action_name": spec["name"],
                        "from_steps": int(spec["demo_start_steps"]),
                        "requested_target_steps": int(
                            spec["demo_requested_target_steps"]
                        ),
                        "actual_position_steps": demo_position,
                        "commanded_delta_steps": int(spec["demo_delta_sign"])
                        * int(spec["steps"]),
                        "delivered_delta_steps": int(spec["demo_delta_sign"])
                        * completed_steps,
                        "from_clamp_offset_steps": int(
                            spec.get("demo_start_clamp_offset", 0)
                        ),
                        "delivered_clamp_delta_steps": clamp_offset_delivered,
                        "actual_clamp_offset_steps": demo_clamp_offset,
                        "tof_before": spec.get("demo_tof_before"),
                        "tof_after": demo_after_tof,
                        "tof_range": spec.get("demo_tof_range", {}),
                        "tof_result": tof_result,
                        "finished_at": time.time(),
                        "state": "stopped" if interrupted else "complete",
                    }
                    self.action_demo_history.append(demo_record)
                    self.action_demo_history = self.action_demo_history[-100:]
                self.last_motion = finished
                self.active_motion = None
                if (
                    interrupted
                    and self._motion_stop.is_set()
                    and completed_steps < int(spec["steps"])
                ):
                    paused_fields = (
                        "group", "group_label", "motors", "directions", "steps",
                        "motor_step_offsets", "motor_step_totals", "max_hz",
                        "ramp_percent", "clamp_profile_enabled",
                        "clamp_profile", "instructions", "demo_key",
                        "demo_start_steps", "demo_requested_target_steps",
                        "demo_delta_sign", "demo_start_clamp_offset",
                        "demo_tof_range",
                    )
                    paused = {
                        field: finished[field]
                        for field in paused_fields if field in finished
                    }
                    paused.update(
                        {
                            "original_motion_id": finished["id"],
                            "name": finished["name"],
                            "completed_steps": completed_steps,
                            "remaining_steps": max(
                                0, int(spec["steps"]) - completed_steps
                            ),
                            "clamp_offset_delta_steps": clamp_offset_delivered,
                            "commanded_steps_by_motor": dict(
                                finished.get("commanded_steps_by_motor", {})
                            ),
                            "stopped_at": finished["finished_at"],
                        }
                    )
                    if demo_position is not None:
                        paused["demo_position_steps"] = demo_position
                        paused["demo_clamp_offset_steps"] = demo_clamp_offset
                    self.paused_motion = paused
                else:
                    self.paused_motion = None
            self._persist_runtime_state()
            self.session_log.append({"type": "motion_finished", **finished})
            if demo_position is not None:
                self.session_log.append({"type": "action_demo_result", **demo_record})
            self.events.emit(
                "motion",
                f"{finished['name']} {finished['state']}: requested "
                f"{completed_steps}/{spec['steps']} open-loop step pulses; "
                + (
                    f"clamp offset {clamp_offset_delivered:+d} steps; "
                    if spec.get("clamp_profile_enabled") else ""
                )
                + "physical travel is not sensed; motors stopped",
                motion=finished,
            )
            if demo_position is not None:
                self.events.emit(
                    "tof",
                    f"Demo finish ToF: {tof_result['label']}",
                    action_id=str(spec["demo_key"]),
                    result=demo_record,
                )
            try:
                self.serial.send("LAC STATUS")
            except RuntimeError:
                pass

    def recover_paused_motion(self, mode: str) -> dict[str, Any]:
        if mode not in {"resume", "return"}:
            raise RuntimeError("Recovery mode must be resume or return")
        with self._lock:
            if self.active_motion is not None:
                raise RuntimeError("Stop the active movement before recovering it")
            paused = dict(self.paused_motion or {})
        if not paused:
            raise RuntimeError("There is no stopped movement to recover")

        if paused.get("kind") == "lac":
            selected = list(paused["selected_actuators"])
            desired = dict(
                paused["target_values"] if mode == "resume"
                else paused["start_values"]
            )
            support_total = int(paused.get("support_clamp_steps", 0))
            support_completed = int(paused.get("support_completed_steps", 0))
            support_steps = (
                max(0, support_total - support_completed)
                if mode == "resume" else support_completed
            )
            original_support_direction = str(
                paused.get("support_direction", "R")
            )
            support_direction = (
                original_support_direction
                if mode == "resume"
                else ("R" if original_support_direction == "F" else "F")
            )
            target_name = "both" if selected == ["1", "2"] else selected[0]
            demo_context = None
            if paused.get("demo_key"):
                demo_context = {
                    "demo_key": str(paused["demo_key"]),
                    "demo_tof_before": self._fresh_tof_snapshot(),
                    "demo_tof_range": dict(paused.get("demo_tof_range", {})),
                }
            self.session_log.append(
                {
                    "type": "motion_recovery_requested",
                    "mode": mode,
                    "paused_motion": paused,
                    "target_values": desired,
                }
            )
            return self.start_lac_motion(
                target_name,
                int(desired[selected[0]]),
                int(paused["speed_percent_s"]),
                (
                    f"{paused['name']} (resume)" if mode == "resume"
                    else f"{paused['name']} (return to start)"
                ),
                target_values=desired,
                demo_context=demo_context,
                recovery_mode=mode,
                support_clamp_steps=support_steps,
                support_max_hz=int(
                    paused.get("support_max_hz", DEFAULT_LAC_SUPPORT_HZ)
                ),
                support_ramp_percent=int(
                    paused.get(
                        "support_ramp_percent",
                        DEFAULT_LAC_SUPPORT_RAMP_PERCENT,
                    )
                ),
                support_direction=support_direction,
            )

        completed = int(paused["completed_steps"])
        total = int(paused["steps"])
        target = total if mode == "resume" else 0
        recovery_steps = (total - completed) if mode == "resume" else completed
        if recovery_steps <= 0:
            raise RuntimeError(
                "That movement is already complete" if mode == "resume"
                else "That movement did not leave its starting position"
            )

        directions = list(paused["directions"])
        motion_data: dict[str, Any] = {
            "name": (
                f"{paused['name']} (resume)" if mode == "resume"
                else f"{paused['name']} (return to start)"
            ),
            "group": paused["group"],
            "direction_a": directions[0],
            "steps": recovery_steps,
            "max_hz": int(paused["max_hz"]),
            "ramp_percent": int(paused["ramp_percent"]),
            "instructions": str(paused.get("instructions", "")),
        }
        if len(directions) > 1:
            motion_data["direction_b"] = directions[1]

        original_offsets = dict(paused.get("motor_step_offsets", {}))
        if any(int(value) for value in original_offsets.values()):
            delivered_by_motor = dict(paused.get("commanded_steps_by_motor", {}))
            recovery_offsets: dict[str, int] = {}
            for motor in paused.get("motors", []):
                motor_key = str(motor)
                delivered = abs(int(delivered_by_motor.get(motor_key, 0)))
                if mode == "resume":
                    target_steps = total + int(original_offsets.get(motor_key, 0))
                    motor_recovery = max(0, target_steps - delivered)
                else:
                    motor_recovery = delivered
                recovery_offsets[motor_key] = max(
                    0, motor_recovery - recovery_steps
                )
            motion_data["motor_step_offsets"] = recovery_offsets

        if paused.get("clamp_profile_enabled"):
            full_profile = normalize_clamp_profile(
                paused.get("clamp_profile"), total
            )
            motion_data.update(
                {
                    "clamp_profile_enabled": True,
                    "clamp_profile": slice_clamp_profile(
                        full_profile,
                        completed,
                        target,
                        int(paused.get("clamp_offset_delta_steps", 0)),
                    ),
                    "clamp_profile_is_traversal": True,
                }
            )

        demo_context = None
        if paused.get("demo_key"):
            current_demo = int(paused.get("demo_position_steps", 0))
            demo_target = (
                int(paused["demo_requested_target_steps"])
                if mode == "resume" else int(paused["demo_start_steps"])
            )
            demo_context = {
                "demo_key": str(paused["demo_key"]),
                "demo_start_steps": current_demo,
                "demo_requested_target_steps": demo_target,
                "demo_delta_sign": 1 if demo_target > current_demo else -1,
                "demo_start_clamp_offset": int(
                    paused.get("demo_clamp_offset_steps", 0)
                ),
                "demo_tof_before": self._fresh_tof_snapshot(),
                "demo_tof_range": dict(paused.get("demo_tof_range", {})),
            }

        self.session_log.append(
            {
                "type": "motion_recovery_requested",
                "mode": mode,
                "paused_motion": paused,
                "recovery_steps": recovery_steps,
            }
        )
        self.events.emit(
            "motion",
            (
                f"Resuming {paused['name']} for the remaining {recovery_steps} steps"
                if mode == "resume"
                else f"Returning {paused['name']} {recovery_steps} steps to its start"
            ),
            paused_motion=paused,
        )
        return self.start_motion(
            motion_data,
            reverse=mode == "return",
            demo_context=demo_context,
            recovery_mode=mode,
        )

    def save_action(self, data: dict[str, Any]) -> dict[str, Any]:
        name = str(data.get("name", "")).strip()[:100]
        if not name:
            raise RuntimeError("Action name is required")
        requested_key = re.sub(
            r"[^a-z0-9-]+", "", str(data.get("action_id", "")).lower()
        ).strip("-")
        key = requested_key or (
            re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-")
            or uuid.uuid4().hex[:8]
        )
        tof_range = self._tof_range_from_data(data)
        with self._lock:
            previous = dict(self.actions.get(key, {}))
            next_order = max(
                (int(action.get("order", 0)) for action in self.actions.values()),
                default=0,
            ) + 10
        built_in_definition = DEFAULT_ACTION_BY_ID.get(key)
        kind = str(
            built_in_definition.get("kind")
            if built_in_definition
            else data.get("kind") or previous.get("kind") or "stepper"
        )
        common = {
            "id": key,
            "name": name,
            "kind": kind,
            "created_at": previous.get("created_at", utc_timestamp()),
            "updated_at": utc_timestamp(),
            "order": int(previous.get("order", next_order)),
            "built_in": bool(previous.get("built_in", False)),
            "configured": True,
            "pair_id": ACTION_PAIRS.get(key, previous.get("pair_id")),
            "tof_mode": tof_range["mode"],
            "tof_min_mm": tof_range["min_mm"],
            "tof_ideal_mm": tof_range["ideal_mm"],
            "tof_max_mm": tof_range["max_mm"],
            "tof_reference_mm": tof_range["ideal_mm"],
            "instructions": str(data.get("instructions", ""))[:1000],
        }
        if kind == "lac":
            lac_percent = int(data.get("lac_percent", 0))
            lac_speed = int(data.get("lac_speed_percent_s", 25))
            support_steps = int(
                data.get(
                    "support_clamp_steps",
                    previous.get(
                        "support_clamp_steps",
                        DEFAULT_LAC_SUPPORT_STEPS,
                    ),
                )
            )
            support_hz = int(
                data.get(
                    "support_max_hz",
                    previous.get("support_max_hz", DEFAULT_LAC_SUPPORT_HZ),
                )
            )
            support_ramp = 100
            if not 0 <= lac_percent <= 100:
                raise RuntimeError("Linear actuator target must be 0..100%")
            if not MIN_LAC_SPEED_PERCENT_S <= lac_speed <= MAX_LAC_SPEED_PERCENT_S:
                raise RuntimeError(
                    f"Actuator signal speed must be {MIN_LAC_SPEED_PERCENT_S}.."
                    f"{MAX_LAC_SPEED_PERCENT_S}% per second"
                )
            if not 0 <= support_steps <= MAX_STEPS:
                raise RuntimeError(
                    f"Actuator clamp steps must be 0..{MAX_STEPS}"
                )
            support_plan = (
                build_full_duration_support_plan(support_steps, support_hz, 0)[0]
                if support_steps else []
            )
            record = {
                **common,
                "group": "lac",
                "lac_target": "both",
                "lac_percent": lac_percent,
                "lac_speed_percent_s": lac_speed,
                "support_clamp_steps": support_steps,
                "support_max_hz": support_hz,
                "support_ramp_percent": support_ramp,
                "support_estimated_duration_s": sum(
                    segment["duration_s"] for segment in support_plan
                ),
                "motors": [],
                "directions": [],
                "estimated_duration_s": None,
            }
        elif kind == "stepper":
            motion_data = dict(data)
            if built_in_definition:
                fixed_directions = list(built_in_definition["directions"])
                motion_data.update(
                    {
                        "group": built_in_definition["group"],
                        "direction_a": fixed_directions[0],
                        "direction_b": fixed_directions[1],
                    }
                )
                if (
                    key in SHIFT_MOTOR_STEP_OFFSETS
                    and "motor_step_offsets" not in motion_data
                ):
                    motion_data["motor_step_offsets"] = dict(
                        previous.get(
                            "motor_step_offsets", SHIFT_MOTOR_STEP_OFFSETS[key]
                        )
                    )
            spec = self._motion_spec(motion_data)
            record = {
                **common,
                **{field: spec[field] for field in (
                    "group", "motors", "directions", "steps", "max_hz",
                    "ramp_percent", "motor_step_offsets", "motor_step_totals",
                    "estimated_duration_s"
                )},
            }
            if spec["group"] == "belts":
                record.update(
                    {
                        "clamp_profile_enabled": spec["clamp_profile_enabled"],
                        "clamp_profile": spec["clamp_profile"] if spec["clamp_profile_enabled"] else [
                            {"at_step": 0, "offset_steps": 0},
                            {"at_step": spec["steps"], "offset_steps": 0},
                        ],
                    }
                )
        else:
            raise RuntimeError("Unknown action type")
        paired_record = None
        sync_clamp_profile = bool(data.get("sync_clamp_profile_to_pair", False))
        with self._lock:
            self.actions[key] = record
            pair_key = ACTION_PAIRS.get(key) if kind == "stepper" else None
            if pair_key:
                paired_previous = dict(self.actions.get(pair_key, {}))
                if paired_previous:
                    paired_record = {
                        **paired_previous,
                        "updated_at": utc_timestamp(),
                        "configured": True,
                        "pair_id": key,
                        "group": record["group"],
                        "motors": list(record["motors"]),
                        "directions": [
                            "R" if direction == "F" else "F"
                            for direction in record["directions"]
                        ],
                        "steps": record["steps"],
                        "max_hz": record["max_hz"],
                        "ramp_percent": record["ramp_percent"],
                        # Alignment corrections belong to the physical action,
                        # not the abstract reverse pair. Preserve the paired
                        # action's independently editable M1/M6 values.
                        "motor_step_offsets": dict(
                            paired_previous.get(
                                "motor_step_offsets",
                                SHIFT_MOTOR_STEP_OFFSETS.get(pair_key, {}),
                            )
                        ),
                        "estimated_duration_s": record["estimated_duration_s"],
                    }
                    if record["group"] == "mechanism":
                        paired_spec = self._motion_spec(
                            {
                                "group": paired_record["group"],
                                "direction_a": paired_record["directions"][0],
                                "direction_b": paired_record["directions"][1],
                                "steps": paired_record["steps"],
                                "max_hz": paired_record["max_hz"],
                                "ramp_percent": paired_record["ramp_percent"],
                                "motor_step_offsets": paired_record[
                                    "motor_step_offsets"
                                ],
                            }
                        )
                        paired_record["motor_step_totals"] = paired_spec[
                            "motor_step_totals"
                        ]
                        paired_record["estimated_duration_s"] = paired_spec[
                            "estimated_duration_s"
                        ]
                    if record["group"] == "belts":
                        if sync_clamp_profile:
                            paired_record["clamp_profile_enabled"] = record.get(
                                "clamp_profile_enabled", False
                            )
                            paired_record["clamp_profile"] = (
                                reverse_clamp_profile(
                                    record["clamp_profile"], record["steps"]
                                )
                                if record.get("clamp_profile_enabled")
                                else [
                                    {"at_step": 0, "offset_steps": 0},
                                    {"at_step": record["steps"], "offset_steps": 0},
                                ]
                            )
                        else:
                            paired_enabled = bool(
                                paired_previous.get("clamp_profile_enabled", False)
                            )
                            paired_record["clamp_profile_enabled"] = paired_enabled
                            if paired_enabled:
                                old_steps = int(
                                    paired_previous.get("steps", record["steps"])
                                )
                                old_profile = normalize_clamp_profile(
                                    paired_previous.get("clamp_profile"), old_steps
                                )
                                paired_record["clamp_profile"] = rescale_clamp_profile(
                                    old_profile, old_steps, record["steps"]
                                )
                            else:
                                paired_record["clamp_profile"] = [
                                    {"at_step": 0, "offset_steps": 0},
                                    {"at_step": record["steps"], "offset_steps": 0},
                                ]
                    self.actions[pair_key] = paired_record
            self.session_log.save_actions(self.actions)
        self.session_log.append({"type": "action_saved", "action": record})
        if paired_record:
            self.session_log.append(
                {
                    "type": "action_pair_synced",
                    "source_action_id": key,
                    "paired_action": paired_record,
                    "clamp_profile_copied_as_reverse": (
                        sync_clamp_profile and record.get("group") == "belts"
                    ),
                }
            )
            pair_detail = (
                "; clamp profile copied as its exact reverse"
                if record.get("group") == "belts" and sync_clamp_profile
                else "; clamp profiles kept separate"
                if record.get("group") == "belts"
                else ""
            )
            self.events.emit(
                "action",
                f"Saved {name}; {paired_record['name']} motion updated as its exact opposite"
                f"{pair_detail}",
                action=record,
                paired_action=paired_record,
            )
        else:
            self.events.emit("action", f"Saved action: {name}", action=record)
        return record

    def delete_action(self, action_id: str) -> None:
        with self._lock:
            action = self.actions.pop(action_id, None)
            if action is None:
                raise RuntimeError("Saved action not found")
            self.session_log.save_actions(self.actions)
        self.session_log.append({"type": "action_deleted", "action": action})
        self.events.emit("action", f"Deleted action: {action['name']}")

    def run_action(self, action_id: str, reverse: bool) -> dict[str, Any]:
        with self._lock:
            action = dict(self.actions.get(action_id, {}))
        if not action:
            raise RuntimeError("Saved action not found")
        if not action.get("configured", True):
            raise RuntimeError("Edit and save this action before running it")
        if action.get("kind", "stepper") == "lac":
            if reverse:
                raise RuntimeError(
                    "Linear-actuator actions use absolute positions and cannot be reversed"
                )
            before = self._fresh_tof_snapshot()
            result = self.start_lac_motion(
                "both",
                int(action["lac_percent"]),
                int(action.get("lac_speed_percent_s", 25)),
                action["name"],
                support_clamp_steps=int(
                    action.get("support_clamp_steps", DEFAULT_LAC_SUPPORT_STEPS)
                ),
                support_max_hz=int(
                    action.get("support_max_hz", DEFAULT_LAC_SUPPORT_HZ)
                ),
                support_ramp_percent=int(
                    action.get(
                        "support_ramp_percent",
                        DEFAULT_LAC_SUPPORT_RAMP_PERCENT,
                    )
                ),
                support_direction="R",
            )
            result["tof_at_start"] = before
            self.session_log.append({"type": "action_run", "action": action, **result})
            return result
        data = {
            "name": action["name"] + (" (reverse)" if reverse else ""),
            "group": action["group"],
            "direction_a": action["directions"][0],
            "steps": action["steps"],
            "max_hz": action["max_hz"],
            "ramp_percent": action["ramp_percent"],
            "motor_step_offsets": action.get("motor_step_offsets", {}),
            "instructions": action.get("instructions", ""),
            "clamp_profile_enabled": action.get("clamp_profile_enabled", False),
            "clamp_profile": action.get("clamp_profile"),
        }
        if len(action["directions"]) > 1:
            data["direction_b"] = action["directions"][1]
        return self.start_motion(data, reverse)

    def demo_lac_action(self, data: dict[str, Any]) -> dict[str, Any]:
        key = re.sub(
            r"[^a-z0-9-]+", "", str(data.get("action_id", "")).lower()
        ).strip("-")
        if not key:
            raise RuntimeError("Save the action before applying its target")
        with self._lock:
            action = dict(self.actions.get(key, {}))
            before_lac = dict(self.lac)
        if action.get("kind") != "lac":
            raise RuntimeError("This is not a linear-actuator action")
        percent = int(data.get("lac_percent", 0))
        speed = int(data.get("lac_speed_percent_s", 25))
        support_steps = int(
            data.get(
                "support_clamp_steps",
                action.get("support_clamp_steps", DEFAULT_LAC_SUPPORT_STEPS),
            )
        )
        support_hz = int(
            data.get(
                "support_max_hz",
                action.get("support_max_hz", DEFAULT_LAC_SUPPORT_HZ),
            )
        )
        support_ramp = int(
            data.get(
                "support_ramp_percent",
                action.get(
                    "support_ramp_percent", DEFAULT_LAC_SUPPORT_RAMP_PERCENT
                ),
            )
        )
        if not 0 <= percent <= 100:
            raise RuntimeError("Linear actuator target must be 0..100%")
        target_range = self._tof_range_from_data(data)
        before_tof = self._fresh_tof_snapshot()
        return self.start_lac_motion(
            "both",
            percent,
            speed,
            f"Demo {str(data.get('name', action['name']))[:80]}",
            demo_context={
                "demo_key": key,
                "demo_tof_before": before_tof,
                "demo_tof_range": target_range,
                "demo_from_lac": before_lac,
            },
            support_clamp_steps=support_steps,
            support_max_hz=support_hz,
            support_ramp_percent=support_ramp,
            support_direction="R",
        )

    def demo_action_target(self, data: dict[str, Any]) -> dict[str, Any]:
        key = re.sub(
            r"[^a-z0-9-]+", "", str(data.get("action_id", "")).lower()
        ).strip("-")
        if not key:
            raise RuntimeError("Save the action before demoing its target")
        target = int(data.get("target_steps", data.get("steps", 0)))
        if not 0 <= target <= MAX_STEPS:
            raise RuntimeError(f"Demo target must be 0..{MAX_STEPS} steps")

        requested_spec = self._motion_spec(data)
        target_range = self._tof_range_from_data(data)
        demo_before_tof = self._fresh_tof_snapshot()
        requested_context = {
            "group": requested_spec["group"],
            "direction_a": requested_spec["directions"][0],
            "direction_b": (
                requested_spec["directions"][1]
                if len(requested_spec["directions"]) > 1 else None
            ),
        }
        requested_signature = (
            requested_context["group"], tuple(requested_spec["directions"])
        )
        with self._lock:
            current = int(self.action_demo_positions.get(key, 0))
            current_clamp_offset = int(
                self.action_demo_clamp_offsets.get(key, 0)
            )
            stored_context = dict(self.action_demo_contexts.get(key, {}))
            stored_signature = (
                stored_context.get("group"),
                tuple(stored_context.get("directions", [])),
            ) if stored_context else None

        if current != 0 and stored_signature != requested_signature:
            if target != 0:
                raise RuntimeError(
                    "Return the demo to 0 before changing its motor movement"
                )
            motion_data = {
                **data,
                "group": stored_context["group"],
                "direction_a": stored_context["directions"][0],
            }
            if len(stored_context["directions"]) > 1:
                motion_data["direction_b"] = stored_context["directions"][1]
        else:
            motion_data = dict(data)

        delta = target - current
        if delta == 0:
            tof_result = self._classify_tof(demo_before_tof, target_range)
            demo_record = {
                "action_id": key,
                "action_name": str(data.get("name", key))[:80],
                "from_steps": current,
                "requested_target_steps": target,
                "actual_position_steps": current,
                "commanded_delta_steps": 0,
                "delivered_delta_steps": 0,
                "from_clamp_offset_steps": current_clamp_offset,
                "delivered_clamp_delta_steps": 0,
                "actual_clamp_offset_steps": current_clamp_offset,
                "tof_before": demo_before_tof,
                "tof_after": demo_before_tof,
                "tof_range": target_range,
                "tof_result": tof_result,
                "finished_at": time.time(),
                "state": "already_at_target",
            }
            with self._lock:
                self.action_demo_history.append(demo_record)
                self.action_demo_history = self.action_demo_history[-100:]
            self.session_log.append({"type": "action_demo_result", **demo_record})
            self.events.emit(
                "action",
                f"Demo already at {target} steps; {tof_result['label']}",
                action_id=key,
                result=demo_record,
            )
            return {
                "action_id": key,
                "demo_position_steps": current,
                "target_steps": target,
                "delta_steps": 0,
                "state": "already_at_target",
                "tof": demo_before_tof,
                "tof_result": tof_result,
            }

        motion_data.update(
            {
                "name": f"Demo {str(data.get('name', key))[:80]} to {target}",
                "steps": abs(delta),
            }
        )
        profile_enabled = bool(requested_spec.get("clamp_profile_enabled"))
        full_profile = list(requested_spec.get("clamp_profile", []))
        if (
            target == 0 and stored_context.get("clamp_profile_enabled")
            and stored_context.get("clamp_profile")
        ):
            profile_enabled = True
            full_profile = list(stored_context["clamp_profile"])
        if profile_enabled:
            traversal_profile = slice_clamp_profile(
                full_profile, current, target, current_clamp_offset
            )
            motion_data.update(
                {
                    "clamp_profile_enabled": True,
                    "clamp_profile": traversal_profile,
                    "clamp_profile_is_traversal": True,
                }
            )
        else:
            motion_data.update(
                {"clamp_profile_enabled": False, "clamp_profile": None}
            )
        context_to_store = {
            "group": motion_data["group"],
            "directions": [motion_data["direction_a"]] + (
                [motion_data["direction_b"]]
                if motion_data.get("direction_b") else []
            ),
            "clamp_profile_enabled": profile_enabled,
            "clamp_profile": full_profile,
        }
        with self._lock:
            self.action_demo_contexts[key] = context_to_store
        try:
            motion = self.start_motion(
                motion_data,
                reverse=delta < 0,
                demo_context={
                    "demo_key": key,
                    "demo_start_steps": current,
                    "demo_requested_target_steps": target,
                    "demo_delta_sign": 1 if delta > 0 else -1,
                    "demo_start_clamp_offset": current_clamp_offset,
                    "demo_tof_before": demo_before_tof,
                    "demo_tof_range": target_range,
                },
            )
        except Exception:
            if current == 0:
                with self._lock:
                    self.action_demo_contexts.pop(key, None)
            raise
        self.session_log.append(
            {
                "type": "action_demo_requested",
                "action_id": key,
                "from_steps": current,
                "target_steps": target,
                "delta_steps": delta,
                "from_clamp_offset_steps": current_clamp_offset,
                "target_clamp_offset_steps": (
                    clamp_profile_value(full_profile, target)
                    if profile_enabled else current_clamp_offset
                ),
                "motion_id": motion["id"],
                "tof_before": demo_before_tof,
                "tof_range": target_range,
            }
        )
        return motion

    def rebase_action_demo(self, action_id: str) -> dict[str, Any]:
        key = re.sub(r"[^a-z0-9-]+", "", action_id.lower()).strip("-")
        if not key:
            raise RuntimeError("Action id is required")
        with self._lock:
            if self.active_motion is not None:
                raise RuntimeError("Stop the active movement before resetting the demo origin")
            previous = int(self.action_demo_positions.get(key, 0))
            previous_clamp = int(self.action_demo_clamp_offsets.get(key, 0))
            self.action_demo_positions[key] = 0
            self.action_demo_clamp_offsets[key] = 0
            self.action_demo_contexts.pop(key, None)
        self._persist_runtime_state()
        self.session_log.append(
            {"type": "action_demo_rebased", "action_id": key,
             "previous_steps": previous,
             "previous_clamp_offset_steps": previous_clamp}
        )
        self.events.emit(
            "action", f"Demo origin reset to 0 for {key}", action_id=key
        )
        return {"action_id": key, "demo_position_steps": 0}

    def reset_counters(self) -> None:
        with self._lock:
            previous = dict(self.motor_steps)
            self.motor_steps = {str(motor): 0 for motor in (1, 3, 4, 6, 7)}
            self.counter_epoch += 1
            epoch = self.counter_epoch
        self._persist_runtime_state()
        self.session_log.append(
            {"type": "step_counters_reset", "previous": previous,
             "counter_epoch": epoch}
        )
        self.events.emit(
            "action", "Commanded-step counters reset; timeline starts a new epoch"
        )

    def add_step_flag(self, label: str) -> dict[str, Any]:
        clean = label.strip()[:100]
        with self._lock:
            number = len(self.step_flags) + 1
            flag = {
                "id": uuid.uuid4().hex[:10],
                "number": number,
                "label": clean or f"Flag {number}",
                "captured_at": time.time(),
                "captured_iso": utc_timestamp(),
                "counter_epoch": self.counter_epoch,
                "motor_steps": dict(self.motor_steps),
                "tof": {
                    "valid": bool(self.tof.get("valid")),
                    "distance_mm": self.tof.get("distance_mm"),
                    "raw_mm": self.tof.get("raw_mm"),
                    "status": self.tof.get("status"),
                },
                "lac": dict(self.lac),
            }
            self.step_flags.append(flag)
        self._persist_runtime_state()
        self.session_log.append({"type": "step_flag", "flag": flag})
        self.events.emit(
            "flag", f"Flag {number} saved: {flag['label']}", flag=flag
        )
        return flag

    def clear_step_flags(self) -> None:
        with self._lock:
            previous = list(self.step_flags)
            self.step_flags = []
        self._persist_runtime_state()
        self.session_log.append({"type": "step_flags_cleared", "flags": previous})
        self.events.emit("flag", "Step timeline cleared")

    def add_note(self, note: str) -> None:
        clean = note.strip()[:1000]
        if not clean:
            raise RuntimeError("Note cannot be empty")
        self.session_log.append({"type": "operator_note", "note": clean})
        self.events.emit("note", clean)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            active = dict(self.active_motion) if self.active_motion else None
            return {
                "connected": self.serial.connected,
                "serial_port": self.serial.port,
                "controls_armed": self.controls_armed,
                "active_motion": active,
                "paused_motion": (
                    dict(self.paused_motion) if self.paused_motion else None
                ),
                "last_motion": dict(self.last_motion) if self.last_motion else None,
                "motor_steps": dict(self.motor_steps),
                "lac": dict(self.lac),
                "tof": dict(self.tof),
                "actions": sorted(
                    self.actions.values(),
                    key=lambda item: (int(item.get("order", 9999)), item["name"].lower()),
                ),
                "action_demo_positions": dict(self.action_demo_positions),
                "action_demo_clamp_offsets": dict(self.action_demo_clamp_offsets),
                "action_demo_history": list(self.action_demo_history),
                "step_flags": list(self.step_flags),
                "counter_epoch": self.counter_epoch,
                "groups": GROUPS,
                "session_log": str(self.session_log.path),
                "actions_path": str(self.session_log.actions_path),
                "event_id": self.events.latest_id(),
            }


class RobotHandler(SimpleHTTPRequestHandler):
    server_version = "RobotControl/1.0"

    @property
    def app(self) -> "RobotServer":
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
            self._send_json(self.app.console.snapshot())
            return
        if parsed.path == "/api/events":
            params = parse_qs(parsed.query)
            try:
                after = int(params.get("after", ["0"])[0])
            except ValueError:
                after = 0
            self._send_json({"events": self.app.events.after(after)})
            return
        if parsed.path == "/":
            self.path = "/index.html"
        super().do_GET()

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        try:
            data = self._json_body()
            if parsed.path == "/api/arm":
                self.app.console.set_controls_armed(bool(data.get("armed")))
                result: Any = self.app.console.snapshot()
            elif parsed.path == "/api/stop":
                self.app.console.stop(str(data.get("reason", "operator")))
                result = self.app.console.snapshot()
            elif parsed.path == "/api/tof":
                self.app.console.read_tof()
                result = self.app.console.snapshot()
            elif parsed.path == "/api/lac":
                result = self.app.console.start_lac_motion(
                    str(data.get("target", "both")),
                    int(data.get("percent", 0)),
                    int(data.get("speed_percent_s", 25)),
                )
            elif parsed.path == "/api/motion":
                result = self.app.console.start_motion(data, bool(data.get("reverse", False)))
            elif parsed.path == "/api/motion/preview":
                result = self.app.console.preview_motion(data)
            elif parsed.path == "/api/motion/recover":
                result = self.app.console.recover_paused_motion(
                    str(data.get("mode", ""))
                )
            elif parsed.path == "/api/action/save":
                result = self.app.console.save_action(data)
            elif parsed.path == "/api/action/run":
                result = self.app.console.run_action(
                    str(data.get("action_id", "")), bool(data.get("reverse", False))
                )
            elif parsed.path == "/api/action/demo":
                result = self.app.console.demo_action_target(data)
            elif parsed.path == "/api/action/demo/lac":
                result = self.app.console.demo_lac_action(data)
            elif parsed.path == "/api/action/demo/rebase":
                result = self.app.console.rebase_action_demo(
                    str(data.get("action_id", ""))
                )
            elif parsed.path == "/api/action/delete":
                self.app.console.delete_action(str(data.get("action_id", "")))
                result = self.app.console.snapshot()
            elif parsed.path == "/api/counters/reset":
                self.app.console.reset_counters()
                result = self.app.console.snapshot()
            elif parsed.path == "/api/flag":
                result = self.app.console.add_step_flag(str(data.get("label", "")))
            elif parsed.path == "/api/flags/clear":
                self.app.console.clear_step_flags()
                result = self.app.console.snapshot()
            elif parsed.path == "/api/note":
                self.app.console.add_note(str(data.get("note", "")))
                result = self.app.console.snapshot()
            else:
                self._send_json({"error": "Unknown API endpoint"}, HTTPStatus.NOT_FOUND)
                return
            self._send_json({"ok": True, "result": result})
        except (RuntimeError, ValueError, TypeError) as exc:
            self.app.events.emit("error", str(exc))
            self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)


class RobotServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], static_dir: Path,
                 events: EventStore, console: RobotConsole) -> None:
        self.events = events
        self.console = console

        def factory(*args: Any, **kwargs: Any) -> RobotHandler:
            return RobotHandler(*args, directory=str(static_dir), **kwargs)

        super().__init__(address, factory)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--serial", dest="serial_port")
    parser.add_argument("--data-dir", type=Path)
    parser.add_argument("--simulate", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = Path(__file__).resolve().parent
    data_dir = args.data_dir or root / "data"
    session_log = SessionLog(data_dir)
    events = EventStore(session_log)
    console = RobotConsole(events, session_log)
    console.configure_serial(args.serial_port, args.simulate)
    server = RobotServer((args.host, args.port), root / "static", events, console)
    stopping = threading.Event()

    def shutdown_handler(signum: int, frame: Any) -> None:
        if stopping.is_set():
            return
        stopping.set()
        console.stop("server signal")
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    console.start()
    url = f"http://{args.host}:{args.port}"
    print(f"Robot control console: {url}")
    print(f"Session log: {session_log.path}")
    print(f"Saved actions: {session_log.actions_path}")

    try:
        server.serve_forever(poll_interval=0.2)
    finally:
        console.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
