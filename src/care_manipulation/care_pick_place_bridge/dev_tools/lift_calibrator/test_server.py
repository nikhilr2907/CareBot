#!/usr/bin/env python3
"""Focused tests for the lift calibration policy and safety limits."""

import tempfile
import unittest
from pathlib import Path

from server import (
    EventStore,
    LiftCalibration,
    MAX_BASE_HZ,
    SessionLog,
    SerialController,
    motor_speeds,
)


class LiftCalibrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_dir = tempfile.TemporaryDirectory()
        session_log = SessionLog(Path(self.temporary_dir.name))
        events = EventStore(session_log)
        serial = SerialController(events, requested_port=None, simulate=True)
        self.calibration = LiftCalibration(serial, events, session_log)
        self.calibration.set_armed(True)

    def tearDown(self) -> None:
        self.temporary_dir.cleanup()

    def test_trim_never_increases_either_side_above_base(self) -> None:
        self.assertEqual(motor_speeds(1000, 10), (1000, 900))
        self.assertEqual(motor_speeds(1000, -10), (900, 1000))

    def test_right_stall_slows_left_on_next_trial(self) -> None:
        self.calibration.start_trial("UP", 1000, 0, 500, 10)
        result = self.calibration.feedback("right_stall")
        self.assertEqual(result["recommendation"]["trim_percent"], 2.5)
        self.assertEqual(motor_speeds(1000, 2.5), (1000, 975))

    def test_left_stall_slows_right_on_next_trial(self) -> None:
        self.calibration.start_trial("UP", 1000, 0, 500, 10)
        result = self.calibration.feedback("left_stall")
        self.assertEqual(result["recommendation"]["trim_percent"], -2.5)
        self.assertEqual(motor_speeds(1000, -2.5), (975, 1000))

    def test_vibration_reduces_base_speed(self) -> None:
        self.calibration.start_trial("DOWN", 1000, 0, 500, 70)
        result = self.calibration.feedback("vibration")
        self.assertEqual(result["recommendation"]["base_hz"], 800)

    def test_motion_above_limit_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "Base speed"):
            self.calibration.start_trial("UP", MAX_BASE_HZ + 1, 0, 500, 10)


if __name__ == "__main__":
    unittest.main()
