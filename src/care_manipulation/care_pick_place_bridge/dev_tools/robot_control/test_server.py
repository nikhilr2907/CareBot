#!/usr/bin/env python3

import tempfile
import time
import unittest
from pathlib import Path

from server import (
    EventStore, RobotConsole, SessionLog, apply_motor_step_offsets,
    build_composite_plan, build_full_duration_support_plan, build_ramp_plan,
    normalize_clamp_profile, reverse_clamp_profile,
)


class RampPlanTests(unittest.TestCase):
    def test_no_ramp_is_one_segment(self) -> None:
        plan = build_ramp_plan(500, 500, 0)
        self.assertEqual(plan, [{"hz": 500, "steps": 500, "duration_s": 1.0}])

    def test_ramp_preserves_requested_steps(self) -> None:
        plan = build_ramp_plan(1237, 800, 30)
        self.assertEqual(sum(segment["steps"] for segment in plan), 1237)
        self.assertEqual(plan[0]["hz"], 160)
        self.assertEqual(plan[-1]["hz"], 160)

    def test_ramp_uses_symmetric_linear_speed_stages(self) -> None:
        plan = build_ramp_plan(500, 500, 20)
        self.assertEqual(
            [segment["hz"] for segment in plan],
            [100, 200, 300, 400, 500, 400, 300, 200, 100],
        )

    def test_long_motion_is_estimated_but_not_time_blocked(self) -> None:
        plan = build_ramp_plan(100_000, 100, 45)
        self.assertGreater(sum(segment["duration_s"] for segment in plan), 60.0)
        self.assertEqual(sum(segment["steps"] for segment in plan), 100_000)

        no_ramp = build_ramp_plan(100_000, 100, 0)
        self.assertEqual(sum(segment["duration_s"] for segment in no_ramp), 1000.0)

    def test_large_action_step_count_is_allowed(self) -> None:
        plan = build_ramp_plan(10_000_000, 4000, 20)
        self.assertEqual(sum(segment["steps"] for segment in plan), 10_000_000)
        self.assertTrue(all(segment["steps"] <= 100_000 for segment in plan))

        with self.assertRaisesRegex(RuntimeError, "Steps must be"):
            build_ramp_plan(10_000_001, 4000, 20)

    def test_per_motor_extra_steps_share_every_ramp_slice(self) -> None:
        plan = apply_motor_step_offsets(
            build_ramp_plan(225_000, 4_000, 10),
            (1, 6),
            {"1": 300, "6": 0},
        )
        self.assertTrue(all(segment["coordinated"] for segment in plan))
        self.assertTrue(all(segment["motors"] == [1, 6] for segment in plan))
        self.assertEqual(sum(segment["primary_steps"] for segment in plan), 225_000)
        self.assertEqual(sum(segment["motor_steps"][0] for segment in plan), 225_300)
        self.assertEqual(sum(segment["motor_steps"][1] for segment in plan), 225_000)
        self.assertTrue(all(segment["hz"] <= 4_000 for segment in plan))
        self.assertTrue(
            all(max(segment["motor_steps"]) <= 100_000 for segment in plan)
        )
        self.assertTrue(
            all(
                segment["motor_steps"][1] / segment["duration_s"]
                <= segment["motor_steps"][0] / segment["duration_s"]
                for segment in plan
            )
        )

    def test_clamp_support_accumulates_across_full_action_without_falloff(self) -> None:
        plan, duration = build_full_duration_support_plan(300, 500, 10.0)
        self.assertEqual(sum(item["steps"] for item in plan), 300)
        self.assertEqual(plan[0]["start_offset_s"], 0.0)
        self.assertAlmostEqual(
            plan[-1]["start_offset_s"] + plan[-1]["duration_s"],
            duration,
            places=6,
        )
        self.assertGreater(plan[-1]["start_offset_s"], duration * 0.9)
        self.assertTrue(
            all(
                earlier["start_offset_s"] <= later["start_offset_s"]
                for earlier, later in zip(plan, plan[1:])
            )
        )

    def test_user_conveyor_clamp_profile_runs_simultaneously(self) -> None:
        profile = normalize_clamp_profile(
            [
                {"at_step": 0, "offset_steps": 0},
                {"at_step": 250, "offset_steps": 3_000},
                {"at_step": 2_000, "offset_steps": 50_000},
            ],
            2_000,
        )
        plan = build_composite_plan(
            build_ramp_plan(2_000, 2_000, 45), ["R", "F"], profile
        )
        duration = sum(segment["duration_s"] for segment in plan)
        self.assertLessEqual(duration, 60.0)
        self.assertTrue(all(segment["coordinated"] for segment in plan))
        self.assertTrue(all(segment["primary_steps"] > 0 for segment in plan))
        self.assertTrue(all(segment["hz"] <= 4_000 for segment in plan))
        self.assertEqual(sum(segment["primary_steps"] for segment in plan), 2_000)
        self.assertEqual(sum(segment["clamp_steps"] for segment in plan), 50_000)

    def test_composite_plan_preserves_belts_and_clamp_offset(self) -> None:
        profile = normalize_clamp_profile(
            [
                {"at_step": 0, "offset_steps": 0},
                {"at_step": 50, "offset_steps": 0},
                {"at_step": 100, "offset_steps": 20},
            ],
            100,
        )
        plan = build_composite_plan(
            build_ramp_plan(100, 1000, 0), ["R", "F"], profile
        )
        self.assertEqual(sum(item["primary_steps"] for item in plan), 100)
        self.assertEqual(
            sum(item["clamp_steps"] * item["clamp_sign"] for item in plan), 20
        )
        self.assertTrue(any(item["motors"] == [1, 3, 4, 6] for item in plan))
        self.assertTrue(all(item["primary_steps"] > 0 for item in plan))
        changing = [item for item in plan if item["clamp_steps"]]
        self.assertTrue(all(item["motor_steps"][0] > 0 for item in changing))

    def test_large_composite_plan_is_split_for_board_protocol(self) -> None:
        profile = normalize_clamp_profile(
            [
                {"at_step": 0, "offset_steps": 0},
                {"at_step": 500_000, "offset_steps": 50_000},
            ],
            500_000,
        )
        plan = build_composite_plan(
            build_ramp_plan(500_000, 4000, 0), ["R", "F"], profile
        )
        self.assertEqual(sum(item["primary_steps"] for item in plan), 500_000)
        self.assertEqual(sum(item["clamp_steps"] for item in plan), 50_000)
        self.assertTrue(
            all(max(item["motor_steps"]) <= 100_000 for item in plan)
        )

    def test_reverse_clamp_profile_retraces_curve(self) -> None:
        reverse = reverse_clamp_profile(
            [
                {"at_step": 0, "offset_steps": 0},
                {"at_step": 400, "offset_steps": 0},
                {"at_step": 1000, "offset_steps": 120},
            ],
            1000,
        )
        self.assertEqual(
            reverse,
            [
                {"at_step": 0, "offset_steps": 0},
                {"at_step": 600, "offset_steps": -120},
                {"at_step": 1000, "offset_steps": -120},
            ],
        )


class ActionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.log = SessionLog(Path(self.temp.name))
        self.events = EventStore(self.log)
        self.console = RobotConsole(self.events, self.log)
        self.console.configure_serial(None, True)
        self.console.serial.start()
        self.console.set_controls_armed(True)

    def tearDown(self) -> None:
        self.console.serial.close()
        self.temp.cleanup()

    def test_reverse_flips_both_directions(self) -> None:
        spec = self.console._motion_spec(
            {"group": "belts", "direction_a": "R", "direction_b": "F",
             "steps": 500, "max_hz": 500, "ramp_percent": 0},
            reverse=True,
        )
        self.assertEqual(spec["directions"], ["F", "R"])

    def test_preview_reports_ramp_aware_time_without_moving(self) -> None:
        preview = self.console.preview_motion(
            {"group": "mechanism", "direction_a": "F", "direction_b": "R",
             "steps": 100_000, "max_hz": 100, "ramp_percent": 45}
        )
        self.assertGreater(preview["estimated_duration_s"], 60.0)
        self.assertEqual(preview["steps"], 100_000)
        self.assertIsNone(self.console.active_motion)
        self.assertEqual(
            self.console.motor_steps,
            {"1": 0, "3": 0, "4": 0, "6": 0, "7": 0},
        )

    def test_saved_action_persists(self) -> None:
        action = self.console.save_action(
            {"name": "Clamp on box", "group": "mechanism",
             "direction_a": "R", "direction_b": "R", "steps": 300,
             "max_hz": 400, "ramp_percent": 20, "instructions": "Test box"}
        )
        self.assertEqual(action["directions"], ["R", "R"])
        self.assertIn(action["id"], self.log.load_actions())

    def test_default_actions_separate_conveyor_and_linear_actuator(self) -> None:
        self.assertEqual(
            self.console.actions["box-out"]["name"], "Conveyor: box out"
        )
        self.assertEqual(
            self.console.actions["actuator-box-out"]["name"],
            "Linear actuator: box out",
        )
        self.assertEqual(self.console.actions["actuator-box-out"]["kind"], "lac")
        self.assertEqual(len(self.console.actions), 8)

    def test_shift_actions_have_independent_editable_alignment_trim(self) -> None:
        self.assertEqual(
            self.console.actions["move-left"]["motor_step_offsets"],
            {"1": 300, "6": 0},
        )
        self.assertEqual(
            self.console.actions["move-right"]["motor_step_offsets"],
            {"1": 0, "6": 300},
        )
        move_left = self.console.save_action(
            {"action_id": "move-left", "name": "System: move left",
             "group": "mechanism", "direction_a": "R", "direction_b": "F",
             "steps": 200, "max_hz": 4000, "ramp_percent": 20,
             "motor_step_offsets": {"1": 420, "6": 10},
             "tof_mode": "unspecified"}
        )
        self.assertEqual(move_left["motor_step_offsets"], {"1": 420, "6": 10})
        self.assertEqual(move_left["motor_step_totals"], {"1": 620, "6": 210})
        self.assertEqual(
            self.console.actions["move-right"]["motor_step_offsets"],
            {"1": 0, "6": 300},
        )

    def test_linear_actuator_action_commands_lac_pair_and_clamp_support(self) -> None:
        action = self.console.save_action(
            {"action_id": "actuator-box-out",
             "name": "Linear actuator: box out", "kind": "lac",
             "lac_percent": 5, "lac_speed_percent_s": 100,
             "support_clamp_steps": 20, "support_max_hz": 1000,
             "support_ramp_percent": 0,
             "tof_mode": "not_applicable"}
        )
        result = self.console.run_action(action["id"], False)
        self.console._motion_thread.join(timeout=1.0)
        self.assertEqual(result["kind"], "lac")
        self.assertEqual(result["speed_percent_s"], 100)
        self.assertEqual(self.console.lac, {"1": 5, "2": 5})
        self.assertEqual(self.console.motor_steps, {"1": -20, "3": 0, "4": 0, "6": -20, "7": 0})
        self.assertIsNone(self.console.active_motion)
        self.assertFalse(self.console.actions["actuator-box-in"]["configured"])

    def test_both_builtin_actuator_actions_default_to_300_clamp_steps(self) -> None:
        for action_id in ("actuator-box-out", "actuator-box-in"):
            action = self.console.actions[action_id]
            self.assertEqual(action["support_clamp_steps"], 300)
            self.assertEqual(action["support_max_hz"], 500)
            self.assertEqual(action["support_ramp_percent"], 100)

    def test_linear_actuator_editor_demo_applies_absolute_percentage(self) -> None:
        self.console.save_action(
            {"action_id": "actuator-box-in",
             "name": "Linear actuator: box in", "kind": "lac",
             "lac_percent": 4, "lac_speed_percent_s": 100,
             "support_clamp_steps": 20, "support_max_hz": 1000,
             "support_ramp_percent": 0,
             "tof_mode": "unspecified"}
        )
        result = self.console.demo_lac_action(
            {"action_id": "actuator-box-in",
             "name": "Linear actuator: box in", "kind": "lac",
             "lac_percent": 4, "lac_speed_percent_s": 100,
             "support_clamp_steps": 20, "support_max_hz": 1000,
             "support_ramp_percent": 0,
             "tof_mode": "unspecified"}
        )
        self.console._motion_thread.join(timeout=1.0)
        self.assertEqual(result["start_values"], {"1": 0, "2": 0})
        self.assertEqual(result["target_values"], {"1": 4, "2": 4})
        self.assertEqual(self.console.lac, {"1": 4, "2": 4})
        self.assertEqual(self.console.action_demo_history[-1]["kind"], "lac")
        self.assertEqual(self.console.action_demo_history[-1]["speed_percent_s"], 100)
        self.assertEqual(self.console.action_demo_history[-1]["support_completed_steps"], 20)

    def test_linear_actuator_speed_ramp_can_stop_and_resume_both(self) -> None:
        self.console.start_lac_motion(
            "both", 20, 100, "Paired actuator ramp"
        )
        time.sleep(0.08)
        self.console.stop("test LAC pause")
        self.console._motion_thread.join(timeout=1.0)
        paused = self.console.paused_motion
        self.assertIsNotNone(paused)
        self.assertEqual(paused["kind"], "lac")
        self.assertEqual(paused["current_values"]["1"], paused["current_values"]["2"])
        self.assertLess(paused["current_values"]["1"], 20)

        self.console.recover_paused_motion("resume")
        self.console._motion_thread.join(timeout=1.0)
        self.assertEqual(self.console.lac, {"1": 20, "2": 20})
        self.assertIsNone(self.console.paused_motion)

    def test_arming_preserves_existing_lac_targets(self) -> None:
        self.console.set_lac("1", 37)
        self.console.set_lac("2", 64)
        self.console.set_controls_armed(False)
        self.console.set_controls_armed(True)
        self.assertEqual(self.console.lac, {"1": 37, "2": 64})
        self.assertEqual(self.console.serial._sim_lac, {"1": 37, "2": 64})

    def test_single_motor_manual_motion_updates_only_that_counter(self) -> None:
        self.console.start_motion(
            {"name": "M1 nudge", "group": "m1", "direction_a": "F",
             "steps": 20, "max_hz": 200, "ramp_percent": 0}
        )
        self.console._motion_thread.join(timeout=1.0)
        self.assertEqual(self.console.motor_steps["1"], 20)
        self.assertEqual(self.console.motor_steps["6"], 0)

    def test_stopped_paired_motion_can_resume_remaining_steps(self) -> None:
        self.console.start_motion(
            {"name": "Shift pair", "group": "mechanism",
             "direction_a": "F", "direction_b": "R", "steps": 400,
             "max_hz": 4000, "ramp_percent": 0}
        )
        time.sleep(0.03)
        self.console.stop("test pause")
        self.console._motion_thread.join(timeout=1.0)
        paused = self.console.paused_motion
        self.assertIsNotNone(paused)
        self.assertGreater(paused["completed_steps"], 0)
        self.assertEqual(
            paused["completed_steps"] + paused["remaining_steps"], 400
        )

        self.console.recover_paused_motion("resume")
        self.console._motion_thread.join(timeout=1.0)
        self.assertIsNone(self.console.paused_motion)
        self.assertEqual(self.console.motor_steps["1"], 400)
        self.assertEqual(self.console.motor_steps["6"], -400)

    def test_stopped_conveyor_and_clamp_profile_return_together(self) -> None:
        self.console.start_motion(
            {"name": "Paired conveyor clamp", "group": "belts",
             "direction_a": "R", "direction_b": "F", "steps": 400,
             "max_hz": 4000, "ramp_percent": 0,
             "clamp_profile_enabled": True,
             "clamp_profile": [
                 {"at_step": 0, "offset_steps": 0},
                 {"at_step": 400, "offset_steps": 200},
             ]}
        )
        time.sleep(0.03)
        self.console.stop("test paired pause")
        self.console._motion_thread.join(timeout=1.0)
        paused = self.console.paused_motion
        self.assertIsNotNone(paused)
        self.assertTrue(paused["clamp_profile_enabled"])
        self.assertGreater(paused["completed_steps"], 0)
        self.assertNotEqual(paused["clamp_offset_delta_steps"], 0)

        self.console.recover_paused_motion("return")
        self.console._motion_thread.join(timeout=1.0)
        self.assertIsNone(self.console.paused_motion)
        self.assertEqual(
            self.console.motor_steps,
            {"1": 0, "3": 0, "4": 0, "6": 0, "7": 0},
        )

    def test_direct_positioning_does_not_modify_saved_actions(self) -> None:
        before = {
            action_id: dict(action)
            for action_id, action in self.console.actions.items()
        }
        self.console.start_motion(
            {"name": "Direct positioning", "group": "m1",
             "direction_a": "F", "steps": 10, "max_hz": 200,
             "ramp_percent": 0,
             "instructions": "Temporary direct positioning; not saved to Actions"}
        )
        self.console._motion_thread.join(timeout=1.0)
        self.assertEqual(self.console.actions, before)

    def test_step_flags_capture_counter_and_tof_snapshots(self) -> None:
        self.console.read_tof()
        first = self.console.add_step_flag("Start")
        self.console.motor_steps["3"] = -250
        second = self.console.add_step_flag("Box out")
        self.assertEqual(first["motor_steps"]["3"], 0)
        self.assertEqual(second["motor_steps"]["3"], -250)
        self.assertEqual(second["tof"]["distance_mm"], 620)

    def test_built_in_action_becomes_configured_when_edited(self) -> None:
        action = self.console.save_action(
            {"action_id": "clamp-box", "name": "Clamp box",
             "group": "mechanism", "direction_a": "R", "direction_b": "R",
             "steps": 240, "max_hz": 300, "ramp_percent": 20,
             "tof_reference_mm": 430}
        )
        self.assertTrue(action["configured"])
        self.assertTrue(action["built_in"])
        self.assertEqual(action["tof_reference_mm"], 430)

    def test_action_demo_moves_only_delta_to_new_target(self) -> None:
        base = {
            "action_id": "release-box", "name": "Release box",
            "group": "mechanism", "direction_a": "F", "direction_b": "F",
            "steps": 100, "max_hz": 1000, "ramp_percent": 0,
            "tof_mode": "range", "tof_min_mm": 610,
            "tof_ideal_mm": 620, "tof_max_mm": 630,
        }
        self.console.demo_action_target({**base, "target_steps": 100})
        self.console._motion_thread.join(timeout=1.0)
        self.assertEqual(self.console.action_demo_positions["release-box"], 100)
        self.assertEqual(self.console.motor_steps["1"], 100)

        self.console.demo_action_target({**base, "target_steps": 110})
        self.console._motion_thread.join(timeout=1.0)
        self.assertEqual(self.console.action_demo_positions["release-box"], 110)
        self.assertEqual(self.console.motor_steps["1"], 110)

        self.console.demo_action_target({**base, "target_steps": 108})
        self.console._motion_thread.join(timeout=1.0)
        self.assertEqual(self.console.action_demo_positions["release-box"], 108)
        self.assertEqual(self.console.motor_steps["1"], 108)
        self.assertEqual(len(self.console.action_demo_history), 3)
        self.assertEqual(
            self.console.action_demo_history[-1]["delivered_delta_steps"], -2
        )
        self.assertEqual(
            self.console.action_demo_history[-1]["tof_result"]["code"],
            "near_ideal",
        )

    def test_built_in_pairs_share_motion_but_keep_separate_tof_ranges(self) -> None:
        self.console.save_action(
            {"action_id": "clamp-box", "name": "Clamp box",
             "group": "mechanism", "direction_a": "R", "direction_b": "R",
             "steps": 200, "max_hz": 300, "ramp_percent": 20,
             "tof_mode": "range", "tof_min_mm": 400,
             "tof_ideal_mm": 420, "tof_max_mm": 440}
        )
        release = self.console.save_action(
            {"action_id": "release-box", "name": "Release box",
             "group": "mechanism", "direction_a": "F", "direction_b": "F",
             "steps": 113, "max_hz": 350, "ramp_percent": 10,
             "tof_mode": "range", "tof_min_mm": 600,
             "tof_ideal_mm": 620, "tof_max_mm": 640}
        )
        clamp = self.console.actions["clamp-box"]
        self.assertEqual(release["directions"], ["F", "F"])
        self.assertEqual(clamp["directions"], ["R", "R"])
        self.assertEqual(clamp["steps"], 113)
        self.assertEqual(clamp["max_hz"], 350)
        self.assertEqual(clamp["ramp_percent"], 10)
        self.assertEqual(clamp["tof_min_mm"], 400)
        self.assertEqual(release["tof_min_mm"], 600)

    def test_clamp_and_release_definitions_cannot_be_reversed(self) -> None:
        clamp = self.console.save_action(
            {"action_id": "clamp-box", "name": "Clamp box",
             "kind": "stepper", "group": "belts",
             "direction_a": "F", "direction_b": "F", "steps": 250,
             "max_hz": 400, "ramp_percent": 20,
             "tof_mode": "unspecified"}
        )
        release = self.console.actions["release-box"]
        self.assertEqual(clamp["group"], "mechanism")
        self.assertEqual(clamp["motors"], [1, 6])
        self.assertEqual(clamp["directions"], ["R", "R"])
        self.assertEqual(release["group"], "mechanism")
        self.assertEqual(release["motors"], [1, 6])
        self.assertEqual(release["directions"], ["F", "F"])

    def test_conveyor_action_saves_inverse_clamp_profile_to_pair(self) -> None:
        box_out = self.console.save_action(
            {"action_id": "box-out", "name": "Conveyor: box out",
             "group": "belts", "direction_a": "R", "direction_b": "F",
             "steps": 1000, "max_hz": 4000, "ramp_percent": 0,
             "clamp_profile_enabled": True,
             "clamp_profile": [
                 {"at_step": 0, "offset_steps": 0},
                 {"at_step": 400, "offset_steps": 0},
                 {"at_step": 1000, "offset_steps": 120},
             ],
             "sync_clamp_profile_to_pair": True,
             "tof_mode": "unspecified"}
        )
        box_in = self.console.actions["box-in"]
        self.assertTrue(box_out["clamp_profile_enabled"])
        self.assertEqual(
            box_in["clamp_profile"],
            [
                {"at_step": 0, "offset_steps": 0},
                {"at_step": 600, "offset_steps": -120},
                {"at_step": 1000, "offset_steps": -120},
            ],
        )

    def test_conveyor_clamp_profiles_can_remain_unique(self) -> None:
        self.console.save_action(
            {"action_id": "box-in", "name": "Conveyor: box in",
             "group": "belts", "direction_a": "F", "direction_b": "R",
             "steps": 1000, "max_hz": 4000, "ramp_percent": 0,
             "clamp_profile_enabled": True,
             "clamp_profile": [
                 {"at_step": 0, "offset_steps": 0},
                 {"at_step": 500, "offset_steps": 50},
                 {"at_step": 1000, "offset_steps": 40},
             ],
             "sync_clamp_profile_to_pair": False,
             "tof_mode": "unspecified"}
        )
        self.console.save_action(
            {"action_id": "box-out", "name": "Conveyor: box out",
             "group": "belts", "direction_a": "R", "direction_b": "F",
             "steps": 2000, "max_hz": 3000, "ramp_percent": 10,
             "clamp_profile_enabled": True,
             "clamp_profile": [
                 {"at_step": 0, "offset_steps": 0},
                 {"at_step": 2000, "offset_steps": 120},
             ],
             "sync_clamp_profile_to_pair": False,
             "tof_mode": "unspecified"}
        )
        box_in = self.console.actions["box-in"]
        self.assertEqual(box_in["steps"], 2000)
        self.assertEqual(
            box_in["clamp_profile"],
            [
                {"at_step": 0, "offset_steps": 0},
                {"at_step": 1000, "offset_steps": 50},
                {"at_step": 2000, "offset_steps": 40},
            ],
        )

    def test_conveyor_clamp_profile_commands_all_four_counters(self) -> None:
        action = self.console.save_action(
            {"action_id": "box-out", "name": "Conveyor: box out",
             "group": "belts", "direction_a": "R", "direction_b": "F",
             "steps": 100, "max_hz": 4000, "ramp_percent": 0,
             "clamp_profile_enabled": True,
             "clamp_profile": [
                 {"at_step": 0, "offset_steps": 0},
                 {"at_step": 50, "offset_steps": 0},
                 {"at_step": 100, "offset_steps": 20},
             ],
             "tof_mode": "unspecified"}
        )
        self.console.run_action(action["id"], False)
        self.console._motion_thread.join(timeout=1.0)
        self.assertEqual(
            self.console.motor_steps,
            {"1": -20, "3": -100, "4": 100, "6": -20, "7": 0},
        )
        self.assertEqual(
            self.console.last_motion["clamp_offset_delta_steps"], 20
        )

    def test_conveyor_demo_follows_profile_and_returns_to_zero(self) -> None:
        base = {
            "action_id": "box-out", "name": "Conveyor: box out",
            "group": "belts", "direction_a": "R", "direction_b": "F",
            "steps": 100, "max_hz": 4000, "ramp_percent": 0,
            "clamp_profile_enabled": True,
            "clamp_profile": [
                {"at_step": 0, "offset_steps": 0},
                {"at_step": 50, "offset_steps": 0},
                {"at_step": 100, "offset_steps": 20},
            ],
            "tof_mode": "unspecified",
        }
        self.console.demo_action_target({**base, "target_steps": 50})
        self.console._motion_thread.join(timeout=1.0)
        self.assertEqual(self.console.action_demo_clamp_offsets["box-out"], 0)
        self.console.demo_action_target({**base, "target_steps": 100})
        self.console._motion_thread.join(timeout=1.0)
        self.assertEqual(self.console.action_demo_clamp_offsets["box-out"], 20)
        self.console.demo_action_target({**base, "target_steps": 0})
        self.console._motion_thread.join(timeout=1.0)
        self.assertEqual(self.console.action_demo_positions["box-out"], 0)
        self.assertEqual(self.console.action_demo_clamp_offsets["box-out"], 0)

    def test_tof_can_be_unspecified_or_not_applicable(self) -> None:
        for mode in ("unspecified", "not_applicable"):
            action = self.console.save_action(
                {"name": f"ToF {mode}", "group": "belts",
                 "direction_a": "R", "direction_b": "F", "steps": 100,
                 "max_hz": 500, "ramp_percent": 0, "tof_mode": mode}
            )
            self.assertEqual(action["tof_mode"], mode)
            self.assertIsNone(action["tof_min_mm"])

    def test_tof_range_requires_bounds_and_contains_ideal(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "minimum and maximum"):
            self.console.save_action(
                {"name": "Bad range", "group": "belts",
                 "direction_a": "R", "direction_b": "F", "steps": 100,
                 "max_hz": 500, "ramp_percent": 0, "tof_mode": "range",
                 "tof_ideal_mm": 620}
            )
        with self.assertRaisesRegex(RuntimeError, "inside"):
            self.console.save_action(
                {"name": "Bad ideal", "group": "belts",
                 "direction_a": "R", "direction_b": "F", "steps": 100,
                 "max_hz": 500, "ramp_percent": 0, "tof_mode": "range",
                 "tof_min_mm": 600, "tof_ideal_mm": 700,
                 "tof_max_mm": 640}
            )


if __name__ == "__main__":
    unittest.main()
