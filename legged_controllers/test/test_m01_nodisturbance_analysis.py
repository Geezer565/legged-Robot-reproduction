#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_m01_nodisturbance.py"
SPEC = importlib.util.spec_from_file_location("analyze_m01_nodisturbance", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def shadow_row(time_s, actual_x, desired_x, applied_force=0.0):
    row = {
        "time_s": str(time_s),
        "actual_base_x_m": str(actual_x),
        "actual_base_y_m": "0.01",
        "actual_base_z_m": "0.34",
        "actual_base_yaw_rad": "0.0",
        "actual_base_pitch_rad": "0.01",
        "actual_base_roll_rad": "0.02",
        "desired_base_x_m": str(desired_x),
        "desired_base_y_m": "0.0",
        "desired_base_z_m": "0.35",
        "desired_base_yaw_rad": "0.0",
        "desired_base_pitch_rad": "0.0",
        "desired_base_roll_rad": "0.0",
    }
    for suffix in MODULE.FORCE_FIELDS:
        row["estimate_after_" + suffix] = "0.0"
        row["achieved_control_" + suffix] = "0.0"
    for suffix in MODULE.TORQUE_FIELDS:
        row["estimate_after_" + suffix] = "0.0"
        row["achieved_control_" + suffix] = "0.0"
    row["achieved_control_fx_n"] = str(applied_force)
    return row


class M01AnalysisTest(unittest.TestCase):
    def test_active_motion_window(self):
        rows = [
            {"active": "0", "ros_time": "1.0", "wall_elapsed": "1.0", "vx": "0.0"},
            {"active": "1", "ros_time": "2.0", "wall_elapsed": "2.0", "vx": "0.1"},
            {"active": "1", "ros_time": "3.0", "wall_elapsed": "3.0", "vx": "0.1"},
        ]
        onset, end, active = MODULE.active_motion_window(rows)
        self.assertEqual((onset, end), (2.0, 3.0))
        self.assertEqual(len(active), 2)

    def test_case_metrics_use_walking_window_and_target_error(self):
        motion = [
            {"active": "1", "ros_time": "2.0", "wall_elapsed": "2.0", "vx": "0.1"},
            {"active": "1", "ros_time": "3.0", "wall_elapsed": "3.0", "vx": "0.1"},
        ]
        shadow = [
            shadow_row(1.0, 0.0, 0.0),
            shadow_row(2.0, 0.09, 0.10, 1.0),
            shadow_row(3.0, 0.18, 0.20, 1.0),
        ]
        state = [
            {"ros_time": "2.0", "base_x": "0.09", "base_z": "0.34", "observation_mode": "15"},
            {"ros_time": "3.0", "base_x": "0.18", "base_z": "0.33", "observation_mode": "15"},
        ]
        result, series = MODULE.analyze_case_rows("MyRBF", 0.1, shadow, motion, state)
        self.assertEqual(result["sample_count"], 2)
        self.assertAlmostEqual(result["forward_rmse_cm"], (2.5) ** 0.5)
        self.assertAlmostEqual(result["applied_force_rms_n"], 1.0)
        self.assertAlmostEqual(result["minimum_base_z_m"], 0.33)
        self.assertAlmostEqual(result["motion_wall_duration_s"], 1.0)
        self.assertEqual(result["end_mode"], 15)
        self.assertEqual(series["time_s"], [0.0, 1.0])

    def test_assessment_detects_tracking_degradation(self):
        results = []
        for speed in (0.05, 0.10, 0.15):
            for method, position in (("Baseline", 1.0), ("ETHRBF", 1.0), ("MyRBF", 2.0)):
                results.append({
                    "name": method,
                    "speed_mps": speed,
                    "position_rmse_cm": position,
                    "orientation_rmse_deg": 1.0,
                    "applied_force_rms_n": 0.0,
                    "applied_torque_rms_nm": 0.0,
                    "minimum_base_z_m": 0.33,
                    "end_mode": 15,
                })
        self.assertFalse(MODULE.assess(results)["pass"])


if __name__ == "__main__":
    unittest.main()
