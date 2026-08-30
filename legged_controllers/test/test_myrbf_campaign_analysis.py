#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_myrbf_campaign.py"
SPEC = importlib.util.spec_from_file_location("analyze_myrbf_campaign", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MyRbfCampaignAnalysisTest(unittest.TestCase):
    def test_nearest_values_skip_duplicate_startup_timestamps(self):
        truth_rows = [
            {"ros_time": "0.0", "fy": "0.0"},
            {"ros_time": "0.0", "fy": "0.0"},
            {"ros_time": "1.0", "fy": "2.0"},
            {"ros_time": "2.0", "fy": "4.0"},
        ]
        self.assertEqual(
            MODULE.nearest_truth_values([0.9, 1.9], truth_rows, "fy"),
            [2.0, 4.0],
        )

    def test_axis_spec_maps_force_and_torque_units(self):
        self.assertEqual(
            MODULE.axis_spec("fy"),
            ("estimate_after_fy_n", "achieved_control_fy_n", "N"),
        )
        self.assertEqual(
            MODULE.axis_spec("tx"),
            ("estimate_after_tx_nm", "achieved_control_tx_nm", "N·m"),
        )

    def test_correlation_handles_exact_and_constant_series(self):
        self.assertAlmostEqual(MODULE.correlation([1.0, 2.0], [2.0, 4.0]), 1.0)
        self.assertEqual(MODULE.correlation([1.0, 1.0], [2.0, 3.0]), 0.0)

    def test_body_series_uses_axis_relevant_motion(self):
        rows = [{
            "actual_base_y_m": "0.012",
            "actual_base_z_m": "0.34",
            "desired_base_z_m": "0.35",
            "actual_base_roll_rad": "0.1",
        }]
        values, label = MODULE.body_series(rows, "fy")
        self.assertEqual(values, [1.2])
        self.assertEqual(label, "Base lateral position (cm)")
        values, label = MODULE.body_series(rows, "fz")
        self.assertAlmostEqual(values[0], -1.0)
        self.assertEqual(label, "Base height error (cm)")

    def test_case_analysis_uses_compensation_sign_and_active_window(self):
        wrench = [
            {"ros_time": "0.0", "fy": "0.0"},
            {"ros_time": "1.0", "fy": "2.0"},
            {"ros_time": "2.0", "fy": "4.0"},
            {"ros_time": "3.0", "fy": "0.0"},
        ]
        shadow = [
            {"time_s": "1.0", "estimate_after_fy_n": "-2.0",
             "achieved_control_fy_n": "-1.0", "actual_base_y_m": "0.01",
             "actual_base_z_m": "0.34", "desired_base_z_m": "0.35",
             "actual_base_roll_rad": "0.1", "allocator_scale": "1.0"},
            {"time_s": "2.0", "estimate_after_fy_n": "-4.0",
             "achieved_control_fy_n": "-2.0", "actual_base_y_m": "0.02",
             "actual_base_z_m": "0.34", "desired_base_z_m": "0.35",
             "actual_base_roll_rad": "0.2", "allocator_scale": "1.0"},
        ]
        result, series = MODULE.analyze_case_rows("method", shadow, wrench, "fy")
        self.assertAlmostEqual(result["estimate_rmse"], 0.0)
        self.assertAlmostEqual(result["estimate_correlation"], 1.0)
        self.assertAlmostEqual(result["body_max_abs"], 2.0)
        self.assertEqual(series["target"], [-2.0, -4.0])


if __name__ == "__main__":
    unittest.main()
