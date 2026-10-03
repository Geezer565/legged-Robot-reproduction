#!/usr/bin/env python3
import importlib.util
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "experiment_metrics.py"
SPEC = importlib.util.spec_from_file_location("experiment_metrics", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def rows(sigma_values, h_values, violation_values):
    result = []
    for index, (sigma, h_value, violation) in enumerate(zip(sigma_values, h_values, violation_values)):
        result.append({
            "time": float(index),
            "position_error": [0.001 * sigma, 0.0, 0.0],
            "orientation_error": [0.0005 * sigma, 0.0, 0.0],
            "sigma": [sigma, 0.0, 0.0, 0.0, 0.0, 0.0],
            "pi_hat": [float(index), 0.0, 0.0, 0.0, 0.0, 0.0],
            "snapshot_time": float(index),
            "h_clf": h_value,
            "violation": violation,
        })
    return result


def manifest(mode):
    profile, clf = mode.split("_")
    return {
        "mode": mode,
        "target_profile": profile,
        "switches": {
            "enableParameterUpdate": True,
            "enableAdaptiveDynamics": True,
            "enableClfConstraint": clf == "on",
        },
        "step_m": 0.10,
        "target_duration_s": 1.0,
        "sample_period_s": 0.02,
        "settle_before_target_s": 2.0,
        "analysis_end_s": 8.0,
        "disturbance_wrench": [0.0] * 6,
        "initial_state": [0.0] * 24,
    }


class TaskCMetricsTest(unittest.TestCase):
    def test_node_windows_and_clf_metrics_are_time_based(self):
        data = rows(
            [0.0, 0.0, 4.0, 1.0, 3.0, 0.0, 0.0],
            [1.0, 1.0, -2.0, 0.5, -1.0, 1.0, 1.0],
            [0.0, 0.0, 2.0, 0.0, 1.0, 0.0, 0.0],
        )
        result = MODULE.compute_task_c_metrics(
            data, target_start=2.0, target_end=3.0, node_window=1.0,
            recovery_band=0.0015, recovery_hold=1.0,
        )
        self.assertAlmostEqual(result["start_node_peak_sigma"], 4.0)
        self.assertAlmostEqual(result["end_node_peak_sigma"], 3.0)
        self.assertAlmostEqual(result["min_h_clf"], -2.0)
        self.assertAlmostEqual(result["violation_integral"], 3.0)
        self.assertAlmostEqual(result["recovery_time_after_target_s"], 2.0)

    def test_task_c_fairness_checks_profile_clf_step_and_duration(self):
        manifests = [manifest(mode) for mode in MODULE.TASK_C_MODES]
        MODULE.validate_task_c_manifests(manifests)
        wrong_clf = [dict(item) for item in manifests]
        wrong_clf[1]["switches"] = dict(wrong_clf[1]["switches"])
        wrong_clf[1]["switches"]["enableClfConstraint"] = False
        with self.assertRaisesRegex(ValueError, "switches"):
            MODULE.validate_task_c_manifests(wrong_clf)
        wrong_duration = [dict(item) for item in manifests]
        wrong_duration[3]["target_duration_s"] = 0.8
        with self.assertRaisesRegex(ValueError, "target_duration"):
            MODULE.validate_task_c_manifests(wrong_duration)


if __name__ == "__main__":
    unittest.main()
