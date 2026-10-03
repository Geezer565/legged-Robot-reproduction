#!/usr/bin/env python3
import importlib.util
import math
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "experiment_metrics.py"
SPEC = importlib.util.spec_from_file_location("experiment_metrics", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_rows(x_errors, sync_error=0.0):
    rows = []
    for index, x_error in enumerate(x_errors):
        rows.append({
            "time": float(index),
            "position_error": [x_error, 0.0, 0.0],
            "orientation_error": [0.5 * x_error, 0.0, 0.0],
            "sigma": [2.0 * x_error, 0.0, 0.0, 0.0, 0.0, 0.0],
            "pi_hat": [3.0 * index, 0.0, 0.0, 0.0, 0.0, 0.0],
            "snapshot_time": float(index) + sync_error,
        })
    return rows


def metadata(mode):
    switches = {
        "nominal": {"enableParameterUpdate": False, "enableAdaptiveDynamics": False, "enableClfConstraint": False},
        "estimate_only": {"enableParameterUpdate": True, "enableAdaptiveDynamics": False, "enableClfConstraint": False},
        "full_adaptive": {"enableParameterUpdate": True, "enableAdaptiveDynamics": True, "enableClfConstraint": False},
    }[mode]
    return {
        "mode": mode,
        "switches": switches,
        "disturbance_start_s": 1.0,
        "disturbance_end_s": 4.0,
        "disturbance_wrench": [20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "analysis_end_s": 6.0,
        "initial_state": [0.0] * 24,
    }


class ExperimentMetricsTest(unittest.TestCase):
    def test_task_b_metrics_include_rms_iae_peak_and_recovery(self):
        rows = make_rows([0.0, 0.01, 0.02, 0.02, 0.01, 0.0, 0.0])
        result = MODULE.compute_task_b_metrics(rows, disturbance_end=4.0,
                                               recovery_band=0.005, recovery_hold=1.0)
        self.assertAlmostEqual(result["peak_position_error_m"], 0.02)
        self.assertAlmostEqual(result["position_error_iae_m_s"], 0.06)
        self.assertGreater(result["rms_position_error_m"], 0.0)
        self.assertAlmostEqual(result["recovery_time_after_disturbance_s"], 1.0)
        self.assertAlmostEqual(result["peak_sigma_norm"], 0.04)
        self.assertAlmostEqual(result["pi_fx_change"], 18.0)
        self.assertAlmostEqual(result["max_snapshot_time_error_s"], 0.0)

    def test_recovery_is_none_when_hold_window_is_not_available(self):
        rows = make_rows([0.0, 0.01, 0.02, 0.02, 0.01, 0.004])
        result = MODULE.compute_task_b_metrics(rows, disturbance_end=4.0,
                                               recovery_band=0.005, recovery_hold=2.0)
        self.assertIsNone(result["recovery_time_after_disturbance_s"])

    def test_nonfinite_or_unsynchronized_rows_are_rejected(self):
        rows = make_rows([0.0, 0.01], sync_error=2.0e-6)
        with self.assertRaisesRegex(ValueError, "synchronization"):
            MODULE.validate_rows(rows, synchronization_tolerance=1.0e-6)
        rows[0]["sigma"][0] = math.nan
        with self.assertRaisesRegex(ValueError, "finite"):
            MODULE.validate_rows(rows, synchronization_tolerance=1.0)

    def test_task_b_fairness_rejects_wrong_mode_or_disturbance(self):
        manifests = [metadata(mode) for mode in MODULE.TASK_B_MODES]
        MODULE.validate_task_b_manifests(manifests)
        wrong_mode = [dict(item) for item in manifests]
        wrong_mode[1]["switches"] = dict(wrong_mode[1]["switches"])
        wrong_mode[1]["switches"]["enableAdaptiveDynamics"] = True
        with self.assertRaisesRegex(ValueError, "switches"):
            MODULE.validate_task_b_manifests(wrong_mode)
        wrong_force = [dict(item) for item in manifests]
        wrong_force[2]["disturbance_wrench"] = [19.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        with self.assertRaisesRegex(ValueError, "disturbance"):
            MODULE.validate_task_b_manifests(wrong_force)


if __name__ == "__main__":
    unittest.main()
