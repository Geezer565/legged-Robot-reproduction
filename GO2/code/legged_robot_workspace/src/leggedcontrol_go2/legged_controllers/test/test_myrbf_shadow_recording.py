#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import sys
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "record_myrbf_shadow.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("record_myrbf_shadow", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MyRbfShadowRecordingTest(unittest.TestCase):
    def test_decodes_appended_update_diagnostics_after_legacy_layout(self):
        values = [float(index) for index in range(124)]

        row = MODULE.decode_diagnostic(values)

        self.assertEqual(row["time_s"], 0.0)
        self.assertEqual(row["dt_s"], 1.0)
        self.assertEqual(row["raw_fx_n"], 2.0)
        self.assertEqual(row["filtered_tz_nm"], 13.0)
        self.assertEqual(row["estimate_after_fx_n"], 20.0)
        self.assertEqual(row["feature_residual_tz"], 31.0)
        self.assertEqual(row["tracking_sigma_wz"], 37.0)
        self.assertEqual(row["actual_base_y_m"], 42.0)
        self.assertEqual(row["desired_base_z_m"], 49.0)
        self.assertEqual(row["applied_control_fy_n"], 50.0)
        self.assertEqual(row["control_injection_enabled"], 51.0)
        self.assertEqual(row["valid_sample_count"], 52.0)
        self.assertEqual(row["rejected_sample_count"], 53.0)
        self.assertEqual(row["net_residual_fy_n"], 55.0)
        self.assertEqual(row["requested_control_tz_nm"], 65.0)
        self.assertEqual(row["achieved_control_fy_n"], 67.0)
        self.assertEqual(row["allocator_scale"], 72.0)
        self.assertEqual(row["control_interface_mode"], 73.0)
        self.assertEqual(row["coverage_fx"], 74.0)
        self.assertEqual(row["coverage_tz"], 79.0)
        self.assertEqual(row["update_frozen"], 80.0)
        self.assertEqual(row["prediction_derivative_norm_fx"], 81.0)
        self.assertEqual(row["prediction_derivative_norm_tz"], 86.0)
        self.assertEqual(row["tracking_bias_derivative_fx"], 87.0)
        self.assertEqual(row["tracking_bias_derivative_tz"], 92.0)
        self.assertEqual(row["leakage_derivative_norm_fx"], 93.0)
        self.assertEqual(row["leakage_derivative_norm_tz"], 98.0)
        self.assertEqual(row["projection_active_fx"], 99.0)
        self.assertEqual(row["projection_active_tz"], 104.0)
        self.assertEqual(row["algorithm_mode"], 105.0)
        self.assertEqual(row["v2_robust_wrench_fx"], 106.0)
        self.assertEqual(row["v2_robust_bound_tz"], 117.0)
        self.assertEqual(row["v2_smoothing_width_tz"], 123.0)
        self.assertEqual(len(MODULE.FIELDS), 124)
        self.assertEqual(
            MODULE.FIELDS[81:105],
            (
                "prediction_derivative_norm_fx",
                "prediction_derivative_norm_fy",
                "prediction_derivative_norm_fz",
                "prediction_derivative_norm_tx",
                "prediction_derivative_norm_ty",
                "prediction_derivative_norm_tz",
                "tracking_bias_derivative_fx",
                "tracking_bias_derivative_fy",
                "tracking_bias_derivative_fz",
                "tracking_bias_derivative_tx",
                "tracking_bias_derivative_ty",
                "tracking_bias_derivative_tz",
                "leakage_derivative_norm_fx",
                "leakage_derivative_norm_fy",
                "leakage_derivative_norm_fz",
                "leakage_derivative_norm_tx",
                "leakage_derivative_norm_ty",
                "leakage_derivative_norm_tz",
                "projection_active_fx",
                "projection_active_fy",
                "projection_active_fz",
                "projection_active_tx",
                "projection_active_ty",
                "projection_active_tz",
            ),
        )
        self.assertEqual(tuple(row), MODULE.FIELDS)
        self.assertTrue(all(math.isfinite(value) for value in row.values()))

    def test_rejects_truncated_diagnostic_message(self):
        with self.assertRaises(ValueError):
            MODULE.decode_diagnostic([0.0] * 123)


if __name__ == "__main__":
    unittest.main()
