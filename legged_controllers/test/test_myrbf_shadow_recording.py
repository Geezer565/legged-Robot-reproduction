#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "record_myrbf_shadow.py"
SPEC = importlib.util.spec_from_file_location("record_myrbf_shadow", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MyRbfShadowRecordingTest(unittest.TestCase):
    def test_decodes_fixed_diagnostic_layout(self):
        values = [float(index) for index in range(87)]

        row = MODULE.decode_diagnostic(values)

        self.assertEqual(row["time_s"], 0.0)
        self.assertEqual(row["dt_s"], 1.0)
        self.assertEqual(row["raw_fx_n"], 2.0)
        self.assertEqual(row["filtered_tz_nm"], 13.0)
        self.assertEqual(row["estimate_after_fx_n"], 20.0)
        self.assertEqual(row["feature_discrepancy_tz"], 37.0)
        self.assertEqual(row["tracking_sigma_wz"], 43.0)
        self.assertEqual(row["actual_base_y_m"], 48.0)
        self.assertEqual(row["desired_base_z_m"], 55.0)
        self.assertEqual(row["applied_control_fy_n"], 56.0)
        self.assertEqual(row["control_injection_enabled"], 57.0)
        self.assertEqual(row["valid_sample_count"], 58.0)
        self.assertEqual(row["rejected_sample_count"], 59.0)
        self.assertEqual(row["net_residual_fy_n"], 61.0)
        self.assertEqual(row["requested_control_tz_nm"], 71.0)
        self.assertEqual(row["achieved_control_fy_n"], 73.0)
        self.assertEqual(row["allocator_scale"], 78.0)
        self.assertEqual(row["control_interface_mode"], 79.0)
        self.assertEqual(row["coverage_fx"], 80.0)
        self.assertEqual(row["coverage_tz"], 85.0)
        self.assertEqual(row["update_frozen"], 86.0)
        self.assertEqual(tuple(row), MODULE.FIELDS)
        self.assertTrue(all(math.isfinite(value) for value in row.values()))

    def test_rejects_truncated_diagnostic_message(self):
        with self.assertRaises(ValueError):
            MODULE.decode_diagnostic([0.0] * 86)


if __name__ == "__main__":
    unittest.main()
