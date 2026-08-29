#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "apply_myrbf_wrench.py"
SPEC = importlib.util.spec_from_file_location("apply_myrbf_wrench", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MyRbfWrenchProfileTest(unittest.TestCase):
    def test_profile_active_window_is_explicit(self):
        self.assertFalse(MODULE.profile_is_active(0.99, 1.0, 3.0))
        self.assertTrue(MODULE.profile_is_active(1.0, 1.0, 3.0))
        self.assertTrue(MODULE.profile_is_active(3.99, 1.0, 3.0))
        self.assertFalse(MODULE.profile_is_active(4.0, 1.0, 3.0))

    def test_constant_profile_has_clean_delay_and_stop(self):
        self.assertEqual(MODULE.profile_value(0.9, "constant", 1.0, 3.0, 20.0, 0.0, 0.5), 0.0)
        self.assertEqual(MODULE.profile_value(1.0, "constant", 1.0, 3.0, 20.0, 0.0, 0.5), 20.0)
        self.assertEqual(MODULE.profile_value(3.99, "constant", 1.0, 3.0, 20.0, 0.0, 0.5), 20.0)
        self.assertEqual(MODULE.profile_value(4.0, "constant", 1.0, 3.0, 20.0, 0.0, 0.5), 0.0)

    def test_sine_profile_uses_requested_offset_amplitude_and_frequency(self):
        value = MODULE.profile_value(1.25, "sine", 1.0, 3.0, 10.0, 6.0, 1.0)
        self.assertAlmostEqual(value, 16.0, places=10)
        value = MODULE.profile_value(1.75, "sine", 1.0, 3.0, 10.0, 6.0, 1.0)
        self.assertAlmostEqual(value, 4.0, places=10)

    def test_rejects_invalid_profile_arguments(self):
        with self.assertRaises(ValueError):
            MODULE.validate_profile("triangle", 1.0, 3.0, 20.0)
        with self.assertRaises(ValueError):
            MODULE.validate_profile("constant", -1.0, 3.0, 20.0)
        with self.assertRaises(ValueError):
            MODULE.validate_profile("sine", 1.0, 0.0, 20.0)
        with self.assertRaises(ValueError):
            MODULE.validate_profile("sine", 1.0, 3.0, math.nan)


if __name__ == "__main__":
    unittest.main()
