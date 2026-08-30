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

    def test_multisine_profile_combines_two_frequencies(self):
        value = MODULE.profile_value(
            1.25, "multisine", 1.0, 3.0, 2.0, 4.0, 1.0,
            secondary_amplitude=3.0, secondary_frequency=2.0)
        self.assertAlmostEqual(value, 2.0 + 4.0 - 1.5, places=10)

    def test_irregular_profile_is_repeatable_and_changes_sign(self):
        samples = [
            MODULE.profile_value(
                1.0 + offset, "irregular", 1.0, 4.0, 1.0, 10.0, 1.0)
            for offset in (0.1, 0.45, 0.95, 1.15, 1.90)
        ]
        self.assertEqual(samples, [1.0, 11.0, -3.5, 8.5, -9.0])
        repeated = MODULE.profile_value(
            1.0 + 3.90, "irregular", 1.0, 4.0, 1.0, 10.0, 1.0)
        self.assertEqual(repeated, 11.0)

    def test_rejects_invalid_profile_arguments(self):
        with self.assertRaises(ValueError):
            MODULE.validate_profile("triangle", 1.0, 3.0, 20.0)
        with self.assertRaises(ValueError):
            MODULE.validate_profile("constant", -1.0, 3.0, 20.0)
        with self.assertRaises(ValueError):
            MODULE.validate_profile("sine", 1.0, 0.0, 20.0)
        with self.assertRaises(ValueError):
            MODULE.validate_profile("sine", 1.0, 3.0, math.nan)

    def test_rejects_invalid_multisine_frequency(self):
        with self.assertRaises(ValueError):
            MODULE.profile_value(
                1.5, "multisine", 1.0, 3.0, 0.0, 5.0, 0.5,
                secondary_amplitude=2.0, secondary_frequency=0.0)


if __name__ == "__main__":
    unittest.main()
