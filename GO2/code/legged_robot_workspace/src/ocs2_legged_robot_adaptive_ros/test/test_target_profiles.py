#!/usr/bin/env python3
import importlib.util
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "target_profiles.py"
SPEC = importlib.util.spec_from_file_location("target_profiles", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TargetProfilesTest(unittest.TestCase):
    def test_quintic_has_zero_endpoint_velocity_and_acceleration(self):
        for tau, expected_position in ((0.0, 0.0), (1.0, 1.0)):
            position, velocity, acceleration = MODULE.quintic_state(tau, 1.0)
            self.assertAlmostEqual(position, expected_position, places=14)
            self.assertAlmostEqual(velocity, 0.0, places=14)
            self.assertAlmostEqual(acceleration, 0.0, places=14)

    def test_quintic_samples_are_monotone_and_include_exact_endpoints(self):
        samples = MODULE.sample_profile("quintic", duration=1.0, sample_period=0.03)
        times = [sample[0] for sample in samples]
        scales = [sample[1] for sample in samples]
        self.assertEqual((times[0], scales[0]), (0.0, 0.0))
        self.assertEqual((times[-1], scales[-1]), (1.0, 1.0))
        self.assertTrue(all(right > left for left, right in zip(times, times[1:])))
        self.assertTrue(all(right >= left for left, right in zip(scales, scales[1:])))
        self.assertTrue(all(0.0 <= value <= 1.0 for value in scales))

    def test_linear_profile_keeps_native_two_knot_semantics(self):
        self.assertEqual(MODULE.sample_profile("linear", 0.4, 0.02), [(0.0, 0.0), (0.4, 1.0)])

    def test_observation_time_gate_uses_simulation_time(self):
        self.assertFalse(MODULE.has_reached_observation_time(1.98, 2.0))
        self.assertTrue(MODULE.has_reached_observation_time(2.0, 2.0))
        self.assertTrue(MODULE.has_reached_observation_time(2.02, 2.0))
        with self.assertRaises(ValueError):
            MODULE.has_reached_observation_time(float("nan"), 2.0)

    def test_invalid_profile_or_timing_is_rejected(self):
        with self.assertRaises(ValueError):
            MODULE.sample_profile("cubic", 1.0, 0.02)
        with self.assertRaises(ValueError):
            MODULE.sample_profile("quintic", 0.0, 0.02)
        with self.assertRaises(ValueError):
            MODULE.sample_profile("quintic", 1.0, 0.0)


if __name__ == "__main__":
    unittest.main()
