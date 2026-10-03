#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest

import numpy as np


SCRIPT = (Path(__file__).resolve().parents[1] / "scripts" /
          "analyze_task_d_d5h2_startup_gate.py")
SPEC = importlib.util.spec_from_file_location("d5h2_startup_gate", str(SCRIPT))
ANALYSIS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYSIS)


class D5H2StartupGateTest(unittest.TestCase):

    def test_freezes_before_active_and_integrates_active_derivative(self):
        time = np.asarray([0.0, 0.1, 0.3, 0.6])
        derivative = np.ones((4, 10))
        active = np.asarray([False, False, True, True])
        estimate, gated_derivative = ANALYSIS.replay(time, derivative, active)
        np.testing.assert_allclose(estimate[0], 0.0)
        np.testing.assert_allclose(estimate[1], 0.0)
        np.testing.assert_allclose(estimate[2], 0.2)
        np.testing.assert_allclose(estimate[3], 0.5)
        np.testing.assert_allclose(gated_derivative[:2], 0.0)
        np.testing.assert_allclose(gated_derivative[2:], 1.0)

    def test_uses_right_endpoint_explicit_euler_convention(self):
        time = np.asarray([0.0, 0.2, 0.5])
        derivative = np.zeros((3, 10))
        derivative[1, 0] = 2.0
        derivative[2, 0] = 3.0
        estimate, _ = ANALYSIS.replay(
            time, derivative, np.asarray([False, True, True]))
        self.assertAlmostEqual(estimate[-1, 0], 0.2 * 2.0 + 0.3 * 3.0)

    def test_invalid_shapes_and_timestamps_are_rejected(self):
        with self.assertRaises(ValueError):
            ANALYSIS.replay([0.0, 0.1], np.zeros((2, 9)), [False, True])
        with self.assertRaises(ValueError):
            ANALYSIS.replay([0.0, 0.0], np.zeros((2, 10)), [False, True])

    def test_cycle_decomposition_keeps_boundary_bin_in_old_cycle(self):
        run = {
            "time": np.asarray([0.0, 0.1, 0.2, 0.3, 0.4]),
            "active": np.asarray([False, True, True, True, True]),
            "phase_bin": np.asarray([-1, 0, 23, 0, 1]),
            "gated_derivative": np.ones((5, 10)),
        }
        cycles = ANALYSIS.cycle_increments(run, np.ones(10))
        self.assertEqual(len(cycles), 2)
        self.assertTrue(cycles[0]["complete"])
        self.assertFalse(cycles[1]["complete"])
        np.testing.assert_allclose(cycles[0]["increment"], 0.2)
        np.testing.assert_allclose(cycles[1]["increment"], 0.2)


if __name__ == "__main__":
    unittest.main()
