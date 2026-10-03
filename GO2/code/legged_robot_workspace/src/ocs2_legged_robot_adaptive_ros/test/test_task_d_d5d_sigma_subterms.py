#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_task_d_d5d_sigma_subterms.py"
SPEC = importlib.util.spec_from_file_location("task_d_d5d", str(SCRIPT))
D5D = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D5D)


class TaskDD5DSigmaSubtermTest(unittest.TestCase):
    def test_right_endpoint_euler_matches_logged_update(self):
        time = np.asarray([0.0, 0.1, 0.25])
        rate = np.asarray([[50.0, 50.0], [2.0, -3.0], [4.0, 5.0]])
        delta, cumulative = D5D.euler_contributions(time, rate)
        expected = np.asarray([[0.0, 0.0], [0.2, -0.3], [0.6, 0.75]])
        np.testing.assert_allclose(delta, expected)
        np.testing.assert_allclose(cumulative[-1], [0.8, 0.45])

    def test_subterm_sum_is_exact_for_linear_update_map(self):
        rng = np.random.RandomState(13)
        gamma = np.diag(np.linspace(0.5, 1.4, 10))
        regressor = rng.randn(6, 10)
        velocity = rng.randn(6)
        pose = rng.randn(6)
        velocity_rate = gamma.dot(regressor.T).dot(velocity)
        pose_rate = gamma.dot(regressor.T).dot(pose)
        total_rate = gamma.dot(regressor.T).dot(velocity + pose)
        np.testing.assert_allclose(velocity_rate + pose_rate, total_rate,
                                   rtol=0.0, atol=1.0e-13)

    def test_cross_topic_matching_rejects_unsynchronized_samples(self):
        primary = np.zeros((2, 52)); primary[:, 0] = [0.1, 0.2]
        secondary = np.zeros((2, 118)); secondary[:, 0] = [0.1, 0.2001]
        with self.assertRaises(ValueError):
            D5D.match_rows_by_time(primary, secondary)

    def test_runtime_schema_is_versioned_and_fixed_width(self):
        source = (Path(__file__).resolve().parents[1] / "src" /
                  "LeggedRobotAdaptiveDdpMpcNode.cpp").read_text()
        self.assertIn("payload_inertial_sigma_decomposition_v1_52_fields", source)
        self.assertIn("sigmaDecompositionSample.data.reserve(52)", source)


if __name__ == "__main__":
    unittest.main()
