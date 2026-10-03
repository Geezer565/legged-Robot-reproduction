#!/usr/bin/env python3
import importlib.util
import os
import unittest

SCRIPT = os.path.join(os.path.dirname(__file__), "..", "scripts", "analyze_adaptive_update.py")
SPEC = importlib.util.spec_from_file_location("analyze_adaptive_update", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class AnalyzeAdaptiveUpdateTest(unittest.TestCase):
    def test_exact_gamma_update_and_positive_direction(self):
        updates = [
            [0.0, 0.0, 0.0, 1.0] + [0.0] * 6 + [0.0] * 6,
            [1.0, 1.0, 0.0, 2.0] + [0.3, 0, 0, 0, 0, 0] + [0.1, 0, 0, 0, 0, 0],
            [2.0, 2.0, 0.0, 3.0] + [0.6, 0, 0, 0, 0, 0] + [0.1, 0, 0, 0, 0, 0],
        ]
        disturbances = [
            [0.0, 0.0] + [0.0] * 6,
            [1.0, 1.0, 5.0, 0, 0, 0, 0, 0],
            [2.0, 1.0, 5.0, 0, 0, 0, 0, 0],
        ]
        rows, summary = MODULE.analyze_samples(updates, disturbances)
        self.assertEqual(len(rows), 3)
        self.assertTrue(summary["version_strictly_increasing"])
        self.assertLess(summary["max_gamma_update_error"], 1e-12)
        self.assertEqual(summary["dominant_component"], "Fx")
        self.assertTrue(summary["estimate_direction_matches_disturbance"])

    def test_drops_only_unpaired_boundary_update(self):
        updates = [
            [0.0, 0.0, 0.0, 1.0] + [0.0] * 6 + [0.0] * 6,
            [1.0, 1.0, 0.0, 2.0] + [0.3, 0, 0, 0, 0, 0] + [0.1, 0, 0, 0, 0, 0],
            [2.0, 2.0, 0.0, 3.0] + [0.6, 0, 0, 0, 0, 0] + [0.1, 0, 0, 0, 0, 0],
        ]
        disturbances = [
            [0.1, 0.0] + [0.0] * 6,
            [1.0, 1.0, 5.0, 0, 0, 0, 0, 0],
            [2.0, 1.0, 5.0, 0, 0, 0, 0, 0],
        ]
        rows, summary = MODULE.analyze_samples(updates, disturbances)
        self.assertEqual(len(rows), 2)
        self.assertEqual(summary["dropped_boundary_sample_count"], 1)
    def test_rejects_bad_message_size(self):
        with self.assertRaises(ValueError):
            MODULE.analyze_samples([[0.0]], [[0.0] * 8])


if __name__ == "__main__":
    unittest.main()