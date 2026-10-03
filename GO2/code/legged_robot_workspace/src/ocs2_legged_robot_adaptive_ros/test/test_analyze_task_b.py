#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from analyze_task_b import common_start_state_difference


class CommonStartFairnessTest(unittest.TestCase):
    def test_compares_same_simulation_time_when_first_recorded_frame_differs(self):
        runs = {
            "nominal": [(0.0, [0.0, 0.0]), (0.1, [1.0, 2.0])],
            "estimate_only": [(0.1, [1.0 + 2.0e-7, 2.0])],
            "full_adaptive": [(0.0, [0.0, 0.0]), (0.1, [1.0, 2.0 - 1.0e-7])],
        }
        start, difference, states = common_start_state_difference(runs)
        self.assertAlmostEqual(start, 0.1)
        self.assertAlmostEqual(difference, 2.0e-7)
        self.assertEqual(states["nominal"], [1.0, 2.0])

    def test_rejects_missing_sample_at_common_start(self):
        runs = {
            "nominal": [(0.0, [0.0]), (0.2, [2.0])],
            "estimate_only": [(0.1, [1.0])],
            "full_adaptive": [(0.1, [1.0])],
        }
        with self.assertRaises(ValueError):
            common_start_state_difference(runs, tolerance=1.0e-6)


if __name__ == "__main__":
    unittest.main()
