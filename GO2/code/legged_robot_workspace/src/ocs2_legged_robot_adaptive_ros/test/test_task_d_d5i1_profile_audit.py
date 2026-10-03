#!/usr/bin/env python3

import importlib.util
import pathlib
import unittest

import numpy as np

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "analyze_task_d_d5i1_profile_audit.py"
SPEC = importlib.util.spec_from_file_location("d5i1", SCRIPT)
D5I1 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D5I1)


class D5I1ProfileAuditTest(unittest.TestCase):
    def test_dimensionless_design_has_expected_shape_and_scaling(self):
        regressor = np.zeros((2, 6, 10))
        regressor[:, 0, 0] = 1.0
        regressor[:, 3, 9] = 1.0
        design, d_pi = D5I1.dimensionless_design(regressor, 0.25)
        self.assertEqual(design.shape, (12, 10))
        self.assertAlmostEqual(design[0, 0], 0.5 * D5I1.MASS)
        self.assertAlmostEqual(design[3, 9], 0.5 * D5I1.MASS * D5I1.LENGTH)
        self.assertAlmostEqual(d_pi[9, 9], D5I1.MASS * D5I1.LENGTH ** 2)

    def test_effective_rank(self):
        self.assertAlmostEqual(D5I1.effective_rank(np.ones(10)), 10.0)
        self.assertAlmostEqual(D5I1.effective_rank(np.array([1.0] + [0.0] * 9)), 1.0)


if __name__ == "__main__":
    unittest.main()
