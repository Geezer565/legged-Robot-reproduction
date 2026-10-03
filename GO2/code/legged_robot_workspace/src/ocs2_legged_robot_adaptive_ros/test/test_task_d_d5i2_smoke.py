#!/usr/bin/env python3

import importlib.util
import pathlib
import unittest

import numpy as np


PACKAGE = pathlib.Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MODE = load("d5i2_mode", PACKAGE / "scripts" / "set_task_d_d5i2_mode.py")
ANALYSIS = load("d5i2_analysis",
                PACKAGE / "scripts" / "analyze_task_d_d5i2_smoke.py")


class D5I2SmokeTest(unittest.TestCase):
    def test_mode_is_nominal_estimator_off_and_default_branch_explicit(self):
        text = MODE.materialize(MODE.BASE_CONFIG.read_text())
        config = MODE.effective_config(text, "E3_combined")
        self.assertFalse(config["enableParameterUpdate"])
        self.assertFalse(config["enableAdaptiveDynamics"])
        self.assertTrue(config["enableEncodedReferenceDerivatives"])
        self.assertFalse(config["truePlantPayloadInertialEnabled"])
        self.assertEqual(text.count("enableEncodedReferenceDerivatives true"), 1)
        self.assertEqual(config["schedule"]["complete_time_s"], 20.4)

    def test_dimensionless_design_and_full_rank_metric(self):
        # Six rows per sample permit a deterministic ten-column full-rank
        # synthetic design without any ROS or plant execution.
        regressor = np.zeros((2, 6, 10))
        for column in range(10):
            sample, row = divmod(column, 6)
            regressor[sample, row, column] = 1.0
        time = np.array([0.0, 1.0])
        design, scaling = ANALYSIS.dimensionless_design(
            regressor, ANALYSIS.trapezoid_weights(time))
        self.assertEqual(design.shape, (12, 10))
        self.assertEqual(scaling.shape, (10, 10))
        metrics = ANALYSIS.information_metrics(time, regressor)
        self.assertEqual(metrics["rank_at_1e_3"], 10)
        self.assertGreater(metrics["smallest_ratio"], 0.0)


if __name__ == "__main__":
    unittest.main()
