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


MODE = load("d5i3_mode", PACKAGE / "scripts" / "set_task_d_d5i3_mode.py")
ANALYSIS = load("d5i3_analysis",
                PACKAGE / "scripts" / "analyze_task_d_d5i3_campaign.py")


def sample(drive, profile=1, gait=2, mode=13, dt=0.1, supercycle=0):
    return {
        "time": 1.0, "dt": dt, "drive": np.asarray(drive, dtype=float),
        "sigma": np.zeros(6), "profile_phase": 0.01,
        "profile_bin": profile, "gait_phase": 0.1, "gait_bin": gait,
        "mode": mode, "cycle_index": 0,
        "supercycle_index": supercycle,
    }


class D5I3CalibrationTest(unittest.TestCase):
    def test_frozen_mode_is_e3_nominal_estimator_off_two_supercycles(self):
        text = MODE.materialize(MODE.BASE_CONFIG.read_text())
        calibration = MODE.effective_config(text, "calibration", 1)
        heldout = MODE.effective_config(text, "heldout", 1)
        for config in (calibration, heldout):
            self.assertEqual(config["candidate"], "E3_combined")
            self.assertFalse(config["enableParameterUpdate"])
            self.assertFalse(config["enableAdaptiveDynamics"])
            self.assertFalse(config["truePlantPayloadInertialEnabled"])
            self.assertEqual(config["schedule"]["steady_duration_s"], 24.0)
            self.assertEqual(config["schedule"]["steady_supercycles"], 2)
        self.assertTrue(calibration["baseline_training_eligible"])
        self.assertTrue(heldout["heldout_excluded_from_training"])
        self.assertEqual(calibration["adaptive_config_sha256"],
                         heldout["adaptive_config_sha256"])

    def test_duration_weighted_template_and_heldout_correction(self):
        first = np.arange(10, dtype=float)
        second = 2.0 * first
        runs = [
            {"role": "calibration", "samples": [sample(first, dt=0.25)]},
            {"role": "calibration", "samples": [sample(second, dt=0.75)]},
        ]
        template = ANALYSIS.build_template(runs)
        expected = 0.25 * first + 0.75 * second
        np.testing.assert_allclose(template["values"][(1, 2, 13)], expected)
        heldout = {"role": "heldout", "samples": [sample(expected, dt=1.0)]}
        result = ANALYSIS.evaluate_template(heldout, template, np.ones(10))
        self.assertEqual(result["lookup_duration_coverage"], 1.0)
        self.assertAlmostEqual(result["corrected_terminal_scaled_norm"], 0.0)

    def test_heldout_cannot_enter_template(self):
        with self.assertRaisesRegex(ValueError, "heldout"):
            ANALYSIS.build_template([
                {"role": "heldout", "samples": [sample(np.ones(10))]}])

    def test_unsupported_composite_key_fails_closed_to_zero_baseline(self):
        template = ANALYSIS.build_template([
            {"role": "calibration", "samples": [sample(np.ones(10))]}])
        run = {"role": "heldout", "samples": [
            sample(2.0 * np.ones(10), profile=7, gait=8, mode=11)]}
        result = ANALYSIS.evaluate_template(run, template, np.ones(10))
        self.assertEqual(result["lookup_duration_coverage"], 0.0)
        np.testing.assert_allclose(result["raw_terminal"],
                                   result["corrected_terminal"])


if __name__ == "__main__":
    unittest.main()
