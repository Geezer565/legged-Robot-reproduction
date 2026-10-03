#!/usr/bin/env python3
import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np


SCRIPT = (Path(__file__).resolve().parents[1] / "scripts" /
          "analyze_adaptive_method_comparison.py")
SPEC = importlib.util.spec_from_file_location("adaptive_method_comparison", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def update_series(active_level, post_level, estimate_gain):
    time = np.linspace(0.0, 8.0, 161)
    data = np.zeros((time.size, 16), dtype=float)
    data[:, 0] = time
    active = (time >= 2.0) & (time <= 6.0)
    post = time > 6.0
    data[active, 4] = 20.0 * estimate_gain
    data[active, 10] = active_level
    data[post, 10] = post_level
    return data


def diagnostic_series(update):
    data = np.zeros((update.shape[0], 50), dtype=float)
    data[:, 0] = update[:, 0]
    data[:, 1] = update[:, 0]
    data[:, 3] = np.arange(1, update.shape[0] + 1)
    data[:, 4:8] = 1.0
    data[:, 11] = np.linspace(0.0, 1.0, update.shape[0])
    data[:, 36:42] = update[:, 4:10]
    data[:, 42:48] = update[:, 4:10]
    data[:, 48] = np.linspace(0.0, 0.2, update.shape[0])
    return data


def observation_series(position_gain, orientation_gain=0.5):
    time = np.linspace(0.0, 8.0, 161)
    data = np.zeros((time.size, 13), dtype=float)
    data[:, 0] = time
    data[:, 9] = 0.32
    active = (time >= 2.0) & (time <= 6.0)
    data[active, 7] = 0.01 * position_gain
    data[active, 10] = 0.02 * orientation_gain
    return data


class AdaptiveMethodComparisonTest(unittest.TestCase):
    def setUp(self):
        self.updates = {
            "baseline": update_series(4.0, 2.0, 0.0),
            "current_rbf": update_series(2.0, 1.5, 0.5),
            "composite_rbf": update_series(1.0, 0.8, 0.8),
            "rff_ogd": update_series(1.5, 1.0, 0.7),
        }
        self.diagnostics = {
            name: diagnostic_series(value)
            for name, value in self.updates.items() if name != "baseline"
        }
        self.observations = {
            "baseline": observation_series(4.0, 4.0),
            "current_rbf": observation_series(2.0, 2.0),
            "composite_rbf": observation_series(1.0, 1.0),
            "rff_ogd": observation_series(1.5, 1.5),
        }

    def test_metrics_keep_all_methods_and_rank_lower_error_first(self):
        summary = MODULE.comparison_metrics(self.updates)
        self.assertEqual(set(summary["methods"]), set(MODULE.METHOD_ORDER))
        self.assertEqual(summary["ranking_by_active_sigma_rms"][0],
                         "composite_rbf")
        self.assertAlmostEqual(
            summary["improvement_vs_baseline"]["current_rbf"]
            ["active_sigma_rms_fraction"], 0.5)
        self.assertIn("BASELINE", MODULE.METHOD_LABELS["baseline"])

    def test_validation_rejects_nonfinite_input(self):
        broken = {name: value.copy() for name, value in self.updates.items()}
        broken["rff_ogd"][3, 10] = np.nan
        with self.assertRaises(RuntimeError):
            MODULE.validate_inputs(broken, self.diagnostics, minimum_rows=100)

    def test_metrics_accept_a_configured_disturbance_window(self):
        summary = MODULE.comparison_metrics(
            self.updates, disturbance_start=3.0, disturbance_end=5.0)
        self.assertEqual(summary["ranking_by_active_sigma_rms"][0],
                         "composite_rbf")

        with self.assertRaises(RuntimeError):
            MODULE.comparison_metrics(
                self.updates, disturbance_start=9.0, disturbance_end=10.0)

    def test_disturbance_trace_supports_reversal_sine_and_pulse_train(self):
        time = np.asarray([1.0, 1.5, 2.0, 2.5, 3.0])
        reversal = MODULE.disturbance_trace(
            time, 1.0, 3.0, (20.0, 0, 0, 0, 0, 0),
            "sign_reversal", 0.25, (-20.0, 0, 0, 0, 0, 0))
        self.assertEqual(reversal[1, 0], 20.0)
        self.assertEqual(reversal[3, 0], -20.0)

        sine = MODULE.disturbance_trace(
            np.asarray([1.0, 2.0]), 1.0, 3.0,
            (20.0, 0, 0, 0, 0, 0), "sine", 0.25)
        self.assertAlmostEqual(sine[0, 0], 0.0)
        self.assertAlmostEqual(sine[1, 0], 20.0)

        pulse = MODULE.disturbance_trace(
            np.asarray([1.0, 1.10, 1.30, 1.60]), 1.0, 3.0,
            (20.0, 0, 0, 0, 0, 0), "pulse_train", 2.0,
            (-10.0, 0, 0, 0, 0, 0))
        self.assertEqual(pulse[0, 0], 0.0)
        self.assertEqual(pulse[1, 0], 20.0)
        self.assertEqual(pulse[2, 0], -10.0)
        self.assertEqual(pulse[3, 0], 20.0)

        sine_cosine = MODULE.disturbance_trace(
            np.asarray([1.0, 1.0 + 1.0e-6, 2.0]), 1.0, 3.0,
            (20.0, 0, 0, 0, 0, 0), "sine_cosine", 0.25,
            (0.0, 10.0, 0, 0, 0, 0))
        self.assertAlmostEqual(sine_cosine[1, 1], 10.0, places=4)
        self.assertAlmostEqual(sine_cosine[2, 0], 20.0)

    def test_seven_figure_categories_are_created(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            MODULE.save_figures(output, self.updates, self.diagnostics)
            figures = sorted(output.glob("figure_*.png"))
            self.assertEqual(len(figures), 7)
            self.assertTrue(all(path.stat().st_size > 1000 for path in figures))

    def test_figures_accept_a_nondefault_wrench(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            MODULE.save_figures(
                output, self.updates, self.diagnostics,
                disturbance_start=2.0, disturbance_end=6.0,
                disturbance_wrench=(0.0, 10.0, 0.0, 0.0, 0.0, 0.0))
            self.assertEqual(len(list(output.glob("figure_*.png"))), 7)

    def test_direct_body_motion_metrics_and_eighth_figure(self):
        _, reference_pose, mismatch = MODULE.common_reference_pose(
            self.observations)
        self.assertAlmostEqual(mismatch, 0.0)
        summary = MODULE.comparison_motion_metrics(
            self.observations, reference_pose)
        self.assertGreater(
            summary["improvement_vs_baseline"]["composite_rbf"]
            ["active_position_rms_m_fraction"], 0.7)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            MODULE.save_figures(
                output, self.updates, self.diagnostics,
                observations=self.observations, reference_pose=reference_pose)
            self.assertEqual(len(list(output.glob("figure_*.png"))), 8)

    def test_walking_motion_uses_commanded_position_reference(self):
        observation = observation_series(0.0, 0.0)
        reference = np.asarray([0.0, 0.0, 0.32, 0.0, 0.0, 0.0])
        phase = np.clip((observation[:, 0] - 1.2) / 6.0, 0.0, 1.0)
        observation[:, 7] = 0.30 * phase
        metrics = MODULE.motion_series_metrics(
            observation, reference, locomotion_mode="walk",
            walking_target_step=0.30, walking_target_start=1.2,
            walking_target_duration=6.0)
        self.assertLess(metrics["full_position_peak_m"], 1.0e-12)


if __name__ == "__main__":
    unittest.main()
