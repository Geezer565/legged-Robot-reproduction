#!/usr/bin/env python3
import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_task_d_d5f_gait_phase_covariance.py"
SPEC = importlib.util.spec_from_file_location("d5f", str(SCRIPT))
D5F = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(D5F)


def test_weighted_mean_covariance_closes_exactly():
    rng = np.random.RandomState(17)
    y = rng.randn(19, 6, 10)
    sigma = rng.randn(19, 6)
    weights = rng.uniform(0.01, 0.04, size=19)
    result = D5F.weighted_mean_covariance(y, sigma, weights)
    np.testing.assert_allclose(
        result["direct"], result["mean_product"] + result["covariance"],
        rtol=0.0, atol=1.0e-14)
    np.testing.assert_allclose(
        result["direct_by_axis"],
        result["mean_product_by_axis"] + result["covariance_by_axis"],
        rtol=0.0, atol=1.0e-14)
    np.testing.assert_allclose(result["direct"],
                               np.sum(result["direct_by_axis"], axis=0),
                               rtol=0.0, atol=1.0e-14)


def test_constant_regressor_has_zero_covariance():
    rng = np.random.RandomState(19)
    y0 = rng.randn(6, 10)
    y = np.repeat(y0[None, :, :], 8, axis=0)
    sigma = rng.randn(8, 6)
    result = D5F.weighted_mean_covariance(y, sigma, np.ones(8))
    np.testing.assert_allclose(result["covariance"], 0.0, atol=1.0e-14)


def test_complete_cycles_require_executed_mode_sequence_and_steady_start():
    sequence = [(0.0, 15), (1.02, 13), (1.32, 7), (1.62, 14),
                (1.92, 11), (2.22, 13), (2.52, 7), (2.82, 14),
                (3.12, 11), (3.42, 13), (3.72, 7), (4.02, 14),
                (4.32, 11), (4.62, 13)]
    observations = [(time, mode, None, None) for time, mode in sequence]
    cycles = D5F.complete_steady_cycles(observations)
    assert len(cycles) == 2
    assert cycles[0]["start"] == 2.22
    assert cycles[0]["transition_sequence"] == [13, 7, 14, 11]


def test_phase_bins_sum_to_explicit_euler_cycle_update():
    time = np.arange(0.0, 1.21, 0.1)
    drive = np.column_stack((time, 2.0 * time))
    contribution, duration = D5F.phase_bin_integral(time, drive, 0.0, 1.2, bins=4)
    expected = np.sum(np.diff(time)[:, None] * drive[1:], axis=0)
    np.testing.assert_allclose(np.sum(contribution, axis=0), expected)
    np.testing.assert_allclose(np.sum(duration), 1.2)
