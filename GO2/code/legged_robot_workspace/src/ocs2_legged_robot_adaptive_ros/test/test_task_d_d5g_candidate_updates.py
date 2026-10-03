#!/usr/bin/env python3
import importlib.util
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_task_d_d5g_candidate_updates.py"
SPEC = importlib.util.spec_from_file_location("d5g", str(SCRIPT))
D5G = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(D5G)


def test_original_replay_matches_explicit_euler_sum():
    rng = np.random.RandomState(31)
    dt = rng.uniform(0.01, 0.03, 20)
    drive = rng.randn(20, 10)
    estimate, effective, active = D5G.replay_update(
        dt, drive, np.ones(20), np.zeros(20, dtype=int), "original")
    np.testing.assert_allclose(estimate[-1], np.sum(dt[:, None] * drive, axis=0))
    np.testing.assert_allclose(effective, drive)
    assert active == 1.0


def test_leakage_replay_uses_previous_estimate():
    dt = np.full(4, 0.1)
    drive = np.ones((4, 10))
    estimate, _, _ = D5G.replay_update(
        dt, drive, np.ones(4), np.zeros(4, dtype=int), "leakage", value=0.5)
    expected = 0.0
    for _ in range(4):
        expected += 0.1 * (1.0 - 0.5 * expected)
    np.testing.assert_allclose(estimate[-1], expected)


def test_hard_deadzone_gates_declared_sigma_norm():
    dt = np.ones(4)
    drive = np.ones((4, 10))
    sigma_norm = np.asarray([0.1, 0.5, 0.51, 2.0])
    estimate, effective, active = D5G.replay_update(
        dt, drive, sigma_norm, np.zeros(4, dtype=int), "deadzone", threshold=0.5)
    np.testing.assert_allclose(estimate[-1], 2.0)
    np.testing.assert_allclose(effective[:2], 0.0)
    assert active == 0.5


def test_nominal_phase_baseline_cancels_identical_trace():
    run = {
        "phase": np.asarray([0, 0, 1, 1]),
        "dt": np.asarray([0.1, 0.2, 0.1, 0.2]),
        "drive": np.asarray([[1.0] * 10, [1.0] * 10,
                             [-2.0] * 10, [-2.0] * 10]),
    }
    baseline, duration = D5G.phase_baseline([run], phase_bins=2)
    estimate, effective, _ = D5G.replay_update(
        run["dt"], run["drive"], np.ones(4), run["phase"],
        "phase_baseline", baseline=baseline)
    np.testing.assert_allclose(baseline[0], 1.0)
    np.testing.assert_allclose(baseline[1], -2.0)
    np.testing.assert_allclose(duration, [0.3, 0.3])
    np.testing.assert_allclose(effective, 0.0, atol=1.0e-15)
    np.testing.assert_allclose(estimate[-1], 0.0, atol=1.0e-15)


def test_nominal_training_split_excludes_evaluation_run():
    runs = {"nominal": [{"run_index": 1}, {"run_index": 2}, {"run_index": 3}]}
    selected, rule = D5G.training_nominal_runs(runs, "nominal", 2)
    assert [run["run_index"] for run in selected] == [1, 3]
    assert rule == "leave_one_nominal_run_out"
    selected, rule = D5G.training_nominal_runs(runs, "uncertain", 2)
    assert [run["run_index"] for run in selected] == [1, 2, 3]
    assert rule == "all_nominal_runs"


def test_weighted_quantile_respects_time_weights():
    value = D5G.weighted_quantile(
        np.asarray([0.0, 1.0, 2.0]), np.asarray([1.0, 8.0, 1.0]), 0.5)
    assert value == 1.0


def test_three_way_leave_one_out_baseline_residual_has_zero_group_mean():
    updates = np.asarray([[1.0, 2.0], [1.2, 2.1], [0.8, 1.9]])
    residuals = []
    for index in range(3):
        training = np.delete(updates, index, axis=0)
        residuals.append(updates[index] - np.mean(training, axis=0))
    np.testing.assert_allclose(np.mean(residuals, axis=0), 0.0, atol=1.0e-15)
