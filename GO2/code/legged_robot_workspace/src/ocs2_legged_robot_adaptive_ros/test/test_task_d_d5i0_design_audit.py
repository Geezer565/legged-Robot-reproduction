#!/usr/bin/env python3
"""Unit tests for D5I0 identifiability-audit mathematics."""

import importlib.util
from pathlib import Path

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "task_d_d5i0", PACKAGE / "scripts" /
    "analyze_task_d_d5i0_design_audit.py")
D5I0 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D5I0)


def test_weighted_regressor_design_matches_gramian():
    rng = np.random.RandomState(21)
    regressor = rng.randn(20, 6, 10)
    weights = np.linspace(0.01, 0.03, 20)
    scales = np.linspace(0.2, 2.0, 10)
    design = D5I0.weighted_regressor_design(regressor, weights, scales)
    expected = np.einsum(
        "n,nji,njk->ik", weights,
        regressor * scales[None, None, :],
        regressor * scales[None, None, :])
    assert np.allclose(design.T @ design, expected, atol=1.0e-12)


def test_wrench_row_scaling_removes_force_torque_unit_choice():
    rng = np.random.RandomState(23)
    regressor = rng.randn(20, 6, 10)
    weights = np.full(20, 0.01)
    scales = np.ones(10)
    row_scales = np.asarray([2.0, 2.0, 2.0, 0.4, 0.4, 0.4])
    design = D5I0.weighted_regressor_design(
        regressor, weights, scales, row_scales)
    changed_units = regressor * row_scales[None, :, None]
    changed_scales = row_scales ** 2
    same = D5I0.weighted_regressor_design(
        changed_units, weights, scales, changed_scales)
    assert np.allclose(design, same, atol=1.0e-12)


def test_spectrum_detects_rank_and_weak_coordinate():
    rng = np.random.RandomState(22)
    matrix = rng.randn(300, 10)
    matrix[:, -1] = 1.0e-5 * rng.randn(300)
    metrics = D5I0.spectrum(matrix)
    assert metrics["ranks"]["1e-03"] == 9
    weakest = metrics["right_singular_vectors"][-1]
    assert abs(weakest[-1]) > 0.999


def test_dimensionless_update_drive_uses_pi_dot_over_scale():
    signal = np.tile(np.arange(1.0, 11.0), (5, 1))
    weights = np.full(5, 0.2)
    scales = np.arange(1.0, 11.0)
    design = D5I0.weighted_signal_design(signal, weights, scales)
    assert np.allclose(design, np.sqrt(0.2))
    centered = D5I0.weighted_signal_design(signal, weights, scales, center=True)
    assert np.max(np.abs(centered)) < 1.0e-14


def test_observable_projection_excludes_declared_weak_direction():
    matrix = np.eye(10)
    matrix[-1, -1] = 1.0e-5
    metrics = D5I0.spectrum(matrix)
    truth = np.ones(10)
    projection = D5I0.observable_projection(metrics, truth, np.ones(10))
    assert projection["rank"] == 9
    assert abs(projection["weak_physical"][-1] - 1.0) < 1.0e-12
    assert np.max(np.abs(projection["weak_physical"][:-1])) < 1.0e-12
