#!/usr/bin/env python3
"""Deterministic math tests for the offline D5A identifiability audit."""

import importlib.util
from pathlib import Path

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "task_d_d5_identifiability",
    PACKAGE / "scripts" / "analyze_task_d_d5_identifiability.py")
ANALYSIS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYSIS)


def test_trapezoid_weights_integrate_constant_over_window():
    time = np.linspace(1.2, 16.2, 751)
    weights = ANALYSIS.trapezoid_weights(time)
    assert abs(np.sum(weights) - 15.0) < 1.0e-12
    assert np.all(weights > 0.0)


def test_column_normalization_removes_pure_coordinate_scaling():
    rng = np.random.RandomState(7)
    matrix = rng.randn(200, 10)
    scaled = matrix * np.logspace(-4, 5, 10)
    left = ANALYSIS.spectrum_metrics(matrix)
    right = ANALYSIS.spectrum_metrics(scaled)
    assert np.allclose(left["normalized_singular_ratios"],
                       right["normalized_singular_ratios"], atol=1.0e-12)
    assert right["raw_condition_number"] > left["raw_condition_number"] * 1.0e4


def test_rank_and_projection_detect_one_unobservable_coordinate():
    rng = np.random.RandomState(11)
    matrix = rng.randn(300, 10)
    matrix[:, -1] = 0.0
    metrics = ANALYSIS.spectrum_metrics(matrix)
    metrics["design_matrix"] = matrix
    assert metrics["ranks"]["1e-06"] == 9
    parameter = np.arange(1.0, 11.0)
    projection = ANALYSIS.observable_projection(metrics, parameter, 1.0e-6)
    assert projection["rank"] == 9
    assert abs(projection["observable"][-1]) < 1.0e-12
    assert abs(projection["null"][-1] - parameter[-1]) < 1.0e-12


def test_payload_coordinate_scale_uses_mass_length_and_mass_length_squared():
    pi_star = np.asarray([5.0, 0.5, 0.0, 0.0, 0.2, 0.0, 0.0, 0.2, 0.0, 0.2])
    scales = ANALYSIS.payload_coordinate_scales(pi_star)
    assert np.allclose(scales[:4], [5.0, 0.5, 0.5, 0.5])
    assert np.allclose(scales[4:], [0.05] * 6)
