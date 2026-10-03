#!/usr/bin/env python3
"""Unit tests for the exact Stage D5C bookkeeping identities."""

import importlib.util
from pathlib import Path

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "d5c", PACKAGE / "scripts/analyze_task_d_d5c_nominal_residual.py")
D5C = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D5C)


def test_axis_terms_reconstruct_eq8_drive_exactly():
    regressor = np.arange(120.0).reshape(2, 6, 10) / 17.0
    sigma = np.asarray([[1.0, -2.0, 0.5, 3.0, -1.0, 4.0],
                        [-0.5, 2.5, 1.5, -1.0, 0.25, 2.0]])
    terms = D5C.axis_drive_terms(regressor, sigma)
    assert terms.shape == (2, 6, 10)
    assert np.allclose(np.sum(terms, axis=1),
                       np.einsum("nij,ni->nj", regressor, sigma))


def test_partition_integrals_follow_right_endpoint_euler_and_are_additive():
    time = np.asarray([0.0, 0.2, 0.5, 1.0])
    values = np.asarray([[0.0, 1.0], [2.0, 3.0], [1.0, -1.0], [4.0, 2.0]])
    sample_modes = np.asarray([13, 13, 7, 7])
    labels = D5C.interval_mode_labels(sample_modes)
    assert labels.tolist() == [13, 7, 7]
    partition = D5C.partition_integrals(time, values, labels)
    signed, absolute, squared, _ = D5C.interval_integrals(time, values)
    assert np.allclose(sum(item["signed"] for item in partition.values()),
                       np.sum(signed, axis=0))
    assert np.allclose(sum(item["absolute"] for item in partition.values()),
                       np.sum(absolute, axis=0))
    assert np.allclose(sum(item["squared"] for item in partition.values()),
                       np.sum(squared, axis=0))
    expected = (time[1:, None] - time[:-1, None]) * values[1:]
    assert np.allclose(np.sum(signed, axis=0), np.sum(expected, axis=0))


def test_time_segments_assign_every_interval_once():
    time = np.asarray([0.0, 1.0, 1.2, 1.3, 1.4, 2.0, 2.6, 3.0])
    labels = D5C.interval_time_labels(time)
    assert len(labels) == len(time) - 1
    assert labels.tolist() == [
        "pre_target_0_1p2", "target_ramp_1p2_1p4",
        "target_ramp_1p2_1p4", "first_post_ramp_cycle_1p4_2p6",
        "first_post_ramp_cycle_1p4_2p6", "steady_walk_2p6_16p2",
        "steady_walk_2p6_16p2"]


def test_clipped_rows_interpolates_exact_boundaries_without_duplicates():
    rows = np.asarray([[0.0, 0.0], [1.0, 2.0], [2.0, 4.0], [3.0, 6.0]])
    result = D5C.clipped_rows(rows, 0.5, 2.5)
    assert np.allclose(result[:, 0], [0.5, 1.0, 2.0, 2.5])
    assert np.allclose(result[:, 1], [1.0, 2.0, 4.0, 5.0])
