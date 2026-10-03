#!/usr/bin/env python3
"""Small deterministic tests for the D4 protocol and common-grid math."""

import importlib.util
from pathlib import Path

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, PACKAGE / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


protocol = load("task_d_d4_protocol", "scripts/audit_task_d_d4_protocol.py")
analysis = load("task_d_d4_analysis", "scripts/analyze_task_d_d4_campaign.py")


def test_d4_protocol_is_exactly_fifteen_seconds_on_20ms_grid():
    assert abs((protocol.WINDOW_END - protocol.WINDOW_START) - 15.0) < 1.0e-12
    assert protocol.ACQUISITION_END > protocol.WINDOW_END
    assert int(round((protocol.WINDOW_END - protocol.WINDOW_START) /
                     protocol.GRID_DT)) + 1 == 751


def test_d4_group_statistics_use_sample_standard_deviation():
    result = analysis.numeric_stats([1.0, 2.0, 3.0])
    assert result["n"] == 3
    assert abs(result["mean"] - 2.0) < 1.0e-12
    assert abs(result["sample_std"] - 1.0) < 1.0e-12


def test_d4_time_weighted_vector_rms_uses_common_grid():
    time = np.linspace(1.2, 16.2, 751)
    vector = np.tile(np.asarray([3.0, 4.0]), (len(time), 1))
    assert abs(analysis.rms(vector, time) - 5.0) < 1.0e-12
    assert analysis.PARAMETERS == (
        "m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy", "Jyz", "Jzz")
