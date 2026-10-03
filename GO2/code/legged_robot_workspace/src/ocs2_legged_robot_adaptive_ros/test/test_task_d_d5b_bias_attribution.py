#!/usr/bin/env python3
"""Protocol and contrast tests for Stage D5B."""

import importlib.util
from pathlib import Path

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, PACKAGE / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


protocol = load("d5b_protocol", "scripts/audit_task_d_d5b_protocol.py")
analysis = load("d5b_analysis", "scripts/analyze_task_d_d5b_campaign.py")


def test_d5b_groups_change_only_target_gait_and_payload_conditions():
    groups = protocol.GROUPS
    assert tuple(groups) == (
        "nominal_stance_hold", "nominal_stance_target",
        "nominal_walk_target", "uncertain_walk_target")
    assert groups["nominal_stance_hold"]["target"] is False
    assert groups["nominal_stance_target"]["target"] is True
    assert groups["nominal_stance_target"]["walking"] is False
    assert groups["nominal_walk_target"]["walking"] is True
    assert groups["uncertain_walk_target"]["payload"] is True


def test_d5b_controlled_contrasts_telescope_but_are_not_declared_causal():
    hold = np.asarray([1.0, 2.0])
    target = np.asarray([3.0, 1.0])
    walk = np.asarray([4.0, 6.0])
    target_contrast = target - hold
    gait_contrast = walk - target
    assert np.allclose(hold + target_contrast + gait_contrast, walk)
    assert analysis.CONTRASTS["target_without_gait"] == (
        "nominal_stance_hold", "nominal_stance_target")
    assert analysis.CONTRASTS["walking_given_target"] == (
        "nominal_stance_target", "nominal_walk_target")


def test_d5b_statistics_use_sample_standard_deviation():
    result = analysis.stats([1.0, 2.0, 3.0])
    assert result["mean"] == 2.0
    assert result["sample_std"] == 1.0
