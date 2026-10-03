#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


FORMAL = load("d5h1_formal", PACKAGE / "scripts" /
              "analyze_task_d_d5h1_formal_campaign.py")
RUN_ANALYZER = load("d5h1_formal_run", PACKAGE / "scripts" /
                    "analyze_task_d_d5h1_formal_run.py")
PROTOCOL = load("d5h1_protocol", PACKAGE / "scripts" /
                "prepare_task_d_d5h1_formal_protocol.py")


def test_scaled_metrics_are_dimensionless_and_exact():
    scales = np.asarray([2.0, 4.0])
    truth = np.asarray([2.0, 4.0])
    estimate = np.asarray([1.0, 2.0])
    assert np.isclose(FORMAL.scaled_norm(estimate, scales), np.sqrt(0.5))
    assert np.isclose(FORMAL.scaled_rmse(estimate, truth, scales), 0.5)
    assert np.isclose(FORMAL.scaled_cosine(estimate, truth, scales), 1.0)
    assert json.dumps({"check": bool(np.bool_(True))}) == '{"check": true}'


def test_protocol_is_balanced_and_has_pre_registered_rules():
    assert len(PROTOCOL.ORDER) == 3
    assert all(sorted(order) == sorted(PROTOCOL.GROUPS)
               for order in PROTOCOL.ORDER)
    source = (PACKAGE / "scripts" /
              "prepare_task_d_d5h1_formal_protocol.py").read_text()
    assert '"startup_gate"' in source
    assert '"confidence_gate"' in source
    assert "nominal_drift_minimum_mean_suppression_fraction" in source
    assert "payload_contrast_minimum_norm_retention_fraction" in source
    assert "tracking_noninferiority_relative_margin" in source


def test_campaign_order_and_formal_analysis_are_frozen_before_runs():
    runner = (PACKAGE / "scripts" /
              "run_task_d_d5h1_formal_campaign.sh").read_text()
    per_run = (PACKAGE / "scripts" /
               "run_task_d_d5h1_experiment.sh").read_text()
    assert runner.count("run_one ") == 12
    assert "prepare_task_d_d5h1_formal_protocol.py" in runner
    assert "analyze_task_d_d5h1_formal_campaign.py" in runner
    assert "analyze_task_d_d5h1_formal_run.py" in per_run
    assert '"common_grid_dt_s": 0.02' in per_run
    assert '"analysis_window_end_mpc_time_s": float(end)' in per_run
    assert '"diagnosticTopicStartToleranceS": 0.10' in per_run
    assert '"topicEndToleranceS": 0.05' in per_run
    assert '"formalProtocolSha256"' in per_run
    assert '"acquisition_sequence"' in per_run


def test_payload_truth_uses_parallel_axis_base_origin_coordinates():
    config = {
        "truePlantPayloadInertialEnabled": True,
        "payload": {
            "mass_kg": 5.213485,
            "com_nominal_c0_base_m": [0.16, -0.03, 0.09],
            "inertia_com_base_kg_m2": [0.08, 0.0, 0.0, 0.1, 0.0, 0.12],
        },
    }
    expected = np.asarray([
        5.213485, 0.8341576, -0.15640455, 0.46921365,
        0.126921365, 0.025024728, -0.075074184,
        0.2756944445, 0.0140764095, 0.2581573525,
    ])
    plant = [[0.0] * 14 + expected.tolist()]
    truth, scales = RUN_ANALYZER.truth_and_scales(config, plant)
    assert np.allclose(truth, expected, rtol=0.0, atol=1.0e-12)
    assert np.allclose(scales[4:], 0.12, rtol=0.0, atol=1.0e-12)
