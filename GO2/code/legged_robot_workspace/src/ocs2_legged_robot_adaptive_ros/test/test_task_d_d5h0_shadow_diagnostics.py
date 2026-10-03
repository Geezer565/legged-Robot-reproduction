#!/usr/bin/env python3
import importlib.util
import json
from pathlib import Path

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "d5h0_shadow_diagnostics.py"
SPEC = importlib.util.spec_from_file_location("d5h0_shadow", str(SCRIPT))
D5H0 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D5H0)
BASELINE = PACKAGE / "config" / "d5h0_nominal_gait_phase_baseline_v1.csv"


def test_packaged_baseline_is_complete_and_frozen():
    baseline = D5H0.load_phase_baseline(BASELINE)
    assert baseline["values"].shape == (24, 10)
    assert baseline["sha256"] == (
        "541ee9118f6e772d01a48daafae7097b91ddb8f9b9cd95e7fa7d38c12af2ac68")
    np.testing.assert_allclose(baseline["phase_bounds"][0], [0.0, 1.0 / 24.0])
    np.testing.assert_allclose(baseline["phase_bounds"][-1], [23.0 / 24.0, 1.0])
    assert np.all(baseline["durations"] > 0.0)


def test_phase_tracker_uses_previous_cycle_for_closing_endpoint():
    tracker = D5H0.StaticWalkPhaseTracker(
        cycle_duration=1.2, phase_bins=24, minimum_cycle_start=2.2)
    samples = [(1.2, 13), (1.5, 7), (1.8, 14), (2.1, 11),
               (2.4, 13), (2.42, 13), (2.7, 7), (3.0, 14),
               (3.3, 11), (3.6, 13)]
    outputs = [tracker.sample(time, mode) for time, mode in samples]
    assert outputs[4]["cycle_boundary"]
    assert outputs[4]["phase_bin"] == 23
    assert not outputs[4]["steady_active"]
    assert outputs[5]["phase_bin"] == 0
    assert outputs[5]["steady_active"]
    assert outputs[-1]["cycle_boundary"]
    assert outputs[-1]["phase_bin"] == 23
    assert outputs[-1]["steady_active"]


def test_phase_tracker_rejects_wrong_mode_sequence_until_next_cycle_start():
    tracker = D5H0.StaticWalkPhaseTracker(minimum_cycle_start=0.0)
    assert tracker.sample(0.0, 13)["phase_valid"]
    assert not tracker.sample(0.3, 14)["phase_valid"]
    assert not tracker.sample(0.6, 11)["phase_valid"]
    restarted = tracker.sample(1.2, 13)
    assert restarted["phase_valid"]
    assert restarted["phase_bin"] == 0


def test_shadow_accumulator_never_changes_supplied_drive_and_integrates_corrected():
    accumulator = D5H0.ShadowAccumulator(np.eye(10))
    raw = np.arange(1.0, 11.0)
    corrected = raw - 0.25
    dt, cumulative_raw, cumulative_corrected = accumulator.update(
        2.42, True, 2.4, raw, corrected)
    assert abs(dt - 0.02) < 1.0e-12
    np.testing.assert_allclose(cumulative_raw, 0.02 * raw)
    np.testing.assert_allclose(cumulative_corrected, 0.02 * corrected)
    np.testing.assert_allclose(raw, np.arange(1.0, 11.0))


def test_shadow_accumulator_does_not_backfill_an_inactive_gap():
    accumulator = D5H0.ShadowAccumulator(np.eye(10))
    drive = np.ones(10)
    accumulator.update(2.40, False, 2.40, drive, drive)
    accumulator.update(2.42, False, 2.40, drive, drive)
    dt, cumulative_raw, _ = accumulator.update(
        2.44, True, 2.40, drive, drive)
    assert abs(dt - 0.02) < 1.0e-12
    np.testing.assert_allclose(cumulative_raw, 0.02 * drive)


def test_fixed_width_shadow_schema_and_exact_drive_closure():
    baseline = np.ones((24, 10)) * 0.5
    tracker = D5H0.StaticWalkPhaseTracker(minimum_cycle_start=0.0)
    accumulator = D5H0.ShadowAccumulator(np.eye(10))
    row = np.zeros(118)
    row[:4] = [0.0, 0.0, 0.0, 1.0]
    row[42:48] = [1.0, -2.0, 0.5, 0.0, 3.0, -1.0]
    regressor = np.arange(60, dtype=float).reshape(6, 10) / 10.0
    row[48:108] = regressor.reshape(-1)
    output = D5H0.shadow_sample(
        row, 13, baseline, tracker, accumulator)
    expected_raw = regressor.T.dot(row[42:48])
    assert output.shape == (D5H0.OUTPUT_FIELDS,)
    assert np.all(np.isfinite(output))
    np.testing.assert_allclose(output[15:25], expected_raw)
    np.testing.assert_allclose(output[25:35], baseline[0])
    np.testing.assert_allclose(output[35:45], expected_raw - baseline[0])
    assert output[14] == 1.0  # fixed proof flag: shadow only


def test_source_contract_contains_no_controller_output_topics():
    source = SCRIPT.read_text()
    assert "actual_estimator_formula_unchanged" in source
    assert "/task_d/d5h0_shadow_update_sample" in source
    for forbidden in ("/legged_robot_mpc_target", "/legged_robot_mpc_mode_schedule",
                      "/legged_robot_mpc_observation"):
        assert "Publisher(\n+            \"{}\"".format(forbidden) not in source


def test_campaign_gate_comparisons_are_json_serializable_native_bools():
    """The Noetic Python 3.8 JSON encoder rejects numpy.bool_ values."""
    values = {
        "suppression": bool(np.float64(0.98) >= 0.95),
        "retention": bool(np.float64(1.01) >= 0.95),
        "rmse": bool(np.float64(0.9) <= 1.05 * np.float64(1.0)),
        "cosine": bool(np.float64(0.7) >= np.float64(0.72) - 0.05),
    }
    assert all(type(value) is bool for value in values.values())
    assert json.loads(json.dumps(values)) == values
