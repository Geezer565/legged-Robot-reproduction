#!/usr/bin/env python3
import importlib.util
import re
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "d5h1_mode", str(PACKAGE / "scripts" / "set_task_d_d5h1_mode.py"))
D5H1 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D5H1)


def token(text, key):
    match = re.search(r"^\s*" + re.escape(key) + r"\s+(\S+)",
                      text, re.MULTILINE)
    assert match
    return match.group(1)


def all_modes():
    base = D5H1.BASE_CONFIG.read_text()
    return {group: D5H1.materialize(base, group, D5H1.BASELINE)
            for group in D5H1.GROUPS}


def test_only_correction_switch_changes_each_same_plant_pair():
    texts = all_modes()
    keys = ("enableParameterUpdate", "enableAdaptiveDynamics",
            "enableGaitPhaseBaselineCorrection", "gaitPhaseBaselineFile",
            "gaitPhaseBins", "gaitCycleDuration",
            "gaitBaselineMinimumCycleStartTime")
    for raw, corrected in (("nominal_raw", "nominal_corrected"),
                           ("uncertain_raw", "uncertain_corrected")):
        changed = [key for key in keys
                   if token(texts[raw], key) != token(texts[corrected], key)]
        assert changed == ["enableGaitPhaseBaselineCorrection"]


def test_locked_controller_settings_remain_full_adaptive():
    for text in all_modes().values():
        assert token(text, "enableParameterUpdate") == "true"
        assert token(text, "enableAdaptiveDynamics") == "true"
        assert token(text, "enableClfConstraint") == "true"
        assert token(text, "enableInputBiasCost") == "false"
        assert token(text, "parameterization") == "payload_inertial_10d"
        assert token(text, "gaitPhaseBins") == "24"
        assert token(text, "gaitCycleDuration") == "1.2"


def test_frozen_baseline_identity_is_the_d5h0_passed_artifact():
    assert D5H1.sha256(D5H1.BASELINE) == (
        "541ee9118f6e772d01a48daafae7097b91ddb8f9b9cd95e7fa7d38c12af2ac68")
    provenance = __import__("json").loads(
        D5H1.APPLICATION_PROVENANCE.read_text())
    assert provenance["baseline"]["packaged_canonical_lf_sha256"] == (
        D5H1.sha256(D5H1.BASELINE))
    assert provenance["training"]["uncertain_or_payload_data_used"] is False
    assert provenance["startup_gate_used"] is False
    assert provenance["confidence_gate_used"] is False


def test_runner_records_exact_applied_update_topic():
    runner = (PACKAGE / "scripts" / "run_task_d_d5h1_experiment.sh").read_text()
    node = (PACKAGE / "src" / "LeggedRobotAdaptiveDdpMpcNode.cpp").read_text()
    assert "/task_d/d5h1_applied_update_sample" in runner
    assert "/task_d/d5h1_applied_update_sample" in node
    assert "payload_inertial_d5h1_applied_v1_71_fields" in node
    assert "/task_d/d5h1_eq11_sample" in runner
    assert "/task_d/d5h1_eq11_sample" in node
    assert "payload_inertial_d5h1_eq11_v1_119_fields" in node
