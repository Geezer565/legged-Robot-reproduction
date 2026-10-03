#!/usr/bin/env python3
"""Freeze and audit the Stage D4 formal acquisition protocol."""

import argparse
import hashlib
import json
from pathlib import Path


GROUPS = ("nominal", "uncertain_off", "uncertain_adaptive")
RUNS = ("run_01", "run_02", "run_03")
WINDOW_START = 1.20
WINDOW_END = 16.20
ACQUISITION_END = 16.40
GRID_DT = 0.02


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--d3-fairness", type=Path, required=True)
    parser.add_argument("--d3-gate", type=Path, required=True)
    parser.add_argument("--config-audit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    d3_fairness = json.loads(args.d3_fairness.read_text())
    d3_gate = json.loads(args.d3_gate.read_text())
    config_audit = json.loads(args.config_audit.read_text())
    expected_pairwise = {
        "nominal__uncertain_adaptive": ["truePlantPayloadInertialEnabled"],
        "uncertain_off__uncertain_adaptive": [
            "enableParameterUpdate", "enableAdaptiveDynamics"],
        "nominal__uncertain_off": [
            "enableParameterUpdate", "enableAdaptiveDynamics",
            "truePlantPayloadInertialEnabled"],
    }
    pairwise_ok = all(
        config_audit["pairwise"][name]["changed_keys"] == expected
        and config_audit["pairwise"][name]["passed"]
        for name, expected in expected_pairwise.items())
    window_samples = int(round((WINDOW_END - WINDOW_START) / GRID_DT)) + 1
    arithmetic_ok = (
        abs((WINDOW_END - WINDOW_START) - 15.0) <= 1.0e-12
        and ACQUISITION_END > WINDOW_END
        and window_samples == 751)
    passed = bool(
        d3_fairness.get("passed") and d3_gate.get("passed")
        and config_audit.get("passed") and pairwise_ok and arithmetic_ok)
    result = {
        "schema_version": 4,
        "stage": "D4_formal_protocol",
        "passed": passed,
        "formal_campaign_started": False,
        "groups": list(GROUPS),
        "runs": list(RUNS),
        "runs_per_group": len(RUNS),
        "acquisition_order": [
            "nominal", "uncertain_off", "uncertain_adaptive"],
        "round_robin": True,
        "round_01_role": (
            "full-horizon gate; retained as formal run_01 only after the gate passes"),
        "analysis_window": {
            "start_mpc_time_s": WINDOW_START,
            "end_mpc_time_s": WINDOW_END,
            "duration_s": WINDOW_END - WINDOW_START,
            "common_grid_dt_s": GRID_DT,
            "common_grid_sample_count": window_samples,
        },
        "acquisition_end_mpc_time_s": ACQUISITION_END,
        "target": {
            "axis": "x", "relative_step_m": 0.10,
            "start_mpc_time_s": 1.20,
            "transition_duration_s": 0.20,
            "target_displacement_velocity_m_s": 0.50,
        },
        "payload": config_audit["groups"]["uncertain_adaptive"]["payload"],
        "allowed_independent_variables":
            config_audit["allowed_independent_variables"],
        "pairwise": config_audit["pairwise"],
        "common_mode_neutral_config_sha256":
            config_audit["common_mode_neutral_config_sha256"],
        "input_evidence": {
            "d3_fairness_sha256": digest(args.d3_fairness),
            "d3_gate_sha256": digest(args.d3_gate),
            "d4_config_audit_sha256": digest(args.config_audit),
        },
        "frozen_rules": {
            "parameter_tuning": False,
            "artificial_matched_wrench": False,
            "overwrite_existing_runs": False,
            "drop_failed_or_outlier_runs": False,
            "interpolation": "linear on one common MPC-time grid",
            "integration": "trapezoidal on the common grid",
            "group_dispersion": "sample standard deviation, ddof=1",
            "constant_truth_correlation": (
                "not reported because pi_star is constant in time"),
            "scientific_boundary": (
                "animal_dummy + fixed 10% off-center payload + fixed 10 cm static-walk target; "
                "no general parameter-convergence or paper-level claim"),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
