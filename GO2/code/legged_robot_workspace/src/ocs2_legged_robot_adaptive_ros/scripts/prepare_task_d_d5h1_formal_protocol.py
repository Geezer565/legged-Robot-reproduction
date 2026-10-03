#!/usr/bin/env python3
"""Freeze the D5H1 repeated-campaign protocol before acquisition."""

import argparse
import hashlib
import json
from pathlib import Path


GROUPS = ("nominal_raw", "nominal_corrected",
          "uncertain_raw", "uncertain_corrected")
ORDER = (
    ("nominal_raw", "nominal_corrected", "uncertain_raw", "uncertain_corrected"),
    ("nominal_corrected", "uncertain_raw", "uncertain_corrected", "nominal_raw"),
    ("uncertain_raw", "uncertain_corrected", "nominal_raw", "nominal_corrected"),
)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--online-gate", type=Path, required=True)
    parser.add_argument("--config-audit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    gate = json.loads(args.online_gate.read_text())
    audit = json.loads(args.config_audit.read_text())
    checks = {
        "four_mode_online_gate_passed": bool(gate.get("passed")),
        "four_mode_config_fairness_passed": bool(audit.get("passed")),
        "startup_or_confidence_gate_not_in_protocol": True,
        "three_repetitions_per_mode": len(ORDER) == 3,
        "each_round_contains_each_mode_once": all(
            sorted(round_order) == sorted(GROUPS) for round_order in ORDER),
    }
    passed = all(checks.values())
    protocol = {
        "schema_version": 1,
        "stage": "D5H1_formal_four_mode_protocol",
        "passed": passed,
        "checks": checks,
        "groups": list(GROUPS),
        "repetitions_per_group": 3,
        "round_robin_order": [list(value) for value in ORDER],
        "analysis_window": {
            "start_mpc_time_s": 0.0,
            "end_mpc_time_s": 15.0,
            "common_grid_dt_s": 0.02,
            "terminal_average_window_s": 2.0,
            "diagnostic_topic_start_tolerance_s": 0.10,
            "topic_end_tolerance_s": 0.05,
        },
        "frozen_engineering_decision_rules": {
            "all_12_runs_must_pass_per_run_gate": True,
            "nominal_drift_primary_metric":
                "last2s_mean_pi_hat_vector_physical_scaled_norm",
            "nominal_drift_minimum_mean_suppression_fraction": 0.50,
            "nominal_corrected_must_be_lower_in_every_paired_run": True,
            "payload_signal_primary_metric":
                "paired_last2s_mean_(uncertain_minus_nominal)_physical_scaled_norm",
            "payload_contrast_minimum_norm_retention_fraction": 0.80,
            "payload_cosine_maximum_allowed_drop": 0.05,
            "payload_scaled_rmse_maximum_relative_increase": 0.05,
            "tracking_primary_metrics": ["x_rmse_m", "position_rmse_m"],
            "tracking_noninferiority_relative_margin": 0.05,
            "tracking_noninferiority_absolute_margin_m": 0.0005,
        },
        "independent_variables": [
            "enableGaitPhaseBaselineCorrection",
            "truePlantPayloadInertialEnabled",
        ],
        "payload_parameter_coordinate_convention": {
            "pi_star_inertia": "payload inertia about base origin",
            "configured_inertia": "payload inertia about payload COM",
            "mapping": "J_origin = J_com + m*((c dot c)*I - c*c^T)",
            "physical_scale_compatibility":
                "D5H0/D5H1 scaling retains max(abs(J_com)) for all six inertia coordinates",
        },
        "explicitly_excluded": [
            "startup_gate", "confidence_gate", "Gamma_tuning",
            "richer_excitation", "10D_identifiability_claim",
        ],
        "upstream_gate": {
            "path": str(args.online_gate.resolve()),
            "sha256": sha256(args.online_gate),
        },
        "config_audit": {
            "path": str(args.config_audit.resolve()),
            "sha256": sha256(args.config_audit),
        },
        "scientific_boundary": (
            "The decision rules are engineering screens for this fixed plant, "
            "target, gait, and 15 s window. Passing is not proof of 10D inertial "
            "parameter convergence or of the paper's general theory."),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(protocol, indent=2, sort_keys=True) + "\n")
    print(json.dumps(protocol, indent=2, sort_keys=True))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
