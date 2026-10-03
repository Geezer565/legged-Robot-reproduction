#!/usr/bin/env python3
"""Materialize the frozen Nominal/estimator-OFF D5I2 smoke configuration."""

import argparse
import hashlib
import importlib.util
import json
import re
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
BASE_CONFIG = (PACKAGE.parent / "ocs2_legged_robot_adaptive" / "config" /
               "adaptive_settings_payload_inertial_10d_smoke.info")
TASK_D_SPEC = importlib.util.spec_from_file_location(
    "task_d_mode", str(PACKAGE / "scripts" / "set_task_d_mode.py"))
TASK_D = importlib.util.module_from_spec(TASK_D_SPEC)
TASK_D_SPEC.loader.exec_module(TASK_D)

CANDIDATES = ("E2_rotation", "E3_combined")
SCHEDULE = {
    "profile_start_time_s": 2.4,
    "ramp_in_duration_s": 2.4,
    "steady_duration_s": 12.0,
    "ramp_out_duration_s": 2.4,
    "settle_duration_s": 1.2,
    "complete_time_s": 20.4,
    "knot_dt_s": 0.01,
    "c2_envelope": "10u3-15u4+6u5",
}


def sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def insert_encoded_reference(text):
    if re.search(r"^\s*enableEncodedReferenceDerivatives\s+", text,
                 re.MULTILINE):
        raise RuntimeError("base unexpectedly contains encoded-reference key")
    closing = text.rfind("}")
    if closing < 0:
        raise RuntimeError("adaptive settings have no closing block")
    addition = (
        "\n  ; D5I2 default-OFF richer-reference transport.\n"
        "  enableEncodedReferenceDerivatives true\n")
    return text[:closing] + addition + text[closing:]


def materialize(base_text):
    # uncertain_off has the required controller bits. Plant identity is a
    # launch-side independent variable, and D5I2 explicitly forces it nominal.
    return insert_encoded_reference(
        TASK_D.materialize_text(base_text, "uncertain_off"))


def effective_config(text, candidate):
    parent = TASK_D.effective_config(text, "uncertain_off")
    parent.update({
        "schema_version": 1,
        "stage": "D5I2_nominal_estimator_off_online_smoke",
        "group": "nominal_estimator_off",
        "candidate": candidate,
        "enableParameterUpdate": False,
        "enableAdaptiveDynamics": False,
        "enableEncodedReferenceDerivatives": True,
        "truePlantPayloadInertialEnabled": False,
        "truePlantMassScale": 1.0,
        "disturbanceEnabled": False,
        "richerReferenceEnabled": True,
        "schedule": SCHEDULE,
        "target": {
            "kind": "D5I2_C2_windowed_multisine",
            "profile_id":
                "d5i1_multisine_v1_T12_translation_h123_rotation_zyx_h81212",
            "candidate": candidate,
            "legacy_x_step_enabled": False,
        },
        "adaptive_config_sha256": sha256_bytes(text.encode()),
        "formal_campaign_started": False,
        "baseline_training_performed": False,
        "estimator_convergence_evaluated": False,
        "allowed_online_candidates": list(CANDIDATES),
    })
    return parent


def emit(args):
    text = materialize(args.base_config.read_text())
    config = effective_config(text, args.candidate)
    args.output_config.parent.mkdir(parents=True, exist_ok=True)
    args.output_config.write_text(text)
    args.output_json.write_text(
        json.dumps(config, indent=2, sort_keys=True) + "\n")
    print(json.dumps(config, indent=2, sort_keys=True))


def audit(args):
    base = args.base_config.read_text()
    text = materialize(base)
    configs = {candidate: effective_config(text, candidate)
               for candidate in CANDIDATES}
    required = {
        "enableParameterUpdate": False,
        "enableAdaptiveDynamics": False,
        "enableEncodedReferenceDerivatives": True,
        "enableClfConstraint": True,
        "enableInputBiasCost": False,
        "truePlantPayloadInertialEnabled": False,
        "truePlantMassScale": 1.0,
        "disturbanceEnabled": False,
        "richerReferenceEnabled": True,
    }
    checks = {
        candidate: all(config.get(key) == value
                       for key, value in required.items())
        for candidate, config in configs.items()
    }
    controller_hashes = {config["adaptive_config_sha256"]
                         for config in configs.values()}
    fixed_hashes = {json.dumps(config["fixed_files"], sort_keys=True)
                    for config in configs.values()}
    candidate_diffs = sorted(
        key for key in set(configs[CANDIDATES[0]]) |
        set(configs[CANDIDATES[1]])
        if configs[CANDIDATES[0]].get(key) !=
        configs[CANDIDATES[1]].get(key))
    passed = (all(checks.values()) and len(controller_hashes) == 1 and
              len(fixed_hashes) == 1 and
              candidate_diffs == ["candidate", "target"])
    result = {
        "schema_version": 1,
        "stage": "D5I2_mode_fairness_audit",
        "passed": passed,
        "required_values": required,
        "candidate_checks": checks,
        "candidate_identity_changed_keys": candidate_diffs,
        "common_adaptive_config_sha256": next(iter(controller_hashes)),
        "groups": configs,
    }
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir / "d5i2_nominal_estimator_off.info").write_text(text)
    (args.output_dir / "fairness_audit.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n")
    (args.output_dir / "effective_config_diff.txt").write_text(
        "E2_rotation vs E3_combined: only scheduler candidate differs\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    if not passed:
        raise RuntimeError("D5I2 mode fairness audit failed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG)
    subparsers = parser.add_subparsers(dest="command", required=True)
    emit_parser = subparsers.add_parser("emit")
    emit_parser.add_argument("candidate", choices=CANDIDATES)
    emit_parser.add_argument("--output-config", type=Path, required=True)
    emit_parser.add_argument("--output-json", type=Path, required=True)
    emit_parser.set_defaults(function=emit)
    audit_parser = subparsers.add_parser("audit")
    audit_parser.add_argument("--output-dir", type=Path, required=True)
    audit_parser.set_defaults(function=audit)
    args = parser.parse_args()
    args.function(args)


if __name__ == "__main__":
    main()
