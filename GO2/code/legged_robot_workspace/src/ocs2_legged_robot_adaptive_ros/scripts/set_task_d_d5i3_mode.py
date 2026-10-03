#!/usr/bin/env python3
"""Materialize and audit frozen E3 Nominal/estimator-OFF D5I3 runs."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
BASE_CONFIG = (PACKAGE.parent / "ocs2_legged_robot_adaptive" / "config" /
               "adaptive_settings_payload_inertial_10d_smoke.info")
PROTOCOL = PACKAGE / "config" / "d5i3_nominal_baseline_protocol_v1.json"
D5I2_SPEC = importlib.util.spec_from_file_location(
    "d5i2_mode", str(PACKAGE / "scripts" / "set_task_d_d5i2_mode.py"))
D5I2 = importlib.util.module_from_spec(D5I2_SPEC)
D5I2_SPEC.loader.exec_module(D5I2)
ROLES = ("calibration", "heldout")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def protocol():
    value = json.loads(PROTOCOL.read_text())
    if value.get("stage") != "D5I3_nominal_only_baseline_calibration":
        raise RuntimeError("unexpected D5I3 protocol identity")
    return value


def materialize(base_text):
    return D5I2.materialize(base_text)


def effective_config(text, role, run_index):
    if role not in ROLES or run_index not in (1, 2, 3):
        raise ValueError("D5I3 role/run index is outside the frozen protocol")
    frozen = protocol()
    config = D5I2.effective_config(text, "E3_combined")
    config.update({
        "schema_version": 1,
        "stage": "D5I3_nominal_only_baseline_calibration_run",
        "group": "nominal_estimator_off",
        "calibration_role": role,
        "run_index": run_index,
        "candidate": "E3_combined",
        "schedule": frozen["schedule"],
        "target": {
            "kind": "D5I3_C2_windowed_E3_combined",
            "profile_id": frozen["profile_id"],
            "candidate": "E3_combined",
            "legacy_x_step_enabled": False,
        },
        "frozen_protocol_file": str(PROTOCOL.resolve()),
        "frozen_protocol_sha256": sha256(PROTOCOL),
        "baseline_key": frozen["baseline"],
        "baseline_training_eligible": role == "calibration",
        "heldout_excluded_from_training": role == "heldout",
        "analysis_artifact_stem": "task_d_d5i3",
        "analysis_display_stage": "D5I3",
        "analysis_summary_stage":
            "D5I3_nominal_only_baseline_calibration_run",
        "bag_filename": "task_d_d5i3.bag",
        "formal_campaign_started": False,
        "baseline_training_performed": False,
        "estimator_convergence_evaluated": False,
    })
    return config


def emit(args):
    text = materialize(args.base_config.read_text())
    config = effective_config(text, args.role, args.run_index)
    args.output_config.parent.mkdir(parents=True, exist_ok=True)
    args.output_config.write_text(text)
    args.output_json.write_text(
        json.dumps(config, indent=2, sort_keys=True) + "\n")
    print(json.dumps(config, indent=2, sort_keys=True))


def audit(args):
    text = materialize(args.base_config.read_text())
    configs = {(role, index): effective_config(text, role, index)
               for role in ROLES for index in (1, 2, 3)}
    frozen = protocol()
    required = {
        "candidate": "E3_combined",
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
    required_ok = all(
        all(config.get(key) == value for key, value in required.items())
        for config in configs.values())
    invariant_keys = (
        "adaptive_config_sha256", "fixed_files", "candidate", "schedule",
        "target", "baseline_key", "frozen_protocol_sha256")
    invariant_ok = all(
        len({json.dumps(config[key], sort_keys=True)
             for config in configs.values()}) == 1
        for key in invariant_keys)
    split_ok = all(
        config["baseline_training_eligible"] == (role == "calibration") and
        config["heldout_excluded_from_training"] == (role == "heldout")
        for (role, _), config in configs.items())
    passed = bool(required_ok and invariant_ok and split_ok and
                  frozen["schedule"]["steady_supercycles"] == 2 and
                  frozen["data_split"]["calibration_runs"] == 3 and
                  frozen["data_split"]["heldout_runs"] == 3)
    result = {
        "schema_version": 1,
        "stage": "D5I3_mode_and_split_audit",
        "passed": passed,
        "required_values": required,
        "scientific_invariants_identical": invariant_ok,
        "pre_registered_split_ok": split_ok,
        "frozen_protocol_file": str(PROTOCOL.resolve()),
        "frozen_protocol_sha256": sha256(PROTOCOL),
        "pre_registered_order": frozen["data_split"]["pre_registered_order"],
        "groups": {"{}_run_{:02d}".format(role, index): config
                   for (role, index), config in configs.items()},
    }
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir / "d5i3_nominal_estimator_off.info").write_text(text)
    (args.output_dir / "stage_d5i3_protocol.json").write_text(
        PROTOCOL.read_text())
    (args.output_dir / "fairness_and_split_audit.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n")
    (args.output_dir / "effective_config_diff.txt").write_text(
        "All six runs: identical controller/plant/reference/schedule.\n"
        "Allowed metadata-only differences: calibration_role, run_index, "
        "baseline_training_eligible, heldout_excluded_from_training.\n"
        "Heldout runs are excluded from template construction.\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    if not passed:
        raise RuntimeError("D5I3 mode/split audit failed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG)
    subparsers = parser.add_subparsers(dest="command", required=True)
    emit_parser = subparsers.add_parser("emit")
    emit_parser.add_argument("role", choices=ROLES)
    emit_parser.add_argument("run_index", type=int, choices=(1, 2, 3))
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
