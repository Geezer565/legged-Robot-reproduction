#!/usr/bin/env python3
"""Materialize/audit the frozen D5I4 E3 four-mode online-smoke configs."""

import argparse
import difflib
import hashlib
import importlib.util
import json
import re
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
BASE_CONFIG = (PACKAGE.parent / "ocs2_legged_robot_adaptive" / "config" /
               "adaptive_settings_payload_inertial_10d_smoke.info")
PROTOCOL = PACKAGE / "config" / "d5i4_online_smoke_protocol_v1.json"
D5I3_ROOT = (WORKSPACE / "records" / "task_d_payload_inertial_10d" /
             "stage_d5i3_nominal_baseline_calibration" / "analysis")
BASELINE = D5I3_ROOT / "nominal_e3_composite_phase_baseline_v1.csv"
PROVENANCE = D5I3_ROOT / "nominal_e3_composite_phase_baseline_v1.json"

TASK_D_SPEC = importlib.util.spec_from_file_location(
    "task_d_mode", str(PACKAGE / "scripts" / "set_task_d_mode.py"))
TASK_D = importlib.util.module_from_spec(TASK_D_SPEC)
TASK_D_SPEC.loader.exec_module(TASK_D)
D5I2_SPEC = importlib.util.spec_from_file_location(
    "d5i2_mode", str(PACKAGE / "scripts" / "set_task_d_d5i2_mode.py"))
D5I2 = importlib.util.module_from_spec(D5I2_SPEC)
D5I2_SPEC.loader.exec_module(D5I2)

GROUPS = {
    "nominal_original": ("nominal", False),
    "nominal_phase_corrected": ("nominal", True),
    "payload_original": ("uncertain_adaptive", False),
    "payload_phase_corrected": ("uncertain_adaptive", True),
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def frozen_protocol():
    value = json.loads(PROTOCOL.read_text())
    if value.get("stage") != "D5I4_E3_combined_four_mode_online_smoke":
        raise RuntimeError("unexpected D5I4 protocol identity")
    return value


def validate_frozen_artifacts(baseline, provenance):
    protocol = frozen_protocol()
    frozen = protocol["frozen_baseline"]
    if sha256(baseline) != frozen["csv_sha256"]:
        raise RuntimeError("D5I4 baseline hash differs from frozen protocol")
    if sha256(provenance) != frozen["provenance_sha256"]:
        raise RuntimeError("D5I4 provenance hash differs from frozen protocol")
    value = json.loads(Path(provenance).read_text())
    if (not value.get("passed") or
            value.get("occupied_composite_keys") !=
            frozen["occupied_composite_keys"] or
            value.get("baseline_csv_sha256") != frozen["csv_sha256"]):
        raise RuntimeError("D5I4 provenance does not attest the frozen baseline")


def insert_composite_settings(text, enabled, baseline):
    if re.search(r"^\s*enableCompositePhaseBaselineCorrection\s+", text,
                 re.MULTILINE):
        raise RuntimeError("base unexpectedly contains D5I4 correction keys")
    closing = text.rfind("}")
    if closing < 0:
        raise RuntimeError("adaptive settings have no closing block")
    value = "true" if enabled else "false"
    addition = (
        "\n  ; D5I4 default-OFF frozen E3 composite-phase baseline.\n"
        "  enableCompositePhaseBaselineCorrection {}\n"
        "  compositePhaseBaselineFile {}\n"
        "  compositeProfilePhaseBins 120\n"
        "  compositeGaitPhaseBins 24\n"
        "  compositeProfilePeriod 12.0\n"
        "  compositeProfileStartTime 2.4\n"
        "  compositeRampInDuration 2.4\n"
        "  compositeSteadyDuration 24.0\n"
        "  compositeGaitCycleDuration 1.2\n".format(
            value, Path(baseline).resolve()))
    return text[:closing] + addition + text[closing:]


def materialize(base_text, group, baseline=BASELINE):
    parent, corrected = GROUPS[group]
    text = TASK_D.materialize_text(base_text, parent)
    text = D5I2.insert_encoded_reference(text)
    return insert_composite_settings(text, corrected, baseline)


def effective_config(text, group, baseline=BASELINE,
                     provenance=PROVENANCE):
    validate_frozen_artifacts(baseline, provenance)
    parent, corrected = GROUPS[group]
    config = TASK_D.effective_config(text, parent)
    schedule = frozen_protocol()["schedule"]
    config.update({
        "schema_version": 1,
        "stage": "D5I4_E3_combined_four_mode_online_smoke_run",
        "group": group,
        "task_d_parent_group": parent,
        "candidate": "E3_combined",
        "richerReferenceEnabled": True,
        "enableEncodedReferenceDerivatives": True,
        "enableCompositePhaseBaselineCorrection": corrected,
        "enableGaitPhaseBaselineCorrection": False,
        "compositePhaseBaselineFile": str(Path(baseline).resolve()),
        "compositePhaseBaselineSha256": sha256(baseline),
        "compositePhaseBaselineProvenanceFile":
            str(Path(provenance).resolve()),
        "compositePhaseBaselineProvenanceSha256": sha256(provenance),
        "compositePhaseBaselineTrainingUsesPayloadData": False,
        "compositePhaseBaselineOnlineUpdateAllowed": False,
        "compositeProfilePhaseBins": 120,
        "compositeGaitPhaseBins": 24,
        "compositeProfilePeriod": 12.0,
        "compositeProfileStartTime": 2.4,
        "compositeRampInDuration": 2.4,
        "compositeSteadyDuration": 24.0,
        "compositeGaitCycleDuration": 1.2,
        "schedule": schedule,
        "target": {
            "kind": "D5I4_C2_windowed_E3_combined",
            "profile_id": frozen_protocol()["profile_id"],
            "candidate": "E3_combined",
            "legacy_x_step_enabled": False,
        },
        "frozen_protocol_file": str(PROTOCOL.resolve()),
        "frozen_protocol_sha256": sha256(PROTOCOL),
        "d5i4_update_topic": "/task_d/d5i4_composite_update_sample",
        "d5i4_update_schema":
            "payload_inertial_d5i4_composite_update_v1_76_fields",
        "formal_campaign_started": False,
        "smoke_only": True,
    })
    return config


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value)


def emit(args):
    text = materialize(args.base_config.read_text(), args.group,
                       args.baseline_file)
    config = effective_config(text, args.group, args.baseline_file,
                              args.provenance_file)
    write(args.output_config, text)
    write(args.output_json, json.dumps(config, indent=2, sort_keys=True) + "\n")
    print(json.dumps(config, indent=2, sort_keys=True))


def audit(args):
    base = args.base_config.read_text()
    texts = {group: materialize(base, group, args.baseline_file)
             for group in GROUPS}
    configs = {group: effective_config(
        texts[group], group, args.baseline_file, args.provenance_file)
        for group in GROUPS}
    relevant = ("enableParameterUpdate", "enableAdaptiveDynamics",
                "enableClfConstraint", "enableInputBiasCost",
                "enableEncodedReferenceDerivatives",
                "enableCompositePhaseBaselineCorrection",
                "truePlantPayloadInertialEnabled")
    expected = {
        "nominal_original__nominal_phase_corrected":
            ["enableCompositePhaseBaselineCorrection"],
        "payload_original__payload_phase_corrected":
            ["enableCompositePhaseBaselineCorrection"],
        "nominal_original__payload_original":
            ["truePlantPayloadInertialEnabled"],
        "nominal_phase_corrected__payload_phase_corrected":
            ["truePlantPayloadInertialEnabled"],
    }
    pairwise, diff_text = {}, []
    for name, expected_keys in expected.items():
        left, right = name.split("__")
        changed = [key for key in relevant
                   if configs[left][key] != configs[right][key]]
        pairwise[name] = {
            "changed_keys": changed,
            "expected_changed_keys": expected_keys,
            "passed": changed == expected_keys,
        }
        diff_text.append("===== {} vs {} =====\n".format(left, right))
        diff_text.extend(difflib.unified_diff(
            texts[left].splitlines(True), texts[right].splitlines(True),
            fromfile=left + ".info", tofile=right + ".info"))
    locked = {
        "enableParameterUpdate": True,
        "enableAdaptiveDynamics": True,
        "enableClfConstraint": True,
        "enableInputBiasCost": False,
        "enableEncodedReferenceDerivatives": True,
    }
    passed = (all(item["passed"] for item in pairwise.values()) and
              all(all(config[key] == value for key, value in locked.items())
                  for config in configs.values()) and
              len({config["compositePhaseBaselineSha256"]
                   for config in configs.values()}) == 1 and
              all(not config["compositePhaseBaselineTrainingUsesPayloadData"]
                  for config in configs.values()))
    result = {
        "schema_version": 1,
        "stage": "D5I4_four_mode_fairness_audit",
        "passed": passed,
        "locked_controller_values": locked,
        "allowed_independent_variables": frozen_protocol()[
            "allowed_independent_variables"],
        "pairwise": pairwise,
        "groups": configs,
    }
    args.output_dir.mkdir(parents=True, exist_ok=False)
    write(args.output_dir / "fairness_audit.json",
          json.dumps(result, indent=2, sort_keys=True) + "\n")
    write(args.output_dir / "effective_config_diff.txt", "".join(diff_text))
    write(args.output_dir / "frozen_d5i4_protocol.json", PROTOCOL.read_text())
    print(json.dumps(result, indent=2, sort_keys=True))
    if not passed:
        raise RuntimeError("D5I4 mode fairness audit failed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG)
    parser.add_argument("--baseline-file", type=Path, default=BASELINE)
    parser.add_argument("--provenance-file", type=Path, default=PROVENANCE)
    subparsers = parser.add_subparsers(dest="command", required=True)
    emit_parser = subparsers.add_parser("emit")
    emit_parser.add_argument("group", choices=tuple(GROUPS))
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
