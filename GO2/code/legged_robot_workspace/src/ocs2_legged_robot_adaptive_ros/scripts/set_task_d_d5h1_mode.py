#!/usr/bin/env python3
"""Materialize and audit the four D5H1 estimator-robustness modes."""

import argparse
import difflib
import hashlib
import importlib.util
import json
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
BASE_CONFIG = (PACKAGE.parent / "ocs2_legged_robot_adaptive" / "config" /
               "adaptive_settings_payload_inertial_10d_smoke.info")
BASELINE = PACKAGE / "config" / "d5h0_nominal_gait_phase_baseline_v1.csv"
APPLICATION_PROVENANCE = (PACKAGE / "config" /
                          "d5h1_phase_baseline_application_v1.json")
TASK_D_SPEC = importlib.util.spec_from_file_location(
    "task_d_mode", str(PACKAGE / "scripts" / "set_task_d_mode.py"))
TASK_D = importlib.util.module_from_spec(TASK_D_SPEC)
TASK_D_SPEC.loader.exec_module(TASK_D)

GROUPS = {
    "nominal_raw": ("nominal", False),
    "nominal_corrected": ("nominal", True),
    "uncertain_raw": ("uncertain_adaptive", False),
    "uncertain_corrected": ("uncertain_adaptive", True),
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def insert_d5h1_settings(text, baseline_file, enabled):
    closing = text.rfind("}")
    if closing < 0:
        raise RuntimeError("adaptive settings have no closing block")
    value = "true" if enabled else "false"
    addition = (
        "\n  ; D5H1 default-off estimator robustness candidate.\n"
        "  enableGaitPhaseBaselineCorrection {}\n"
        "  gaitPhaseBaselineFile {}\n"
        "  gaitPhaseBins 24\n"
        "  gaitCycleDuration 1.2\n"
        "  gaitBaselineMinimumCycleStartTime 2.2\n".format(
            value, Path(baseline_file).resolve()))
    return text[:closing] + addition + text[closing:]


def materialize(base_text, group, baseline_file):
    task_d_group, enabled = GROUPS[group]
    task_d_text = TASK_D.materialize_text(base_text, task_d_group)
    return insert_d5h1_settings(task_d_text, baseline_file, enabled)


def effective_config(text, group, baseline_file):
    task_d_group, enabled = GROUPS[group]
    provenance = json.loads(APPLICATION_PROVENANCE.read_text())
    baseline_hash = sha256(baseline_file)
    if baseline_hash != provenance["baseline"]["packaged_canonical_lf_sha256"]:
        raise RuntimeError("D5H1 baseline differs from application provenance")
    if provenance["training"]["uncertain_or_payload_data_used"]:
        raise RuntimeError("D5H1 provenance reports payload data leakage")
    config = TASK_D.effective_config(text, task_d_group)
    config.update({
        "schema_version": 1,
        "stage": "D5H1_estimator_phase_baseline",
        "group": group,
        "task_d_parent_group": task_d_group,
        "enableGaitPhaseBaselineCorrection": enabled,
        "gaitPhaseBaselineFile": str(Path(baseline_file).resolve()),
        "gaitPhaseBaselineSha256": baseline_hash,
        "gaitPhaseBaselineApplicationProvenanceFile":
            str(APPLICATION_PROVENANCE.resolve()),
        "gaitPhaseBaselineApplicationProvenanceSha256":
            sha256(APPLICATION_PROVENANCE),
        "gaitPhaseBaselineTrainingUsesPayloadData": False,
        "gaitPhaseBins": 24,
        "gaitCycleDuration": 1.2,
        "gaitBaselineMinimumCycleStartTime": 2.2,
        "d5h1_applied_update_topic": "/task_d/d5h1_applied_update_sample",
        "d5h1_applied_update_schema":
            "payload_inertial_d5h1_applied_v1_71_fields",
        "d5h1_eq11_topic": "/task_d/d5h1_eq11_sample",
        "d5h1_eq11_schema":
            "payload_inertial_d5h1_eq11_v1_119_fields",
    })
    return config


def write(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)


def emit(args):
    text = materialize(args.base_config.read_text(), args.group,
                       args.baseline_file)
    config = effective_config(text, args.group, args.baseline_file)
    write(args.output_config, text)
    write(args.output_json, json.dumps(config, indent=2, sort_keys=True) + "\n")
    print(json.dumps(config, indent=2, sort_keys=True))


def audit(args):
    base = args.base_config.read_text()
    texts = {group: materialize(base, group, args.baseline_file)
             for group in GROUPS}
    configs = {group: effective_config(texts[group], group,
                                       args.baseline_file)
               for group in GROUPS}
    expected = {
        "nominal_raw__nominal_corrected":
            ["enableGaitPhaseBaselineCorrection"],
        "uncertain_raw__uncertain_corrected":
            ["enableGaitPhaseBaselineCorrection"],
        "nominal_raw__uncertain_raw":
            ["truePlantPayloadInertialEnabled"],
        "nominal_corrected__uncertain_corrected":
            ["truePlantPayloadInertialEnabled"],
    }
    relevant = ["enableParameterUpdate", "enableAdaptiveDynamics",
                "enableGaitPhaseBaselineCorrection",
                "truePlantPayloadInertialEnabled"]
    pairwise, diffs = {}, []
    for name, expected_keys in expected.items():
        left, right = name.split("__")
        changed = [key for key in relevant
                   if configs[left][key] != configs[right][key]]
        pairwise[name] = {
            "changed_keys": changed,
            "expected_changed_keys": expected_keys,
            "passed": changed == expected_keys,
        }
        diffs.append("===== {} vs {} =====\n".format(left, right))
        diffs.extend(difflib.unified_diff(
            texts[left].splitlines(True), texts[right].splitlines(True),
            fromfile=left + ".info", tofile=right + ".info"))
    baseline_hashes = {config["gaitPhaseBaselineSha256"]
                       for config in configs.values()}
    passed = all(item["passed"] for item in pairwise.values()) and \
        len(baseline_hashes) == 1
    result = {
        "schema_version": 1,
        "stage": "D5H1_mode_audit",
        "passed": passed,
        "groups": configs,
        "pairwise": pairwise,
        "baseline_hashes": sorted(baseline_hashes),
        "relevant_keys": relevant,
    }
    args.output_dir.mkdir(parents=True, exist_ok=False)
    write(args.output_dir / "fairness_audit.json",
          json.dumps(result, indent=2, sort_keys=True) + "\n")
    write(args.output_dir / "effective_config_diff.txt", "".join(diffs))
    print(json.dumps(result, indent=2, sort_keys=True))
    if not passed:
        raise RuntimeError("D5H1 mode fairness audit failed")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG)
    parser.add_argument("--baseline-file", type=Path, default=BASELINE)
    subparsers = parser.add_subparsers(dest="command", required=True)
    emit_parser = subparsers.add_parser("emit")
    emit_parser.add_argument("group", choices=sorted(GROUPS))
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
