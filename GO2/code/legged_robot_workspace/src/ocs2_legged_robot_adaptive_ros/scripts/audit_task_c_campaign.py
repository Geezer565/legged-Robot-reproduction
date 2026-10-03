#!/usr/bin/env python3
"""Audit Stage C3 acquisition fairness without computing Stage C4 metrics."""

import argparse
import hashlib
import json
import os
from pathlib import Path


GROUPS = ("nominal", "uncertain_adaptive")
RUN_NAMES = ("run_01", "run_02", "run_03")
ALLOWED_TASK_C_KEYS = {
    "group", "plant_model_id", "true_plant_mass_scale",
    "true_mass_kg", "payload_delta_kg",
}


def read_json(path):
    with path.open() as stream:
        return json.load(stream)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def normalized(mapping, removed_keys):
    return {key: value for key, value in mapping.items() if key not in removed_keys}


def require(condition, message, errors):
    if not condition:
        errors.append(message)


def audit(root):
    errors = []
    run_entries = []
    controller_configs = []
    task_c_configs = []
    schedules = []
    source_manifest_hashes = []
    adaptive_snapshot_hashes = []

    required_files = (
        "task_c_run.bag", "task_c_run.csv", "task_c_run.png",
        "task_c_run_summary.json", "effective_config.json",
        "effective_adaptive_settings.info", "task_c_effective_config.json",
        "experiment_schedule.json", "controller_model_info.json",
        "plant_model_info.json", "source_files.sha256", "terminal.log",
        "roslaunch.log", "rosbag.log", "roscore.log", "run_validation.log",
    )

    for group in GROUPS:
        for run_name in RUN_NAMES:
            run_dir = root / group / run_name
            missing = [name for name in required_files if not (run_dir / name).is_file()]
            require(not missing, "{} missing {}".format(run_dir, missing), errors)
            if missing:
                continue

            controller = read_json(run_dir / "effective_config.json")
            task_c = read_json(run_dir / "task_c_effective_config.json")
            schedule = read_json(run_dir / "experiment_schedule.json")
            summary = read_json(run_dir / "task_c_run_summary.json")
            controller_info = read_json(run_dir / "controller_model_info.json")
            plant_info = read_json(run_dir / "plant_model_info.json")

            expected_scale = 1.0 if group == "nominal" else 1.10
            expected_true_mass = 52.13485 * expected_scale
            expected_override = group != "nominal"
            expected_plant_id = (
                "nominal_ocs2_rollout_v1" if group == "nominal"
                else "srbd_mass_scalar_v1")

            require(summary.get("passed") is True,
                    "{} per-run validation did not pass".format(run_dir), errors)
            require(summary.get("stage") == "C3", "{} wrong stage".format(run_dir), errors)
            require(task_c.get("group") == group, "{} wrong group".format(run_dir), errors)
            require(task_c.get("controller_mode") == "full_adaptive",
                    "{} controller not Full Adaptive".format(run_dir), errors)
            require(abs(float(task_c.get("true_plant_mass_scale", -1)) - expected_scale) <= 1.0e-12,
                    "{} wrong plant scale".format(run_dir), errors)
            require(abs(float(task_c.get("true_mass_kg", -1)) - expected_true_mass) <= 1.0e-10,
                    "{} wrong true mass".format(run_dir), errors)
            require(controller.get("mode") == "full_adaptive"
                    and controller.get("enableParameterUpdate") is True
                    and controller.get("enableAdaptiveDynamics") is True
                    and controller.get("enableClfConstraint") is True
                    and controller.get("enableInputBiasCost") is False,
                    "{} adaptive controller flags changed".format(run_dir), errors)
            require(plant_info.get("override_active") is expected_override,
                    "{} wrong override status".format(run_dir), errors)
            require(plant_info.get("plant_model_id") == expected_plant_id,
                    "{} wrong plant model identity".format(run_dir), errors)
            require(controller_info.get("controller_model_id") == "nominal_urdf_srbd_v1",
                    "{} wrong controller model identity".format(run_dir), errors)
            require(not summary.get("missing_required_topics"),
                    "{} missing required topics".format(run_dir), errors)
            require(summary.get("all_recorded_numeric_values_finite") is True,
                    "{} contains non-finite values".format(run_dir), errors)
            require(summary.get("artificial_constant_wrench", {}).get("ok") is True,
                    "{} artificial wrench check failed".format(run_dir), errors)
            require(summary.get("target", {}).get("ok") is True,
                    "{} deterministic target check failed".format(run_dir), errors)

            controller_configs.append((str(run_dir), controller))
            task_c_configs.append((str(run_dir), task_c))
            schedules.append((str(run_dir), schedule))
            source_manifest_hashes.append((str(run_dir), sha256(run_dir / "source_files.sha256")))
            adaptive_snapshot_hashes.append((str(run_dir), sha256(run_dir / "effective_adaptive_settings.info")))
            run_entries.append({
                "group": group,
                "run": run_name,
                "path": str(run_dir),
                "passed": summary.get("passed"),
                "bag_size_bytes": (run_dir / "task_c_run.bag").stat().st_size,
                "bag_sha256": sha256(run_dir / "task_c_run.bag"),
                "summary_sha256": sha256(run_dir / "task_c_run_summary.json"),
                "source_manifest_sha256": source_manifest_hashes[-1][1],
                "controller_config_sha256": controller.get("config_sha256"),
                "fixed_file_hashes": {
                    key: value["sha256"] for key, value in controller["fixed_files"].items()
                },
                "true_plant_mass_scale": task_c["true_plant_mass_scale"],
                "true_mass_kg": task_c["true_mass_kg"],
            })

    if controller_configs:
        controller_reference = controller_configs[0][1]
        for path, config in controller_configs[1:]:
            require(config == controller_reference,
                    "{} controller effective config differs".format(path), errors)
    require(len({value for _, value in source_manifest_hashes}) == 1,
            "source_files.sha256 differs across runs", errors)
    require(len({value for _, value in adaptive_snapshot_hashes}) == 1,
            "effective adaptive settings differ across runs", errors)

    if task_c_configs:
        normalized_reference = normalized(task_c_configs[0][1], ALLOWED_TASK_C_KEYS)
        for path, config in task_c_configs[1:]:
            require(normalized(config, ALLOWED_TASK_C_KEYS) == normalized_reference,
                    "{} has Task C config differences outside independent variable".format(path), errors)
        for group in GROUPS:
            group_values = [config for path, config in task_c_configs
                            if config.get("group") == group]
            require(all(config == group_values[0] for config in group_values[1:]),
                    "{} repeated-run Task C config differs".format(group), errors)

    if schedules:
        def normalized_schedule(value):
            output = dict(value)
            output.pop("plant_mass_override", None)
            return output
        schedule_reference = normalized_schedule(schedules[0][1])
        for path, schedule in schedules[1:]:
            require(normalized_schedule(schedule) == schedule_reference,
                    "{} schedule differs outside plant mass".format(path), errors)

    fairness = {
        "stage": "C3",
        "passed": not errors and len(run_entries) == 6,
        "groups": list(GROUPS),
        "runs_per_group": 3,
        "controller_mode_both_groups": "full_adaptive",
        "experimental_independent_variable": "true_plant_mass_scale",
        "nominal_scale": 1.0,
        "uncertain_scale": 1.10,
        "allowed_task_c_config_differences": sorted(ALLOWED_TASK_C_KEYS),
        "controller_effective_config_identical": bool(controller_configs) and not any(
            "controller effective config differs" in error for error in errors),
        "adaptive_settings_snapshot_identical": len({value for _, value in adaptive_snapshot_hashes}) == 1,
        "source_manifest_identical": len({value for _, value in source_manifest_hashes}) == 1,
        "schedule_identical_except_plant_mass": not any(
            "schedule differs" in error for error in errors),
        "all_per_run_validations_passed": all(entry["passed"] is True for entry in run_entries),
        "errors": errors,
        "scientific_scope": "Acquisition/fairness audit only; no Stage C4 paired performance metrics.",
    }
    manifest = {
        "stage": "C3",
        "passed": fairness["passed"],
        "acquisition_order": [
            "nominal/run_01", "uncertain_adaptive/run_01",
            "nominal/run_02", "uncertain_adaptive/run_02",
            "nominal/run_03", "uncertain_adaptive/run_03",
        ],
        "runs": run_entries,
        "fairness_audit": "stage_c3_fairness.json",
        "formal_analysis_status": "not_started_stage_c4",
    }
    return fairness, manifest


def main():
    parser = argparse.ArgumentParser(description="Audit Task C Stage C3 paired acquisition")
    parser.add_argument("root_directory")
    args = parser.parse_args()
    root = Path(args.root_directory).resolve()
    fairness, manifest = audit(root)
    with (root / "stage_c3_fairness.json").open("w") as stream:
        json.dump(fairness, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with (root / "stage_c_manifest.json").open("w") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with (root / "stage_c3_fairness.md").open("w") as stream:
        stream.write("# Task C Stage C3 Fairness Audit\n\n")
        stream.write("Result: **{}**\n\n".format("PASS" if fairness["passed"] else "FAIL"))
        stream.write("- Groups: nominal, uncertain_adaptive\n")
        stream.write("- Runs per group: 3\n")
        stream.write("- Controller mode: Full Adaptive in both groups\n")
        stream.write("- Only experimental variable: true_plant_mass_scale (1.00 vs 1.10)\n")
        stream.write("- Unified paired metrics: deferred to Stage C4\n")
        if fairness["errors"]:
            stream.write("\n## Errors\n\n")
            for error in fairness["errors"]:
                stream.write("- {}\n".format(error))
    print(json.dumps(fairness, indent=2, sort_keys=True))
    return 0 if fairness["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
