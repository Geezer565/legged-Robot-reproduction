#!/usr/bin/env python3
"""Audit fairness and provenance of the six-run PayloadMass1D campaign."""

import argparse
import hashlib
import json
from pathlib import Path


GROUPS = ("nominal", "uncertain")
RUNS = ("run_01", "run_02", "run_03")


def load(path):
    return json.loads(path.read_text())


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def neutral_schedule(schedule):
    result = json.loads(json.dumps(schedule))
    result.pop("plant_mass", None)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records_root", type=Path)
    args = parser.parse_args()
    root = args.records_root.resolve()
    errors = []
    entries = []
    controllers = []
    neutral_schedules = []
    source_manifest_hashes = []

    required = (
        "payload_mass_smoke.bag", "payload_mass_smoke.csv",
        "payload_mass_smoke.png", "payload_mass_smoke_summary.json",
        "effective_adaptive_settings_payload_mass_1d.info",
        "effective_settings.sha256", "payload_mass_controller_config.json",
        "payload_mass_experiment_config.json", "experiment_schedule.json",
        "controller_model_info.json", "plant_model_info.json",
        "source_files.sha256", "artifact_checksums.sha256", "terminal.log",
        "roslaunch.log", "rosbag.log", "roscore.log", "run_validation.log",
    )

    for group in GROUPS:
        expected_scale = 1.0 if group == "nominal" else 1.1
        for run_name in RUNS:
            run = root / group / run_name
            missing = [name for name in required if not (run / name).is_file()]
            if missing:
                errors.append("{} missing {}".format(run, missing))
                continue
            controller = load(run / "payload_mass_controller_config.json")
            experiment = load(run / "payload_mass_experiment_config.json")
            schedule = load(run / "experiment_schedule.json")
            summary = load(run / "payload_mass_smoke_summary.json")
            controller_info = load(run / "controller_model_info.json")
            plant_info = load(run / "plant_model_info.json")

            if summary.get("passed") is not True:
                errors.append("{} per-run validation failed".format(run))
            if experiment.get("group") != group:
                errors.append("{} group identity mismatch".format(run))
            if abs(float(experiment.get("true_plant_mass_scale", -1.0)) - expected_scale) > 1.0e-12:
                errors.append("{} wrong true plant mass scale".format(run))
            settings = controller.get("adaptive_settings", {})
            expected_settings = {
                "enableParameterUpdate": True,
                "enableAdaptiveDynamics": True,
                "enableClfConstraint": True,
                "enableInputBiasCost": False,
                "useSoftConstraint": True,
                "parameterization": "payload_mass_1d",
                "payloadMassAdaptationGain": 5.0,
            }
            for name, value in expected_settings.items():
                if settings.get(name) != value:
                    errors.append("{} controller setting {} changed".format(run, name))
            if controller_info.get("controller_model_id") != "nominal_urdf_srbd_v1":
                errors.append("{} controller model identity changed".format(run))
            expected_override = group == "uncertain"
            if plant_info.get("override_active") is not expected_override:
                errors.append("{} plant override identity mismatch".format(run))
            if abs(float(plant_info.get("true_plant_mass_scale", -1.0)) - expected_scale) > 1.0e-12:
                errors.append("{} plant info scale mismatch".format(run))
            if abs(float(summary.get("max_snapshot_time_error_s", 1.0))) > 1.0e-6:
                errors.append("{} snapshot synchronization failed".format(run))
            for key in ("max_total_mass_identity_error", "max_mass_ratio_identity_error",
                        "max_eq10_mapping_error"):
                if abs(float(summary.get(key, 1.0))) > 1.0e-10:
                    errors.append("{} {} failed".format(run, key))

            controllers.append((str(run), controller))
            neutral_schedules.append((str(run), neutral_schedule(schedule)))
            source_manifest_hashes.append((str(run), digest(run / "source_files.sha256")))
            entries.append({
                "group": group,
                "run": run_name,
                "path": str(run),
                "bag_size_bytes": (run / "payload_mass_smoke.bag").stat().st_size,
                "bag_sha256": digest(run / "payload_mass_smoke.bag"),
                "summary_sha256": digest(run / "payload_mass_smoke_summary.json"),
                "source_manifest_sha256": source_manifest_hashes[-1][1],
                "adaptive_settings_sha256": digest(
                    run / "effective_adaptive_settings_payload_mass_1d.info"),
                "true_plant_mass_scale": expected_scale,
            })

    if controllers:
        reference_path, reference = controllers[0]
        for path, value in controllers[1:]:
            if value != reference:
                errors.append("controller effective config differs: {} vs {}".format(
                    reference_path, path))
    if neutral_schedules:
        reference_path, reference = neutral_schedules[0]
        for path, value in neutral_schedules[1:]:
            if value != reference:
                errors.append("mass-neutral schedule differs: {} vs {}".format(
                    reference_path, path))
    if len({value for _, value in source_manifest_hashes}) > 1:
        errors.append("source manifest changed during formal acquisition")
    if entries and len({entry["adaptive_settings_sha256"] for entry in entries}) > 1:
        errors.append("PayloadMass1D settings snapshot changed between runs")
    if len(entries) != 6:
        errors.append("expected six complete formal runs, found {}".format(len(entries)))

    report = {
        "passed": not errors,
        "scope": "PayloadMass1D formal paired-campaign fairness and provenance audit",
        "allowed_between_group_difference": ["true_plant_mass_scale", "true_mass_kg",
                                              "true_payload_delta_kg", "plant_model_id",
                                              "override_active", "group"],
        "controller_configuration_identical": bool(controllers) and
            all(value == controllers[0][1] for _, value in controllers),
        "mass_neutral_schedule_identical": bool(neutral_schedules) and
            all(value == neutral_schedules[0][1] for _, value in neutral_schedules),
        "source_manifest_identical": bool(source_manifest_hashes) and
            len({value for _, value in source_manifest_hashes}) == 1,
        "run_count": len(entries),
        "runs": entries,
        "errors": errors,
    }
    (root / "fairness_audit.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    with (root / "fairness_audit.md").open("w") as stream:
        stream.write("# PayloadMass1D formal fairness audit\n\n")
        stream.write("- Result: **{}**\n".format("PASS" if report["passed"] else "FAIL"))
        stream.write("- Complete runs: {} / 6\n".format(len(entries)))
        stream.write("- Controller config identical: {}\n".format(
            report["controller_configuration_identical"]))
        stream.write("- Target/schedule identical after removing plant mass: {}\n".format(
            report["mass_neutral_schedule_identical"]))
        stream.write("- Source manifest identical: {}\n".format(
            report["source_manifest_identical"]))
        stream.write("- Only scientific independent variable: true plant mass scale 1.00 vs 1.10.\n")
        if errors:
            stream.write("\n## Errors\n\n")
            for error in errors:
                stream.write("- {}\n".format(error))
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
