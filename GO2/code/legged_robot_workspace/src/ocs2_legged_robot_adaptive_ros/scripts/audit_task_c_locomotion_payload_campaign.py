#!/usr/bin/env python3
"""Audit the nine-run deterministic static-walk PayloadMass1D campaign."""

import argparse
import hashlib
import json
import re
from pathlib import Path


GROUPS = ("nominal", "uncertain", "uncertain_off")
RUNS = ("run_01", "run_02", "run_03")
MODE_KEYS = ("enableParameterUpdate", "enableAdaptiveDynamics")


def load(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def neutral_settings(text):
    for key in MODE_KEYS:
        text, count = re.subn(r"^(\s*" + key + r"\s+)(true|false)",
                              r"\1<MODE>", text, flags=re.MULTILINE)
        if count != 1:
            raise RuntimeError("cannot normalize " + key)
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records_root", type=Path)
    args = parser.parse_args()
    root = args.records_root.resolve()
    errors, entries = [], []
    source_hashes, neutral_hashes, fixed_hash_sets, schedules = [], [], [], []
    required = (
        "payload_mass_smoke.bag", "payload_mass_smoke.csv",
        "payload_mass_smoke_summary.json", "payload_mass_smoke.png",
        "locomotion_smoke_summary.json", "locomotion_smoke.png",
        "locomotion_smoke_observation.csv", "payload_mass_controller_config.json",
        "payload_mass_experiment_config.json", "experiment_schedule.json",
        "effective_adaptive_settings_payload_mass_1d.info",
        "controller_model_info.json", "plant_model_info.json",
        "source_files.sha256", "artifact_checksums.sha256",
        "run_validation.log", "locomotion_validation.log", "roslaunch.log", "rosbag.log",
    )

    for group in GROUPS:
        for run_name in RUNS:
            run = root / group / run_name
            missing = [name for name in required if not (run / name).is_file()]
            if missing:
                errors.append("{} missing {}".format(run, missing))
                continue
            payload = load(run / "payload_mass_smoke_summary.json")
            locomotion = load(run / "locomotion_smoke_summary.json")
            controller = load(run / "payload_mass_controller_config.json")
            experiment = load(run / "payload_mass_experiment_config.json")
            schedule = load(run / "experiment_schedule.json")
            plant = load(run / "plant_model_info.json")
            adaptive = controller.get("adaptive_settings", {})
            expected_on = group != "uncertain_off"
            expected_scale = 1.0 if group == "nominal" else 1.1

            if payload.get("passed") is not True or \
                    payload.get("analysis_window_pairing_complete") is not True:
                errors.append("{} payload validation failed".format(run))
            if locomotion.get("passed_structural_validation") is not True or \
                    locomotion.get("locomotion_confirmed") is not True:
                errors.append("{} locomotion validation failed".format(run))
            target = locomotion.get("target", {})
            if abs(float(target.get("start_time_s", -1.0)) - 1.20) > 1.0e-9 or \
                    abs(float(target.get("end_time_s", -1.0)) - 1.40) > 1.0e-9 or \
                    abs(float(target.get("start_x_m", -1.0))) > 1.0e-8 or \
                    abs(float(target.get("final_x_m", -1.0)) - 0.10) > 1.0e-8:
                errors.append("{} target trajectory is not fixed 0->0.10 m".format(run))
            if sorted(locomotion.get("non_stance_modes", [])) != [7, 11, 13, 14] or \
                    int(locomotion.get("swinging_foot_count", 0)) != 4:
                errors.append("{} executed gait evidence incomplete".format(run))
            if experiment.get("group") != group or \
                    abs(float(experiment.get("true_plant_mass_scale", -1.0)) - expected_scale) > 1.0e-12:
                errors.append("{} group/plant identity mismatch".format(run))
            if adaptive.get("enableParameterUpdate") is not expected_on or \
                    adaptive.get("enableAdaptiveDynamics") is not expected_on:
                errors.append("{} adaptive mode flags mismatch".format(run))
            locked = {
                "enableClfConstraint": True, "enableInputBiasCost": False,
                "useSoftConstraint": True, "softConstraintMu": 0.1,
                "softConstraintDelta": 5.0, "parameterization": "payload_mass_1d",
                "payloadMassAdaptationGain": 5.0, "minimumEstimatedTotalMass": 1.0,
            }
            for key, expected in locked.items():
                if adaptive.get(key) != expected:
                    errors.append("{} locked setting {} changed".format(run, key))
            if plant.get("override_active") is not (group != "nominal"):
                errors.append("{} plant override identity mismatch".format(run))
            if group == "uncertain_off" and (
                    abs(float(payload.get("payload_mass_estimate_last_kg", 1.0))) > 1.0e-12 or
                    abs(float(payload.get("peak_applied_correction_norm", 1.0))) > 1.0e-12):
                errors.append("{} OFF estimate/applied correction nonzero".format(run))

            fixed = controller["fixed_files"]
            fixed_hash_sets.append({name: fixed[name]["sha256"]
                                    for name in ("mpc_task", "reference", "gait", "urdf")})
            schedules.append({name: schedule[name]
                              for name in ("target", "gait", "analysis_window",
                                           "artificial_disturbance")})
            source_hash = digest(run / "source_files.sha256")
            settings_text = (run / "effective_adaptive_settings_payload_mass_1d.info").read_text()
            neutral_hash = hashlib.sha256(neutral_settings(settings_text).encode()).hexdigest()
            source_hashes.append(source_hash)
            neutral_hashes.append(neutral_hash)
            entries.append({
                "group": group, "run": run_name, "path": str(run),
                "true_plant_mass_scale": expected_scale,
                "enableParameterUpdate": expected_on,
                "enableAdaptiveDynamics": expected_on,
                "source_manifest_sha256": source_hash,
                "mode_neutral_settings_sha256": neutral_hash,
                "bag_sha256": digest(run / "payload_mass_smoke.bag"),
            })

    checks = {
        "complete_run_count": len(entries) == 9,
        "source_manifest_identical_at_acquisition": len(set(source_hashes)) == 1,
        "mode_neutral_adaptive_settings_identical": len(set(neutral_hashes)) == 1,
        "controller_urdf_task_reference_gait_hashes_identical":
            bool(fixed_hash_sets) and all(value == fixed_hash_sets[0] for value in fixed_hash_sets),
        "target_gait_analysis_schedule_identical":
            bool(schedules) and all(value == schedules[0] for value in schedules),
        "all_runs_confirm_executed_static_walk": len(entries) == 9 and not any(
            "locomotion" in error or "gait" in error for error in errors),
    }
    for name, passed in checks.items():
        if not passed:
            errors.append(name + " failed")
    report = {
        "passed": not errors,
        "scope": "nine-run deterministic static-walk PayloadMass1D fairness audit",
        "checks": checks,
        "allowed_differences": [
            "true_plant_mass_scale/true mass/plant identity between nominal and uncertain",
            "enableParameterUpdate and enableAdaptiveDynamics between uncertain ON and OFF",
        ],
        "fixed_reference": "absolute x 0.00->0.10 m at MPC time 1.20->1.40 s",
        "acquisition_order": "round-robin N, U-OFF, U-ON for each repetition",
        "runs": entries,
        "errors": errors,
    }
    (root / "fairness_audit_adaptive_off.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n")
    with (root / "fairness_audit_adaptive_off.md").open("w") as stream:
        stream.write("# Static-walk PayloadMass1D fairness audit\n\n")
        stream.write("- Result: **{}**\n".format("PASS" if report["passed"] else "FAIL"))
        stream.write("- Complete runs: {} / 9\n".format(len(entries)))
        stream.write("- Reference: {}\n".format(report["fixed_reference"]))
        for name, passed in checks.items():
            stream.write("- {}: {}\n".format(name, passed))
        if errors:
            stream.write("\n## Errors\n\n")
            for error in errors:
                stream.write("- {}\n".format(error))
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
