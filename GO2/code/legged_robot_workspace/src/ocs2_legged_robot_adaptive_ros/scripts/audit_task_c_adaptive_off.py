#!/usr/bin/env python3
"""Audit the Task C three-group Adaptive-OFF campaign extension."""

import argparse
import hashlib
import json
from pathlib import Path


GROUPS = ("nominal", "uncertain_adaptive", "uncertain_off")
RUNS = ("run_01", "run_02", "run_03")
EXPECTED = {
    "nominal": {"scale": 1.0, "mode": "full_adaptive", "update": True, "dynamics": True},
    "uncertain_adaptive": {"scale": 1.10, "mode": "full_adaptive", "update": True, "dynamics": True},
    "uncertain_off": {"scale": 1.10, "mode": "nominal", "update": False, "dynamics": False},
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


def source_hashes(path):
    values = {}
    for line in path.read_text().splitlines():
        digest, filename = line.split(None, 1)
        values[filename.strip()] = digest
    return values


def controller_runtime_hashes(values):
    selected = {}
    for filename, digest in values.items():
        normalized = filename.replace("\\", "/")
        is_core_code = "/ocs2_legged_robot_adaptive/src/" in normalized \
            or "/ocs2_legged_robot_adaptive/include/" in normalized
        is_ros_code = "/ocs2_legged_robot_adaptive_ros/src/" in normalized \
            or "/ocs2_legged_robot_adaptive_ros/include/" in normalized
        if is_core_code or is_ros_code:
            selected[normalized.split("/catkin_ws/src/", 1)[-1]] = digest
    return selected


def normalized_schedule(value):
    output = dict(value)
    output.pop("stage", None)
    return output


def require(condition, message, errors):
    if not condition:
        errors.append(message)


def main():
    parser = argparse.ArgumentParser(description="Audit Task C uncertain Adaptive-OFF extension")
    parser.add_argument("records_directory")
    args = parser.parse_args()
    root = Path(args.records_directory).resolve()
    errors = []
    entries = []
    mode_neutral_hashes = set()
    fixed_files = []
    runtime_sources = []
    uncertain_schedules = []
    uncertain_plant_configs = []
    off_source_manifests = []
    required_files = (
        "task_c_run.bag", "task_c_run.csv", "task_c_run.png",
        "task_c_run_summary.json", "effective_config.json",
        "effective_adaptive_settings.info", "task_c_effective_config.json",
        "experiment_schedule.json", "controller_model_info.json",
        "plant_model_info.json", "source_files.sha256", "terminal.log",
        "roslaunch.log", "rosbag.log", "roscore.log", "run_validation.log",
    )

    for group in GROUPS:
        expected = EXPECTED[group]
        for run_name in RUNS:
            directory = root / group / run_name
            missing = [name for name in required_files if not (directory / name).is_file()]
            require(not missing, "{} missing {}".format(directory, missing), errors)
            if missing:
                continue
            validation = read_json(directory / "task_c_run_summary.json")
            controller = read_json(directory / "effective_config.json")
            experiment = read_json(directory / "task_c_effective_config.json")
            schedule = read_json(directory / "experiment_schedule.json")
            plant = read_json(directory / "plant_model_info.json")
            source = source_hashes(directory / "source_files.sha256")
            expected_mass = 52.13485 * expected["scale"]

            require(validation.get("passed") is True,
                    "{} per-run validation failed".format(directory), errors)
            require(experiment.get("group") == group,
                    "{} group mismatch".format(directory), errors)
            require(controller.get("mode") == expected["mode"],
                    "{} controller mode mismatch".format(directory), errors)
            require(controller.get("enableParameterUpdate") is expected["update"]
                    and controller.get("enableAdaptiveDynamics") is expected["dynamics"],
                    "{} adaptive flags mismatch".format(directory), errors)
            require(controller.get("enableClfConstraint") is True
                    and controller.get("enableInputBiasCost") is False,
                    "{} locked CLF/input-bias settings changed".format(directory), errors)
            require(abs(float(experiment.get("true_plant_mass_scale", -1.0))
                        - expected["scale"]) <= 1.0e-12,
                    "{} mass scale mismatch".format(directory), errors)
            require(abs(float(experiment.get("true_mass_kg", -1.0))
                        - expected_mass) <= 1.0e-9,
                    "{} true mass mismatch".format(directory), errors)
            require(plant.get("plant_model_id") == (
                        "nominal_ocs2_rollout_v1" if group == "nominal"
                        else "srbd_mass_scalar_v1"),
                    "{} plant identity mismatch".format(directory), errors)
            require(validation.get("all_recorded_numeric_values_finite") is True,
                    "{} contains non-finite values".format(directory), errors)
            require(not validation.get("missing_required_topics"),
                    "{} missing required topics".format(directory), errors)
            require(validation.get("target", {}).get("ok") is True
                    and validation.get("ground_truth", {}).get("ok") is True,
                    "{} target/ground-truth validation failed".format(directory), errors)

            mode_neutral_hashes.add(controller["mode_neutral_config_sha256"])
            fixed_files.append({key: value["sha256"]
                                for key, value in controller["fixed_files"].items()})
            runtime_sources.append(controller_runtime_hashes(source))
            if group != "nominal":
                uncertain_schedules.append(normalized_schedule(schedule))
                uncertain_plant_configs.append({
                    key: experiment[key] for key in (
                        "controller_model_id", "plant_model_id", "nominal_mass_kg",
                        "true_plant_mass_scale", "true_mass_kg", "payload_delta_kg",
                        "task_c_diagnostics_enabled", "artificial_constant_wrench_enabled",
                        "artificial_constant_wrench_world", "analysis_window_start_mpc_time_s",
                        "analysis_window_duration_s", "experiment_end_mpc_time_s")})
            if group == "uncertain_off":
                off_source_manifests.append(sha256(directory / "source_files.sha256"))

            entries.append({
                "group": group,
                "run": run_name,
                "path": str(directory),
                "passed": validation.get("passed"),
                "controller_mode": controller["mode"],
                "enableParameterUpdate": controller["enableParameterUpdate"],
                "enableAdaptiveDynamics": controller["enableAdaptiveDynamics"],
                "true_plant_mass_scale": experiment["true_plant_mass_scale"],
                "true_mass_kg": experiment["true_mass_kg"],
                "bag_sha256": sha256(directory / "task_c_run.bag"),
                "summary_sha256": sha256(directory / "task_c_run_summary.json"),
                "source_manifest_sha256": sha256(directory / "source_files.sha256"),
            })

    require(len(entries) == 9, "expected 9 complete formal runs", errors)
    require(len(mode_neutral_hashes) == 1,
            "mode-neutral adaptive config differs across groups", errors)
    require(bool(fixed_files) and all(value == fixed_files[0] for value in fixed_files[1:]),
            "URDF/reference/task/gait hashes differ", errors)
    require(bool(runtime_sources) and all(value == runtime_sources[0] for value in runtime_sources[1:]),
            "controller C++/header runtime source differs across acquisition stages", errors)
    require(bool(uncertain_schedules)
            and all(value == uncertain_schedules[0] for value in uncertain_schedules[1:]),
            "uncertain ON/OFF schedule differs", errors)
    require(bool(uncertain_plant_configs)
            and all(value == uncertain_plant_configs[0] for value in uncertain_plant_configs[1:]),
            "uncertain ON/OFF plant/reference experiment config differs", errors)
    require(len(set(off_source_manifests)) == 1,
            "uncertain_off source manifest changed between repetitions", errors)

    fairness = {
        "stage": "C5",
        "passed": not errors,
        "groups": list(GROUPS),
        "runs_per_group": 3,
        "comparison_of_interest": "uncertain_adaptive versus uncertain_off",
        "true_plant_mass_scale_both_uncertain_groups": 1.10,
        "allowed_controller_differences": [
            "enableParameterUpdate", "enableAdaptiveDynamics", "mode", "config_sha256"],
        "mode_neutral_adaptive_config_identical": len(mode_neutral_hashes) == 1,
        "fixed_urdf_reference_task_gait_hashes_identical": bool(fixed_files)
            and all(value == fixed_files[0] for value in fixed_files[1:]),
        "controller_runtime_cpp_headers_identical": bool(runtime_sources)
            and all(value == runtime_sources[0] for value in runtime_sources[1:]),
        "uncertain_on_off_schedule_identical": bool(uncertain_schedules)
            and all(value == uncertain_schedules[0] for value in uncertain_schedules[1:]),
        "uncertain_on_off_plant_config_identical": bool(uncertain_plant_configs)
            and all(value == uncertain_plant_configs[0] for value in uncertain_plant_configs[1:]),
        "off_repetition_source_manifest_identical": len(set(off_source_manifests)) == 1,
        "full_package_source_manifest_identical_across_old_and_new_stages": False,
        "full_package_source_manifest_note": (
            "Expected false: C4/C5 offline analysis and acquisition scripts changed after the "
            "frozen C3 runs. Controller runtime C++/headers and fixed model/config hashes are audited identical."),
        "errors": errors,
    }
    manifest = {
        "stage": "C5",
        "passed": fairness["passed"],
        "acquisition_extension_order": [
            "uncertain_off/run_01", "uncertain_off/run_02", "uncertain_off/run_03"],
        "runs": entries,
        "fairness_audit": "stage_c5_fairness.json",
        "formal_analysis_status": "pending" if fairness["passed"] else "blocked",
    }
    (root / "stage_c5_fairness.json").write_text(
        json.dumps(fairness, indent=2, sort_keys=True) + "\n")
    (root / "stage_c5_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(fairness, indent=2, sort_keys=True))
    return 0 if fairness["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
