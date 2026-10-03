#!/usr/bin/env python3
"""Validate one Stage C2 mass-mismatch smoke run without a performance claim."""

import argparse
import bisect
import csv
import json
import math
import os


COMPONENTS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
OBSERVATION_TOPIC = "/legged_robot_mpc_observation"
TARGET_TOPIC = "/legged_robot_mpc_target"
UPDATE_TOPIC = "/adaptive_update/sample"
DISTURBANCE_TOPIC = "/adaptive_update/disturbance_sample"
EQUATION10_TOPIC = "/task_b/equation10_sample"
MODEL_IDENTITY_TOPIC = "/task_c/model_identity"
MODEL_MISMATCH_TOPIC = "/task_c/model_mismatch_sample"

REQUIRED_TOPICS = {
    OBSERVATION_TOPIC,
    TARGET_TOPIC,
    UPDATE_TOPIC,
    DISTURBANCE_TOPIC,
    EQUATION10_TOPIC,
    MODEL_IDENTITY_TOPIC,
    MODEL_MISMATCH_TOPIC,
    "/task_b/policy_input_sample",
    "/aclf_experiment/x_error",
    "/aclf_experiment/position_error_norm",
    "/aclf_experiment/executed_sigma_norm",
    "/aclf_experiment/executed_observation_time",
    "/aclf_experiment/executed_snapshot_time",
    "/aclf_experiment/executed_time_error",
}


def finite(values):
    return all(math.isfinite(float(value)) for value in values)


def norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def nearest_index(times, value):
    index = bisect.bisect_left(times, value)
    candidates = []
    if index < len(times):
        candidates.append(index)
    if index:
        candidates.append(index - 1)
    return min(candidates, key=lambda item: abs(times[item] - value))


def interpolate(time, times, states):
    if time <= times[0]:
        return states[0]
    if time >= times[-1]:
        return states[-1]
    index = bisect.bisect_left(times, time)
    left, right = index - 1, index
    alpha = (times[right] - time) / (times[right] - times[left])
    return [alpha * a + (1.0 - alpha) * b
            for a, b in zip(states[left], states[right])]


def numeric_values(topic, message):
    if topic == OBSERVATION_TOPIC:
        return [message.time, float(message.mode)] + list(message.state.value) + list(message.input.value)
    if topic == TARGET_TOPIC:
        values = list(message.timeTrajectory)
        for state in message.stateTrajectory:
            values.extend(state.value)
        for control in message.inputTrajectory:
            values.extend(control.value)
        return values
    if topic == MODEL_IDENTITY_TOPIC:
        return []
    data = getattr(message, "data", None)
    if data is None:
        return []
    if isinstance(data, (list, tuple)):
        return list(data)
    return [data]


def load_bag(path):
    import rosbag

    data = {
        "updates": [], "disturbances": [], "equation10": [],
        "model_identity": [], "model_mismatch": [], "observations": [],
        "targets": [], "topic_counts": {}, "nonfinite_topics": [],
    }
    with rosbag.Bag(path, "r") as bag:
        info = bag.get_type_and_topic_info().topics
        data["topic_counts"] = {topic: value.message_count for topic, value in info.items()}
        for topic, message, stamp in bag.read_messages():
            values = numeric_values(topic, message)
            if values and not finite(values):
                data["nonfinite_topics"].append(topic)
            if topic == UPDATE_TOPIC:
                data["updates"].append(list(message.data))
            elif topic == DISTURBANCE_TOPIC:
                data["disturbances"].append(list(message.data))
            elif topic == EQUATION10_TOPIC:
                data["equation10"].append(list(message.data))
            elif topic == MODEL_IDENTITY_TOPIC:
                data["model_identity"].append(json.loads(message.data))
            elif topic == MODEL_MISMATCH_TOPIC:
                data["model_mismatch"].append(list(message.data))
            elif topic == OBSERVATION_TOPIC:
                data["observations"].append({
                    "bag_time": stamp.to_sec(),
                    "time": float(message.time),
                    "mode": int(message.mode),
                    "state": list(message.state.value),
                    "input": list(message.input.value),
                })
            elif topic == TARGET_TOPIC:
                data["targets"].append({
                    "bag_time": stamp.to_sec(),
                    "times": list(message.timeTrajectory),
                    "states": [list(state.value) for state in message.stateTrajectory],
                })
    data["nonfinite_topics"] = sorted(set(data["nonfinite_topics"]))
    return data


def validate_and_align(data, adaptive_config, task_c_config):
    updates = sorted(([float(value) for value in row] for row in data["updates"]), key=lambda row: row[0])
    disturbances = sorted(([float(value) for value in row] for row in data["disturbances"]), key=lambda row: row[0])
    equation10 = sorted(([float(value) for value in row] for row in data["equation10"]), key=lambda row: row[0])
    mismatch = sorted(([float(value) for value in row] for row in data["model_mismatch"]), key=lambda row: row[0])
    observations = sorted(data["observations"], key=lambda row: row["time"])
    targets = data["targets"]

    if not all((updates, disturbances, equation10, mismatch, observations, targets, data["model_identity"])):
        raise RuntimeError("Task C run bag 缺少核心数据")
    if any(len(row) != 16 or not finite(row) for row in updates):
        raise RuntimeError("adaptive update sample schema/finite check failed")
    if any(len(row) != 8 or not finite(row) for row in disturbances):
        raise RuntimeError("disturbance sample schema/finite check failed")
    if any(len(row) != 24 or not finite(row) for row in equation10):
        raise RuntimeError("Eq.(10) sample schema/finite check failed")
    if any(len(row) != 41 or not finite(row) for row in mismatch):
        raise RuntimeError("Task C model mismatch sample must have 41 finite fields")
    if len(targets) != 1 or len(targets[0]["times"]) != 2 or len(targets[0]["states"]) != 2:
        raise RuntimeError("Task C run requires exactly one deterministic two-knot target")

    identity = data["model_identity"][-1]
    expected_nominal = float(task_c_config["nominal_mass_kg"])
    expected_scale = float(task_c_config["true_plant_mass_scale"])
    expected_true = expected_nominal * expected_scale
    expected_override = abs(expected_scale - 1.0) > 1.0e-12
    expected_controller_model_id = task_c_config.get(
        "controller_model_id", "nominal_urdf_srbd_v1")
    expected_plant_model_id = task_c_config.get(
        "plant_model_id",
        "srbd_mass_scalar_v1" if expected_override else "nominal_ocs2_rollout_v1")
    identity_ok = (
        identity.get("schema_version") == 1
        and identity.get("controller_model_id") == expected_controller_model_id
        and identity.get("plant_model_id") == expected_plant_model_id
        and abs(float(identity.get("nominal_mass", math.nan)) - expected_nominal) <= 1.0e-10
        and abs(float(identity.get("true_mass", math.nan)) - expected_true) <= 1.0e-10
        and abs(float(identity.get("true_plant_mass_scale", math.nan)) - expected_scale) <= 1.0e-12
        and identity.get("override_active") is expected_override
        and identity.get("task_c_diagnostics_enabled") is True
    )

    expected_task_b_mode = task_c_config.get("task_b_mode", "full_adaptive")
    expected_update = expected_task_b_mode == "full_adaptive"
    expected_dynamics = expected_task_b_mode == "full_adaptive"
    config_ok = (
        adaptive_config.get("mode") == expected_task_b_mode
        and adaptive_config.get("enableParameterUpdate") is expected_update
        and adaptive_config.get("enableAdaptiveDynamics") is expected_dynamics
        and adaptive_config.get("enableClfConstraint") is True
        and adaptive_config.get("enableInputBiasCost") is False
        and task_c_config.get("artificial_constant_wrench_enabled") is False
        and task_c_config.get("task_c_diagnostics_enabled") is True
    )

    mass_consistency_error = 0.0
    invalid_model_rows = 0
    max_reconstruction_error = 0.0
    max_pi_star_norm = 0.0
    max_residual_norm = 0.0
    max_full_residual_norm = 0.0
    max_unmatched_residual_norm = 0.0
    nonzero_pi_star_count = 0
    for row in mismatch:
        mass_consistency_error = max(
            mass_consistency_error,
            abs(row[1] - expected_nominal), abs(row[2] - expected_true),
            abs(row[3] - (expected_true - expected_nominal)),
            abs(row[4] - expected_scale),
            abs(row[5] - (1.0 if expected_override else 0.0)))
        invalid_model_rows += int(round(row[40])) != 1
        max_reconstruction_error = max(max_reconstruction_error, max(abs(value) for value in row[30:36]), abs(row[39]))
        current_pi_star_norm = norm(row[24:30])
        max_pi_star_norm = max(max_pi_star_norm, current_pi_star_norm)
        max_residual_norm = max(max_residual_norm, norm(row[18:24]))
        max_full_residual_norm = max(max_full_residual_norm, abs(row[36]))
        max_unmatched_residual_norm = max(max_unmatched_residual_norm, abs(row[37]))
        if current_pi_star_norm > 1.0e-8:
            nonzero_pi_star_count += 1
    common_ground_truth_ok = (
        mass_consistency_error <= 1.0e-9
        and invalid_model_rows == 0
        and max_reconstruction_error <= 1.0e-10
    )
    if expected_override:
        ground_truth_path_ok = (
            max_pi_star_norm > 1.0e-6
            and max_residual_norm > 1.0e-9
            and nonzero_pi_star_count > 0
        )
    else:
        ground_truth_path_ok = (
            max_pi_star_norm <= 1.0e-10
            and max_residual_norm <= 1.0e-12
            and max_full_residual_norm <= 1.0e-12
            and max_unmatched_residual_norm <= 1.0e-12
            and nonzero_pi_star_count == 0
        )
    ground_truth_ok = common_ground_truth_ok and ground_truth_path_ok

    versions = [int(round(row[3])) for row in updates]
    max_snapshot_time_error = max(abs(row[2]) for row in updates)
    snapshot_base_ok = (
        max_snapshot_time_error <= 1.0e-9
        and all(right > left for left, right in zip(versions, versions[1:]))
        and len(updates) >= 50
    )

    eq_by_version = {int(round(row[3])): row for row in equation10}
    equation10_versions = sorted(eq_by_version)
    unmatched_update_versions = [
        int(round(row[3])) for row in updates
        if int(round(row[3])) not in eq_by_version
    ]
    # ROS publishers/subscribers do not start atomically.  A bounded prefix of
    # update samples may therefore precede the first Eq.(10) diagnostic, but a
    # hole after Eq.(10) recording begins would indicate a broken applied path.
    diagnostic_coverage_ok = (
        len(unmatched_update_versions) <= 5
        and all(version < equation10_versions[0]
                for version in unmatched_update_versions)
        and len(updates) - len(unmatched_update_versions) >= 50
    )
    mismatch_times = [row[0] for row in mismatch]
    observation_times = [row["time"] for row in observations]
    rows = []
    mode_flags = set()
    max_eq_time_error = 0.0
    max_ground_truth_pair_error = 0.0
    max_observation_pair_error = 0.0
    max_yu_pi_error = 0.0
    max_candidate_formula_error = 0.0
    max_applied_candidate_error = 0.0
    for update in updates:
        version = int(round(update[3]))
        equation = eq_by_version.get(version)
        if equation is None:
            continue
        max_eq_time_error = max(max_eq_time_error, abs(equation[0] - update[0]))
        mode_flags.add((int(round(equation[4])), int(round(equation[5]))))
        max_yu_pi_error = max(max_yu_pi_error, max(abs(a - b) for a, b in zip(equation[6:12], update[4:10])))
        expected_candidate = [-value / expected_nominal for value in update[4:10]]
        max_candidate_formula_error = max(
            max_candidate_formula_error,
            max(abs(a - b) for a, b in zip(equation[12:18], expected_candidate)))
        max_applied_candidate_error = max(
            max_applied_candidate_error,
            max(abs(a - b) for a, b in zip(equation[18:24], equation[12:18])))

        model_index = nearest_index(mismatch_times, update[0])
        model = mismatch[model_index]
        max_ground_truth_pair_error = max(max_ground_truth_pair_error, abs(model[0] - update[0]))
        observation_index = nearest_index(observation_times, update[0])
        observation = observations[observation_index]
        max_observation_pair_error = max(max_observation_pair_error, abs(observation["time"] - update[0]))
        desired = interpolate(update[0], targets[0]["times"], targets[0]["states"])

        row = {
            "observation_time": update[0],
            "snapshot_time": update[1],
            "snapshot_version": version,
            "model_sample_time": model[0],
            "nominal_mass": model[1],
            "true_mass": model[2],
            "payload_delta": model[3],
            "mass_scale": model[4],
            "contact_mode": observation["mode"],
            "desired_x": desired[6],
            "actual_x": observation["state"][6],
            "x_error": desired[6] - observation["state"][6],
            "position_error_norm": norm([desired[i] - observation["state"][i] for i in (6, 7, 8)]),
            "pi_hat_norm": norm(update[4:10]),
            "pi_star_norm": norm(model[24:30]),
            "estimate_error_norm": norm([a - b for a, b in zip(update[4:10], model[24:30])]),
            "full_flow_residual_norm": model[36],
            "unmatched_flow_residual_norm": model[37],
            "reconstruction_max_error": model[39],
        }
        for index, name in enumerate(COMPONENTS):
            row["pi_hat_" + name] = update[4 + index]
            row["sigma_" + name] = update[10 + index]
            row["pi_star_" + name] = model[24 + index]
            row["dynamics_residual_" + name] = model[18 + index]
            row["candidate_correction_" + name] = equation[12 + index]
            row["applied_correction_" + name] = equation[18 + index]
        for index, value in enumerate(observation["input"][:12]):
            row["contact_force_{}".format(index)] = value
        rows.append(row)

    peak_pi_hat_norm = max(row["pi_hat_norm"] for row in rows)
    pi_hat_change_norm = norm([updates[-1][4 + i] - updates[0][4 + i] for i in range(6)])
    peak_candidate_norm = max(norm(row[12:18]) for row in equation10)
    peak_applied_norm = max(norm(row[18:24]) for row in equation10)
    estimate_behavior_ok = (
        peak_pi_hat_norm > 1.0e-8 and pi_hat_change_norm > 1.0e-8
        if expected_update else
        peak_pi_hat_norm <= 1.0e-12 and pi_hat_change_norm <= 1.0e-12
    )
    correction_behavior_ok = (
        peak_candidate_norm > 1.0e-10 and peak_applied_norm > 1.0e-10
        if expected_dynamics else
        peak_candidate_norm <= 1.0e-12 and peak_applied_norm <= 1.0e-12
    )
    equation10_ok = (
        diagnostic_coverage_ok
        and mode_flags == {(int(expected_update), int(expected_dynamics))}
        and max_eq_time_error <= 1.0e-12
        and max_yu_pi_error <= 1.0e-12
        and max_candidate_formula_error <= 1.0e-12
        and max_applied_candidate_error <= 1.0e-12
        and estimate_behavior_ok
        and correction_behavior_ok
    )

    artificial_disturbance_ok = (
        all(int(round(row[1])) == 0 for row in disturbances)
        and max(abs(value) for row in disturbances for value in row[2:8]) <= 1.0e-12
    )

    target = targets[0]
    prior = [row for row in observations if row["bag_time"] <= target["bag_time"]]
    publish_time = prior[-1]["time"] if prior else observations[0]["time"]
    target_step = target["states"][-1][6] - target["states"][0][6]
    target_ok = (
        abs(target["times"][0] - 0.50) <= 1.0e-12
        and abs(target["times"][1] - 0.70) <= 1.0e-12
        and abs(target_step - 0.10) <= 1.0e-6
        and publish_time <= 0.45
    )

    modes = sorted(set(row["mode"] for row in observations))
    input_dimensions = sorted(set(len(row["input"]) for row in observations))
    peak_contact_force_norm = max(norm(row["input"][:12]) for row in observations)
    contact_ok = modes == [15] and input_dimensions == [24] and peak_contact_force_norm > 1.0
    expected_end_time = float(task_c_config.get(
        "experiment_end_mpc_time_s", task_c_config.get("smoke_end_mpc_time_s")))
    runtime_ok = observations[-1]["time"] >= expected_end_time - 0.03

    missing_topics = sorted(REQUIRED_TOPICS - set(data["topic_counts"]))
    summary = {
        "stage": task_c_config.get("stage", "C2"),
        "run_type": task_c_config.get("run_type", "smoke_only"),
        "passed": False,
        "config_ok": config_ok,
        "model_identity": {"ok": identity_ok, "value": identity},
        "ground_truth": {
            "ok": ground_truth_ok,
            "sample_count": len(mismatch),
            "mass_consistency_max_error": mass_consistency_error,
            "invalid_sample_count": invalid_model_rows,
            "nonzero_pi_star_sample_count": nonzero_pi_star_count,
            "peak_pi_star_norm": max_pi_star_norm,
            "peak_normalized_residual_norm": max_residual_norm,
            "peak_full_flow_residual_norm": max_full_residual_norm,
            "peak_unmatched_flow_residual_norm": max_unmatched_residual_norm,
            "max_reconstruction_error": max_reconstruction_error,
        },
        "adaptive_update": {
            "ok": snapshot_base_ok and estimate_behavior_ok,
            "expected_enabled": expected_update,
            "sample_count": len(updates),
            "first_version": versions[0],
            "last_version": versions[-1],
            "versions_strictly_increasing": all(b > a for a, b in zip(versions, versions[1:])),
            "max_snapshot_time_error_s": max_snapshot_time_error,
            "peak_pi_hat_norm": peak_pi_hat_norm,
            "first_to_last_pi_hat_change_norm": pi_hat_change_norm,
        },
        "equation10": {
            "ok": equation10_ok,
            "expected_update_enabled": expected_update,
            "expected_dynamics_enabled": expected_dynamics,
            "mode_flags": [list(value) for value in sorted(mode_flags)],
            "max_update_pair_time_error_s": max_eq_time_error,
            "max_yu_pi_error": max_yu_pi_error,
            "max_candidate_formula_error": max_candidate_formula_error,
            "max_applied_candidate_error": max_applied_candidate_error,
            "peak_candidate_correction_norm": peak_candidate_norm,
            "peak_applied_correction_norm": peak_applied_norm,
            "matched_update_count": len(rows),
            "unmatched_leading_update_versions": unmatched_update_versions,
            "diagnostic_coverage_ok": diagnostic_coverage_ok,
        },
        "pairing": {
            "max_model_sample_time_error_s": max_ground_truth_pair_error,
            "max_observation_time_error_s": max_observation_pair_error,
        },
        "target": {
            "ok": target_ok,
            "publish_observation_time_s": publish_time,
            "start_s": target["times"][0],
            "end_s": target["times"][1],
            "step_x_m": target_step,
        },
        "artificial_constant_wrench": {"ok": artificial_disturbance_ok, "enabled": False},
        "contact_and_runtime": {
            "ok": contact_ok and runtime_ok,
            "observed_modes": modes,
            "input_dimensions": input_dimensions,
            "peak_first_12_contact_force_norm": peak_contact_force_norm,
            "observation_start_s": observations[0]["time"],
            "observation_end_s": observations[-1]["time"],
            "runtime_reached_end": runtime_ok,
        },
        "tracking_descriptive_only": {
            "peak_abs_x_error_m": max(abs(row["x_error"]) for row in rows),
            "peak_position_error_norm_m": max(row["position_error_norm"] for row in rows),
        },
        "missing_required_topics": missing_topics,
        "all_recorded_numeric_values_finite": not data["nonfinite_topics"],
        "nonfinite_topics": data["nonfinite_topics"],
        "topic_message_counts": data["topic_counts"],
        "scientific_scope": task_c_config.get(
            "scientific_scope",
            "Per-run validation only; no tracking-performance or pi_hat convergence conclusion."),
    }
    summary["passed"] = (
        config_ok and identity_ok and ground_truth_ok
        and summary["adaptive_update"]["ok"] and equation10_ok
        and target_ok and artificial_disturbance_ok
        and contact_ok and runtime_ok
        and not missing_topics and not data["nonfinite_topics"]
        and max_ground_truth_pair_error <= 0.003
        and max_observation_pair_error <= 0.021
    )
    return rows, summary


def write_csv(path, rows):
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_plot(path, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    time = [row["observation_time"] for row in rows]
    figure, axes = plt.subplots(6, 1, figsize=(11, 15), sharex=True)
    axes[0].plot(time, [row["pi_star_Fz"] for row in rows], label="pi_star Fz")
    axes[0].plot(time, [row["pi_hat_Fz"] for row in rows], label="pi_hat Fz")
    axes[1].plot(time, [row["pi_star_norm"] for row in rows], label="||pi_star||")
    axes[1].plot(time, [row["pi_hat_norm"] for row in rows], label="||pi_hat||")
    axes[2].plot(time, [row["candidate_correction_Fz"] for row in rows], label="candidate correction Fz")
    axes[2].plot(time, [row["applied_correction_Fz"] for row in rows], label="applied correction Fz", linestyle="--")
    axes[3].plot(time, [row["desired_x"] for row in rows], label="desired x")
    axes[3].plot(time, [row["actual_x"] for row in rows], label="actual x")
    axes[4].plot(time, [row["x_error"] for row in rows], label="x error")
    axes[4].plot(time, [row["position_error_norm"] for row in rows], label="position error norm")
    axes[5].plot(time, [row["reconstruction_max_error"] for row in rows], label="reconstruction max error")
    axes[5].plot(time, [row["unmatched_flow_residual_norm"] for row in rows], label="unmatched residual norm")
    for axis in axes:
        axis.axvspan(0.50, 0.70, color="green", alpha=0.10)
        axis.grid(True)
        axis.legend(loc="best")
    axes[-1].set_xlabel("MPC / simulation time (s)")
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description="Validate one Task C mass-mismatch run")
    parser.add_argument("run_directory")
    args = parser.parse_args()
    run_dir = os.path.abspath(args.run_directory)
    with open(os.path.join(run_dir, "effective_config.json")) as stream:
        adaptive_config = json.load(stream)
    with open(os.path.join(run_dir, "task_c_effective_config.json")) as stream:
        task_c_config = json.load(stream)
    bag_basename = task_c_config.get("bag_basename", "task_c_smoke.bag")
    output_basename = task_c_config.get("analysis_output_basename", "task_c_smoke")
    rows, summary = validate_and_align(
        load_bag(os.path.join(run_dir, bag_basename)),
        adaptive_config, task_c_config)
    csv_path = os.path.join(run_dir, output_basename + ".csv")
    json_path = os.path.join(run_dir, output_basename + "_summary.json")
    png_path = os.path.join(run_dir, output_basename + ".png")
    write_csv(csv_path, rows)
    write_plot(png_path, rows)
    summary["outputs"] = {
        "csv": os.path.basename(csv_path),
        "json": os.path.basename(json_path),
        "png": os.path.basename(png_path),
    }
    with open(json_path, "w") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
