#!/usr/bin/env python3
"""Validate one Task B acquisition without making a performance claim."""

import argparse
import bisect
import csv
import json
import math
import os


COMPONENTS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
UPDATE_TOPIC = "/adaptive_update/sample"
DISTURBANCE_TOPIC = "/adaptive_update/disturbance_sample"
EQUATION10_TOPIC = "/task_b/equation10_sample"
OBSERVATION_TOPIC = "/legged_robot_mpc_observation"
TARGET_TOPIC = "/legged_robot_mpc_target"

REQUIRED_TOPICS = {
    OBSERVATION_TOPIC,
    TARGET_TOPIC,
    UPDATE_TOPIC,
    DISTURBANCE_TOPIC,
    EQUATION10_TOPIC,
    "/task_b/observation_time",
    "/task_b/desired_x",
    "/task_b/actual_x",
    "/task_b/desired_position",
    "/task_b/actual_position",
    "/task_b/desired_rpy",
    "/task_b/actual_rpy",
    "/task_b/orientation_error_rpy",
    "/task_b/orientation_error_norm",
    "/task_b/policy_input_sample",
    "/aclf_experiment/x_error",
    "/aclf_experiment/pitch_error",
    "/aclf_experiment/position_error_norm",
    "/aclf_experiment/executed_h_clf",
    "/aclf_experiment/executed_sigma_norm",
    "/aclf_experiment/executed_observation_time",
    "/aclf_experiment/executed_snapshot_time",
    "/aclf_experiment/executed_time_error",
}


def finite(values):
    return all(math.isfinite(float(value)) for value in values)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def interpolate(time, times, states):
    if time <= times[0]:
        return states[0]
    if time >= times[-1]:
        return states[-1]
    index = bisect.bisect_left(times, time)
    left, right = index - 1, index
    alpha = (times[right] - time) / (times[right] - times[left])
    return [alpha * a + (1.0 - alpha) * b for a, b in zip(states[left], states[right])]


def nearest_index(times, value):
    index = bisect.bisect_left(times, value)
    candidates = []
    if index < len(times):
        candidates.append(index)
    if index:
        candidates.append(index - 1)
    return min(candidates, key=lambda item: abs(times[item] - value))


def numeric_message_values(topic, message):
    if topic == OBSERVATION_TOPIC:
        return [message.time, float(message.mode)] + list(message.state.value) + list(message.input.value)
    if topic == TARGET_TOPIC:
        values = list(message.timeTrajectory)
        for state in message.stateTrajectory:
            values.extend(state.value)
        for control in message.inputTrajectory:
            values.extend(control.value)
        return values
    data = getattr(message, "data", None)
    if data is None:
        return []
    if isinstance(data, (list, tuple)):
        return list(data)
    return [data]


def load_bag(path):
    import rosbag

    updates, disturbances, equation10, observations, targets = [], [], [], [], []
    topic_counts = {}
    nonfinite = []
    with rosbag.Bag(path, "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        topic_counts = {topic: info.message_count for topic, info in topic_info.items()}
        for topic, message, stamp in bag.read_messages():
            values = numeric_message_values(topic, message)
            if values and not finite(values):
                nonfinite.append(topic)
            bag_time = stamp.to_sec()
            if topic == UPDATE_TOPIC:
                updates.append(list(message.data))
            elif topic == DISTURBANCE_TOPIC:
                disturbances.append(list(message.data))
            elif topic == EQUATION10_TOPIC:
                equation10.append(list(message.data))
            elif topic == OBSERVATION_TOPIC:
                observations.append((bag_time, message.time, list(message.state.value), list(message.input.value)))
            elif topic == TARGET_TOPIC:
                targets.append((bag_time, list(message.timeTrajectory),
                                [list(state.value) for state in message.stateTrajectory]))
    return {
        "updates": updates,
        "disturbances": disturbances,
        "equation10": equation10,
        "observations": observations,
        "targets": targets,
        "topic_counts": topic_counts,
        "nonfinite_topics": sorted(set(nonfinite)),
    }


def analyze(data, effective_config):
    updates = sorted(([float(value) for value in sample] for sample in data["updates"]), key=lambda row: row[0])
    disturbances = sorted(([float(value) for value in sample] for sample in data["disturbances"]), key=lambda row: row[0])
    equation10 = sorted(([float(value) for value in sample] for sample in data["equation10"]), key=lambda row: row[0])
    observations = sorted(data["observations"], key=lambda row: row[1])
    targets = data["targets"]

    if not updates or not disturbances or not equation10 or not observations or not targets:
        raise RuntimeError("smoke bag 缺少 update/disturbance/Eq.(10)/observation/target 数据")
    if any(len(row) != 16 or not finite(row) for row in updates):
        raise RuntimeError("update sample 必须是 16 个有限值")
    if any(len(row) != 8 or not finite(row) for row in disturbances):
        raise RuntimeError("disturbance sample 必须是 8 个有限值")
    if any(len(row) != 24 or not finite(row) for row in equation10):
        raise RuntimeError("Eq.(10) sample 必须是 24 个有限值")
    if len(targets) != 1:
        raise RuntimeError("smoke 必须恰好记录一次自动 target")

    target_bag_time, target_times, target_states = targets[0]
    if len(target_times) != 2 or len(target_states) != 2 or min(map(len, target_states)) < 12:
        raise RuntimeError("target trajectory 结构不符合两 knot 原生 base target")
    prior_observations = [row for row in observations if row[0] <= target_bag_time]
    target_publish_mpc_time = prior_observations[-1][1] if prior_observations else observations[0][1]
    target_start, target_end = target_times
    target_step = target_states[-1][6] - target_states[0][6]

    observation_times = [row[1] for row in observations]
    disturbance_times = [row[0] for row in disturbances]
    eq_by_key = {(row[0], int(round(row[3]))): row for row in equation10}

    versions = [int(round(row[3])) for row in updates]
    max_snapshot_error = max(abs(row[2]) for row in updates)
    version_strict = all(right > left for left, right in zip(versions, versions[1:]))

    active_mismatch = 0
    active_rows = []
    for sample in disturbances:
        expected = 1 if 1.0 < sample[0] <= 4.0 else 0
        if int(round(sample[1])) != expected:
            active_mismatch += 1
        if int(round(sample[1])) == 1:
            active_rows.append(sample)
    wrench_error = max((abs(row[2 + index] - (20.0 if index == 0 else 0.0))
                        for row in active_rows for index in range(6)), default=float("inf"))

    rows = []
    max_eq_pair_error = 0.0
    max_yu_pi_error = 0.0
    max_applied_candidate_error = 0.0
    max_candidate_norm = 0.0
    max_applied_norm = 0.0
    max_pi_norm = 0.0
    mode_flags = set()
    max_observation_pair_error = 0.0
    max_disturbance_pair_error = 0.0
    disturbance_pair_violation_count = 0
    initial_disturbance_pair_exception_count = 0
    input_dim = None
    for update in updates:
        key = (update[0], int(round(update[3])))
        eq = eq_by_key.get(key)
        if eq is None:
            candidates = [row for row in equation10 if int(round(row[3])) == key[1]]
            if not candidates:
                raise RuntimeError("Eq.(10) sample 无法按 snapshot version 与 update 配对")
            eq = min(candidates, key=lambda row: abs(row[0] - update[0]))
        max_eq_pair_error = max(max_eq_pair_error, abs(eq[0] - update[0]))
        mode_flags.add((int(round(eq[4])), int(round(eq[5]))))
        yu_pi = eq[6:12]
        candidate = eq[12:18]
        applied = eq[18:24]
        max_yu_pi_error = max(max_yu_pi_error, max(abs(a - b) for a, b in zip(yu_pi, update[4:10])))
        max_applied_candidate_error = max(
            max_applied_candidate_error, max(abs(a - b) for a, b in zip(applied, candidate)))
        max_candidate_norm = max(max_candidate_norm, math.sqrt(sum(value * value for value in candidate)))
        max_applied_norm = max(max_applied_norm, math.sqrt(sum(value * value for value in applied)))
        max_pi_norm = max(max_pi_norm, math.sqrt(sum(value * value for value in update[4:10])))

        obs_index = nearest_index(observation_times, update[0])
        _, obs_time, actual, policy_input = observations[obs_index]
        max_observation_pair_error = max(max_observation_pair_error, abs(obs_time - update[0]))
        input_dim = len(policy_input) if input_dim is None else input_dim
        disturbance_index = nearest_index(disturbance_times, update[0])
        disturbance = disturbances[disturbance_index]
        disturbance_pair_error = abs(disturbance[0] - update[0])
        max_disturbance_pair_error = max(max_disturbance_pair_error, disturbance_pair_error)
        if disturbance_pair_error > 1e-5:
            # The dummy publishes its first disturbance truth after the first
            # 400 Hz integration step.  An update at exactly t=0 may therefore
            # pair with the first inactive truth at t=0.0025 s.  Keep the
            # strict tolerance everywhere else, including both ON/OFF edges.
            initial_exception = (
                abs(update[0]) <= 1e-12
                and 0.0 < disturbance[0] <= 0.00251
                and int(round(disturbance[1])) == 0
            )
            if initial_exception:
                initial_disturbance_pair_exception_count += 1
            else:
                disturbance_pair_violation_count += 1
        desired = interpolate(update[0], target_times, target_states)
        desired_rpy = [desired[11], desired[10], desired[9]]
        actual_rpy = [actual[11], actual[10], actual[9]]
        orientation_error = [wrap(wanted - measured) for wanted, measured in zip(desired_rpy, actual_rpy)]
        row = {
            "observation_time": update[0],
            "snapshot_time": update[1],
            "snapshot_time_error": update[2],
            "snapshot_version": int(round(update[3])),
            "disturbance_active": int(round(disturbance[1])),
            "desired_x": desired[6],
            "actual_x": actual[6],
            "x_error": desired[6] - actual[6],
            "position_error_norm": math.sqrt(sum((desired[i] - actual[i]) ** 2 for i in (6, 7, 8))),
            "orientation_error_norm": math.sqrt(sum(value * value for value in orientation_error)),
            "enable_parameter_update": int(round(eq[4])),
            "enable_adaptive_dynamics": int(round(eq[5])),
        }
        for index, name in enumerate(COMPONENTS):
            row["disturbance_" + name] = disturbance[2 + index]
            row["pi_hat_" + name] = update[4 + index]
            row["sigma_" + name] = update[10 + index]
            row["yu_pi_" + name] = yu_pi[index]
            row["candidate_correction_" + name] = candidate[index]
            row["applied_correction_" + name] = applied[index]
        for label, values in (("desired_position", desired[6:9]), ("actual_position", actual[6:9]),
                              ("desired_rpy", desired_rpy), ("actual_rpy", actual_rpy),
                              ("orientation_error", orientation_error)):
            for axis, value in zip(("x", "y", "z") if "position" in label else ("roll", "pitch", "yaw"), values):
                row[label + "_" + axis] = value
        for index, value in enumerate(policy_input):
            row["policy_input_{}".format(index)] = value
        rows.append(row)

    missing_topics = sorted(REQUIRED_TOPICS - set(data["topic_counts"]))
    mode = effective_config.get("mode")
    expected_flags = {
        "nominal": (0, 0),
        "estimate_only": (1, 0),
        "full_adaptive": (1, 1),
    }
    if mode not in expected_flags:
        raise RuntimeError("effective config 中没有合法 Task B mode")
    expected_update, expected_dynamics = expected_flags[mode]
    config_ok = (
        effective_config.get("enableParameterUpdate") is bool(expected_update)
        and effective_config.get("enableAdaptiveDynamics") is bool(expected_dynamics)
        and effective_config.get("enableClfConstraint") is True
        and effective_config.get("enableInputBiasCost") is False
    )
    target_ok = (
        abs(target_start - 0.50) <= 1e-12
        and abs(target_end - 0.70) <= 1e-12
        # ROS message storage may round the 0.10 m command at float precision.
        and abs(target_step - 0.10) <= 1e-6
        and target_publish_mpc_time <= 0.45
    )
    disturbance_ok = (
        active_mismatch == 0 and bool(active_rows) and wrench_error <= 1e-12
        and active_rows[0][0] > 1.0 and active_rows[-1][0] <= 4.0
    )
    common_equation10_ok = (
        mode_flags == {(expected_update, expected_dynamics)}
        and max_eq_pair_error <= 1e-12
        and max_yu_pi_error <= 1e-12
    )
    if mode == "nominal":
        mode_path_ok = max_pi_norm <= 1e-12 and max_candidate_norm <= 1e-12 and max_applied_norm <= 1e-12
    elif mode == "estimate_only":
        mode_path_ok = max_pi_norm > 0.0 and max_candidate_norm > 0.0 and max_applied_norm <= 1e-12
    else:
        mode_path_ok = (
            max_pi_norm > 0.0 and max_candidate_norm > 0.0
            and max_applied_candidate_error <= 1e-12
        )
    equation10_ok = common_equation10_ok and mode_path_ok
    summary = {
        "mode": mode,
        "passed": False,
        "config_ok": config_ok,
        "required_topic_count": len(REQUIRED_TOPICS),
        "missing_required_topics": missing_topics,
        "all_recorded_numeric_values_finite": not data["nonfinite_topics"],
        "nonfinite_topics": data["nonfinite_topics"],
        "topic_message_counts": data["topic_counts"],
        "target": {
            "ok": target_ok,
            "message_count": len(targets),
            "publish_observation_time_s": target_publish_mpc_time,
            "start_s": target_start,
            "end_s": target_end,
            "step_x_m": target_step,
        },
        "disturbance": {
            "ok": disturbance_ok,
            "active_flag_mismatch_count": active_mismatch,
            "active_sample_count": len(active_rows),
            "first_active_time_s": active_rows[0][0] if active_rows else None,
            "last_active_time_s": active_rows[-1][0] if active_rows else None,
            "max_active_wrench_error": wrench_error,
        },
        "snapshot": {
            "ok": max_snapshot_error <= 1e-9 and version_strict,
            "sample_count": len(updates),
            "max_abs_time_error_s": max_snapshot_error,
            "version_strictly_increasing": version_strict,
            "first_version": versions[0],
            "last_version": versions[-1],
        },
        "equation10": {
            "ok": equation10_ok,
            "sample_count": len(equation10),
            "observed_mode_flags": [list(flags) for flags in sorted(mode_flags)],
            "max_update_pair_time_error_s": max_eq_pair_error,
            "max_yu_pi_vs_pi_hat_error": max_yu_pi_error,
            "max_applied_vs_candidate_error": max_applied_candidate_error,
            "peak_pi_hat_norm": max_pi_norm,
            "peak_candidate_correction_norm": max_candidate_norm,
            "peak_applied_correction_norm": max_applied_norm,
            "mode_path_ok": mode_path_ok,
        },
        "pairing": {
            "max_observation_time_error_s": max_observation_pair_error,
            "max_disturbance_time_error_s": max_disturbance_pair_error,
            "strict_disturbance_tolerance_s": 1e-5,
            "initial_t0_exception_count": initial_disturbance_pair_exception_count,
            "disturbance_pair_violation_count": disturbance_pair_violation_count,
        },
        "tracking": {
            "row_count": len(rows),
            "observation_start_s": rows[0]["observation_time"],
            "observation_end_s": rows[-1]["observation_time"],
            "policy_input_dimension": input_dim,
            "peak_abs_x_error_m": max(abs(row["x_error"]) for row in rows),
            "peak_position_error_norm_m": max(row["position_error_norm"] for row in rows),
        },
    }
    summary["passed"] = (
        config_ok and not missing_topics and not data["nonfinite_topics"]
        and target_ok and disturbance_ok and summary["snapshot"]["ok"]
        and equation10_ok and len(rows) >= 10
        and max_observation_pair_error <= 0.021
        and disturbance_pair_violation_count == 0
    )
    return rows, summary


def write_csv(path, rows):
    fieldnames = list(rows[0])
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_plot(path, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    time = [row["observation_time"] for row in rows]
    figure, axes = plt.subplots(6, 1, figsize=(11, 15), sharex=True)
    axes[0].plot(time, [row["disturbance_Fx"] for row in rows], label="true Fx")
    axes[1].plot(time, [row["pi_hat_Fx"] for row in rows], label="pi_hat Fx")
    axes[2].plot(time, [row["sigma_Fx"] for row in rows], label="sigma Fx")
    axes[3].plot(time, [row["x_error"] for row in rows], label="x error")
    axes[4].plot(time, [row["position_error_norm"] for row in rows], label="position error norm")
    axes[5].plot(time, [row["yu_pi_Fx"] for row in rows], label="Yu*pi Fx")
    axes[5].plot(time, [row["applied_correction_Fx"] for row in rows], label="applied correction Fx")
    for axis in axes:
        axis.axvspan(1.0, 4.0, color="orange", alpha=0.12)
        axis.grid(True)
        axis.legend(loc="best")
    axes[-1].set_xlabel("MPC / simulation time (s)")
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description="Validate one Task B nominal/estimate_only/full_adaptive run")
    parser.add_argument("run_directory")
    args = parser.parse_args()
    run_dir = os.path.abspath(args.run_directory)
    bag_path = os.path.join(run_dir, "task_b.bag")
    config_path = os.path.join(run_dir, "effective_config.json")
    with open(config_path) as stream:
        effective_config = json.load(stream)
    rows, summary = analyze(load_bag(bag_path), effective_config)
    csv_path = os.path.join(run_dir, "task_b_smoke.csv")
    json_path = os.path.join(run_dir, "task_b_smoke_summary.json")
    png_path = os.path.join(run_dir, "task_b_smoke.png")
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
