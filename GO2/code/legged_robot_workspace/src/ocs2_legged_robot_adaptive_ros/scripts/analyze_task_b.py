#!/usr/bin/env python3
"""Analyze the three formal Task B standing-disturbance runs."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

from experiment_metrics import TASK_B_MODES, compute_task_b_metrics, validate_task_b_manifests
from set_adaptive_experiment_mode import read_active_switches


OBSERVATION_TOPIC = "/legged_robot_mpc_observation"
UPDATE_TOPIC = "/adaptive_update/sample"
DISTURBANCE_TOPIC = "/adaptive_update/disturbance_sample"
COMPONENTS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")


def _nearest_index(times, value):
    right = bisect.bisect_left(times, value)
    candidates = []
    if right < len(times):
        candidates.append(right)
    if right:
        candidates.append(right - 1)
    if not candidates:
        raise ValueError("no observation samples")
    return min(candidates, key=lambda index: abs(times[index] - value))


def common_start_state_difference(observations_by_mode, tolerance=1.0e-6):
    if set(observations_by_mode) != set(TASK_B_MODES):
        raise ValueError("Task B observations must contain all three modes")
    if any(not observations_by_mode[mode] for mode in TASK_B_MODES):
        raise ValueError("Task B observations cannot be empty")

    common_start = max(observations_by_mode[mode][0][0] for mode in TASK_B_MODES)
    states = {}
    for mode in TASK_B_MODES:
        observations = observations_by_mode[mode]
        times = [item[0] for item in observations]
        index = _nearest_index(times, common_start)
        if abs(times[index] - common_start) > tolerance:
            raise ValueError("Task B has no synchronized observation at the common start")
        states[mode] = observations[index][1]
    state_size = len(states[TASK_B_MODES[0]])
    if state_size == 0 or any(len(states[mode]) != state_size for mode in TASK_B_MODES):
        raise ValueError("Task B common-start states have inconsistent dimensions")
    reference = states[TASK_B_MODES[0]]
    difference = max(abs(states[mode][index] - reference[index])
                     for mode in TASK_B_MODES for index in range(state_size))
    return common_start, difference, states


def _wrapped_error(desired, actual):
    return math.atan2(math.sin(desired - actual), math.cos(desired - actual))


def load_bag(path):
    import rosbag
    observations, updates, disturbances = [], [], []
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, _ in bag.read_messages(
                topics=[OBSERVATION_TOPIC, UPDATE_TOPIC, DISTURBANCE_TOPIC]):
            if topic == OBSERVATION_TOPIC:
                observations.append((float(message.time), [float(value) for value in message.state.value]))
            elif topic == UPDATE_TOPIC:
                updates.append([float(value) for value in message.data])
            elif topic == DISTURBANCE_TOPIC:
                disturbances.append([float(value) for value in message.data])
    observations.sort(key=lambda item: item[0])
    updates.sort(key=lambda item: item[0])
    disturbances.sort(key=lambda item: item[0])
    if not observations or not updates or not disturbances:
        raise ValueError("Task B bag is missing observation, update, or disturbance data")
    if len(observations[0][1]) < 12:
        raise ValueError("Task B observation state is too short")
    return observations, updates, disturbances


def build_rows(observations, updates, reference_pose, analysis_end=6.0, pairing_tolerance=1.0e-6):
    observation_times = [item[0] for item in observations]
    rows = []
    for sample in updates:
        if len(sample) != 16 or sample[0] > analysis_end:
            continue
        index = _nearest_index(observation_times, sample[0])
        if abs(observation_times[index] - sample[0]) > pairing_tolerance:
            raise ValueError("Task B update cannot be paired to the same observation time")
        state = observations[index][1]
        position_error = [reference_pose[i] - state[6 + i] for i in range(3)]
        orientation_error = [_wrapped_error(reference_pose[3 + i], state[9 + i]) for i in range(3)]
        rows.append({
            "time": sample[0],
            "snapshot_time": sample[1],
            "position_error": position_error,
            "orientation_error": orientation_error,
            "pi_hat": sample[4:10],
            "sigma": sample[10:16],
        })
    if len(rows) < 5:
        raise ValueError("Task B has too few synchronized analysis rows")
    return rows


def disturbance_summary(samples):
    if any(len(sample) != 8 for sample in samples):
        raise ValueError("Task B disturbance sample must contain eight values")
    active = [sample for sample in samples if sample[1] >= 0.5]
    inactive = [sample for sample in samples if sample[1] < 0.5]
    if not active or not inactive:
        raise ValueError("Task B disturbance record must include active and inactive periods")
    mean_wrench = [sum(sample[2 + i] for sample in active) / len(active) for i in range(6)]
    max_active_deviation = max(
        abs(sample[2 + i] - mean_wrench[i]) for sample in active for i in range(6))
    max_inactive_wrench = max(abs(sample[2 + i]) for sample in inactive for i in range(6))
    return {
        "actual_start_s": active[0][0],
        "actual_end_s": active[-1][0],
        "active_sample_count": len(active),
        "mean_wrench": mean_wrench,
        "max_active_deviation": max_active_deviation,
        "max_inactive_wrench": max_inactive_wrench,
    }


def load_switches(path):
    values = read_active_switches(Path(path).read_text(encoding="utf-8"))
    return {key: value == "true" for key, value in values.items()}


def write_rows(path, rows):
    fields = ["time", "snapshot_time"]
    fields += ["position_error_" + axis for axis in "xyz"]
    fields += ["orientation_error_" + axis for axis in ("yaw", "pitch", "roll")]
    fields += ["sigma_" + name for name in COMPONENTS]
    fields += ["pi_" + name for name in COMPONENTS]
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            flat = {"time": row["time"], "snapshot_time": row["snapshot_time"]}
            flat.update({"position_error_" + axis: value for axis, value in zip("xyz", row["position_error"])})
            flat.update({"orientation_error_" + axis: value for axis, value in
                         zip(("yaw", "pitch", "roll"), row["orientation_error"])})
            flat.update({"sigma_" + name: value for name, value in zip(COMPONENTS, row["sigma"])})
            flat.update({"pi_" + name: value for name, value in zip(COMPONENTS, row["pi_hat"])})
            writer.writerow(flat)


def _norm(values):
    return math.sqrt(sum(value * value for value in values))


def write_plot(path, rows_by_mode):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    labels = {"nominal": "Nominal", "estimate_only": "Estimate-only", "full_adaptive": "Full Adaptive"}
    for mode in TASK_B_MODES:
        rows = rows_by_mode[mode]
        times = [row["time"] for row in rows]
        axes[0].plot(times, [_norm(row["position_error"]) for row in rows], label=labels[mode])
        axes[1].plot(times, [_norm(row["sigma"]) for row in rows], label=labels[mode])
        axes[2].plot(times, [row["pi_hat"][0] for row in rows], label=labels[mode])
    for axis in axes:
        axis.axvspan(1.0, 4.0, color="tab:red", alpha=0.10)
        axis.grid(True)
        axis.legend()
    axes[0].set_ylabel("position error [m]")
    axes[1].set_ylabel("sigma norm")
    axes[2].set_ylabel("pi_hat Fx [N]")
    axes[2].set_xlabel("observation time [s]")
    figure.tight_layout()
    figure.savefig(path, dpi=170)
    plt.close(figure)


def _percent_reduction(baseline, candidate):
    return None if abs(baseline) < 1.0e-15 else 100.0 * (baseline - candidate) / baseline


def write_conclusion(path, metrics):
    nominal = metrics["nominal"]
    full = metrics["full_adaptive"]
    estimate = metrics["estimate_only"]
    position_change = _percent_reduction(nominal["rms_position_error_m"], full["rms_position_error_m"])
    sigma_change = _percent_reduction(nominal["rms_sigma_norm"], full["rms_sigma_norm"])
    lines = ["# Task B 结论", ""]
    if position_change is None:
        lines.append("Nominal 的位置误差接近零，无法计算有意义的相对改善比例。")
    else:
        direction = "降低" if position_change >= 0.0 else "增加"
        lines.append("Full Adaptive 相对 Nominal 的位置 RMS {} {:.3f}%。".format(direction, abs(position_change)))
    if sigma_change is not None:
        direction = "降低" if sigma_change >= 0.0 else "增加"
        lines.append("Full Adaptive 相对 Nominal 的 sigma RMS {} {:.3f}%。".format(direction, abs(sigma_change)))
    lines.append("Estimate-only 的 Fx 估计变化为 {:.6g} N，Full Adaptive 为 {:.6g} N。".format(
        estimate["pi_fx_change"], full["pi_fx_change"]))
    if full["recovery_time_after_disturbance_s"] is None:
        lines.append("Full Adaptive 在记录窗口内没有满足持续恢复条件。")
    else:
        lines.append("Full Adaptive 在扰动结束后 {:.3f} s 满足持续恢复条件。".format(
            full["recovery_time_after_disturbance_s"]))
    Path(path).write_text("\n\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--nominal-bag", required=True)
    parser.add_argument("--nominal-config", required=True)
    parser.add_argument("--estimate-only-bag", required=True)
    parser.add_argument("--estimate-only-config", required=True)
    parser.add_argument("--full-adaptive-bag", required=True)
    parser.add_argument("--full-adaptive-config", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)

    paths = {
        "nominal": (args.nominal_bag, args.nominal_config),
        "estimate_only": (args.estimate_only_bag, args.estimate_only_config),
        "full_adaptive": (args.full_adaptive_bag, args.full_adaptive_config),
    }
    raw = {mode: load_bag(bag) for mode, (bag, _) in paths.items()}
    common_start, max_initial_difference, common_start_states = common_start_state_difference(
        {mode: raw[mode][0] for mode in TASK_B_MODES})
    reference_state = raw["nominal"][0][0][1]
    reference_pose = reference_state[6:12]
    if max_initial_difference > 1.0e-5:
        raise ValueError("Task B common-start states differ by more than 1e-5")

    rows_by_mode, metrics, disturbances, manifests = {}, {}, {}, []
    for mode in TASK_B_MODES:
        observations, updates, disturbance = raw[mode]
        rows_by_mode[mode] = build_rows(observations, updates, reference_pose)
        metrics[mode] = compute_task_b_metrics(rows_by_mode[mode], disturbance_end=4.0)
        disturbances[mode] = disturbance_summary(disturbance)
        if (disturbances[mode]["mean_wrench"] != [20.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                or disturbances[mode]["max_active_deviation"] > 1.0e-12
                or disturbances[mode]["max_inactive_wrench"] > 1.0e-12
                or abs(disturbances[mode]["actual_start_s"] - 1.0) > 0.01
                or abs(disturbances[mode]["actual_end_s"] - 4.0) > 0.01):
            raise ValueError("Task B actual disturbance does not match the formal +20 N schedule")
        manifests.append({
            "mode": mode,
            "switches": load_switches(paths[mode][1]),
            "disturbance_start_s": 1.0,
            "disturbance_end_s": 4.0,
            "disturbance_wrench": [20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "analysis_end_s": 6.0,
            "initial_state": reference_state,
            "comparison_start_s": common_start,
            "comparison_state": common_start_states[mode],
            "max_common_start_difference": max_initial_difference,
        })
        write_rows(output / (mode + "_samples.csv"), rows_by_mode[mode])

    validate_task_b_manifests(manifests)
    with (output / "task_b_results.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = ["mode"] + sorted(next(iter(metrics.values())).keys())
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for mode in TASK_B_MODES:
            writer.writerow(dict({"mode": mode}, **metrics[mode]))

    summary = {
        "passed": True,
        "max_initial_state_difference": max_initial_difference,
        "manifests": manifests,
        "actual_disturbance": disturbances,
        "metrics": metrics,
    }
    (output / "task_b_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    write_plot(output / "task_b_comparison.png", rows_by_mode)
    write_conclusion(output / "task_b_conclusion.md", metrics)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
