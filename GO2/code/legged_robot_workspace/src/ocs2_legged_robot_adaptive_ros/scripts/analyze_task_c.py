#!/usr/bin/env python3
"""Analyze the four formal Task C reference and CLF runs."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

from analyze_aclf_ab import pair_executed_diagnostics
from experiment_metrics import TASK_C_MODES, compute_task_c_metrics, validate_task_c_manifests
from set_adaptive_experiment_mode import read_active_switches
from target_profiles import quintic_state


OBSERVATION = "/legged_robot_mpc_observation"
TARGET = "/legged_robot_mpc_target"
UPDATE = "/adaptive_update/sample"
DISTURBANCE = "/adaptive_update/disturbance_sample"
EXECUTED_TOPICS = {
    "/aclf_experiment/executed_sigma_norm": "sigma",
    "/aclf_experiment/executed_h_clf": "h_clf",
    "/aclf_experiment/executed_violation": "violation",
    "/aclf_experiment/executed_contribution": "contribution",
    "/aclf_experiment/executed_residual_wrench": "residual_wrench",
    "/aclf_experiment/executed_damping_term": "damping_term",
    "/aclf_experiment/executed_linear_term": "linear_term",
    "/aclf_experiment/executed_observation_time": "observation_time",
    "/aclf_experiment/executed_snapshot_time": "snapshot_time",
    "/aclf_experiment/executed_time_error": "time_error",
}


def _nearest_index(times, value):
    right = bisect.bisect_left(times, value)
    candidates = []
    if right < len(times):
        candidates.append(right)
    if right:
        candidates.append(right - 1)
    if not candidates:
        raise ValueError("no samples available for pairing")
    return min(candidates, key=lambda index: abs(times[index] - value))


def _interpolate(time, times, states):
    if time <= times[0]:
        return states[0]
    if time >= times[-1]:
        return states[-1]
    right = bisect.bisect_right(times, time)
    left = right - 1
    ratio = (time - times[left]) / (times[right] - times[left])
    return [(1.0 - ratio) * start + ratio * finish
            for start, finish in zip(states[left], states[right])]


def _wrapped_error(desired, actual):
    return math.atan2(math.sin(desired - actual), math.cos(desired - actual))


def ensure_complete_pairing(executed, pairing, anchor_times, required_start, required_end,
                            tolerance=1.0e-9):
    del pairing  # Extra edge messages remain in the report; completeness is checked by observation time.
    if not executed or not anchor_times or not required_start <= required_end:
        raise ValueError("Task C executed diagnostics are incomplete")
    complete_times = sorted(float(sample["observation_time"]) for sample in executed)
    for anchor in anchor_times:
        anchor = float(anchor)
        if anchor < required_start - tolerance or anchor > required_end + tolerance:
            continue
        index = _nearest_index(complete_times, anchor)
        if abs(complete_times[index] - anchor) > tolerance:
            raise ValueError("Task C executed diagnostics are incomplete inside the analysis window")
    return executed


def load_bag(path):
    import rosbag
    observations, targets, updates, disturbances = [], [], [], []
    raw = {field: [] for field in (
        "sigma", "h_clf", "violation", "contribution", "residual_wrench",
        "damping_term", "linear_term", "observation_time", "snapshot_time", "time_error")}
    topics = [OBSERVATION, TARGET, UPDATE, DISTURBANCE] + list(EXECUTED_TOPICS)
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, timestamp in bag.read_messages(topics=topics):
            bag_time = timestamp.to_sec()
            if topic == OBSERVATION:
                observations.append((float(message.time), [float(value) for value in message.state.value]))
            elif topic == TARGET:
                targets.append((bag_time, [float(value) for value in message.timeTrajectory],
                                [[float(value) for value in state.value] for state in message.stateTrajectory]))
            elif topic == UPDATE:
                updates.append([float(value) for value in message.data])
            elif topic == DISTURBANCE:
                disturbances.append([float(value) for value in message.data])
            elif topic in EXECUTED_TOPICS:
                field = EXECUTED_TOPICS[topic]
                value = list(message.data) if hasattr(message, "data") and not isinstance(message.data, float) else message.data
                if field in ("contribution", "residual_wrench") and len(value) != 6:
                    continue
                raw[field].append((bag_time, value))
    observations.sort(key=lambda item: item[0])
    targets.sort(key=lambda item: item[0])
    updates.sort(key=lambda item: item[0])
    disturbances.sort(key=lambda item: item[0])
    if not observations or not targets or not updates or not disturbances:
        raise ValueError("Task C bag is missing required topics")
    executed, pairing = pair_executed_diagnostics(raw)
    step_times, _ = select_step_target(targets)
    anchor_times = [value for _, value in raw["observation_time"]]
    executed = ensure_complete_pairing(
        executed, pairing, anchor_times, step_times[0], 8.0)
    return observations, targets, updates, disturbances, executed, pairing


def select_step_target(targets):
    for _, times, states in targets:
        if len(times) >= 2 and len(times) == len(states) and len(states[0]) >= 12:
            amplitude = states[-1][6] - states[0][6]
            if abs(amplitude) >= 0.01:
                return times, states
    raise ValueError("Task C bag does not contain a valid x target")


def validate_target(profile, times, states, amplitude=0.10, duration=1.0):
    if len(times) != len(states) or any(right <= left for left, right in zip(times, times[1:])):
        raise ValueError("Task C target times are invalid")
    actual_amplitude = states[-1][6] - states[0][6]
    actual_duration = times[-1] - times[0]
    if abs(actual_amplitude - amplitude) > 1.0e-6 or abs(actual_duration - duration) > 1.0e-6:
        raise ValueError("Task C target amplitude or duration mismatch")
    if profile == "linear" and len(times) != 2:
        raise ValueError("Task C linear target must contain two knots")
    if profile == "quintic":
        if len(times) < 3:
            raise ValueError("Task C quintic target requires dense knots")
        for time, state in zip(times, states):
            tau = (time - times[0]) / actual_duration
            scale = (state[6] - states[0][6]) / actual_amplitude
            if abs(scale - quintic_state(tau, actual_duration)[0]) > 1.0e-6:
                raise ValueError("Task C quintic target does not match the declared profile")
    return {"start_s": times[0], "end_s": times[-1], "duration_s": actual_duration,
            "amplitude_m": actual_amplitude, "knot_count": len(times)}


def build_rows(data, target, analysis_end=8.0, tolerance=1.0e-6):
    observations, _, updates, _, executed, _ = data
    target_times, target_states = target
    observation_times = [item[0] for item in observations]
    update_times = [sample[0] for sample in updates]
    rows = []
    for diagnostic in executed:
        time = float(diagnostic["observation_time"])
        if time < target_times[0] or time > analysis_end:
            continue
        observation_index = _nearest_index(observation_times, time)
        update_index = _nearest_index(update_times, time)
        if (abs(observation_times[observation_index] - time) > tolerance
                or abs(update_times[update_index] - time) > tolerance):
            raise ValueError("Task C diagnostics cannot be paired to observation and update")
        actual = observations[observation_index][1]
        update = updates[update_index]
        desired = _interpolate(time, target_times, target_states)
        row = {
            "time": time,
            "snapshot_time": float(diagnostic["snapshot_time"]),
            "position_error": [desired[6 + i] - actual[6 + i] for i in range(3)],
            "orientation_error": [_wrapped_error(desired[9 + i], actual[9 + i]) for i in range(3)],
            "sigma": update[10:16],
            "pi_hat": update[4:10],
            "h_clf": float(diagnostic["h_clf"]),
            "violation": float(diagnostic["violation"]),
            "desired_x": desired[6],
            "actual_x": actual[6],
        }
        rows.append(row)
    if len(rows) < 5:
        raise ValueError("Task C has too few synchronized rows")
    return rows


def load_switches(path):
    values = read_active_switches(Path(path).read_text(encoding="utf-8"))
    return {key: value == "true" for key, value in values.items()}


def write_rows(path, rows):
    fields = ["time", "snapshot_time", "desired_x", "actual_x", "h_clf", "violation"]
    fields += ["position_error_" + axis for axis in "xyz"]
    fields += ["orientation_error_" + axis for axis in ("yaw", "pitch", "roll")]
    fields += ["sigma_" + str(index) for index in range(6)]
    fields += ["pi_" + str(index) for index in range(6)]
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            flat = {key: row[key] for key in ("time", "snapshot_time", "desired_x", "actual_x", "h_clf", "violation")}
            flat.update({"position_error_" + axis: value for axis, value in zip("xyz", row["position_error"])})
            flat.update({"orientation_error_" + axis: value for axis, value in
                         zip(("yaw", "pitch", "roll"), row["orientation_error"])})
            flat.update({"sigma_" + str(index): value for index, value in enumerate(row["sigma"])})
            flat.update({"pi_" + str(index): value for index, value in enumerate(row["pi_hat"])})
            writer.writerow(flat)


def _norm(values):
    return math.sqrt(sum(value * value for value in values))


def write_plot(path, rows_by_mode):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    for mode in TASK_C_MODES:
        rows = rows_by_mode[mode]
        times = [row["time"] for row in rows]
        axes[0].plot(times, [row["actual_x"] for row in rows], label=mode)
        axes[1].plot(times, [_norm(row["position_error"]) for row in rows], label=mode)
        axes[2].plot(times, [_norm(row["sigma"]) for row in rows], label=mode)
        axes[3].plot(times, [row["h_clf"] for row in rows], label=mode)
    axes[0].set_ylabel("actual x [m]")
    axes[1].set_ylabel("position error [m]")
    axes[2].set_ylabel("sigma norm")
    axes[3].set_ylabel("h_clf")
    axes[3].set_xlabel("observation time [s]")
    for axis in axes:
        axis.grid(True)
        axis.legend(ncol=2, fontsize=8)
    figure.tight_layout()
    figure.savefig(path, dpi=170)
    plt.close(figure)


def _reduction(before, after):
    return None if abs(before) < 1.0e-15 else 100.0 * (before - after) / before


def write_conclusion(path, metrics):
    smooth_off = _reduction(metrics["linear_off"]["start_node_peak_sigma"],
                            metrics["quintic_off"]["start_node_peak_sigma"])
    smooth_on = _reduction(metrics["linear_on"]["start_node_peak_sigma"],
                           metrics["quintic_on"]["start_node_peak_sigma"])
    clf_position = _reduction(metrics["quintic_off"]["rms_position_error_m"],
                              metrics["quintic_on"]["rms_position_error_m"])
    lines = ["# Task C 结论", ""]
    for label, value in (("CLF 关闭", smooth_off), ("CLF 开启", smooth_on)):
        if value is None:
            lines.append("{}时线性目标节点值接近零，无法计算平滑改善比例。".format(label))
        else:
            direction = "降低" if value >= 0.0 else "增加"
            lines.append("{}时，五次目标让起点 sigma 峰值{} {:.3f}%。".format(label, direction, abs(value)))
    if clf_position is not None:
        direction = "降低" if clf_position >= 0.0 else "增加"
        lines.append("五次目标下，CLF 开启相对关闭让位置 RMS {} {:.3f}%。".format(direction, abs(clf_position)))
    Path(path).write_text("\n\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    for mode in TASK_C_MODES:
        option = mode.replace("_", "-")
        parser.add_argument("--" + option + "-bag", required=True)
        parser.add_argument("--" + option + "-config", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)

    paths = {}
    for mode in TASK_C_MODES:
        attribute = mode.replace("_", "_")
        paths[mode] = (getattr(args, attribute + "_bag"), getattr(args, attribute + "_config"))
    data = {mode: load_bag(paths[mode][0]) for mode in TASK_C_MODES}
    initial_reference = data["linear_off"][0][0][1]
    max_initial_difference = max(
        abs(data[mode][0][0][1][index] - initial_reference[index])
        for mode in TASK_C_MODES for index in range(len(initial_reference)))
    if max_initial_difference > 1.0e-5:
        raise ValueError("Task C initial states differ by more than 1e-5")

    rows_by_mode, metrics, target_summaries, manifests = {}, {}, {}, []
    for mode in TASK_C_MODES:
        profile, _ = mode.split("_")
        target = select_step_target(data[mode][1])
        target_summaries[mode] = validate_target(profile, *target)
        rows_by_mode[mode] = build_rows(data[mode], target)
        metrics[mode] = compute_task_c_metrics(
            rows_by_mode[mode], target_start=target[0][0], target_end=target[0][-1], node_window=0.25)
        if any(abs(sample[index]) > 1.0e-12 for sample in data[mode][3] for index in range(2, 8)):
            raise ValueError("Task C actual disturbance must remain zero")
        manifests.append({
            "mode": mode,
            "target_profile": profile,
            "switches": load_switches(paths[mode][1]),
            "step_m": 0.10,
            "target_duration_s": 1.0,
            "sample_period_s": 0.02,
            "settle_before_target_s": 2.0,
            "analysis_end_s": 8.0,
            "disturbance_wrench": [0.0] * 6,
            "initial_state": initial_reference,
        })
        write_rows(output / (mode + "_samples.csv"), rows_by_mode[mode])

    target_starts = [target_summaries[mode]["start_s"] for mode in TASK_C_MODES]
    if max(target_starts) - min(target_starts) > 0.021:
        raise ValueError("Task C target starts are not synchronized within one 0.02 s sample")

    validate_task_c_manifests(manifests)
    with (output / "task_c_results.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = ["mode"] + sorted(next(iter(metrics.values())).keys())
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for mode in TASK_C_MODES:
            writer.writerow(dict({"mode": mode}, **metrics[mode]))
    summary = {
        "passed": True,
        "max_initial_state_difference": max_initial_difference,
        "manifests": manifests,
        "targets": target_summaries,
        "metrics": metrics,
    }
    (output / "task_c_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    write_plot(output / "task_c_comparison.png", rows_by_mode)
    write_conclusion(output / "task_c_conclusion.md", metrics)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
