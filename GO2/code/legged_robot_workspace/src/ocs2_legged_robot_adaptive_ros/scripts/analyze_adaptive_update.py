#!/usr/bin/env python3
"""Analyze the online ConstantWrench6D update experiment."""

import argparse
import bisect
import csv
import json
import math
import os

COMPONENTS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
GAMMA = (3.0, 3.0, 3.0, 1.5, 1.5, 1.5)
UPDATE_TOPIC = "/adaptive_update/sample"
DISTURBANCE_TOPIC = "/adaptive_update/disturbance_sample"


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def _pair_disturbance(update_time, disturbance_times, disturbances, tolerance):
    index = bisect.bisect_left(disturbance_times, update_time)
    candidates = []
    if index < len(disturbance_times):
        candidates.append(index)
    if index:
        candidates.append(index - 1)
    if not candidates:
        raise ValueError("no disturbance samples")
    nearest = min(candidates, key=lambda item: abs(disturbance_times[item] - update_time))
    error = abs(disturbance_times[nearest] - update_time)
    if error > tolerance:
        raise ValueError("no disturbance sample within {:.3g}s of update time {:.9f}".format(
            tolerance, update_time))
    return disturbances[nearest], error


def analyze_samples(update_samples, disturbance_samples, pairing_tolerance=1e-5):
    if not update_samples:
        raise ValueError("no adaptive update samples")
    if not disturbance_samples:
        raise ValueError("no disturbance samples")
    for sample in update_samples:
        if len(sample) != 16 or not _finite(sample):
            raise ValueError("adaptive update samples must contain 16 finite values")
    for sample in disturbance_samples:
        if len(sample) != 8 or not _finite(sample):
            raise ValueError("disturbance samples must contain 8 finite values")

    updates = sorted(([float(value) for value in sample] for sample in update_samples),
                     key=lambda sample: sample[0])
    disturbances = sorted(([float(value) for value in sample] for sample in disturbance_samples),
                          key=lambda sample: sample[0])
    disturbance_times = [sample[0] for sample in disturbances]

    rows = []
    dropped_boundary_samples = 0
    max_pairing_error = 0.0
    for sample in updates:
        if (sample[0] < disturbance_times[0] - pairing_tolerance
                or sample[0] > disturbance_times[-1] + pairing_tolerance):
            dropped_boundary_samples += 1
            continue
        paired, pairing_error = _pair_disturbance(
            sample[0], disturbance_times, disturbances, pairing_tolerance)
        max_pairing_error = max(max_pairing_error, pairing_error)
        row = {
            "observation_time": sample[0],
            "snapshot_time": sample[1],
            "time_error": sample[2],
            "snapshot_version": int(round(sample[3])),
            "disturbance_active": int(round(paired[1])),
            "disturbance_time": paired[0],
        }
        for index, name in enumerate(COMPONENTS):
            row["pi_" + name] = sample[4 + index]
            row["sigma_" + name] = sample[10 + index]
            row["disturbance_" + name] = paired[2 + index]
        rows.append(row)

    versions = [row["snapshot_version"] for row in rows]
    version_strict = all(right > left for left, right in zip(versions, versions[1:]))
    sync_error = max(abs(row["time_error"]) for row in rows)

    gamma_errors = []
    parameter_steps = []
    for previous, current in zip(rows, rows[1:]):
        dt = current["observation_time"] - previous["observation_time"]
        if dt <= 0.0:
            raise ValueError("observation times must be strictly increasing")
        step = []
        for index, name in enumerate(COMPONENTS):
            actual = current["pi_" + name] - previous["pi_" + name]
            expected = dt * GAMMA[index] * current["sigma_" + name]
            gamma_errors.append(abs(actual - expected))
            step.append(actual)
        parameter_steps.append(math.sqrt(sum(value * value for value in step)))

    active_rows = [row for row in rows if row["disturbance_active"] == 1]
    if active_rows:
        mean_wrench = [
            sum(row["disturbance_" + name] for row in active_rows) / len(active_rows)
            for name in COMPONENTS
        ]
        dominant_index = max(range(6), key=lambda index: abs(mean_wrench[index]))
        dominant_name = COMPONENTS[dominant_index]
        first_active_index = rows.index(active_rows[0])
        baseline = rows[max(0, first_active_index - 1)]["pi_" + dominant_name]
        estimate_change = active_rows[-1]["pi_" + dominant_name] - baseline
        applied_value = mean_wrench[dominant_index]
        direction_matches = (
            abs(applied_value) > 1e-12 and abs(estimate_change) > 1e-12
            and applied_value * estimate_change > 0.0
        )
        disturbance_deviation = max(
            abs(row["disturbance_" + name] - mean_wrench[index])
            for row in active_rows
            for index, name in enumerate(COMPONENTS)
        )
    else:
        mean_wrench = [0.0] * 6
        dominant_name = None
        estimate_change = 0.0
        direction_matches = False
        disturbance_deviation = float("inf")

    summary = {
        "sample_count": len(rows),
        "dropped_boundary_sample_count": dropped_boundary_samples,
        "active_sample_count": len(active_rows),
        "observation_start_s": rows[0]["observation_time"],
        "observation_end_s": rows[-1]["observation_time"],
        "max_snapshot_time_error_s": sync_error,
        "max_disturbance_pairing_error_s": max_pairing_error,
        "version_strictly_increasing": version_strict,
        "max_gamma_update_error": max(gamma_errors) if gamma_errors else float("inf"),
        "rms_gamma_update_error": (
            math.sqrt(sum(value * value for value in gamma_errors) / len(gamma_errors))
            if gamma_errors else float("inf")
        ),
        "max_parameter_step": max(parameter_steps) if parameter_steps else float("inf"),
        "disturbance_is_constant_when_active": disturbance_deviation <= 1e-12,
        "active_mean_wrench": dict(zip(COMPONENTS, mean_wrench)),
        "dominant_component": dominant_name,
        "dominant_estimate_change": estimate_change,
        "estimate_direction_matches_disturbance": direction_matches,
    }
    summary["passed"] = (
        summary["sample_count"] >= 5
        and summary["active_sample_count"] >= 3
        and summary["max_snapshot_time_error_s"] <= 1e-6
        and summary["max_disturbance_pairing_error_s"] <= pairing_tolerance
        and summary["version_strictly_increasing"]
        and summary["max_gamma_update_error"] <= 1e-5
        and summary["disturbance_is_constant_when_active"]
        and summary["estimate_direction_matches_disturbance"]
    )
    return rows, summary


def load_bag(path):
    import rosbag
    updates = []
    disturbances = []
    with rosbag.Bag(path, "r") as bag:
        for topic, message, _ in bag.read_messages(
                topics=[UPDATE_TOPIC, DISTURBANCE_TOPIC]):
            if topic == UPDATE_TOPIC:
                updates.append(list(message.data))
            elif topic == DISTURBANCE_TOPIC:
                disturbances.append(list(message.data))
    return updates, disturbances


def write_csv(path, rows):
    fieldnames = [
        "observation_time", "snapshot_time", "time_error", "snapshot_version",
        "disturbance_active", "disturbance_time",
    ]
    for prefix in ("pi_", "sigma_", "disturbance_"):
        fieldnames.extend(prefix + name for name in COMPONENTS)
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_plot(path, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = [row["observation_time"] for row in rows]
    figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for name in COMPONENTS:
        axes[0].plot(times, [row["pi_" + name] for row in rows], label="pi_" + name)
    for name in COMPONENTS:
        values = [row["disturbance_" + name] for row in rows]
        if any(abs(value) > 1e-12 for value in values):
            axes[0].plot(times, values, "--", linewidth=2, label="applied_" + name)
    axes[0].set_ylabel("wrench estimate")
    axes[0].grid(True)
    axes[0].legend(ncol=4, fontsize=8)
    for name in COMPONENTS:
        axes[1].plot(times, [row["sigma_" + name] for row in rows], label="sigma_" + name)
    axes[1].set_xlabel("observation time (s)")
    axes[1].set_ylabel("sigma")
    axes[1].grid(True)
    axes[1].legend(ncol=3, fontsize=8)
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output-dir", default=None)
    args = parser.parse_args()
    output_dir = args.output_dir or os.path.dirname(os.path.abspath(args.bag))
    os.makedirs(output_dir, exist_ok=True)

    rows, summary = analyze_samples(*load_bag(args.bag))
    csv_path = os.path.join(output_dir, "adaptive_update.csv")
    json_path = os.path.join(output_dir, "adaptive_update_summary.json")
    plot_path = os.path.join(output_dir, "adaptive_update.png")
    write_csv(csv_path, rows)
    write_plot(plot_path, rows)
    with open(json_path, "w") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())