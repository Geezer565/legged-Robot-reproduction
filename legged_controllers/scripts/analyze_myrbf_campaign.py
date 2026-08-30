#!/usr/bin/env python3

import argparse
import bisect
import csv
import json
import math
from pathlib import Path


AXIS_SUFFIX = {
    "fx": ("fx_n", "N"),
    "fy": ("fy_n", "N"),
    "fz": ("fz_n", "N"),
    "tx": ("tx_nm", "N·m"),
    "ty": ("ty_nm", "N·m"),
    "tz": ("tz_nm", "N·m"),
}


def mean(values):
    return sum(values) / len(values)


def rms(values):
    return math.sqrt(mean([value * value for value in values]))


def correlation(left, right):
    left_mean = mean(left)
    right_mean = mean(right)
    covariance = sum(
        (x - left_mean) * (y - right_mean) for x, y in zip(left, right)
    )
    left_energy = sum((x - left_mean) ** 2 for x in left)
    right_energy = sum((y - right_mean) ** 2 for y in right)
    denominator = math.sqrt(left_energy * right_energy)
    return covariance / denominator if denominator > 0.0 else 0.0


def axis_spec(axis):
    if axis not in AXIS_SUFFIX:
        raise ValueError("unknown wrench axis")
    suffix, unit = AXIS_SUFFIX[axis]
    return "estimate_after_" + suffix, "achieved_control_" + suffix, unit


def nearest_truth_values(sample_times, truth_rows, axis):
    times = [float(row["ros_time"]) for row in truth_rows]
    values = [float(row[axis]) for row in truth_rows]
    output = []
    for sample_time in sample_times:
        insertion = bisect.bisect_left(times, sample_time)
        candidates = {
            max(0, insertion - 1),
            min(len(times) - 1, insertion),
        }
        nearest = min(candidates, key=lambda index: abs(times[index] - sample_time))
        output.append(values[nearest])
    return output


def body_series(shadow_rows, axis):
    if axis in ("fx", "fy"):
        coordinate = "actual_base_x_m" if axis == "fx" else "actual_base_y_m"
        label = "Base longitudinal position (cm)" if axis == "fx" else "Base lateral position (cm)"
        return [100.0 * float(row[coordinate]) for row in shadow_rows], label
    if axis == "fz":
        return [
            100.0 * (float(row["actual_base_z_m"]) - float(row["desired_base_z_m"]))
            for row in shadow_rows
        ], "Base height error (cm)"
    angle = {"tx": "roll", "ty": "pitch", "tz": "yaw"}[axis]
    return [
        math.degrees(float(row[f"actual_base_{angle}_rad"])) for row in shadow_rows
    ], f"Base {angle} angle (deg)"


def analyze_case_rows(name, shadow_rows, wrench_rows, axis):
    estimate_field, applied_field, unit = axis_spec(axis)
    active_truth = [row for row in wrench_rows if abs(float(row[axis])) > 0.05]
    if not shadow_rows or not active_truth:
        raise ValueError(f"{name} has no usable shadow or wrench samples")
    onset = float(active_truth[0]["ros_time"])
    end = float(active_truth[-1]["ros_time"])
    active_shadow = [
        row for row in shadow_rows if onset <= float(row["time_s"]) <= end
    ]
    if not active_shadow:
        raise ValueError(f"{name} has no shadow samples in the disturbance window")

    times = [float(row["time_s"]) for row in active_shadow]
    external = nearest_truth_values(times, wrench_rows, axis)
    target = [-value for value in external]
    estimate = [float(row[estimate_field]) for row in active_shadow]
    applied = [float(row[applied_field]) for row in active_shadow]
    body, body_label = body_series(active_shadow, axis)
    roll = [math.degrees(float(row["actual_base_roll_rad"])) for row in active_shadow]
    estimation_error = [value - desired for value, desired in zip(estimate, target)]
    allocator = [float(row["allocator_scale"]) for row in shadow_rows]

    result = {
        "name": name,
        "axis": axis,
        "unit": unit,
        "onset_s": onset,
        "end_s": end,
        "sample_count": len(active_shadow),
        "estimate_rmse": rms(estimation_error),
        "estimate_correlation": correlation(target, estimate),
        "body_rms": rms(body),
        "body_max_abs": max(abs(value) for value in body),
        "body_label": body_label,
        "roll_rms_deg": rms(roll),
        "roll_max_abs_deg": max(abs(value) for value in roll),
        "mean_abs_applied": mean([abs(value) for value in applied]),
        "max_abs_applied": max(abs(value) for value in applied),
        "minimum_allocator_scale": min(allocator),
    }
    series = {
        "time": [value - onset for value in times],
        "target": target,
        "estimate": estimate,
        "body": body,
        "applied": applied,
    }
    return result, series


def read_rows(path):
    with Path(path).open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def analyze_case(name, directory, axis):
    directory = Path(directory)
    result, series = analyze_case_rows(
        name,
        read_rows(directory / "shadow.csv"),
        read_rows(directory / "wrench.csv"),
        axis,
    )
    state_path = directory / "state.csv"
    motion_path = directory / "motion.csv"
    if state_path.exists():
        state = [
            row for row in read_rows(state_path)
            if row.get("base_z", "nan").lower() != "nan"
        ]
        if state:
            result["minimum_base_z_m"] = min(float(row["base_z"]) for row in state)
            result["end_mode"] = int(float(state[-1]["observation_mode"]))
    if motion_path.exists():
        motion = [row for row in read_rows(motion_path) if row["active"] == "1"]
        state = [
            row for row in read_rows(state_path)
            if row.get("base_x", "nan").lower() != "nan"
        ]
        if motion and state:
            start = float(motion[0]["ros_time"])
            end = float(motion[-1]["ros_time"])
            walking = [row for row in state if start <= float(row["ros_time"]) <= end]
            if walking:
                result["walk_distance_m"] = (
                    float(walking[-1]["base_x"]) - float(walking[0]["base_x"])
                )
    return result, series


def write_markdown(path, title, results):
    body_label = results[0]["body_label"]
    unit = results[0]["unit"]
    lines = [
        f"# {title}",
        "",
        f"| Method | {body_label} RMS ↓ | {body_label} max ↓ | "
        f"Estimate RMSE ({unit}) ↓ | Correlation ↑ | Mean applied ({unit}) |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for result in results:
        lines.append(
            "| {name} | {body_rms:.3f} | {body_max_abs:.3f} | "
            "{estimate_rmse:.3f} | {estimate_correlation:.3f} | "
            "{mean_abs_applied:.3f} |".format(**result)
        )
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_comparison(output_base, title, results, all_series):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = ("#5B6770", "#E69F00", "#56B4E9", "#0072B2")
    hatches = ("//", "\\\\", "..", "xx")
    fig, axes = plt.subplots(2, 2, figsize=(11.0, 7.2), constrained_layout=True)
    estimate_axis, body_axis, rmse_axis, response_axis = axes.flat

    first = all_series[0]
    estimate_axis.plot(
        first["time"], first["target"], color="black", linewidth=2.0,
        linestyle="--", label="Target compensation", zorder=5,
    )
    for index, (result, series) in enumerate(zip(results, all_series)):
        estimate_axis.plot(
            series["time"], series["estimate"], color=colors[index],
            linewidth=1.6, label=result["name"],
        )
        body_axis.plot(
            series["time"], series["body"], color=colors[index],
            linewidth=1.6, label=result["name"],
        )

    names = [result["name"] for result in results]
    positions = list(range(len(names)))
    rmse_bars = rmse_axis.bar(
        positions, [result["estimate_rmse"] for result in results],
        color=colors, edgecolor="black", linewidth=0.6,
    )
    response_bars = response_axis.bar(
        positions, [result["body_rms"] for result in results],
        color=colors, edgecolor="black", linewidth=0.6,
    )
    for bars in (rmse_bars, response_bars):
        for bar, hatch in zip(bars, hatches):
            bar.set_hatch(hatch)

    estimate_axis.set_ylabel(f"Estimated compensation ({results[0]['unit']})")
    body_axis.set_ylabel(results[0]["body_label"])
    rmse_axis.set_ylabel(f"Estimation RMSE ({results[0]['unit']})")
    response_axis.set_ylabel("Body-response RMS")
    for axis in (estimate_axis, body_axis):
        axis.set_xlabel("Time after disturbance onset (s)")
        axis.grid(True, alpha=0.25)
    for axis in (rmse_axis, response_axis):
        axis.set_xticks(positions)
        axis.set_xticklabels(names, rotation=18, ha="right")
        axis.grid(True, axis="y", alpha=0.25)
    estimate_axis.legend(ncol=2, fontsize=8, frameon=False)
    fig.suptitle(title, fontsize=14, fontweight="bold")
    output_base = Path(output_base)
    fig.savefig(output_base.with_suffix(".png"), dpi=300, bbox_inches="tight", facecolor="white")
    fig.savefig(output_base.with_suffix(".pdf"), bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Analyze and plot a MyRBF Gazebo comparison campaign.")
    parser.add_argument("--case", action="append", nargs=2, metavar=("NAME", "DIRECTORY"), required=True)
    parser.add_argument("--axis", choices=tuple(AXIS_SUFFIX), required=True)
    parser.add_argument("--title", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--prefix", required=True)
    args = parser.parse_args()

    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    analyzed = [analyze_case(name, directory, args.axis) for name, directory in args.case]
    results = [item[0] for item in analyzed]
    all_series = [item[1] for item in analyzed]
    (output / f"{args.prefix}_metrics.json").write_text(
        json.dumps(results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    write_markdown(output / f"{args.prefix}_summary.md", args.title, results)
    plot_comparison(output / args.prefix, args.title, results, all_series)


if __name__ == "__main__":
    main()
