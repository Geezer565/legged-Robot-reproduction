#!/usr/bin/env python3
"""Create time-resolved MyRBF tracking and six-axis estimate figures.

The script intentionally consumes the recorded CSV files instead of summary
metrics.  It is suitable for the four-method and paired-method experiments.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


METHOD_COLORS = {
    "Baseline": "#0072B2",
    "ETHRBF": "#E69F00",
    "MyRBF": "#009E73",
    "MyRBF-v4": "#D55E00",
    "MyRBF-v5": "#CC79A7",
    "V4": "#D55E00",
    "V5-A": "#009E73",
}

TRAJECTORY_FIELDS = (
    ("x", "actual_base_x_m", "desired_base_x_m", "Base x (m)"),
    ("y", "actual_base_y_m", "desired_base_y_m", "Base y (m)"),
    ("z", "actual_base_z_m", "desired_base_z_m", "Base z (m)"),
    ("yaw", "actual_base_yaw_rad", "desired_base_yaw_rad", "Yaw (rad)"),
    ("pitch", "actual_base_pitch_rad", "desired_base_pitch_rad", "Pitch (rad)"),
    ("roll", "actual_base_roll_rad", "desired_base_roll_rad", "Roll (rad)"),
)

ESTIMATE_FIELDS = (
    ("Fx", "net_residual_fx_n", "estimate_after_fx_n", "Fx (N)"),
    ("Fy", "net_residual_fy_n", "estimate_after_fy_n", "Fy (N)"),
    ("Fz", "net_residual_fz_n", "estimate_after_fz_n", "Fz (N)"),
    ("Tx", "net_residual_tx_nm", "estimate_after_tx_nm", "Tx (N m)"),
    ("Ty", "net_residual_ty_nm", "estimate_after_ty_nm", "Ty (N m)"),
    ("Tz", "net_residual_tz_nm", "estimate_after_tz_nm", "Tz (N m)"),
)


def _finite(value: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("non-finite value")
    return number


def read_columns(path: Path, fields: list[str]) -> dict[str, list[float]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        names = set(reader.fieldnames or [])
        missing = [field for field in fields if field not in names]
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(missing)}")
        values = {field: [] for field in fields}
        for row in reader:
            try:
                parsed = {field: _finite(row[field]) for field in fields}
            except (TypeError, ValueError):
                continue
            for field, value in parsed.items():
                values[field].append(value)
    if not values[fields[0]]:
        raise ValueError(f"{path}: no finite rows")
    return values


def load_case(name: str, directory: Path) -> dict:
    shadow_path = directory / "shadow.csv"
    if not shadow_path.is_file():
        raise FileNotFoundError(f"{name}: missing {shadow_path}")
    trajectory_columns = ["time_s"] + [field for _, field, _, _ in TRAJECTORY_FIELDS]
    trajectory_columns += [desired for _, _, desired, _ in TRAJECTORY_FIELDS]
    estimate_columns = ["time_s"] + [field for _, field, estimate, _ in ESTIMATE_FIELDS]
    estimate_columns += [estimate for _, _, estimate, _ in ESTIMATE_FIELDS]
    fields = list(dict.fromkeys(trajectory_columns + estimate_columns))
    data = read_columns(shadow_path, fields)

    motion_window = None
    motion_path = directory / "motion.csv"
    if motion_path.is_file():
        try:
            motion = read_columns(motion_path, ["ros_time", "active"])
            active_times = [
                time for time, active in zip(motion["ros_time"], motion["active"])
                if active > 0.5
            ]
            if active_times:
                motion_window = (min(active_times), max(active_times))
        except ValueError:
            motion_window = None

    return {
        "name": name,
        "directory": str(directory),
        "data": data,
        "motion_window": motion_window,
    }


def parse_case(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("case must have the form NAME=DIR")
    name, directory = value.split("=", 1)
    if not name.strip() or not directory.strip():
        raise argparse.ArgumentTypeError("case must have the form NAME=DIR")
    return name.strip(), Path(directory.strip())


def color_for(name: str, index: int) -> str:
    palette = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9"]
    return METHOD_COLORS.get(name, palette[index % len(palette)])


def common_window(cases: list[dict], start: float | None, end: float | None):
    all_times = [value for case in cases for value in case["data"]["time_s"]]
    if not all_times:
        raise ValueError("no time samples")
    lower = min(all_times) if start is None else start
    upper = max(all_times) if end is None else end
    if upper <= lower:
        raise ValueError("time-end must be greater than time-start")
    active_windows = [case["motion_window"] for case in cases if case["motion_window"]]
    active_window = None
    if active_windows:
        active_window = (min(window[0] for window in active_windows),
                         max(window[1] for window in active_windows))
    return lower, upper, active_window


def style_axes(axis, label: str, lower: float, upper: float, active_window):
    axis.set_ylabel(label)
    axis.set_xlim(lower, upper)
    axis.grid(True, color="#D9D9D9", linewidth=0.55, alpha=0.75)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    if active_window:
        start, stop = active_window
        if stop >= lower and start <= upper:
            axis.axvspan(max(start, lower), min(stop, upper),
                         color="#E69F00", alpha=0.10, zorder=0)


def save_figure(figure, output_dir: Path, stem: str):
    output_dir.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_dir / f"{stem}.png", dpi=300, bbox_inches="tight")
    figure.savefig(output_dir / f"{stem}.pdf", bbox_inches="tight")
    plt.close(figure)


def plot_trajectory(cases: list[dict], output_dir: Path, lower: float,
                    upper: float, active_window):
    figure, axes = plt.subplots(2, 3, figsize=(12.0, 6.5), sharex=True)
    axes = axes.ravel()
    for panel, (short_name, actual, desired, label) in zip(axes, TRAJECTORY_FIELDS):
        for index, case in enumerate(cases):
            data = case["data"]
            color = color_for(case["name"], index)
            times = data["time_s"]
            mask = [lower <= time <= upper for time in times]
            panel.plot(
                [time for time, keep in zip(times, mask) if keep],
                [value for value, keep in zip(data[actual], mask) if keep],
                color=color, linewidth=1.05, label=case["name"],
            )
            panel.plot(
                [time for time, keep in zip(times, mask) if keep],
                [value for value, keep in zip(data[desired], mask) if keep],
                color=color, linewidth=0.8, linestyle="--", alpha=0.70,
            )
        panel.set_title(short_name, fontsize=10)
        style_axes(panel, label, lower, upper, active_window)
    for panel in axes[3:]:
        panel.set_xlabel("Time (s)")
    handles = [Line2D([0], [0], color=color_for(case["name"], index),
                      linewidth=1.4, label=case["name"])
               for index, case in enumerate(cases)]
    handles.append(Line2D([0], [0], color="#333333", linewidth=1.0,
                           linestyle="-", label="actual"))
    handles.append(Line2D([0], [0], color="#333333", linewidth=1.0,
                           linestyle="--", label="target"))
    figure.legend(handles=handles, loc="upper center", ncol=min(5, len(handles)),
                  frameon=False, bbox_to_anchor=(0.5, 1.02))
    figure.suptitle("Target–actual body trajectory tracking", y=1.07, fontsize=13)
    figure.text(0.5, -0.01, "Shaded region: active walking interval", ha="center", fontsize=8)
    figure.tight_layout()
    save_figure(figure, output_dir, "01_target_actual_tracking")


def plot_estimates(cases: list[dict], output_dir: Path, lower: float,
                   upper: float, active_window):
    figure, axes = plt.subplots(2, 3, figsize=(12.0, 6.5), sharex=True)
    axes = axes.ravel()
    for panel, (short_name, residual, estimate, label) in zip(axes, ESTIMATE_FIELDS):
        for index, case in enumerate(cases):
            data = case["data"]
            color = color_for(case["name"], index)
            times = data["time_s"]
            mask = [lower <= time <= upper for time in times]
            panel.plot(
                [time for time, keep in zip(times, mask) if keep],
                [value for value, keep in zip(data[estimate], mask) if keep],
                color=color, linewidth=1.0, label=case["name"],
            )
            panel.plot(
                [time for time, keep in zip(times, mask) if keep],
                [value for value, keep in zip(data[residual], mask) if keep],
                color=color, linewidth=0.55, linestyle=":", alpha=0.42,
            )
        panel.axhline(0.0, color="#666666", linewidth=0.55)
        panel.set_title(short_name, fontsize=10)
        style_axes(panel, label, lower, upper, active_window)
    for panel in axes[3:]:
        panel.set_xlabel("Time (s)")
    handles = [Line2D([0], [0], color=color_for(case["name"], index),
                      linewidth=1.4, label=case["name"])
               for index, case in enumerate(cases)]
    handles.append(Line2D([0], [0], color="#333333", linewidth=1.0,
                           linestyle="-", label="RBF estimate"))
    handles.append(Line2D([0], [0], color="#333333", linewidth=1.0,
                           linestyle=":", label="observed residual"))
    figure.legend(handles=handles, loc="upper center", ncol=min(5, len(handles)),
                  frameon=False, bbox_to_anchor=(0.5, 1.02))
    figure.suptitle("Six-axis residual and MyRBF estimate", y=1.07, fontsize=13)
    figure.text(0.5, -0.01, "This residual is an observation signal, not an external-force ground truth.",
                ha="center", fontsize=8)
    figure.tight_layout()
    save_figure(figure, output_dir, "02_six_axis_estimates")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Plot target/actual tracking and six-axis MyRBF time series.")
    parser.add_argument(
        "--case", action="append", required=True, type=parse_case,
        help="case in the form NAME=DIR; repeat for each method")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--time-start", type=float)
    parser.add_argument("--time-end", type=float)
    args = parser.parse_args()

    cases = [load_case(name, directory) for name, directory in args.case]
    lower, upper, active_window = common_window(
        cases, args.time_start, args.time_end)
    plot_trajectory(cases, args.output_dir, lower, upper, active_window)
    plot_estimates(cases, args.output_dir, lower, upper, active_window)
    manifest = {
        "cases": [{"name": case["name"], "directory": case["directory"]}
                  for case in cases],
        "time_window_s": [lower, upper],
        "active_walking_window_s": active_window,
        "figures": ["01_target_actual_tracking", "02_six_axis_estimates"],
    }
    (args.output_dir / "FIGURE_MANIFEST.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"saved figures to {args.output_dir}")
    print(f"time window: {lower:.3f} to {upper:.3f} s")
    if active_window:
        print(f"walking window: {active_window[0]:.3f} to {active_window[1]:.3f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
