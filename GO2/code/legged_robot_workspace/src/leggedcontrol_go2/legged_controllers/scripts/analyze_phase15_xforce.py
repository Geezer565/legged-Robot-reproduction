#!/usr/bin/env python3
"""Offline, raw-sample Phase 1.5 comparison plots and stability metrics."""

import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml


PLOTTER = Path(__file__).with_name("plot_go2_method_comparison.py")
SPEC = importlib.util.spec_from_file_location("go2_plotter", PLOTTER)
PLOTTER_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PLOTTER_MODULE)

NAMES = {
    "pure_mpc": "Pure MPC",
    "direct_wrench": "Direct wrench (6D)",
    "v11_equivalent_wrench": "RBF V1.1",
}
COLORS = {"pure_mpc": "#666666", "direct_wrench": "#E69F00",
          "v11_equivalent_wrench": "#0072B2"}


def number(rows, field):
    return np.asarray([float(row[field]) for row in rows], dtype=float)


def load(case_path):
    case_path = Path(case_path).resolve()
    manifest = json.loads((case_path / "case_manifest.json").read_text())
    _, state = PLOTTER_MODULE.read_rows(case_path / "state.csv")
    _, target = PLOTTER_MODULE.read_rows(case_path / "mpc_target.csv")
    model = yaml.safe_load((case_path / "effective_rosparams.yaml").read_text())["go2_nominal_model"]
    mass = float(model["mass"])
    inertia = np.asarray(model["inertia_body_row_major"], dtype=float).reshape(3, 3)
    t = number(state, "wall_elapsed")
    angles = np.column_stack([number(state, "observation_base_" + axis)
                              for axis in ("yaw", "pitch", "roll")])
    momentum = np.column_stack([number(state, "observation_normalized_momentum_" + str(i))
                                for i in range(6)])
    actual = PLOTTER_MODULE.physical_velocity(
        momentum, PLOTTER_MODULE.rotation_zyx(angles), mass, inertia)
    target_map = PLOTTER_MODULE.planner_on_state(
        {"data": {"state": {"rows": state}, "mpc_target": {"rows": target}}})
    target_angles = np.column_stack([target_map[axis + "_rad"]
                                     for axis in ("yaw", "pitch", "roll")])
    target_momentum = np.column_stack([
        target_map[field] for field in PLOTTER_MODULE.LINEAR_TARGET_RATE_FIELDS +
        tuple("normalized_angular_momentum_" + axis for axis in ("x", "y", "z"))])
    reference = PLOTTER_MODULE.physical_velocity(
        target_momentum, PLOTTER_MODULE.rotation_zyx(target_angles), mass, inertia)
    return {"path": case_path, "manifest": manifest, "state": state, "t": t,
            "actual": actual, "reference": reference, "angles": angles,
            "height": number(state, "observation_base_z")}


def interval(data):
    return (data["t"] >= 6.0) & (data["t"] < 20.0)


def finite_or_nan(values, mask):
    values = values[mask]
    return values[np.isfinite(values)]


def metrics(data):
    mask = interval(data)
    actual, reference = data["actual"], data["reference"]
    error = actual - reference
    vx_error = finite_or_nan(error[:, 0], mask)
    vy_error = finite_or_nan(error[:, 1], mask)
    late = (data["t"] >= 16.0) & (data["t"] < 20.0)
    yaw_unwrapped = np.unwrap(data["angles"][:, 0])
    yaw_rate = np.gradient(yaw_unwrapped, data["t"])
    all_numeric = np.column_stack([
        number(data["state"], field) for field in data["state"][0]
        if field not in ("event", "scheduled_modes")])
    nonfinite = int(np.size(all_numeric) - np.isfinite(all_numeric).sum())
    return {
        "method": data["manifest"]["method"],
        "statistics_interval_s": {"start_inclusive": 6.0, "end_exclusive": 20.0},
        "vx_rmse": float(np.sqrt(np.mean(vx_error ** 2))),
        "vx_peak_abs_error": float(np.max(np.abs(vx_error))),
        "vx_late_mean_error": float(np.mean(finite_or_nan(error[:, 0], late))),
        "vy_rmse": float(np.sqrt(np.mean(vy_error ** 2))),
        "max_abs_roll_rad": float(np.max(np.abs(finite_or_nan(data["angles"][:, 2], mask)))),
        "max_abs_pitch_rad": float(np.max(np.abs(finite_or_nan(data["angles"][:, 1], mask)))),
        "max_abs_yaw_rate_rad_s_fd": float(np.max(np.abs(finite_or_nan(yaw_rate, mask)))),
        "max_abs_vy_mps": float(np.max(np.abs(finite_or_nan(actual[:, 1], mask)))),
        "body_height_min_m": float(np.min(finite_or_nan(data["height"], mask))),
        "body_height_max_m": float(np.max(finite_or_nan(data["height"], mask))),
        "state_numeric_nonfinite_count": nonfinite,
    }


def save_vx(data, output):
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
    for item in data:
        method = item["manifest"]["method"]
        mask = interval(item)
        color = COLORS[method]
        axes[0].plot(item["t"][mask], item["actual"][mask, 0], color=color, label=NAMES[method])
        axes[1].plot(item["t"][mask], (item["actual"] - item["reference"])[mask, 0], color=color, label=NAMES[method])
    ref = data[0]
    mask = interval(ref)
    axes[0].plot(ref["t"][mask], ref["reference"][mask, 0], "k--", label="vx ref")
    axes[0].set_ylabel("vx (m/s)"); axes[1].set_ylabel("vx error (m/s)")
    axes[1].set_xlabel("Campaign time (s)")
    for axis in axes:
        axis.grid(alpha=.25); axis.legend()
    fig.tight_layout(); fig.savefig(output / "01_vx_tracking_and_error.png", dpi=250); plt.close(fig)


def save_stability(data, output):
    fields = ((1, "vy (m/s)"),)
    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(9, 10))
    for item in data:
        method = item["manifest"]["method"]; mask = interval(item); color = COLORS[method]
        axes[0].plot(item["t"][mask], item["actual"][mask, 1], color=color, label=NAMES[method])
        axes[1].plot(item["t"][mask], item["angles"][mask, 2], color=color)
        axes[2].plot(item["t"][mask], item["angles"][mask, 1], color=color)
        axes[3].plot(item["t"][mask], item["angles"][mask, 0], color=color)
        axes[4].plot(item["t"][mask], item["height"][mask], color=color)
    for axis, label in zip(axes, ("vy (m/s)", "roll (rad)", "pitch (rad)", "yaw (rad)", "body height z (m)")):
        axis.set_ylabel(label); axis.grid(alpha=.25)
    axes[0].legend(); axes[-1].set_xlabel("Campaign time (s)")
    fig.tight_layout(); fig.savefig(output / "02_lateral_attitude_height.png", dpi=250); plt.close(fig)


def save_direct(data, output):
    direct = next((item for item in data if item["manifest"]["method"] == "direct_wrench"), None)
    if direct is None or not (direct["path"] / "shadow.csv").exists():
        return
    _, rows = PLOTTER_MODULE.read_rows(direct["path"] / "shadow.csv")
    axes_names = ("fx", "fy", "fz", "tx", "ty", "tz")
    fig, axes = plt.subplots(2, 3, sharex=True, figsize=(10, 5))
    t = number(rows, "wall_elapsed")
    mask = interval({"t": t})
    for ax, axis in zip(axes.flat, axes_names):
        field = "estimate_after_" + axis + ("_n" if axis.startswith("f") else "_nm")
        ax.plot(t[mask], number(rows, field)[mask], color=COLORS["direct_wrench"])
        ax.set_title(axis); ax.grid(alpha=.25)
    fig.suptitle("Direct wrench comparator estimate (not full ACLF-MPC)")
    fig.tight_layout(); fig.savefig(output / "03_direct_wrench_estimate.png", dpi=250); plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    data = [load(path) for path in args.case]
    methods = {item["manifest"]["method"] for item in data}
    if methods != set(NAMES):
        raise ValueError("expected exactly pure_mpc, direct_wrench, v11_equivalent_wrench")
    args.output.mkdir(parents=True, exist_ok=False)
    values = [metrics(item) for item in data]
    (args.output / "metrics.json").write_text(json.dumps(values, indent=2) + "\n")
    save_vx(data, args.output); save_stability(data, args.output); save_direct(data, args.output)
    print(args.output)


if __name__ == "__main__":
    main()
