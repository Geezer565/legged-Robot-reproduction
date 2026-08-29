#!/usr/bin/env python3

"""Summarize and plot the final Go2 MyRBF shadow-mode experiments."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


CASES = {
    "G0 Baseline": "G0_baseline",
    "G1 MyRBF / no disturbance": "G1_shadow_no_disturbance",
    "G2 MyRBF / constant 20 N Fy": "G2_constant_20N_fy_final",
    "G3 MyRBF / sine 15 N Fy": "G3_sine_15N_fy_final",
}


def read_numeric_csv(path):
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    data = {}
    if not rows:
        return data
    for key in rows[0]:
        values = []
        for row in rows:
            try:
                values.append(float(row[key]))
            except (TypeError, ValueError):
                values.append(float("nan"))
        data[key] = np.asarray(values, dtype=float)
    return data


def finite_mask(data, *keys):
    mask = np.ones(len(data[keys[0]]), dtype=bool)
    for key in keys:
        mask &= np.isfinite(data[key])
    return mask


def rms(values):
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(values * values))) if values.size else float("nan")


def pearson(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    if a.size < 3 or np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def best_lag_correlation(time_s, estimate, truth, max_lag_s=1.0):
    if len(time_s) < 5:
        return float("nan"), float("nan")
    dt = float(np.median(np.diff(time_s)))
    max_steps = max(1, int(max_lag_s / dt))
    best = (float("nan"), float("nan"))
    for shift in range(-max_steps, max_steps + 1):
        if shift < 0:
            corr = pearson(estimate[-shift:], truth[:shift])
        elif shift > 0:
            corr = pearson(estimate[:-shift], truth[shift:])
        else:
            corr = pearson(estimate, truth)
        if math.isfinite(corr) and (not math.isfinite(best[1]) or corr > best[1]):
            best = (shift * dt, corr)
    return best


def state_metrics(state, active_window=None):
    mask = finite_mask(state, "ros_time", "base_y", "base_z", "observation_base_roll", "observation_base_pitch")
    mask &= state["ros_time"] > 0.5
    t = state["ros_time"][mask]
    y = state["base_y"][mask]
    z = state["base_z"][mask]
    roll = state["observation_base_roll"][mask]
    pitch = state["observation_base_pitch"][mask]
    result = {
        "samples": int(t.size),
        "z_min_m": float(np.min(z)),
        "z_max_m": float(np.max(z)),
        "max_abs_roll_deg": float(np.degrees(np.max(np.abs(roll)))),
        "max_abs_pitch_deg": float(np.degrees(np.max(np.abs(pitch)))),
        "max_abs_y_m": float(np.max(np.abs(y))),
        "final_z_m": float(z[-1]),
    }
    if active_window is not None:
        active = (t >= active_window[0]) & (t <= active_window[1])
        pre = (t >= max(0.5, active_window[0] - 1.0)) & (t < active_window[0])
        reference_y = float(np.mean(y[pre])) if np.any(pre) else float(y[0])
        result.update({
            "active_max_y_shift_m": float(np.max(np.abs(y[active] - reference_y))),
            "active_min_z_m": float(np.min(z[active])),
            "active_max_abs_roll_deg": float(np.degrees(np.max(np.abs(roll[active])))),
            "active_max_abs_pitch_deg": float(np.degrees(np.max(np.abs(pitch[active])))),
        })
    return result


def load_case(root, directory):
    case_dir = root / directory
    result = {"dir": case_dir, "state": read_numeric_csv(case_dir / "state.csv")}
    shadow_path = case_dir / "shadow.csv"
    wrench_path = case_dir / "wrench.csv"
    if shadow_path.exists():
        result["shadow"] = read_numeric_csv(shadow_path)
    if wrench_path.exists():
        result["wrench"] = read_numeric_csv(wrench_path)
    return result


def active_window(wrench):
    active = (wrench["service_called"] > 0.5) & (wrench["service_success"] > 0.5)
    if not np.any(active):
        raise RuntimeError("No successfully applied wrench samples")
    return float(np.min(wrench["ros_time"][active])), float(np.max(wrench["ros_time"][active]))


def estimator_metrics(case, axis="fy"):
    shadow = case["shadow"]
    wrench = case["wrench"]
    window = active_window(wrench)
    smask = finite_mask(shadow, "time_s", "estimate_after_{}_n".format(axis))
    smask &= (shadow["time_s"] >= window[0]) & (shadow["time_s"] <= window[1])
    t = shadow["time_s"][smask]
    estimate = shadow["estimate_after_{}_n".format(axis)][smask]
    external = np.interp(t, wrench["ros_time"], wrench[axis])
    target = -external
    lag_s, lag_corr = best_lag_correlation(t, estimate, target)
    called = wrench["service_called"] > 0.5
    return {
        "active_start_s": window[0],
        "active_end_s": window[1],
        "active_samples": int(t.size),
        "wrench_calls": int(np.sum(called)),
        "wrench_success_rate": float(np.mean(wrench["service_success"][called])),
        "external_mean_n": float(np.mean(external)),
        "external_peak_abs_n": float(np.max(np.abs(external))),
        "ideal_compensation_mean_n": float(np.mean(target)),
        "estimate_mean_n": float(np.mean(estimate)),
        "estimate_peak_abs_n": float(np.max(np.abs(estimate))),
        "compensation_rmse_n": rms(estimate - target),
        "correlation_zero_lag": pearson(estimate, target),
        "best_lag_s": lag_s,
        "best_lag_correlation": lag_corr,
    }


def plot_stability(cases, output):
    colors = ["#4C78A8", "#59A14F", "#E45756", "#B279A2"]
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=False, constrained_layout=True)
    for (label, case), color in zip(cases.items(), colors):
        state = case["state"]
        mask = finite_mask(state, "ros_time", "base_y", "base_z") & (state["ros_time"] > 0.0)
        axes[0].plot(state["ros_time"][mask], state["base_z"][mask], label=label, color=color, linewidth=1.4)
        axes[1].plot(state["ros_time"][mask], 100.0 * state["base_y"][mask], label=label, color=color, linewidth=1.4)
    axes[0].set_ylabel("Base height z (m)")
    axes[1].set_ylabel("Lateral position y (cm)")
    axes[1].set_xlabel("Simulation time (s)")
    axes[0].set_title("Go2 stability during baseline and MyRBF shadow-mode trials")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
    fig.savefig(str(output), dpi=300, facecolor="white")
    plt.close(fig)


def plot_disturbance(case, title, output):
    state, shadow, wrench = case["state"], case["shadow"], case["wrench"]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
    axes[0].plot(wrench["ros_time"], wrench["fy"], color="#E45756", linewidth=1.0, alpha=0.55, label="External Fy")
    axes[0].plot(wrench["ros_time"], -wrench["fy"], color="#F28E2B", linewidth=1.7, linestyle="--", label="Ideal compensation (-Fy)")
    axes[0].plot(shadow["time_s"], shadow["estimate_after_fy_n"], color="#4C78A8", linewidth=1.4, label="MyRBF estimate")
    mask = finite_mask(state, "ros_time", "base_y", "observation_base_roll", "observation_base_pitch")
    axes[1].plot(state["ros_time"][mask], 100.0 * state["base_y"][mask], color="#59A14F", linewidth=1.4)
    axes[2].plot(state["ros_time"][mask], np.degrees(state["observation_base_roll"][mask]), label="Roll", color="#F28E2B")
    axes[2].plot(state["ros_time"][mask], np.degrees(state["observation_base_pitch"][mask]), label="Pitch", color="#B279A2")
    axes[0].set_ylabel("Fy (N)")
    axes[1].set_ylabel("Base y (cm)")
    axes[2].set_ylabel("Angle (deg)")
    axes[2].set_xlabel("Simulation time (s)")
    axes[0].set_title(title)
    axes[0].legend(loc="best")
    axes[2].legend(loc="best")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.savefig(str(output), dpi=300, facecolor="white")
    plt.close(fig)


def plot_six_axis(case, output):
    shadow = case["shadow"]
    fig, axes = plt.subplots(3, 2, figsize=(11, 9), sharex=True, constrained_layout=True)
    labels = (("fx", "Fx (N)"), ("fy", "Fy (N)"), ("fz", "Fz (N)"),
              ("tx", "Tx (Nm)"), ("ty", "Ty (Nm)"), ("tz", "Tz (Nm)"))
    for ax, (axis, ylabel) in zip(axes.flat, labels):
        suffix = "n" if axis.startswith("f") else "nm"
        ax.plot(shadow["time_s"], shadow["estimate_after_{}_{}".format(axis, suffix)], color="#4C78A8", linewidth=1.2)
        ax.axhline(0.0, color="black", linewidth=0.6)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)
    axes[-1, 0].set_xlabel("Simulation time (s)")
    axes[-1, 1].set_xlabel("Simulation time (s)")
    fig.suptitle("MyRBF six-axis estimate under constant 20 N lateral force")
    fig.savefig(str(output), dpi=300, facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    cases = {label: load_case(args.root, directory) for label, directory in CASES.items()}
    summary = {}
    for label, case in cases.items():
        window = active_window(case["wrench"]) if "wrench" in case else None
        summary[label] = {"state": state_metrics(case["state"], window)}
        if "wrench" in case:
            summary[label]["estimator_fy"] = estimator_metrics(case)

    g1 = cases["G1 MyRBF / no disturbance"]["shadow"]
    valid = g1["time_s"] > 1.0
    summary["G1 MyRBF / no disturbance"]["estimate_noise"] = {
        axis: {
            "rms": rms(g1["estimate_after_{}_{}".format(axis, "n" if axis.startswith("f") else "nm")][valid]),
            "peak_abs": float(np.max(np.abs(g1["estimate_after_{}_{}".format(axis, "n" if axis.startswith("f") else "nm")][valid]))),
        }
        for axis in ("fx", "fy", "fz", "tx", "ty", "tz")
    }

    with (args.output / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
    plot_stability(cases, args.output / "01_go2_stability_overview.png")
    plot_disturbance(cases["G2 MyRBF / constant 20 N Fy"], "Constant 20 N lateral force: applied force, estimate, and body response", args.output / "02_constant_20N_response.png")
    plot_disturbance(cases["G3 MyRBF / sine 15 N Fy"], "Sinusoidal lateral force: applied force, estimate, and body response", args.output / "03_sine_15N_response.png")
    plot_six_axis(cases["G2 MyRBF / constant 20 N Fy"], args.output / "04_constant_20N_six_axis_estimate.png")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
