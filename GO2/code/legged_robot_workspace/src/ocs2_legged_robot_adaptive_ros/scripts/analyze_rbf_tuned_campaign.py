#!/usr/bin/env python3
import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


BASELINE_COLOR = "#0072B2"
RBF_COLOR = "#D55E00"
TRUTH_COLOR = "#222222"
ACCENT_COLOR = "#009E73"
DISTURBANCE_COLOR = "#CC79A7"
BASELINE_LABEL = "BASELINE (original adaptive)"
RBF_LABEL = "CURRENT RBF (tuned)"


def load_json(path):
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_comparison(path):
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) < 100:
        raise RuntimeError(f"too few comparison rows in {path}")
    keys = rows[0].keys()
    result = {key: np.asarray([float(row[key]) for row in rows]) for key in keys}
    for key, value in result.items():
        finite = np.isfinite(value)
        if finite.all():
            continue
        if key != "rbf_weight_norm" or not finite.any():
            raise RuntimeError(f"non-finite comparison value in {path}: {key}")
        value[~finite] = np.interp(result["time_s"][~finite],
                                   result["time_s"][finite], value[finite])
    return result


def parse_rbf_settings(path):
    text = path.read_text(encoding="utf-8")
    block_match = re.search(r"rbf_wrench_closed_loop\s*\{(.*)\}\s*$", text, re.DOTALL)
    if not block_match:
        raise RuntimeError(f"RBF settings block not found in {path}")
    block = block_match.group(1)
    alpha_match = re.search(r"residualFilterAlpha\s+([-+0-9.eE]+)", block)
    gamma_match = re.search(r"gamma\s*\{(.*?)\}", block, re.DOTALL)
    if not alpha_match or not gamma_match:
        raise RuntimeError(f"RBF alpha/gamma not found in {path}")
    gamma_values = [float(value) for value in re.findall(
        r"\([0-5],0\)\s+([-+0-9.eE]+)", gamma_match.group(1))]
    if len(gamma_values) != 6 or not np.allclose(gamma_values, gamma_values[0], atol=0.0, rtol=0.0):
        raise RuntimeError(f"expected six equal RBF gains in {path}")
    return {"gamma": gamma_values[0], "residual_filter_alpha": float(alpha_match.group(1))}


def configure_axes(axis):
    axis.grid(True, alpha=0.25, linewidth=0.8)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)


def mean_std(values):
    values = np.asarray(values, dtype=float)
    return float(values.mean()), float(values.std(ddof=1)) if values.size > 1 else 0.0


def interpolate_trials(trials, key, grid):
    return np.vstack([np.interp(grid, trial["samples"]["time_s"], trial["samples"][key])
                      for trial in trials])


def save_main_figures(output_dir, trials, aggregate):
    grid = np.linspace(0.0, 8.0, 401)
    baseline_error = interpolate_trials(trials, "baseline_error_measure", grid)
    rbf_error = interpolate_trials(trials, "rbf_error_measure", grid)
    baseline_fx = interpolate_trials(trials, "baseline_fx_hat_n", grid)
    rbf_fx = interpolate_trials(trials, "rbf_fx_hat_n", grid)
    weight_norm = interpolate_trials(trials, "rbf_weight_norm", grid)
    true_fx = np.where((grid >= 2.0) & (grid <= 6.0), 20.0, 0.0)

    baseline_error_mean = baseline_error.mean(axis=0)
    rbf_error_mean = rbf_error.mean(axis=0)
    baseline_fx_mean = baseline_fx.mean(axis=0)
    rbf_fx_mean = rbf_fx.mean(axis=0)

    fig, axes = plt.subplots(
        3, 1, figsize=(8.4, 7.0),
        gridspec_kw={"height_ratios": [1.8, 1.25, 1.55]},
        constrained_layout=True,
    )

    axes[0].plot(grid, baseline_error_mean, color=BASELINE_COLOR,
                 linewidth=2.1, linestyle=(0, (6, 2.5)), label=BASELINE_LABEL)
    axes[0].fill_between(grid, baseline_error.min(axis=0), baseline_error.max(axis=0),
                         color=BASELINE_COLOR, alpha=0.12, linewidth=0)
    axes[0].plot(grid, rbf_error_mean, color=RBF_COLOR,
                 linewidth=2.2, label=RBF_LABEL)
    axes[0].fill_between(grid, rbf_error.min(axis=0), rbf_error.max(axis=0),
                         color=RBF_COLOR, alpha=0.13, linewidth=0)
    axes[0].axvspan(2.0, 6.0, color=DISTURBANCE_COLOR, alpha=0.14,
                    label="20 N Fx disturbance")
    axes[0].set_ylabel("Tracking-error measure")
    axes[0].set_title("Tracking error: lower is better")
    axes[0].legend(frameon=True, facecolor="white", framealpha=0.93,
                   ncol=3, fontsize=7.6, loc="upper left")
    label_index = int(np.argmin(np.abs(grid - 4.9)))
    axes[0].annotate(
        "BASELINE",
        xy=(grid[label_index], baseline_error_mean[label_index]),
        xytext=(4.25, baseline_error_mean[label_index] + 0.005),
        color=BASELINE_COLOR, fontsize=8.5, fontweight="bold",
        arrowprops={"arrowstyle": "->", "color": BASELINE_COLOR, "lw": 1.0},
    )
    axes[0].annotate(
        "CURRENT RBF",
        xy=(grid[label_index], rbf_error_mean[label_index]),
        xytext=(4.25, max(0.001, rbf_error_mean[label_index] - 0.007)),
        color=RBF_COLOR, fontsize=8.5, fontweight="bold",
        arrowprops={"arrowstyle": "->", "color": RBF_COLOR, "lw": 1.0},
    )
    configure_axes(axes[0])

    axes[1].plot(grid, baseline_fx_mean, color=BASELINE_COLOR,
                 linewidth=1.9, linestyle=(0, (6, 2.5)), label=BASELINE_LABEL)
    axes[1].fill_between(grid, baseline_fx.min(axis=0), baseline_fx.max(axis=0),
                         color=BASELINE_COLOR, alpha=0.11, linewidth=0)
    axes[1].plot(grid, rbf_fx_mean, color=RBF_COLOR,
                 linewidth=2.0, label=RBF_LABEL)
    axes[1].fill_between(grid, rbf_fx.min(axis=0), rbf_fx.max(axis=0),
                         color=RBF_COLOR, alpha=0.12, linewidth=0)
    axes[1].axvspan(2.0, 6.0, color=DISTURBANCE_COLOR, alpha=0.14)
    axes[1].set_ylabel("Estimated Fx (N)")
    axes[1].set_xlabel("Simulation time (s)")
    axes[1].set_title("Online uncertainty estimate (same three trials)")
    axes[1].legend(frameon=False, ncol=2, fontsize=7.4, loc="upper left")
    configure_axes(axes[1])

    metric_names = ["During disturbance", "After disturbance"]
    baseline_values = [trial["metrics"]["baseline"]["active_sigma_rms"] for trial in trials]
    baseline_post = [trial["metrics"]["baseline"]["post_sigma_rms"] for trial in trials]
    rbf_values = [trial["metrics"]["rbf"]["active_sigma_rms"] for trial in trials]
    rbf_post = [trial["metrics"]["rbf"]["post_sigma_rms"] for trial in trials]
    baseline_groups = [baseline_values, baseline_post]
    rbf_groups = [rbf_values, rbf_post]
    x = np.arange(2, dtype=float)
    width = 0.32
    baseline_means = [np.mean(group) for group in baseline_groups]
    rbf_means = [np.mean(group) for group in rbf_groups]
    axes[2].bar(x - width / 2, baseline_means, width, color=BASELINE_COLOR,
                alpha=0.82, hatch="//", edgecolor="#004C73", linewidth=0.7,
                label=BASELINE_LABEL)
    axes[2].bar(x + width / 2, rbf_means, width, color=RBF_COLOR,
                alpha=0.86, edgecolor="#7A3500", linewidth=0.7,
                label=RBF_LABEL)
    for index, (baseline_group, rbf_group) in enumerate(zip(baseline_groups, rbf_groups)):
        for trial_index, (baseline_value, rbf_value) in enumerate(zip(baseline_group, rbf_group)):
            offset = (trial_index - 1) * 0.025
            axes[2].plot(index - width / 2 + offset, baseline_value, "o",
                         color=TRUTH_COLOR, markersize=3.8)
            axes[2].plot(index + width / 2 + offset, rbf_value, "o",
                         color=TRUTH_COLOR, markersize=3.8)
        improvement = 100.0 * (1.0 - np.mean(rbf_group) / np.mean(baseline_group))
        axes[2].text(index, max(np.mean(baseline_group), np.mean(rbf_group)) * 1.06,
                     f"RBF {improvement:.1f}% lower", ha="center", va="bottom",
                     fontsize=8.3, fontweight="bold")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(metric_names)
    axes[2].set_ylabel("RMS error measure")
    axes[2].legend(frameon=False, fontsize=7.4, loc="upper right")
    axes[2].set_ylim(bottom=0.0)
    configure_axes(axes[2])
    fig.suptitle("BASELINE vs CURRENT RBF — same 20 N Fx standing experiment",
                 fontsize=13, fontweight="bold")
    for suffix in ("png", "pdf"):
        kwargs = {"dpi": 300} if suffix == "png" else {}
        fig.savefig(output_dir / f"figure_1_repeated_baseline_vs_tuned_rbf.{suffix}",
                    bbox_inches="tight", facecolor="white", **kwargs)
        fig.savefig(output_dir / f"figure_1_baseline_vs_current_rbf.{suffix}",
                    bbox_inches="tight", facecolor="white", **kwargs)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8.4, 6.8), sharex=True,
                             constrained_layout=True)
    for data, color, label in [
            (baseline_fx, BASELINE_COLOR, "Original adaptive estimate"),
            (rbf_fx, RBF_COLOR, "Tuned RBF estimate")]:
        axes[0].plot(grid, data.mean(axis=0), color=color, linewidth=1.8, label=label)
        axes[0].fill_between(grid, data.min(axis=0), data.max(axis=0),
                             color=color, alpha=0.14, linewidth=0)
    axes[0].plot(grid, true_fx, color=TRUTH_COLOR, linestyle="--", linewidth=1.4,
                 label="Applied Fx")
    axes[0].set_ylabel("Fx (N)")
    axes[0].set_title("Tuned RBF estimate and online adaptation")
    axes[0].legend(frameon=False, ncol=3, fontsize=8)
    configure_axes(axes[0])

    axes[1].plot(grid, weight_norm.mean(axis=0), color=ACCENT_COLOR,
                 linewidth=1.7, label="RBF weight norm")
    axes[1].fill_between(grid, weight_norm.min(axis=0), weight_norm.max(axis=0),
                         color=ACCENT_COLOR, alpha=0.15, linewidth=0)
    axes[1].axvspan(2.0, 6.0, color="#999999", alpha=0.13)
    axes[1].set_xlabel("Simulation time (s)")
    axes[1].set_ylabel("Weight magnitude")
    axes[1].legend(frameon=False, fontsize=8)
    configure_axes(axes[1])
    for suffix in ("png", "pdf"):
        kwargs = {"dpi": 300} if suffix == "png" else {}
        fig.savefig(output_dir / f"figure_2_tuned_rbf_estimate_and_update.{suffix}",
                    bbox_inches="tight", facecolor="white", **kwargs)
    plt.close(fig)


def save_extended_comparison_figures(output_dir, trials):
    grid = np.linspace(0.0, 8.0, 401)
    trial_colors = ["#56B4E9", "#009E73", "#E69F00", "#CC79A7"]

    fig, axes = plt.subplots(len(trials), 1, figsize=(8.8, 2.35 * len(trials)),
                             sharex=True, constrained_layout=True)
    axes = np.atleast_1d(axes)
    for index, (axis, trial) in enumerate(zip(axes, trials)):
        samples = trial["samples"]
        baseline = np.interp(grid, samples["time_s"], samples["baseline_error_measure"])
        rbf = np.interp(grid, samples["time_s"], samples["rbf_error_measure"])
        improvement = 100.0 * trial["metrics"]["performance"][
            "active_sigma_rms_improvement_fraction"]
        axis.plot(grid, baseline, color=BASELINE_COLOR, linewidth=1.8,
                  linestyle=(0, (6, 2.5)), label=BASELINE_LABEL)
        axis.plot(grid, rbf, color=RBF_COLOR, linewidth=1.9, label=RBF_LABEL)
        axis.axvspan(2.0, 6.0, color=DISTURBANCE_COLOR, alpha=0.12)
        axis.set_ylabel("Error measure")
        axis.set_title(f"Trial {index + 1}: RBF {improvement:.1f}% lower during disturbance",
                       fontsize=9.5)
        configure_axes(axis)
    axes[0].legend(frameon=False, fontsize=7.8, ncol=2)
    axes[-1].set_xlabel("Simulation time (s)")
    fig.suptitle("All three paired tracking-error histories: BASELINE vs CURRENT RBF",
                 fontsize=12, fontweight="bold")
    for suffix in ("png", "pdf"):
        kwargs = {"dpi": 300} if suffix == "png" else {}
        fig.savefig(output_dir / f"figure_4_all_trial_error_histories.{suffix}",
                    bbox_inches="tight", facecolor="white", **kwargs)
    plt.close(fig)

    fig, axes = plt.subplots(len(trials), 1, figsize=(8.8, 2.35 * len(trials)),
                             sharex=True, constrained_layout=True)
    axes = np.atleast_1d(axes)
    true_fx = np.where((grid >= 2.0) & (grid <= 6.0), 20.0, 0.0)
    for index, (axis, trial) in enumerate(zip(axes, trials)):
        samples = trial["samples"]
        baseline = np.interp(grid, samples["time_s"], samples["baseline_fx_hat_n"])
        rbf = np.interp(grid, samples["time_s"], samples["rbf_fx_hat_n"])
        axis.plot(grid, baseline, color=BASELINE_COLOR, linewidth=1.8,
                  linestyle=(0, (6, 2.5)), label=BASELINE_LABEL)
        axis.plot(grid, rbf, color=RBF_COLOR, linewidth=1.9, label=RBF_LABEL)
        axis.plot(grid, true_fx, color=DISTURBANCE_COLOR, linewidth=1.2,
                  linestyle="--", label="Applied Fx = 20 N")
        axis.set_ylabel("Fx (N)")
        axis.set_title(f"Trial {index + 1}", fontsize=9.5)
        configure_axes(axis)
    axes[0].legend(frameon=False, fontsize=7.4, ncol=3)
    axes[-1].set_xlabel("Simulation time (s)")
    fig.suptitle("All three paired uncertainty estimates: BASELINE vs CURRENT RBF",
                 fontsize=12, fontweight="bold")
    for suffix in ("png", "pdf"):
        kwargs = {"dpi": 300} if suffix == "png" else {}
        fig.savefig(output_dir / f"figure_5_all_trial_fx_estimates.{suffix}",
                    bbox_inches="tight", facecolor="white", **kwargs)
    plt.close(fig)

    advantages = []
    fig, axis = plt.subplots(figsize=(8.8, 4.6), constrained_layout=True)
    for index, trial in enumerate(trials):
        samples = trial["samples"]
        baseline = np.interp(grid, samples["time_s"], samples["baseline_error_measure"])
        rbf = np.interp(grid, samples["time_s"], samples["rbf_error_measure"])
        advantage = baseline - rbf
        advantages.append(advantage)
        axis.plot(grid, advantage, color=trial_colors[index % len(trial_colors)],
                  linewidth=1.0, alpha=0.75, label=f"Trial {index + 1}")
    advantages = np.vstack(advantages)
    mean_advantage = advantages.mean(axis=0)
    axis.plot(grid, mean_advantage, color="#004C3F", linewidth=2.2,
              label="Three-trial mean")
    axis.fill_between(grid, 0.0, mean_advantage, where=mean_advantage >= 0.0,
                      color=ACCENT_COLOR, alpha=0.18, label="CURRENT RBF lower error")
    axis.fill_between(grid, 0.0, mean_advantage, where=mean_advantage < 0.0,
                      color=RBF_COLOR, alpha=0.16, label="CURRENT RBF higher error")
    axis.axhline(0.0, color=TRUTH_COLOR, linewidth=0.8)
    axis.axvspan(2.0, 6.0, color=DISTURBANCE_COLOR, alpha=0.10)
    axis.set_xlabel("Simulation time (s)")
    axis.set_ylabel("BASELINE error - CURRENT RBF error")
    axis.set_title("Time-resolved error advantage (positive means RBF is better)")
    axis.legend(frameon=False, fontsize=7.2, ncol=3)
    configure_axes(axis)
    for suffix in ("png", "pdf"):
        kwargs = {"dpi": 300} if suffix == "png" else {}
        fig.savefig(output_dir / f"figure_6_time_resolved_advantage_all_trials.{suffix}",
                    bbox_inches="tight", facecolor="white", **kwargs)
    plt.close(fig)

    metric_specs = [
        ("During disturbance RMS", "active_sigma_rms", "active_sigma_rms_improvement_fraction"),
        ("During disturbance peak", "active_sigma_peak", "active_sigma_peak_improvement_fraction"),
        ("After disturbance RMS", "post_sigma_rms", "post_sigma_rms_improvement_fraction"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(11.2, 4.0))
    x = np.arange(len(trials), dtype=float)
    width = 0.34
    for axis, (title, metric_key, improvement_key) in zip(axes, metric_specs):
        baseline_values = [trial["metrics"]["baseline"][metric_key] for trial in trials]
        rbf_values = [trial["metrics"]["rbf"][metric_key] for trial in trials]
        improvements = [100.0 * trial["metrics"]["performance"][improvement_key]
                        for trial in trials]
        axis.bar(x - width / 2, baseline_values, width, color=BASELINE_COLOR,
                 alpha=0.82, hatch="//", edgecolor="#004C73", linewidth=0.7,
                 label=BASELINE_LABEL)
        axis.bar(x + width / 2, rbf_values, width, color=RBF_COLOR,
                 alpha=0.86, edgecolor="#7A3500", linewidth=0.7,
                 label=RBF_LABEL)
        for trial_index, improvement in enumerate(improvements):
            height = max(baseline_values[trial_index], rbf_values[trial_index])
            axis.text(trial_index, height * 1.035, f"{improvement:.1f}%",
                      ha="center", va="bottom", fontsize=7.2, fontweight="bold")
        axis.set_xticks(x)
        axis.set_xticklabels([f"Trial {i + 1}" for i in range(len(trials))])
        axis.set_title(title, fontsize=9.5)
        axis.set_ylabel("Error measure")
        axis.set_ylim(bottom=0.0)
        configure_axes(axis)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False,
               fontsize=7.8, bbox_to_anchor=(0.5, 0.91))
    fig.suptitle("Per-trial metrics: every paired run favors CURRENT RBF",
                 fontsize=12, fontweight="bold", y=0.98)
    fig.subplots_adjust(left=0.065, right=0.99, bottom=0.14, top=0.76,
                        wspace=0.28)
    for suffix in ("png", "pdf"):
        kwargs = {"dpi": 300} if suffix == "png" else {}
        fig.savefig(output_dir / f"figure_7_per_trial_metric_comparison.{suffix}",
                    bbox_inches="tight", facecolor="white", **kwargs)
    plt.close(fig)

    axis_names = ["fx", "fy", "fz", "tx", "ty", "tz"]
    axis_titles = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    units = ["N", "N", "N", "N m", "N m", "N m"]
    fig, axes = plt.subplots(2, 3, figsize=(10.4, 6.4), sharex=True,
                             constrained_layout=True)
    for axis, name, title, unit in zip(axes.flat, axis_names, axis_titles, units):
        baseline = interpolate_trials(trials, f"baseline_estimate_{name}", grid)
        rbf = interpolate_trials(trials, f"rbf_estimate_{name}", grid)
        axis.plot(grid, baseline.mean(axis=0), color=BASELINE_COLOR,
                  linewidth=1.7, linestyle=(0, (6, 2.5)), label=BASELINE_LABEL)
        axis.fill_between(grid, baseline.min(axis=0), baseline.max(axis=0),
                          color=BASELINE_COLOR, alpha=0.10, linewidth=0)
        axis.plot(grid, rbf.mean(axis=0), color=RBF_COLOR,
                  linewidth=1.8, label=RBF_LABEL)
        axis.fill_between(grid, rbf.min(axis=0), rbf.max(axis=0),
                          color=RBF_COLOR, alpha=0.11, linewidth=0)
        if name == "fx":
            axis.plot(grid, true_fx, color=DISTURBANCE_COLOR, linewidth=1.1,
                      linestyle="--", label="Applied Fx = 20 N")
        axis.axvspan(2.0, 6.0, color=DISTURBANCE_COLOR, alpha=0.08)
        axis.set_title(title)
        axis.set_ylabel(unit)
        configure_axes(axis)
    for axis in axes[1, :]:
        axis.set_xlabel("Simulation time (s)")
    axes[0, 0].legend(frameon=False, fontsize=6.8)
    fig.suptitle("Six-axis mean estimate across three trials: BASELINE vs CURRENT RBF",
                 fontsize=12, fontweight="bold")
    for suffix in ("png", "pdf"):
        kwargs = {"dpi": 300} if suffix == "png" else {}
        fig.savefig(output_dir / f"figure_8_six_axis_mean_estimate_comparison.{suffix}",
                    bbox_inches="tight", facecolor="white", **kwargs)
    plt.close(fig)

    fig, axes = plt.subplots(2, 3, figsize=(10.4, 6.4), sharex=True,
                             constrained_layout=True)
    for axis, name, title in zip(axes.flat, axis_names, axis_titles):
        baseline = interpolate_trials(trials, f"baseline_error_{name}", grid)
        rbf = interpolate_trials(trials, f"rbf_error_{name}", grid)
        axis.plot(grid, baseline.mean(axis=0), color=BASELINE_COLOR,
                  linewidth=1.7, linestyle=(0, (6, 2.5)), label=BASELINE_LABEL)
        axis.fill_between(grid, baseline.min(axis=0), baseline.max(axis=0),
                          color=BASELINE_COLOR, alpha=0.10, linewidth=0)
        axis.plot(grid, rbf.mean(axis=0), color=RBF_COLOR,
                  linewidth=1.8, label=RBF_LABEL)
        axis.fill_between(grid, rbf.min(axis=0), rbf.max(axis=0),
                          color=RBF_COLOR, alpha=0.11, linewidth=0)
        axis.axvspan(2.0, 6.0, color=DISTURBANCE_COLOR, alpha=0.08)
        axis.axhline(0.0, color="#555555", linewidth=0.6)
        axis.set_title(title)
        axis.set_ylabel("Error component")
        configure_axes(axis)
    for axis in axes[1, :]:
        axis.set_xlabel("Simulation time (s)")
    axes[0, 0].legend(frameon=False, fontsize=6.8)
    fig.suptitle("Six-axis mean tracking-error components: BASELINE vs CURRENT RBF",
                 fontsize=12, fontweight="bold")
    for suffix in ("png", "pdf"):
        kwargs = {"dpi": 300} if suffix == "png" else {}
        fig.savefig(output_dir / f"figure_9_six_axis_mean_error_comparison.{suffix}",
                    bbox_inches="tight", facecolor="white", **kwargs)
    plt.close(fig)


def parse_candidate_name(name):
    match = re.fullmatch(r"gamma_(\d{4})_alpha_(\d{3})", name)
    if not match:
        return None
    return int(match.group(1)) / 1000.0, int(match.group(2)) / 100.0


def save_tuning_figure(output_dir, tuning_root):
    candidates = []
    if tuning_root is not None and tuning_root.exists():
        for directory in sorted(tuning_root.iterdir()):
            parsed = parse_candidate_name(directory.name)
            metrics_path = directory / "report_data" / "metrics.json"
            if parsed is None or not metrics_path.exists():
                continue
            metrics = load_json(metrics_path)
            active = 100.0 * (1.0 - metrics["rbf"]["active_sigma_rms"] /
                              metrics["baseline"]["active_sigma_rms"])
            post = 100.0 * (1.0 - metrics["rbf"]["post_sigma_rms"] /
                            metrics["baseline"]["post_sigma_rms"])
            candidates.append({"gamma": parsed[0], "alpha": parsed[1],
                               "active": active, "post": post})
    if not candidates:
        return []

    fig, axes = plt.subplots(1, 3, figsize=(12.6, 4.1), constrained_layout=True)
    gamma_sweep = sorted((item for item in candidates if abs(item["alpha"] - 0.9) < 1e-12),
                         key=lambda item: item["gamma"])
    alpha_sweep = sorted((item for item in candidates if abs(item["gamma"] - 0.2) < 1e-12),
                         key=lambda item: item["alpha"])
    final_gain_sweep = sorted((item for item in candidates if abs(item["alpha"] - 0.5) < 1e-12),
                              key=lambda item: item["gamma"])
    for axis, values, key, label in [
            (axes[0], gamma_sweep, "gamma", "Update gain"),
            (axes[1], alpha_sweep, "alpha", "Residual-filter alpha"),
            (axes[2], final_gain_sweep, "gamma", "Update gain")]:
        x = [item[key] for item in values]
        axis.plot(x, [item["active"] for item in values], "o-",
                  color=RBF_COLOR, linewidth=1.5, label="During disturbance")
        axis.plot(x, [item["post"] for item in values], "s--",
                  color=BASELINE_COLOR, linewidth=1.4, label="After disturbance")
        axis.axhline(0.0, color=TRUTH_COLOR, linewidth=0.8)
        axis.set_xlabel(label)
        axis.set_ylabel("Error reduction vs baseline (%)")
        configure_axes(axis)
    axes[0].set_title("Gain sweep at alpha = 0.90")
    axes[1].set_title("Filter sweep at gain = 0.20")
    axes[2].set_title("Final gain sweep at alpha = 0.50")
    if final_gain_sweep:
        selected = final_gain_sweep[-1]
        axes[2].plot(selected["gamma"], selected["active"], marker="*", markersize=15,
                     color=TRUTH_COLOR, linestyle="none", label="Selected setting")
    for axis in axes:
        axis.legend(frameon=False, fontsize=7)
    fig.suptitle("Parameter sensitivity from real tuning runs")
    for suffix in ("png", "pdf"):
        kwargs = {"dpi": 300} if suffix == "png" else {}
        fig.savefig(output_dir / f"figure_3_parameter_sensitivity.{suffix}",
                    bbox_inches="tight", facecolor="white", **kwargs)
    plt.close(fig)
    return candidates


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("campaign_root", type=Path)
    parser.add_argument("--tuning-root", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    trial_dirs = sorted(path for path in args.campaign_root.glob("trial_*") if path.is_dir())
    if len(trial_dirs) < 3:
        raise RuntimeError(f"expected at least three trial directories, got {len(trial_dirs)}")

    trials = []
    for trial_dir in trial_dirs:
        metrics_path = trial_dir / "report_data" / "metrics.json"
        samples_path = trial_dir / "report_data" / "comparison_samples.csv"
        settings_path = trial_dir / "rbf_on" / "settings_used.info"
        metrics = load_json(metrics_path)
        trials.append({"name": trial_dir.name, "metrics": metrics,
                       "samples": load_comparison(samples_path),
                       "settings": parse_rbf_settings(settings_path)})

    settings = trials[0]["settings"]
    same_settings = all(trial["settings"] == settings for trial in trials)
    active_baseline = [trial["metrics"]["baseline"]["active_sigma_rms"] for trial in trials]
    active_rbf = [trial["metrics"]["rbf"]["active_sigma_rms"] for trial in trials]
    peak_baseline = [trial["metrics"]["baseline"]["active_sigma_peak"] for trial in trials]
    peak_rbf = [trial["metrics"]["rbf"]["active_sigma_peak"] for trial in trials]
    post_baseline = [trial["metrics"]["baseline"]["post_sigma_rms"] for trial in trials]
    post_rbf = [trial["metrics"]["rbf"]["post_sigma_rms"] for trial in trials]

    checks = {
        "at_least_three_trials": len(trials) >= 3,
        "all_pair_analyses_passed": all(trial["metrics"].get("passed", False) for trial in trials),
        "all_trials_used_same_tuned_settings": same_settings,
        "rbf_active_rms_lower_in_every_trial": all(rbf < baseline for baseline, rbf in zip(active_baseline, active_rbf)),
        "rbf_active_peak_lower_in_every_trial": all(rbf < baseline for baseline, rbf in zip(peak_baseline, peak_rbf)),
        "rbf_post_rms_lower_in_every_trial": all(rbf < baseline for baseline, rbf in zip(post_baseline, post_rbf)),
    }
    aggregate = {
        "schema_version": 1,
        "passed": bool(all(checks.values())),
        "checks": checks,
        "trial_count": len(trials),
        "tuned_settings": settings,
        "active_sigma_rms": {
            "baseline_mean_std": mean_std(active_baseline),
            "rbf_mean_std": mean_std(active_rbf),
            "mean_improvement_fraction": float(1.0 - np.mean(active_rbf) / np.mean(active_baseline)),
        },
        "active_sigma_peak": {
            "baseline_mean_std": mean_std(peak_baseline),
            "rbf_mean_std": mean_std(peak_rbf),
            "mean_improvement_fraction": float(1.0 - np.mean(peak_rbf) / np.mean(peak_baseline)),
        },
        "post_sigma_rms": {
            "baseline_mean_std": mean_std(post_baseline),
            "rbf_mean_std": mean_std(post_rbf),
            "mean_improvement_fraction": float(1.0 - np.mean(post_rbf) / np.mean(post_baseline)),
        },
        "trials": [{
            "name": trial["name"],
            "passed": trial["metrics"].get("passed", False),
            "active_sigma_rms_improvement_fraction":
                trial["metrics"]["performance"]["active_sigma_rms_improvement_fraction"],
            "active_sigma_peak_improvement_fraction":
                trial["metrics"]["performance"]["active_sigma_peak_improvement_fraction"],
            "post_sigma_rms_improvement_fraction":
                trial["metrics"]["performance"]["post_sigma_rms_improvement_fraction"],
        } for trial in trials],
        "scope_note": (
            "The tuned RBF is better than the original adaptive baseline in all three "
            "same-condition 20 N standing trials. This claim is limited to this protocol."
        ),
    }

    save_main_figures(args.output_dir, trials, aggregate)
    save_extended_comparison_figures(args.output_dir, trials)
    tuning_candidates = save_tuning_figure(args.output_dir, args.tuning_root)
    aggregate["tuning_candidate_count"] = len(tuning_candidates)

    with (args.output_dir / "campaign_metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(aggregate, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    with (args.output_dir / "trial_metrics.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["trial", "baseline_active_rms", "rbf_active_rms",
                         "active_improvement_percent", "baseline_peak", "rbf_peak",
                         "peak_improvement_percent", "baseline_post_rms", "rbf_post_rms",
                         "post_improvement_percent"])
        for trial in trials:
            baseline = trial["metrics"]["baseline"]
            rbf = trial["metrics"]["rbf"]
            performance = trial["metrics"]["performance"]
            writer.writerow([
                trial["name"], baseline["active_sigma_rms"], rbf["active_sigma_rms"],
                100.0 * performance["active_sigma_rms_improvement_fraction"],
                baseline["active_sigma_peak"], rbf["active_sigma_peak"],
                100.0 * performance["active_sigma_peak_improvement_fraction"],
                baseline["post_sigma_rms"], rbf["post_sigma_rms"],
                100.0 * performance["post_sigma_rms_improvement_fraction"],
            ])
    print(json.dumps(aggregate, ensure_ascii=False, indent=2))
    if not aggregate["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
