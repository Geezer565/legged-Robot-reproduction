#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


RBF_TOPIC = "/rbf_closed_loop/sample"
UPDATE_TOPIC = "/adaptive_update/sample"
EQ10_TOPIC = "/task_b/equation10_sample"
DISTURBANCE_TOPIC = "/adaptive_update/disturbance_sample"


def read_arrays(path):
    topics = {RBF_TOPIC: [], UPDATE_TOPIC: [], EQ10_TOPIC: [], DISTURBANCE_TOPIC: []}
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, _ in bag.read_messages(topics=list(topics)):
            topics[topic].append(np.asarray(message.data, dtype=float))
    return {key: np.vstack(value) if value else np.empty((0, 0))
            for key, value in topics.items()}


def require_shape(name, value, columns, minimum_rows=1):
    if value.ndim != 2 or value.shape[0] < minimum_rows or value.shape[1] != columns:
        raise RuntimeError(
            f"{name} expected at least {minimum_rows} rows and {columns} columns, got {value.shape}")
    if not np.isfinite(value).all():
        raise RuntimeError(f"{name} contains NaN or Inf")


def nearest_max_error(left, right, left_slice, right_slice):
    errors = []
    right_times = right[:, 0]
    for row in left:
        index = int(np.argmin(np.abs(right_times - row[0])))
        if abs(right_times[index] - row[0]) <= 1.0e-6:
            errors.append(np.linalg.norm(row[left_slice] - right[index, right_slice]))
    if not errors:
        raise RuntimeError("no time-matched samples were found")
    return float(max(errors)), len(errors)


def series_metrics(update):
    sigma_norm = np.linalg.norm(update[:, 10:16], axis=1)
    estimate_norm = np.linalg.norm(update[:, 4:10], axis=1)
    active = (update[:, 0] >= 2.0) & (update[:, 0] <= 6.0)
    post = update[:, 0] > 6.0
    return {
        "sample_count": int(update.shape[0]),
        "max_estimate_norm": float(estimate_norm.max()),
        "active_sigma_rms": float(np.sqrt(np.mean(np.square(sigma_norm[active])))),
        "active_sigma_peak": float(sigma_norm[active].max()),
        "post_sigma_rms": float(np.sqrt(np.mean(np.square(sigma_norm[post])))) if post.any() else None,
        "final_estimate": update[-1, 4:10].tolist(),
    }


def configure_axes(ax):
    ax.grid(True, alpha=0.25, linewidth=0.8)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def save_figures(output, baseline, rbf_update, rbf):
    colors = {"baseline": "#0072B2", "rbf": "#D55E00", "truth": "#222222",
              "disturbance": "#CC79A7"}
    baseline_label = "BASELINE (original adaptive)"
    rbf_label = "CURRENT RBF (tuned)"

    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.5), sharex=True,
                             constrained_layout=True)
    axes[0].plot(baseline[:, 0], baseline[:, 4], color=colors["baseline"],
                 linewidth=2.0, linestyle=(0, (6, 2.5)), label=baseline_label)
    axes[0].plot(rbf_update[:, 0], rbf_update[:, 4], color=colors["rbf"],
                 linewidth=2.1, label=rbf_label)
    axes[0].plot([0.0, 2.0, 2.0, 6.0, 6.0, 8.0],
                 [0.0, 0.0, 20.0, 20.0, 0.0, 0.0], color=colors["disturbance"],
                 linestyle="--", linewidth=1.5, label="Applied Fx = 20 N")
    axes[0].set_ylabel("Fx (N)")
    axes[0].set_title("Same-condition external-force experiment")
    axes[0].legend(frameon=False, ncol=3, fontsize=8)
    configure_axes(axes[0])

    baseline_sigma = np.linalg.norm(baseline[:, 10:16], axis=1)
    rbf_sigma = np.linalg.norm(rbf_update[:, 10:16], axis=1)
    axes[1].plot(baseline[:, 0], baseline_sigma, color=colors["baseline"],
                 linewidth=2.0, linestyle=(0, (6, 2.5)), label=baseline_label)
    axes[1].plot(rbf_update[:, 0], rbf_sigma, color=colors["rbf"],
                 linewidth=2.1, label=rbf_label)
    axes[1].axvspan(2.0, 6.0, color=colors["disturbance"], alpha=0.14,
                    label="20 N Fx disturbance")
    axes[1].set_xlabel("Simulation time (s)")
    axes[1].set_ylabel("Tracking-error measure")
    axes[1].legend(frameon=False, fontsize=8)
    configure_axes(axes[1])
    fig.savefig(output / "figure_1_baseline_vs_rbf.png", dpi=300,
                bbox_inches="tight", facecolor="white")
    fig.savefig(output / "figure_1_baseline_vs_rbf.pdf", bbox_inches="tight",
                facecolor="white")
    plt.close(fig)

    fig, axes = plt.subplots(2, 3, figsize=(9.2, 5.8), sharex=True,
                             constrained_layout=True)
    names = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    units = ["N", "N", "N", "N m", "N m", "N m"]
    for axis, name, unit, index in zip(axes.flat, names, units, range(6)):
        axis.plot(rbf_update[:, 0], rbf_update[:, 4 + index],
                  color=colors["rbf"], linewidth=1.45)
        if index == 0:
            axis.plot([0.0, 2.0, 2.0, 6.0, 6.0, 8.0],
                      [0.0, 0.0, 20.0, 20.0, 0.0, 0.0],
                      color=colors["truth"], linestyle="--", linewidth=1.1)
        axis.axvspan(2.0, 6.0, color="#999999", alpha=0.12)
        axis.set_title(name)
        axis.set_ylabel(unit)
        configure_axes(axis)
    for axis in axes[1, :]:
        axis.set_xlabel("Time (s)")
    fig.suptitle("Six-axis RBF closed-loop estimate")
    fig.savefig(output / "figure_2_six_axis_rbf_estimate.png", dpi=300,
                bbox_inches="tight", facecolor="white")
    fig.savefig(output / "figure_2_six_axis_rbf_estimate.pdf", bbox_inches="tight",
                facecolor="white")
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8.2, 6.3), sharex=True,
                             constrained_layout=True)
    axes[0].plot(rbf[:, 0], np.linalg.norm(rbf[:, 18:24], axis=1),
                 color="#009E73", linewidth=1.4, label="Filtered residual norm")
    axes[0].plot(rbf[:, 0], np.linalg.norm(rbf[:, 36:42], axis=1),
                 color=colors["rbf"], linewidth=1.4, label="RBF estimate norm")
    axes[0].axvspan(2.0, 6.0, color="#999999", alpha=0.14)
    axes[0].set_ylabel("Magnitude")
    axes[0].legend(frameon=False, fontsize=8)
    axes[0].set_title("RBF update evidence from the controller loop")
    configure_axes(axes[0])
    axes[1].plot(rbf[:, 0], rbf[:, 11], color="#CC79A7", linewidth=1.4,
                 label="Online weight norm")
    axes[1].plot(rbf[:, 0], rbf[:, 48], color="#E69F00", linewidth=1.2,
                 label="Weight-update rate norm")
    axes[1].axvspan(2.0, 6.0, color="#999999", alpha=0.14)
    axes[1].set_xlabel("Simulation time (s)")
    axes[1].set_ylabel("Magnitude")
    axes[1].legend(frameon=False, fontsize=8)
    configure_axes(axes[1])
    fig.savefig(output / "figure_3_rbf_update_evidence.png", dpi=300,
                bbox_inches="tight", facecolor="white")
    fig.savefig(output / "figure_3_rbf_update_evidence.pdf", bbox_inches="tight",
                facecolor="white")
    plt.close(fig)

    names = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    units = ["N", "N", "N", "N m", "N m", "N m"]
    fig, axes = plt.subplots(2, 3, figsize=(10.2, 6.3), sharex=True,
                             constrained_layout=True)
    for axis, name, unit, index in zip(axes.flat, names, units, range(6)):
        axis.plot(baseline[:, 0], baseline[:, 4 + index],
                  color=colors["baseline"], linewidth=1.7,
                  linestyle=(0, (6, 2.5)), label=baseline_label)
        axis.plot(rbf_update[:, 0], rbf_update[:, 4 + index],
                  color=colors["rbf"], linewidth=1.8, label=rbf_label)
        if index == 0:
            axis.plot([0.0, 2.0, 2.0, 6.0, 6.0, 8.0],
                      [0.0, 0.0, 20.0, 20.0, 0.0, 0.0],
                      color=colors["disturbance"], linestyle="--",
                      linewidth=1.2, label="Applied Fx = 20 N")
        axis.axvspan(2.0, 6.0, color=colors["disturbance"], alpha=0.11)
        axis.set_title(name)
        axis.set_ylabel(unit)
        configure_axes(axis)
    for axis in axes[1, :]:
        axis.set_xlabel("Simulation time (s)")
    axes[0, 0].legend(frameon=False, fontsize=7.0)
    fig.suptitle("Six-axis estimate comparison: BASELINE vs CURRENT RBF")
    fig.savefig(output / "figure_4_six_axis_estimate_comparison.png", dpi=300,
                bbox_inches="tight", facecolor="white")
    fig.savefig(output / "figure_4_six_axis_estimate_comparison.pdf",
                bbox_inches="tight", facecolor="white")
    plt.close(fig)

    fig, axes = plt.subplots(2, 3, figsize=(10.2, 6.3), sharex=True,
                             constrained_layout=True)
    for axis, name, index in zip(axes.flat, names, range(6)):
        axis.plot(baseline[:, 0], baseline[:, 10 + index],
                  color=colors["baseline"], linewidth=1.7,
                  linestyle=(0, (6, 2.5)), label=baseline_label)
        axis.plot(rbf_update[:, 0], rbf_update[:, 10 + index],
                  color=colors["rbf"], linewidth=1.8, label=rbf_label)
        axis.axvspan(2.0, 6.0, color=colors["disturbance"], alpha=0.11)
        axis.axhline(0.0, color="#555555", linewidth=0.7)
        axis.set_title(name)
        axis.set_ylabel("Error component")
        configure_axes(axis)
    for axis in axes[1, :]:
        axis.set_xlabel("Simulation time (s)")
    axes[0, 0].legend(frameon=False, fontsize=7.0)
    fig.suptitle("Six-axis tracking-error components: BASELINE vs CURRENT RBF")
    fig.savefig(output / "figure_5_six_axis_error_comparison.png", dpi=300,
                bbox_inches="tight", facecolor="white")
    fig.savefig(output / "figure_5_six_axis_error_comparison.pdf",
                bbox_inches="tight", facecolor="white")
    plt.close(fig)

    baseline_sigma = np.linalg.norm(baseline[:, 10:16], axis=1)
    rbf_sigma = np.linalg.norm(rbf_update[:, 10:16], axis=1)
    fig, axes = plt.subplots(2, 1, figsize=(8.4, 6.0), constrained_layout=True)
    for axis, limits, title in [
            (axes[0], (1.75, 3.25), "Disturbance onset (2 s)"),
            (axes[1], (5.75, 7.25), "Disturbance release (6 s)")]:
        axis.plot(baseline[:, 0], baseline_sigma, color=colors["baseline"],
                  linewidth=2.0, linestyle=(0, (6, 2.5)), label=baseline_label)
        axis.plot(rbf_update[:, 0], rbf_sigma, color=colors["rbf"],
                  linewidth=2.1, label=rbf_label)
        axis.axvline(2.0 if limits[0] < 2.0 else 6.0,
                    color=colors["disturbance"], linestyle="--", linewidth=1.2)
        axis.set_xlim(*limits)
        axis.set_ylabel("Tracking-error measure")
        axis.set_title(title)
        configure_axes(axis)
    axes[0].legend(frameon=False, fontsize=7.8)
    axes[1].set_xlabel("Simulation time (s)")
    fig.suptitle("Transient response detail: BASELINE vs CURRENT RBF")
    fig.savefig(output / "figure_6_disturbance_transition_zoom.png", dpi=300,
                bbox_inches="tight", facecolor="white")
    fig.savefig(output / "figure_6_disturbance_transition_zoom.pdf",
                bbox_inches="tight", facecolor="white")
    plt.close(fig)

    common_time = rbf_update[:, 0]
    baseline_common = np.interp(common_time, baseline[:, 0], baseline_sigma)
    advantage = baseline_common - rbf_sigma
    fig, axis = plt.subplots(figsize=(8.4, 4.2), constrained_layout=True)
    axis.plot(common_time, advantage, color="#009E73", linewidth=1.9,
              label="BASELINE error minus CURRENT RBF error")
    axis.fill_between(common_time, 0.0, advantage, where=advantage >= 0.0,
                      color="#009E73", alpha=0.20, label="RBF lower error")
    axis.fill_between(common_time, 0.0, advantage, where=advantage < 0.0,
                      color="#D55E00", alpha=0.18, label="RBF higher error")
    axis.axhline(0.0, color="#222222", linewidth=0.8)
    axis.axvspan(2.0, 6.0, color=colors["disturbance"], alpha=0.10)
    axis.set_xlabel("Simulation time (s)")
    axis.set_ylabel("Error advantage")
    axis.set_title("When does CURRENT RBF outperform BASELINE? (positive is better)")
    axis.legend(frameon=False, fontsize=7.6, ncol=3)
    configure_axes(axis)
    fig.savefig(output / "figure_7_time_resolved_error_advantage.png", dpi=300,
                bbox_inches="tight", facecolor="white")
    fig.savefig(output / "figure_7_time_resolved_error_advantage.pdf",
                bbox_inches="tight", facecolor="white")
    plt.close(fig)


def write_csv(output, baseline, rbf_update, rbf):
    times = rbf_update[:, 0]
    baseline_estimates = np.column_stack([
        np.interp(times, baseline[:, 0], baseline[:, 4 + index])
        for index in range(6)
    ])
    baseline_sigma_axes = np.column_stack([
        np.interp(times, baseline[:, 0], baseline[:, 10 + index])
        for index in range(6)
    ])
    baseline_fx = baseline_estimates[:, 0]
    baseline_sigma = np.interp(times, baseline[:, 0],
                               np.linalg.norm(baseline[:, 10:16], axis=1))
    rbf_by_time = {round(row[0], 9): row for row in rbf}
    with (output / "comparison_samples.csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        axis_names = ["fx", "fy", "fz", "tx", "ty", "tz"]
        header = ["time_s", "true_fx_n", "baseline_fx_hat_n",
                  "rbf_fx_hat_n", "baseline_error_measure",
                  "rbf_error_measure", "rbf_weight_norm"]
        for name in axis_names:
            header.extend([f"baseline_estimate_{name}", f"rbf_estimate_{name}"])
        for name in axis_names:
            header.extend([f"baseline_error_{name}", f"rbf_error_{name}"])
        writer.writerow(header)
        for index, row in enumerate(rbf_update):
            key = round(row[0], 9)
            diagnostic = rbf_by_time.get(key)
            weight_norm = diagnostic[11] if diagnostic is not None else math.nan
            true_fx = 20.0 if 2.0 <= row[0] <= 6.0 else 0.0
            values = [row[0], true_fx, baseline_fx[index], row[4],
                      baseline_sigma[index], np.linalg.norm(row[10:16]),
                      weight_norm]
            for axis_index in range(6):
                values.extend([baseline_estimates[index, axis_index],
                               row[4 + axis_index]])
            for axis_index in range(6):
                values.extend([baseline_sigma_axes[index, axis_index],
                               row[10 + axis_index]])
            writer.writerow(values)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline_bag", type=Path)
    parser.add_argument("rbf_bag", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    baseline_topics = read_arrays(args.baseline_bag)
    rbf_topics = read_arrays(args.rbf_bag)
    baseline = baseline_topics[UPDATE_TOPIC]
    rbf_update = rbf_topics[UPDATE_TOPIC]
    rbf = rbf_topics[RBF_TOPIC]
    eq10 = rbf_topics[EQ10_TOPIC]
    disturbance = rbf_topics[DISTURBANCE_TOPIC]

    require_shape("baseline adaptive update", baseline, 16, 100)
    require_shape("RBF adaptive update", rbf_update, 16, 100)
    require_shape("RBF closed-loop diagnostic", rbf, 50, 100)
    require_shape("RBF Eq.10 diagnostic", eq10, 24, 100)
    require_shape("RBF disturbance truth", disturbance, 8, 100)
    if baseline_topics[RBF_TOPIC].shape[0] != 0:
        raise RuntimeError("baseline unexpectedly published RBF closed-loop diagnostics")

    snapshot_update_error, update_matches = nearest_max_error(
        rbf, rbf_update, slice(42, 48), slice(4, 10))
    snapshot_eq10_error, eq10_matches = nearest_max_error(
        rbf, eq10, slice(42, 48), slice(6, 12))
    applied_candidate_error = float(np.max(np.abs(eq10[:, 12:18] - eq10[:, 18:24])))

    matched_count = int(np.sum(rbf[:, 4] == 1.0))
    residual_valid_count = int(np.sum(rbf[:, 5] == 1.0))
    weights_updated_count = int(np.sum(rbf[:, 7] == 1.0))
    checks = {
        "baseline_has_no_rbf_writer": baseline_topics[RBF_TOPIC].shape[0] == 0,
        "rbf_has_many_closed_loop_samples": rbf.shape[0] >= 100,
        "all_observations_matched": matched_count == rbf.shape[0],
        "residual_and_weight_updates_occurred": residual_valid_count >= 100 and weights_updated_count >= 100,
        "solver_cycle_time_is_exact": float(np.max(np.abs(rbf[:, 2]))) <= 1.0e-12,
        "published_snapshot_equals_rbf_output": float(np.max(np.abs(rbf[:, 49]))) <= 1.0e-12,
        "controller_monitor_reads_same_estimate": snapshot_update_error <= 1.0e-10,
        "dynamics_reads_same_estimate": snapshot_eq10_error <= 1.0e-10,
        "adaptive_dynamics_applies_candidate": applied_candidate_error <= 1.0e-12,
        "weights_changed_from_zero": float(np.max(rbf[:, 11])) > 1.0e-12,
        "estimate_changed_from_zero": float(np.max(np.linalg.norm(rbf[:, 36:42], axis=1))) > 1.0e-12,
        "all_recorded_values_are_finite": bool(
            np.isfinite(baseline).all() and np.isfinite(rbf_update).all() and
            np.isfinite(rbf).all() and np.isfinite(eq10).all() and
            np.isfinite(disturbance).all()),
    }

    baseline_metrics = series_metrics(baseline)
    rbf_metrics = series_metrics(rbf_update)
    performance = {
        "active_sigma_rms_improvement_fraction": float(
            1.0 - rbf_metrics["active_sigma_rms"] / baseline_metrics["active_sigma_rms"]),
        "active_sigma_peak_improvement_fraction": float(
            1.0 - rbf_metrics["active_sigma_peak"] / baseline_metrics["active_sigma_peak"]),
        "post_sigma_rms_improvement_fraction": float(
            1.0 - rbf_metrics["post_sigma_rms"] / baseline_metrics["post_sigma_rms"]),
    }
    performance_checks = {
        "rbf_active_sigma_rms_at_least_5_percent_lower":
            performance["active_sigma_rms_improvement_fraction"] >= 0.05,
        "rbf_active_sigma_peak_is_lower":
            performance["active_sigma_peak_improvement_fraction"] > 0.0,
        "rbf_post_sigma_rms_at_least_5_percent_lower":
            performance["post_sigma_rms_improvement_fraction"] >= 0.05,
    }
    integration_passed = bool(all(checks.values()))
    performance_passed = bool(all(performance_checks.values()))

    metrics = {
        "schema_version": 2,
        "passed": integration_passed and performance_passed,
        "integration_passed": integration_passed,
        "performance_passed": performance_passed,
        "checks": checks,
        "performance_checks": performance_checks,
        "performance": performance,
        "baseline": baseline_metrics,
        "rbf": rbf_metrics,
        "closed_loop_evidence": {
            "rbf_sample_count": int(rbf.shape[0]),
            "observation_matched_count": matched_count,
            "residual_valid_count": residual_valid_count,
            "weights_updated_count": weights_updated_count,
            "max_solver_cycle_time_error_s": float(np.max(np.abs(rbf[:, 2]))),
            "max_snapshot_output_mismatch": float(np.max(np.abs(rbf[:, 49]))),
            "max_controller_monitor_estimate_mismatch": snapshot_update_error,
            "controller_monitor_time_matches": update_matches,
            "max_dynamics_estimate_mismatch": snapshot_eq10_error,
            "dynamics_time_matches": eq10_matches,
            "max_applied_candidate_mismatch": applied_candidate_error,
            "max_weight_norm": float(np.max(rbf[:, 11])),
            "max_weight_update_rate_norm": float(np.max(rbf[:, 48])),
            "max_rbf_estimate_norm": float(np.max(np.linalg.norm(rbf[:, 36:42], axis=1))),
            "final_rbf_estimate": rbf[-1, 36:42].tolist(),
        },
        "scope_note": (
            "This paired run verifies integration and same-condition performance. "
            "Repeated paired runs are required before claiming a robust improvement."
        ),
    }
    with (args.output_dir / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    write_csv(args.output_dir, baseline, rbf_update, rbf)
    save_figures(args.output_dir, baseline, rbf_update, rbf)
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    if not metrics["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
