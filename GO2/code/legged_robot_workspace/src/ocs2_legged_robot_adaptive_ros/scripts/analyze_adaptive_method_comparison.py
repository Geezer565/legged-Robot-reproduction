#!/usr/bin/env python3
"""Analyze one same-condition run of four adaptive-wrench estimators."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

try:
    import rosbag
except ImportError:  # Unit tests exercise array helpers without a ROS install.
    rosbag = None


RBF_TOPIC = "/rbf_closed_loop/sample"
UPDATE_TOPIC = "/adaptive_update/sample"
EQ10_TOPIC = "/task_b/equation10_sample"
DISTURBANCE_TOPIC = "/adaptive_update/disturbance_sample"
OBSERVATION_TOPIC = "/legged_robot_mpc_observation"

METHOD_ORDER = ("baseline", "current_rbf", "composite_rbf", "rff_ogd")
METHOD_LABELS = {
    "baseline": "BASELINE (original adaptive)",
    "current_rbf": "CURRENT RBF (tuned)",
    "composite_rbf": "PROPOSED RBF v3 (ours)",
    "rff_ogd": "RFF-OGD (SSI-MPC-inspired)",
}
METHOD_COLORS = {
    "baseline": "#0072B2",
    "current_rbf": "#D55E00",
    "composite_rbf": "#009E73",
    "rff_ogd": "#CC79A7",
}
METHOD_LINESTYLES = {
    "baseline": (0, (7, 2.5)),
    "current_rbf": "-",
    "composite_rbf": (0, (3, 1.5)),
    "rff_ogd": (0, (1, 1.5)),
}
AXIS_NAMES = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
AXIS_UNITS = ("N", "N", "N", "N m", "N m", "N m")
DEFAULT_DISTURBANCE_START = 2.0
DEFAULT_DISTURBANCE_END = 6.0
DEFAULT_DISTURBANCE_WRENCH = (20.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def disturbance_trace(time, disturbance_start, disturbance_end,
                      disturbance_wrench, disturbance_profile="constant",
                      disturbance_frequency_hz=0.25,
                      disturbance_secondary_wrench=(0.0,) * 6):
    time = np.asarray(time, dtype=float)
    primary = np.asarray(disturbance_wrench, dtype=float)
    secondary = np.asarray(disturbance_secondary_wrench, dtype=float)
    trace = np.zeros((time.size, 6), dtype=float)
    active = ((time > disturbance_start) &
              (time <= disturbance_end))
    if disturbance_profile == "constant":
        trace[active] = primary
    elif disturbance_profile == "sign_reversal":
        midpoint = 0.5 * (disturbance_start + disturbance_end)
        trace[active & (time <= midpoint)] = primary
        trace[active & (time > midpoint)] = secondary
    elif disturbance_profile == "sine":
        phase = (2.0 * math.pi * disturbance_frequency_hz *
                 (time[active] - disturbance_start))
        trace[active] = np.sin(phase)[:, None] * primary[None, :]
    elif disturbance_profile == "pulse_train":
        cycles = (disturbance_frequency_hz *
                  (time[active] - disturbance_start))
        primary_half = np.mod(cycles, 1.0) < 0.5
        trace[active] = np.where(primary_half[:, None],
                                 primary[None, :], secondary[None, :])
    elif disturbance_profile == "sine_cosine":
        phase = (2.0 * math.pi * disturbance_frequency_hz *
                 (time[active] - disturbance_start))
        trace[active] = (np.sin(phase)[:, None] * primary[None, :] +
                         np.cos(phase)[:, None] * secondary[None, :])
    else:
        raise RuntimeError(f"unsupported disturbance profile: {disturbance_profile}")
    return trace


def read_arrays(path):
    if rosbag is None:
        raise RuntimeError("rosbag is required to read experiment recordings")
    topics = {RBF_TOPIC: [], UPDATE_TOPIC: [], EQ10_TOPIC: [],
              DISTURBANCE_TOPIC: [], OBSERVATION_TOPIC: []}
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, _ in bag.read_messages(topics=list(topics)):
            if topic == OBSERVATION_TOPIC:
                topics[topic].append(np.concatenate((
                    np.asarray([message.time], dtype=float),
                    np.asarray(message.state.value, dtype=float))))
            else:
                topics[topic].append(np.asarray(message.data, dtype=float))
    return {key: np.vstack(value) if value else np.empty((0, 0))
            for key, value in topics.items()}


def _require_shape(name, value, columns, minimum_rows):
    if value.ndim != 2 or value.shape[0] < minimum_rows or value.shape[1] != columns:
        raise RuntimeError(
            f"{name} expected at least {minimum_rows} rows and {columns} columns, got {value.shape}")
    if not np.isfinite(value).all():
        raise RuntimeError(f"{name} contains NaN or Inf")


def _require_minimum_shape(name, value, minimum_columns, minimum_rows):
    if (value.ndim != 2 or value.shape[0] < minimum_rows or
            value.shape[1] < minimum_columns):
        raise RuntimeError(
            f"{name} expected at least {minimum_rows} rows and "
            f"{minimum_columns} columns, got {value.shape}")
    if not np.isfinite(value).all():
        raise RuntimeError(f"{name} contains NaN or Inf")


def validate_inputs(updates, diagnostics, observations=None, minimum_rows=100):
    if set(updates) != set(METHOD_ORDER):
        raise RuntimeError(f"updates must contain exactly {METHOD_ORDER}")
    expected_diagnostics = set(METHOD_ORDER) - {"baseline"}
    if set(diagnostics) != expected_diagnostics:
        raise RuntimeError(
            f"diagnostics must contain exactly {sorted(expected_diagnostics)}")
    for method in METHOD_ORDER:
        _require_shape(f"{method} adaptive update", updates[method], 16,
                       minimum_rows)
    for method in expected_diagnostics:
        _require_shape(f"{method} closed-loop diagnostic", diagnostics[method],
                       50, minimum_rows)
    if observations is not None:
        if set(observations) != set(METHOD_ORDER):
            raise RuntimeError(f"observations must contain exactly {METHOD_ORDER}")
        for method in METHOD_ORDER:
            _require_minimum_shape(f"{method} robot observation",
                                   observations[method], 13, minimum_rows)


def common_reference_pose(observations):
    """Return a shared initial pose and the largest inter-run mismatch."""
    if set(observations) != set(METHOD_ORDER):
        raise RuntimeError(f"observations must contain exactly {METHOD_ORDER}")
    common_start = max(float(observations[method][0, 0])
                       for method in METHOD_ORDER)
    poses = {}
    for method in METHOD_ORDER:
        data = observations[method]
        poses[method] = np.asarray([
            np.interp(common_start, data[:, 0], data[:, column])
            for column in range(7, 13)
        ])
    reference = poses["baseline"]
    mismatch = max(float(np.max(np.abs(poses[method] - reference)))
                   for method in METHOD_ORDER)
    return common_start, reference, mismatch


def _wrapped_angles(values):
    return np.arctan2(np.sin(values), np.cos(values))


def pose_reference_trace(time, reference_pose, locomotion_mode="stance",
                         walking_target_step=0.60,
                         walking_target_start=1.20,
                         walking_target_duration=12.0):
    """Build the commanded body-pose reference for stance or straight walking."""
    time = np.asarray(time, dtype=float)
    reference_pose = np.asarray(reference_pose, dtype=float)
    if reference_pose.shape != (6,) or not np.isfinite(reference_pose).all():
        raise RuntimeError("reference_pose must contain six finite values")
    if locomotion_mode not in {"stance", "walk"}:
        raise RuntimeError("locomotion_mode must be stance or walk")
    trace = np.tile(reference_pose, (time.size, 1))
    if locomotion_mode == "walk":
        phase = np.clip((time - walking_target_start) /
                        walking_target_duration, 0.0, 1.0)
        trace[:, 0] += walking_target_step * phase
    return trace


def motion_series_metrics(observation, reference_pose,
                          disturbance_start=DEFAULT_DISTURBANCE_START,
                          disturbance_end=DEFAULT_DISTURBANCE_END,
                          locomotion_mode="stance",
                          walking_target_step=0.60,
                          walking_target_start=1.20,
                          walking_target_duration=12.0):
    """Compute direct body-motion outcomes from the MPC observation state."""
    _require_minimum_shape("robot observation", observation, 13, 1)
    reference_trace = pose_reference_trace(
        observation[:, 0], reference_pose, locomotion_mode,
        walking_target_step, walking_target_start, walking_target_duration)
    position_error = observation[:, 7:10] - reference_trace[:, :3]
    orientation_error = _wrapped_angles(
        observation[:, 10:13] - reference_trace[:, 3:])
    position_norm = np.linalg.norm(position_error, axis=1)
    orientation_norm = np.linalg.norm(orientation_error, axis=1)
    height_abs = np.abs(position_error[:, 2])
    active = ((observation[:, 0] >= disturbance_start) &
              (observation[:, 0] <= disturbance_end))
    post = observation[:, 0] > disturbance_end
    if not active.any():
        raise RuntimeError(
            "robot observation has no samples in the disturbance window")

    def rms(values, mask):
        return float(np.sqrt(np.mean(np.square(values[mask]))))

    def peak(values, mask):
        return float(np.max(values[mask]))

    return {
        "sample_count": int(observation.shape[0]),
        "active_position_rms_m": rms(position_norm, active),
        "active_position_peak_m": peak(position_norm, active),
        "active_orientation_rms_rad": rms(orientation_norm, active),
        "active_orientation_peak_rad": peak(orientation_norm, active),
        "active_height_rms_m": rms(height_abs, active),
        "active_height_peak_m": peak(height_abs, active),
        "post_position_rms_m": rms(position_norm, post) if post.any() else None,
        "post_position_peak_m": peak(position_norm, post) if post.any() else None,
        "post_orientation_rms_rad": rms(orientation_norm, post) if post.any() else None,
        "post_orientation_peak_rad": peak(orientation_norm, post) if post.any() else None,
        "post_height_rms_m": rms(height_abs, post) if post.any() else None,
        "post_height_peak_m": peak(height_abs, post) if post.any() else None,
        "full_position_peak_m": float(np.max(position_norm)),
        "full_orientation_peak_rad": float(np.max(orientation_norm)),
        "full_height_peak_m": float(np.max(height_abs)),
    }


def comparison_motion_metrics(observations, reference_pose,
                              disturbance_start=DEFAULT_DISTURBANCE_START,
                              disturbance_end=DEFAULT_DISTURBANCE_END,
                              locomotion_mode="stance",
                              walking_target_step=0.60,
                              walking_target_start=1.20,
                              walking_target_duration=12.0):
    metrics = {
        method: motion_series_metrics(
            observations[method], reference_pose, disturbance_start,
            disturbance_end, locomotion_mode, walking_target_step,
            walking_target_start, walking_target_duration)
        for method in METHOD_ORDER
    }
    baseline = metrics["baseline"]
    improvements = {}
    for method in METHOD_ORDER[1:]:
        improvements[method] = {}
        for metric in ("active_position_rms_m",
                       "active_orientation_rms_rad",
                       "active_height_rms_m"):
            denominator = baseline[metric]
            improvements[method][metric + "_fraction"] = (
                float(1.0 - metrics[method][metric] / denominator)
                if denominator > 1.0e-15 else None)
    return {"metrics": metrics,
            "improvement_vs_baseline": improvements}


def series_metrics(update, disturbance_start=DEFAULT_DISTURBANCE_START,
                   disturbance_end=DEFAULT_DISTURBANCE_END,
                   disturbance_wrench=DEFAULT_DISTURBANCE_WRENCH,
                   disturbance_profile="constant",
                   disturbance_frequency_hz=0.25,
                   disturbance_secondary_wrench=(0.0,) * 6,
                   disturbance_truth=None):
    sigma_norm = np.linalg.norm(update[:, 10:16], axis=1)
    estimate_norm = np.linalg.norm(update[:, 4:10], axis=1)
    active = ((update[:, 0] >= disturbance_start) &
              (update[:, 0] <= disturbance_end))
    post = update[:, 0] > disturbance_end
    if not active.any():
        raise RuntimeError(
            "recording has no samples in the configured disturbance window")
    disturbance_wrench = np.asarray(disturbance_wrench, dtype=float)
    if disturbance_truth is None:
        truth_at_update = disturbance_trace(
            update[:, 0], disturbance_start, disturbance_end,
            disturbance_wrench, disturbance_profile,
            disturbance_frequency_hz, disturbance_secondary_wrench)
    else:
        _require_shape("disturbance truth", disturbance_truth, 8, 1)
        truth_at_update = np.column_stack([
            np.interp(update[:, 0], disturbance_truth[:, 0],
                      disturbance_truth[:, 2 + axis])
            for axis in range(6)
        ])
    active_truth = truth_at_update[active]
    dominant_axis = int(np.argmax(np.max(np.abs(active_truth), axis=0)))
    applied_value = float(disturbance_wrench[dominant_axis])
    estimate = update[:, 4 + dominant_axis]
    tail = active & (update[:, 0] >= max(
        disturbance_start, disturbance_end - 2.0))
    tail_estimate = estimate[tail]
    tail_time = update[tail, 0]
    tail_slope = (float(np.polyfit(tail_time, tail_estimate, 1)[0])
                  if tail_time.size >= 2 else None)
    reach_times = {}
    for fraction in (0.5, 0.8, 0.9):
        if applied_value == 0.0 or disturbance_profile != "constant":
            reach_times[f"reach_time_{int(fraction * 100)}_s"] = None
            continue
        reached = active & ((estimate / applied_value) >= fraction)
        reach_times[f"reach_time_{int(fraction * 100)}_s"] = (
            float(update[reached, 0][0] - disturbance_start)
            if reached.any() else None)
    return {
        "sample_count": int(update.shape[0]),
        "max_estimate_norm": float(estimate_norm.max()),
        "active_sigma_rms": float(np.sqrt(np.mean(np.square(sigma_norm[active])))),
        "active_sigma_peak": float(sigma_norm[active].max()),
        "post_sigma_rms": float(np.sqrt(np.mean(np.square(sigma_norm[post]))))
        if post.any() else None,
        "dominant_axis": AXIS_NAMES[dominant_axis],
        "applied_dominant_value": applied_value,
        "active_max_dominant_estimate": float(
            np.max(estimate[active]) if applied_value >= 0.0 else
            np.min(estimate[active])),
        "active_tail_dominant_estimate_mean": float(np.mean(tail_estimate)),
        "active_tail_dominant_estimate_std": float(np.std(tail_estimate)),
        "active_tail_estimate_fraction_of_applied": (
            float(np.mean(tail_estimate) / applied_value)
            if applied_value != 0.0 and disturbance_profile == "constant"
            else None),
        "active_tail_dominant_estimate_slope_per_s": tail_slope,
        "active_dominant_estimate_rmse_vs_applied": float(np.sqrt(np.mean(
            np.square(estimate[active] - active_truth[:, dominant_axis])))),
        "active_dominant_estimate_correlation_vs_applied": (
            float(np.corrcoef(estimate[active],
                              active_truth[:, dominant_axis])[0, 1])
            if np.std(estimate[active]) > 1.0e-12 and
            np.std(active_truth[:, dominant_axis]) > 1.0e-12 else None),
        "final_estimate": update[-1, 4:10].tolist(),
        **reach_times,
    }


def comparison_metrics(updates, disturbance_start=DEFAULT_DISTURBANCE_START,
                       disturbance_end=DEFAULT_DISTURBANCE_END,
                       disturbance_wrench=DEFAULT_DISTURBANCE_WRENCH,
                       disturbance_profile="constant",
                       disturbance_frequency_hz=0.25,
                       disturbance_secondary_wrench=(0.0,) * 6,
                       disturbances=None):
    metrics = {method: series_metrics(updates[method], disturbance_start,
                                      disturbance_end, disturbance_wrench,
                                      disturbance_profile,
                                      disturbance_frequency_hz,
                                      disturbance_secondary_wrench,
                                      disturbances[method]
                                      if disturbances is not None else None)
               for method in METHOD_ORDER}
    baseline = metrics["baseline"]
    improvements = {}
    for method in METHOD_ORDER[1:]:
        candidate = metrics[method]
        improvements[method] = {
            "active_sigma_rms_fraction": float(
                1.0 - candidate["active_sigma_rms"] /
                baseline["active_sigma_rms"]),
            "active_sigma_peak_fraction": float(
                1.0 - candidate["active_sigma_peak"] /
                baseline["active_sigma_peak"]),
            "post_sigma_rms_fraction": float(
                1.0 - candidate["post_sigma_rms"] /
                baseline["post_sigma_rms"])
            if baseline["post_sigma_rms"] not in (None, 0.0) and
            candidate["post_sigma_rms"] is not None else None,
        }
    ranking = sorted(METHOD_ORDER,
                     key=lambda method: metrics[method]["active_sigma_rms"])
    return {
        "methods": list(METHOD_ORDER),
        "metrics": metrics,
        "improvement_vs_baseline": improvements,
        "ranking_by_active_sigma_rms": ranking,
    }


def _configure_axes(axis):
    axis.grid(True, alpha=0.25, linewidth=0.8)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)


def _plot_method(axis, method, x, y, linewidth=1.7, label=True):
    axis.plot(x, y, color=METHOD_COLORS[method],
              linestyle=METHOD_LINESTYLES[method], linewidth=linewidth,
              label=METHOD_LABELS[method] if label else None)


def _save_figure(fig, output, stem):
    fig.savefig(output / f"{stem}.png", dpi=260, bbox_inches="tight",
                facecolor="white")
    fig.savefig(output / f"{stem}.pdf", bbox_inches="tight",
                facecolor="white")
    plt.close(fig)


def save_figures(output, updates, diagnostics,
                 disturbance_start=DEFAULT_DISTURBANCE_START,
                 disturbance_end=DEFAULT_DISTURBANCE_END,
                 disturbance_wrench=DEFAULT_DISTURBANCE_WRENCH,
                 disturbance_profile="constant",
                 disturbance_frequency_hz=0.25,
                 disturbance_secondary_wrench=(0.0,) * 6,
                 disturbance_truth=None,
                 observations=None,
                 reference_pose=None,
                 locomotion_mode="stance",
                 walking_target_step=0.60,
                 walking_target_start=1.20,
                 walking_target_duration=12.0):
    output.mkdir(parents=True, exist_ok=True)
    disturbance_color = "#E69F00"
    disturbance_wrench = np.asarray(disturbance_wrench, dtype=float)
    if disturbance_wrench.shape != (6,) or not np.isfinite(disturbance_wrench).all():
        raise RuntimeError("disturbance_wrench must contain six finite values")
    final_time = max(float(data[-1, 0]) for data in updates.values())
    if disturbance_truth is None:
        truth_time = np.linspace(0.0, final_time, 1001)
        truth_wrench = disturbance_trace(
            truth_time, disturbance_start, disturbance_end,
            disturbance_wrench, disturbance_profile,
            disturbance_frequency_hz, disturbance_secondary_wrench)
    else:
        _require_shape("disturbance truth", disturbance_truth, 8, 1)
        truth_time = disturbance_truth[:, 0]
        truth_wrench = disturbance_truth[:, 2:8]
    dominant_axis = int(np.argmax(np.max(np.abs(truth_wrench), axis=0)))
    dominant_truth = truth_wrench[:, dominant_axis]
    dominant_label = (f"Applied {AXIS_NAMES[dominant_axis]} "
                      f"({disturbance_profile})")

    fig, axes = plt.subplots(2, 1, figsize=(9.0, 6.8), sharex=True,
                             constrained_layout=True)
    for method in METHOD_ORDER:
        data = updates[method]
        _plot_method(axes[0], method, data[:, 0],
                     data[:, 4 + dominant_axis], linewidth=1.9)
    axes[0].plot(truth_time, dominant_truth,
                 color="#222222", linestyle="--", linewidth=1.4,
                 label=dominant_label)
    axes[0].set_ylabel(
        f"{AXIS_NAMES[dominant_axis]} estimate ({AXIS_UNITS[dominant_axis]})")
    axes[0].set_title("Dominant-wrench estimate comparison")
    axes[0].legend(frameon=False, ncol=2, fontsize=7.6)
    _configure_axes(axes[0])
    for method in METHOD_ORDER:
        data = updates[method]
        _plot_method(axes[1], method, data[:, 0],
                     np.linalg.norm(data[:, 10:16], axis=1), linewidth=1.9)
    axes[1].axvspan(disturbance_start, disturbance_end,
                    color=disturbance_color, alpha=0.12,
                    label="Applied disturbance")
    axes[1].set_xlabel("Simulation time (s)")
    axes[1].set_ylabel("Tracking-error measure")
    axes[1].legend(frameon=False, ncol=3, fontsize=7.2)
    _configure_axes(axes[1])
    _save_figure(fig, output, "figure_1_wrench_estimate_overview")

    fig, axes = plt.subplots(2, 3, figsize=(11.0, 6.5), sharex=True,
                             constrained_layout=True)
    for index, (axis, name, unit) in enumerate(
            zip(axes.flat, AXIS_NAMES, AXIS_UNITS)):
        for method in METHOD_ORDER:
            data = updates[method]
            _plot_method(axis, method, data[:, 0], data[:, 4 + index],
                         linewidth=1.35, label=index == 0)
        if np.max(np.abs(truth_wrench[:, index])) > 0.0:
            axis.plot(truth_time, truth_wrench[:, index],
                      color="#222222", linestyle="--", linewidth=1.0,
                      label=f"Applied {name}")
        axis.axvspan(disturbance_start, disturbance_end,
                    color=disturbance_color, alpha=0.09)
        axis.set_title(name)
        axis.set_ylabel(unit)
        _configure_axes(axis)
    axes[0, 0].legend(frameon=False, fontsize=6.3)
    for axis in axes[1, :]:
        axis.set_xlabel("Time (s)")
    fig.suptitle("Six-axis estimate comparison")
    _save_figure(fig, output, "figure_2_six_axis_estimates")

    fig, axes = plt.subplots(2, 3, figsize=(11.0, 6.5), sharex=True,
                             constrained_layout=True)
    for index, (axis, name) in enumerate(zip(axes.flat, AXIS_NAMES)):
        for method in METHOD_ORDER:
            data = updates[method]
            _plot_method(axis, method, data[:, 0], data[:, 10 + index],
                         linewidth=1.35, label=index == 0)
        axis.axvspan(disturbance_start, disturbance_end,
                    color=disturbance_color, alpha=0.09)
        axis.axhline(0.0, color="#555555", linewidth=0.7)
        axis.set_title(name)
        axis.set_ylabel("Error component")
        _configure_axes(axis)
    axes[0, 0].legend(frameon=False, fontsize=6.3)
    for axis in axes[1, :]:
        axis.set_xlabel("Time (s)")
    fig.suptitle("Six-axis tracking-error comparison")
    _save_figure(fig, output, "figure_3_six_axis_errors")

    fig, axes = plt.subplots(2, 1, figsize=(9.0, 6.2),
                             constrained_layout=True)
    transitions = (
        (axes[0], (max(0.0, disturbance_start - 0.3),
                   min(final_time, disturbance_start + 1.2)),
         disturbance_start, "Disturbance onset"),
        (axes[1], (max(0.0, disturbance_end - 0.3),
                   min(final_time, disturbance_end + 1.3)),
         disturbance_end, "Disturbance release"),
    )
    for axis, limits, event, title in transitions:
        for method in METHOD_ORDER:
            data = updates[method]
            _plot_method(axis, method, data[:, 0],
                         np.linalg.norm(data[:, 10:16], axis=1),
                         linewidth=1.8)
        axis.axvline(event, color=disturbance_color, linestyle="--",
                    linewidth=1.2)
        axis.set_xlim(*limits)
        axis.set_ylabel("Tracking-error measure")
        axis.set_title(title)
        _configure_axes(axis)
    axes[0].legend(frameon=False, ncol=2, fontsize=7.1)
    axes[1].set_xlabel("Simulation time (s)")
    fig.suptitle("Transient response detail")
    _save_figure(fig, output, "figure_4_transition_zoom")

    summary = comparison_metrics(
        updates, disturbance_start, disturbance_end, disturbance_wrench,
        disturbance_profile, disturbance_frequency_hz,
        disturbance_secondary_wrench)
    metric_names = ("active_sigma_rms", "active_sigma_peak", "post_sigma_rms")
    metric_labels = ("Active RMS", "Active peak", "Post RMS")
    x = np.arange(len(metric_names))
    width = 0.19
    fig, axis = plt.subplots(figsize=(9.2, 4.9), constrained_layout=True)
    for offset, method in enumerate(METHOD_ORDER):
        values = [summary["metrics"][method][name] for name in metric_names]
        axis.bar(x + (offset - 1.5) * width, values, width,
                 color=METHOD_COLORS[method], label=METHOD_LABELS[method])
    axis.set_xticks(x, metric_labels)
    axis.set_ylabel("Tracking-error measure")
    axis.set_title("Same-condition performance summary (lower is better)")
    axis.legend(frameon=False, ncol=2, fontsize=7.2)
    _configure_axes(axis)
    _save_figure(fig, output, "figure_5_metric_summary")

    fig, axes = plt.subplots(3, 1, figsize=(9.0, 8.2), sharex=True,
                             constrained_layout=True)
    for axis, method in zip(axes, METHOD_ORDER[1:]):
        data = diagnostics[method]
        axis.plot(data[:, 0], data[:, 11], color=METHOD_COLORS[method],
                  linewidth=1.6, label="Online weight norm")
        axis.plot(data[:, 0], data[:, 48], color="#E69F00",
                  linewidth=1.1, label="Weight-update rate norm")
        axis.axvspan(disturbance_start, disturbance_end,
                    color="#999999", alpha=0.10)
        axis.set_ylabel("Magnitude")
        axis.set_title(METHOD_LABELS[method])
        axis.legend(frameon=False, fontsize=7.0)
        _configure_axes(axis)
    axes[-1].set_xlabel("Simulation time (s)")
    fig.suptitle("Online adaptation evidence")
    _save_figure(fig, output, "figure_6_adaptation_evidence")

    baseline = updates["baseline"]
    baseline_sigma = np.linalg.norm(baseline[:, 10:16], axis=1)
    fig, axis = plt.subplots(figsize=(9.1, 4.8), constrained_layout=True)
    for method in METHOD_ORDER[1:]:
        data = updates[method]
        candidate_sigma = np.linalg.norm(data[:, 10:16], axis=1)
        common_baseline = np.interp(data[:, 0], baseline[:, 0], baseline_sigma)
        advantage = common_baseline - candidate_sigma
        axis.plot(data[:, 0], advantage, color=METHOD_COLORS[method],
                  linestyle=METHOD_LINESTYLES[method], linewidth=1.8,
                  label=f"BASELINE minus {METHOD_LABELS[method]}")
    axis.axhline(0.0, color="#222222", linewidth=0.8)
    axis.axvspan(disturbance_start, disturbance_end,
                color=disturbance_color, alpha=0.10)
    axis.set_xlabel("Simulation time (s)")
    axis.set_ylabel("Error advantage")
    axis.set_title("Positive means lower error than BASELINE")
    axis.legend(frameon=False, fontsize=6.8)
    _configure_axes(axis)
    _save_figure(fig, output, "figure_7_baseline_advantage")

    if observations is not None:
        if reference_pose is None:
            raise RuntimeError(
                "reference_pose is required when observations are supplied")
        fig, axes = plt.subplots(3, 1, figsize=(9.0, 8.0), sharex=True,
                                 constrained_layout=True)
        for method in METHOD_ORDER:
            data = observations[method]
            reference_trace = pose_reference_trace(
                data[:, 0], reference_pose, locomotion_mode,
                walking_target_step, walking_target_start,
                walking_target_duration)
            position_error = data[:, 7:10] - reference_trace[:, :3]
            orientation_error = _wrapped_angles(
                data[:, 10:13] - reference_trace[:, 3:])
            _plot_method(axes[0], method, data[:, 0],
                         np.linalg.norm(position_error, axis=1),
                         linewidth=1.7)
            _plot_method(axes[1], method, data[:, 0],
                         np.linalg.norm(orientation_error, axis=1),
                         linewidth=1.7)
            _plot_method(axes[2], method, data[:, 0],
                         np.abs(position_error[:, 2]), linewidth=1.7)
        labels = ("Body-position deviation (m)",
                  "Body-orientation deviation (rad)",
                  "Body-height deviation (m)")
        for axis, label in zip(axes, labels):
            axis.axvspan(disturbance_start, disturbance_end,
                        color=disturbance_color, alpha=0.10)
            axis.set_ylabel(label)
            _configure_axes(axis)
        axes[0].legend(frameon=False, ncol=2, fontsize=7.2)
        axes[-1].set_xlabel("Simulation time (s)")
        fig.suptitle("Direct body tracking and stability comparison")
        _save_figure(fig, output, "figure_8_direct_body_motion")


def _nearest_max_error(left, right, left_slice, right_slice):
    errors = []
    for row in left:
        index = int(np.argmin(np.abs(right[:, 0] - row[0])))
        if abs(right[index, 0] - row[0]) <= 1.0e-6:
            errors.append(np.linalg.norm(row[left_slice] -
                                         right[index, right_slice]))
    if not errors:
        raise RuntimeError("no time-matched samples were found")
    return float(max(errors)), len(errors)


def audit_integration(method, topics):
    diagnostic = topics[RBF_TOPIC]
    update = topics[UPDATE_TOPIC]
    eq10 = topics[EQ10_TOPIC]
    disturbance = topics[DISTURBANCE_TOPIC]
    _require_shape(f"{method} Eq.10 diagnostic", eq10, 24, 100)
    _require_shape(f"{method} disturbance truth", disturbance, 8, 100)
    monitor_error, monitor_matches = _nearest_max_error(
        diagnostic, update, slice(42, 48), slice(4, 10))
    dynamics_error, dynamics_matches = _nearest_max_error(
        diagnostic, eq10, slice(42, 48), slice(6, 12))
    checks = {
        "has_many_closed_loop_samples": diagnostic.shape[0] >= 100,
        "all_observations_matched": int(np.sum(diagnostic[:, 4] == 1.0)) == diagnostic.shape[0],
        "residual_and_weight_updates_occurred":
            int(np.sum(diagnostic[:, 5] == 1.0)) >= 100 and
            int(np.sum(diagnostic[:, 7] == 1.0)) >= 100,
        "solver_cycle_time_is_exact": float(np.max(np.abs(diagnostic[:, 2]))) <= 1.0e-12,
        "published_snapshot_equals_output": float(np.max(np.abs(diagnostic[:, 49]))) <= 1.0e-12,
        "controller_monitor_reads_same_estimate": monitor_error <= 1.0e-10,
        "dynamics_reads_same_estimate": dynamics_error <= 1.0e-10,
        "adaptive_dynamics_applies_candidate":
            float(np.max(np.abs(eq10[:, 12:18] - eq10[:, 18:24]))) <= 1.0e-12,
        "weights_changed_from_zero": float(np.max(diagnostic[:, 11])) > 1.0e-12,
        "estimate_changed_from_zero":
            float(np.max(np.linalg.norm(diagnostic[:, 36:42], axis=1))) > 1.0e-12,
        "all_recorded_values_are_finite": bool(
            np.isfinite(update).all() and np.isfinite(diagnostic).all() and
            np.isfinite(eq10).all() and np.isfinite(disturbance).all()),
    }
    return {
        "passed": bool(all(checks.values())),
        "checks": checks,
        "sample_count": int(diagnostic.shape[0]),
        "max_weight_norm": float(np.max(diagnostic[:, 11])),
        "max_weight_update_rate_norm": float(np.max(diagnostic[:, 48])),
        "max_estimate_norm": float(np.max(np.linalg.norm(
            diagnostic[:, 36:42], axis=1))),
        "max_controller_monitor_estimate_mismatch": monitor_error,
        "controller_monitor_time_matches": monitor_matches,
        "max_dynamics_estimate_mismatch": dynamics_error,
        "dynamics_time_matches": dynamics_matches,
    }


def write_csv(output, updates, diagnostics,
              disturbance_start=DEFAULT_DISTURBANCE_START,
              disturbance_end=DEFAULT_DISTURBANCE_END,
              disturbance_wrench=DEFAULT_DISTURBANCE_WRENCH,
              disturbance_profile="constant",
              disturbance_frequency_hz=0.25,
              disturbance_secondary_wrench=(0.0,) * 6,
              disturbance_truth=None,
              observations=None,
              reference_pose=None,
              locomotion_mode="stance",
              walking_target_step=0.60,
              walking_target_start=1.20,
              walking_target_duration=12.0):
    reference = updates[METHOD_ORDER[1]][:, 0]
    disturbance_wrench = np.asarray(disturbance_wrench, dtype=float)
    if disturbance_truth is None:
        reference_truth = disturbance_trace(
            reference, disturbance_start, disturbance_end,
            disturbance_wrench, disturbance_profile,
            disturbance_frequency_hz, disturbance_secondary_wrench)
    else:
        reference_truth = np.column_stack([
            np.interp(reference, disturbance_truth[:, 0],
                      disturbance_truth[:, 2 + axis])
            for axis in range(6)
        ])
    with (output / "comparison_samples.csv").open(
            "w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        header = ["time_s"] + [f"true_{name.lower()}_{unit.lower().replace(' ', '_')}"
                               for name, unit in zip(AXIS_NAMES, AXIS_UNITS)]
        for method in METHOD_ORDER:
            header.extend([
                f"{method}_{name.lower()}_estimate_{unit.lower().replace(' ', '_')}"
                for name, unit in zip(AXIS_NAMES, AXIS_UNITS)
            ])
            header.append(f"{method}_sigma_norm")
        for method in METHOD_ORDER[1:]:
            header.extend([f"{method}_weight_norm",
                           f"{method}_update_rate_norm"])
        if observations is not None:
            if reference_pose is None:
                raise RuntimeError(
                    "reference_pose is required when observations are supplied")
            for method in METHOD_ORDER:
                header.extend([f"{method}_position_deviation_m",
                               f"{method}_orientation_deviation_rad",
                               f"{method}_height_deviation_m"])
        writer.writerow(header)
        for row_index, time_value in enumerate(reference):
            row = [time_value] + reference_truth[row_index].tolist()
            for method in METHOD_ORDER:
                data = updates[method]
                row.extend([
                    np.interp(time_value, data[:, 0], data[:, 4 + axis])
                    for axis in range(6)
                ])
                row.append(np.interp(
                    time_value, data[:, 0],
                    np.linalg.norm(data[:, 10:16], axis=1)))
            for method in METHOD_ORDER[1:]:
                data = diagnostics[method]
                row.extend([
                    np.interp(time_value, data[:, 0], data[:, 11]),
                    np.interp(time_value, data[:, 0], data[:, 48]),
                ])
            if observations is not None:
                for method in METHOD_ORDER:
                    data = observations[method]
                    reference_at_time = pose_reference_trace(
                        np.asarray([time_value]), reference_pose,
                        locomotion_mode, walking_target_step,
                        walking_target_start, walking_target_duration)[0]
                    position_components = np.asarray([
                        np.interp(time_value, data[:, 0], data[:, column])
                        for column in range(7, 10)
                    ]) - reference_at_time[:3]
                    orientation_components = _wrapped_angles(np.asarray([
                        np.interp(time_value, data[:, 0], data[:, column])
                        for column in range(10, 13)
                    ]) - reference_at_time[3:])
                    row.extend([np.linalg.norm(position_components),
                                np.linalg.norm(orientation_components),
                                abs(position_components[2])])
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline_bag", type=Path)
    parser.add_argument("current_rbf_bag", type=Path)
    parser.add_argument("composite_rbf_bag", type=Path)
    parser.add_argument("rff_ogd_bag", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--disturbance-start", type=float,
                        default=DEFAULT_DISTURBANCE_START)
    parser.add_argument("--disturbance-end", type=float,
                        default=DEFAULT_DISTURBANCE_END)
    parser.add_argument("--disturbance-wrench", type=float, nargs=6,
                        default=DEFAULT_DISTURBANCE_WRENCH,
                        metavar=("FX", "FY", "FZ", "TX", "TY", "TZ"))
    parser.add_argument("--disturbance-profile",
                        choices=("constant", "sign_reversal", "sine",
                                 "pulse_train", "sine_cosine"),
                        default="constant")
    parser.add_argument("--disturbance-frequency-hz", type=float,
                        default=0.25)
    parser.add_argument("--disturbance-secondary-wrench", type=float, nargs=6,
                        default=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                        metavar=("FX2", "FY2", "FZ2", "TX2", "TY2", "TZ2"))
    parser.add_argument("--true-plant-mass-scale", type=float, default=1.0)
    parser.add_argument("--locomotion-mode", choices=("stance", "walk"),
                        default="stance")
    parser.add_argument("--walking-target-step", type=float, default=0.60)
    parser.add_argument("--walking-target-start", type=float, default=1.20)
    parser.add_argument("--walking-target-duration", type=float, default=12.0)
    args = parser.parse_args()
    if (not math.isfinite(args.disturbance_start) or
            not math.isfinite(args.disturbance_end) or
            args.disturbance_start < 0.0 or
            args.disturbance_end <= args.disturbance_start):
        parser.error("disturbance window must satisfy 0 <= start < end")
    if not all(math.isfinite(value) for value in args.disturbance_wrench):
        parser.error("disturbance wrench must contain finite values")
    if (not math.isfinite(args.disturbance_frequency_hz) or
            args.disturbance_frequency_hz <= 0.0):
        parser.error("disturbance frequency must be positive and finite")
    if not all(math.isfinite(value)
               for value in args.disturbance_secondary_wrench):
        parser.error("secondary disturbance wrench must contain finite values")
    if (not math.isfinite(args.true_plant_mass_scale) or
            args.true_plant_mass_scale <= 0.0):
        parser.error("true plant mass scale must be positive and finite")
    if (not math.isfinite(args.walking_target_step) or
            args.walking_target_step <= 0.0 or
            not math.isfinite(args.walking_target_start) or
            args.walking_target_start < 0.0 or
            not math.isfinite(args.walking_target_duration) or
            args.walking_target_duration <= 0.0):
        parser.error("walking target parameters are invalid")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    paths = {
        "baseline": args.baseline_bag,
        "current_rbf": args.current_rbf_bag,
        "composite_rbf": args.composite_rbf_bag,
        "rff_ogd": args.rff_ogd_bag,
    }
    all_topics = {method: read_arrays(path) for method, path in paths.items()}
    updates = {method: topics[UPDATE_TOPIC]
               for method, topics in all_topics.items()}
    diagnostics = {method: all_topics[method][RBF_TOPIC]
                   for method in METHOD_ORDER[1:]}
    disturbances = {method: all_topics[method][DISTURBANCE_TOPIC]
                    for method in METHOD_ORDER}
    observations = {method: all_topics[method][OBSERVATION_TOPIC]
                    for method in METHOD_ORDER}
    validate_inputs(updates, diagnostics, observations)
    if all_topics["baseline"][RBF_TOPIC].shape[0] != 0:
        raise RuntimeError("baseline unexpectedly published estimator diagnostics")

    integration = {method: audit_integration(method, all_topics[method])
                   for method in METHOD_ORDER[1:]}
    integration_passed = bool(all(item["passed"]
                                  for item in integration.values()))
    metrics = comparison_metrics(updates, args.disturbance_start,
                                 args.disturbance_end,
                                 args.disturbance_wrench,
                                 args.disturbance_profile,
                                 args.disturbance_frequency_hz,
                                 args.disturbance_secondary_wrench,
                                 disturbances)
    common_start, reference_pose, initial_pose_mismatch = (
        common_reference_pose(observations))
    initial_pose_passed = initial_pose_mismatch <= 1.0e-5
    integration_passed = integration_passed and initial_pose_passed
    motion = comparison_motion_metrics(
        observations, reference_pose, args.disturbance_start,
        args.disturbance_end, args.locomotion_mode,
        args.walking_target_step, args.walking_target_start,
        args.walking_target_duration)
    metrics.update({
        "schema_version": 4,
        "experiment_protocol": {
            "robot_model": "ANYmal-C",
            "simulation_backend": "OCS2 dummy simulation",
            "disturbance_start_s": args.disturbance_start,
            "disturbance_end_s": args.disturbance_end,
            "disturbance_wrench": list(args.disturbance_wrench),
            "disturbance_profile": args.disturbance_profile,
            "disturbance_frequency_hz": args.disturbance_frequency_hz,
            "disturbance_secondary_wrench":
                list(args.disturbance_secondary_wrench),
            "true_plant_mass_scale": args.true_plant_mass_scale,
            "locomotion_mode": args.locomotion_mode,
            "walking_target_step_m": args.walking_target_step,
            "walking_target_start_s": args.walking_target_start,
            "walking_target_duration_s": args.walking_target_duration,
        },
        "integration_passed": integration_passed,
        "integration": integration,
        "initial_pose_audit": {
            "common_start_s": common_start,
            "max_pose_difference_between_runs": initial_pose_mismatch,
            "reference_pose": reference_pose.tolist(),
            "passed": initial_pose_passed,
        },
        "direct_body_motion": motion,
        "scope_note": (
            "One same-condition run verifies wiring and provides a tuning comparison; "
            "repeat trials are needed for a robust performance claim."),
    })
    with (args.output_dir / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    write_csv(args.output_dir, updates, diagnostics, args.disturbance_start,
              args.disturbance_end, args.disturbance_wrench,
              args.disturbance_profile, args.disturbance_frequency_hz,
              args.disturbance_secondary_wrench,
              disturbances["baseline"], observations, reference_pose,
              args.locomotion_mode, args.walking_target_step,
              args.walking_target_start, args.walking_target_duration)
    save_figures(args.output_dir, updates, diagnostics,
                 args.disturbance_start, args.disturbance_end,
                 args.disturbance_wrench, args.disturbance_profile,
                 args.disturbance_frequency_hz,
                 args.disturbance_secondary_wrench,
                 disturbances["baseline"], observations, reference_pose,
                 args.locomotion_mode, args.walking_target_step,
                 args.walking_target_start, args.walking_target_duration)
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    if not integration_passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
