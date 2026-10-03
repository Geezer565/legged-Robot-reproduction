#!/usr/bin/env python3
"""Unified analysis of the Task C nominal, uncertain-ON, and uncertain-OFF campaign."""

import argparse
import csv
import hashlib
import json
import math
import os

import numpy as np


GROUPS = ("nominal", "uncertain_adaptive", "uncertain_off")
RUNS = ("run_01", "run_02", "run_03")
COMPONENTS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
FORCE_COMPONENTS = ("Fx", "Fy", "Fz")
TORQUE_COMPONENTS = ("Tx", "Ty", "Tz")
WINDOW_START = 0.50
WINDOW_END = 15.50
GRID_DT = 0.02
TRANSITION_END = 1.50
POLICY_DIMENSION = 24
CONTACT_FORCE_DIMENSION = 12
SIGN_THRESHOLDS = {
    "Fx": 1.0, "Fy": 1.0, "Fz": 1.0,
    "Tx": 0.1, "Ty": 0.1, "Tz": 0.1,
}
RATIO_DENOMINATOR_THRESHOLDS = {
    "Fx": 0.1, "Fy": 0.1, "Fz": 0.1,
    "Tx": 0.01, "Ty": 0.01, "Tz": 0.01,
}
GROUP_LABELS = {
    "nominal": "Nominal plant + Full Adaptive",
    "uncertain_adaptive": "+10% mass plant + Full Adaptive",
    "uncertain_off": "+10% mass plant + Adaptive OFF",
}
GROUP_COLORS = {
    "nominal": "#4c4c4c",
    "uncertain_adaptive": "#d62728",
    "uncertain_off": "#1f77b4",
}
PAIR_SPECS = (
    ("uncertain_adaptive_minus_nominal", "nominal", "uncertain_adaptive"),
    ("uncertain_off_minus_nominal", "nominal", "uncertain_off"),
    ("uncertain_adaptive_minus_uncertain_off", "uncertain_off", "uncertain_adaptive"),
)
OBSERVATION_TOPIC = "/legged_robot_mpc_observation"
TARGET_TOPIC = "/legged_robot_mpc_target"


def load_json(path):
    with open(path) as stream:
        return json.load(stream)


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_csv(path):
    with open(path, newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        names = reader.fieldnames
    if not rows:
        raise RuntimeError("empty Task C CSV: {}".format(path))
    data = {name: np.asarray([float(row[name]) for row in rows], dtype=float)
            for name in names}
    if any(not np.all(np.isfinite(values)) for values in data.values()):
        raise RuntimeError("non-finite Task C CSV: {}".format(path))
    time = data["observation_time"]
    if not np.all(np.diff(time) > 0.0):
        raise RuntimeError("non-increasing Task C CSV time: {}".format(path))
    if time[0] > WINDOW_START or time[-1] < WINDOW_END:
        raise RuntimeError("Task C CSV does not cover common window: {}".format(path))
    return data


def load_observation_and_target(path):
    import rosbag

    observations = []
    targets = []
    with rosbag.Bag(path, "r") as bag:
        for topic, message, _ in bag.read_messages(topics=[OBSERVATION_TOPIC, TARGET_TOPIC]):
            if topic == OBSERVATION_TOPIC:
                observations.append((
                    float(message.time), int(message.mode),
                    np.asarray(message.state.value, dtype=float),
                    np.asarray(message.input.value, dtype=float),
                ))
            else:
                targets.append((
                    np.asarray(message.timeTrajectory, dtype=float),
                    [np.asarray(value.value, dtype=float) for value in message.stateTrajectory],
                ))
    if not observations or len(targets) != 1:
        raise RuntimeError("bag must contain observations and exactly one target: {}".format(path))

    # Keep the last sample if a ROS startup boundary publishes a duplicate time.
    by_time = {row[0]: row for row in observations}
    observations = [by_time[key] for key in sorted(by_time)]
    time = np.asarray([row[0] for row in observations])
    modes = np.asarray([row[1] for row in observations])
    states = np.vstack([row[2] for row in observations])
    policy = np.vstack([row[3] for row in observations])
    if time[0] > WINDOW_START or time[-1] < WINDOW_END:
        raise RuntimeError("observation bag does not cover common window: {}".format(path))
    if states.shape[1] < 12 or policy.shape[1] != POLICY_DIMENSION:
        raise RuntimeError("unexpected observation dimensions in {}".format(path))
    if not (np.all(np.isfinite(states)) and np.all(np.isfinite(policy))):
        raise RuntimeError("non-finite observation in {}".format(path))

    target_times, target_states = targets[0]
    if len(target_times) != 2 or len(target_states) != 2 or min(map(len, target_states)) < 12:
        raise RuntimeError("target is not the deterministic two-knot trajectory: {}".format(path))
    return {
        "time": time,
        "modes": modes,
        "states": states,
        "policy": policy,
        "target_times": target_times,
        "target_states": np.vstack(target_states),
    }


def interpolate_matrix(grid, time, matrix):
    return np.column_stack([np.interp(grid, time, matrix[:, index])
                            for index in range(matrix.shape[1])])


def interpolate_csv(raw, grid):
    time = raw["observation_time"]
    return {name: np.interp(grid, time, values) for name, values in raw.items()}


def vector(values, prefix, components=COMPONENTS):
    return np.column_stack([values[prefix + component] for component in components])


def wrap_angle(value):
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def trapz(values, time):
    return float(np.trapz(values, time))


def scalar_rmse(values, time):
    return math.sqrt(trapz(values ** 2, time) / float(time[-1] - time[0]))


def vector_rmse(values, time):
    return math.sqrt(trapz(np.sum(values ** 2, axis=1), time) /
                     float(time[-1] - time[0]))


def time_mean(values, time):
    return trapz(values, time) / float(time[-1] - time[0])


def lag_estimate(pi_star, pi_hat, threshold):
    star_rms = math.sqrt(float(np.mean(pi_star ** 2)))
    star_std = float(np.std(pi_star))
    hat_std = float(np.std(pi_hat))
    # A nearly constant or negligible truth signal has no meaningful lag.
    if (star_rms < threshold or star_std < max(0.1 * threshold, 0.05 * star_rms)
            or hat_std < 1.0e-9):
        return None, None
    truth = pi_star - np.mean(pi_star)
    estimate = pi_hat - np.mean(pi_hat)
    best_lag = None
    best_correlation = -float("inf")
    max_lag_samples = int(round(2.0 / GRID_DT))
    for lag in range(-max_lag_samples, max_lag_samples + 1):
        if lag > 0:
            left, right = truth[:-lag], estimate[lag:]
        elif lag < 0:
            left, right = truth[-lag:], estimate[:lag]
        else:
            left, right = truth, estimate
        if len(left) < 20 or np.std(left) <= 1.0e-12 or np.std(right) <= 1.0e-12:
            continue
        correlation = float(np.corrcoef(left, right)[0, 1])
        if math.isfinite(correlation) and correlation > best_correlation:
            best_correlation = correlation
            best_lag = lag * GRID_DT
    if best_lag is None or best_correlation < 0.2:
        return None, None
    # Positive means pi_hat follows pi_star later in time.
    return float(best_lag), float(best_correlation)


def parameter_metrics(time, pi_hat, pi_star):
    duration = float(time[-1] - time[0])
    error = pi_hat - pi_star
    metrics = {
        "pi_vector_rmse": vector_rmse(error, time),
        "pi_error_norm_mean": time_mean(np.linalg.norm(error, axis=1), time),
        "pi_error_norm_integral": trapz(np.linalg.norm(error, axis=1), time),
        "pi_error_norm_peak": float(np.max(np.linalg.norm(error, axis=1))),
    }
    truth_norm_rms = vector_rmse(pi_star, time)
    estimate_norm_rms = vector_rmse(pi_hat, time)
    metrics["pi_norm_rms_truth"] = truth_norm_rms
    metrics["pi_norm_rms_estimate"] = estimate_norm_rms
    metrics["pi_norm_magnitude_ratio"] = (
        estimate_norm_rms / truth_norm_rms if truth_norm_rms > 1.0e-8 else None)

    for index, component in enumerate(COMPONENTS):
        truth = pi_star[:, index]
        estimate = pi_hat[:, index]
        difference = error[:, index]
        threshold = SIGN_THRESHOLDS[component]
        ratio_threshold = RATIO_DENOMINATOR_THRESHOLDS[component]
        truth_rms = scalar_rmse(truth, time)
        estimate_rms = scalar_rmse(estimate, time)
        metrics["pi_{}_rmse".format(component)] = scalar_rmse(difference, time)
        metrics["pi_{}_signed_bias".format(component)] = trapz(difference, time) / duration
        metrics["pi_{}_peak_abs_error".format(component)] = float(np.max(np.abs(difference)))
        metrics["pi_{}_truth_rms".format(component)] = truth_rms
        metrics["pi_{}_estimate_rms".format(component)] = estimate_rms
        metrics["pi_{}_magnitude_ratio".format(component)] = (
            estimate_rms / truth_rms if truth_rms >= ratio_threshold else None)
        truth_std = float(np.std(truth))
        estimate_std = float(np.std(estimate))
        if truth_std > 1.0e-12 and estimate_std > 1.0e-12:
            raw_correlation = float(np.corrcoef(truth, estimate)[0, 1])
            metrics["pi_{}_correlation_raw".format(component)] = (
                raw_correlation if math.isfinite(raw_correlation) else None)
        else:
            metrics["pi_{}_correlation_raw".format(component)] = None
        if truth_rms >= threshold and truth_std >= max(0.1 * threshold, 0.05 * truth_rms):
            correlation = float(np.corrcoef(truth, estimate)[0, 1])
            metrics["pi_{}_correlation".format(component)] = (
                correlation if math.isfinite(correlation) else None)
        else:
            metrics["pi_{}_correlation".format(component)] = None
        mask = np.abs(truth) >= threshold
        metrics["pi_{}_sign_sample_count".format(component)] = int(np.sum(mask))
        metrics["pi_{}_sign_agreement_ratio".format(component)] = (
            float(np.mean(np.sign(estimate[mask]) == np.sign(truth[mask])))
            if np.any(mask) else None)
        lag, lag_correlation = lag_estimate(truth, estimate, threshold)
        metrics["pi_{}_lag_s".format(component)] = lag
        metrics["pi_{}_lag_correlation".format(component)] = lag_correlation
    return metrics


def window_metrics(time, x_error, position_norm, orientation_norm, sigma,
                   pi_hat, pi_star, candidate, applied, policy, contact):
    duration = float(time[-1] - time[0])
    return {
        "duration_s": duration,
        "x_rmse_m": scalar_rmse(x_error, time),
        "x_iae_m_s": trapz(np.abs(x_error), time),
        "x_peak_abs_m": float(np.max(np.abs(x_error))),
        "position_rmse_m": scalar_rmse(position_norm, time),
        "position_peak_m": float(np.max(position_norm)),
        "orientation_rmse_rad": scalar_rmse(orientation_norm, time),
        "orientation_peak_rad": float(np.max(orientation_norm)),
        "sigma_peak_norm": float(np.max(np.linalg.norm(sigma, axis=1))),
        "sigma_mean_norm": time_mean(np.linalg.norm(sigma, axis=1), time),
        "sigma_integral_norm_s": trapz(np.linalg.norm(sigma, axis=1), time),
        "pi_hat_peak_norm": float(np.max(np.linalg.norm(pi_hat, axis=1))),
        "pi_star_peak_norm": float(np.max(np.linalg.norm(pi_star, axis=1))),
        "candidate_correction_rms_norm": vector_rmse(candidate, time),
        "candidate_correction_peak_norm": float(np.max(np.linalg.norm(candidate, axis=1))),
        "applied_correction_rms_norm": vector_rmse(applied, time),
        "applied_correction_peak_norm": float(np.max(np.linalg.norm(applied, axis=1))),
        "policy_input_rms_norm": vector_rmse(policy, time),
        "contact_force_rms_norm_N": vector_rmse(contact, time),
    }


def prefixed(prefix, values):
    return {prefix + "_" + key: value for key, value in values.items()}


def analyze_run(records, group, run_name, grid):
    directory = os.path.join(records, group, run_name)
    validation = load_json(os.path.join(directory, "task_c_run_summary.json"))
    config = load_json(os.path.join(directory, "task_c_effective_config.json"))
    controller = load_json(os.path.join(directory, "effective_config.json"))
    if not validation.get("passed") or validation.get("stage") not in ("C3", "C5"):
        raise RuntimeError("Task C per-run validation failed: {}".format(directory))
    expected_mode = "nominal" if group == "uncertain_off" else "full_adaptive"
    if config.get("group") != group or controller.get("mode") != expected_mode:
        raise RuntimeError("Task C effective group/controller mismatch: {}".format(directory))

    raw = load_csv(os.path.join(directory, "task_c_run.csv"))
    values = interpolate_csv(raw, grid)
    bag = load_observation_and_target(os.path.join(directory, "task_c_run.bag"))
    actual_state = interpolate_matrix(grid, bag["time"], bag["states"])
    policy = interpolate_matrix(grid, bag["time"], bag["policy"])
    desired_state = interpolate_matrix(grid, bag["target_times"], bag["target_states"])

    desired_position = desired_state[:, 6:9]
    actual_position = actual_state[:, 6:9]
    position_error = desired_position - actual_position
    position_norm = np.linalg.norm(position_error, axis=1)
    desired_rpy = desired_state[:, [11, 10, 9]]
    actual_rpy = actual_state[:, [11, 10, 9]]
    orientation_error = wrap_angle(desired_rpy - actual_rpy)
    orientation_norm = np.linalg.norm(orientation_error, axis=1)
    x_error = position_error[:, 0]

    pi_hat = vector(values, "pi_hat_")
    pi_star = vector(values, "pi_star_")
    sigma = vector(values, "sigma_")
    candidate = vector(values, "candidate_correction_")
    applied = vector(values, "applied_correction_")
    contact = policy[:, :CONTACT_FORCE_DIMENSION]
    if not all(np.all(np.isfinite(item)) for item in (
            actual_state, policy, desired_state, pi_hat, pi_star, sigma, candidate, applied)):
        raise RuntimeError("non-finite regridded data: {}".format(directory))

    masks = {
        "full": np.ones(len(grid), dtype=bool),
        "target_transition": grid <= TRANSITION_END + 1.0e-12,
        "post_transition": grid >= TRANSITION_END - 1.0e-12,
    }
    metrics = {}
    for name, mask in masks.items():
        metrics.update(prefixed(name, window_metrics(
            grid[mask], x_error[mask], position_norm[mask], orientation_norm[mask],
            sigma[mask], pi_hat[mask], pi_star[mask], candidate[mask], applied[mask],
            policy[mask], contact[mask])))
        for index, axis in enumerate(("x", "y", "z")):
            component_error = position_error[mask, index]
            metrics["{}_{}_rmse_m".format(name, axis)] = scalar_rmse(
                component_error, grid[mask])
            metrics["{}_{}_signed_bias_m".format(name, axis)] = time_mean(
                component_error, grid[mask])
            metrics["{}_{}_peak_abs_m".format(name, axis)] = float(
                np.max(np.abs(component_error)))
        for index, axis in enumerate(("roll", "pitch", "yaw")):
            component_error = orientation_error[mask, index]
            metrics["{}_{}_rmse_rad".format(name, axis)] = scalar_rmse(
                component_error, grid[mask])
            metrics["{}_{}_signed_bias_rad".format(name, axis)] = time_mean(
                component_error, grid[mask])
            metrics["{}_{}_peak_abs_rad".format(name, axis)] = float(
                np.max(np.abs(component_error)))
    metrics.update(parameter_metrics(grid, pi_hat, pi_star))
    metrics.update({
        "max_reconstruction_error": float(np.max(values["reconstruction_max_error"])),
        "full_flow_residual_rms": scalar_rmse(values["full_flow_residual_norm"], grid),
        "unmatched_flow_residual_rms": scalar_rmse(values["unmatched_flow_residual_norm"], grid),
        "max_abs_applied_minus_candidate": float(np.max(np.abs(applied - candidate))),
    })

    expected_scale = 1.0 if group == "nominal" else 1.10
    if abs(config["true_plant_mass_scale"] - expected_scale) > 1.0e-12:
        raise RuntimeError("mass scale changed: {}".format(directory))
    if metrics["max_reconstruction_error"] > 1.0e-10:
        raise RuntimeError("ground-truth reconstruction failed: {}".format(directory))
    if group != "uncertain_off" and metrics["max_abs_applied_minus_candidate"] > 1.0e-12:
        raise RuntimeError("Eq.(10) applied path mismatch: {}".format(directory))
    if group == "nominal" and metrics["full_pi_star_peak_norm"] > 1.0e-10:
        raise RuntimeError("nominal pi_star is nonzero: {}".format(directory))
    if group != "nominal" and metrics["full_pi_star_peak_norm"] < 1.0:
        raise RuntimeError("uncertain pi_star is not active: {}".format(directory))
    if group == "uncertain_off":
        if metrics["full_pi_hat_peak_norm"] > 1.0e-12 \
                or metrics["full_candidate_correction_peak_norm"] > 1.0e-12 \
                or metrics["full_applied_correction_peak_norm"] > 1.0e-12:
            raise RuntimeError("Adaptive-OFF estimate/correction is nonzero: {}".format(directory))

    signals = {
        "desired_x": desired_position[:, 0],
        "actual_x": actual_position[:, 0],
        "x_error": x_error,
        "position_error_norm": position_norm,
        "orientation_error_norm": orientation_norm,
        "sigma_norm": np.linalg.norm(sigma, axis=1),
        "pi_error_norm": np.linalg.norm(pi_hat - pi_star, axis=1),
        "candidate_correction_norm": np.linalg.norm(candidate, axis=1),
        "applied_correction_norm": np.linalg.norm(applied, axis=1),
    }
    for index, component in enumerate(COMPONENTS):
        signals["pi_hat_" + component] = pi_hat[:, index]
        signals["pi_star_" + component] = pi_star[:, index]
        signals["sigma_" + component] = sigma[:, index]
        signals["candidate_" + component] = candidate[:, index]
        signals["applied_" + component] = applied[:, index]
    for index, axis in enumerate(("x", "y", "z")):
        signals["desired_position_" + axis] = desired_position[:, index]
        signals["actual_position_" + axis] = actual_position[:, index]
    for index, axis in enumerate(("roll", "pitch", "yaw")):
        signals["desired_rpy_" + axis] = desired_rpy[:, index]
        signals["actual_rpy_" + axis] = actual_rpy[:, index]
        signals["orientation_error_" + axis] = orientation_error[:, index]

    return {
        "group": group,
        "run": run_name,
        "directory": directory,
        "metrics": metrics,
        "signals": signals,
        "policy": policy,
        "contact": contact,
    }


def numeric_group_statistics(runs):
    result = {}
    for name in sorted(runs[0]["metrics"]):
        values = [run["metrics"][name] for run in runs]
        numeric = [float(value) for value in values
                   if value is not None and math.isfinite(float(value))]
        if not numeric:
            result[name] = {"n": 0, "mean": None, "std": None,
                            "min": None, "max": None}
            continue
        array = np.asarray(numeric)
        result[name] = {
            "n": len(numeric),
            "mean": float(np.mean(array)),
            "std": float(np.std(array, ddof=1)) if len(array) > 1 else 0.0,
            "min": float(np.min(array)),
            "max": float(np.max(array)),
        }
    return result


def write_per_run_metrics(path, runs):
    names = sorted(runs[0]["metrics"])
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["group", "run"] + names)
        writer.writeheader()
        for run in runs:
            row = {"group": run["group"], "run": run["run"]}
            row.update(run["metrics"])
            writer.writerow(row)


def write_group_statistics(path, statistics):
    with open(path, "w", newline="") as stream:
        fields = ("group", "metric", "n", "mean", "std", "min", "max")
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for group in GROUPS:
            for metric, values in sorted(statistics[group].items()):
                row = {"group": group, "metric": metric}
                row.update(values)
                writer.writerow(row)


def aggregate_rows(rows, grouping, values):
    grouped = {}
    for row in rows:
        key = tuple(row[name] for name in grouping)
        grouped.setdefault(key, {name: [] for name in values})
        for name in values:
            value = row[name]
            if value is not None and math.isfinite(float(value)):
                grouped[key][name].append(float(value))
    output = []
    for key, columns in sorted(grouped.items()):
        item = dict(zip(grouping, key))
        for name, samples in columns.items():
            array = np.asarray(samples)
            item[name] = {
                "n": len(samples),
                "mean": float(np.mean(array)) if len(array) else None,
                "std": float(np.std(array, ddof=1)) if len(array) > 1 else 0.0 if len(array) else None,
                "min": float(np.min(array)) if len(array) else None,
                "max": float(np.max(array)) if len(array) else None,
            }
        output.append(item)
    return output


def paired_differences(runs_by_key, grid):
    tracking_metrics = (
        "full_x_rmse_m", "full_x_iae_m_s", "full_x_peak_abs_m",
        "full_position_rmse_m", "full_position_peak_m",
        "full_orientation_rmse_rad", "full_orientation_peak_rad",
        "full_y_rmse_m", "full_y_peak_abs_m", "full_z_rmse_m", "full_z_peak_abs_m",
        "full_roll_rmse_rad", "full_pitch_rmse_rad", "full_yaw_rmse_rad",
        "full_sigma_peak_norm", "full_sigma_mean_norm", "full_sigma_integral_norm_s",
        "target_transition_x_rmse_m", "target_transition_position_rmse_m",
        "post_transition_x_rmse_m", "post_transition_position_rmse_m",
    )
    tracking_rows = []
    policy_rows = []
    force_delta_signals = {name: [] for name, _, _ in PAIR_SPECS}
    windows = {
        "full": np.ones(len(grid), dtype=bool),
        "target_transition": grid <= TRANSITION_END + 1.0e-12,
        "post_transition": grid >= TRANSITION_END - 1.0e-12,
    }
    for run_name in RUNS:
        for pair_name, baseline_group, comparison_group in PAIR_SPECS:
            baseline_run = runs_by_key[(baseline_group, run_name)]
            comparison_run = runs_by_key[(comparison_group, run_name)]
            for metric in tracking_metrics:
                baseline = baseline_run["metrics"][metric]
                value = comparison_run["metrics"][metric]
                tracking_rows.append({
                    "run": run_name,
                    "pair": pair_name,
                    "baseline_group": baseline_group,
                    "comparison_group": comparison_group,
                    "metric": metric,
                    "difference": value - baseline,
                    "relative_change_percent": (
                        100.0 * (value - baseline) / baseline if baseline != 0.0 else None),
                })
            full_force_delta = comparison_run["contact"] - baseline_run["contact"]
            force_delta_signals[pair_name].append(np.linalg.norm(full_force_delta, axis=1))
            for window, mask in windows.items():
                time = grid[mask]
                policy_delta = comparison_run["policy"][mask] - baseline_run["policy"][mask]
                force_delta = full_force_delta[mask]
                row = {
                    "run": run_name,
                    "pair": pair_name,
                    "baseline_group": baseline_group,
                    "comparison_group": comparison_group,
                    "window": window,
                    "policy_rms_vector_difference": vector_rmse(policy_delta, time),
                    "contact_force_rms_vector_difference_N": vector_rmse(force_delta, time),
                }
                for leg in range(4):
                    leg_delta = force_delta[:, 3 * leg:3 * leg + 3]
                    row["leg_{}_force_rms_difference_N".format(leg)] = vector_rmse(leg_delta, time)
                    row["leg_{}_mean_Fz_difference_N".format(leg)] = time_mean(leg_delta[:, 2], time)
                policy_rows.append(row)
    return tracking_rows, policy_rows, {
        name: np.vstack(values) for name, values in force_delta_signals.items()}


def write_rows(path, rows):
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def trajectory_statistics(runs, signal):
    matrix = np.vstack([run["signals"][signal] for run in runs])
    return np.mean(matrix, axis=0), np.std(matrix, axis=0, ddof=1)


def decorate(axis, ylabel):
    axis.axvspan(WINDOW_START, 0.70, color="#2ca02c", alpha=0.08)
    axis.axvline(TRANSITION_END, color="#777777", linewidth=0.8, linestyle=":")
    axis.set_xlim(WINDOW_START, WINDOW_END)
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.3)


def plot_group_signal(path, grid, runs_by_group, signal, ylabel):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(11, 4.8))
    for group in GROUPS:
        mean, std = trajectory_statistics(runs_by_group[group], signal)
        color = GROUP_COLORS[group]
        axis.plot(grid, mean, color=color, linewidth=2.0, label=GROUP_LABELS[group])
        axis.fill_between(grid, mean - std, mean + std, color=color, alpha=0.14)
    decorate(axis, ylabel)
    axis.set_xlabel("MPC / simulation time (s)")
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def plot_desired_actual_x(path, grid, runs_by_group):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(11, 4.8))
    desired, _ = trajectory_statistics(runs_by_group["nominal"], "desired_x")
    axis.plot(grid, desired, color="black", linewidth=1.8, linestyle="--", label="Desired x")
    for group in GROUPS:
        mean, std = trajectory_statistics(runs_by_group[group], "actual_x")
        color = GROUP_COLORS[group]
        axis.plot(grid, mean, color=color, linewidth=2.0, label=GROUP_LABELS[group])
        axis.fill_between(grid, mean - std, mean + std, color=color, alpha=0.14)
    decorate(axis, "x position (m)")
    axis.set_xlabel("MPC / simulation time (s)")
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def plot_estimate_component(path, grid, runs_by_group, component):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(len(GROUPS), 1, figsize=(11, 3.6 * len(GROUPS)),
                               sharex=True, sharey=True)
    for axis, group in zip(axes, GROUPS):
        hat_mean, hat_std = trajectory_statistics(runs_by_group[group], "pi_hat_" + component)
        star_mean, star_std = trajectory_statistics(runs_by_group[group], "pi_star_" + component)
        color = GROUP_COLORS[group]
        axis.plot(grid, hat_mean, color=color, linewidth=2.0, label="pi_hat " + component)
        axis.fill_between(grid, hat_mean - hat_std, hat_mean + hat_std, color=color, alpha=0.14)
        axis.plot(grid, star_mean, color="black", linewidth=1.8, linestyle="--",
                  label="pi_star " + component)
        axis.fill_between(grid, star_mean - star_std, star_mean + star_std,
                          color="black", alpha=0.07)
        decorate(axis, "{} ({})".format(component, "N" if component[0] == "F" else "N m"))
        axis.set_title(GROUP_LABELS[group])
        axis.legend(loc="best")
    axes[-1].set_xlabel("MPC / simulation time (s)")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def plot_torque_components(path, grid, runs_by_group):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(3, len(GROUPS), figsize=(6 * len(GROUPS), 11),
                               sharex=True, sharey="row")
    for row_index, component in enumerate(TORQUE_COMPONENTS):
        for column_index, group in enumerate(GROUPS):
            axis = axes[row_index, column_index]
            hat_mean, hat_std = trajectory_statistics(runs_by_group[group], "pi_hat_" + component)
            star_mean, star_std = trajectory_statistics(runs_by_group[group], "pi_star_" + component)
            color = GROUP_COLORS[group]
            axis.plot(grid, hat_mean, color=color, linewidth=1.8, label="pi_hat")
            axis.fill_between(grid, hat_mean - hat_std, hat_mean + hat_std, color=color, alpha=0.14)
            axis.plot(grid, star_mean, color="black", linewidth=1.5, linestyle="--", label="pi_star")
            axis.fill_between(grid, star_mean - star_std, star_mean + star_std,
                              color="black", alpha=0.07)
            decorate(axis, component + " (N m)")
            if row_index == 0:
                axis.set_title(GROUP_LABELS[group])
            if row_index == 2:
                axis.set_xlabel("MPC / simulation time (s)")
            if row_index == 0 and column_index == 0:
                axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def plot_equation10(path, grid, runs_by_group):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    for group in GROUPS:
        color = GROUP_COLORS[group]
        for axis, signal in zip(axes, ("candidate_correction_norm", "applied_correction_norm")):
            mean, std = trajectory_statistics(runs_by_group[group], signal)
            axis.plot(grid, mean, color=color, linewidth=2.0, label=GROUP_LABELS[group])
            axis.fill_between(grid, mean - std, mean + std, color=color, alpha=0.14)
    decorate(axes[0], "Candidate correction norm")
    decorate(axes[1], "Applied correction norm")
    axes[0].legend(loc="best")
    axes[1].legend(loc="best")
    axes[1].set_xlabel("MPC / simulation time (s)")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def plot_contact_difference(path, grid, force_delta_norms_by_pair, runs_by_key):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    pair_name = "uncertain_adaptive_minus_uncertain_off"
    force_delta_norms = force_delta_norms_by_pair[pair_name]
    mean = np.mean(force_delta_norms, axis=0)
    std = np.std(force_delta_norms, axis=0, ddof=1)
    axes[0].plot(grid, mean, color="#9467bd", linewidth=2.0,
                 label="||contact force U-ON - U-OFF||")
    axes[0].fill_between(grid, mean - std, mean + std, color="#9467bd", alpha=0.16)
    for leg in range(4):
        differences = []
        for run_name in RUNS:
            adaptive_off = runs_by_key[("uncertain_off", run_name)]["contact"][:, 3 * leg + 2]
            adaptive_on = runs_by_key[("uncertain_adaptive", run_name)]["contact"][:, 3 * leg + 2]
            differences.append(adaptive_on - adaptive_off)
        matrix = np.vstack(differences)
        leg_mean = np.mean(matrix, axis=0)
        axes[1].plot(grid, leg_mean, linewidth=1.6, label="leg {} delta Fz".format(leg))
    decorate(axes[0], "Force difference norm (N)")
    decorate(axes[1], "Uncertain ON - OFF Fz (N)")
    axes[0].legend(loc="best")
    axes[1].legend(loc="best", ncol=2)
    axes[1].set_xlabel("MPC / simulation time (s)")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def write_group_trajectories(path, grid, runs_by_group, force_delta_norms_by_pair):
    signals = (
        "desired_x", "actual_x", "x_error", "position_error_norm",
        "orientation_error_norm", "sigma_norm", "pi_error_norm",
        "candidate_correction_norm", "applied_correction_norm",
    ) + tuple("pi_hat_" + value for value in COMPONENTS) \
      + tuple("pi_star_" + value for value in COMPONENTS) \
      + tuple("desired_position_" + value for value in ("x", "y", "z")) \
      + tuple("actual_position_" + value for value in ("x", "y", "z")) \
      + tuple("orientation_error_" + value for value in ("roll", "pitch", "yaw"))
    columns = {"time_s": grid}
    for group in GROUPS:
        for signal in signals:
            mean, std = trajectory_statistics(runs_by_group[group], signal)
            columns["{}_{}_mean".format(group, signal)] = mean
            columns["{}_{}_std".format(group, signal)] = std
    for pair_name, values in force_delta_norms_by_pair.items():
        columns[pair_name + "_contact_force_difference_norm_mean"] = np.mean(values, axis=0)
        columns[pair_name + "_contact_force_difference_norm_std"] = np.std(values, axis=0, ddof=1)
    names = list(columns)
    with open(path, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        for index in range(len(grid)):
            writer.writerow([columns[name][index] for name in names])


def main():
    parser = argparse.ArgumentParser(description="Analyze Task C three-group Adaptive-OFF comparison")
    parser.add_argument("records_directory")
    parser.add_argument("--output-directory", default=None)
    args = parser.parse_args()
    records = os.path.abspath(args.records_directory)
    output = os.path.abspath(args.output_directory or os.path.join(records, "analysis_adaptive_off"))
    os.makedirs(output, exist_ok=True)

    fairness = load_json(os.path.join(records, "stage_c5_fairness.json"))
    manifest = load_json(os.path.join(records, "stage_c5_manifest.json"))
    if not fairness.get("passed") or not manifest.get("passed"):
        raise RuntimeError("Stage C5 fairness/manifest has not passed")

    grid = np.linspace(WINDOW_START, WINDOW_END,
                       int(round((WINDOW_END - WINDOW_START) / GRID_DT)) + 1)
    runs = []
    runs_by_group = {group: [] for group in GROUPS}
    runs_by_key = {}
    for group in GROUPS:
        for run_name in RUNS:
            run = analyze_run(records, group, run_name, grid)
            runs.append(run)
            runs_by_group[group].append(run)
            runs_by_key[(group, run_name)] = run

    statistics = {group: numeric_group_statistics(runs_by_group[group]) for group in GROUPS}
    tracking_rows, policy_rows, force_delta_norms = paired_differences(runs_by_key, grid)

    per_run_path = os.path.join(output, "metrics_per_run.csv")
    group_path = os.path.join(output, "metrics_group_mean_std.csv")
    tracking_path = os.path.join(output, "tracking_pairwise_differences.csv")
    policy_path = os.path.join(output, "policy_pairwise_differences.csv")
    trajectories_path = os.path.join(output, "group_trajectories_mean_std.csv")
    write_per_run_metrics(per_run_path, runs)
    write_group_statistics(group_path, statistics)
    write_rows(tracking_path, tracking_rows)
    write_rows(policy_path, policy_rows)
    write_group_trajectories(trajectories_path, grid, runs_by_group, force_delta_norms)

    figures = {
        "C1": os.path.join(output, "figure_C1_desired_vs_actual_x.png"),
        "C2": os.path.join(output, "figure_C2_x_tracking_error.png"),
        "C3": os.path.join(output, "figure_C3_pi_hat_vs_pi_star_Fx.png"),
        "C4": os.path.join(output, "figure_C4_pi_hat_vs_pi_star_Fy.png"),
        "C5": os.path.join(output, "figure_C5_pi_hat_vs_pi_star_Fz.png"),
        "C6": os.path.join(output, "figure_C6_pi_hat_vs_pi_star_torques.png"),
        "C7": os.path.join(output, "figure_C7_pi_estimation_error_norm.png"),
        "C8": os.path.join(output, "figure_C8_sigma_norm.png"),
        "C9": os.path.join(output, "figure_C9_equation10_correction.png"),
        "C10": os.path.join(output, "figure_C10_contact_force_policy_difference.png"),
    }
    plot_desired_actual_x(figures["C1"], grid, runs_by_group)
    plot_group_signal(figures["C2"], grid, runs_by_group, "x_error", "x tracking error (m)")
    plot_estimate_component(figures["C3"], grid, runs_by_group, "Fx")
    plot_estimate_component(figures["C4"], grid, runs_by_group, "Fy")
    plot_estimate_component(figures["C5"], grid, runs_by_group, "Fz")
    plot_torque_components(figures["C6"], grid, runs_by_group)
    plot_group_signal(figures["C7"], grid, runs_by_group, "pi_error_norm", "||pi_hat - pi_star||")
    plot_group_signal(figures["C8"], grid, runs_by_group, "sigma_norm", "||sigma||")
    plot_equation10(figures["C9"], grid, runs_by_group)
    plot_contact_difference(figures["C10"], grid, force_delta_norms, runs_by_key)

    selected_metrics = (
        "full_x_rmse_m", "full_x_iae_m_s", "full_x_peak_abs_m",
        "full_position_rmse_m", "full_position_peak_m",
        "full_orientation_rmse_rad", "full_orientation_peak_rad",
        "full_y_rmse_m", "full_y_signed_bias_m", "full_y_peak_abs_m",
        "full_z_rmse_m", "full_z_signed_bias_m", "full_z_peak_abs_m",
        "full_roll_rmse_rad", "full_pitch_rmse_rad", "full_yaw_rmse_rad",
        "full_sigma_peak_norm", "full_sigma_mean_norm", "full_sigma_integral_norm_s",
        "full_candidate_correction_rms_norm", "full_candidate_correction_peak_norm",
        "full_applied_correction_rms_norm", "full_applied_correction_peak_norm",
        "full_policy_input_rms_norm", "full_contact_force_rms_norm_N",
        "full_flow_residual_rms", "unmatched_flow_residual_rms",
        "max_reconstruction_error",
        "pi_vector_rmse", "pi_error_norm_mean", "pi_error_norm_peak",
        "pi_norm_rms_truth", "pi_norm_rms_estimate", "pi_norm_magnitude_ratio",
    ) + tuple("pi_{}_rmse".format(value) for value in COMPONENTS) \
      + tuple("pi_{}_signed_bias".format(value) for value in COMPONENTS) \
      + tuple("pi_{}_peak_abs_error".format(value) for value in COMPONENTS) \
      + tuple("pi_{}_correlation_raw".format(value) for value in COMPONENTS) \
      + tuple("pi_{}_correlation".format(value) for value in COMPONENTS) \
      + tuple("pi_{}_sign_agreement_ratio".format(value) for value in COMPONENTS) \
      + tuple("pi_{}_magnitude_ratio".format(value) for value in COMPONENTS) \
      + tuple("pi_{}_lag_s".format(value) for value in COMPONENTS)
    selected_statistics = {
        group: {name: statistics[group][name] for name in selected_metrics}
        for group in GROUPS
    }

    lower_is_better = (
        "full_x_rmse_m", "full_x_iae_m_s", "full_x_peak_abs_m",
        "full_position_rmse_m", "full_position_peak_m",
        "full_orientation_rmse_rad", "full_orientation_peak_rad",
        "full_y_rmse_m", "full_y_peak_abs_m",
        "full_z_rmse_m", "full_z_peak_abs_m",
        "full_roll_rmse_rad", "full_pitch_rmse_rad", "full_yaw_rmse_rad",
        "full_sigma_peak_norm", "full_sigma_mean_norm", "full_sigma_integral_norm_s",
    )
    relative_changes = {}
    for pair_name, baseline_group, comparison_group in PAIR_SPECS:
        relative_changes[pair_name] = {}
        for metric in lower_is_better:
            baseline = statistics[baseline_group][metric]["mean"]
            comparison = statistics[comparison_group][metric]["mean"]
            relative_changes[pair_name][metric] = (
                100.0 * (comparison - baseline) / baseline if baseline else None)

    degradation_recovery = {}
    for metric in lower_is_better:
        nominal = statistics["nominal"][metric]["mean"]
        adaptive_off = statistics["uncertain_off"][metric]["mean"]
        adaptive_on = statistics["uncertain_adaptive"][metric]["mean"]
        degradation = adaptive_off - nominal
        recovery_eligible = degradation > max(1.0e-12, 0.01 * abs(nominal))
        degradation_recovery[metric] = {
            "nominal_group_mean": nominal,
            "uncertain_off_group_mean": adaptive_off,
            "uncertain_adaptive_group_mean": adaptive_on,
            "off_minus_nominal_degradation": degradation,
            "on_minus_nominal_residual_degradation": adaptive_on - nominal,
            "recovery_fraction_eligible": recovery_eligible,
            "recovered_fraction_percent": (
                100.0 * (adaptive_off - adaptive_on) / degradation
                if recovery_eligible else None),
        }

    policy_value_fields = (
        "policy_rms_vector_difference", "contact_force_rms_vector_difference_N",
    ) + tuple("leg_{}_force_rms_difference_N".format(value) for value in range(4)) \
      + tuple("leg_{}_mean_Fz_difference_N".format(value) for value in range(4))

    summary = {
        "stage": "C5",
        "passed": True,
        "performance_analysis_performed": True,
        "input": {
            "stage_c5_fairness_passed": True,
            "run_count": len(runs),
            "groups": list(GROUPS),
            "runs_per_group": 3,
            "all_runs_cover_common_window": True,
            "common_grid_sample_count": len(grid),
        },
        "definitions": {
            "analysis_window_s": [WINDOW_START, WINDOW_END],
            "target_transition_descriptive_window_s": [WINDOW_START, TRANSITION_END],
            "post_transition_descriptive_window_s": [TRANSITION_END, WINDOW_END],
            "common_grid_dt_s": GRID_DT,
            "integration": "trapezoidal integration on common 20 ms MPC-time grid",
            "group_std": "sample standard deviation, ddof=1, n=3",
            "parameter_vector_rmse": "sqrt(time mean of ||pi_hat-pi_star||^2)",
            "sign_agreement_thresholds": SIGN_THRESHOLDS,
            "magnitude_ratio_denominator_thresholds": RATIO_DENOMINATOR_THRESHOLDS,
            "correlation_raw": "Pearson correlation when both sampled signals have nonzero numerical variance",
            "correlation_qualified": "reported only when pi_star RMS and temporal variation exceed physical thresholds",
            "lag": "cross-correlation within +/-2 s; positive means pi_hat lags pi_star; null when truth variation is not clear",
            "recovery_time": "not applicable because the mass mismatch is active for the entire run",
            "pairing": "same run index for all three declared group pairs on the common grid",
        },
        "selected_group_mean_std": selected_statistics,
        "group_mean_relative_change_percent": relative_changes,
        "adaptive_recovery_of_off_vs_nominal_degradation": degradation_recovery,
        "tracking_paired_difference_mean_std": aggregate_rows(
            tracking_rows, ("pair", "metric"),
            ("difference", "relative_change_percent")),
        "policy_pairwise_mean_std": aggregate_rows(
            policy_rows, ("pair", "window"), policy_value_fields),
        "scientific_scope": (
            "Observed results for the frozen animal_dummy, +10% mass mismatch, fixed x target, "
            "and fixed controller settings. Uncertain Adaptive-ON versus Adaptive-OFF isolates the "
            "two adaptive mode bits, but does not prove parameter convergence or the paper's general claims."
        ),
        "artifacts": {},
    }
    output_files = (per_run_path, group_path, tracking_path, policy_path,
                    trajectories_path, *figures.values())
    for path in output_files:
        summary["artifacts"][os.path.basename(path)] = {
            "sha256": sha256(path), "size_bytes": os.path.getsize(path)}
    summary_path = os.path.join(output, "stage_c5_adaptive_off_summary.json")
    with open(summary_path, "w") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    manifest_path = os.path.join(output, "stage_c5_adaptive_off_manifest.json")
    analysis_source = os.path.abspath(__file__)
    with open(manifest_path, "w") as stream:
        json.dump({
            "stage": "C5", "passed": True,
            "input_stage_c5_manifest_sha256": sha256(os.path.join(records, "stage_c5_manifest.json")),
            "analysis_source": analysis_source,
            "analysis_source_sha256": sha256(analysis_source),
            "summary": os.path.basename(summary_path),
            "summary_sha256": sha256(summary_path),
            "formal_interpretation_status": "adaptive_on_off_comparison_complete",
        }, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
