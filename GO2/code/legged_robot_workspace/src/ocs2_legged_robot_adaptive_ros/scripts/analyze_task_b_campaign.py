#!/usr/bin/env python3
"""Stage B4 analysis for the fixed nine-run Task B campaign."""

import argparse
import csv
import hashlib
import json
import math
import os

import numpy as np


MODES = ("nominal", "estimate_only", "full_adaptive")
RUNS = ("run_01", "run_02", "run_03")
COMPONENTS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
WINDOW_START = 0.5
WINDOW_END = 15.5
DISTURBANCE_START = 1.0
DISTURBANCE_END = 4.0
POST_END = WINDOW_END
GRID_DT = 0.02
RECOVERY_BASELINE_START = 0.8
RECOVERY_BASELINE_END = 1.0
RECOVERY_MARGIN_M = 0.001
RECOVERY_DWELL_S = 0.5
CONTACT_FORCE_DIMENSION = 12

MODE_LABELS = {
    "nominal": "Nominal",
    "estimate_only": "Estimate-only",
    "full_adaptive": "Full adaptive",
}
MODE_COLORS = {
    "nominal": "#4c4c4c",
    "estimate_only": "#1f77b4",
    "full_adaptive": "#d62728",
}


def load_json(path):
    with open(path) as stream:
        return json.load(stream)


def load_csv(path):
    with open(path, newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    if not rows:
        raise RuntimeError("empty run CSV: {}".format(path))
    data = {name: np.asarray([float(row[name]) for row in rows], dtype=float)
            for name in reader.fieldnames}
    if any(not np.all(np.isfinite(values)) for values in data.values()):
        raise RuntimeError("non-finite value in {}".format(path))
    time = data["observation_time"]
    if not np.all(np.isfinite(time)) or not np.all(np.diff(time) > 0.0):
        raise RuntimeError("non-finite or non-increasing time in {}".format(path))
    if time[0] > WINDOW_START or time[-1] < WINDOW_END:
        raise RuntimeError("run does not cover fixed analysis window: {}".format(path))
    return data


def interpolate_columns(raw, grid):
    time = raw["observation_time"]
    return {name: np.interp(grid, time, values) for name, values in raw.items()}


def vector(interpolated, prefix, components=COMPONENTS):
    return np.column_stack([interpolated[prefix + component] for component in components])


def trapz(values, time):
    return float(np.trapz(values, time))


def time_metrics(time, x_error, position_norm, orientation_norm, sigma_norm,
                 pi_norm, candidate_norm, applied_norm, policy, contact_forces):
    duration = float(time[-1] - time[0])
    if duration <= 0.0:
        raise RuntimeError("metric window duration must be positive")
    return {
        "duration_s": duration,
        "x_rmse_m": math.sqrt(trapz(x_error ** 2, time) / duration),
        "x_iae_m_s": trapz(np.abs(x_error), time),
        "x_peak_abs_m": float(np.max(np.abs(x_error))),
        "position_rmse_m": math.sqrt(trapz(position_norm ** 2, time) / duration),
        "position_peak_m": float(np.max(position_norm)),
        "orientation_rmse_rad": math.sqrt(trapz(orientation_norm ** 2, time) / duration),
        "orientation_peak_rad": float(np.max(orientation_norm)),
        "sigma_peak_norm": float(np.max(sigma_norm)),
        "sigma_mean_norm": trapz(sigma_norm, time) / duration,
        "sigma_integral_norm_s": trapz(sigma_norm, time),
        "pi_hat_peak_norm": float(np.max(pi_norm)),
        "candidate_correction_peak_norm": float(np.max(candidate_norm)),
        "applied_correction_peak_norm": float(np.max(applied_norm)),
        "policy_input_rms_norm": math.sqrt(trapz(np.sum(policy ** 2, axis=1), time) / duration),
        "contact_force_rms_norm_N": math.sqrt(
            trapz(np.sum(contact_forces ** 2, axis=1), time) / duration
        ),
    }


def flatten_window_metrics(prefix, metrics):
    return {prefix + "_" + key: value for key, value in metrics.items()}


def recovery_metrics(grid, x_error):
    baseline_mask = (
        (grid >= RECOVERY_BASELINE_START - 1e-12)
        & (grid <= RECOVERY_BASELINE_END + 1e-12)
    )
    post_mask = (grid >= DISTURBANCE_END - 1e-12) & (grid <= POST_END + 1e-12)
    baseline = np.abs(x_error[baseline_mask])
    post_time = grid[post_mask]
    post_abs = np.abs(x_error[post_mask])
    threshold = float(np.percentile(baseline, 95.0) + RECOVERY_MARGIN_M)
    dwell_samples = int(round(RECOVERY_DWELL_S / GRID_DT)) + 1

    dwell_time = None
    for index in range(0, len(post_time) - dwell_samples + 1):
        if np.all(post_abs[index:index + dwell_samples] <= threshold):
            dwell_time = float(post_time[index] - DISTURBANCE_END)
            break

    sustained_time = None
    for index in range(len(post_time)):
        if np.all(post_abs[index:] <= threshold):
            sustained_time = float(post_time[index] - DISTURBANCE_END)
            break

    return {
        "recovery_baseline_window_start_s": RECOVERY_BASELINE_START,
        "recovery_baseline_window_end_s": RECOVERY_BASELINE_END,
        "recovery_baseline_abs_x_p95_m": float(np.percentile(baseline, 95.0)),
        "recovery_threshold_m": threshold,
        "recovery_margin_m": RECOVERY_MARGIN_M,
        "recovery_dwell_requirement_s": RECOVERY_DWELL_S,
        "recovery_time_0p5s_dwell_s": dwell_time,
        "recovery_time_sustained_s": sustained_time,
        "recovery_0p5s_dwell_censored": dwell_time is None,
        "recovery_sustained_censored": sustained_time is None,
        "recovery_excess_x_iae_m_s": trapz(np.maximum(post_abs - threshold, 0.0), post_time),
    }


def analyze_run(records, mode, run_id, grid):
    directory = os.path.join(records, mode, run_id)
    validation = load_json(os.path.join(directory, "task_b_smoke_summary.json"))
    config = load_json(os.path.join(directory, "effective_config.json"))
    if not validation.get("passed"):
        raise RuntimeError("single-run validation failed: {}".format(directory))
    raw = load_csv(os.path.join(directory, "task_b_smoke.csv"))
    values = interpolate_columns(raw, grid)

    x_error = values["x_error"]
    position_error = np.column_stack([
        values["desired_position_" + axis] - values["actual_position_" + axis]
        for axis in ("x", "y", "z")
    ])
    position_norm = np.linalg.norm(position_error, axis=1)
    orientation_error = np.column_stack([
        values["orientation_error_" + axis] for axis in ("roll", "pitch", "yaw")
    ])
    orientation_norm = np.linalg.norm(orientation_error, axis=1)
    sigma = vector(values, "sigma_")
    pi_hat = vector(values, "pi_hat_")
    candidate = vector(values, "candidate_correction_")
    applied = vector(values, "applied_correction_")
    yu_pi = vector(values, "yu_pi_")
    policy_names = sorted(
        (name for name in values if name.startswith("policy_input_")),
        key=lambda name: int(name.rsplit("_", 1)[1]),
    )
    policy = np.column_stack([values[name] for name in policy_names])
    if policy.shape[1] != 24:
        raise RuntimeError("expected 24-dimensional OCS2 policy input in {}".format(directory))
    contact_forces = policy[:, :CONTACT_FORCE_DIMENSION]

    signals = {
        "disturbance_Fx": np.where(
            (grid > DISTURBANCE_START) & (grid <= DISTURBANCE_END), 20.0, 0.0
        ),
        "pi_hat_Fx": pi_hat[:, 0],
        "sigma_Fx": sigma[:, 0],
        "x_error": x_error,
        "position_error_norm": position_norm,
        "yu_pi_Fx": yu_pi[:, 0],
        "candidate_correction_Fx": candidate[:, 0],
        "applied_correction_Fx": applied[:, 0],
    }

    masks = {
        "full": (grid >= WINDOW_START - 1e-12) & (grid <= WINDOW_END + 1e-12),
        "disturbance_on": (
            (grid >= DISTURBANCE_START - 1e-12) & (grid <= DISTURBANCE_END + 1e-12)
        ),
        "post_disturbance": (
            (grid >= DISTURBANCE_END - 1e-12) & (grid <= POST_END + 1e-12)
        ),
    }
    metrics = {}
    for prefix, mask in masks.items():
        metrics.update(flatten_window_metrics(prefix, time_metrics(
            grid[mask], x_error[mask], position_norm[mask], orientation_norm[mask],
            np.linalg.norm(sigma[mask], axis=1), np.linalg.norm(pi_hat[mask], axis=1),
            np.linalg.norm(candidate[mask], axis=1), np.linalg.norm(applied[mask], axis=1),
            policy[mask], contact_forces[mask],
        )))
    metrics.update(recovery_metrics(grid, x_error))
    metrics.update({
        "max_abs_yu_pi_minus_pi_hat": float(np.max(np.abs(yu_pi - pi_hat))),
        "max_abs_applied_minus_candidate": float(np.max(np.abs(applied - candidate))),
        "max_abs_applied_correction": float(np.max(np.abs(applied))),
    })

    expected_flags = {
        "nominal": (False, False),
        "estimate_only": (True, False),
        "full_adaptive": (True, True),
    }[mode]
    flags = (config["enableParameterUpdate"], config["enableAdaptiveDynamics"])
    if flags != expected_flags:
        raise RuntimeError("effective flags changed after B3 validation: {}".format(directory))
    if metrics["max_abs_yu_pi_minus_pi_hat"] > 1e-12:
        raise RuntimeError("Yu*pi_hat mismatch in {}".format(directory))
    if mode == "nominal" and (
        metrics["full_pi_hat_peak_norm"] > 1e-12
        or metrics["full_candidate_correction_peak_norm"] > 1e-12
        or metrics["full_applied_correction_peak_norm"] > 1e-12
    ):
        raise RuntimeError("nominal adaptive path is nonzero in {}".format(directory))
    if mode == "estimate_only" and (
        metrics["full_pi_hat_peak_norm"] <= 0.0
        or metrics["full_candidate_correction_peak_norm"] <= 0.0
        or metrics["full_applied_correction_peak_norm"] > 1e-12
    ):
        raise RuntimeError("estimate-only path check failed in {}".format(directory))
    if mode == "full_adaptive" and (
        metrics["full_pi_hat_peak_norm"] <= 0.0
        or metrics["full_candidate_correction_peak_norm"] <= 0.0
        or metrics["max_abs_applied_minus_candidate"] > 1e-12
    ):
        raise RuntimeError("full-adaptive Eq.(10) path check failed in {}".format(directory))
    return {
        "mode": mode,
        "run": run_id,
        "directory": directory,
        "metrics": metrics,
        "signals": signals,
        "policy": policy,
        "contact_forces": contact_forces,
    }


def numeric_group_statistics(runs):
    metric_names = sorted(runs[0]["metrics"])
    result = {}
    for name in metric_names:
        values = [run["metrics"][name] for run in runs]
        finite_values = [float(value) for value in values
                         if value is not None and not isinstance(value, bool) and math.isfinite(float(value))]
        if not finite_values:
            continue
        array = np.asarray(finite_values)
        result[name] = {
            "n": len(finite_values),
            "mean": float(np.mean(array)),
            "std": float(np.std(array, ddof=1)) if len(array) > 1 else 0.0,
            "min": float(np.min(array)),
            "max": float(np.max(array)),
        }
    for time_key, censor_key in (
        ("recovery_time_0p5s_dwell_s", "recovery_0p5s_dwell_censored"),
        ("recovery_time_sustained_s", "recovery_sustained_censored"),
    ):
        if time_key not in result:
            result[time_key] = {"n": 0, "mean": None, "std": None, "min": None, "max": None}
        result[time_key]["censored_count"] = sum(run["metrics"][censor_key] for run in runs)
    return result


def write_per_run_metrics(path, runs):
    metric_names = sorted(runs[0]["metrics"])
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["mode", "run"] + metric_names)
        writer.writeheader()
        for run in runs:
            row = {"mode": run["mode"], "run": run["run"]}
            row.update(run["metrics"])
            writer.writerow(row)


def write_group_statistics(path, group_statistics):
    with open(path, "w", newline="") as stream:
        fields = ["mode", "metric", "n", "mean", "std", "min", "max", "censored_count"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for mode in MODES:
            for metric, stats in sorted(group_statistics[mode].items()):
                row = {"mode": mode, "metric": metric, "censored_count": stats.get("censored_count", 0)}
                row.update({key: stats[key] for key in ("n", "mean", "std", "min", "max")})
                writer.writerow(row)


def policy_pairwise(runs_by_key, grid):
    windows = {
        "full": (grid >= WINDOW_START - 1e-12) & (grid <= WINDOW_END + 1e-12),
        "disturbance_on": (
            (grid >= DISTURBANCE_START - 1e-12) & (grid <= DISTURBANCE_END + 1e-12)
        ),
        "post_disturbance": (
            (grid >= DISTURBANCE_END - 1e-12) & (grid <= POST_END + 1e-12)
        ),
    }
    pairs = (
        ("estimate_only", "nominal"),
        ("full_adaptive", "nominal"),
        ("full_adaptive", "estimate_only"),
    )
    rows = []
    for left, right in pairs:
        for run_id in RUNS:
            a = runs_by_key[(left, run_id)]
            b = runs_by_key[(right, run_id)]
            for window, mask in windows.items():
                time = grid[mask]
                duration = float(time[-1] - time[0])
                policy_delta = a["policy"][mask] - b["policy"][mask]
                force_delta = a["contact_forces"][mask] - b["contact_forces"][mask]
                rows.append({
                    "left_mode": left,
                    "right_mode": right,
                    "run": run_id,
                    "window": window,
                    "policy_rms_vector_difference": math.sqrt(
                        trapz(np.sum(policy_delta ** 2, axis=1), time) / duration
                    ),
                    "contact_force_rms_vector_difference_N": math.sqrt(
                        trapz(np.sum(force_delta ** 2, axis=1), time) / duration
                    ),
                })
    return rows


def write_rows(path, rows):
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def paired_tracking_differences(runs_by_key):
    metrics = (
        "full_x_rmse_m", "full_x_iae_m_s", "full_x_peak_abs_m",
        "full_position_rmse_m", "full_position_peak_m",
        "disturbance_on_x_rmse_m", "disturbance_on_x_iae_m_s",
        "disturbance_on_x_peak_abs_m", "disturbance_on_position_rmse_m",
        "disturbance_on_position_peak_m", "disturbance_on_sigma_peak_norm",
        "disturbance_on_sigma_mean_norm", "disturbance_on_sigma_integral_norm_s",
        "post_disturbance_x_peak_abs_m", "post_disturbance_x_iae_m_s",
        "post_disturbance_position_peak_m", "recovery_excess_x_iae_m_s",
    )
    rows = []
    for adaptive_mode in ("estimate_only", "full_adaptive"):
        for run_id in RUNS:
            adaptive = runs_by_key[(adaptive_mode, run_id)]["metrics"]
            nominal = runs_by_key[("nominal", run_id)]["metrics"]
            for metric in metrics:
                difference = adaptive[metric] - nominal[metric]
                relative = None if nominal[metric] == 0.0 else 100.0 * difference / nominal[metric]
                rows.append({
                    "adaptive_mode": adaptive_mode,
                    "baseline_mode": "nominal",
                    "run": run_id,
                    "metric": metric,
                    "adaptive_minus_nominal": difference,
                    "relative_change_percent": relative,
                })
    return rows


def aggregate_rows(rows, grouping_fields, value_fields):
    grouped = {}
    for row in rows:
        key = tuple(row[field] for field in grouping_fields)
        grouped.setdefault(key, {field: [] for field in value_fields})
        for field in value_fields:
            grouped[key][field].append(float(row[field]))
    result = []
    for key, values_by_field in sorted(grouped.items()):
        item = dict(zip(grouping_fields, key))
        for field, values in values_by_field.items():
            array = np.asarray(values)
            item[field] = {
                "n": len(values),
                "mean": float(np.mean(array)),
                "std": float(np.std(array, ddof=1)) if len(values) > 1 else 0.0,
                "min": float(np.min(array)),
                "max": float(np.max(array)),
            }
        result.append(item)
    return result


def trajectory_statistics(mode_runs, signal):
    matrix = np.vstack([run["signals"][signal] for run in mode_runs])
    return np.mean(matrix, axis=0), np.std(matrix, axis=0, ddof=1)


def decorate_axis(axis, ylabel):
    axis.axvspan(DISTURBANCE_START, DISTURBANCE_END, color="#ffbf00", alpha=0.12)
    axis.axvline(DISTURBANCE_START, color="#a06000", linewidth=0.8, alpha=0.6)
    axis.axvline(DISTURBANCE_END, color="#a06000", linewidth=0.8, alpha=0.6)
    axis.set_xlim(WINDOW_START, WINDOW_END)
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.3)


def plot_signal(path, grid, runs_by_mode, signal, ylabel):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(11, 4.6))
    for mode in MODES:
        color = MODE_COLORS[mode]
        for run in runs_by_mode[mode]:
            axis.plot(grid, run["signals"][signal], color=color, alpha=0.18, linewidth=0.7)
        mean, std = trajectory_statistics(runs_by_mode[mode], signal)
        axis.plot(grid, mean, color=color, linewidth=2.0, label=MODE_LABELS[mode])
        axis.fill_between(grid, mean - std, mean + std, color=color, alpha=0.12)
    decorate_axis(axis, ylabel)
    axis.set_xlabel("MPC / simulation time (s)")
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def plot_equation10(path, grid, runs_by_mode):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 1, figsize=(11, 8.0), sharex=True)
    for mode in MODES:
        color = MODE_COLORS[mode]
        for axis, signal in zip(axes, ("yu_pi_Fx", "applied_correction_Fx")):
            for run in runs_by_mode[mode]:
                axis.plot(grid, run["signals"][signal], color=color, alpha=0.18, linewidth=0.7)
            mean, std = trajectory_statistics(runs_by_mode[mode], signal)
            axis.plot(grid, mean, color=color, linewidth=2.0, label=MODE_LABELS[mode])
            axis.fill_between(grid, mean - std, mean + std, color=color, alpha=0.12)
    decorate_axis(axes[0], r"$Y_u\hat{\pi}$ Fx (N)")
    decorate_axis(axes[1], "Applied Eq.(10) Fx correction (m/s²)")
    axes[0].legend(loc="best")
    axes[1].legend(loc="best")
    axes[1].set_xlabel("MPC / simulation time (s)")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def write_group_trajectories(path, grid, runs_by_mode):
    signals = (
        "disturbance_Fx", "pi_hat_Fx", "sigma_Fx", "x_error",
        "position_error_norm", "yu_pi_Fx", "candidate_correction_Fx",
        "applied_correction_Fx",
    )
    columns = {"time_s": grid}
    for mode in MODES:
        for signal in signals:
            mean, std = trajectory_statistics(runs_by_mode[mode], signal)
            columns["{}_{}_mean".format(mode, signal)] = mean
            columns["{}_{}_std".format(mode, signal)] = std
    names = list(columns)
    with open(path, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        for index in range(len(grid)):
            writer.writerow([columns[name][index] for name in names])


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description="Analyze the fixed three-mode Task B formal campaign")
    parser.add_argument("records_directory")
    parser.add_argument("--output-directory", default=None)
    arguments = parser.parse_args()
    records = os.path.abspath(arguments.records_directory)
    output = os.path.abspath(arguments.output_directory or os.path.join(records, "analysis"))
    os.makedirs(output, exist_ok=True)

    acquisition_manifest = load_json(os.path.join(records, "stage_b3_acquisition_manifest.json"))
    if not acquisition_manifest.get("passed"):
        raise RuntimeError("Stage B3 acquisition manifest has not passed")

    grid = np.linspace(WINDOW_START, WINDOW_END, int(round((WINDOW_END - WINDOW_START) / GRID_DT)) + 1)
    runs = []
    runs_by_mode = {mode: [] for mode in MODES}
    runs_by_key = {}
    for mode in MODES:
        for run_id in RUNS:
            run = analyze_run(records, mode, run_id, grid)
            runs.append(run)
            runs_by_mode[mode].append(run)
            runs_by_key[(mode, run_id)] = run

    group_statistics = {mode: numeric_group_statistics(runs_by_mode[mode]) for mode in MODES}
    policy_rows = policy_pairwise(runs_by_key, grid)
    paired_rows = paired_tracking_differences(runs_by_key)

    per_run_path = os.path.join(output, "metrics_per_run.csv")
    group_path = os.path.join(output, "metrics_group_mean_std.csv")
    policy_path = os.path.join(output, "policy_pairwise_differences.csv")
    paired_path = os.path.join(output, "tracking_paired_differences.csv")
    trajectories_path = os.path.join(output, "group_trajectories_mean_std.csv")
    write_per_run_metrics(per_run_path, runs)
    write_group_statistics(group_path, group_statistics)
    write_rows(policy_path, policy_rows)
    write_rows(paired_path, paired_rows)
    write_group_trajectories(trajectories_path, grid, runs_by_mode)

    figure_paths = {
        "A": os.path.join(output, "figure_A_disturbance_Fx.png"),
        "B": os.path.join(output, "figure_B_pi_hat_Fx.png"),
        "C": os.path.join(output, "figure_C_sigma_Fx.png"),
        "D": os.path.join(output, "figure_D_x_error.png"),
        "E": os.path.join(output, "figure_E_position_error_norm.png"),
        "F": os.path.join(output, "figure_F_equation10_correction.png"),
    }
    plot_signal(figure_paths["A"], grid, runs_by_mode, "disturbance_Fx", "True disturbance Fx (N)")
    plot_signal(figure_paths["B"], grid, runs_by_mode, "pi_hat_Fx", "pi_hat Fx (N)")
    plot_signal(figure_paths["C"], grid, runs_by_mode, "sigma_Fx", "sigma Fx")
    plot_signal(figure_paths["D"], grid, runs_by_mode, "x_error", "x error (m)")
    plot_signal(figure_paths["E"], grid, runs_by_mode, "position_error_norm", "Position error norm (m)")
    plot_equation10(figure_paths["F"], grid, runs_by_mode)

    selected_metrics = (
        "full_x_rmse_m", "full_x_iae_m_s", "full_x_peak_abs_m",
        "full_position_rmse_m", "full_position_peak_m",
        "full_sigma_peak_norm", "full_sigma_mean_norm", "full_sigma_integral_norm_s",
        "disturbance_on_x_rmse_m", "disturbance_on_x_iae_m_s",
        "disturbance_on_x_peak_abs_m", "disturbance_on_position_rmse_m",
        "disturbance_on_position_peak_m", "disturbance_on_sigma_peak_norm",
        "disturbance_on_sigma_mean_norm", "disturbance_on_sigma_integral_norm_s",
        "post_disturbance_x_peak_abs_m", "post_disturbance_x_iae_m_s",
        "post_disturbance_position_peak_m", "recovery_excess_x_iae_m_s",
        "recovery_time_0p5s_dwell_s", "recovery_time_sustained_s",
    )
    selected_group_statistics = {
        mode: {metric: group_statistics[mode][metric] for metric in selected_metrics}
        for mode in MODES
    }
    lower_is_better = (
        "full_x_rmse_m", "full_x_iae_m_s", "full_x_peak_abs_m",
        "full_position_rmse_m", "full_position_peak_m",
        "disturbance_on_x_rmse_m", "disturbance_on_x_iae_m_s",
        "disturbance_on_x_peak_abs_m", "disturbance_on_position_rmse_m",
        "disturbance_on_position_peak_m", "post_disturbance_x_peak_abs_m",
        "post_disturbance_x_iae_m_s", "post_disturbance_position_peak_m",
        "recovery_excess_x_iae_m_s",
    )
    relative_changes = {}
    for mode in ("estimate_only", "full_adaptive"):
        relative_changes[mode] = {}
        for metric in lower_is_better:
            nominal_mean = group_statistics["nominal"][metric]["mean"]
            adaptive_mean = group_statistics[mode][metric]["mean"]
            relative_changes[mode][metric] = 100.0 * (adaptive_mean - nominal_mean) / nominal_mean
    summary = {
        "stage": "B4 automated analysis and plots",
        "passed": True,
        "scientific_scope": (
            "Observed effects for the current animal_dummy, fixed 20 N Fx disturbance, fixed target, "
            "and fixed configuration only; this is not proof of the paper's general theoretical claims."
        ),
        "performance_analysis_performed": True,
        "definitions": {
            "analysis_window_s": [WINDOW_START, WINDOW_END],
            "disturbance_on_window_s": [DISTURBANCE_START, DISTURBANCE_END],
            "post_disturbance_window_s": [DISTURBANCE_END, POST_END],
            "common_grid_dt_s": GRID_DT,
            "integration": "trapezoidal on a common 20 ms MPC-time grid",
            "position_rmse": "sqrt(time mean of squared Euclidean position-error norm)",
            "sigma_mean": "time mean of the Euclidean six-dimensional sigma norm",
            "contact_force_input_indices": [0, CONTACT_FORCE_DIMENSION - 1],
            "recovery": {
                "signal": "absolute x tracking error",
                "baseline_window_s": [RECOVERY_BASELINE_START, RECOVERY_BASELINE_END],
                "threshold": "per-run baseline 95th percentile plus 0.001 m",
                "dwell_s": RECOVERY_DWELL_S,
                "sustained": "first post-release time remaining below threshold through 15.5 s",
                "unrecovered": "null time and censored=true",
            },
            "group_std": "sample standard deviation with ddof=1, n=3",
        },
        "input_validation": {
            "stage_b3_manifest_passed": True,
            "run_count": len(runs),
            "all_run_times_cover_analysis_window": True,
            "all_inputs_regridded_to_same_sample_count": len(grid),
        },
        "selected_group_mean_std": selected_group_statistics,
        "group_mean_relative_change_vs_nominal_percent": relative_changes,
        "policy_pairwise_mean_std": aggregate_rows(
            policy_rows,
            ("left_mode", "right_mode", "window"),
            ("policy_rms_vector_difference", "contact_force_rms_vector_difference_N"),
        ),
        "tracking_paired_difference_mean_std": aggregate_rows(
            paired_rows,
            ("adaptive_mode", "baseline_mode", "metric"),
            ("adaptive_minus_nominal", "relative_change_percent"),
        ),
        "per_run_recovery": {
            "{}/{}".format(run["mode"], run["run"]): {
                key: run["metrics"][key] for key in (
                    "recovery_threshold_m", "recovery_time_0p5s_dwell_s",
                    "recovery_time_sustained_s", "recovery_0p5s_dwell_censored",
                    "recovery_sustained_censored", "recovery_excess_x_iae_m_s",
                )
            }
            for run in runs
        },
        "mechanism_checks": {
            "nominal_peak_pi_hat_norm": [
                run["metrics"]["full_pi_hat_peak_norm"] for run in runs_by_mode["nominal"]
            ],
            "estimate_only_peak_pi_hat_norm": [
                run["metrics"]["full_pi_hat_peak_norm"] for run in runs_by_mode["estimate_only"]
            ],
            "estimate_only_peak_applied_correction_norm": [
                run["metrics"]["full_applied_correction_peak_norm"]
                for run in runs_by_mode["estimate_only"]
            ],
            "full_adaptive_peak_applied_correction_norm": [
                run["metrics"]["full_applied_correction_peak_norm"]
                for run in runs_by_mode["full_adaptive"]
            ],
        },
        "repeatability_warning": (
            "Estimate-only run_03 follows a different closed-loop/estimator branch. "
            "Retain all three runs and interpret mean plus standard deviation."
        ),
        "artifacts": {},
    }
    summary_path = os.path.join(output, "stage_b4_summary.json")
    for path in (per_run_path, group_path, policy_path, paired_path, trajectories_path, *figure_paths.values()):
        summary["artifacts"][os.path.basename(path)] = {"sha256": sha256(path), "size_bytes": os.path.getsize(path)}
    with open(summary_path, "w") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
