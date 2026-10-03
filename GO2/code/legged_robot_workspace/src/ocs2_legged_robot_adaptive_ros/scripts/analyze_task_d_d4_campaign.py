#!/usr/bin/env python3
"""Unified common-grid analysis for the repeated Stage D4 10D campaign."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


GROUPS = ("nominal", "uncertain_off", "uncertain_adaptive")
RUNS = ("run_01", "run_02", "run_03")
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy", "Jyz", "Jzz")
PARAMETER_UNITS = ("kg", "kg m", "kg m", "kg m", "kg m^2", "kg m^2",
                   "kg m^2", "kg m^2", "kg m^2", "kg m^2")
SIGN_THRESHOLDS = {
    "m": 0.05, "hx": 0.005, "hy": 0.005, "hz": 0.005,
    "Jxx": 0.002, "Jxy": 0.002, "Jxz": 0.002,
    "Jyy": 0.002, "Jyz": 0.002, "Jzz": 0.002,
}
LABELS = {
    "nominal": "Nominal plant + Full Adaptive",
    "uncertain_off": "Payload plant + Adaptive OFF",
    "uncertain_adaptive": "Payload plant + Full Adaptive",
}
COLORS = {
    "nominal": "#4c4c4c", "uncertain_off": "#1f77b4",
    "uncertain_adaptive": "#d62728",
}
PAIRS = (
    ("uncertain_off_minus_nominal", "nominal", "uncertain_off"),
    ("uncertain_adaptive_minus_nominal", "nominal", "uncertain_adaptive"),
    ("uncertain_adaptive_minus_uncertain_off", "uncertain_off", "uncertain_adaptive"),
)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_csv(path):
    with Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        names = reader.fieldnames
    if not rows or not names:
        raise RuntimeError("empty D4 CSV: " + str(path))
    data = {name: np.asarray([float(row[name]) for row in rows], dtype=float)
            for name in names}
    if any(not np.all(np.isfinite(values)) for values in data.values()):
        raise RuntimeError("non-finite D4 CSV: " + str(path))
    return data


def trapz(values, time):
    return float(np.trapz(values, time))


def rms(values, time):
    values = np.asarray(values)
    squared = values ** 2 if values.ndim == 1 else np.sum(values ** 2, axis=1)
    return math.sqrt(trapz(squared, time) / float(time[-1] - time[0]))


def time_mean(values, time):
    return trapz(values, time) / float(time[-1] - time[0])


def matrix(data, prefix, names):
    return np.column_stack([data[prefix + name] for name in names])


def analyze_run(root, group, run_name, expected_grid):
    run_dir = root / group / run_name
    summary = json.loads((run_dir / "task_d_d4_run_summary.json").read_text())
    config = json.loads((run_dir / "effective_config.json").read_text())
    if not summary.get("pass") or not summary.get("formal_eligibility"):
        raise RuntimeError("D4 run is not formally eligible: {}/{}".format(group, run_name))
    data = load_csv(run_dir / "task_d_d4_run.csv")
    time = data["time_s"]
    if len(time) != len(expected_grid) or not np.allclose(time, expected_grid, rtol=0.0, atol=1.0e-12):
        raise RuntimeError("D4 common grid mismatch: {}/{}".format(group, run_name))
    pi_hat = matrix(data, "pi_hat_", PARAMETERS)
    pi_star = matrix(data, "pi_star_", PARAMETERS)
    pi_error = pi_hat - pi_star
    position_error = np.column_stack([data[name] for name in ("x_error", "y_error", "z_error")])
    orientation_error = np.column_stack([
        data[name] for name in ("roll_error", "pitch_error", "yaw_error")])
    policy = np.column_stack([data["policy_{}".format(index)] for index in range(24)])
    contact = policy[:, :12]
    metrics = {
        "x_rmse_m": rms(data["x_error"], time),
        "x_iae_m_s": trapz(np.abs(data["x_error"]), time),
        "x_peak_abs_m": float(np.max(np.abs(data["x_error"]))),
        "y_rmse_m": rms(data["y_error"], time),
        "y_peak_abs_m": float(np.max(np.abs(data["y_error"]))),
        "z_rmse_m": rms(data["z_error"], time),
        "z_peak_abs_m": float(np.max(np.abs(data["z_error"]))),
        "position_rmse_m": rms(position_error, time),
        "position_peak_m": float(np.max(np.linalg.norm(position_error, axis=1))),
        "orientation_rmse_rad": rms(orientation_error, time),
        "orientation_peak_rad": float(np.max(np.linalg.norm(orientation_error, axis=1))),
        "sigma_mean_norm": time_mean(data["sigma_norm"], time),
        "sigma_peak_norm": float(np.max(data["sigma_norm"])),
        "sigma_integral_norm_s": trapz(data["sigma_norm"], time),
        "pi_coordinate_vector_rmse_mixed_units": rms(pi_error, time),
        "pi_coordinate_error_mean_norm_mixed_units": time_mean(
            np.linalg.norm(pi_error, axis=1), time),
        "pi_coordinate_error_peak_norm_mixed_units": float(
            np.max(np.linalg.norm(pi_error, axis=1))),
        "candidate_correction_rms_norm": rms(data["candidate_flow_correction_norm"], time),
        "candidate_correction_peak_norm": float(np.max(data["candidate_flow_correction_norm"])),
        "applied_correction_rms_norm": rms(data["applied_flow_correction_norm"], time),
        "applied_correction_peak_norm": float(np.max(data["applied_flow_correction_norm"])),
        "true_vs_nominal_flow_residual_rms": rms(
            data["true_vs_nominal_flow_residual_norm"], time),
        "policy_rms_norm": rms(policy, time),
        "contact_force_rms_norm_N": rms(contact, time),
        "actual_x_displacement_m": float(data["actual_x"][-1] - data["actual_x"][0]),
        "final_actual_x_m": float(data["actual_x"][-1]),
        "final_x_error_m": float(data["x_error"][-1]),
        "final_actual_z_m": float(data["actual_z"][-1]),
        "final_z_error_m": float(data["z_error"][-1]),
        "estimated_total_mass_min_kg": float(np.min(data["estimated_total_mass_kg"])),
        "estimated_total_mass_max_kg": float(np.max(data["estimated_total_mass_kg"])),
    }
    last_two = time >= time[-1] - 2.0
    for index, name in enumerate(PARAMETERS):
        error = pi_error[:, index]
        truth = pi_star[:, index]
        estimate = pi_hat[:, index]
        truth_value = float(np.mean(truth))
        metrics["pi_{}_rmse".format(name)] = rms(error, time)
        metrics["pi_{}_signed_bias".format(name)] = time_mean(error, time)
        metrics["pi_{}_peak_abs_error".format(name)] = float(np.max(np.abs(error)))
        metrics["pi_{}_final_estimate".format(name)] = float(estimate[-1])
        metrics["pi_{}_last2s_mean_estimate".format(name)] = float(np.mean(estimate[last_two]))
        metrics["pi_{}_truth".format(name)] = truth_value
        if abs(truth_value) >= SIGN_THRESHOLDS[name]:
            metrics["pi_{}_sign_agreement_ratio".format(name)] = float(np.mean(
                np.sign(estimate) == np.sign(truth_value)))
            metrics["pi_{}_last2s_magnitude_ratio".format(name)] = float(
                abs(np.mean(estimate[last_two])) / abs(truth_value))
        else:
            metrics["pi_{}_sign_agreement_ratio".format(name)] = None
            metrics["pi_{}_last2s_magnitude_ratio".format(name)] = None
    return {
        "group": group, "run": run_name, "directory": run_dir,
        "summary": summary, "config": config, "data": data,
        "pi_hat": pi_hat, "pi_star": pi_star, "pi_error": pi_error,
        "policy": policy, "contact": contact, "metrics": metrics,
    }


def numeric_stats(values):
    values = [float(value) for value in values if value is not None]
    if not values:
        return {"n": 0, "mean": None, "sample_std": None, "min": None, "max": None}
    return {
        "n": len(values), "mean": float(np.mean(values)),
        "sample_std": float(np.std(values, ddof=1)) if len(values) > 1 else None,
        "min": min(values), "max": max(values),
    }


def group_statistics(runs):
    names = sorted(set.intersection(*[set(run["metrics"]) for run in runs]))
    return {name: numeric_stats([run["metrics"][name] for run in runs])
            for name in names}


def trajectory_stats(runs, key):
    values = np.vstack([run["data"][key] for run in runs])
    return np.mean(values, axis=0), np.std(values, axis=0, ddof=1)


def write_rows(path, rows):
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def decorate(axis, ylabel, start, end):
    axis.set_xlim(start, end)
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.3)


def plot_signal(path, grid, grouped, key, ylabel):
    fig, axis = plt.subplots(figsize=(11, 4.8))
    for group in GROUPS:
        mean, std = trajectory_stats(grouped[group], key)
        axis.plot(grid, mean, color=COLORS[group], lw=2, label=LABELS[group])
        axis.fill_between(grid, mean - std, mean + std, color=COLORS[group], alpha=0.14)
    decorate(axis, ylabel, grid[0], grid[-1])
    axis.set_xlabel("MPC time [s]")
    axis.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_desired_actual(path, grid, grouped):
    fig, axis = plt.subplots(figsize=(11, 4.8))
    desired, _ = trajectory_stats(grouped["nominal"], "desired_x")
    axis.plot(grid, desired, "k--", lw=1.8, label="Desired x")
    for group in GROUPS:
        mean, std = trajectory_stats(grouped[group], "actual_x")
        axis.plot(grid, mean, color=COLORS[group], lw=2, label=LABELS[group])
        axis.fill_between(grid, mean - std, mean + std, color=COLORS[group], alpha=0.14)
    decorate(axis, "x position [m]", grid[0], grid[-1])
    axis.set_xlabel("MPC time [s]")
    axis.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_parameters(path, grid, grouped, indices, title):
    columns = min(3, len(indices))
    rows = int(math.ceil(len(indices) / columns))
    fig, axes = plt.subplots(rows, columns, figsize=(6 * columns, 3.7 * rows),
                             sharex=True, squeeze=False)
    for axis, index in zip(axes.flat, indices):
        name = PARAMETERS[index]
        for group in GROUPS:
            mean, std = trajectory_stats(grouped[group], "pi_hat_" + name)
            axis.plot(grid, mean, color=COLORS[group], lw=1.8, label=LABELS[group])
            axis.fill_between(grid, mean - std, mean + std,
                              color=COLORS[group], alpha=0.12)
        truth, _ = trajectory_stats(grouped["uncertain_adaptive"], "pi_star_" + name)
        axis.plot(grid, truth, "k--", lw=1.5, label="Payload pi_star")
        decorate(axis, "{} [{}]".format(name, PARAMETER_UNITS[index]), grid[0], grid[-1])
    for axis in axes.flat[len(indices):]:
        axis.set_visible(False)
    axes.flat[0].legend(loc="best", fontsize=8)
    for axis in axes[-1, :]:
        if axis.get_visible():
            axis.set_xlabel("MPC time [s]")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_component_errors(path, grid, grouped):
    fig, axes = plt.subplots(5, 2, figsize=(13, 17), sharex=True)
    for axis, name, unit in zip(axes.flat, PARAMETERS, PARAMETER_UNITS):
        for group in GROUPS:
            mean, std = trajectory_stats(grouped[group], "pi_error_" + name)
            axis.plot(grid, mean, color=COLORS[group], lw=1.6, label=LABELS[group])
            axis.fill_between(grid, mean - std, mean + std,
                              color=COLORS[group], alpha=0.12)
        axis.axhline(0.0, color="black", ls="--", lw=1)
        decorate(axis, "{} error [{}]".format(name, unit), grid[0], grid[-1])
    axes.flat[0].legend(loc="best", fontsize=8)
    for axis in axes[-1, :]:
        axis.set_xlabel("MPC time [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_bias_corrected(path, grid, bias_corrected_by_run, truth_by_run):
    fig, axes = plt.subplots(5, 2, figsize=(13, 17), sharex=True)
    estimates = np.stack(bias_corrected_by_run)
    truths = np.stack(truth_by_run)
    for index, (axis, name, unit) in enumerate(
            zip(axes.flat, PARAMETERS, PARAMETER_UNITS)):
        mean = np.mean(estimates[:, :, index], axis=0)
        std = np.std(estimates[:, :, index], axis=0, ddof=1)
        truth = np.mean(truths[:, :, index], axis=0)
        axis.plot(grid, mean, color="#9467bd", lw=1.8,
                  label="pi_hat U-ON - pi_hat Nominal")
        axis.fill_between(grid, mean - std, mean + std,
                          color="#9467bd", alpha=0.14)
        axis.plot(grid, truth, "k--", lw=1.5, label="payload pi_star")
        decorate(axis, "{} [{}]".format(name, unit), grid[0], grid[-1])
    axes.flat[0].legend(loc="best", fontsize=8)
    for axis in axes[-1, :]:
        axis.set_xlabel("MPC time [s]")
    fig.suptitle("Paired nominal-bias-corrected payload estimate")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_equation10(path, grid, grouped):
    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    for group in GROUPS:
        for axis, key in zip(axes, (
                "candidate_flow_correction_norm", "applied_flow_correction_norm")):
            mean, std = trajectory_stats(grouped[group], key)
            axis.plot(grid, mean, color=COLORS[group], lw=2, label=LABELS[group])
            axis.fill_between(grid, mean - std, mean + std,
                              color=COLORS[group], alpha=0.14)
    decorate(axes[0], "Candidate full-flow norm", grid[0], grid[-1])
    decorate(axes[1], "Applied full-flow norm", grid[0], grid[-1])
    axes[0].legend(loc="best")
    axes[1].legend(loc="best")
    axes[1].set_xlabel("MPC time [s]")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--output-directory", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    output = args.output_directory.resolve()
    output.mkdir(parents=True, exist_ok=True)
    fairness = json.loads((root / "stage_d4_fairness_audit.json").read_text())
    protocol = json.loads((root / "protocol" / "stage_d4_protocol.json").read_text())
    if not fairness.get("passed") or not protocol.get("passed"):
        raise RuntimeError("D4 protocol/fairness gate has not passed")
    start = protocol["analysis_window"]["start_mpc_time_s"]
    end = protocol["analysis_window"]["end_mpc_time_s"]
    dt = protocol["analysis_window"]["common_grid_dt_s"]
    grid = np.linspace(start, end, int(round((end - start) / dt)) + 1)

    runs, grouped, keyed = [], {group: [] for group in GROUPS}, {}
    for group in GROUPS:
        for run_name in RUNS:
            run = analyze_run(root, group, run_name, grid)
            runs.append(run)
            grouped[group].append(run)
            keyed[(group, run_name)] = run
    stats = {group: group_statistics(grouped[group]) for group in GROUPS}

    per_run_rows = []
    for run in runs:
        row = {"group": run["group"], "run": run["run"]}
        row.update(run["metrics"])
        per_run_rows.append(row)
    metric_rows = []
    for group in GROUPS:
        for metric, values in stats[group].items():
            metric_rows.append({"group": group, "metric": metric, **values})

    paired_rows, contact_difference = [], {}
    common_metrics = sorted(set.intersection(*[
        set(keyed[(group, "run_01")]["metrics"]) for group in GROUPS]))
    for pair_name, baseline_group, comparison_group in PAIRS:
        contact_difference[pair_name] = []
        for run_name in RUNS:
            baseline = keyed[(baseline_group, run_name)]
            comparison = keyed[(comparison_group, run_name)]
            contact_delta = comparison["contact"] - baseline["contact"]
            policy_delta = comparison["policy"] - baseline["policy"]
            contact_difference[pair_name].append(np.linalg.norm(contact_delta, axis=1))
            paired_rows.append({
                "pair": pair_name, "run": run_name, "metric": "policy_rms_vector_difference",
                "difference": rms(policy_delta, grid),
            })
            paired_rows.append({
                "pair": pair_name, "run": run_name,
                "metric": "contact_force_rms_vector_difference_N",
                "difference": rms(contact_delta, grid),
            })
            for metric in common_metrics:
                left = baseline["metrics"][metric]
                right = comparison["metrics"][metric]
                if left is not None and right is not None:
                    paired_rows.append({
                        "pair": pair_name, "run": run_name, "metric": metric,
                        "difference": right - left,
                    })

    # The nominal adaptive group has pi_star=0 but a nonzero estimator response.
    # Preserve the direct comparison and additionally report a paired difference
    # that removes this common nominal trajectory/model bias. This is a
    # diagnostic contrast, not a replacement estimator and not a convergence claim.
    bias_corrected_rows = []
    bias_corrected_by_run, bias_truth_by_run = [], []
    for run_name in RUNS:
        nominal = keyed[("nominal", run_name)]
        adaptive = keyed[("uncertain_adaptive", run_name)]
        corrected = adaptive["pi_hat"] - nominal["pi_hat"]
        truth = adaptive["pi_star"]
        error = corrected - truth
        bias_corrected_by_run.append(corrected)
        bias_truth_by_run.append(truth)
        last_two = grid >= grid[-1] - 2.0
        bias_corrected_rows.append({
            "run": run_name, "metric": "coordinate_vector_rmse_mixed_units",
            "value": rms(error, grid)})
        for index, name in enumerate(PARAMETERS):
            truth_value = float(np.mean(truth[:, index]))
            bias_corrected_rows.extend((
                {"run": run_name, "metric": "pi_{}_rmse".format(name),
                 "value": rms(error[:, index], grid)},
                {"run": run_name, "metric": "pi_{}_signed_bias".format(name),
                 "value": time_mean(error[:, index], grid)},
                {"run": run_name, "metric": "pi_{}_last2s_mean_estimate".format(name),
                 "value": float(np.mean(corrected[last_two, index]))},
                {"run": run_name, "metric": "pi_{}_truth".format(name),
                 "value": truth_value},
            ))
            if abs(truth_value) >= SIGN_THRESHOLDS[name]:
                bias_corrected_rows.extend((
                    {"run": run_name, "metric": "pi_{}_sign_agreement_ratio".format(name),
                     "value": float(np.mean(
                         np.sign(corrected[:, index]) == np.sign(truth_value)))},
                    {"run": run_name, "metric": "pi_{}_last2s_magnitude_ratio".format(name),
                     "value": float(abs(np.mean(corrected[last_two, index])) /
                                    abs(truth_value))},
                ))

    trajectory_keys = (
        "desired_x", "actual_x", "x_error", "position_error_norm",
        "orientation_error_norm", "sigma_norm", "pi_error_coordinate_norm",
        "candidate_flow_correction_norm", "applied_flow_correction_norm",
        "true_vs_nominal_flow_residual_norm", "estimated_total_mass_kg",
    ) + tuple("pi_hat_" + name for name in PARAMETERS) \
      + tuple("pi_star_" + name for name in PARAMETERS) \
      + tuple("pi_error_" + name for name in PARAMETERS)
    trajectory_columns = {"time_s": grid}
    for group in GROUPS:
        for key in trajectory_keys:
            mean, std = trajectory_stats(grouped[group], key)
            trajectory_columns["{}_{}_mean".format(group, key)] = mean
            trajectory_columns["{}_{}_sample_std".format(group, key)] = std
    for pair_name, values in contact_difference.items():
        values = np.vstack(values)
        trajectory_columns[pair_name + "_contact_difference_norm_mean"] = np.mean(values, axis=0)
        trajectory_columns[pair_name + "_contact_difference_norm_sample_std"] = np.std(
            values, axis=0, ddof=1)

    per_run_path = output / "metrics_per_run.csv"
    group_path = output / "metrics_group_mean_std.csv"
    paired_path = output / "paired_differences.csv"
    bias_corrected_path = output / "bias_corrected_payload_metrics.csv"
    trajectories_path = output / "group_trajectories_mean_std.csv"
    write_rows(per_run_path, per_run_rows)
    write_rows(group_path, metric_rows)
    write_rows(paired_path, paired_rows)
    write_rows(bias_corrected_path, bias_corrected_rows)
    with trajectories_path.open("w", newline="") as stream:
        names = list(trajectory_columns)
        writer = csv.writer(stream)
        writer.writerow(names)
        for index in range(len(grid)):
            writer.writerow([trajectory_columns[name][index] for name in names])

    figures = {
        "D4_01": output / "figure_D4_01_desired_vs_actual_x.png",
        "D4_02": output / "figure_D4_02_x_error.png",
        "D4_03": output / "figure_D4_03_mass_estimate.png",
        "D4_04": output / "figure_D4_04_first_moments.png",
        "D4_05": output / "figure_D4_05_inertia_parameters.png",
        "D4_06": output / "figure_D4_06_parameter_component_errors.png",
        "D4_07": output / "figure_D4_07_parameter_coordinate_error_norm.png",
        "D4_08": output / "figure_D4_08_sigma_norm.png",
        "D4_09": output / "figure_D4_09_equation10_correction.png",
        "D4_10": output / "figure_D4_10_contact_force_policy_difference.png",
        "D4_11": output / "figure_D4_11_bias_corrected_payload_estimate.png",
    }
    plot_desired_actual(figures["D4_01"], grid, grouped)
    plot_signal(figures["D4_02"], grid, grouped, "x_error", "x error [m]")
    plot_parameters(figures["D4_03"], grid, grouped, (0,), "Payload mass coordinate")
    plot_parameters(figures["D4_04"], grid, grouped, (1, 2, 3), "Payload first moments")
    plot_parameters(figures["D4_05"], grid, grouped, tuple(range(4, 10)), "Payload inertia coordinates")
    plot_component_errors(figures["D4_06"], grid, grouped)
    plot_signal(figures["D4_07"], grid, grouped, "pi_error_coordinate_norm",
                "10D coordinate error norm [mixed units]")
    plot_signal(figures["D4_08"], grid, grouped, "sigma_norm", "sigma norm")
    plot_equation10(figures["D4_09"], grid, grouped)
    fig, axis = plt.subplots(figsize=(11, 4.8))
    for pair_name, values in contact_difference.items():
        values = np.vstack(values)
        mean, std = np.mean(values, axis=0), np.std(values, axis=0, ddof=1)
        axis.plot(grid, mean, lw=1.8, label=pair_name)
        axis.fill_between(grid, mean - std, mean + std, alpha=0.12)
    decorate(axis, "Contact-force difference norm [N]", start, end)
    axis.set_xlabel("MPC time [s]")
    axis.legend(loc="best")
    fig.tight_layout()
    fig.savefig(figures["D4_10"], dpi=180)
    plt.close(fig)
    plot_bias_corrected(figures["D4_11"], grid, bias_corrected_by_run,
                        bias_truth_by_run)

    lower_is_better = (
        "x_rmse_m", "x_iae_m_s", "x_peak_abs_m",
        "y_rmse_m", "y_peak_abs_m", "z_rmse_m", "z_peak_abs_m",
        "position_rmse_m",
        "position_peak_m", "orientation_rmse_rad", "orientation_peak_rad",
        "sigma_mean_norm", "sigma_peak_norm", "sigma_integral_norm_s")
    recovery = {}
    for metric in lower_is_better:
        nominal = stats["nominal"][metric]["mean"]
        off = stats["uncertain_off"][metric]["mean"]
        on = stats["uncertain_adaptive"][metric]["mean"]
        degradation = off - nominal
        eligible = degradation > max(1.0e-12, 0.01 * abs(nominal))
        recovery[metric] = {
            "nominal_mean": nominal, "uncertain_off_mean": off,
            "uncertain_adaptive_mean": on,
            "off_minus_nominal_degradation": degradation,
            "on_minus_nominal_residual_degradation": on - nominal,
            "recovery_fraction_eligible": eligible,
            "recovered_fraction_percent": (
                100.0 * (off - on) / degradation if eligible else None),
        }

    paired_aggregate = {}
    for row in paired_rows:
        key = row["pair"] + "/" + row["metric"]
        paired_aggregate.setdefault(key, []).append(row["difference"])
    paired_aggregate = {key: numeric_stats(values)
                        for key, values in paired_aggregate.items()}
    bias_corrected_aggregate = {}
    for row in bias_corrected_rows:
        bias_corrected_aggregate.setdefault(row["metric"], []).append(row["value"])
    bias_corrected_aggregate = {
        key: numeric_stats(values) for key, values in bias_corrected_aggregate.items()}
    selected_metrics = lower_is_better + (
        "pi_coordinate_vector_rmse_mixed_units",
        "pi_coordinate_error_mean_norm_mixed_units",
        "pi_coordinate_error_peak_norm_mixed_units",
        "candidate_correction_rms_norm", "candidate_correction_peak_norm",
        "applied_correction_rms_norm", "applied_correction_peak_norm",
        "true_vs_nominal_flow_residual_rms", "policy_rms_norm",
        "contact_force_rms_norm_N", "actual_x_displacement_m",
        "final_actual_x_m", "final_x_error_m", "final_actual_z_m",
        "final_z_error_m",
    ) + tuple("pi_{}_rmse".format(name) for name in PARAMETERS) \
      + tuple("pi_{}_signed_bias".format(name) for name in PARAMETERS) \
      + tuple("pi_{}_last2s_mean_estimate".format(name) for name in PARAMETERS) \
      + tuple("pi_{}_truth".format(name) for name in PARAMETERS) \
      + tuple("pi_{}_sign_agreement_ratio".format(name) for name in PARAMETERS) \
      + tuple("pi_{}_last2s_magnitude_ratio".format(name) for name in PARAMETERS)
    selected_stats = {
        group: {metric: stats[group][metric] for metric in selected_metrics}
        for group in GROUPS}

    artifacts = {}
    for path in (per_run_path, group_path, paired_path, bias_corrected_path,
                 trajectories_path, *figures.values()):
        artifacts[path.name] = {"sha256": digest(path), "size_bytes": path.stat().st_size}
    summary = {
        "schema_version": 4,
        "stage": "D4_formal_unified_analysis",
        "passed": True,
        "formal_campaign_complete": True,
        "input": {
            "groups": list(GROUPS), "runs_per_group": 3,
            "total_runs": 9, "fairness_audit_passed": True,
            "all_runs_common_grid": True},
        "definitions": {
            "analysis_window_s": [start, end],
            "analysis_duration_s": end - start,
            "common_grid_dt_s": dt,
            "common_grid_sample_count": len(grid),
            "integration": "trapezoidal on common MPC-time grid",
            "group_dispersion": "sample standard deviation, ddof=1, n=3",
            "parameter_order": list(PARAMETERS),
            "parameter_units": list(PARAMETER_UNITS),
            "pi_coordinate_vector_norm_warning": (
                "coordinates have mixed physical units; use component metrics for physical interpretation"),
            "correlation_and_lag": (
                "not reported because physical pi_star is constant throughout each run"),
            "pairing": "same round index in the frozen N -> U-OFF -> U-ON order",
        },
        "selected_group_mean_std": selected_stats,
        "adaptive_recovery_of_off_vs_nominal_degradation": recovery,
        "paired_differences_mean_std": paired_aggregate,
        "paired_nominal_bias_corrected_payload_reconstruction_mean_std":
            bias_corrected_aggregate,
        "artifacts": artifacts,
        "scientific_scope": (
            "Observed results only for the frozen animal_dummy, fixed 10% off-center payload, "
            "10 cm static-walk target and untuned Gamma=I10. U-ON versus U-OFF isolates the "
            "two adaptive mode bits; it does not by itself prove parameter convergence or the paper's general claims."),
    }
    summary_path = output / "stage_d4_analysis_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    manifest = {
        "schema_version": 4, "stage": "D4_analysis_manifest", "passed": True,
        "fairness_audit_sha256": digest(root / "stage_d4_fairness_audit.json"),
        "protocol_sha256": digest(root / "protocol" / "stage_d4_protocol.json"),
        "analysis_source": str(Path(__file__).resolve()),
        "analysis_source_sha256": digest(Path(__file__).resolve()),
        "summary": summary_path.name,
        "summary_sha256": digest(summary_path),
    }
    (output / "stage_d4_analysis_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
