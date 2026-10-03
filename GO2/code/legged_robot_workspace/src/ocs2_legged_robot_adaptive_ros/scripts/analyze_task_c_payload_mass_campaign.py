#!/usr/bin/env python3
"""Common-grid analysis for PayloadMass1D formal and Adaptive-OFF campaigns."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


GROUPS = ("nominal", "uncertain")
RUNS = ("run_01", "run_02", "run_03")
GRID = np.arange(0.50, 30.0 + 1.0e-9, 0.02)
EXPECTED_TARGET_START = 0.50
OBSERVATION_TOPIC = "/legged_robot_mpc_observation"
TARGET_TOPIC = "/legged_robot_mpc_target"
EQ10_TOPIC = "/task_b/equation10_sample"


def load_json(path):
    return json.loads(path.read_text())


def load_payload_csv(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("empty payload CSV: {}".format(path))
    names = rows[0].keys()
    raw = {name: np.asarray([float(row[name]) for row in rows], dtype=float)
           for name in names}
    order = np.argsort(raw["observation_time"])
    time = raw["observation_time"][order]
    keep = np.r_[np.diff(time) > 0.0, True]
    return {name: values[order][keep] for name, values in raw.items()}


def load_bag(path):
    import rosbag
    observations = []
    targets = []
    eq10 = []
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, _ in bag.read_messages(
                topics=[OBSERVATION_TOPIC, TARGET_TOPIC, EQ10_TOPIC]):
            if topic == OBSERVATION_TOPIC:
                observations.append((float(message.time),
                                     np.asarray(message.state.value, dtype=float),
                                     np.asarray(message.input.value, dtype=float)))
            elif topic == TARGET_TOPIC:
                targets.append((np.asarray(message.timeTrajectory, dtype=float),
                                np.vstack([np.asarray(value.value, dtype=float)
                                           for value in message.stateTrajectory])))
            else:
                values = np.asarray(message.data, dtype=float)
                if len(values) != 24 or not np.all(np.isfinite(values)):
                    raise RuntimeError("invalid Eq.(10) sample in {}".format(path))
                eq10.append(values)
    if not observations or not targets or not eq10:
        raise RuntimeError("bag missing observation/target/Eq.(10): {}".format(path))
    by_time = {row[0]: row for row in observations}
    observations = [by_time[key] for key in sorted(by_time)]
    target_times, target_states = targets[-1]
    if len(target_times) != 2 or target_states.shape[0] != 2:
        raise RuntimeError("target is not the deterministic two-knot trajectory")
    return {
        "observation_time": np.asarray([row[0] for row in observations]),
        "state": np.vstack([row[1] for row in observations]),
        "input": np.vstack([row[2] for row in observations]),
        "target_time": target_times,
        "target_state": target_states,
        "eq10": np.vstack(eq10),
    }


def interpolate(time, values):
    return np.interp(GRID, time, values)


def matrix_interpolate(time, values):
    return np.column_stack([interpolate(time, values[:, index])
                            for index in range(values.shape[1])])


def integral(values):
    return float(np.trapz(values, GRID))


def rmse(values):
    return math.sqrt(integral(np.asarray(values) ** 2) / (GRID[-1] - GRID[0]))


def sustained_entry(error, tolerance):
    bad = np.flatnonzero(np.abs(error) > tolerance)
    if not len(bad):
        return float(GRID[0])
    index = int(bad[-1]) + 1
    return float(GRID[index]) if index < len(GRID) else None


def run_analysis(root, group, run_name):
    run = root / group / run_name
    validation = load_json(run / "payload_mass_smoke_summary.json")
    experiment = load_json(run / "payload_mass_experiment_config.json")
    if validation.get("passed") is not True or experiment.get("group") != group:
        raise RuntimeError("per-run validation/config failed: {}".format(run))
    raw = load_payload_csv(run / "payload_mass_smoke.csv")
    time = raw["observation_time"]
    if time[0] > GRID[0] or time[-1] < GRID[-1]:
        raise RuntimeError("run does not cover common 0.50--30.00 s window: {}".format(run))
    signals = {name: interpolate(time, values) for name, values in raw.items()
               if name != "observation_time"}
    bag = load_bag(run / "payload_mass_smoke.bag")
    if bag["observation_time"][0] > GRID[0] or bag["observation_time"][-1] < GRID[-1]:
        raise RuntimeError("observation does not cover common window: {}".format(run))
    state = matrix_interpolate(bag["observation_time"], bag["state"])
    policy = matrix_interpolate(bag["observation_time"], bag["input"])
    desired = matrix_interpolate(bag["target_time"], bag["target_state"])
    if state.shape[1] < 9 or desired.shape[1] < 9:
        raise RuntimeError("unexpected state dimension: {}".format(run))
    target_duration = float(bag["target_time"][1] - bag["target_time"][0])
    target_step = float(bag["target_state"][1, 6] - bag["target_state"][0, 6])
    if abs(bag["target_time"][0] - EXPECTED_TARGET_START) > 1.0e-6 or \
            abs(target_duration - 0.20) > 1.0e-5 or abs(target_step - 0.10) > 1.0e-5:
        raise RuntimeError("deterministic target schedule changed: {}".format(run))

    eq10 = bag["eq10"]
    eq10_time = eq10[:, 0]
    applied = matrix_interpolate(eq10_time, eq10[:, 18:24])
    candidate = matrix_interpolate(eq10_time, eq10[:, 12:18])
    estimate = signals["payload_mass_estimate"]
    derivative = signals["payload_mass_derivative"]
    truth = float(np.median(signals["true_payload_delta"]))
    error = estimate - truth
    actual_position = state[:, 6:9]
    desired_position = desired[:, 6:9]
    position_error = desired_position - actual_position
    x_error = position_error[:, 0]
    z_error = position_error[:, 2]
    position_error_norm = np.linalg.norm(position_error, axis=1)
    applied_norm = np.linalg.norm(applied, axis=1)
    candidate_norm = np.linalg.norm(candidate, axis=1)
    last10 = GRID >= 20.0
    last5 = GRID >= 25.0
    slope = float(np.polyfit(GRID[last10], estimate[last10], 1)[0])
    metrics = {
        "true_payload_delta_kg": truth,
        "estimate_at_30s_kg": float(estimate[-1]),
        "final_signed_error_kg": float(error[-1]),
        "final_abs_error_kg": float(abs(error[-1])),
        "estimate_rmse_full_kg": rmse(error),
        "estimate_peak_kg": float(np.max(estimate)),
        "estimate_min_kg": float(np.min(estimate)),
        "last10_mean_estimate_kg": float(np.mean(estimate[last10])),
        "last10_temporal_std_kg": float(np.std(estimate[last10], ddof=1)),
        "last10_mean_signed_error_kg": float(np.mean(error[last10])),
        "last10_rmse_kg": float(np.sqrt(np.mean(error[last10] ** 2))),
        "last10_slope_kg_s": slope,
        "last5_mean_estimate_kg": float(np.mean(estimate[last5])),
        "last5_temporal_std_kg": float(np.std(estimate[last5], ddof=1)),
        "derivative_rms_kg_s": rmse(derivative),
        "derivative_last10_rms_kg_s": float(np.sqrt(np.mean(derivative[last10] ** 2))),
        "x_rmse_m": rmse(x_error),
        "x_iae_m_s": integral(np.abs(x_error)),
        "x_peak_abs_m": float(np.max(np.abs(x_error))),
        "z_rmse_m": rmse(z_error),
        "z_iae_m_s": integral(np.abs(z_error)),
        "z_peak_abs_m": float(np.max(np.abs(z_error))),
        "position_rmse_m": rmse(position_error_norm),
        "position_peak_m": float(np.max(position_error_norm)),
        "candidate_correction_rms": rmse(candidate_norm),
        "candidate_correction_peak": float(np.max(candidate_norm)),
        "applied_correction_rms": rmse(applied_norm),
        "applied_correction_peak": float(np.max(applied_norm)),
        "policy_rms_norm": rmse(np.linalg.norm(policy, axis=1)),
        "target_start_time_s": float(bag["target_time"][0]),
        "target_transition_duration_s": target_duration,
        "target_relative_x_step_m": target_step,
    }
    if abs(truth) > 1.0e-12:
        metrics.update({
            "final_relative_error_percent": 100.0 * error[-1] / truth,
            "last10_relative_bias_percent": 100.0 * np.mean(error[last10]) / truth,
            "peak_overshoot_percent": 100.0 * (np.max(estimate) - truth) / truth,
            "sustained_entry_20_percent_s": sustained_entry(error, 0.20 * abs(truth)),
            "sustained_entry_10_percent_s": sustained_entry(error, 0.10 * abs(truth)),
            "sustained_entry_5_percent_s": sustained_entry(error, 0.05 * abs(truth)),
            "time_within_10_percent_ratio": float(np.mean(np.abs(error) <= 0.10 * abs(truth))),
        })
    else:
        metrics.update({
            "final_relative_error_percent": None,
            "last10_relative_bias_percent": None,
            "peak_overshoot_percent": None,
            "sustained_entry_20_percent_s": None,
            "sustained_entry_10_percent_s": None,
            "sustained_entry_5_percent_s": None,
            "time_within_10_percent_ratio": None,
            "false_estimate_peak_abs_kg": float(np.max(np.abs(estimate))),
            "false_estimate_last10_mean_abs_kg": float(np.mean(np.abs(estimate[last10]))),
        })
    signals.update({
        "estimate": estimate, "truth": np.full(len(GRID), truth),
        "estimation_error": error, "derivative": derivative,
        "estimated_total_mass": signals["estimated_total_mass"],
        "true_total_mass": np.full(len(GRID), 52.13485 + truth),
        "applied_correction_norm": applied_norm,
        "candidate_correction_norm": candidate_norm,
        "desired_x": desired_position[:, 0], "actual_x": actual_position[:, 0],
        "desired_z": desired_position[:, 2], "actual_z": actual_position[:, 2],
        "x_error": x_error, "z_error": z_error,
        "position_error_norm": position_error_norm,
    })
    return {"group": group, "run": run_name, "metrics": metrics,
            "signals": signals, "policy": policy, "contact": policy[:, :12]}


def group_statistics(runs):
    keys = sorted(set.intersection(*(set(run["metrics"]) for run in runs)))
    result = {}
    for key in keys:
        values = [run["metrics"][key] for run in runs]
        if any(value is None for value in values):
            result[key] = {"n": 0, "mean": None, "sample_std": None}
            continue
        array = np.asarray(values, dtype=float)
        result[key] = {
            "n": len(array), "mean": float(np.mean(array)),
            "sample_std": float(np.std(array, ddof=1)) if len(array) > 1 else 0.0,
            "min": float(np.min(array)), "max": float(np.max(array)),
        }
    return result


def mean_signal(runs, name):
    matrix = np.vstack([run["signals"][name] for run in runs])
    return np.mean(matrix, axis=0), np.std(matrix, axis=0, ddof=1)


def plot_campaign(output, grouped):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"nominal": "#4c4c4c", "uncertain": "#d62728",
              "uncertain_off": "#1f77b4"}
    labels = {"nominal": "Nominal plant + Adaptive ON",
              "uncertain": "+10% mass + Adaptive ON",
              "uncertain_off": "+10% mass + Adaptive OFF"}

    figure, axes = plt.subplots(len(GROUPS), 1, figsize=(10, 4 * len(GROUPS)), sharex=True)
    for axis, group in zip(axes, GROUPS):
        for run in grouped[group]:
            axis.plot(GRID, run["signals"]["estimate"], alpha=0.35, linewidth=1.0)
        mean, std = mean_signal(grouped[group], "estimate")
        axis.plot(GRID, mean, color=colors[group], linewidth=2.2,
                  label=labels[group] + " mean")
        axis.fill_between(GRID, mean - std, mean + std, color=colors[group], alpha=0.18,
                          label="± sample std")
        axis.plot(GRID, grouped[group][0]["signals"]["truth"], "k--", linewidth=1.5,
                  label="true payload mass")
        axis.set_ylabel(r"$\Delta\hat m$ (kg)")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    axes[-1].set_xlabel("MPC / simulation time (s)")
    figure.suptitle("PayloadMass1D estimate vs true payload (three repeats)")
    figure.tight_layout()
    figure.savefig(output / "figure_P1_payload_estimate_vs_truth.png", dpi=180)
    plt.close(figure)

    specs = (
        ("estimation_error", "Payload estimation error (kg)",
         "figure_P2_payload_estimation_error.png"),
        ("derivative", "Candidate Eq.(8) derivative (kg/s)",
         "figure_P3_payload_update_rate.png"),
        ("estimated_total_mass", "Estimated total mass (kg)",
         "figure_P4_estimated_total_mass.png"),
        ("applied_correction_norm", "Eq.(10) applied correction norm", 
         "figure_P5_eq10_applied_correction.png"),
        ("x_error", "x tracking error (m)", "figure_P7_x_tracking_error.png"),
        ("z_error", "z tracking error (m)", "figure_P8_z_tracking_error.png"),
        ("position_error_norm", "Position error norm (m)",
         "figure_P9_position_error_norm.png"),
    )
    for signal, ylabel, filename in specs:
        figure, axis = plt.subplots(figsize=(10, 4.8))
        for group in GROUPS:
            mean, std = mean_signal(grouped[group], signal)
            axis.plot(GRID, mean, color=colors[group], linewidth=2.0, label=labels[group])
            axis.fill_between(GRID, mean - std, mean + std,
                              color=colors[group], alpha=0.18)
        if signal == "estimation_error":
            axis.axhline(0.0, color="black", linestyle="--", linewidth=1.0)
        if signal == "estimated_total_mass":
            for group in GROUPS:
                axis.axhline(grouped[group][0]["signals"]["true_total_mass"][0],
                             color=colors[group], linestyle="--", linewidth=1.0)
        axis.set_xlabel("MPC / simulation time (s)")
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)
        axis.legend()
        figure.tight_layout()
        figure.savefig(output / filename, dpi=180)
        plt.close(figure)

    figure, axes = plt.subplots(len(GROUPS), 1, figsize=(10, 3.5 * len(GROUPS)), sharex=True)
    for axis, group in zip(axes, GROUPS):
        desired, _ = mean_signal(grouped[group], "desired_x")
        actual, std = mean_signal(grouped[group], "actual_x")
        axis.plot(GRID, desired, "k--", linewidth=1.5, label="desired x")
        axis.plot(GRID, actual, color=colors[group], linewidth=2.0,
                  label=labels[group] + " actual x")
        axis.fill_between(GRID, actual - std, actual + std,
                          color=colors[group], alpha=0.18)
        axis.set_ylabel("x (m)")
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[-1].set_xlabel("MPC / simulation time (s)")
    figure.suptitle("Frozen 10 cm reference and executed x")
    figure.tight_layout()
    figure.savefig(output / "figure_P6_desired_vs_actual_x.png", dpi=180)
    plt.close(figure)

    figure, axes = plt.subplots(len(GROUPS), 1, figsize=(10, 3.5 * len(GROUPS)), sharex=True)
    for axis, group in zip(axes, GROUPS):
        desired, _ = mean_signal(grouped[group], "desired_z")
        actual, std = mean_signal(grouped[group], "actual_z")
        axis.plot(GRID, desired, "k--", linewidth=1.5, label="desired z")
        axis.plot(GRID, actual, color=colors[group], linewidth=2.0,
                  label=labels[group] + " actual z")
        axis.fill_between(GRID, actual - std, actual + std,
                          color=colors[group], alpha=0.18)
        axis.set_ylabel("z (m)")
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[-1].set_xlabel("MPC / simulation time (s)")
    figure.suptitle("Desired and executed base height")
    figure.tight_layout()
    figure.savefig(output / "figure_P10_desired_vs_actual_z.png", dpi=180)
    plt.close(figure)

    if "uncertain_off" in grouped:
        policy_norms, contact_norms = [], []
        for index in range(len(RUNS)):
            policy_norms.append(np.linalg.norm(
                grouped["uncertain"][index]["policy"] -
                grouped["uncertain_off"][index]["policy"], axis=1))
            contact_norms.append(np.linalg.norm(
                grouped["uncertain"][index]["contact"] -
                grouped["uncertain_off"][index]["contact"], axis=1))
        figure, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        for axis, matrix, ylabel in (
                (axes[0], np.vstack(policy_norms), "ON−OFF full input difference norm"),
                (axes[1], np.vstack(contact_norms), "ON−OFF contact-force difference norm")):
            mean = np.mean(matrix, axis=0)
            std = np.std(matrix, axis=0, ddof=1)
            axis.plot(GRID, mean, color="#9467bd", linewidth=2.0,
                      label="paired mean")
            axis.fill_between(GRID, mean - std, mean + std,
                              color="#9467bd", alpha=0.2,
                              label="± sample std")
            axis.set_ylabel(ylabel)
            axis.grid(True, alpha=0.3)
            axis.legend()
        axes[-1].set_xlabel("MPC / simulation time (s)")
        figure.suptitle("PayloadMass1D changes the MPC policy under identical +10% plant mass")
        figure.tight_layout()
        figure.savefig(output / "figure_P11_policy_contact_on_off_difference.png", dpi=180)
        plt.close(figure)


def main():
    global GROUPS, GRID, EXPECTED_TARGET_START
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records_root", type=Path)
    parser.add_argument("--include-adaptive-off", action="store_true",
                        help="analyze nominal, uncertain ON, and uncertain OFF (nine runs)")
    parser.add_argument("--analysis-directory", default=None,
                        help="output directory name below records_root")
    parser.add_argument("--grid-start", type=float, default=0.50)
    parser.add_argument("--grid-end", type=float, default=30.0)
    parser.add_argument("--expected-target-start", type=float, default=0.50)
    args = parser.parse_args()
    root = args.records_root.resolve()
    if args.grid_end <= args.grid_start or abs(args.grid_end - 30.0) > 1.0e-9:
        raise RuntimeError("PayloadMass1D analysis currently requires grid end=30.0 s")
    GRID = np.arange(args.grid_start, args.grid_end + 1.0e-9, 0.02)
    EXPECTED_TARGET_START = args.expected_target_start
    if args.include_adaptive_off:
        GROUPS = ("nominal", "uncertain", "uncertain_off")
    fairness_name = ("fairness_audit_adaptive_off.json" if args.include_adaptive_off
                     else "fairness_audit.json")
    fairness = load_json(root / fairness_name)
    if fairness.get("passed") is not True:
        raise RuntimeError("fairness audit must pass before unified analysis")
    default_directory = "analysis_adaptive_off" if args.include_adaptive_off else "analysis"
    analysis_dir = root / (args.analysis_directory or default_directory)
    analysis_dir.mkdir(parents=True, exist_ok=True)
    all_runs = [run_analysis(root, group, run_name)
                for group in GROUPS for run_name in RUNS]
    grouped = {group: [run for run in all_runs if run["group"] == group]
               for group in GROUPS}
    stats = {group: group_statistics(runs) for group, runs in grouped.items()}

    metric_names = sorted(set.union(*(set(run["metrics"]) for run in all_runs)))
    with (analysis_dir / "per_run_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["group", "run"] + metric_names)
        writer.writeheader()
        for run in all_runs:
            row = {"group": run["group"], "run": run["run"]}
            row.update(run["metrics"])
            writer.writerow(row)
    with (analysis_dir / "group_statistics.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["group", "metric", "n", "mean", "sample_std", "min", "max"])
        for group in GROUPS:
            for metric, value in sorted(stats[group].items()):
                writer.writerow([group, metric, value.get("n"), value.get("mean"),
                                 value.get("sample_std"), value.get("min"), value.get("max")])

    paired = []
    paired_policy = []
    tracking_names = ("x_rmse_m", "x_iae_m_s", "x_peak_abs_m",
                      "z_rmse_m", "z_iae_m_s", "z_peak_abs_m",
                      "position_rmse_m", "position_peak_m")
    for index, run_name in enumerate(RUNS):
        nominal = grouped["nominal"][index]["metrics"]
        uncertain = grouped["uncertain"][index]["metrics"]
        entry = {"pair": run_name}
        for name in tracking_names:
            entry["uncertain_minus_nominal_" + name] = uncertain[name] - nominal[name]
            if args.include_adaptive_off:
                off = grouped["uncertain_off"][index]["metrics"]
                entry["uncertain_off_minus_nominal_" + name] = off[name] - nominal[name]
                entry["uncertain_on_minus_off_" + name] = uncertain[name] - off[name]
        paired.append(entry)
        if args.include_adaptive_off:
            policy_difference = (grouped["uncertain"][index]["policy"] -
                                 grouped["uncertain_off"][index]["policy"])
            contact_difference = (grouped["uncertain"][index]["contact"] -
                                  grouped["uncertain_off"][index]["contact"])
            paired_policy.append({
                "pair": run_name,
                "full_input_rms_difference": rmse(np.linalg.norm(policy_difference, axis=1)),
                "full_input_peak_difference": float(np.max(
                    np.linalg.norm(policy_difference, axis=1))),
                "contact_force_rms_difference_N": rmse(
                    np.linalg.norm(contact_difference, axis=1)),
                "contact_force_peak_difference_N": float(np.max(
                    np.linalg.norm(contact_difference, axis=1))),
            })

    adaptive_off_comparison = None
    if args.include_adaptive_off:
        adaptive_off_comparison = {}
        for name in tracking_names:
            nominal_mean = stats["nominal"][name]["mean"]
            on_mean = stats["uncertain"][name]["mean"]
            off_mean = stats["uncertain_off"][name]["mean"]
            mismatch_degradation = off_mean - nominal_mean
            on_improvement = off_mean - on_mean
            adaptive_off_comparison[name] = {
                "nominal_mean": nominal_mean,
                "uncertain_adaptive_on_mean": on_mean,
                "uncertain_adaptive_off_mean": off_mean,
                "on_minus_off": on_mean - off_mean,
                "on_improvement_over_off": on_improvement,
                "mismatch_degradation_off_minus_nominal": mismatch_degradation,
                "degradation_recovery_fraction": (
                    on_improvement / mismatch_degradation
                    if mismatch_degradation > 1.0e-12 else None),
                "on_relative_change_vs_off_percent": (
                    100.0 * (on_mean / off_mean - 1.0) if off_mean > 1.0e-12 else None),
            }
        for name in ("full_input_rms_difference", "full_input_peak_difference",
                     "contact_force_rms_difference_N", "contact_force_peak_difference_N"):
            values = np.asarray([row[name] for row in paired_policy])
            adaptive_off_comparison[name] = {
                "n": len(values), "mean": float(np.mean(values)),
                "sample_std": float(np.std(values, ddof=1)),
                "min": float(np.min(values)), "max": float(np.max(values)),
            }

    summary = {
        "passed": True,
        "scope": ("nine-run PayloadMass1D Adaptive-ON/OFF comparison" if args.include_adaptive_off
                  else "formal three-repeat PayloadMass1D convergence/repeatability characterization"),
        "common_time_grid": {"start_s": float(GRID[0]), "end_s": float(GRID[-1]),
                             "dt_s": 0.02, "sample_count": len(GRID)},
        "group_statistics": stats,
        "paired_tracking_differences": paired,
        "paired_policy_differences": paired_policy,
        "adaptive_off_comparison": adaptive_off_comparison,
        "scientific_boundary": (
            "This campaign characterizes scalar payload-mass estimation under one animal_dummy "
            "target and +10% mass mismatch; it does not prove general payload identification or "
            "the full paper theory."),
    }
    (analysis_dir / "payload_mass_campaign_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    plot_campaign(analysis_dir, grouped)

    nominal = stats["nominal"]
    uncertain = stats["uncertain"]
    uncertain_off = stats.get("uncertain_off")
    with (analysis_dir / "payload_mass_campaign_report.md").open("w") as stream:
        stream.write("# PayloadMass1D formal paired campaign\n\n")
        if args.include_adaptive_off:
            stream.write("Nine runs were analyzed on a common {:.2f}--{:.2f} s grid: ".format(
                         GRID[0], GRID[-1]) +
                         "Nominal-ON ×3, +10% mass ON ×3, and +10% mass OFF ×3.\n\n")
        else:
            stream.write("Six runs were analyzed on a common {:.2f}--{:.2f} s grid: ".format(
                         GRID[0], GRID[-1]) +
                         "Nominal ×3 and +10% mass ×3.\n\n")
        stream.write("## Direct mass-estimation result\n\n")
        stream.write("- True added payload: {:.6f} kg.\n".format(
            uncertain["true_payload_delta_kg"]["mean"]))
        stream.write("- Uncertain 30 s estimate: {:.6f} ± {:.6f} kg.\n".format(
            uncertain["estimate_at_30s_kg"]["mean"],
            uncertain["estimate_at_30s_kg"]["sample_std"]))
        stream.write("- Uncertain final relative error: {:.3f} ± {:.3f} %.\n".format(
            uncertain["final_relative_error_percent"]["mean"],
            uncertain["final_relative_error_percent"]["sample_std"]))
        stream.write("- Uncertain last-10-s estimate: {:.6f} ± {:.6f} kg across runs.\n".format(
            uncertain["last10_mean_estimate_kg"]["mean"],
            uncertain["last10_mean_estimate_kg"]["sample_std"]))
        stream.write("- Nominal 30 s false estimate: {:.6f} ± {:.6f} kg.\n".format(
            nominal["estimate_at_30s_kg"]["mean"],
            nominal["estimate_at_30s_kg"]["sample_std"]))
        stream.write("\n## Tracking context\n\n")
        stream.write("- Nominal x RMSE: {:.6f} ± {:.6f} m.\n".format(
            nominal["x_rmse_m"]["mean"], nominal["x_rmse_m"]["sample_std"]))
        stream.write("- Uncertain x RMSE: {:.6f} ± {:.6f} m.\n".format(
            uncertain["x_rmse_m"]["mean"], uncertain["x_rmse_m"]["sample_std"]))
        if args.include_adaptive_off:
            stream.write("- Uncertain Adaptive-OFF x RMSE: {:.6f} ± {:.6f} m.\n".format(
                uncertain_off["x_rmse_m"]["mean"],
                uncertain_off["x_rmse_m"]["sample_std"]))
            comparison = adaptive_off_comparison["x_rmse_m"]
            stream.write("- Adaptive-ON relative x-RMSE change vs OFF: {:.3f} %.\n".format(
                comparison["on_relative_change_vs_off_percent"]))
            recovery = comparison["degradation_recovery_fraction"]
            stream.write("- Recovered fraction of OFF mismatch degradation: {}.\n".format(
                "undefined" if recovery is None else "{:.3f}".format(recovery)))
        stream.write("\n## Conclusion boundary\n\n")
        stream.write(summary["scientific_boundary"] + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
