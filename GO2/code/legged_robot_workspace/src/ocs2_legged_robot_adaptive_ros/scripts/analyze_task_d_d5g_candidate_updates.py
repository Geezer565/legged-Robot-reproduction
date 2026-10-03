#!/usr/bin/env python3
"""D5G offline screening of candidate 10D adaptive update laws.

This analysis reuses the frozen, formally audited D5B nominal/uncertain walking
bags.  It does not replay the controller and cannot predict the counterfactual
closed-loop trajectory that an online estimator change would create.
"""

import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


PACKAGE = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


D5F = load_module("d5f_for_d5g", PACKAGE / "scripts" /
                  "analyze_task_d_d5f_gait_phase_covariance.py")
D5I = load_module("d5i_for_d5g", PACKAGE / "scripts" /
                  "analyze_task_d_d5_identifiability.py")
ESTIMATE = "/task_d/payload_inertial_10d_v3_sample"
OBSERVATION = "/legged_robot_mpc_observation"
ESTIMATE_LABEL = "payload_inertial_10d_v3_118_fields"
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz",
              "Jyy", "Jyz", "Jzz")
GROUP_DIRS = {
    "nominal": "nominal_walk_target",
    "uncertain": "uncertain_walk_target",
}
LEAKAGE_VALUES = (0.05, 0.1, 0.25, 0.5, 1.0)
DEADZONE_QUANTILES = (0.50, 0.75, 0.90, 0.95, 0.99)
PHASE_BINS = 24
MINIMUM_STEADY_START = 2.2
EVALUATION_COMPLETE_CYCLES = 11


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sample_std(values, axis=0):
    values = np.asarray(values, dtype=float)
    return np.std(values, axis=axis, ddof=1) if values.shape[axis] > 1 else np.full(
        values.shape[1:] if axis == 0 else (), np.nan)


def weighted_quantile(values, weights, quantile):
    values = np.asarray(values, dtype=float)
    weights = np.asarray(weights, dtype=float)
    if values.ndim != 1 or weights.shape != values.shape or len(values) == 0:
        raise ValueError("weighted quantile inputs must be nonempty 1D arrays")
    if np.any(weights <= 0.0) or not 0.0 <= quantile <= 1.0:
        raise ValueError("weights must be positive and quantile must be in [0,1]")
    order = np.argsort(values)
    sorted_values, sorted_weights = values[order], weights[order]
    centers = (np.cumsum(sorted_weights) - 0.5 * sorted_weights) / np.sum(sorted_weights)
    return float(np.interp(quantile, centers, sorted_values,
                           left=sorted_values[0], right=sorted_values[-1]))


def load_run(root, group, run_index, fairness):
    run_name = "run_{:02d}".format(run_index)
    relative = GROUP_DIRS[group] + "/" + run_name
    path = root / "formal" / relative
    bag_path = path / "task_d_d5b.bag"
    config = json.loads((path / "effective_config.json").read_text())
    validation = json.loads((path / "task_d_d5b_run_summary.json").read_text())
    expected_hash = fairness["artifact_sha256"][relative]["task_d_d5b.bag"]
    actual_hash = sha256(bag_path)
    if actual_hash != expected_hash:
        raise ValueError("frozen D5B bag hash mismatch: " + relative)
    if not validation.get("formal_eligibility"):
        raise ValueError("D5B run is not formally eligible: " + relative)
    expected_group = GROUP_DIRS[group]
    if config.get("group") != expected_group:
        raise ValueError("D5B group identity mismatch: " + relative)

    estimates, observations, labels = [], [], set()
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, message, _ in bag.read_messages(topics=[ESTIMATE, OBSERVATION]):
            if topic == ESTIMATE:
                labels.add(message.layout.dim[0].label if message.layout.dim else "")
                estimates.append(np.asarray(message.data, dtype=float))
            else:
                observations.append((float(message.time), int(message.mode),
                                     np.asarray(message.state.value, dtype=float),
                                     np.asarray(message.input.value, dtype=float)))
    estimate = np.asarray(sorted(estimates, key=lambda row: row[0]), dtype=float)
    observations = [item for _, item in sorted(
        {item[0]: item for item in observations}.items())]
    if estimate.ndim != 2 or estimate.shape[1] != 118 or labels != {ESTIMATE_LABEL}:
        raise ValueError("invalid D5B v3 schema: " + relative)
    if not np.all(np.isfinite(estimate)) or not observations:
        raise ValueError("nonfinite or missing D5B data: " + relative)
    if not np.all(np.diff(estimate[:, 0]) > 0.0):
        raise ValueError("D5B v3 time is not strictly increasing: " + relative)
    regressor = estimate[:, 48:108].reshape(-1, 6, 10)
    sigma = estimate[:, 42:48]
    drive = np.einsum("nij,ni->nj", regressor, sigma)
    derivative_error = float(np.max(np.abs(drive - estimate[:, 32:42])))
    if derivative_error > 1.0e-12:
        raise ValueError("Eq.(8) drive reconstruction failed: " + relative)
    available_cycles = D5F.complete_steady_cycles(observations, MINIMUM_STEADY_START)
    if len(available_cycles) < EVALUATION_COMPLETE_CYCLES:
        raise ValueError("insufficient complete steady cycles: " + relative)
    cycles = available_cycles[:EVALUATION_COMPLETE_CYCLES]
    selected = extract_complete_cycle_intervals(estimate[:, 0], drive, sigma, cycles,
                                                PHASE_BINS)
    return {
        "group": group, "run_index": run_index, "path": path,
        "bag": bag_path, "bag_sha256": actual_hash, "config": config,
        "validation": validation, "derivative_error": derivative_error,
        "cycles": cycles, **selected,
    }


def extract_complete_cycle_intervals(time, drive, sigma, cycles, phase_bins):
    all_time, all_dt, all_drive, all_sigma, all_phase, all_cycle = [], [], [], [], [], []
    elapsed = 0.0
    for cycle_index, cycle in enumerate(cycles):
        selected_time, selected_drive, dt = D5F.interval_samples(
            time, drive, cycle["start"], cycle["end"])
        sigma_time, selected_sigma, sigma_dt = D5F.interval_samples(
            time, sigma, cycle["start"], cycle["end"])
        if not np.allclose(selected_time, sigma_time, rtol=0.0, atol=1.0e-12):
            raise ValueError("drive/sigma timestamps differ")
        if not np.allclose(dt, sigma_dt, rtol=0.0, atol=1.0e-12):
            raise ValueError("drive/sigma interval weights differ")
        phase = np.minimum((selected_time - cycle["start"]) / cycle["duration"],
                           np.nextafter(1.0, 0.0))
        phase_index = np.minimum((phase * phase_bins).astype(int), phase_bins - 1)
        all_time.append(elapsed + np.cumsum(dt))
        all_dt.append(dt); all_drive.append(selected_drive); all_sigma.append(selected_sigma)
        all_phase.append(phase_index)
        all_cycle.append(np.full(len(dt), cycle_index, dtype=int))
        elapsed += cycle["duration"]
    result = {
        "elapsed": np.concatenate(all_time), "dt": np.concatenate(all_dt),
        "drive": np.vstack(all_drive), "sigma": np.vstack(all_sigma),
        "phase": np.concatenate(all_phase), "cycle_index": np.concatenate(all_cycle),
    }
    if abs(np.sum(result["dt"]) - elapsed) > 1.0e-8:
        raise ValueError("complete-cycle duration does not close")
    return result


def phase_baseline(training_runs, phase_bins=PHASE_BINS):
    numerator = np.zeros((phase_bins, 10))
    duration = np.zeros(phase_bins)
    for run in training_runs:
        for index in range(phase_bins):
            mask = run["phase"] == index
            numerator[index] += np.sum(run["dt"][mask, None] * run["drive"][mask], axis=0)
            duration[index] += np.sum(run["dt"][mask])
    if np.any(duration <= 0.0):
        raise ValueError("nominal phase baseline has an empty bin")
    return numerator / duration[:, None], duration


def nominal_sigma_threshold(training_runs, quantile):
    values = np.concatenate([np.linalg.norm(run["sigma"], axis=1)
                             for run in training_runs])
    weights = np.concatenate([run["dt"] for run in training_runs])
    return weighted_quantile(values, weights, quantile)


def replay_update(dt, drive, sigma_norm, phase, strategy, value=0.0,
                  baseline=None, threshold=None):
    """Explicit-Euler counterfactual update on one frozen state/input trace."""
    dt = np.asarray(dt, dtype=float)
    drive = np.asarray(drive, dtype=float)
    sigma_norm = np.asarray(sigma_norm, dtype=float)
    phase = np.asarray(phase, dtype=int)
    if drive.shape != (len(dt), 10) or sigma_norm.shape != dt.shape:
        raise ValueError("invalid replay arrays")
    effective = drive.copy()
    active = np.ones(len(dt), dtype=bool)
    if strategy == "phase_baseline":
        baseline = np.asarray(baseline, dtype=float)
        effective -= baseline[phase]
    elif strategy == "deadzone":
        if threshold is None:
            raise ValueError("dead-zone threshold is required")
        active = sigma_norm > threshold
        effective[~active] = 0.0
    elif strategy not in ("original", "leakage"):
        raise ValueError("unknown replay strategy: " + strategy)
    estimate = np.zeros((len(dt) + 1, 10))
    for index in range(len(dt)):
        rate = effective[index]
        if strategy == "leakage":
            rate = rate - value * estimate[index]
        estimate[index + 1] = estimate[index] + dt[index] * rate
    active_fraction = float(np.sum(dt[active]) / np.sum(dt))
    return estimate[1:], effective, active_fraction


def candidate_definitions():
    result = [{"name": "original", "strategy": "original", "value": 0.0}]
    result.extend({"name": "leakage_{:.2f}".format(value),
                   "strategy": "leakage", "value": value}
                  for value in LEAKAGE_VALUES)
    result.extend({"name": "deadzone_q{:02d}".format(int(round(100 * value))),
                   "strategy": "deadzone", "value": value}
                  for value in DEADZONE_QUANTILES)
    result.append({"name": "phase_baseline", "strategy": "phase_baseline", "value": 0.0})
    return result


def training_nominal_runs(runs, group, run_index):
    nominal = runs["nominal"]
    if group == "nominal":
        selected = [run for run in nominal if run["run_index"] != run_index]
        if len(selected) != len(nominal) - 1:
            raise ValueError("leave-one-run-out nominal training split failed")
        return selected, "leave_one_nominal_run_out"
    return nominal, "all_nominal_runs"


def replay_all(runs, candidates):
    results = []
    phase_models = {}
    threshold_models = {}
    for group in ("nominal", "uncertain"):
        for run in runs[group]:
            training, training_rule = training_nominal_runs(runs, group, run["run_index"])
            baseline, baseline_duration = phase_baseline(training)
            phase_models[(group, run["run_index"])] = (baseline, baseline_duration)
            thresholds = {candidate["value"]: nominal_sigma_threshold(
                training, candidate["value"]) for candidate in candidates
                if candidate["strategy"] == "deadzone"}
            threshold_models[(group, run["run_index"])] = thresholds
            sigma_norm = np.linalg.norm(run["sigma"], axis=1)
            for candidate in candidates:
                threshold = (thresholds[candidate["value"]]
                             if candidate["strategy"] == "deadzone" else None)
                trajectory, effective, active_fraction = replay_update(
                    run["dt"], run["drive"], sigma_norm, run["phase"],
                    candidate["strategy"], candidate["value"], baseline, threshold)
                results.append({
                    "group": group, "run_index": run["run_index"],
                    "candidate": candidate["name"], "strategy": candidate["strategy"],
                    "value": candidate["value"], "training_rule": training_rule,
                    "deadzone_threshold": threshold, "active_time_fraction": active_fraction,
                    "terminal": trajectory[-1], "trajectory": trajectory,
                    "effective_drive": effective, "elapsed": run["elapsed"],
                    "duration": float(np.sum(run["dt"])),
                    "complete_cycles": len(run["cycles"]),
                })
    return results, phase_models, threshold_models


def normalized(vector, scales):
    return np.asarray(vector, dtype=float) / np.asarray(scales, dtype=float)


def cosine(left, right):
    left, right = np.asarray(left, dtype=float), np.asarray(right, dtype=float)
    denominator = np.linalg.norm(left) * np.linalg.norm(right)
    return float(np.dot(left, right) / denominator) if denominator > 0.0 else math.nan


def aggregate_results(results, candidates, truth, scales):
    per_run, comparisons, components = [], [], []
    truth_scaled = normalized(truth, scales)
    original_nominal_norm = None
    original_contrast_norm = None
    original_error = None
    cached = {}
    for candidate in candidates:
        name = candidate["name"]
        endpoints = {}
        for group in ("nominal", "uncertain"):
            selected = sorted([item for item in results
                               if item["candidate"] == name and item["group"] == group],
                              key=lambda item: item["run_index"])
            endpoints[group] = np.vstack([item["terminal"] for item in selected])
            for item in selected:
                row = {
                    "candidate": name, "strategy": candidate["strategy"],
                    "candidate_value": candidate["value"], "group": group,
                    "run_index": item["run_index"], "duration_s": item["duration"],
                    "complete_cycles": item["complete_cycles"],
                    "training_rule": item["training_rule"],
                    "deadzone_threshold": item["deadzone_threshold"],
                    "active_time_fraction": item["active_time_fraction"],
                    "terminal_scaled_norm": float(np.linalg.norm(
                        normalized(item["terminal"], scales))),
                }
                row.update({"terminal_" + parameter: item["terminal"][index]
                            for index, parameter in enumerate(PARAMETERS)})
                per_run.append(row)
        nominal_norms = np.linalg.norm(normalized(endpoints["nominal"], scales), axis=1)
        paired_contrast = endpoints["uncertain"] - endpoints["nominal"]
        contrast_mean = np.mean(paired_contrast, axis=0)
        contrast_scaled = normalized(contrast_mean, scales)
        error_scaled = contrast_scaled - truth_scaled
        contrast_norm = float(np.linalg.norm(contrast_scaled))
        contrast_error = float(np.sqrt(np.mean(error_scaled ** 2)))
        if name == "original":
            original_nominal_norm = float(np.mean(nominal_norms))
            original_contrast_norm = contrast_norm
            original_error = contrast_error
        cached[name] = {
            "nominal_norms": nominal_norms, "contrast_mean": contrast_mean,
            "contrast_std": sample_std(paired_contrast), "contrast_norm": contrast_norm,
            "contrast_error": contrast_error,
            "cosine": cosine(contrast_scaled, truth_scaled),
            "uncertain_mean": np.mean(endpoints["uncertain"], axis=0),
            "nominal_mean": np.mean(endpoints["nominal"], axis=0),
        }
    for candidate in candidates:
        name, item = candidate["name"], cached[candidate["name"]]
        nominal_mean = float(np.mean(item["nominal_norms"]))
        row = {
            "candidate": name, "strategy": candidate["strategy"],
            "candidate_value": candidate["value"],
            "nominal_terminal_scaled_norm_mean": nominal_mean,
            "nominal_terminal_scaled_norm_sample_std": float(sample_std(
                item["nominal_norms"][:, None])[0]),
            "nominal_suppression_fraction_vs_original":
                1.0 - nominal_mean / original_nominal_norm,
            "payload_contrast_scaled_norm": item["contrast_norm"],
            "payload_contrast_norm_retention_vs_original":
                item["contrast_norm"] / original_contrast_norm,
            "payload_contrast_scaled_rmse_vs_truth": item["contrast_error"],
            "payload_contrast_error_ratio_vs_original": item["contrast_error"] / original_error,
            "payload_contrast_cosine_vs_truth": item["cosine"],
        }
        comparisons.append(row)
        for index, parameter in enumerate(PARAMETERS):
            truth_value = truth[index]
            contrast_value = item["contrast_mean"][index]
            components.append({
                "candidate": name, "strategy": candidate["strategy"],
                "candidate_value": candidate["value"], "parameter": parameter,
                "nominal_terminal_mean": item["nominal_mean"][index],
                "uncertain_terminal_mean": item["uncertain_mean"][index],
                "paired_payload_contrast_mean": contrast_value,
                "paired_payload_contrast_sample_std": item["contrast_std"][index],
                "pi_star": truth_value, "signed_error": contrast_value - truth_value,
                "ratio_to_truth": (contrast_value / truth_value
                                   if abs(truth_value) > 1.0e-12 else math.nan),
            })
    for row in comparisons:
        row["pareto_nondominated_nominal_drift_and_truth_error"] = not any(
            other["candidate"] != row["candidate"] and
            other["nominal_terminal_scaled_norm_mean"] <= row["nominal_terminal_scaled_norm_mean"] and
            other["payload_contrast_scaled_rmse_vs_truth"] <= row["payload_contrast_scaled_rmse_vs_truth"] and
            (other["nominal_terminal_scaled_norm_mean"] < row["nominal_terminal_scaled_norm_mean"] or
             other["payload_contrast_scaled_rmse_vs_truth"] < row["payload_contrast_scaled_rmse_vs_truth"])
            for other in comparisons)
    return per_run, comparisons, components


def write_csv(path, rows):
    if not rows:
        raise ValueError("cannot write an empty CSV")
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def plot_results(output, comparisons, components, results, candidates, truth):
    colors = {"original": "black", "leakage": "#1f77b4",
              "deadzone": "#ff7f0e", "phase_baseline": "#2ca02c"}
    fig, axis = plt.subplots(figsize=(11, 7))
    for row in comparisons:
        axis.scatter(row["nominal_terminal_scaled_norm_mean"],
                     row["payload_contrast_scaled_rmse_vs_truth"], s=60,
                     color=colors[row["strategy"]])
        axis.annotate(row["candidate"],
                      (row["nominal_terminal_scaled_norm_mean"],
                       row["payload_contrast_scaled_rmse_vs_truth"]),
                      xytext=(4, 4), textcoords="offset points", fontsize=8)
    axis.set_xlabel("Nominal terminal dimensionless norm (lower is better)")
    axis.set_ylabel("Payload contrast dimensionless RMSE vs pi_star (lower is better)")
    axis.grid(True, alpha=0.3)
    axis.set_title("D5G frozen-trace mechanism screen (not closed-loop performance)")
    fig.tight_layout(); fig.savefig(output / "figure_D5G_01_tradeoff.png", dpi=180); plt.close(fig)

    representative = ["original", "leakage_0.50", "deadzone_q95", "phase_baseline"]
    focus = (0, 1, 2, 3, 4, 7)
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    x = np.arange(len(representative)); width = 0.24
    for plot, parameter_index in zip(axes.ravel(), focus):
        values = []
        for candidate in representative:
            row = next(item for item in components
                       if item["candidate"] == candidate and
                       item["parameter"] == PARAMETERS[parameter_index])
            values.append(row)
        plot.bar(x - width, [item["nominal_terminal_mean"] for item in values],
                 width, label="Nominal")
        plot.bar(x, [item["uncertain_terminal_mean"] for item in values],
                 width, label="Uncertain")
        plot.bar(x + width, [item["paired_payload_contrast_mean"] for item in values],
                 width, label="U-N contrast")
        plot.axhline(truth[parameter_index], color="black", linestyle="--", label="pi_star")
        plot.set_title(PARAMETERS[parameter_index]); plot.grid(True, axis="y", alpha=0.3)
        plot.set_xticks(x); plot.set_xticklabels(("original", "leak .5", "DZ q95", "phase"), rotation=20)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("D5G terminal estimates and paired payload contrasts", y=0.995)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    fig.savefig(output / "figure_D5G_02_component_contrasts.png", dpi=180); plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=False)
    for plot, parameter_index in zip(axes, (3, 4, 7)):
        for candidate, style, color in (("original", "--", "#1f77b4"),
                                        ("phase_baseline", "-", "#d62728")):
            selected = [item for item in results if item["group"] == "nominal" and
                        item["candidate"] == candidate]
            for run in selected:
                plot.plot(run["elapsed"], run["trajectory"][:, parameter_index],
                          style, color=color, alpha=0.55,
                          label=candidate if run["run_index"] == 1 else None)
        plot.set_ylabel(PARAMETERS[parameter_index]); plot.grid(True, alpha=0.3); plot.legend()
    axes[-1].set_xlabel("elapsed complete-cycle time [s]")
    fig.suptitle("D5G Nominal drift: original versus LOO gait-phase baseline")
    fig.tight_layout(); fig.savefig(output / "figure_D5G_03_nominal_trajectories.png", dpi=180); plt.close(fig)


def analyze(root, output):
    root, output = Path(root).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    fairness_path = root / "formal_fairness_audit.json"
    fairness = json.loads(fairness_path.read_text())
    required_fairness = (fairness.get("passed") and
                         fairness.get("checks", {}).get("formal_eligibility") and
                         fairness.get("checks", {}).get("normalized_configs_identical") and
                         fairness.get("checks", {}).get("source_snapshots_identical"))
    if not required_fairness:
        raise ValueError("D5B formal fairness audit is not eligible for D5G")
    runs = {group: [load_run(root, group, index, fairness) for index in (1, 2, 3)]
            for group in ("nominal", "uncertain")}
    truths = np.vstack([np.asarray(run["validation"]["pi_star"], dtype=float)
                        for run in runs["uncertain"]])
    if not np.allclose(truths, truths[0], rtol=0.0, atol=1.0e-12):
        raise ValueError("uncertain runs do not share one pi_star")
    truth = truths[0]
    scales = D5I.payload_coordinate_scales(truth)
    candidates = candidate_definitions()
    replay, phase_models, thresholds = replay_all(runs, candidates)
    per_run, comparisons, components = aggregate_results(
        replay, candidates, truth, scales)
    write_csv(output / "candidate_metrics_per_run.csv", per_run)
    write_csv(output / "candidate_comparison_summary.csv", comparisons)
    write_csv(output / "candidate_contrast_components.csv", components)

    baseline_all, baseline_duration = phase_baseline(runs["nominal"])
    phase_rows = []
    for phase in range(PHASE_BINS):
        for index, parameter in enumerate(PARAMETERS):
            phase_rows.append({
                "phase_bin": phase, "phase_start": phase / PHASE_BINS,
                "phase_end": (phase + 1) / PHASE_BINS,
                "training_duration_s": baseline_duration[phase],
                "parameter": parameter, "nominal_baseline_drive_rate": baseline_all[phase, index],
            })
    write_csv(output / "nominal_gait_phase_baseline.csv", phase_rows)
    plot_results(output, comparisons, components, replay, candidates, truth)

    input_bags = {group: [{"run_index": run["run_index"], "path": str(run["bag"]),
                           "sha256": run["bag_sha256"],
                           "complete_cycles": len(run["cycles"]),
                           "steady_duration_s": float(np.sum(run["dt"]))}
                          for run in runs[group]] for group in runs}
    best_nominal = min(comparisons, key=lambda row: row["nominal_terminal_scaled_norm_mean"])
    best_truth = min(comparisons, key=lambda row: row["payload_contrast_scaled_rmse_vs_truth"])
    original = next(row for row in comparisons if row["candidate"] == "original")
    strictly_improves_both = [row["candidate"] for row in comparisons
                              if row["candidate"] != "original" and
                              row["nominal_terminal_scaled_norm_mean"] <
                              original["nominal_terminal_scaled_norm_mean"] and
                              row["payload_contrast_scaled_rmse_vs_truth"] <
                              original["payload_contrast_scaled_rmse_vs_truth"]]
    materially_improves_both = [row["candidate"] for row in comparisons
                                if row["candidate"] != "original" and
                                row["nominal_suppression_fraction_vs_original"] >= 0.01 and
                                row["payload_contrast_error_ratio_vs_original"] <= 0.99]
    preserves_contrast_and_suppresses_bias = [
        row["candidate"] for row in comparisons
        if row["candidate"] != "original" and
        row["nominal_suppression_fraction_vs_original"] >= 0.5 and
        row["payload_contrast_error_ratio_vs_original"] <= 1.01]
    passed = bool(len(replay) == len(candidates) * 6 and
                  all(run["derivative_error"] <= 1.0e-12
                      for group in runs.values() for run in group) and
                  all(len(run["cycles"]) >= 8 for group in runs.values() for run in group) and
                  all(np.all(np.isfinite(item["trajectory"])) for item in replay))
    summary = {
        "schema_version": 1, "stage": "D5G_offline_candidate_update_screen",
        "passed": passed,
        "scope": "frozen D5B traces; estimator/controller/plant were not rerun or modified",
        "data_audit": {
            "d5b_formal_fairness_passed": bool(required_fairness),
            "fairness_audit_path": str(fairness_path),
            "fairness_audit_sha256": sha256(fairness_path),
            "groups": ["nominal_walk_target", "uncertain_walk_target"],
            "runs_per_group": 3, "input_bags": input_bags,
            "maximum_eq8_reconstruction_error": max(
                run["derivative_error"] for group in runs.values() for run in group),
        },
        "evaluation_protocol": {
            "steady_cycle_rule": "first 11 complete executed 13->7->14->11->13 cycles, start >= 2.2 s",
            "initial_candidate_estimate": [0.0] * 10,
            "integration": "right-endpoint explicit Euler on raw v3 diagnostic timestamps",
            "nominal_training": "leave-one-run-out for each held-out nominal evaluation",
            "uncertain_training": "all three nominal runs only",
            "deadzone": "hard gate: update=0 when ||sigma|| <= nominal weighted quantile",
            "phase_baseline": "subtract duration-weighted nominal mean Y^T sigma in 24 normalized gait-phase bins",
            "leakage": "update = Y^T sigma - leakage_rate*pi",
            "candidate_values_are_controller_tuning": False,
        },
        "payload_truth": truth.tolist(), "payload_coordinate_scales": scales.tolist(),
        "candidate_comparisons": comparisons,
        "screening_findings": {
            "best_nominal_drift_candidate": best_nominal["candidate"],
            "numerically_best_payload_truth_error_candidate": best_truth["candidate"],
            "candidates_strictly_improving_both_offline_objectives_vs_original": strictly_improves_both,
            "candidates_materially_improving_both_by_at_least_1pct": materially_improves_both,
            "candidates_suppressing_nominal_at_least_50pct_and_preserving_truth_error_within_1pct":
                preserves_contrast_and_suppresses_bias,
        },
        "scientific_limit": (
            "Frozen-trace replay holds sigma and Y fixed although an online law would change "
            "Eq.(10), policy, state, and later regressors. Results screen mechanisms only; "
            "they are not closed-loop tracking or stability evidence."),
    }
    (output / "stage_d5g_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")

    focus_names = ("m", "hx", "hy", "hz", "Jxx", "Jyy")
    table_rows = []
    for row in comparisons:
        table_rows.append(
            "| {candidate} | {nominal_terminal_scaled_norm_mean:.5g} | "
            "{nominal_suppression_fraction_vs_original:.1%} | "
            "{payload_contrast_scaled_rmse_vs_truth:.5g} | "
            "{payload_contrast_error_ratio_vs_original:.3f} | "
            "{payload_contrast_cosine_vs_truth:.4f} | {pareto_nondominated_nominal_drift_and_truth_error} |".format(**row))
    component_rows = []
    representative = ("original", "leakage_0.50", "deadzone_q95", "phase_baseline")
    for candidate in representative:
        for parameter in focus_names:
            row = next(item for item in components
                       if item["candidate"] == candidate and item["parameter"] == parameter)
            component_rows.append("| {} | {} | {:.6g} | {:.6g} | {:.6g} |".format(
                candidate, parameter, row["nominal_terminal_mean"],
                row["paired_payload_contrast_mean"], row["pi_star"]))
    report = """# Stage D5G — Offline candidate update-law screening

## Data and scope

只读取 D5B 已冻结且 formal fairness 通过的 `nominal_walk_target` 与
`uncertain_walk_target`，每组 3 次。所有候选从零开始，只在 2.2 s 后完整执行步态周期上重积分。
为避免 endpoint exposure 不同，每条 run 固定取前 11 个完整周期（13.2 s）。
没有修改或重跑在线 estimator、Eq.(10)、MPC、plant；候选数值是 sensitivity sweep，**不是已选定的 controller 参数**。

Nominal 的 dead-zone threshold 和 gait-phase baseline 使用 leave-one-run-out；
Uncertain 只使用三条 Nominal run 训练，不使用 Uncertain truth 或自身样本训练消偏。

## Two-objective screen

| candidate | Nominal scaled terminal norm | suppression vs original | payload contrast scaled RMSE vs pi_star | error/original | cosine vs pi_star | Pareto |
|---|---:|---:|---:|---:|---:|---|
{comparison_rows}

坐标在比较前按本次 payload 的 mass、first-moment、inertia 物理尺度无量纲化。
`payload contrast = mean(Uncertain - paired Nominal)`，不是把 Uncertain 的绝对估计直接当 payload。

## Representative component results

| candidate | parameter | Nominal terminal mean | paired U-N contrast mean | pi_star |
|---|---|---:|---:|---:|
{component_rows}

严格数值上双目标同时优于 original 的候选：`{strictly_improves}`；
两个目标都至少改善 1% 的候选：`{materially_improves}`。
Nominal drift 最低的是 `{best_nominal}`；payload truth scaled error 数值最低的是 `{best_truth}`。
后者必须结合 `error/original` 判断是否只是数值上近乎不变，不能写成参数重建得到实质改善。

`phase_baseline` 的组均值 payload contrast 与 original 完全相同，是因为两组减去同一个
Nominal phase template，而三折 LOO Nominal residual 的组均值代数上为零；这不是新的
payload information，也不是 `pi_hat` 更接近 `pi_star` 的证据。它真正提供的离线证据是：
对 held-out Nominal 单条 run，残余 terminal norm 仍从 6.416 降到平均 0.00671。

## Interpretation boundary and next gate

这一步只能回答：在**同一批冻结的 Y(t)、sigma(t)** 上，哪种数学机制值得进入在线 smoke test。
真实在线修改会经 Eq.(10) 改变 MPC policy、state、sigma 和后续 regressor，因此不能由本报告宣称 tracking 改善、稳定性或参数收敛。
若选定候选进入下一阶段，必须新增可关闭配置、默认保持旧 baseline，并按 Nominal/Uncertain 配对重新执行 smoke 与 >=3 次正式闭环实验。
""".format(comparison_rows="\n".join(table_rows),
           component_rows="\n".join(component_rows),
           strictly_improves=(", ".join(strictly_improves_both)
                              if strictly_improves_both else "none"),
           materially_improves=(", ".join(materially_improves_both)
                                if materially_improves_both else "none"),
           best_nominal=best_nominal["candidate"], best_truth=best_truth["candidate"])
    (output / "stage_d5g_report.md").write_text(report)

    artifacts = []
    for path in sorted(output.iterdir()):
        if path.is_file() and path.name != "stage_d5g_artifact_manifest.json":
            artifacts.append({"path": path.name, "bytes": path.stat().st_size,
                              "sha256": sha256(path)})
    (output / "stage_d5g_artifact_manifest.json").write_text(json.dumps(
        {"schema_version": 1, "passed": passed, "artifacts": artifacts},
        indent=2, sort_keys=True) + "\n")
    print(json.dumps({"passed": passed,
                      "screening_findings": summary["screening_findings"],
                      "candidate_comparisons": comparisons}, indent=2))
    return 0 if passed else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("d5b_root")
    parser.add_argument("output_directory")
    args = parser.parse_args()
    return analyze(args.d5b_root, args.output_directory)


if __name__ == "__main__":
    raise SystemExit(main())
