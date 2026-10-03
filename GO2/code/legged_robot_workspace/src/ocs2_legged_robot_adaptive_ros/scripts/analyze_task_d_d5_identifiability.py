#!/usr/bin/env python3
"""Offline finite-horizon identifiability audit of the frozen D4 campaign.

This script reads the 6x10 PayloadInertial10D regressor from the D4 v3 bag
diagnostic and combines it with the already materialized D4 common-grid CSV.
It never changes or replays the controller and it does not claim parameter
convergence from a finite-horizon rank test.
"""

import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


ESTIMATE_TOPIC = "/task_d/payload_inertial_10d_v3_sample"
ESTIMATE_LABEL = "payload_inertial_10d_v3_118_fields"
GROUPS = ("nominal", "uncertain_off", "uncertain_adaptive")
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy", "Jyz", "Jzz")
RELATIVE_THRESHOLDS = (1.0e-2, 1.0e-3, 1.0e-6)
PRACTICAL_THRESHOLD = 1.0e-3
SLIDING_WINDOW_S = 2.0
SLIDING_STEP_S = 0.2


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def trapezoid_weights(time):
    """Quadrature weights satisfying sum(w*f) == trapezoid integral."""
    time = np.asarray(time, dtype=float)
    if len(time) < 2 or not np.all(np.diff(time) > 0.0):
        raise ValueError("time must contain at least two strictly increasing samples")
    dt = np.diff(time)
    weights = np.empty(len(time), dtype=float)
    weights[0] = 0.5 * dt[0]
    weights[-1] = 0.5 * dt[-1]
    if len(time) > 2:
        weights[1:-1] = 0.5 * (dt[:-1] + dt[1:])
    return weights


def design_matrix(regressor, weights):
    regressor = np.asarray(regressor, dtype=float)
    weights = np.asarray(weights, dtype=float)
    if regressor.ndim != 3 or regressor.shape[1:] != (6, 10):
        raise ValueError("regressor must have shape [samples, 6, 10]")
    if weights.shape != (len(regressor),) or np.any(weights < 0.0):
        raise ValueError("invalid quadrature weights")
    return (regressor * np.sqrt(weights)[:, None, None]).reshape(-1, 10)


def spectrum_metrics(matrix):
    """Raw and column-normalized finite-horizon spectrum."""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.ndim != 2 or matrix.shape[1] != 10 or not np.all(np.isfinite(matrix)):
        raise ValueError("design matrix must be finite with ten columns")
    column_energy = np.linalg.norm(matrix, axis=0)
    active = column_energy > max(float(np.max(column_energy)), 1.0) * 1.0e-14
    safe_energy = np.where(active, column_energy, 1.0)
    normalized = matrix / safe_energy
    normalized[:, ~active] = 0.0
    raw_singular = np.linalg.svd(matrix, compute_uv=False)
    _, singular, right = np.linalg.svd(normalized, full_matrices=False)
    ratios = singular / singular[0] if singular[0] > 0.0 else np.zeros_like(singular)
    raw_ratios = (raw_singular / raw_singular[0]
                  if raw_singular[0] > 0.0 else np.zeros_like(raw_singular))
    gram = normalized.T @ normalized
    coherence = np.abs(gram.copy())
    np.fill_diagonal(coherence, 0.0)
    pair = np.unravel_index(int(np.argmax(coherence)), coherence.shape)
    ranks = {"{:.0e}".format(threshold): int(np.sum(ratios >= threshold))
             for threshold in RELATIVE_THRESHOLDS}
    smallest = float(singular[-1])
    raw_smallest = float(raw_singular[-1])
    return {
        "column_energy": column_energy,
        "active_columns": active,
        "raw_singular_values": raw_singular,
        "raw_singular_ratios": raw_ratios,
        "normalized_singular_values": singular,
        "normalized_singular_ratios": ratios,
        "right_singular_vectors": right,
        "normalized_gram": gram,
        "ranks": ranks,
        "raw_condition_number": (float(raw_singular[0] / raw_smallest)
                                 if raw_smallest > 0.0 else math.inf),
        "normalized_condition_number": (float(singular[0] / smallest)
                                        if smallest > 0.0 else math.inf),
        "maximum_column_coherence": float(coherence[pair]),
        "maximum_coherence_pair": [int(pair[0]), int(pair[1])],
    }


def coordinate_scaled_metrics(matrix, coordinate_scales):
    """Spectrum for dimensionless coordinates pi = diag(scales) * z."""
    matrix = np.asarray(matrix, dtype=float)
    coordinate_scales = np.asarray(coordinate_scales, dtype=float)
    if coordinate_scales.shape != (10,) or np.any(coordinate_scales <= 0.0):
        raise ValueError("coordinate scales must contain ten positive values")
    scaled_matrix = matrix * coordinate_scales
    _, singular, right = np.linalg.svd(scaled_matrix, full_matrices=False)
    ratios = singular / singular[0] if singular[0] > 0.0 else np.zeros(10)
    return {
        "coordinate_scales": coordinate_scales,
        "design_matrix": scaled_matrix,
        "singular_values": singular,
        "singular_ratios": ratios,
        "right_singular_vectors": right,
        "ranks": {"{:.0e}".format(threshold): int(np.sum(ratios >= threshold))
                  for threshold in RELATIVE_THRESHOLDS},
        "condition_number": (float(singular[0] / singular[-1])
                             if singular[-1] > 0.0 else math.inf),
    }


def payload_coordinate_scales(pi_star):
    """Retrospective payload-scale nondimensionalization for this frozen audit."""
    pi_star = np.asarray(pi_star, dtype=float)
    mass = abs(float(pi_star[0]))
    if mass <= 0.0:
        raise ValueError("nonzero payload mass is required for physical scaling")
    length = float(np.linalg.norm(pi_star[1:4]) / mass)
    if length <= 0.0:
        raise ValueError("nonzero payload COM offset is required for physical scaling")
    first_moment = mass * length
    inertia = mass * length * length
    return np.asarray([mass] + [first_moment] * 3 + [inertia] * 6)


def observable_projection(metrics, parameter, threshold):
    """Project a physical parameter vector using normalized regressor coordinates."""
    parameter = np.asarray(parameter, dtype=float)
    energy = metrics["column_energy"]
    right = metrics["right_singular_vectors"]
    mask = metrics["normalized_singular_ratios"] >= threshold
    scaled = energy * parameter
    basis = right[mask, :]
    scaled_observable = basis.T @ (basis @ scaled) if len(basis) else np.zeros(10)
    scaled_null = scaled - scaled_observable
    observable = np.divide(scaled_observable, energy, out=np.zeros(10), where=energy > 0.0)
    null = parameter - observable
    denominator = float(np.linalg.norm(scaled))
    matrix = metrics["design_matrix"]
    action_denominator = float(np.linalg.norm(matrix @ parameter))
    return {
        "threshold": threshold,
        "rank": int(np.sum(mask)),
        "observable": observable,
        "null": null,
        "scaled_observable_fraction": (float(np.linalg.norm(scaled_observable) / denominator)
                                       if denominator > 0.0 else 0.0),
        "scaled_null_fraction": (float(np.linalg.norm(scaled_null) / denominator)
                                 if denominator > 0.0 else 0.0),
        "observable_action_fraction": (float(np.linalg.norm(matrix @ observable) /
                                             action_denominator)
                                      if action_denominator > 0.0 else 0.0),
        "null_action_fraction": (float(np.linalg.norm(matrix @ null) /
                                       action_denominator)
                                if action_denominator > 0.0 else 0.0),
    }


def scaled_observable_projection(metrics, parameter, threshold):
    """Observable projection in a declared dimensionless physical coordinate."""
    parameter = np.asarray(parameter, dtype=float)
    scales = metrics["coordinate_scales"]
    dimensionless = parameter / scales
    mask = metrics["singular_ratios"] >= threshold
    basis = metrics["right_singular_vectors"][mask, :]
    observable_dimensionless = (basis.T @ (basis @ dimensionless)
                                if len(basis) else np.zeros(10))
    null_dimensionless = dimensionless - observable_dimensionless
    observable = scales * observable_dimensionless
    null = parameter - observable
    denominator = float(np.linalg.norm(dimensionless))
    action_denominator = float(np.linalg.norm(metrics["design_matrix"] @ dimensionless))
    return {
        "threshold": threshold,
        "rank": int(np.sum(mask)),
        "observable": observable,
        "null": null,
        "dimensionless_observable_fraction": (
            float(np.linalg.norm(observable_dimensionless) / denominator)
            if denominator > 0.0 else 0.0),
        "dimensionless_null_fraction": (
            float(np.linalg.norm(null_dimensionless) / denominator)
            if denominator > 0.0 else 0.0),
        "observable_action_fraction": (
            float(np.linalg.norm(metrics["design_matrix"] @ observable_dimensionless) /
                  action_denominator) if action_denominator > 0.0 else 0.0),
        "null_action_fraction": (
            float(np.linalg.norm(metrics["design_matrix"] @ null_dimensionless) /
                  action_denominator) if action_denominator > 0.0 else 0.0),
    }


def interpolate_tensor(grid, times, tensor):
    grid = np.asarray(grid, dtype=float)
    times = np.asarray(times, dtype=float)
    flat = np.asarray(tensor, dtype=float).reshape(len(times), -1)
    result = np.column_stack([np.interp(grid, times, flat[:, index])
                              for index in range(flat.shape[1])])
    return result.reshape((len(grid),) + np.asarray(tensor).shape[1:])


def read_csv(path):
    values = np.genfromtxt(str(path), delimiter=",", names=True, dtype=None,
                           encoding="utf-8")
    return {name: np.asarray(values[name]) for name in values.dtype.names}


def read_estimates(path):
    rows = []
    labels = set()
    with rosbag.Bag(str(path)) as bag:
        for _, message, _ in bag.read_messages(topics=[ESTIMATE_TOPIC]):
            labels.add(message.layout.dim[0].label if message.layout.dim else "")
            row = np.asarray(message.data, dtype=float)
            if len(row) != 118:
                raise RuntimeError("unexpected D4 v3 row length in {}".format(path))
            rows.append(row)
    rows = np.asarray(sorted(rows, key=lambda row: row[0]), dtype=float)
    if labels != {ESTIMATE_LABEL} or len(rows) < 100 or not np.all(np.isfinite(rows)):
        raise RuntimeError("invalid D4 v3 diagnostics in {}".format(path))
    if not np.all(np.diff(rows[:, 0]) > 0.0):
        raise RuntimeError("non-monotonic estimate time in {}".format(path))
    return rows


def load_run(run):
    config = json.loads((run / "effective_config.json").read_text())
    csv_data = read_csv(run / "task_d_d4_run.csv")
    rows = read_estimates(run / "task_d_d4.bag")
    time = np.asarray(csv_data["time_s"], dtype=float)
    raw_time = rows[:, 0]
    if raw_time[0] > time[0] or raw_time[-1] < time[-1]:
        raise RuntimeError("bag does not cover frozen D4 grid: {}".format(run))
    regressor_raw = rows[:, 48:108].reshape(-1, 6, 10)
    sigma_raw = rows[:, 42:48]
    derivative_raw = rows[:, 32:42]
    pi_hat_raw = rows[:, 22:32]
    regressor = interpolate_tensor(time, raw_time, regressor_raw)
    sigma = interpolate_tensor(time, raw_time, sigma_raw)
    derivative = interpolate_tensor(time, raw_time, derivative_raw)
    pi_hat = interpolate_tensor(time, raw_time, pi_hat_raw)
    theoretical_drive_raw = np.einsum("nij,ni->nj", regressor_raw, sigma_raw)
    truth = np.asarray([csv_data["pi_star_" + name][0] for name in PARAMETERS], dtype=float)
    weights = trapezoid_weights(time)
    matrix = design_matrix(regressor, weights)
    metrics = spectrum_metrics(matrix)
    metrics["design_matrix"] = matrix
    update_enabled = bool(config["enableParameterUpdate"])
    # Evaluator diagnostics expose the Eq.(8) candidate derivative even when
    # estimator integration is disabled. The OFF group must therefore match
    # Y^T*sigma while pi_hat itself remains frozen at its initial value.
    derivative_identity_error = float(np.max(
        np.abs(derivative_raw - theoretical_drive_raw)))
    return {
        "path": run,
        "group": config["group"],
        "run": run.name,
        "config": config,
        "time": time,
        "mode": np.asarray(csv_data["mode"], dtype=int),
        "regressor": regressor,
        "sigma": sigma,
        "derivative": derivative,
        "pi_hat": pi_hat,
        "truth": truth,
        "weights": weights,
        "matrix": matrix,
        "metrics": metrics,
        "theoretical_drive": np.einsum("nij,ni->nj", regressor, sigma),
        "derivative_identity_error": derivative_identity_error,
    }


def sliding_windows(run, coordinate_scales):
    time = run["time"]
    starts = np.arange(time[0], time[-1] - SLIDING_WINDOW_S + 1.0e-9,
                       SLIDING_STEP_S)
    output = []
    for start in starts:
        end = start + SLIDING_WINDOW_S
        mask = (time >= start - 1.0e-10) & (time <= end + 1.0e-10)
        weights = trapezoid_weights(time[mask])
        matrix = design_matrix(run["regressor"][mask], weights)
        metrics = spectrum_metrics(matrix)
        scaled = coordinate_scaled_metrics(matrix, coordinate_scales)
        output.append({
            "start": float(start), "end": float(end),
            "ratios": metrics["normalized_singular_ratios"],
            "ranks": metrics["ranks"],
            "condition": metrics["normalized_condition_number"],
            "scaled_ratios": scaled["singular_ratios"],
            "scaled_ranks": scaled["ranks"],
            "scaled_condition": scaled["condition_number"],
        })
    return output


def grouped_metrics(runs):
    matrix = np.vstack([run["matrix"] for run in runs])
    metrics = spectrum_metrics(matrix)
    metrics["design_matrix"] = matrix
    return metrics


def serializable_metrics(metrics):
    return {
        "column_energy": metrics["column_energy"].tolist(),
        "active_columns": metrics["active_columns"].tolist(),
        "raw_singular_values": metrics["raw_singular_values"].tolist(),
        "raw_singular_ratios": metrics["raw_singular_ratios"].tolist(),
        "normalized_singular_values": metrics["normalized_singular_values"].tolist(),
        "normalized_singular_ratios": metrics["normalized_singular_ratios"].tolist(),
        "ranks": metrics["ranks"],
        "raw_condition_number": metrics["raw_condition_number"],
        "normalized_condition_number": metrics["normalized_condition_number"],
        "maximum_column_coherence": metrics["maximum_column_coherence"],
        "maximum_coherence_pair": [PARAMETERS[index]
                                    for index in metrics["maximum_coherence_pair"]],
    }


def write_csv(path, fieldnames, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot_results(output, group_metrics_map, group_scaled_map, runs_by_group,
                 sliding, projections, per_mode_rows, drive_rows):
    colors = {"nominal": "tab:blue", "uncertain_off": "tab:orange",
              "uncertain_adaptive": "tab:green"}

    fig, axes = plt.subplots(2, 1, figsize=(9, 9), sharex=True)
    x = np.arange(1, 11)
    for group in GROUPS:
        axes[0].semilogy(x, group_metrics_map[group]["normalized_singular_ratios"],
                         "o-", label=group, color=colors[group])
        axes[1].semilogy(x, group_scaled_map[group]["singular_ratios"],
                         "o-", label=group, color=colors[group])
    for threshold in RELATIVE_THRESHOLDS:
        for axis in axes:
            axis.axhline(threshold, color="0.6", ls="--", lw=0.8)
    axes[0].set(ylabel="normalized singular value",
                title="Column-normalized spectrum: directional independence only")
    axes[1].set(xlabel="singular-value index", ylabel="normalized singular value",
                title="Payload-scale dimensionless spectrum: physical sensitivity")
    axes[1].set_xticks(x)
    for axis in axes:
        axis.grid(True, which="both", alpha=0.3)
        axis.legend()
    fig.suptitle("D5A full-window regressor spectrum", y=0.995)
    fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.965])
    fig.savefig(output / "figure_D5_01_full_window_spectrum.png", dpi=170)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    for group in GROUPS:
        group_windows = [sliding[(group, run["run"])] for run in runs_by_group[group]]
        centers = np.asarray([(row["start"] + row["end"]) / 2.0
                              for row in group_windows[0]])
        ratios = np.asarray([[row["scaled_ratios"][-1] for row in values]
                             for values in group_windows])
        ranks = np.asarray([[row["scaled_ranks"]["1e-03"] for row in values]
                            for values in group_windows])
        axes[0].semilogy(centers, np.mean(ratios, axis=0), label=group,
                         color=colors[group])
        axes[0].fill_between(centers, np.min(ratios, axis=0), np.max(ratios, axis=0),
                             color=colors[group], alpha=0.15)
        axes[1].plot(centers, np.mean(ranks, axis=0), label=group,
                     color=colors[group])
        axes[1].fill_between(centers, np.min(ranks, axis=0), np.max(ranks, axis=0),
                             color=colors[group], alpha=0.15)
    axes[0].axhline(PRACTICAL_THRESHOLD, color="k", ls="--", lw=0.8)
    axes[0].set_ylabel("smallest / largest singular value")
    axes[0].set_title("2 s sliding-window persistent-excitation diagnostic")
    axes[1].set_ylabel("payload-scale rank at threshold 1e-3")
    axes[1].set_xlabel("window center MPC time [s]")
    axes[1].set_ylim(-0.3, 10.3)
    for axis in axes:
        axis.grid(True, which="both", alpha=0.3)
        axis.legend()
    fig.tight_layout()
    fig.savefig(output / "figure_D5_02_sliding_window_pe.png", dpi=170)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(11, 5.5))
    width = 0.25
    indices = np.arange(10)
    for offset, group in enumerate(GROUPS):
        duration = sum(run["time"][-1] - run["time"][0]
                       for run in runs_by_group[group])
        rms = group_metrics_map[group]["column_energy"] / math.sqrt(duration)
        axis.bar(indices + (offset - 1) * width, rms, width, label=group,
                 color=colors[group])
    axis.set_yscale("log")
    axis.set_xticks(indices)
    axis.set_xticklabels(PARAMETERS)
    axis.set_ylabel("column RMS over 6 wrench rows [mixed units]")
    axis.set_title("D5A regressor-column excitation (do not compare units as parameters)")
    axis.grid(True, axis="y", which="both", alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output / "figure_D5_03_column_excitation.png", dpi=170)
    plt.close(fig)

    coherence = np.abs(group_metrics_map["uncertain_adaptive"]["normalized_gram"])
    fig, axis = plt.subplots(figsize=(8, 7))
    image = axis.imshow(coherence, vmin=0.0, vmax=1.0, cmap="magma")
    axis.set_xticks(range(10))
    axis.set_xticklabels(PARAMETERS, rotation=45, ha="right")
    axis.set_yticks(range(10))
    axis.set_yticklabels(PARAMETERS)
    axis.set_title("Uncertain adaptive: absolute column-normalized Gram matrix")
    fig.colorbar(image, ax=axis, label="absolute normalized inner product")
    fig.tight_layout()
    fig.savefig(output / "figure_D5_04_column_coherence.png", dpi=170)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    truth = runs_by_group["uncertain_adaptive"][0]["truth"]
    for axis, threshold in zip(axes, RELATIVE_THRESHOLDS):
        projection = projections["{:.0e}".format(threshold)]
        indices = np.arange(10)
        axis.bar(indices - 0.25, truth, 0.25, label="pi_star", color="0.3")
        axis.bar(indices, projection["observable"], 0.25, label="observable projection")
        axis.bar(indices + 0.25, projection["null"], 0.25, label="weak/null projection")
        axis.set_ylabel("mixed parameter units")
        axis.set_title("threshold {:.0e}, rank {}".format(threshold, projection["rank"]))
        axis.grid(True, axis="y", alpha=0.3)
    axes[0].legend(ncol=3)
    axes[-1].set_xticks(range(10))
    axes[-1].set_xticklabels(PARAMETERS)
    fig.suptitle("pi_star decomposition in payload-scale dimensionless coordinates",
                 y=0.995)
    fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.97])
    fig.savefig(output / "figure_D5_05_pi_star_observable_projection.png", dpi=170)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(11, 5.5))
    width = 0.25
    for offset, group in enumerate(GROUPS):
        values = [row["drive_rms"] for row in drive_rows if row["group"] == group]
        by_parameter = np.asarray(values).reshape(len(runs_by_group[group]), 10)
        axis.bar(np.arange(10) + (offset - 1) * width, np.mean(by_parameter, axis=0),
                 width, label=group, color=colors[group])
    axis.set_yscale("log")
    axis.set_xticks(range(10))
    axis.set_xticklabels(PARAMETERS)
    axis.set_ylabel("RMS of Y^T sigma [parameter-rate units]")
    axis.set_title("D5A Eq.(8) theoretical update drive")
    axis.grid(True, axis="y", which="both", alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output / "figure_D5_06_update_drive.png", dpi=170)
    plt.close(fig)

    mode_names = sorted({int(row["mode"]) for row in per_mode_rows})
    fig, axis = plt.subplots(figsize=(10, 5.5))
    width = 0.25
    for offset, group in enumerate(GROUPS):
        ranks = []
        for mode in mode_names:
            values = [float(row["payload_scaled_rank_1e_3"]) for row in per_mode_rows
                      if row["group"] == group and int(row["mode"]) == mode]
            ranks.append(float(np.mean(values)))
        axis.bar(np.arange(len(mode_names)) + (offset - 1) * width, ranks, width,
                 label=group, color=colors[group])
    axis.set_xticks(range(len(mode_names)))
    axis.set_xticklabels([str(value) for value in mode_names])
    axis.set(xlabel="contact mode", ylabel="mean payload-scale rank at threshold 1e-3",
             title="Per-contact-mode finite-horizon regressor rank")
    axis.set_ylim(0.0, 10.5)
    axis.grid(True, axis="y", alpha=0.3)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output / "figure_D5_07_per_mode_rank.png", dpi=170)
    plt.close(fig)


def main():
    if len(sys.argv) not in (2, 3):
        raise SystemExit("usage: analyze_task_d_d5_identifiability.py D4_ROOT [OUTPUT_DIR]")
    root = Path(sys.argv[1]).resolve()
    output = (Path(sys.argv[2]).resolve() if len(sys.argv) == 3 else
              root.parent / "stage_d5_identifiability_v1")
    output.mkdir(parents=True, exist_ok=True)

    d4_summary_path = root / "analysis" / "stage_d4_analysis_summary.json"
    fairness_path = root / "stage_d4_fairness_audit.json"
    d4_summary = json.loads(d4_summary_path.read_text())
    fairness = json.loads(fairness_path.read_text())
    if not d4_summary.get("passed") or not fairness.get("passed"):
        raise RuntimeError("D5A requires the frozen, passed D4 campaign")

    runs_by_group = {}
    inputs = []
    for group in GROUPS:
        paths = sorted((root / group).glob("run_*"))
        if len(paths) != 3:
            raise RuntimeError("expected exactly three D4 runs for {}".format(group))
        runs_by_group[group] = [load_run(path) for path in paths]
        for path in paths:
            for name in ("task_d_d4.bag", "task_d_d4_run.csv", "effective_config.json",
                         "task_d_d4_run_summary.json", "source_files.sha256"):
                inputs.append(path / name)

    reference_time = runs_by_group[GROUPS[0]][0]["time"]
    common_grid_ok = all(np.array_equal(run["time"], reference_time)
                         for group in GROUPS for run in runs_by_group[group])
    truth = runs_by_group["uncertain_adaptive"][0]["truth"]
    truth_ok = (np.linalg.norm(truth) > 0.0 and
                all(np.allclose(run["truth"], truth, rtol=0.0, atol=1.0e-12)
                    for group in ("uncertain_off", "uncertain_adaptive")
                    for run in runs_by_group[group]) and
                all(np.linalg.norm(run["truth"]) <= 1.0e-12
                    for run in runs_by_group["nominal"]))

    coordinate_scales = payload_coordinate_scales(truth)
    group_metrics_map = {group: grouped_metrics(runs_by_group[group]) for group in GROUPS}
    group_scaled_map = {
        group: coordinate_scaled_metrics(group_metrics_map[group]["design_matrix"],
                                         coordinate_scales)
        for group in GROUPS}
    per_run_rows = []
    column_rows = []
    spectrum_rows = []
    drive_rows = []
    sliding = {}
    sliding_rows = []
    per_mode_rows = []
    derivative_errors = []
    for group in GROUPS:
        for run in runs_by_group[group]:
            metrics = run["metrics"]
            scaled_metrics = coordinate_scaled_metrics(run["matrix"], coordinate_scales)
            per_run_rows.append({
                "group": group, "run": run["run"],
                "rank_1e_2": metrics["ranks"]["1e-02"],
                "rank_1e_3": metrics["ranks"]["1e-03"],
                "rank_1e_6": metrics["ranks"]["1e-06"],
                "normalized_condition": metrics["normalized_condition_number"],
                "raw_condition": metrics["raw_condition_number"],
                "smallest_normalized_ratio": metrics["normalized_singular_ratios"][-1],
                "maximum_coherence": metrics["maximum_column_coherence"],
                "coherence_pair": "{}:{}".format(*[PARAMETERS[i]
                    for i in metrics["maximum_coherence_pair"]]),
                "eq8_derivative_identity_max_error": run["derivative_identity_error"],
                "payload_scaled_rank_1e_2": scaled_metrics["ranks"]["1e-02"],
                "payload_scaled_rank_1e_3": scaled_metrics["ranks"]["1e-03"],
                "payload_scaled_rank_1e_6": scaled_metrics["ranks"]["1e-06"],
                "payload_scaled_condition": scaled_metrics["condition_number"],
                "payload_scaled_smallest_ratio": scaled_metrics["singular_ratios"][-1],
            })
            derivative_errors.append(run["derivative_identity_error"])
            duration = run["time"][-1] - run["time"][0]
            for index, name in enumerate(PARAMETERS):
                column_rows.append({
                    "group": group, "run": run["run"], "parameter": name,
                    "column_energy": metrics["column_energy"][index],
                    "column_rms": metrics["column_energy"][index] / math.sqrt(duration),
                })
                drive = run["theoretical_drive"][:, index]
                drive_rows.append({
                    "group": group, "run": run["run"], "parameter": name,
                    "drive_rms": math.sqrt(float(np.sum(run["weights"] * drive ** 2) /
                                                      np.sum(run["weights"]))),
                    "drive_signed_integral": float(np.sum(run["weights"] * drive)),
                    "observed_pi_change": float(run["pi_hat"][-1, index] -
                                                run["pi_hat"][0, index]),
                })
            windows = sliding_windows(run, coordinate_scales)
            sliding[(group, run["run"])] = windows
            for row in windows:
                sliding_rows.append({
                    "group": group, "run": run["run"],
                    "start_s": row["start"], "end_s": row["end"],
                    "rank_1e_2": row["ranks"]["1e-02"],
                    "rank_1e_3": row["ranks"]["1e-03"],
                    "rank_1e_6": row["ranks"]["1e-06"],
                    "smallest_normalized_ratio": row["ratios"][-1],
                    "normalized_condition": row["condition"],
                    "payload_scaled_rank_1e_2": row["scaled_ranks"]["1e-02"],
                    "payload_scaled_rank_1e_3": row["scaled_ranks"]["1e-03"],
                    "payload_scaled_rank_1e_6": row["scaled_ranks"]["1e-06"],
                    "payload_scaled_smallest_ratio": row["scaled_ratios"][-1],
                    "payload_scaled_condition": row["scaled_condition"],
                })
            for mode in sorted(set(run["mode"])):
                mask = run["mode"] == mode
                # Each selected common-grid sample represents one 20 ms bin;
                # this avoids integrating across gaps between repeated gait phases.
                weights = np.full(int(np.sum(mask)), float(run["config"]["common_grid_dt_s"]))
                mode_metrics = spectrum_metrics(design_matrix(run["regressor"][mask], weights))
                mode_scaled = coordinate_scaled_metrics(
                    design_matrix(run["regressor"][mask], weights), coordinate_scales)
                per_mode_rows.append({
                    "group": group, "run": run["run"], "mode": int(mode),
                    "sample_count": int(np.sum(mask)),
                    "represented_duration_s": float(np.sum(weights)),
                    "rank_1e_2": mode_metrics["ranks"]["1e-02"],
                    "rank_1e_3": mode_metrics["ranks"]["1e-03"],
                    "rank_1e_6": mode_metrics["ranks"]["1e-06"],
                    "smallest_normalized_ratio": mode_metrics["normalized_singular_ratios"][-1],
                    "maximum_coherence": mode_metrics["maximum_column_coherence"],
                    "payload_scaled_rank_1e_2": mode_scaled["ranks"]["1e-02"],
                    "payload_scaled_rank_1e_3": mode_scaled["ranks"]["1e-03"],
                    "payload_scaled_rank_1e_6": mode_scaled["ranks"]["1e-06"],
                    "payload_scaled_smallest_ratio": mode_scaled["singular_ratios"][-1],
                })

    for group in GROUPS:
        metrics = group_metrics_map[group]
        scaled_metrics = group_scaled_map[group]
        for index in range(10):
            spectrum_rows.append({
                "group": group, "index": index + 1,
                "raw_singular_value": metrics["raw_singular_values"][index],
                "raw_ratio": metrics["raw_singular_ratios"][index],
                "normalized_singular_value": metrics["normalized_singular_values"][index],
                "normalized_ratio": metrics["normalized_singular_ratios"][index],
                "payload_scaled_singular_value": scaled_metrics["singular_values"][index],
                "payload_scaled_ratio": scaled_metrics["singular_ratios"][index],
            })
        np.savetxt(output / ("normalized_gram_{}.csv".format(group)),
                   metrics["normalized_gram"], delimiter=",",
                   header=",".join(PARAMETERS), comments="")
        np.savetxt(output / ("payload_scaled_right_singular_vectors_{}.csv".format(group)),
                   scaled_metrics["right_singular_vectors"], delimiter=",",
                   header=",".join(PARAMETERS), comments="")

    uncertain_metrics = group_metrics_map["uncertain_adaptive"]
    uncertain_scaled = group_scaled_map["uncertain_adaptive"]
    projections = {}
    projection_rows = []
    for threshold in RELATIVE_THRESHOLDS:
        projection = scaled_observable_projection(uncertain_scaled, truth, threshold)
        key = "{:.0e}".format(threshold)
        projections[key] = projection
        for index, name in enumerate(PARAMETERS):
            projection_rows.append({
                "threshold": threshold, "rank": projection["rank"], "parameter": name,
                "pi_star": truth[index],
                "observable_projection": projection["observable"][index],
                "weak_or_null_projection": projection["null"][index],
            })

    write_csv(output / "identifiability_per_run.csv", list(per_run_rows[0]), per_run_rows)
    write_csv(output / "group_singular_spectrum.csv", list(spectrum_rows[0]), spectrum_rows)
    write_csv(output / "column_excitation_per_run.csv", list(column_rows[0]), column_rows)
    write_csv(output / "sliding_window_identifiability.csv", list(sliding_rows[0]), sliding_rows)
    write_csv(output / "per_mode_identifiability.csv", list(per_mode_rows[0]), per_mode_rows)
    write_csv(output / "eq8_update_drive_per_run.csv", list(drive_rows[0]), drive_rows)
    write_csv(output / "pi_star_observable_projection.csv", list(projection_rows[0]), projection_rows)

    plot_results(output, group_metrics_map, group_scaled_map, runs_by_group,
                 sliding, projections, per_mode_rows, drive_rows)

    sliding_summary = {}
    for group in GROUPS:
        rows = [row for row in sliding_rows if row["group"] == group]
        ranks = np.asarray([row["rank_1e_3"] for row in rows], dtype=float)
        ratios = np.asarray([row["smallest_normalized_ratio"] for row in rows], dtype=float)
        scaled_ranks = np.asarray([row["payload_scaled_rank_1e_3"] for row in rows],
                                  dtype=float)
        scaled_ratios = np.asarray([row["payload_scaled_smallest_ratio"] for row in rows],
                                   dtype=float)
        sliding_summary[group] = {
            "window_s": SLIDING_WINDOW_S,
            "window_run_count": len(rows),
            "practical_rank_min": int(np.min(ranks)),
            "practical_rank_median": float(np.median(ranks)),
            "practical_rank_max": int(np.max(ranks)),
            "full_rank_window_fraction": float(np.mean(ranks == 10)),
            "smallest_ratio_min": float(np.min(ratios)),
            "smallest_ratio_median": float(np.median(ratios)),
            "smallest_ratio_max": float(np.max(ratios)),
            "payload_scaled_practical_rank_min": int(np.min(scaled_ranks)),
            "payload_scaled_practical_rank_median": float(np.median(scaled_ranks)),
            "payload_scaled_practical_rank_max": int(np.max(scaled_ranks)),
            "payload_scaled_full_rank_window_fraction": float(np.mean(scaled_ranks == 10)),
            "payload_scaled_smallest_ratio_min": float(np.min(scaled_ratios)),
            "payload_scaled_smallest_ratio_median": float(np.median(scaled_ratios)),
            "payload_scaled_smallest_ratio_max": float(np.max(scaled_ratios)),
        }

    projection_json = {}
    for key, value in projections.items():
        projection_json[key] = {
            "rank": value["rank"],
            "observable": value["observable"].tolist(),
            "weak_or_null": value["null"].tolist(),
            "dimensionless_observable_fraction": value["dimensionless_observable_fraction"],
            "dimensionless_null_fraction": value["dimensionless_null_fraction"],
            "observable_action_fraction": value["observable_action_fraction"],
            "null_action_fraction": value["null_action_fraction"],
        }

    practical = group_metrics_map["uncertain_adaptive"]
    practical_rank = practical["ranks"]["1e-03"]
    physical = group_scaled_map["uncertain_adaptive"]
    physical_rank = physical["ranks"]["1e-03"]
    persistent_full_rank = sliding_summary["uncertain_adaptive"][
        "payload_scaled_full_rank_window_fraction"]
    passed = bool(common_grid_ok and truth_ok and max(derivative_errors) <= 1.0e-10)
    summary = {
        "schema_version": 1,
        "stage": "D5A_offline_identifiability",
        "passed": passed,
        "scope": "offline analysis of frozen D4 bags/CSVs; no new acquisition or controller change",
        "d4_root": str(root),
        "output_dir": str(output),
        "checks": {
            "d4_campaign_passed": True,
            "nine_runs_loaded": sum(len(value) for value in runs_by_group.values()) == 9,
            "common_grid_identical": common_grid_ok,
            "payload_truth_consistent": truth_ok,
            "eq8_logged_derivative_identity_max_error_le_1e_10": max(derivative_errors) <= 1.0e-10,
        },
        "definitions": {
            "design_matrix": "A = stack_t(sqrt(w_t) * Y(t)), so A^T A approximates integral Y^T Y dt",
            "column_normalization": "A_n = A diag(1/||A_col||_2); required because 10D coordinates have mixed units",
            "payload_coordinate_scaling": (
                "pi = diag(s) z with s=[m*, m*L*, m*L*, m*L*, "
                "m*L*^2 x6], L*=||h*||/m*; retrospective dimensionless "
                "sensitivity audit using the frozen known test payload"),
            "payload_coordinate_scales": coordinate_scales.tolist(),
            "rank_rule": "count singular values with s_i/s_max >= relative threshold",
            "practical_threshold": PRACTICAL_THRESHOLD,
            "persistent_excitation_diagnostic": "2 s sliding windows; finite data evidence, not a theorem or convergence proof",
        },
        "groups": {group: {
            "column_normalized": serializable_metrics(group_metrics_map[group]),
            "payload_scaled": {
                "singular_values": group_scaled_map[group]["singular_values"].tolist(),
                "singular_ratios": group_scaled_map[group]["singular_ratios"].tolist(),
                "ranks": group_scaled_map[group]["ranks"],
                "condition_number": group_scaled_map[group]["condition_number"],
            }} for group in GROUPS},
        "sliding_window_summary": sliding_summary,
        "payload_pi_star": truth.tolist(),
        "uncertain_adaptive_pi_star_projection": projection_json,
        "headline": {
            "uncertain_adaptive_full_window_practical_rank": practical_rank,
            "uncertain_adaptive_full_window_smallest_ratio": float(practical["normalized_singular_ratios"][-1]),
            "uncertain_adaptive_full_window_condition": practical["normalized_condition_number"],
            "uncertain_adaptive_payload_scaled_full_window_rank": physical_rank,
            "uncertain_adaptive_payload_scaled_smallest_ratio": float(physical["singular_ratios"][-1]),
            "uncertain_adaptive_payload_scaled_condition": physical["condition_number"],
            "uncertain_adaptive_payload_scaled_2s_full_rank_window_fraction": persistent_full_rank,
            "full_window_rank_is_not_parameter_convergence": True,
        },
        "scientific_boundary": [
            "A finite-horizon full-rank Gramian is only evidence that the logged regressor spans ten directions over that horizon.",
            "It does not prove persistent excitation, estimator convergence, unbiasedness, or physical payload identification.",
            "Nominal pi_hat bias and feedback coupling remain possible causes of the D4 reconstruction error.",
        ],
    }
    (output / "stage_d5_identifiability_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")

    pair = practical["maximum_coherence_pair"]
    projection = projections["1e-03"]
    report = """# Stage D5A — D4 离线可辨识性分析

## 1. 范围与数据

- 输入：D4 正式 campaign 的 9 个冻结 bag/CSV（3 组 × 3 次）。
- 分析窗口：1.2–16.2 s，公共时间步 0.02 s。
- 本阶段没有重新采集数据，没有修改控制器、Eq.(8)、Eq.(10)、Gamma 或轨迹。
- 结论只适用于当前 +0.10 m static-walk 激励与当前 15 s 窗口。

## 2. 方法

对每个样本读取 bag 中的 `Y(t) ∈ R^(6×10)`，构造
`A = stack_t(sqrt(w_t) Y(t))`，使 `A^T A ≈ ∫Y^T Y dt`。
由于参数坐标分别为 kg、kg·m 和 kg·m²，同时报告原始谱和列归一化谱。
秩采用相对阈值 `s_i/s_max`，并额外用 2 s 滑动窗口检查持续激励。

## 3. 核心结果

- Uncertain + Adaptive 的 15 s 全窗口列归一化 rank（1e-3）：**{rank}/10**；条件数 **{condition:.6g}**。
- 按本次 payload 的质量/长度/惯量尺度无量纲化后，15 s practical rank：**{physical_rank}/10**；
  最小/最大奇异值比 **{physical_ratio:.6g}**，条件数 **{physical_condition:.6g}**。
- 物理尺度下 2 s 窗口 practical rank 范围：**{rank_min}–{rank_max}**，中位数 **{rank_median:.3g}**。
- 物理尺度下 2 s 窗口达到 10/10 的比例：**{full_fraction:.2%}**。
- 1e-2 的稳健 rank 只有 **{robust_rank}/10**；主导方向约为 `m`、`hx/hy` 两个组合和 `hz`。
- 第 5 个临界方向主要是 `Jxy`；较弱方向主要由 `Jyz/Jyy/Jxz/Jxx/Jzz` 构成，其中 `Jzz` 最弱。
- 最大列相干性：**{coherence:.6g}**，对应 **{pair_left} / {pair_right}**。
- 1e-3 子空间中 pi_star 的无量纲范数分量：observable **{observable_fraction:.2%}**、weak/null **{null_fraction:.2%}**；
  对应平方能量占比约 **{observable_energy:.2%} / {null_energy:.2%}**。但 weak/null 部分在当前 `Y*pi_star`
  作用中的占比仅 **{null_action:.3%}**，这正说明这些物理参数变化对当前轨迹动力学输出不敏感。
- Eq.(8) 记录的 `pi_dot` 与 `Y^T sigma`（Gamma=I10）最大一致性误差：**{identity_error:.3e}**。

## 4. 科学解释

列归一化满秩只说明方向线性独立，不说明各物理坐标具有相近灵敏度；payload-scale 谱才量化相对物理参数变化的可见程度。
全窗口 rank 说明“整个 15 s 合起来”覆盖多少方向；2 s 滑动窗口结果才更接近持续激励诊断。
即使某种缩放下满秩，也不能据此宣称 pi_hat 应收敛到 pi_star：反馈闭环中的 sigma、有限时间、灵敏度不平衡、
名义组自身的 pi_hat bias，以及未调的 Gamma=I10 都会影响参数轨迹。

D4 已观察到 Eq.(10) 改变 policy 并恢复 tracking；本报告只回答参数可辨识条件，
不改变 D4 的控制性能结论，也不把 10D 参数重构误差解释为论文失败。

## 5. 输出

- `stage_d5_identifiability_summary.json`：机器可读总表。
- `identifiability_per_run.csv`、`group_singular_spectrum.csv`：每次及分组谱。
- `sliding_window_identifiability.csv`：2 s 持续激励诊断。
- `per_mode_identifiability.csv`：按接触模式分解。
- `column_excitation_per_run.csv`、`normalized_gram_*.csv`：列能量与相干性。
- `payload_scaled_right_singular_vectors_*.csv`：物理尺度下各强/弱参数组合。
- `eq8_update_drive_per_run.csv`：`Y^T sigma` 更新驱动。
- `pi_star_observable_projection.csv`：pi_star 的可观/弱观子空间投影。
- Figure D5_01–D5_07：统一图形展示。

## 6. Gate

D5A 数据/数学一致性检查：**{gate}**。这表示离线审计产物有效；不等于“10D 参数已经收敛”。
""".format(
        rank=practical_rank,
        condition=practical["normalized_condition_number"],
        physical_rank=physical_rank,
        physical_ratio=physical["singular_ratios"][-1],
        physical_condition=physical["condition_number"],
        robust_rank=physical["ranks"]["1e-02"],
        rank_min=sliding_summary["uncertain_adaptive"]["payload_scaled_practical_rank_min"],
        rank_max=sliding_summary["uncertain_adaptive"]["payload_scaled_practical_rank_max"],
        rank_median=sliding_summary["uncertain_adaptive"]["payload_scaled_practical_rank_median"],
        full_fraction=persistent_full_rank,
        coherence=practical["maximum_column_coherence"],
        pair_left=PARAMETERS[pair[0]], pair_right=PARAMETERS[pair[1]],
        observable_fraction=projection["dimensionless_observable_fraction"],
        null_fraction=projection["dimensionless_null_fraction"],
        observable_energy=projection["dimensionless_observable_fraction"] ** 2,
        null_energy=projection["dimensionless_null_fraction"] ** 2,
        null_action=projection["null_action_fraction"],
        identity_error=max(derivative_errors), gate="PASS" if passed else "FAIL")
    (output / "stage_d5_identifiability_report.md").write_text(report)

    input_manifest = {
        "schema_version": 1,
        "d4_summary_sha256": sha256(d4_summary_path),
        "d4_fairness_sha256": sha256(fairness_path),
        "analyzer_sha256": sha256(Path(__file__)),
        "inputs": [{"path": str(path), "sha256": sha256(path)} for path in inputs],
    }
    (output / "stage_d5_input_manifest.json").write_text(
        json.dumps(input_manifest, indent=2, sort_keys=True) + "\n")
    artifacts = sorted(path for path in output.iterdir()
                       if path.is_file() and path.name != "stage_d5_artifact_manifest.json")
    (output / "stage_d5_artifact_manifest.json").write_text(json.dumps({
        "schema_version": 1,
        "artifacts": [{"name": path.name, "sha256": sha256(path)} for path in artifacts],
    }, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary["headline"], indent=2, sort_keys=True))
    print("D5A {}: {}".format("PASS" if passed else "FAIL", output))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
