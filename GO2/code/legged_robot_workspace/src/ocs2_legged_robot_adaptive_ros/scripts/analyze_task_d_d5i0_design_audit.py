#!/usr/bin/env python3
"""D5I0 read-only richer-excitation / 10D identifiability design audit.

The analyzer consumes the frozen, passed D5H1 four-mode bags.  It measures
the finite-horizon 6x10 regressor information and the directions actually
visited by the Eq.(8) update drive.  It does not replay or modify the
controller, choose Gamma, publish a target, or claim parameter convergence.
"""

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
import rosbag


GROUPS = ("nominal_raw", "nominal_corrected",
          "uncertain_raw", "uncertain_corrected")
RUNS = ("run_01", "run_02", "run_03")
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz",
              "Jyy", "Jyz", "Jzz")
V3_TOPIC = "/task_d/payload_inertial_10d_v3_sample"
D5H1_TOPIC = "/task_d/d5h1_applied_update_sample"
V3_LABEL = "payload_inertial_10d_v3_118_fields"
D5H1_LABEL = "payload_inertial_d5h1_applied_v1_71_fields"
THRESHOLDS = (1.0e-2, 1.0e-3, 1.0e-6)
PRIMARY_THRESHOLD = 1.0e-3
WINDOW_START_S = 0.10
WINDOW_END_S = 15.0
PHASE_START_S = 2.2


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def trapezoid_weights(time):
    """Positive quadrature weights whose sum is the sampled time span."""
    time = np.asarray(time, dtype=float)
    if time.ndim != 1 or len(time) < 2 or np.any(np.diff(time) <= 0.0):
        raise ValueError("time must contain at least two increasing samples")
    dt = np.diff(time)
    result = np.empty(len(time), dtype=float)
    result[0], result[-1] = 0.5 * dt[0], 0.5 * dt[-1]
    if len(time) > 2:
        result[1:-1] = 0.5 * (dt[:-1] + dt[1:])
    return result


def weighted_regressor_design(regressor, weights, coordinate_scales=None,
                              wrench_row_scales=None):
    """Weighted design for scaled parameters and, optionally, wrench rows.

    ``wrench_row_scales=[F,F,F,T,T,T]`` removes the otherwise arbitrary
    numerical comparison of newtons with newton-metres.
    """
    regressor = np.asarray(regressor, dtype=float)
    weights = np.asarray(weights, dtype=float)
    if regressor.ndim != 3 or regressor.shape[1:] != (6, 10):
        raise ValueError("regressor must be [N,6,10]")
    if weights.shape != (len(regressor),) or np.any(weights <= 0.0):
        raise ValueError("weights must be positive and match the regressor")
    scales = (np.ones(10) if coordinate_scales is None else
              np.asarray(coordinate_scales, dtype=float))
    if scales.shape != (10,) or np.any(scales <= 0.0):
        raise ValueError("coordinate scales must be ten positive values")
    row_scales = (np.ones(6) if wrench_row_scales is None else
                  np.asarray(wrench_row_scales, dtype=float))
    if row_scales.shape != (6,) or np.any(row_scales <= 0.0):
        raise ValueError("wrench row scales must be six positive values")
    scaled = regressor * scales[None, None, :] / row_scales[None, :, None]
    return (scaled *
            np.sqrt(weights)[:, None, None]).reshape(-1, 10)


def weighted_signal_design(signal, weights, coordinate_scales, center=False):
    """Dimensionless estimator-rate direction matrix for pi_dot/s."""
    signal = np.asarray(signal, dtype=float)
    weights = np.asarray(weights, dtype=float)
    scales = np.asarray(coordinate_scales, dtype=float)
    if signal.ndim != 2 or signal.shape[1] != 10:
        raise ValueError("signal must be [N,10]")
    if weights.shape != (len(signal),) or np.any(weights <= 0.0):
        raise ValueError("invalid signal weights")
    if scales.shape != (10,) or np.any(scales <= 0.0):
        raise ValueError("invalid coordinate scales")
    values = signal.copy()
    if center:
        values -= np.sum(weights[:, None] * values, axis=0) / np.sum(weights)
    return np.sqrt(weights)[:, None] * values / scales[None, :]


def spectrum(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.ndim != 2 or matrix.shape[1] != 10 or not np.all(np.isfinite(matrix)):
        raise ValueError("spectrum matrix must be finite with ten columns")
    _, singular, right = np.linalg.svd(matrix, full_matrices=False)
    ratios = singular / singular[0] if singular[0] > 0.0 else np.zeros(10)
    positive = singular[-1] > 0.0
    probability = singular ** 2
    probability = probability / np.sum(probability) if np.sum(probability) else probability
    effective_rank = (math.exp(-float(np.sum(
        probability[probability > 0.0] * np.log(probability[probability > 0.0]))))
                      if np.any(probability > 0.0) else 0.0)
    return {
        "singular_values": singular,
        "singular_ratios": ratios,
        "right_singular_vectors": right,
        "ranks": {"{:.0e}".format(value): int(np.sum(ratios >= value))
                  for value in THRESHOLDS},
        "condition_number": float(singular[0] / singular[-1]) if positive else math.inf,
        "effective_rank": effective_rank,
    }


def column_normalized_spectrum(matrix):
    matrix = np.asarray(matrix, dtype=float)
    energy = np.linalg.norm(matrix, axis=0)
    normalized = np.divide(matrix, energy[None, :], out=np.zeros_like(matrix),
                           where=energy[None, :] > 0.0)
    metrics = spectrum(normalized)
    gram = normalized.T @ normalized
    coherence = np.abs(gram.copy())
    np.fill_diagonal(coherence, 0.0)
    pair = np.unravel_index(int(np.argmax(coherence)), coherence.shape)
    metrics.update({
        "column_energy": energy,
        "normalized_gram": gram,
        "maximum_coherence": float(coherence[pair]),
        "maximum_coherence_pair": [int(pair[0]), int(pair[1])],
    })
    return metrics


def observable_projection(metrics, physical_parameter, coordinate_scales,
                          threshold=PRIMARY_THRESHOLD):
    """Project pi_star in dimensionless physical coordinates."""
    scales = np.asarray(coordinate_scales, dtype=float)
    parameter = np.asarray(physical_parameter, dtype=float)
    dimensionless = parameter / scales
    mask = metrics["singular_ratios"] >= threshold
    basis = metrics["right_singular_vectors"][mask]
    observable = basis.T @ (basis @ dimensionless) if len(basis) else np.zeros(10)
    weak = dimensionless - observable
    denominator = np.linalg.norm(dimensionless)
    return {
        "rank": int(np.sum(mask)),
        "dimensionless_observable_fraction": (
            float(np.linalg.norm(observable) / denominator) if denominator else 0.0),
        "dimensionless_weak_fraction": (
            float(np.linalg.norm(weak) / denominator) if denominator else 0.0),
        "observable_physical": (observable * scales).tolist(),
        "weak_physical": (weak * scales).tolist(),
    }


def match_rows(left, right, tolerance=1.0e-9):
    """Match two time-sorted diagnostic arrays without interpolation."""
    left = np.asarray(left, dtype=float)
    right = np.asarray(right, dtype=float)
    output_left, output_right = [], []
    index = 0
    for row in left:
        while index + 1 < len(right) and right[index, 0] < row[0] - tolerance:
            index += 1
        candidates = [index]
        if index + 1 < len(right):
            candidates.append(index + 1)
        best = min(candidates, key=lambda item: abs(right[item, 0] - row[0]))
        if abs(right[best, 0] - row[0]) <= tolerance:
            output_left.append(row)
            output_right.append(right[best])
    return np.asarray(output_left), np.asarray(output_right)


def read_bag(path):
    rows = {V3_TOPIC: [], D5H1_TOPIC: []}
    labels = {V3_TOPIC: set(), D5H1_TOPIC: set()}
    with rosbag.Bag(str(path)) as bag:
        for topic, message, _ in bag.read_messages(topics=list(rows)):
            rows[topic].append(np.asarray(message.data, dtype=float))
            labels[topic].add(message.layout.dim[0].label
                              if message.layout.dim else "")
    for topic in rows:
        rows[topic] = np.asarray(sorted(rows[topic], key=lambda row: row[0]))
    if (labels[V3_TOPIC] != {V3_LABEL} or
            labels[D5H1_TOPIC] != {D5H1_LABEL} or
            rows[V3_TOPIC].ndim != 2 or rows[V3_TOPIC].shape[1] != 118 or
            rows[D5H1_TOPIC].ndim != 2 or rows[D5H1_TOPIC].shape[1] != 71 or
            min(len(rows[V3_TOPIC]), len(rows[D5H1_TOPIC])) < 100 or
            not np.all(np.isfinite(rows[V3_TOPIC])) or
            not np.all(np.isfinite(rows[D5H1_TOPIC]))):
        raise RuntimeError("invalid D5H1 diagnostic schema: " + str(path))
    return rows[V3_TOPIC], rows[D5H1_TOPIC]


def load_run(directory, scales):
    config = json.loads((directory / "effective_config.json").read_text())
    gate = json.loads((directory / "d5h1_gate_audit_summary.json").read_text())
    formal = json.loads((directory / "task_d_d5h1_formal_run_summary.json").read_text())
    if not gate.get("passed") or not formal.get("pass") or not formal.get("formal_eligibility"):
        raise RuntimeError("D5I0 requires a passed formal D5H1 run: " + str(directory))
    v3, d5 = read_bag(directory / "task_d_d5h1.bag")
    matched_d5, matched_v3 = match_rows(d5, v3)
    match_fraction = len(matched_d5) / max(len(d5), len(v3))
    if match_fraction < 0.99:
        raise RuntimeError("D5H1/V3 cross-topic match below 99%: " + str(directory))
    mask = ((matched_d5[:, 0] >= WINDOW_START_S) &
            (matched_d5[:, 0] <= WINDOW_END_S))
    d5, v3 = matched_d5[mask], matched_v3[mask]
    if len(d5) < 100 or np.any(np.diff(d5[:, 0]) <= 0.0):
        raise RuntimeError("invalid D5I0 analysis window: " + str(directory))
    time = d5[:, 0]
    weights = trapezoid_weights(time)
    regressor = v3[:, 48:108].reshape(-1, 6, 10)
    sigma = v3[:, 42:48]
    raw = d5[:, 15:25]
    baseline = d5[:, 25:35]
    applied = d5[:, 35:45]
    raw_expected = np.einsum("nij,ni->nj", regressor, sigma)
    raw_closure = float(np.max(np.abs(raw - raw_expected)))
    applied_closure = float(np.max(np.abs(applied - (raw - baseline))))
    corrected = bool(config["enableGaitPhaseBaselineCorrection"])
    mode_contract = bool(corrected == ("corrected" in config["group"]))
    regressor_design = weighted_regressor_design(regressor, weights, scales)
    update_design = weighted_signal_design(applied, weights, scales, center=False)
    centered_update_design = weighted_signal_design(applied, weights, scales, center=True)
    return {
        "directory": directory, "group": config["group"], "run": directory.name,
        "time": time, "weights": weights, "duration": float(np.sum(weights)),
        "mode": d5[:, 10].astype(int), "phase_bin": d5[:, 9].astype(int),
        "correction_applied": d5[:, 7] > 0.5,
        "regressor": regressor, "raw": raw, "applied": applied,
        "regressor_design": regressor_design,
        "update_design": update_design,
        "centered_update_design": centered_update_design,
        "match_fraction": match_fraction, "raw_closure": raw_closure,
        "applied_closure": applied_closure, "mode_contract": mode_contract,
    }


def serializable_spectrum(metrics):
    return {
        "singular_values": metrics["singular_values"].tolist(),
        "singular_ratios": metrics["singular_ratios"].tolist(),
        "ranks": metrics["ranks"],
        "condition_number": metrics["condition_number"],
        "effective_rank": metrics["effective_rank"],
    }


def top_direction(vector, count=4):
    order = np.argsort(np.abs(vector))[::-1][:count]
    return [{"parameter": PARAMETERS[index], "coefficient": float(vector[index])}
            for index in order]


def write_csv(path, rows):
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("d5h1_root", type=Path)
    parser.add_argument("--output-directory", type=Path, required=True)
    args = parser.parse_args()
    root = args.d5h1_root.resolve()
    output = args.output_directory.resolve()
    output.mkdir(parents=True, exist_ok=False)

    upstream_path = root / "analysis" / "stage_d5h1_formal_campaign_summary.json"
    upstream = json.loads(upstream_path.read_text())
    if not upstream.get("passed") or upstream.get("run_count") != 12:
        raise RuntimeError("D5I0 requires the passed 12-run D5H1 v3 campaign")
    reference_summary = json.loads((root / "uncertain_corrected" / "run_01" /
                                    "task_d_d5h1_formal_run_summary.json").read_text())
    scales = np.asarray(reference_summary["physical_coordinate_scales"], dtype=float)
    truth = np.asarray(reference_summary["plant_pi_star_10d"], dtype=float)
    if scales.shape != (10,) or truth.shape != (10,) or np.any(scales <= 0.0):
        raise RuntimeError("invalid frozen D5H1 physical coordinate convention")
    # D5A used m*L^2 for all inertia coordinates, whereas D5H1 froze
    # max(J_payload_com)=0.12 kg m^2 for reporting.  The fifth direction is
    # close enough to 1e-3 that this declared coordinate choice can change a
    # thresholded rank by one.  Preserve D5H1 as primary and report the D5A
    # convention as a sensitivity check instead of silently mixing them.
    characteristic_length = scales[1] / scales[0]
    d5a_scales = np.asarray(
        [scales[0]] + [scales[1]] * 3 +
        [scales[0] * characteristic_length ** 2] * 6)
    # A global force scale cancels from singular-value ratios.  Taking F0=1
    # leaves the physically relevant torque scale T0=F0*L.  This converts
    # torque rows to force-equivalent rows before computing information.
    wrench_row_scales = np.asarray(
        [1.0, 1.0, 1.0, characteristic_length, characteristic_length,
         characteristic_length])

    runs = {group: [load_run(root / group / run, scales) for run in RUNS]
            for group in GROUPS}
    closure_max = max(max(run["raw_closure"], run["applied_closure"])
                      for group in GROUPS for run in runs[group])
    all_contracts = all(run["mode_contract"] for group in GROUPS for run in runs[group])

    groups, per_run_rows, spectrum_rows, column_rows, weak_rows = {}, [], [], [], []
    for group in GROUPS:
        regressor_matrix = np.vstack([run["regressor_design"] for run in runs[group]])
        d5a_regressor_matrix = np.vstack([
            weighted_regressor_design(run["regressor"], run["weights"], d5a_scales)
            for run in runs[group]])
        dimensionless_regressor_matrix = np.vstack([
            weighted_regressor_design(run["regressor"], run["weights"],
                                      d5a_scales, wrench_row_scales)
            for run in runs[group]])
        update_matrix = np.vstack([run["update_design"] for run in runs[group]])
        centered_matrix = np.vstack([run["centered_update_design"] for run in runs[group]])
        regressor_metrics = spectrum(regressor_matrix)
        d5a_regressor_metrics = spectrum(d5a_regressor_matrix)
        dimensionless_regressor_metrics = spectrum(dimensionless_regressor_matrix)
        direction_metrics = column_normalized_spectrum(dimensionless_regressor_matrix)
        update_metrics = spectrum(update_matrix)
        centered_update_metrics = spectrum(centered_matrix)
        historical_projection = (
            observable_projection(regressor_metrics, truth, scales)
            if group.startswith("uncertain") else None)
        dimensionless_projection = (
            observable_projection(dimensionless_regressor_metrics, truth, d5a_scales)
            if group.startswith("uncertain") else None)
        groups[group] = {
            "payload_scaled_regressor": serializable_spectrum(regressor_metrics),
            "d5a_characteristic_scaled_regressor":
                serializable_spectrum(d5a_regressor_metrics),
            "fully_dimensionless_regressor":
                serializable_spectrum(dimensionless_regressor_metrics),
            "column_normalized_regressor": {
                **serializable_spectrum(direction_metrics),
                "maximum_coherence": direction_metrics["maximum_coherence"],
                "maximum_coherence_pair": [PARAMETERS[index] for index in
                                            direction_metrics["maximum_coherence_pair"]],
            },
            "dimensionless_applied_update_drive": serializable_spectrum(update_metrics),
            "centered_dimensionless_applied_update_drive":
                serializable_spectrum(centered_update_metrics),
            "historical_pi_star_projection_at_1e_3": historical_projection,
            "fully_dimensionless_pi_star_projection_at_1e_3":
                dimensionless_projection,
            "weakest_fully_dimensionless_directions": [
                top_direction(dimensionless_regressor_metrics
                              ["right_singular_vectors"][-index])
                for index in range(1, 4)],
        }
        for index, ratio in enumerate(regressor_metrics["singular_ratios"], 1):
            spectrum_rows.append({
                "group": group, "index": index,
                "payload_scaled_regressor_ratio": ratio,
                "d5a_characteristic_scaled_regressor_ratio":
                    d5a_regressor_metrics["singular_ratios"][index - 1],
                "fully_dimensionless_regressor_ratio":
                    dimensionless_regressor_metrics["singular_ratios"][index - 1],
                "applied_update_drive_ratio": update_metrics["singular_ratios"][index - 1],
                "centered_applied_update_drive_ratio":
                    centered_update_metrics["singular_ratios"][index - 1],
            })
        total_duration = sum(run["duration"] for run in runs[group])
        column_rms = (np.linalg.norm(dimensionless_regressor_matrix, axis=0) /
                      math.sqrt(total_duration))
        for index, name in enumerate(PARAMETERS):
            column_rows.append({"group": group, "parameter": name,
                                "dimensionless_column_rms": column_rms[index]})
        for weak_index in range(1, 4):
            vector = dimensionless_regressor_metrics["right_singular_vectors"][-weak_index]
            for parameter_index, name in enumerate(PARAMETERS):
                weak_rows.append({
                    "group": group, "weak_order": weak_index,
                    "singular_ratio": dimensionless_regressor_metrics
                        ["singular_ratios"][-weak_index],
                    "parameter": name, "dimensionless_coefficient": vector[parameter_index],
                })
        for run in runs[group]:
            run_regressor = spectrum(weighted_regressor_design(
                run["regressor"], run["weights"], d5a_scales,
                wrench_row_scales))
            run_update = spectrum(run["centered_update_design"])
            per_run_rows.append({
                "group": group, "run": run["run"], "samples": len(run["time"]),
                "start_s": run["time"][0], "end_s": run["time"][-1],
                "duration_s": run["duration"],
                "regressor_rank_1e_3": run_regressor["ranks"]["1e-03"],
                "regressor_smallest_ratio": run_regressor["singular_ratios"][-1],
                "centered_update_rank_1e_3": run_update["ranks"]["1e-03"],
                "centered_update_smallest_ratio": run_update["singular_ratios"][-1],
                "cross_topic_match_fraction": run["match_fraction"],
                "closure_max_abs_error": max(run["raw_closure"], run["applied_closure"]),
            })

    phase_rows = []
    group = "uncertain_corrected"
    for phase in range(24):
        phase_design = []
        sample_count = 0
        for run in runs[group]:
            mask = ((run["time"] >= PHASE_START_S) &
                    (run["phase_bin"] == phase) & run["correction_applied"])
            sample_count += int(np.sum(mask))
            if np.sum(mask):
                # Phase samples are disjoint repeated intervals.  Use the
                # local median sample period instead of integrating gaps.
                dt = float(np.median(np.diff(run["time"])))
                weights = np.full(int(np.sum(mask)), dt)
                phase_design.append(weighted_regressor_design(
                    run["regressor"][mask], weights, d5a_scales,
                    wrench_row_scales))
        metrics = spectrum(np.vstack(phase_design))
        phase_rows.append({
            "phase_bin": phase, "sample_count": sample_count,
            "rank_1e_2": metrics["ranks"]["1e-02"],
            "rank_1e_3": metrics["ranks"]["1e-03"],
            "rank_1e_6": metrics["ranks"]["1e-06"],
            "smallest_ratio": metrics["singular_ratios"][-1],
        })

    write_csv(output / "d5i0_per_run.csv", per_run_rows)
    write_csv(output / "d5i0_group_spectra.csv", spectrum_rows)
    write_csv(output / "d5i0_scaled_column_action_rms.csv", column_rows)
    write_csv(output / "d5i0_weak_directions.csv", weak_rows)
    write_csv(output / "d5i0_uncertain_corrected_phase_rank.csv", phase_rows)

    colors = {"nominal_raw": "#4c4c4c", "nominal_corrected": "#2ca02c",
              "uncertain_raw": "#d62728", "uncertain_corrected": "#1f77b4"}
    x = np.arange(1, 11)
    figure, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for group in GROUPS:
        axes[0].semilogy(x, groups[group]["fully_dimensionless_regressor"]
                         ["singular_ratios"], "o-", label=group, color=colors[group])
        axes[1].semilogy(x, groups[group]["centered_dimensionless_applied_update_drive"]
                         ["singular_ratios"], "o-", label=group, color=colors[group])
    for axis in axes:
        axis.axhline(PRIMARY_THRESHOLD, color="k", ls="--", lw=0.8)
        axis.grid(True, which="both", alpha=0.3)
        axis.legend(fontsize=8)
    axes[0].set_ylabel("relative singular value")
    axes[0].set_title("Fully dimensionless 6x10 regressor information")
    axes[1].set_ylabel("relative singular value")
    axes[1].set_xlabel("singular-value index")
    axes[1].set_title("Centered dimensionless Eq.(8) applied-drive directions")
    axes[1].set_xticks(x)
    figure.tight_layout()
    figure.savefig(output / "figure_D5I0_01_information_and_drive_spectra.png", dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(11, 5.5))
    width = 0.19
    for offset, group in enumerate(GROUPS):
        values = [row["dimensionless_column_rms"] for row in column_rows
                  if row["group"] == group]
        axis.bar(np.arange(10) + (offset - 1.5) * width, values, width,
                 label=group, color=colors[group])
    axis.set_yscale("log")
    axis.set_xticks(range(10)); axis.set_xticklabels(PARAMETERS)
    axis.set_ylabel("dimensionless regressor-column RMS")
    axis.set_title("D5I0 current-trajectory dimensionless sensitivity imbalance")
    axis.grid(True, axis="y", which="both", alpha=0.3); axis.legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(output / "figure_D5I0_02_scaled_column_sensitivity.png", dpi=180)
    plt.close(figure)

    corrected_metrics = spectrum(np.vstack([
        weighted_regressor_design(run["regressor"], run["weights"],
                                  d5a_scales, wrench_row_scales)
        for run in runs["uncertain_corrected"]]))
    figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for index, axis in enumerate(axes, 1):
        vector = corrected_metrics["right_singular_vectors"][-index]
        axis.bar(range(10), vector, color="#1f77b4")
        axis.set_ylabel("coefficient")
        axis.set_title("weak order {} (relative singular value {:.3g})".format(
            index, corrected_metrics["singular_ratios"][-index]))
        axis.grid(True, axis="y", alpha=0.3)
    axes[-1].set_xticks(range(10)); axes[-1].set_xticklabels(PARAMETERS)
    figure.suptitle("Uncertain PhaseCorrected: weakest dimensionless combinations")
    figure.tight_layout()
    figure.savefig(output / "figure_D5I0_03_weak_parameter_combinations.png", dpi=180)
    plt.close(figure)

    current = groups["uncertain_corrected"]["payload_scaled_regressor"]
    current_dimensionless = groups["uncertain_corrected"]["fully_dimensionless_regressor"]
    projection = groups["uncertain_corrected"][
        "fully_dimensionless_pi_star_projection_at_1e_3"]
    current_smallest = current["singular_ratios"][-1]
    admission = {
        "purpose": "pre-registered screen for a future richer-excitation candidate; not a convergence theorem",
        "primary": {
            "fully_dimensionless_full_window_rank_at_1e_3": 10,
            "fully_dimensionless_smallest_ratio_minimum": PRIMARY_THRESHOLD,
            "pi_star_dimensionless_observable_fraction_minimum": 0.90,
        },
        "baseline_relative": {
            "smallest_ratio_improvement_minimum_factor": 10.0,
            "weakest_six_coordinates_action_rms_must_not_decrease": True,
        },
        "required_validity": [
            "deterministic target/gait schedule and controller configuration hashes",
            "desired/actual pose, velocity, acceleration and angular-rate diagnostics",
            "same 10D regressor in Eq.(8) and Eq.(11)",
            "no NaN, solver crash, unexpected contact mode, or target timing mismatch",
            "nominal PhaseBaselineCorrected false-estimate robustness retained",
        ],
    }
    checks = {
        "upstream_d5h1_formal_v3_passed": True,
        "twelve_runs_loaded": sum(len(value) for value in runs.values()) == 12,
        "all_cross_topic_match_at_least_99_percent": all(
            run["match_fraction"] >= 0.99 for group in GROUPS for run in runs[group]),
        "raw_and_applied_drive_closure_le_1e_12": closure_max <= 1.0e-12,
        "group_mode_contracts_match": all_contracts,
        "no_new_online_experiment_or_controller_change": True,
    }
    summary = {
        "schema_version": 1,
        "stage": "D5I0_richer_excitation_10D_identifiability_design_audit",
        "passed": bool(all(checks.values())),
        "scope": "read-only audit of frozen D5H1 v3 data plus next-stage design; no new target or online run",
        "checks": checks,
        "analysis_window_s": [WINDOW_START_S, WINDOW_END_S],
        "physical_coordinate_scales": scales.tolist(),
        "d5a_characteristic_coordinate_scales": d5a_scales.tolist(),
        "wrench_row_scales_force_torque": wrench_row_scales.tolist(),
        "pi_star_10d": truth.tolist(),
        "groups": groups,
        "current_uncertain_corrected_headline": {
            "payload_scaled_rank_1e_3": current["ranks"]["1e-03"],
            "payload_scaled_smallest_ratio": current_smallest,
            "condition_number": current["condition_number"],
            "pi_star_dimensionless_observable_fraction_at_1e_3":
                projection["dimensionless_observable_fraction"],
            "minimum_factor_needed_to_cross_1e_3":
                float(PRIMARY_THRESHOLD /
                      current_dimensionless["singular_ratios"][-1]),
            "phase_rank_1e_3_min": min(row["rank_1e_3"] for row in phase_rows),
            "phase_rank_1e_3_max": max(row["rank_1e_3"] for row in phase_rows),
            "d5a_characteristic_scaled_rank_1e_3": groups["uncertain_corrected"]
                ["d5a_characteristic_scaled_regressor"]["ranks"]["1e-03"],
            "d5a_characteristic_scaled_smallest_ratio": groups["uncertain_corrected"]
                ["d5a_characteristic_scaled_regressor"]["singular_ratios"][-1],
            "fully_dimensionless_rank_1e_3": groups["uncertain_corrected"]
                ["fully_dimensionless_regressor"]["ranks"]["1e-03"],
            "fully_dimensionless_smallest_ratio": groups["uncertain_corrected"]
                ["fully_dimensionless_regressor"]["singular_ratios"][-1],
        },
        "reference_pipeline_audit": {
            "current_target": "+0.10 m x, 1.2-1.4 s, then fixed pose; static_walk 1.2 s cycle",
            "current_derivative_contract": (
                "piecewise-linear pose secants produce desired linear/angular velocity; "
                "desired linear/angular acceleration is zero inside every segment"),
            "consequence": (
                "larger x displacement alone mainly increases duration and does not deliberately "
                "excite the six inertia coordinates; multi-knot pose commands alone still omit "
                "feedforward angular acceleration"),
            "phase_baseline_compatibility": (
                "the frozen D5H1 gait-phase template was trained on the old x-only reference and "
                "must not be assumed valid for a nonstationary richer reference; a new nominal-only "
                "calibration/provenance gate is required before corrected online comparison"),
        },
        "future_candidate_admission": admission,
        "scale_interpretation": {
            "historical_compatibility": (
                "payload_scaled_regressor and d5a_characteristic_scaled_regressor retain mixed "
                "force/torque row units solely for comparison with D5A"),
            "primary_future_design_metric": (
                "fully_dimensionless_regressor uses parameter scales "
                "[m,mL x3,mL^2 x6] and wrench row scales [F x3,F*L x3]"),
            "rule": "never select a trajectory by a rank reported under only one unstated scaling",
        },
        "design_decision": (
            "Do not run a formal richer-excitation campaign yet. First implement a default-OFF, "
            "deterministic candidate reference that logs exact desired v/a/omega/alpha, then pass "
            "offline ideal-reference and estimator-OFF safety/Gramian gates, then freeze a "
            "nominal-only correction template before any corrected estimator comparison."),
        "scientific_boundary": [
            "Finite-horizon regressor rank is not persistent-excitation proof.",
            "Update-drive rank is an estimator-trajectory diagnostic, not a plant-identifiability theorem.",
            "This stage neither changes nor validates Gamma and does not claim 10D convergence.",
        ],
        "provenance": {
            "d5h1_root": str(root), "upstream_summary": str(upstream_path),
            "upstream_summary_sha256": sha256(upstream_path),
            "analysis_script": str(Path(__file__).resolve()),
            "analysis_script_sha256": sha256(Path(__file__).resolve()),
        },
    }
    (output / "stage_d5i0_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "passed": summary["passed"],
        "headline": summary["current_uncertain_corrected_headline"],
        "output": str(output),
    }, indent=2, sort_keys=True))
    raise SystemExit(0 if summary["passed"] else 1)


if __name__ == "__main__":
    main()
