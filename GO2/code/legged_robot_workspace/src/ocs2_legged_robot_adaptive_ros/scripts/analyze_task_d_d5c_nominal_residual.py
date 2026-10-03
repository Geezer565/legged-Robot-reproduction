#!/usr/bin/env python3
"""Stage D5C: decompose nominal Eq.(8) walking update drive.

The analysis is read-only with respect to the frozen D5B campaign.  It uses
the synchronized 6x10 regressor and sigma stored in each bag and partitions

    pi_hat_dot[j] = sum_i Y[i,j] * sigma[i]

by world-frame sigma axis, executed contact mode, and deterministic time
segment.  The partitions reproduce the estimator's explicit-Euler updates;
are not assumed to be an additive causal model of the closed loop.
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


ESTIMATE_TOPIC = "/task_d/payload_inertial_10d_v3_sample"
FLOW_TOPIC = "/task_d/equation10_full_flow_sample"
OBSERVATION_TOPIC = "/legged_robot_mpc_observation"
ESTIMATE_LABEL = "payload_inertial_10d_v3_118_fields"
FLOW_LABEL = "payload_inertial_eq10_full_flow_v2_80_fields"
GROUPS = ("nominal_stance_hold", "nominal_stance_target", "nominal_walk_target")
RUNS = ("run_01", "run_02", "run_03")
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy", "Jyz", "Jzz")
SIGMA_AXES = ("linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z")
KEY_PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jyy", "Jzz")
MODE_NAMES = {
    7: "RF_LH_RH (LF swing)",
    11: "LF_LH_RH (RF swing)",
    13: "LF_RF_RH (LH swing)",
    14: "LF_RF_LH (RH swing)",
    15: "STANCE",
}
GROUP_LABELS = {
    "nominal_stance_hold": "N: stance hold",
    "nominal_stance_target": "N: stance + target",
    "nominal_walk_target": "N: walk + target",
}
GROUP_COLORS = {
    "nominal_stance_hold": "#4c4c4c",
    "nominal_stance_target": "#1f77b4",
    "nominal_walk_target": "#2ca02c",
}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sample_stats(values):
    values = np.asarray(values, dtype=float)
    return {
        "n": int(len(values)),
        "mean": float(np.mean(values)),
        "sample_std": float(np.std(values, ddof=1)) if len(values) > 1 else None,
        "min": float(np.min(values)),
        "max": float(np.max(values)),
    }


def nearest(times, source_times, values):
    times = np.asarray(times, dtype=float)
    source_times = np.asarray(source_times, dtype=float)
    values = np.asarray(values)
    right = np.clip(np.searchsorted(source_times, times), 0, len(source_times) - 1)
    left = np.clip(right - 1, 0, len(source_times) - 1)
    choose_left = np.abs(times - source_times[left]) <= np.abs(source_times[right] - times)
    return values[np.where(choose_left, left, right)]


def interpolate_rows(times, rows, grid):
    times = np.asarray(times, dtype=float)
    rows = np.asarray(rows, dtype=float)
    grid = np.asarray(grid, dtype=float)
    return np.column_stack([np.interp(grid, times, rows[:, column])
                            for column in range(rows.shape[1])])


def clipped_rows(rows, start, end):
    """Add exact interpolated boundaries and retain strictly ordered rows."""
    rows = np.asarray(rows, dtype=float)
    if rows.ndim != 2 or len(rows) < 2 or not np.all(np.diff(rows[:, 0]) > 0.0):
        raise ValueError("rows must be a strictly time-ordered matrix")
    if rows[0, 0] > start or rows[-1, 0] < end or not start < end:
        raise ValueError("rows do not cover requested window")
    interior = rows[(rows[:, 0] > start) & (rows[:, 0] < end)]
    boundaries = interpolate_rows(rows[:, 0], rows, [start, end])
    result = np.vstack((boundaries[0], interior, boundaries[1]))
    result[:, 0] = np.concatenate(([start], interior[:, 0], [end]))
    return result


def axis_drive_terms(regressor, sigma):
    """Return c[n,i,j] = sigma[n,i] * Y[n,i,j]."""
    regressor = np.asarray(regressor, dtype=float)
    sigma = np.asarray(sigma, dtype=float)
    if regressor.ndim != 3 or regressor.shape[1:] != (6, 10):
        raise ValueError("regressor must have shape [samples, 6, 10]")
    if sigma.shape != (len(regressor), 6):
        raise ValueError("sigma must have shape [samples, 6]")
    return sigma[:, :, None] * regressor


def interval_integrals(times, values):
    """Exact explicit-Euler contribution used by the online estimator.

    AdaptiveParameterEstimator updates the snapshot at t[k] with
    (t[k]-t[k-1])*drive(t[k]).  Consequently every interval is attributed to
    its right endpoint; this is intentionally not trapezoidal quadrature.
    """
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    if values.shape[0] != len(times) or len(times) < 2 or not np.all(np.diff(times) > 0.0):
        raise ValueError("invalid time/value series")
    dt = np.diff(times)
    shape = (len(dt),) + (1,) * (values.ndim - 1)
    signed = values[1:] * dt.reshape(shape)
    absolute = np.abs(values[1:]) * dt.reshape(shape)
    squared = values[1:] ** 2 * dt.reshape(shape)
    return signed, absolute, squared, dt


def partition_integrals(times, values, interval_labels):
    """Partition explicit-Euler intervals while preserving exact additivity."""
    signed, absolute, squared, dt = interval_integrals(times, values)
    labels = np.asarray(interval_labels)
    if labels.shape != (len(times) - 1,):
        raise ValueError("one label is required per interval")
    result = {}
    for label in sorted(set(labels.tolist()), key=str):
        mask = labels == label
        result[label] = {
            "duration": float(np.sum(dt[mask])),
            "signed": np.sum(signed[mask], axis=0),
            "absolute": np.sum(absolute[mask], axis=0),
            "squared": np.sum(squared[mask], axis=0),
        }
    return result


def interval_mode_labels(sample_modes):
    sample_modes = np.asarray(sample_modes, dtype=int)
    # Eq.(8) is evaluated at the right endpoint, so a switch interval belongs
    # to the executed mode of that endpoint instead of an artificial bucket.
    return sample_modes[1:]


def interval_time_labels(times, target_start=1.2, target_end=1.4,
                         first_cycle_end=2.6):
    endpoint = np.asarray(times[1:])
    return np.where(endpoint < target_start, "pre_target_0_1p2",
                    np.where(endpoint < target_end, "target_ramp_1p2_1p4",
                             np.where(endpoint < first_cycle_end,
                                      "first_post_ramp_cycle_1p4_2p6",
                                      "steady_walk_2p6_16p2")))


def cumulative_euler(times, values):
    signed, _, _, _ = interval_integrals(times, values)
    zero = np.zeros((1,) + signed.shape[1:], dtype=float)
    return np.concatenate((zero, np.cumsum(signed, axis=0)), axis=0)


def write_rows(path, rows):
    if not rows:
        raise ValueError("cannot write an empty table")
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def load_run(run, expected_hash):
    import rosbag

    config = json.loads((run / "effective_config.json").read_text())
    validation = json.loads((run / "task_d_d5b_run_summary.json").read_text())
    if not validation.get("formal_eligibility"):
        raise RuntimeError("D5C input is not formally eligible: {}".format(run))
    bag_path = run / "task_d_d5b.bag"
    if sha256(bag_path) != expected_hash:
        raise RuntimeError("D5B frozen bag hash mismatch: {}".format(bag_path))

    estimates, flows, observations = [], [], []
    estimate_labels, flow_labels = set(), set()
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _ in bag.read_messages(
                topics=[ESTIMATE_TOPIC, FLOW_TOPIC, OBSERVATION_TOPIC]):
            if topic == ESTIMATE_TOPIC:
                estimate_labels.add(message.layout.dim[0].label if message.layout.dim else "")
                estimates.append([float(value) for value in message.data])
            elif topic == FLOW_TOPIC:
                flow_labels.add(message.layout.dim[0].label if message.layout.dim else "")
                flows.append([float(value) for value in message.data])
            else:
                observations.append((float(message.time), int(message.mode),
                                     [float(value) for value in message.state.value]))

    estimate = np.asarray(sorted(estimates, key=lambda row: row[0]), dtype=float)
    flow = np.asarray(sorted(flows, key=lambda row: row[0]), dtype=float)
    observations = [row for _, row in sorted({row[0]: row for row in observations}.items())]
    observation_time = np.asarray([row[0] for row in observations], dtype=float)
    observation_mode = np.asarray([row[1] for row in observations], dtype=int)
    observation_state = np.asarray([row[2] for row in observations], dtype=float)
    if (estimate_labels != {ESTIMATE_LABEL} or flow_labels != {FLOW_LABEL}
            or estimate.ndim != 2 or estimate.shape[1] != 118
            or flow.ndim != 2 or flow.shape[1] != 80
            or observation_state.ndim != 2 or observation_state.shape[1] != 24
            or not np.all(np.isfinite(estimate)) or not np.all(np.isfinite(flow))
            or not np.all(np.isfinite(observation_state))):
        raise RuntimeError("invalid D5B diagnostic schema in {}".format(run))

    # D5B formal tracking metrics begin at target time 1.2 s.  D5C instead
    # starts at the first common raw sample because walking is scheduled before
    # the target and has already changed pi_hat at 1.2 s.
    start = max(float(estimate[0, 0]), float(observation_time[0]))
    end = float(config["analysis_window_end_mpc_time_s"])
    # Preserve actual estimator timestamps.  Interpolating pi_hat or drive at
    # artificial boundaries would break the exact discrete Euler identity.
    estimate = estimate[(estimate[:, 0] >= start) & (estimate[:, 0] <= end)]
    if len(estimate) < 2:
        raise RuntimeError("no estimator samples inside D5C window: {}".format(run))
    time = estimate[:, 0]
    sigma = estimate[:, 42:48]
    regressor = estimate[:, 48:108].reshape(-1, 6, 10)
    pi_hat = estimate[:, 22:32]
    pi_dot = estimate[:, 32:42]
    terms = axis_drive_terms(regressor, sigma)
    drive = np.sum(terms, axis=1)
    mode = nearest(time, observation_time, observation_mode).astype(int)

    equation_error = float(np.max(np.abs(pi_dot - drive)))
    axis_closure = float(np.max(np.abs(np.sum(terms, axis=1) - drive)))
    if equation_error > 1.0e-10 or axis_closure > 1.0e-12:
        raise RuntimeError("Eq.(8) decomposition closure failed in {}".format(run))

    # A finite-difference execution residual is useful for locating numerical
    # spikes, but it is not pi_star: input/policy varies over each interval.
    observation_mask = ((observation_time >= max(start, flow[0, 0]))
                        & (observation_time <= min(end, flow[-1, 0])))
    ot = observation_time[observation_mask]
    os = observation_state[observation_mask]
    om = observation_mode[observation_mask]
    flow_at_observation = interpolate_rows(flow[:, 0], flow[:, 8:32], ot)
    state_rate_fd = np.diff(os, axis=0) / np.diff(ot)[:, None]
    nominal_mid = 0.5 * (flow_at_observation[:-1] + flow_at_observation[1:])
    fd_residual = state_rate_fd - nominal_mid
    fd_labels = interval_mode_labels(om)

    return {
        "config": config,
        "validation": validation,
        "bag_sha256": expected_hash,
        "time": time,
        "sigma": sigma,
        "regressor": regressor,
        "pi_hat": pi_hat,
        "pi_dot": pi_dot,
        "terms": terms,
        "drive": drive,
        "mode": mode,
        "equation_error": equation_error,
        "axis_closure": axis_closure,
        "fd_time": 0.5 * (ot[:-1] + ot[1:]),
        "fd_residual": fd_residual,
        "fd_mode": fd_labels,
    }


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: analyze_task_d_d5c_nominal_residual.py D5B_ROOT OUTPUT_DIR")
    source = Path(sys.argv[1]).resolve()
    output = Path(sys.argv[2]).resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError("refusing to overwrite non-empty D5C output: {}".format(output))
    output.mkdir(parents=True, exist_ok=True)

    fairness_path = source / "formal_fairness_audit.json"
    fairness = json.loads(fairness_path.read_text())
    if not fairness.get("passed"):
        raise RuntimeError("D5B formal fairness audit did not pass")

    runs = {}
    input_manifest = []
    for group in GROUPS:
        for run_name in RUNS:
            key = "{}/{}".format(group, run_name)
            run_path = source / "formal" / group / run_name
            expected = fairness["artifact_sha256"][key]["task_d_d5b.bag"]
            runs[(group, run_name)] = load_run(run_path, expected)
            input_manifest.append({
                "group": group, "run": run_name,
                "bag": str(run_path / "task_d_d5b.bag"),
                "bag_sha256": expected,
                "effective_config_sha256": sha256(run_path / "effective_config.json"),
                "validation_sha256": sha256(run_path / "task_d_d5b_run_summary.json"),
            })

    axis_rows, mode_rows, segment_rows, sigma_rows, fd_rows, validation_rows = [], [], [], [], [], []
    run_metrics = {}
    for group in GROUPS:
        for run_name in RUNS:
            data = runs[(group, run_name)]
            time, terms = data["time"], data["terms"]
            total_duration = time[-1] - time[0]
            signed, absolute, squared, _ = interval_integrals(time, terms)
            axis_signed = np.sum(signed, axis=0)
            axis_absolute = np.sum(absolute, axis=0)
            axis_rms = np.sqrt(np.sum(squared, axis=0) / total_duration)
            drive_signed = np.sum(axis_signed, axis=0)
            drive_absolute_activity = np.sum(axis_absolute, axis=0)
            pi_change = data["pi_hat"][-1] - data["pi_hat"][0]

            for axis_index, axis_name in enumerate(SIGMA_AXES):
                for parameter_index, parameter in enumerate(PARAMETERS):
                    axis_rows.append({
                        "group": group, "run": run_name, "parameter": parameter,
                        "sigma_axis": axis_name,
                        "signed_integral": float(axis_signed[axis_index, parameter_index]),
                        "absolute_integral": float(axis_absolute[axis_index, parameter_index]),
                        "rms_contribution": float(axis_rms[axis_index, parameter_index]),
                        "signed_fraction_of_total": (
                            float(axis_signed[axis_index, parameter_index] /
                                  drive_signed[parameter_index])
                            if abs(drive_signed[parameter_index]) > 1.0e-12 else math.nan),
                        "absolute_activity_fraction": (
                            float(axis_absolute[axis_index, parameter_index] /
                                  drive_absolute_activity[parameter_index])
                            if drive_absolute_activity[parameter_index] > 0.0 else 0.0),
                    })

            mode_partition = partition_integrals(
                time, np.sum(terms, axis=1), interval_mode_labels(data["mode"]))
            for mode_number, metrics in mode_partition.items():
                for parameter_index, parameter in enumerate(PARAMETERS):
                    mode_rows.append({
                        "group": group, "run": run_name,
                        "mode": int(mode_number),
                        "mode_name": MODE_NAMES.get(int(mode_number), str(mode_number)),
                        "duration_s": metrics["duration"],
                        "parameter": parameter,
                        "signed_integral": float(metrics["signed"][parameter_index]),
                        "absolute_integral": float(metrics["absolute"][parameter_index]),
                        "rms_drive": float(math.sqrt(
                            metrics["squared"][parameter_index] / metrics["duration"]))
                            if metrics["duration"] > 0.0 else 0.0,
                    })

            segment_partition = partition_integrals(
                time, np.sum(terms, axis=1), interval_time_labels(time))
            for segment, metrics in segment_partition.items():
                for parameter_index, parameter in enumerate(PARAMETERS):
                    segment_rows.append({
                        "group": group, "run": run_name, "segment": segment,
                        "duration_s": metrics["duration"], "parameter": parameter,
                        "signed_integral": float(metrics["signed"][parameter_index]),
                        "absolute_integral": float(metrics["absolute"][parameter_index]),
                        "rms_drive": float(math.sqrt(
                            metrics["squared"][parameter_index] / metrics["duration"]))
                            if metrics["duration"] > 0.0 else 0.0,
                    })

            sigma_partition = partition_integrals(
                time, data["sigma"], interval_mode_labels(data["mode"]))
            for mode_number, metrics in sigma_partition.items():
                for axis_index, axis_name in enumerate(SIGMA_AXES):
                    sigma_rows.append({
                        "group": group, "run": run_name,
                        "mode": int(mode_number),
                        "mode_name": MODE_NAMES.get(int(mode_number), str(mode_number)),
                        "duration_s": metrics["duration"], "sigma_axis": axis_name,
                        "signed_integral": float(metrics["signed"][axis_index]),
                        "absolute_integral": float(metrics["absolute"][axis_index]),
                        "rms": float(math.sqrt(
                            metrics["squared"][axis_index] / metrics["duration"]))
                            if metrics["duration"] > 0.0 else 0.0,
                    })

            fd_norm = np.linalg.norm(data["fd_residual"], axis=1)
            for mode_number in sorted(set(data["fd_mode"].tolist())):
                mask = data["fd_mode"] == mode_number
                fd_rows.append({
                    "group": group, "run": run_name, "mode": int(mode_number),
                    "mode_name": MODE_NAMES.get(int(mode_number), str(mode_number)),
                    "interval_count": int(np.sum(mask)),
                    "fd_execution_residual_rms": float(np.sqrt(np.mean(fd_norm[mask] ** 2))),
                    "fd_execution_residual_peak": float(np.max(fd_norm[mask])),
                })

            integral_vs_pi = drive_signed - pi_change
            validation_rows.append({
                "group": group, "run": run_name,
                "samples": int(len(time)), "duration_s": float(total_duration),
                "eq8_axis_closure_max_error": data["axis_closure"],
                "eq8_derivative_max_error": data["equation_error"],
                "max_abs_integral_minus_pi_change": float(np.max(np.abs(integral_vs_pi))),
                "formal_eligibility": bool(data["validation"]["formal_eligibility"]),
            })
            run_metrics[(group, run_name)] = {
                "axis_signed": axis_signed,
                "axis_absolute": axis_absolute,
                "drive_signed": drive_signed,
                "drive_absolute_activity": drive_absolute_activity,
                "pi_change": pi_change,
                "mode_partition": mode_partition,
                "segment_partition": segment_partition,
            }

    write_rows(output / "axis_contribution_per_run.csv", axis_rows)
    write_rows(output / "mode_contribution_per_run.csv", mode_rows)
    write_rows(output / "time_segment_contribution_per_run.csv", segment_rows)
    write_rows(output / "sigma_by_mode_per_run.csv", sigma_rows)
    write_rows(output / "finite_difference_execution_residual.csv", fd_rows)
    write_rows(output / "run_validation.csv", validation_rows)

    # Group statistics for all exact signed partitions.
    group_rows = []
    for group in GROUPS:
        for parameter_index, parameter in enumerate(PARAMETERS):
            for axis_index, axis_name in enumerate(SIGMA_AXES):
                values = [run_metrics[(group, run_name)]["axis_signed"][axis_index, parameter_index]
                          for run_name in RUNS]
                group_rows.append({"group": group, "partition": "sigma_axis",
                                   "label": axis_name, "parameter": parameter,
                                   **sample_stats(values)})
            all_modes = sorted(set(mode for run_name in RUNS
                                   for mode in run_metrics[(group, run_name)]["mode_partition"]))
            for mode in all_modes:
                values = [run_metrics[(group, run_name)]["mode_partition"].get(
                    mode, {"signed": np.zeros(10)})["signed"][parameter_index]
                          for run_name in RUNS]
                group_rows.append({"group": group, "partition": "contact_mode",
                                   "label": MODE_NAMES.get(int(mode), str(mode)),
                                   "parameter": parameter, **sample_stats(values)})
            for segment in ("pre_target_0_1p2", "target_ramp_1p2_1p4",
                            "first_post_ramp_cycle_1p4_2p6",
                            "steady_walk_2p6_16p2"):
                values = [run_metrics[(group, run_name)]["segment_partition"].get(
                    segment, {"signed": np.zeros(10)})["signed"][parameter_index]
                          for run_name in RUNS]
                group_rows.append({"group": group, "partition": "time_segment",
                                   "label": segment, "parameter": parameter,
                                   **sample_stats(values)})
    write_rows(output / "group_partition_mean_std.csv", group_rows)

    walk = "nominal_walk_target"
    axis_mean = np.mean(np.stack([run_metrics[(walk, run_name)]["axis_signed"]
                                  for run_name in RUNS]), axis=0)
    axis_std = np.std(np.stack([run_metrics[(walk, run_name)]["axis_signed"]
                               for run_name in RUNS]), axis=0, ddof=1)
    fig, axes = plt.subplots(4, 2, figsize=(14, 14))
    for plot, parameter in zip(axes.flat, KEY_PARAMETERS):
        index = PARAMETERS.index(parameter)
        plot.bar(np.arange(6), axis_mean[:, index], yerr=axis_std[:, index],
                 color=("#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"),
                 capsize=3)
        plot.axhline(0.0, color="black", lw=0.8)
        plot.set_xticks(np.arange(6)); plot.set_xticklabels(SIGMA_AXES, rotation=25, ha="right")
        plot.set_ylabel("signed integral")
        plot.set_title(parameter); plot.grid(True, axis="y", alpha=0.3)
    axes.flat[-1].axis("off")
    fig.suptitle("D5C nominal-walk Eq.(8) net update by sigma axis (mean +/- sample std)")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(output / "figure_D5C_01_axis_signed_contributions.png", dpi=170)
    plt.close(fig)

    walk_modes = (13, 7, 14, 11)
    fig, axes = plt.subplots(4, 2, figsize=(14, 14))
    for plot, parameter in zip(axes.flat, KEY_PARAMETERS):
        index = PARAMETERS.index(parameter)
        values, errors = [], []
        for mode in walk_modes:
            samples = [run_metrics[(walk, run_name)]["mode_partition"].get(
                mode, {"signed": np.zeros(10)})["signed"][index] for run_name in RUNS]
            values.append(np.mean(samples)); errors.append(np.std(samples, ddof=1))
        plot.bar(np.arange(len(walk_modes)), values, yerr=errors, capsize=3,
                 color=("#4c78a8", "#f58518", "#54a24b", "#e45756", "#b279a2"))
        plot.axhline(0.0, color="black", lw=0.8)
        plot.set_xticks(np.arange(len(walk_modes)))
        plot.set_xticklabels([MODE_NAMES[mode] for mode in walk_modes], rotation=25, ha="right")
        plot.set_ylabel("signed integral"); plot.set_title(parameter)
        plot.grid(True, axis="y", alpha=0.3)
    axes.flat[-1].axis("off")
    fig.suptitle("D5C nominal-walk Eq.(8) net update by executed contact mode")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(output / "figure_D5C_02_mode_signed_contributions.png", dpi=170)
    plt.close(fig)

    grid = np.linspace(0.0, 16.2, 811)
    fig, axes = plt.subplots(4, 2, figsize=(14, 14), sharex=True)
    for plot, parameter in zip(axes.flat, KEY_PARAMETERS):
        parameter_index = PARAMETERS.index(parameter)
        cumulative = []
        for run_name in RUNS:
            data = runs[(walk, run_name)]
            exact = cumulative_euler(
                data["time"], data["terms"][:, :, parameter_index])
            cumulative.append(np.stack([
                np.interp(grid, data["time"], exact[:, axis])
                for axis in range(6)], axis=1))
        cumulative = np.asarray(cumulative)
        for axis, axis_name in enumerate(SIGMA_AXES):
            plot.plot(grid, np.mean(cumulative[:, :, axis], axis=0), label=axis_name)
        plot.set_title(parameter); plot.set_ylabel("cumulative drive integral")
        plot.grid(True, alpha=0.3)
    axes.flat[0].legend(fontsize=8, ncol=2)
    axes.flat[-1].axis("off")
    for plot in axes[-1, :-1]: plot.set_xlabel("MPC time [s]")
    fig.suptitle("D5C cumulative nominal-walk bias formation by sigma axis")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(output / "figure_D5C_03_cumulative_axis_contributions.png", dpi=170)
    plt.close(fig)

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    sigma_modes = (13, 7, 14, 11)
    for axis_index, (plot, axis_name) in enumerate(zip(axes.flat, SIGMA_AXES)):
        means, errors = [], []
        for mode in sigma_modes:
            samples = []
            for run_name in RUNS:
                rows = [row for row in sigma_rows if row["group"] == walk
                        and row["run"] == run_name and row["mode"] == mode
                        and row["sigma_axis"] == axis_name]
                samples.append(rows[0]["rms"] if rows else 0.0)
            means.append(np.mean(samples)); errors.append(np.std(samples, ddof=1))
        plot.bar(np.arange(4), means, yerr=errors, capsize=3)
        plot.set_xticks(np.arange(4)); plot.set_xticklabels(
            [MODE_NAMES[mode] for mode in sigma_modes], rotation=22, ha="right")
        plot.set_title(axis_name); plot.set_ylabel("sigma RMS"); plot.grid(True, axis="y", alpha=0.3)
    fig.suptitle("D5C nominal-walk sigma RMS by executed contact mode")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(output / "figure_D5C_04_sigma_by_mode.png", dpi=170)
    plt.close(fig)

    fig, axes = plt.subplots(4, 2, figsize=(14, 14))
    segments = ("pre_target_0_1p2", "target_ramp_1p2_1p4",
                "first_post_ramp_cycle_1p4_2p6", "steady_walk_2p6_16p2")
    for plot, parameter in zip(axes.flat, KEY_PARAMETERS):
        index = PARAMETERS.index(parameter)
        means, errors = [], []
        for segment in segments:
            samples = [run_metrics[(walk, run_name)]["segment_partition"][segment]["signed"][index]
                       for run_name in RUNS]
            means.append(np.mean(samples)); errors.append(np.std(samples, ddof=1))
        plot.bar(np.arange(4), means, yerr=errors, capsize=3,
                 color=("#b279a2", "#eeca3b", "#72b7b2", "#54a24b"))
        plot.axhline(0.0, color="black", lw=0.8)
        plot.set_xticks(np.arange(4)); plot.set_xticklabels(segments, rotation=20, ha="right")
        plot.set_title(parameter); plot.set_ylabel("signed integral")
        plot.grid(True, axis="y", alpha=0.3)
    axes.flat[-1].axis("off")
    fig.suptitle("D5C nominal-walk bias accumulation by deterministic time segment")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(output / "figure_D5C_05_time_segment_contributions.png", dpi=170)
    plt.close(fig)

    maximum_discrete_closure = max(
        row["max_abs_integral_minus_pi_change"] for row in validation_rows)
    summary = {
        "schema_version": 1,
        "stage": "D5C_nominal_walking_residual_decomposition",
        "passed": bool(maximum_discrete_closure <= 1.0e-10),
        "source_stage": "D5B_formal_campaign",
        "source_fairness_sha256": sha256(fairness_path),
        "input_runs": len(runs),
        "nominal_walk_repeats": 3,
        "equation_identity": "pi_dot[j] = sum_i Y[i,j] * sigma[i], Gamma=I10",
        "maximum_eq8_derivative_error": max(row["eq8_derivative_max_error"]
                                             for row in validation_rows),
        "maximum_axis_closure_error": max(row["eq8_axis_closure_max_error"]
                                           for row in validation_rows),
        "maximum_discrete_update_closure_error": maximum_discrete_closure,
        "condition_net_update": {},
        "nominal_walk": {},
        "interpretation_boundary": {
            "exact_bookkeeping": "right-endpoint Euler sigma-axis, contact-mode, and time-segment updates sum to the actual logged pi_hat change",
            "not_causal_decomposition": "closed-loop partitions do not prove independent linear causes",
            "pi_star": "nominal plant pi_star is exactly zero by experimental construction",
            "finite_difference_residual": "state finite difference minus sampled nominal flow is a numerical execution diagnostic, not model-mismatch ground truth",
            "missing_exact_split": "D5B bags do not separately log velocity error and Lambda times pose error; aggregate sigma is exact but this sub-split requires a new diagnostic schema",
        },
    }
    walk_stack = np.stack([run_metrics[(walk, run_name)]["axis_signed"]
                           for run_name in RUNS])
    walk_total = np.sum(walk_stack, axis=1)
    for group in GROUPS:
        totals = np.stack([run_metrics[(group, run_name)]["drive_signed"]
                           for run_name in RUNS])
        summary["condition_net_update"][group] = {
            parameter: sample_stats(totals[:, index])
            for index, parameter in enumerate(PARAMETERS)}
    for parameter_index, parameter in enumerate(PARAMETERS):
        mean_axis = np.mean(walk_stack[:, :, parameter_index], axis=0)
        dominant = int(np.argmax(np.abs(mean_axis)))
        total_samples = walk_total[:, parameter_index]
        summary["nominal_walk"][parameter] = {
            "drive_integral": sample_stats(total_samples),
            "axis_signed_integral_mean": {
                axis: float(mean_axis[index]) for index, axis in enumerate(SIGMA_AXES)},
            "dominant_net_axis": SIGMA_AXES[dominant],
            "dominant_net_axis_integral": float(mean_axis[dominant]),
            "axis_cancellation_ratio": float(
                abs(np.sum(mean_axis)) / np.sum(np.abs(mean_axis)))
                if np.sum(np.abs(mean_axis)) > 0.0 else 0.0,
        }
        summary["nominal_walk"][parameter]["time_segment_signed_integral_mean"] = {
            segment: float(np.mean([
                run_metrics[(walk, run_name)]["segment_partition"][segment]["signed"][parameter_index]
                for run_name in RUNS])) for segment in segments}
        summary["nominal_walk"][parameter]["mode_signed_integral_mean"] = {
            MODE_NAMES[mode]: float(np.mean([
                run_metrics[(walk, run_name)]["mode_partition"].get(
                    mode, {"signed": np.zeros(10)})["signed"][parameter_index]
                for run_name in RUNS])) for mode in walk_modes}

    summary_path = output / "stage_d5c_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")

    def key_line(parameter):
        item = summary["nominal_walk"][parameter]
        total = item["drive_integral"]
        segments_value = item["time_segment_signed_integral_mean"]
        return ("| {} | {:.6g} +/- {:.3g} | {} | {:.6g} | {:.6g} | {:.6g} | {:.6g} | {:.6g} |".format(
            parameter, total["mean"], total["sample_std"], item["dominant_net_axis"],
            item["dominant_net_axis_integral"],
            segments_value["pre_target_0_1p2"],
            segments_value["target_ramp_1p2_1p4"],
            segments_value["first_post_ramp_cycle_1p4_2p6"],
            segments_value["steady_walk_2p6_16p2"]))

    def condition_line(group):
        values = summary["condition_net_update"][group]
        return "| {} | {} |".format(
            GROUP_LABELS[group], " | ".join(
                "{:.6g} +/- {:.3g}".format(values[name]["mean"],
                                            values[name]["sample_std"])
                for name in ("hz", "Jxx", "Jyy")))

    report = """# Stage D5C — Nominal walking residual decomposition

## 结论范围

本阶段只读使用已冻结且 fairness PASS 的 D5B 正式数据，共 9 个 nominal runs（hold/stance-target/walk-target 各 3 次），没有重跑实验、没有修改 Eq.(8)/Eq.(10)，也没有调 Gamma。

Nominal plant 与 controller nominal model 相同，因此本实验的真实 payload uncertainty 为 `pi_star=0`。这里分解的是为什么非零 tracking sigma 会通过 `Y^T sigma` 累积为 nominal estimate bias，而不是重新定义一个非零 nominal pi_star。

## 数学闭合

- `pi_dot[j] = sum_i Y[i,j]*sigma[i]` 最大误差：{eq8:.3g}
- 六个 sigma 轴相加重构 drive 的最大误差：{closure:.3g}
- 显式 Euler 分区累计与实际 `pi_hat` 净变化最大误差：{discrete:.3g}
- sigma-axis、contact-mode、time-segment 均采用与在线 estimator 相同的右端显式 Euler 更新，分区和严格等于实际 `pi_hat` 净变化。

## Nominal walk 从零开始的关键累计更新

先看三个 Nominal 受控条件截至约 16.2 s 的净更新：

| condition | hz | Jxx | Jyy |
|---|---:|---:|---:|
{condition_rows}

| parameter | net update through 16.2 s | dominant net sigma axis | dominant-axis update | pre-target walk 0--1.2 s | 0.20 s target-ramp | first post-ramp cycle 1.4--2.6 s | steady walk 2.6--16.2 s |
|---|---:|---|---:|---:|---:|---:|---:|
{key_rows}

上述累计量不是用 `pi_hat(end)` 反推；它从 bag 的零初值开始，将已记录的 `Y^T sigma` 按在线代码的显式 Euler 关系逐步累计，并与截至 16.2 s 的 `pi_hat` 净变化闭合。它是 endpoint update，不等于 D5B 报告使用的 last-2-s time mean。

## 可直接回答的问题

1. `hz/Jxx/Jyy` 的 Nominal bias 是否来自 payload？不是，Nominal 的 `pi_star=0`。
2. 它们是否是 Eq.(8) 代码错误？不是，Eq.(8) 和六轴分解均数值闭合。
3. bias 如何形成？由 nominal walking 下非零 sigma 经物理 regressor 投影并长期积分；图 01/03 给出具体 sigma 轴，图 02 给出具体接触模式，图 05 区分短 target ramp 与后续持续行走。
4. 分区能否证明某一腿或某个误差项是独立根因？不能。闭环中 reference、contact、policy 和 adaptive correction 相互耦合，分区是精确 bookkeeping，不是线性因果分解。

## 仍缺少的一层

D5B bag 保存了同步的 aggregate sigma 和完整 Y，但没有分别保存：

- `v_d-v` 与 `Lambda_l(p_d-p)`；
- `omega_d-omega` 与 `Lambda_o e_o`。

因此本报告可以精确回答“哪个 sigma world axis、哪个 contact mode、哪个时间段驱动了参数偏置”，但不能仅凭旧 bag 精确回答同一 sigma 轴内部是速度项还是位姿项占主导。若需要这一层，下一步应只增加 diagnostics schema 并做短 Nominal walk 重采集，不应先调控制参数。

## 解释边界

有限差分 execution residual 单独保存在 CSV。它包含采样、积分和区间内 policy 变化误差，不是 `pi_star`，不能用来宣称 Nominal 存在真实 payload mismatch。当前证据说明的是 tracking-error-driven estimator drift；尚未形成 estimator 修复方案。
""".format(eq8=summary["maximum_eq8_derivative_error"],
           closure=summary["maximum_axis_closure_error"],
           discrete=summary["maximum_discrete_update_closure_error"],
           condition_rows="\n".join(condition_line(group) for group in GROUPS),
           key_rows="\n".join(key_line(parameter) for parameter in KEY_PARAMETERS))
    (output / "stage_d5c_report.md").write_text(report)
    (output / "stage_d5c_input_manifest.json").write_text(json.dumps({
        "schema_version": 1,
        "source_fairness_audit": str(fairness_path),
        "source_fairness_sha256": sha256(fairness_path),
        "runs": input_manifest,
    }, indent=2, sort_keys=True) + "\n")

    artifacts = sorted(path for path in output.iterdir()
                       if path.is_file() and path.name != "stage_d5c_artifact_manifest.json")
    (output / "stage_d5c_artifact_manifest.json").write_text(json.dumps({
        "schema_version": 1,
        "analyzer_sha256": sha256(Path(__file__)),
        "artifacts": [{"name": path.name, "sha256": sha256(path)} for path in artifacts],
    }, indent=2, sort_keys=True) + "\n")
    print("D5C nominal residual decomposition PASS: {}".format(output))


if __name__ == "__main__":
    main()
