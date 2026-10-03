#!/usr/bin/env python3
"""Materialize one D5H1 formal run on the frozen 0--15 s grid."""

import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


D5H1 = "/task_d/d5h1_applied_update_sample"
FLOW = "/task_d/equation10_full_flow_sample"
CLF = "/task_d/d5h1_eq11_sample"
PLANT = "/task_d/payload_inertial_plant_sample"
OBSERVATION = "/legged_robot_mpc_observation"
TARGET = "/legged_robot_mpc_target"
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz",
              "Jyy", "Jyz", "Jzz")


def interpolate(grid, times, values):
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    return np.column_stack([
        np.interp(grid, times, values[:, column])
        for column in range(values.shape[1])])


def nearest(grid, times, values):
    times = np.asarray(times, dtype=float)
    values = np.asarray(values)
    right = np.clip(np.searchsorted(times, grid, side="left"), 0,
                    len(times) - 1)
    left = np.clip(right - 1, 0, len(times) - 1)
    use_left = np.abs(grid - times[left]) <= np.abs(times[right] - grid)
    return values[np.where(use_left, left, right)]


def wrap_angle(values):
    return (values + np.pi) % (2.0 * np.pi) - np.pi


def trapz(values, time):
    return float(np.trapz(values, time))


def rms(values, time):
    values = np.asarray(values, dtype=float)
    squared = values ** 2 if values.ndim == 1 else np.sum(values ** 2, axis=1)
    return math.sqrt(trapz(squared, time) / (time[-1] - time[0]))


def time_mean(values, time):
    return trapz(values, time) / (time[-1] - time[0])


def truth_and_scales(config, plants):
    payload = config["payload"]
    mass = float(payload["mass_kg"])
    com = np.asarray(payload["com_nominal_c0_base_m"], dtype=float)
    inertia = np.asarray(payload["inertia_com_base_kg_m2"], dtype=float)
    inertia_com = np.asarray([
        [inertia[0], inertia[1], inertia[2]],
        [inertia[1], inertia[3], inertia[4]],
        [inertia[2], inertia[4], inertia[5]]])
    parallel_axis = mass * (float(np.dot(com, com)) * np.eye(3)
                            - np.outer(com, com))
    inertia_origin = inertia_com + parallel_axis
    configured_truth = np.asarray([
        mass, *(mass * com), inertia_origin[0, 0], inertia_origin[0, 1],
        inertia_origin[0, 2], inertia_origin[1, 1], inertia_origin[1, 2],
        inertia_origin[2, 2]])
    if config["truePlantPayloadInertialEnabled"]:
        truth = np.asarray(plants[0][14:24], dtype=float)
        if not np.allclose(truth, configured_truth, rtol=0.0, atol=1.0e-12):
            raise RuntimeError("plant pi_star differs from the frozen payload")
    else:
        truth = np.zeros(10)
    length = float(np.linalg.norm(com))
    # Keep the D5H0/D5H1 pre-registered coordinate normalization based on the
    # physical payload COM inertia; pi_star itself is in base-origin coordinates.
    inertia_scale = float(np.max(np.abs(inertia)))
    scales = np.asarray([mass] + [mass * length] * 3 + [inertia_scale] * 6)
    if np.any(scales <= 0.0):
        raise RuntimeError("invalid physical parameter scales")
    return truth, scales


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_task_d_d5h1_formal_run.py RUN_DIR")
    run = Path(sys.argv[1]).resolve()
    config = json.loads((run / "effective_config.json").read_text())
    if config.get("run_scope") != "formal":
        raise RuntimeError("formal analyzer received a non-formal run")
    start = float(config["analysis_window_start_mpc_time_s"])
    end = float(config["analysis_window_end_mpc_time_s"])
    dt = float(config["common_grid_dt_s"])
    grid = np.linspace(start, end, int(round((end - start) / dt)) + 1)

    d5_rows, flow_rows, clf_rows, plant_rows = [], [], [], []
    observations, targets = [], []
    labels = {D5H1: set(), FLOW: set(), CLF: set(), PLANT: set()}
    bag_path = run / "task_d_d5h1.bag"
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _ in bag.read_messages(topics=[
                D5H1, FLOW, CLF, PLANT, OBSERVATION, TARGET]):
            if topic in labels:
                labels[topic].add(message.layout.dim[0].label
                                  if message.layout.dim else "")
                row = [float(value) for value in message.data]
                {D5H1: d5_rows, FLOW: flow_rows, CLF: clf_rows,
                 PLANT: plant_rows}[topic].append(row)
            elif topic == OBSERVATION:
                observations.append((
                    float(message.time), int(message.mode),
                    [float(value) for value in message.state.value],
                    [float(value) for value in message.input.value]))
            else:
                targets.append((
                    np.asarray(message.timeTrajectory, dtype=float),
                    np.asarray([value.value for value in message.stateTrajectory],
                               dtype=float)))

    for rows in (d5_rows, flow_rows, clf_rows, plant_rows):
        rows.sort(key=lambda row: row[0])
    observations = [value for _, value in sorted(
        {row[0]: row for row in observations}.items())]
    expected_payload = bool(config["truePlantPayloadInertialEnabled"])
    schema_ok = bool(
        len(d5_rows) >= 100 and len(flow_rows) >= 100 and len(clf_rows) >= 100
        and all(len(row) == 71 for row in d5_rows)
        and all(len(row) == 80 for row in flow_rows)
        and all(len(row) == 119 for row in clf_rows)
        and labels[D5H1] == {"payload_inertial_d5h1_applied_v1_71_fields"}
        and labels[FLOW] == {"payload_inertial_eq10_full_flow_v2_80_fields"}
        and labels[CLF] == {"payload_inertial_d5h1_eq11_v1_119_fields"}
        and (expected_payload and len(plant_rows) >= 100
             and all(len(row) == 74 for row in plant_rows)
             and labels[PLANT] == {"payload_inertial_plant_v2_74_fields"}
             or not expected_payload and not plant_rows))
    dimensions_ok = bool(
        len(observations) >= 100
        and all(len(row[2]) == 24 and len(row[3]) == 24
                for row in observations))
    arrays = [np.asarray(rows, dtype=float) for rows in
              (d5_rows, flow_rows, clf_rows)]
    all_finite = bool(schema_ok and dimensions_ok
                      and all(np.all(np.isfinite(array)) for array in arrays)
                      and all(np.all(np.isfinite(row)) for row in plant_rows)
                      and all(np.all(np.isfinite(row[2]))
                              and np.all(np.isfinite(row[3]))
                              for row in observations))
    if not targets:
        raise RuntimeError("missing deterministic target")
    target_times, target_states = targets[-1]
    target_ok = bool(
        len(target_times) == 2 and target_states.shape == (2, 24)
        and abs(target_times[0] - 1.2) <= 1.0e-9
        and abs(target_times[1] - 1.4) <= 1.0e-9
        and abs(target_states[1, 6] - target_states[0, 6] - 0.1) <= 1.0e-6)

    d5, flow, clf = arrays
    obs_time = np.asarray([row[0] for row in observations], dtype=float)
    time_sets = [d5[:, 0], flow[:, 0], clf[:, 0], obs_time]
    start_tolerance = float(config["diagnosticTopicStartToleranceS"])
    end_tolerance = float(config["topicEndToleranceS"])
    coverage_ok = bool(
        obs_time[0] <= start + 0.021
        and obs_time[-1] >= end - end_tolerance
        and all(times[0] <= start + start_tolerance
                and times[-1] >= end - end_tolerance
                for times in time_sets[:3]))
    monotonic_ok = all(np.all(np.diff(times) > 0.0) for times in time_sets)

    d5_grid = interpolate(grid, d5[:, 0], d5)
    flow_grid = interpolate(grid, flow[:, 0], flow)
    clf_grid = interpolate(grid, clf[:, 0], clf)
    state_grid = interpolate(grid, obs_time, [row[2] for row in observations])
    input_grid = interpolate(grid, obs_time, [row[3] for row in observations])
    mode_grid = nearest(grid, obs_time, [row[1] for row in observations])
    desired_grid = interpolate(grid, target_times, target_states)

    actual_position = state_grid[:, 6:9]
    desired_position = desired_grid[:, 6:9]
    position_error = desired_position - actual_position
    orientation_error = wrap_angle(desired_grid[:, 9:12] - state_grid[:, 9:12])
    sigma = d5_grid[:, 65:71]
    pi_hat = d5_grid[:, 55:65]
    raw_drive = d5_grid[:, 15:25]
    baseline = d5_grid[:, 25:35]
    applied_drive = d5_grid[:, 35:45]
    candidate_norm = np.linalg.norm(flow_grid[:, 32:56], axis=1)
    applied_norm = np.linalg.norm(flow_grid[:, 56:80], axis=1)
    sigma_norm = np.linalg.norm(sigma, axis=1)
    truth, scales = truth_and_scales(config, plant_rows)
    pi_scaled_norm = np.linalg.norm(pi_hat / scales, axis=1)
    contact = input_grid[:, :12]
    position_norm = np.linalg.norm(position_error, axis=1)
    orientation_norm = np.linalg.norm(orientation_error, axis=1)
    last_two = grid >= end - 2.0
    last_two_vector = np.mean(pi_hat[last_two], axis=0)

    columns = {
        "time_s": grid, "mode": mode_grid,
        "desired_x": desired_position[:, 0], "actual_x": actual_position[:, 0],
        "x_error": position_error[:, 0],
        "desired_y": desired_position[:, 1], "actual_y": actual_position[:, 1],
        "y_error": position_error[:, 1],
        "desired_z": desired_position[:, 2], "actual_z": actual_position[:, 2],
        "z_error": position_error[:, 2],
        "position_error_norm": position_norm,
        "orientation_error_norm": orientation_norm,
        "sigma_norm": sigma_norm, "pi_hat_scaled_norm": pi_scaled_norm,
        "eq10_candidate_norm": candidate_norm,
        "eq10_applied_norm": applied_norm,
        "eq11_h": clf_grid[:, 8], "eq11_violation": clf_grid[:, 10],
        "raw_update_drive_norm": np.linalg.norm(raw_drive, axis=1),
        "baseline_norm": np.linalg.norm(baseline, axis=1),
        "applied_update_drive_norm": np.linalg.norm(applied_drive, axis=1),
    }
    for index, name in enumerate(PARAMETERS):
        columns["pi_hat_" + name] = pi_hat[:, index]
        columns["pi_star_" + name] = np.full(len(grid), truth[index])
    for index in range(24):
        columns["policy_{}".format(index)] = input_grid[:, index]
    csv_path = run / "task_d_d5h1_formal_run.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(list(columns))
        for row in range(len(grid)):
            writer.writerow([columns[name][row] for name in columns])

    metrics = {
        "x_rmse_m": rms(columns["x_error"], grid),
        "x_iae_m_s": trapz(np.abs(columns["x_error"]), grid),
        "x_peak_abs_m": float(np.max(np.abs(columns["x_error"]))),
        "position_rmse_m": rms(position_error, grid),
        "position_peak_m": float(np.max(position_norm)),
        "orientation_rmse_rad": rms(orientation_error, grid),
        "orientation_peak_rad": float(np.max(orientation_norm)),
        "sigma_mean_norm": time_mean(sigma_norm, grid),
        "sigma_peak_norm": float(np.max(sigma_norm)),
        "sigma_integral_norm_s": trapz(sigma_norm, grid),
        "pi_hat_scaled_norm_mean": time_mean(pi_scaled_norm, grid),
        "pi_hat_scaled_norm_peak": float(np.max(pi_scaled_norm)),
        "pi_hat_scaled_norm_integral_s": trapz(pi_scaled_norm, grid),
        "pi_hat_terminal_scaled_norm": float(np.linalg.norm(pi_hat[-1] / scales)),
        "pi_hat_last2s_mean_vector_scaled_norm": float(
            np.linalg.norm(last_two_vector / scales)),
        "eq10_applied_rms_norm": rms(applied_norm, grid),
        "eq10_applied_peak_norm": float(np.max(applied_norm)),
        "eq10_applied_integral_norm_s": trapz(applied_norm, grid),
        "eq11_h_mean": time_mean(columns["eq11_h"], grid),
        "eq11_violation_mean": time_mean(columns["eq11_violation"], grid),
        "eq11_violation_peak": float(np.max(columns["eq11_violation"])),
        "policy_rms_norm": rms(input_grid, grid),
        "contact_force_rms_norm_N": rms(contact, grid),
        "actual_x_displacement_m": float(actual_position[-1, 0] - actual_position[0, 0]),
    }
    for index, name in enumerate(PARAMETERS):
        metrics["pi_{}_terminal".format(name)] = float(pi_hat[-1, index])
        metrics["pi_{}_last2s_mean".format(name)] = float(last_two_vector[index])

    smoke = json.loads((run / "task_d_d5h1_smoke_summary.json").read_text())
    gate = json.loads((run / "d5h1_gate_audit_summary.json").read_text())
    checks = {
        "per_run_smoke_pass": bool(smoke.get("passed")),
        "strict_provenance_eq10_eq11_gate_pass": bool(gate.get("passed")),
        "schemas_and_dimensions": schema_ok and dimensions_ok,
        "all_finite": all_finite,
        "frozen_window_coverage": coverage_ok,
        "strictly_increasing_topic_times": monotonic_ok,
        "deterministic_target": target_ok,
        "walking_observed": any(int(mode) != 15 for mode in mode_grid),
    }
    passed = all(checks.values())
    summary = {
        "schema_version": 1,
        "stage": "D5H1_formal_run",
        "group": config["group"], "run_index": config["run_index"],
        "acquisition_sequence": config.get("acquisition_sequence"),
        "pass": passed, "formal_eligibility": passed,
        "checks": checks, "analysis_window_s": [start, end],
        "common_grid_dt_s": dt, "common_grid_sample_count": len(grid),
        "plant_pi_star_10d": truth.tolist(),
        "physical_coordinate_scales": scales.tolist(),
        "terminal_pi_hat_10d": pi_hat[-1].tolist(),
        "last2s_mean_pi_hat_10d": last_two_vector.tolist(),
        "metrics": metrics,
        "boundary": (
            "One repeated formal sample. Scientific claims require the frozen "
            "four-mode campaign aggregate."),
        "artifacts": {"csv": str(csv_path)},
    }
    (run / "task_d_d5h1_formal_run_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")

    figure, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    axes[0].plot(grid, columns["desired_x"], "k--", label="desired x")
    axes[0].plot(grid, columns["actual_x"], label="actual x")
    axes[1].plot(grid, columns["x_error"], label="x error")
    axes[1].plot(grid, columns["position_error_norm"], label="position error norm")
    axes[2].plot(grid, pi_scaled_norm, label="scaled ||pi_hat||")
    axes[2].plot(grid, sigma_norm, label="||sigma||")
    axes[3].plot(grid, applied_norm, label="Eq.(10) applied norm")
    axes[3].plot(grid, columns["eq11_violation"], label="Eq.(11) violation")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    axes[-1].set_xlabel("MPC time [s]")
    figure.suptitle("D5H1 formal: {} run_{:02d}".format(
        config["group"], int(config["run_index"])))
    figure.tight_layout()
    plot_path = run / "task_d_d5h1_formal_run.png"
    figure.savefig(plot_path, dpi=170)
    plt.close(figure)
    print(json.dumps(summary, indent=2, sort_keys=True))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
