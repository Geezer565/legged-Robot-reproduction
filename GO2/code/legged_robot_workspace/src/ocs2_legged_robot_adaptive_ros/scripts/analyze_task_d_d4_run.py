#!/usr/bin/env python3
"""Validate and materialize one full-horizon Stage D4 run on a fixed time grid."""

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


ESTIMATE = "/task_d/payload_inertial_10d_v3_sample"
FLOW = "/task_d/equation10_full_flow_sample"
PLANT = "/task_d/payload_inertial_plant_sample"
IDENTITY = "/task_d/model_identity"
OBSERVATION = "/legged_robot_mpc_observation"
TARGET = "/legged_robot_mpc_target"
STATUS = "/task_c/locomotion_scheduler_status"
POLICY = "/task_b/policy_input_sample"
LEGACY = ("/adaptive_update/sample", "/task_b/equation10_sample")
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy", "Jyz", "Jzz")


def finite(values):
    return bool(np.all(np.isfinite(np.asarray(values, dtype=float))))


def vector_norm(values):
    return np.linalg.norm(np.asarray(values, dtype=float), axis=-1)


def interpolate_matrix(grid, times, matrix):
    times = np.asarray(times, dtype=float)
    matrix = np.asarray(matrix, dtype=float)
    return np.column_stack([
        np.interp(grid, times, matrix[:, index])
        for index in range(matrix.shape[1])])


def nearest_values(grid, times, values):
    times = np.asarray(times, dtype=float)
    values = np.asarray(values)
    right = np.searchsorted(times, grid, side="left")
    right = np.clip(right, 0, len(times) - 1)
    left = np.clip(right - 1, 0, len(times) - 1)
    choose_left = np.abs(grid - times[left]) <= np.abs(times[right] - grid)
    return values[np.where(choose_left, left, right)]


def wrap_angle(values):
    return (values + np.pi) % (2.0 * np.pi) - np.pi


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_task_d_d4_run.py RUN_DIR")
    run = Path(sys.argv[1]).resolve()
    config = json.loads((run / "effective_config.json").read_text())
    group = config["group"]
    window_start = float(config["analysis_window_start_mpc_time_s"])
    window_end = float(config["analysis_window_end_mpc_time_s"])
    grid_dt = float(config["common_grid_dt_s"])
    expected_end = float(config["acquisition_end_mpc_time_s"])
    expected_update = bool(config["enableParameterUpdate"])
    expected_dynamics = bool(config["enableAdaptiveDynamics"])
    expected_payload = bool(config["truePlantPayloadInertialEnabled"])
    grid = np.linspace(window_start, window_end,
                       int(round((window_end - window_start) / grid_dt)) + 1)

    estimates, flows, plants, observations, targets = [], [], [], [], []
    statuses, policies, identity = [], [], None
    labels = {ESTIMATE: set(), FLOW: set(), PLANT: set()}
    legacy_counts = {topic: 0 for topic in LEGACY}
    with rosbag.Bag(str(run / "task_d_d4.bag")) as bag:
        topics = [ESTIMATE, FLOW, PLANT, IDENTITY, OBSERVATION, TARGET,
                  STATUS, POLICY] + list(LEGACY)
        for topic, message, _ in bag.read_messages(topics=topics):
            if topic in (ESTIMATE, FLOW, PLANT):
                labels[topic].add(message.layout.dim[0].label
                                  if message.layout.dim else "")
                row = [float(value) for value in message.data]
                {ESTIMATE: estimates, FLOW: flows, PLANT: plants}[topic].append(row)
            elif topic == IDENTITY:
                identity = json.loads(message.data)
            elif topic == OBSERVATION:
                observations.append((
                    float(message.time), int(message.mode),
                    [float(value) for value in message.state.value],
                    [float(value) for value in message.input.value]))
            elif topic == TARGET:
                targets.append((
                    np.asarray(message.timeTrajectory, dtype=float),
                    np.asarray([value.value for value in message.stateTrajectory], dtype=float)))
            elif topic == STATUS:
                statuses.append(json.loads(message.data))
            elif topic == POLICY:
                policies.append([float(value) for value in message.data])
            else:
                legacy_counts[topic] += 1

    estimates.sort(key=lambda row: row[0])
    flows.sort(key=lambda row: row[0])
    plants.sort(key=lambda row: row[0])
    observations = [value for _, value in sorted(
        {row[0]: row for row in observations}.items())]
    schema_ok = (
        len(estimates) >= 100 and len(flows) >= 100
        and all(len(row) == 118 for row in estimates)
        and all(len(row) == 80 for row in flows)
        and labels[ESTIMATE] == {"payload_inertial_10d_v3_118_fields"}
        and labels[FLOW] == {"payload_inertial_eq10_full_flow_v2_80_fields"})
    if expected_payload:
        schema_ok = (
            schema_ok and len(plants) >= 100
            and all(len(row) == 74 for row in plants)
            and labels[PLANT] == {"payload_inertial_plant_v2_74_fields"})
    else:
        schema_ok = schema_ok and not plants
    observation_dimensions_ok = (
        len(observations) >= 100
        and all(len(row[2]) == 24 and len(row[3]) == 24 for row in observations))
    all_finite = (
        schema_ok and observation_dimensions_ok
        and all(finite(row) for row in estimates + flows + plants)
        and all(finite(row[2]) and finite(row[3]) for row in observations))

    estimate_time = np.asarray([row[0] for row in estimates])
    flow_time = np.asarray([row[0] for row in flows])
    observation_time = np.asarray([row[0] for row in observations])
    coverage_ok = bool(
        len(estimate_time) and len(flow_time) and len(observation_time)
        and estimate_time[0] <= window_start and estimate_time[-1] >= window_end
        and flow_time[0] <= window_start and flow_time[-1] >= window_end
        and observation_time[0] <= window_start and observation_time[-1] >= expected_end)
    monotonic_ok = all(
        np.all(np.diff(values) > 0.0)
        for values in (estimate_time, flow_time, observation_time) if len(values))
    versions = [int(round(row[3])) for row in estimates]
    synchronization_ok = (
        max((abs(row[2]) for row in estimates), default=math.inf) <= 1.0e-6
        and all(right > left for left, right in zip(versions, versions[1:])))
    estimate_flags_ok = all(
        row[4] == float(expected_update)
        and row[5] == float(expected_dynamics)
        and row[6] == 1.0 and row[7] == 10.0
        and row[8] == 1.0 and row[9] == float(expected_dynamics)
        and row[10] == 0.0 for row in estimates)
    flow_flags_ok = all(
        row[4] == float(expected_update)
        and row[5] == float(expected_dynamics)
        and row[6] == 1.0 and row[7] == float(expected_dynamics)
        for row in flows)
    estimated_total_mass = np.asarray([row[12] for row in estimates])
    inertia_eigenvalues = []
    for row in estimates:
        inertia = np.asarray([
            [row[16], row[17], row[18]],
            [row[17], row[19], row[20]],
            [row[18], row[20], row[21]]])
        inertia_eigenvalues.extend(np.linalg.eigvalsh(inertia))
    long_horizon_model_valid = bool(
        estimate_flags_ok and flow_flags_ok
        and np.min(estimated_total_mass) >= 1.0
        and np.min(inertia_eigenvalues) > 0.0)

    flow_matrix = np.asarray(flows, dtype=float)
    candidate_flow_norm = vector_norm(flow_matrix[:, 32:56])
    applied_flow_norm = vector_norm(flow_matrix[:, 56:80])
    if expected_dynamics:
        applied_path_error = float(np.max(np.abs(flow_matrix[:, 32:56] - flow_matrix[:, 56:80])))
    else:
        applied_path_error = float(np.max(np.abs(flow_matrix[:, 56:80])))
    pi_matrix = np.asarray([row[22:32] for row in estimates])
    pi_peak = float(np.max(vector_norm(pi_matrix)))
    mode_path_ok = bool(
        (expected_update and pi_peak > 1.0e-12
         or not expected_update and pi_peak <= 1.0e-12)
        and (expected_dynamics and np.max(candidate_flow_norm) > 1.0e-12
             and np.max(applied_flow_norm) > 1.0e-12
             or not expected_dynamics and np.max(candidate_flow_norm) <= 1.0e-12
             and np.max(applied_flow_norm) <= 1.0e-12)
        and applied_path_error <= 1.0e-12)

    nominal_mass = identity.get("controller_nominal_mass") if identity else None
    identity_ok = bool(identity) and identity.get("schema_version") == 2
    if expected_payload:
        identity_ok = bool(
            identity_ok and identity.get("payload_inertial_override_active") is True
            and identity.get("plant_model_id") ==
                "srbd_payload_inertial_10d_nominal_coordinates_v2"
            and identity.get("plant_true_total_mass", 0.0) > nominal_mass)
        truth = np.asarray(plants[0][14:24] if plants else [math.nan] * 10)
        truth_constant_error = max((
            max(abs(row[14 + index] - truth[index]) for index in range(10))
            for row in plants), default=math.inf)
        roundtrip_peak = max((row[24] for row in plants), default=math.inf)
        residual_peak = max((row[25] for row in plants), default=0.0)
        plant_ok = bool(
            truth_constant_error <= 1.0e-12 and roundtrip_peak <= 1.0e-10
            and residual_peak > 1.0e-10)
    else:
        identity_ok = bool(
            identity_ok and identity.get("payload_inertial_override_active") is False
            and identity.get("plant_model_id") == "nominal_ocs2_rollout_v1"
            and abs(identity.get("plant_true_total_mass", math.inf) - nominal_mass) <= 1.0e-12)
        truth = np.zeros(10)
        truth_constant_error, roundtrip_peak, residual_peak = 0.0, 0.0, 0.0
        plant_ok = not plants

    target_status = [value for value in statuses if value.get("stage") == "target_published"]
    schedule_ok = bool(target_status) and abs(
        target_status[-1].get("target_start_time", 0.0) - 1.2) <= 1.0e-12
    if not targets:
        schedule_ok = False
        target_times, target_states = np.asarray([]), np.empty((0, 0))
    else:
        target_times, target_states = targets[-1]
        schedule_ok = bool(
            schedule_ok and len(target_times) == 2 and target_states.shape == (2, 24)
            and abs(target_times[0] - 1.2) <= 1.0e-9
            and abs(target_times[1] - 1.4) <= 1.0e-9
            and abs((target_states[1, 6] - target_states[0, 6]) - 0.1) <= 1.0e-6)
    observed_modes = sorted(set(row[1] for row in observations))
    locomotion_schedule_ok = bool(schedule_ok and any(mode != 15 for mode in observed_modes))
    legacy_ok = all(value == 0 for value in legacy_counts.values())

    # Materialize every signal on the one frozen D4 grid.
    observation_states = interpolate_matrix(grid, observation_time, [row[2] for row in observations])
    observation_inputs = interpolate_matrix(grid, observation_time, [row[3] for row in observations])
    modes = nearest_values(grid, observation_time, [row[1] for row in observations]).astype(int)
    desired = interpolate_matrix(grid, target_times, target_states)
    estimate_grid = interpolate_matrix(grid, estimate_time, estimates)
    flow_grid = interpolate_matrix(grid, flow_time, flows)
    if expected_payload:
        plant_time = np.asarray([row[0] for row in plants])
        plant_grid = interpolate_matrix(grid, plant_time, plants)
        flow_residual_norm = plant_grid[:, 25]
    else:
        flow_residual_norm = np.zeros(len(grid))
    actual_position = observation_states[:, 6:9]
    desired_position = desired[:, 6:9]
    position_error = desired_position - actual_position
    actual_rpy = observation_states[:, 9:12]
    desired_rpy = desired[:, 9:12]
    orientation_error = wrap_angle(desired_rpy - actual_rpy)
    sigma = estimate_grid[:, 42:48]
    pi_hat = estimate_grid[:, 22:32]
    pi_dot = estimate_grid[:, 32:42]
    pi_star = np.tile(truth, (len(grid), 1))
    pi_error = pi_hat - pi_star
    y_pi = estimate_grid[:, 108:114]
    candidate_grid_norm = vector_norm(flow_grid[:, 32:56])
    applied_grid_norm = vector_norm(flow_grid[:, 56:80])

    columns = {
        "time_s": grid,
        "mode": modes,
        "desired_x": desired_position[:, 0],
        "desired_y": desired_position[:, 1],
        "desired_z": desired_position[:, 2],
        "actual_x": actual_position[:, 0],
        "actual_y": actual_position[:, 1],
        "actual_z": actual_position[:, 2],
        "x_error": position_error[:, 0],
        "y_error": position_error[:, 1],
        "z_error": position_error[:, 2],
        "position_error_norm": vector_norm(position_error),
        "desired_roll": desired_rpy[:, 0],
        "desired_pitch": desired_rpy[:, 1],
        "desired_yaw": desired_rpy[:, 2],
        "actual_roll": actual_rpy[:, 0],
        "actual_pitch": actual_rpy[:, 1],
        "actual_yaw": actual_rpy[:, 2],
        "roll_error": orientation_error[:, 0],
        "pitch_error": orientation_error[:, 1],
        "yaw_error": orientation_error[:, 2],
        "orientation_error_norm": vector_norm(orientation_error),
        "sigma_norm": vector_norm(sigma),
        "pi_error_coordinate_norm": vector_norm(pi_error),
        "candidate_flow_correction_norm": candidate_grid_norm,
        "applied_flow_correction_norm": applied_grid_norm,
        "true_vs_nominal_flow_residual_norm": flow_residual_norm,
        "estimated_total_mass_kg": estimate_grid[:, 12],
    }
    for index in range(6):
        columns["sigma_{}".format(index)] = sigma[:, index]
        columns["Y_pi_hat_{}".format(index)] = y_pi[:, index]
    for index, name in enumerate(PARAMETERS):
        columns["pi_hat_" + name] = pi_hat[:, index]
        columns["pi_dot_" + name] = pi_dot[:, index]
        columns["pi_star_" + name] = pi_star[:, index]
        columns["pi_error_" + name] = pi_error[:, index]
    for index in range(24):
        columns["policy_{}".format(index)] = observation_inputs[:, index]

    csv_path = run / "task_d_d4_run.csv"
    names = list(columns)
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        for index in range(len(grid)):
            writer.writerow([columns[name][index] for name in names])

    figure_path = run / "task_d_d4_overview.png"
    fig, axes = plt.subplots(5, 1, figsize=(12, 15), sharex=True)
    axes[0].plot(grid, columns["desired_x"], "k--", label="desired x")
    axes[0].plot(grid, columns["actual_x"], label="actual x")
    axes[1].plot(grid, columns["pi_hat_m"], label="pi_hat m")
    axes[1].plot(grid, columns["pi_star_m"], "k--", label="pi_star m")
    axes[2].plot(grid, columns["pi_error_coordinate_norm"], label="10D coordinate error norm")
    axes[2].plot(grid, columns["sigma_norm"], label="sigma norm")
    axes[3].plot(grid, candidate_grid_norm, label="candidate correction")
    axes[3].plot(grid, applied_grid_norm, "--", label="applied correction")
    axes[4].plot(grid, flow_residual_norm, label="true-vs-nominal flow residual")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[-1].set_xlabel("MPC time [s]")
    fig.suptitle("D4 {} run {} full-horizon validation".format(
        group, config["round_index"]))
    fig.tight_layout()
    fig.savefig(figure_path, dpi=160)
    plt.close(fig)

    passed = bool(
        schema_ok and all_finite and coverage_ok and monotonic_ok
        and synchronization_ok and long_horizon_model_valid and mode_path_ok
        and identity_ok and plant_ok and locomotion_schedule_ok and legacy_ok)
    last_two = grid >= window_end - 2.0
    summary = {
        "schema_version": 4,
        "stage": "D4_full_horizon_run_validation",
        "group": group,
        "round_index": config["round_index"],
        "acquisition_sequence": config["acquisition_sequence"],
        "pass": passed,
        "formal_eligibility": passed,
        "analysis_window_s": [window_start, window_end],
        "common_grid_dt_s": grid_dt,
        "common_grid_sample_count": len(grid),
        "last_observation_time_s": float(observation_time[-1]) if len(observation_time) else None,
        "sample_counts": {
            "estimate": len(estimates), "flow": len(flows),
            "plant": len(plants), "observation": len(observations),
            "policy_diagnostic": len(policies)},
        "checks": {
            "schema": schema_ok,
            "all_finite": all_finite,
            "window_coverage": coverage_ok,
            "strictly_increasing_time": monotonic_ok,
            "snapshot_synchronization_and_version": synchronization_ok,
            "long_horizon_estimated_model_valid": long_horizon_model_valid,
            "mode_flags_and_applied_path": mode_path_ok,
            "model_identity": identity_ok,
            "plant_truth_and_coordinates": plant_ok,
            "deterministic_target_and_locomotion_schedule": locomotion_schedule_ok,
            "legacy_6d_topics_suppressed": legacy_ok,
        },
        "expected": {
            "parameter_update": expected_update,
            "adaptive_dynamics": expected_dynamics,
            "payload_plant": expected_payload},
        "model_identity": identity,
        "observed_modes": observed_modes,
        "legacy_topic_counts": legacy_counts,
        "plant_pi_star_10d": truth.tolist(),
        "plant_truth_constant_error": truth_constant_error,
        "plant_coordinate_roundtrip_peak_error": roundtrip_peak,
        "plant_full_flow_residual_peak_norm": residual_peak,
        "estimated_total_mass_min_kg": float(np.min(estimated_total_mass)),
        "estimated_total_mass_max_kg": float(np.max(estimated_total_mass)),
        "estimated_total_inertia_min_eigenvalue": float(np.min(inertia_eigenvalues)),
        "peak_pi_hat_coordinate_norm": pi_peak,
        "peak_candidate_full_flow_norm": float(np.max(candidate_flow_norm)),
        "peak_applied_full_flow_norm": float(np.max(applied_flow_norm)),
        "max_candidate_applied_path_error": applied_path_error,
        "actual_x_displacement_over_analysis_window_m": float(
            actual_position[-1, 0] - actual_position[0, 0]),
        "final_pi_hat_10d": pi_hat[-1].tolist(),
        "last_2s_mean_pi_hat_10d": np.mean(pi_hat[last_two], axis=0).tolist(),
        "final_pi_error_10d": pi_error[-1].tolist(),
        "final_pi_error_coordinate_norm": float(vector_norm(pi_error[-1])),
        "tracking_descriptive_only": {
            "x_rmse_m": float(np.sqrt(np.mean(columns["x_error"] ** 2))),
            "x_iae_m_s": float(np.trapz(np.abs(columns["x_error"]), grid)),
            "peak_abs_x_error_m": float(np.max(np.abs(columns["x_error"]))),
            "position_rmse_m": float(np.sqrt(np.mean(columns["position_error_norm"] ** 2))),
            "peak_position_error_m": float(np.max(columns["position_error_norm"])),
            "sigma_mean_norm": float(np.mean(columns["sigma_norm"])),
            "sigma_peak_norm": float(np.max(columns["sigma_norm"])),
        },
        "artifacts": {"csv": str(csv_path), "plot": str(figure_path)},
        "scientific_scope": (
            "full-horizon run validity only until the three-group round-01 gate passes; "
            "no single-run performance inference"),
    }
    (run / "task_d_d4_run_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
