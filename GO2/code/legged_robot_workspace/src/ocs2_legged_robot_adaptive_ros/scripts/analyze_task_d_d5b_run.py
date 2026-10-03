#!/usr/bin/env python3
"""Validate and materialize one controlled D5B bias-attribution run."""

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
SCHEDULE = "/legged_robot_mpc_mode_schedule"
STATUS = "/task_c/locomotion_scheduler_status"
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy", "Jyz", "Jzz")


def interp(grid, times, values):
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    return np.column_stack([np.interp(grid, times, values[:, index])
                            for index in range(values.shape[1])])


def nearest(grid, times, values):
    times = np.asarray(times, dtype=float)
    values = np.asarray(values)
    right = np.clip(np.searchsorted(times, grid), 0, len(times) - 1)
    left = np.clip(right - 1, 0, len(times) - 1)
    choose = np.abs(grid - times[left]) <= np.abs(times[right] - grid)
    return values[np.where(choose, left, right)]


def norm(values):
    return np.linalg.norm(np.asarray(values, dtype=float), axis=-1)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_task_d_d5b_run.py RUN_DIR")
    run = Path(sys.argv[1]).resolve()
    config = json.loads((run / "effective_config.json").read_text())
    group = config["group"]
    expected_target = bool(config["expected_target_published"])
    expected_walk = bool(config["expected_walking"])
    expected_payload = bool(config["truePlantPayloadInertialEnabled"])
    start = float(config["analysis_window_start_mpc_time_s"])
    end = float(config["analysis_window_end_mpc_time_s"])
    dt = float(config["common_grid_dt_s"])
    grid = np.linspace(start, end, int(round((end - start) / dt)) + 1)

    estimates, flows, plants, observations, targets, statuses = [], [], [], [], [], []
    schedule_count, identity = 0, None
    labels = {ESTIMATE: set(), FLOW: set(), PLANT: set()}
    with rosbag.Bag(str(run / "task_d_d5b.bag")) as bag:
        for topic, message, _ in bag.read_messages(topics=[
                ESTIMATE, FLOW, PLANT, IDENTITY, OBSERVATION, TARGET,
                SCHEDULE, STATUS]):
            if topic in (ESTIMATE, FLOW, PLANT):
                labels[topic].add(message.layout.dim[0].label
                                  if message.layout.dim else "")
                {ESTIMATE: estimates, FLOW: flows, PLANT: plants}[topic].append(
                    [float(value) for value in message.data])
            elif topic == IDENTITY:
                identity = json.loads(message.data)
            elif topic == OBSERVATION:
                observations.append((float(message.time), int(message.mode),
                                     [float(value) for value in message.state.value],
                                     [float(value) for value in message.input.value]))
            elif topic == TARGET:
                targets.append((np.asarray(message.timeTrajectory, dtype=float),
                                np.asarray([value.value for value in message.stateTrajectory],
                                           dtype=float)))
            elif topic == SCHEDULE:
                schedule_count += 1
            elif topic == STATUS:
                statuses.append(json.loads(message.data))

    estimates.sort(key=lambda row: row[0])
    flows.sort(key=lambda row: row[0])
    plants.sort(key=lambda row: row[0])
    observations = [value for _, value in sorted(
        {row[0]: row for row in observations}.items())]
    estimate = np.asarray(estimates, dtype=float)
    flow = np.asarray(flows, dtype=float)
    observation_time = np.asarray([row[0] for row in observations], dtype=float)
    schema_ok = bool(
        estimate.ndim == 2 and estimate.shape[1] == 118 and len(estimate) >= 50
        and flow.ndim == 2 and flow.shape[1] == 80 and len(flow) >= 50
        and labels[ESTIMATE] == {"payload_inertial_10d_v3_118_fields"}
        and labels[FLOW] == {"payload_inertial_eq10_full_flow_v2_80_fields"}
        and len(observations) >= 50
        and all(len(row[2]) == 24 and len(row[3]) == 24 for row in observations))
    if expected_payload:
        schema_ok = bool(schema_ok and plants and all(len(row) == 74 for row in plants)
                         and labels[PLANT] == {"payload_inertial_plant_v2_74_fields"})
    else:
        schema_ok = bool(schema_ok and not plants)
    finite_ok = bool(schema_ok and np.all(np.isfinite(estimate))
                     and np.all(np.isfinite(flow))
                     and all(np.all(np.isfinite(row[2] + row[3])) for row in observations)
                     and (not plants or np.all(np.isfinite(plants))))
    coverage_ok = bool(estimate[0, 0] <= start and estimate[-1, 0] >= end
                       and flow[0, 0] <= start and flow[-1, 0] >= end
                       and observation_time[0] <= start and observation_time[-1] >= end)
    monotonic_ok = bool(np.all(np.diff(estimate[:, 0]) > 0.0)
                        and np.all(np.diff(flow[:, 0]) > 0.0)
                        and np.all(np.diff(observation_time) > 0.0))
    versions = np.rint(estimate[:, 3]).astype(int)
    sync_ok = bool(np.max(np.abs(estimate[:, 2])) <= 1.0e-6
                   and np.all(np.diff(versions) > 0))
    flags_ok = bool(np.all(estimate[:, 4] == 1.0)
                    and np.all(estimate[:, 5] == 1.0)
                    and np.all(estimate[:, 6] == 1.0)
                    and np.all(estimate[:, 7] == 10.0)
                    and np.all(estimate[:, 9] == 1.0))
    y = estimate[:, 48:108].reshape(-1, 6, 10)
    sigma_raw = estimate[:, 42:48]
    derivative_error = float(np.max(np.abs(
        estimate[:, 32:42] - np.einsum("nij,ni->nj", y, sigma_raw))))
    applied_error = float(np.max(np.abs(flow[:, 32:56] - flow[:, 56:80])))
    equation_paths_ok = derivative_error <= 1.0e-10 and applied_error <= 1.0e-12

    observed_modes = sorted(set(row[1] for row in observations))
    condition_status = [value for value in statuses
                        if value.get("experiment_condition") ==
                        config["scheduler_condition"]]
    if expected_target:
        target_ok = bool(len(targets) == 1 and
                         any(value.get("stage") == "target_published"
                             for value in condition_status))
        if target_ok:
            target_time, target_state = targets[0]
            target_ok = bool(len(target_time) == 2 and target_state.shape == (2, 24)
                             and abs(target_time[0] - 1.2) <= 1.0e-9
                             and abs(target_time[1] - 1.4) <= 1.0e-9
                             and abs(target_state[1, 6] - target_state[0, 6] - 0.1)
                             <= 1.0e-6)
    else:
        target_ok = bool(not targets and any(
            value.get("stage") == "condition_ready"
            and value.get("target_published") is False for value in condition_status))
        target_time, target_state = np.asarray([]), np.empty((0, 24))
    gait_ok = bool((expected_walk and schedule_count == 1 and any(
                        mode != 15 for mode in observed_modes))
                   or (not expected_walk and schedule_count == 0
                       and observed_modes == [15]))
    schedule_ok = bool(target_ok and gait_ok and condition_status)

    identity_ok = bool(identity and identity.get("schema_version") == 2)
    if expected_payload:
        identity_ok = bool(identity_ok
                           and identity.get("payload_inertial_override_active") is True
                           and identity.get("plant_true_total_mass", 0.0) >
                           identity.get("controller_nominal_mass", math.inf))
        truth = np.asarray(plants[0][14:24], dtype=float)
    else:
        identity_ok = bool(identity_ok
                           and identity.get("payload_inertial_override_active") is False)
        truth = np.zeros(10)

    estimate_grid = interp(grid, estimate[:, 0], estimate)
    flow_grid = interp(grid, flow[:, 0], flow)
    state_grid = interp(grid, observation_time, [row[2] for row in observations])
    input_grid = interp(grid, observation_time, [row[3] for row in observations])
    mode_grid = nearest(grid, observation_time, [row[1] for row in observations]).astype(int)
    pi_hat = estimate_grid[:, 22:32]
    pi_dot = estimate_grid[:, 32:42]
    sigma = estimate_grid[:, 42:48]
    y_grid = estimate_grid[:, 48:108].reshape(-1, 6, 10)
    drive = np.einsum("nij,ni->nj", y_grid, sigma)
    if expected_target:
        desired = interp(grid, target_time, target_state)
        anchor_position = desired[:, 6:9]
    else:
        anchor_position = np.tile(state_grid[0, 6:9], (len(grid), 1))
    position_error = anchor_position - state_grid[:, 6:9]

    columns = {
        "time_s": grid, "mode": mode_grid,
        "actual_x": state_grid[:, 6], "actual_y": state_grid[:, 7],
        "actual_z": state_grid[:, 8], "anchor_x": anchor_position[:, 0],
        "anchor_y": anchor_position[:, 1], "anchor_z": anchor_position[:, 2],
        "x_error": position_error[:, 0],
        "position_error_norm": norm(position_error), "sigma_norm": norm(sigma),
        "candidate_correction_norm": norm(flow_grid[:, 32:56]),
        "applied_correction_norm": norm(flow_grid[:, 56:80]),
    }
    for index in range(6):
        columns["sigma_{}".format(index)] = sigma[:, index]
    for index, name in enumerate(PARAMETERS):
        columns["pi_hat_" + name] = pi_hat[:, index]
        columns["pi_dot_" + name] = pi_dot[:, index]
        columns["drive_" + name] = drive[:, index]
        columns["pi_star_" + name] = np.full(len(grid), truth[index])
    for index in range(24):
        columns["policy_{}".format(index)] = input_grid[:, index]
    csv_path = run / "task_d_d5b_run.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(list(columns))
        for row in range(len(grid)):
            writer.writerow([columns[name][row] for name in columns])

    fig, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    axes[0].plot(grid, columns["anchor_x"], "k--", label="target/hold anchor")
    axes[0].plot(grid, columns["actual_x"], label="actual x")
    axes[1].plot(grid, columns["pi_hat_m"], label="pi_hat m")
    axes[1].plot(grid, columns["pi_hat_hz"], label="pi_hat hz")
    axes[2].plot(grid, columns["pi_hat_Jxx"], label="pi_hat Jxx")
    axes[2].plot(grid, columns["pi_hat_Jyy"], label="pi_hat Jyy")
    axes[2].plot(grid, columns["pi_hat_Jzz"], label="pi_hat Jzz")
    axes[3].plot(grid, columns["sigma_norm"], label="||sigma||")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[-1].set_xlabel("MPC time [s]")
    fig.suptitle("D5B {} {}".format(group, run.name))
    fig.tight_layout()
    figure_path = run / "task_d_d5b_overview.png"
    fig.savefig(figure_path, dpi=160)
    plt.close(fig)

    passed = bool(schema_ok and finite_ok and coverage_ok and monotonic_ok
                  and sync_ok and flags_ok and equation_paths_ok and schedule_ok
                  and identity_ok)
    last_two = grid >= grid[-1] - min(2.0, 0.5 * (grid[-1] - grid[0]))
    summary = {
        "schema_version": 1, "stage": "D5B_run_validation",
        "group": group, "run_index": config["run_index"],
        "run_scope": config["run_scope"], "pass": passed,
        "formal_eligibility": bool(passed and config["run_scope"] == "formal"),
        "analysis_window_s": [start, end], "sample_count": len(grid),
        "checks": {
            "schema": schema_ok, "finite": finite_ok, "coverage": coverage_ok,
            "monotonic": monotonic_ok, "snapshot_sync": sync_ok,
            "controller_flags": flags_ok, "equation_paths": equation_paths_ok,
            "deterministic_condition": schedule_ok, "model_identity": identity_ok,
        },
        "scheduler_condition": config["scheduler_condition"],
        "observed_modes": observed_modes, "target_message_count": len(targets),
        "gait_schedule_message_count": schedule_count,
        "eq8_derivative_max_error": derivative_error,
        "eq10_applied_max_error": applied_error,
        "pi_star": truth.tolist(),
        "metrics": {
            "sigma_mean_norm": float(np.trapz(norm(sigma), grid) / (end - start)),
            "sigma_peak_norm": float(np.max(norm(sigma))),
            "actual_x_displacement_m": float(state_grid[-1, 6] - state_grid[0, 6]),
            "position_rmse_m": float(np.sqrt(np.trapz(norm(position_error) ** 2, grid)
                                             / (end - start))),
            "applied_correction_rms": float(np.sqrt(np.trapz(
                norm(flow_grid[:, 56:80]) ** 2, grid) / (end - start))),
            "pi_last2s_mean": np.mean(pi_hat[last_two], axis=0).tolist(),
            "pi_net_change": (pi_hat[-1] - pi_hat[0]).tolist(),
            "drive_rms": np.sqrt(np.trapz(drive ** 2, grid, axis=0)
                                  / (end - start)).tolist(),
            "drive_signed_integral": np.trapz(drive, grid, axis=0).tolist(),
        },
    }
    (run / "task_d_d5b_run_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
