#!/usr/bin/env python3
"""Analyze one Nominal/estimator-OFF D5I2 online safety-information smoke."""

import csv
import hashlib
import json
import math
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ESTIMATE = "/task_d/payload_inertial_10d_v3_sample"
FLOW = "/task_d/equation10_full_flow_sample"
REFERENCE = "/task_d/d5i1_reference_sample"
SCHEDULER = "/task_d/d5i2_scheduler_sample"
SCHEDULER_IDENTITY = "/task_d/d5i2_scheduler_identity"
MODEL_IDENTITY = "/task_d/model_identity"
OBSERVATION = "/legged_robot_mpc_observation"
TARGET = "/legged_robot_mpc_target"
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy",
              "Jyz", "Jzz")
MASS = 5.213485
LENGTH = 0.18601075237738277
PI_STAR_DESIGN = np.array([
    5.213485, 0.8341576, -0.15640455, 0.46921365, 0.126921365,
    0.025024728, -0.075074184, 0.2756944445, 0.0140764095,
    0.2581573525,
])


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def interpolate_matrix(grid, times, values):
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    return np.column_stack([
        np.interp(grid, times, values[:, index])
        for index in range(values.shape[1])])


def trapezoid_weights(time):
    time = np.asarray(time, dtype=float)
    if len(time) < 2 or not np.all(np.diff(time) > 0.0):
        raise ValueError("time must be strictly increasing")
    dt = np.diff(time)
    weights = np.empty(len(time))
    weights[0], weights[-1] = 0.5 * dt[0], 0.5 * dt[-1]
    if len(time) > 2:
        weights[1:-1] = 0.5 * (dt[:-1] + dt[1:])
    return weights


def dimensionless_design(regressor, weights):
    d_pi = np.diag([MASS] + [MASS * LENGTH] * 3 +
                   [MASS * LENGTH * LENGTH] * 6)
    d_w_inverse = np.diag([1.0, 1.0, 1.0] + [1.0 / LENGTH] * 3)
    scaled = np.einsum("ab,nbc,cd->nad", d_w_inverse, regressor, d_pi)
    return (scaled * np.sqrt(weights)[:, None, None]).reshape(-1, 10), d_pi


def effective_rank(singular):
    power = np.square(singular)
    if not np.any(power > 0.0):
        return 0.0
    probability = power / np.sum(power)
    probability = probability[probability > 0.0]
    return float(np.exp(-np.sum(probability * np.log(probability))))


def information_metrics(time, regressor):
    weights = trapezoid_weights(time)
    matrix, d_pi = dimensionless_design(regressor, weights)
    gram = matrix.T @ matrix
    eigenvalues, eigenvectors = np.linalg.eigh(gram)
    order = np.argsort(eigenvalues)[::-1]
    singular = np.sqrt(np.maximum(eigenvalues[order], 0.0))
    right = eigenvectors[:, order].T
    ratios = singular / singular[0] if singular[0] > 0.0 else np.zeros(10)
    observable = ratios >= 1.0e-3
    theta = np.linalg.solve(d_pi, PI_STAR_DESIGN)
    projection = (right[observable].T @ (right[observable] @ theta)
                  if np.any(observable) else np.zeros(10))
    observable_fraction = float(np.linalg.norm(projection) /
                                np.linalg.norm(theta))
    duration = float(time[-1] - time[0])
    column_rms = np.sqrt(np.sum(np.square(matrix), axis=0) /
                         max(6.0 * duration, 1.0e-12))
    minimum_gate = bool(np.count_nonzero(observable) == 10 and
                        ratios[-1] >= 1.0e-3 and
                        observable_fraction >= 0.9)
    strict_d5i1_target = bool(minimum_gate and ratios[-1] >=
                             10.0 * 0.0003458642441519824)
    return {
        "singular_values": singular.tolist(),
        "singular_value_ratios": ratios.tolist(),
        "rank_at_1e_2": int(np.count_nonzero(ratios >= 1.0e-2)),
        "rank_at_1e_3": int(np.count_nonzero(observable)),
        "smallest_ratio": float(ratios[-1]),
        "condition_number": (float(singular[0] / singular[-1])
                             if singular[-1] > 0.0 else None),
        "effective_rank": effective_rank(singular),
        "hypothetical_payload_observable_fraction_at_1e_3":
            observable_fraction,
        "time_normalized_dimensionless_column_rms":
            dict(zip(PARAMETERS, map(float, column_rms))),
        "weakest_direction": dict(zip(PARAMETERS, map(float, right[-1]))),
        "minimum_online_information_gate_passed": minimum_gate,
        "strict_d5i1_10x_weakest_ratio_target_passed": strict_d5i1_target,
    }


def deduplicate(rows):
    return np.asarray([row for _, row in sorted(
        {float(row[0]): row for row in rows}.items())], dtype=float)


def main():
    import rosbag
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_task_d_d5i2_smoke.py RUN_DIR")
    run = Path(sys.argv[1]).resolve()
    config = json.loads((run / "effective_config.json").read_text())
    artifact_stem = config.get("analysis_artifact_stem", "task_d_d5i2")
    display_stage = config.get("analysis_display_stage", "D5I2")
    summary_stage = config.get(
        "analysis_summary_stage",
        "D5I2_nominal_estimator_off_online_safety_information_smoke")
    bag_path = run / config.get("bag_filename", "task_d_d5i2.bag")

    estimates, flows, references, schedulers, observations = [], [], [], [], []
    targets, identities, scheduler_identities = [], [], []
    labels = {ESTIMATE: set(), FLOW: set(), REFERENCE: set(), SCHEDULER: set()}
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _ in bag.read_messages(topics=[
                ESTIMATE, FLOW, REFERENCE, SCHEDULER, SCHEDULER_IDENTITY,
                MODEL_IDENTITY, OBSERVATION, TARGET]):
            if topic in labels:
                labels[topic].add(message.layout.dim[0].label
                                  if message.layout.dim else "")
                {ESTIMATE: estimates, FLOW: flows, REFERENCE: references,
                 SCHEDULER: schedulers}[topic].append(
                    [float(value) for value in message.data])
            elif topic == MODEL_IDENTITY:
                identities.append(json.loads(message.data))
            elif topic == SCHEDULER_IDENTITY:
                scheduler_identities.append(json.loads(message.data))
            elif topic == OBSERVATION:
                observations.append([
                    float(message.time), float(message.mode),
                    *map(float, message.state.value),
                    *map(float, message.input.value)])
            elif topic == TARGET:
                targets.append((np.asarray(message.timeTrajectory, dtype=float),
                                np.asarray([state.value for state in
                                            message.stateTrajectory], dtype=float),
                                np.asarray([value.value for value in
                                            message.inputTrajectory], dtype=float)))

    estimates = deduplicate(estimates)
    flows = deduplicate(flows)
    references = deduplicate(references)
    schedulers = deduplicate(schedulers)
    observations = deduplicate(observations)
    expected_lengths = {ESTIMATE: 118, FLOW: 80, REFERENCE: 39, SCHEDULER: 28}
    expected_labels = {
        ESTIMATE: "payload_inertial_10d_v3_118_fields",
        FLOW: "payload_inertial_eq10_full_flow_v2_80_fields",
        REFERENCE: "d5i1_reference_v1_39_fields",
        SCHEDULER: "d5i2_scheduler_v1_28_fields",
    }
    arrays = {ESTIMATE: estimates, FLOW: flows, REFERENCE: references,
              SCHEDULER: schedulers}
    schema_ok = all(
        len(array) >= 100 and array.ndim == 2 and
        array.shape[1] == expected_lengths[topic] and
        labels[topic] == {expected_labels[topic]}
        for topic, array in arrays.items())
    all_finite = bool(schema_ok and len(observations) >= 100 and
                      all(np.all(np.isfinite(value)) for value in
                          list(arrays.values()) + [observations]))

    identity = identities[-1] if identities else {}
    scheduler_identity = scheduler_identities[-1] if scheduler_identities else {}
    identity_ok = bool(
        identity.get("schema_version") == 2 and
        identity.get("plant_model_id") == "nominal_ocs2_rollout_v1" and
        identity.get("payload_inertial_override_active") is False and
        abs(identity.get("plant_true_total_mass", math.inf) -
            identity.get("controller_nominal_mass", -math.inf)) <= 1.0e-12)
    schedule_config = config["schedule"]
    scheduler_identity_ok = bool(
        scheduler_identity.get("stage") ==
            "D5I2_online_safety_information_smoke" and
        scheduler_identity.get("candidate") == config["candidate"] and
        abs(scheduler_identity.get("knot_dt", math.inf) -
            schedule_config["knot_dt_s"]) <= 1.0e-12 and
        abs(scheduler_identity.get("complete_time", math.inf) -
            schedule_config["complete_time_s"]) <= 1.0e-12 and
        scheduler_identity.get("plant_override") is False)

    target_ok = False
    target_times = np.asarray([])
    target_states = np.empty((0, 0))
    if targets:
        target_times, target_states, target_inputs = targets[-1]
        expected_count = int(round(schedule_config["complete_time_s"] /
                                   schedule_config["knot_dt_s"])) + 1
        target_ok = bool(
            len(target_times) == expected_count and
            target_states.shape == (expected_count, 24) and
            target_inputs.shape == (expected_count, 24) and
            np.all(np.diff(target_times) > 0.0) and
            abs(target_times[0]) <= 1.0e-12 and
            abs(target_times[-1] - schedule_config["complete_time_s"]) <=
                1.0e-12 and
            np.all(np.isfinite(target_states)) and
            np.all(np.isfinite(target_inputs)))

    mode_flags_ok = bool(
        schema_ok and
        np.all(estimates[:, 4] == 0.0) and
        np.all(estimates[:, 5] == 0.0) and
        np.all(estimates[:, 6] == 1.0) and
        np.all(estimates[:, 7] == 10.0) and
        np.max(np.abs(estimates[:, 22:32])) <= 1.0e-12 and
        np.all(flows[:, 4] == 0.0) and np.all(flows[:, 5] == 0.0) and
        np.all(flows[:, 7] == 0.0) and
        np.max(np.abs(flows[:, 32:80])) <= 1.0e-12)
    versions = np.rint(references[:, 3]).astype(np.int64) if len(references) else []
    synchronization_ok = bool(
        len(references) and np.max(np.abs(references[:, 2])) <= 1.0e-6 and
        np.all(np.diff(versions) > 0) and
        np.max(np.abs(estimates[:, 2])) <= 1.0e-6)

    # Compare the analytic scheduler truth with the values actually decoded by
    # AdaptiveReferenceKinematics.  The latter is the signal used in sigma/Y.
    ref_time = references[:, 0]
    common_mask = ((ref_time >= schedulers[0, 0]) &
                   (ref_time <= schedulers[-1, 0])) if len(schedulers) else np.zeros(len(ref_time), bool)
    ref_time_common = ref_time[common_mask]
    scheduler_on_ref = interpolate_matrix(ref_time_common, schedulers[:, 0],
                                          schedulers[:, 1:])
    ref_common = references[common_mask]
    position_error = ref_common[:, 5:8] - scheduler_on_ref[:, 8:11]
    velocity_error = ref_common[:, 12:15] - scheduler_on_ref[:, 11:14]
    acceleration_error = ref_common[:, 15:18] - scheduler_on_ref[:, 14:17]
    omega_error = ref_common[:, 18:21] - scheduler_on_ref[:, 20:23]
    alpha_error = ref_common[:, 21:24] - scheduler_on_ref[:, 23:26]
    transport = {
        "matched_sample_count": int(len(ref_common)),
        "position_max_error_m": float(np.max(np.linalg.norm(position_error, axis=1))),
        "velocity_max_error_m_s": float(np.max(np.linalg.norm(velocity_error, axis=1))),
        "acceleration_max_error_m_s2": float(np.max(np.linalg.norm(acceleration_error, axis=1))),
        "angular_velocity_max_error_rad_s": float(np.max(np.linalg.norm(omega_error, axis=1))),
        "angular_acceleration_max_error_rad_s2": float(np.max(np.linalg.norm(alpha_error, axis=1))),
        "encoded_flag_all_true": bool(np.all(ref_common[:, 4] == 1.0)),
    }
    transport_ok = bool(
        len(ref_common) >= 100 and transport["encoded_flag_all_true"] and
        transport["position_max_error_m"] <= 5.0e-4 and
        transport["velocity_max_error_m_s"] <= 5.0e-3 and
        transport["acceleration_max_error_m_s2"] <= 5.0e-2 and
        transport["angular_velocity_max_error_rad_s"] <= 5.0e-3 and
        transport["angular_acceleration_max_error_rad_s2"] <= 5.0e-2)

    steady_start = schedule_config["profile_start_time_s"] + schedule_config["ramp_in_duration_s"]
    steady_end = steady_start + schedule_config["steady_duration_s"]
    steady_mask = ((estimates[:, 0] >= steady_start) &
                   (estimates[:, 0] <= steady_end))
    steady_time = estimates[steady_mask, 0]
    regressor = estimates[steady_mask, 48:108].reshape(-1, 6, 10)
    information = information_metrics(steady_time, regressor)

    observed_modes = [int(value) for value in sorted(
        set(np.rint(observations[:, 1]).astype(int)))]
    expected_modes = {7, 11, 13, 14, 15}
    stage_values = set(int(value) for value in
                       np.rint(schedulers[:, 6]).astype(int))
    gait_ok = bool(set(observed_modes).issubset(expected_modes) and
                   any(mode != 15 for mode in observed_modes) and
                   {1, 2, 3, 4}.issubset(stage_values))
    coverage_ok = bool(
        observations[0, 0] < schedule_config["profile_start_time_s"] and
        observations[-1, 0] >= schedule_config["complete_time_s"] and
        estimates[0, 0] <= steady_start and estimates[-1, 0] >= steady_end and
        schedulers[-1, 0] >= schedule_config["complete_time_s"])

    target_on_observation = interpolate_matrix(
        observations[:, 0], target_times, target_states)
    actual_state = observations[:, 2:26]
    track_mask = ((observations[:, 0] >= schedule_config["profile_start_time_s"]) &
                  (observations[:, 0] <= steady_end +
                   schedule_config["ramp_out_duration_s"]))
    position_tracking = target_on_observation[:, 6:9] - actual_state[:, 6:9]
    orientation_tracking = ((target_on_observation[:, 9:12] -
                             actual_state[:, 9:12] + np.pi) %
                            (2.0 * np.pi) - np.pi)
    tracking = {
        "position_rmse_m": float(np.sqrt(np.mean(np.sum(
            np.square(position_tracking[track_mask]), axis=1)))),
        "position_peak_m": float(np.max(np.linalg.norm(
            position_tracking[track_mask], axis=1))),
        "orientation_rmse_rad": float(np.sqrt(np.mean(np.sum(
            np.square(orientation_tracking[track_mask]), axis=1)))),
        "orientation_peak_rad": float(np.max(np.linalg.norm(
            orientation_tracking[track_mask], axis=1))),
        "sigma_peak": float(np.max(np.linalg.norm(estimates[:, 42:48], axis=1))),
        "contact_force_peak": float(np.max(np.abs(observations[:, 26:38]))),
    }
    safety_ok = bool(
        all_finite and coverage_ok and gait_ok and
        tracking["position_peak_m"] < 0.20 and
        tracking["orientation_peak_rad"] < 0.50 and
        tracking["contact_force_peak"] < 1.0e5)

    launch_text = (run / "roslaunch.log").read_text(errors="replace")
    fatal_pattern = re.compile(
        r"\[FATAL\]|Segmentation fault|terminate called|ROSException|nan detected",
        re.IGNORECASE)
    runtime_log_ok = fatal_pattern.search(launch_text) is None
    acquisition_valid = bool(schema_ok and target_ok and identity_ok and
                             scheduler_identity_ok and mode_flags_ok and
                             synchronization_ok and transport_ok and
                             safety_ok and runtime_log_ok)
    candidate_admitted = bool(
        acquisition_valid and
        information["minimum_online_information_gate_passed"])

    # A compact common-grid CSV for direct inspection.
    grid = ref_time_common
    observation_on_grid = interpolate_matrix(grid, observations[:, 0], observations[:, 1:])
    target_on_grid = interpolate_matrix(grid, target_times, target_states)
    estimate_on_grid = interpolate_matrix(grid, estimates[:, 0], estimates[:, 1:])
    columns = [
        "time_s", "stage", "envelope", "desired_base_x", "actual_base_x",
        "base_position_error_norm", "base_orientation_error_norm",
        "sigma_norm", "pi_hat_norm", "Y_frobenius_norm"]
    target_position = target_on_grid[:, 6:9]
    actual_position = observation_on_grid[:, 7:10]
    target_rpy = target_on_grid[:, 9:12]
    actual_rpy = observation_on_grid[:, 10:13]
    scheduler_grid = interpolate_matrix(grid, schedulers[:, 0], schedulers[:, 1:])
    y_grid = estimate_on_grid[:, 47:107].reshape(-1, 6, 10)
    data = np.column_stack([
        grid,
        np.rint(scheduler_grid[:, 5]),
        scheduler_grid[:, 2],
        target_position[:, 0], actual_position[:, 0],
        np.linalg.norm(target_position - actual_position, axis=1),
        np.linalg.norm((target_rpy - actual_rpy + np.pi) % (2*np.pi) - np.pi,
                       axis=1),
        np.linalg.norm(estimate_on_grid[:, 41:47], axis=1),
        np.linalg.norm(estimate_on_grid[:, 21:31], axis=1),
        np.linalg.norm(y_grid, axis=(1, 2)),
    ])
    with (run / (artifact_stem + "_timeseries.csv")).open(
            "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(data)

    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    axes[0].plot(grid, target_position[:, 0], label="desired base x")
    axes[0].plot(grid, actual_position[:, 0], label="actual base x")
    axes[0].set_ylabel("x [m]")
    axes[1].plot(grid, data[:, 5], label="position error norm")
    axes[1].plot(grid, data[:, 6], label="orientation error norm")
    axes[1].set_ylabel("tracking error")
    axes[2].plot(grid, data[:, 7], label="||sigma||")
    axes[2].plot(grid, data[:, 8], label="||pi_hat||")
    axes[2].set_ylabel("adaptive signals")
    axes[3].plot(grid, data[:, 2], label="C2 envelope")
    axes[3].set_ylabel("envelope")
    axes[3].set_xlabel("MPC time [s]")
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8)
        axis.axvspan(steady_start, steady_end, color="tab:green", alpha=0.08)
    fig.suptitle("{} {} Nominal / estimator OFF".format(
        display_stage, config["candidate"]))
    fig.tight_layout()
    fig.savefig(run / ("figure_{}_online_smoke.png".format(
        artifact_stem.replace("task_d_", ""))), dpi=180)
    plt.close(fig)

    ratios = information["singular_value_ratios"]
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.semilogy(range(1, 11), ratios, marker="o", label="executed Y(t)")
    ax.axhline(1.0e-3, color="black", linestyle="--", label="minimum rank threshold")
    ax.axhline(10.0 * 0.0003458642441519824, color="tab:red",
               linestyle=":", label="D5I1 strict 10x target")
    ax.set(xlabel="Singular direction", ylabel="s_i / s_max",
           title="{} executed fully-dimensionless regressor spectrum".format(
               display_stage))
    ax.grid(True, which="both", alpha=0.25)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(run / ("figure_{}_executed_spectrum.png".format(
        artifact_stem.replace("task_d_", ""))), dpi=180)
    plt.close(fig)

    summary = {
        "schema_version": 1,
        "stage": summary_stage,
        "candidate": config["candidate"],
        "acquisition_valid": acquisition_valid,
        "candidate_admitted_for_next_stage": candidate_admitted,
        "checks": {
            "schema_ok": schema_ok,
            "all_finite": all_finite,
            "coverage_ok": coverage_ok,
            "target_ok": target_ok,
            "controller_plant_identity_ok": identity_ok,
            "scheduler_identity_ok": scheduler_identity_ok,
            "estimator_and_dynamics_off_ok": mode_flags_ok,
            "snapshot_synchronization_ok": synchronization_ok,
            "reference_transport_ok": transport_ok,
            "gait_and_phase_ok": gait_ok,
            "safety_ok": safety_ok,
            "runtime_log_ok": runtime_log_ok,
        },
        "transport": transport,
        "tracking": tracking,
        "executed_information": information,
        "observed_modes": observed_modes,
        "observed_stage_codes": sorted(stage_values),
        "topic_labels": {key: sorted(value) for key, value in labels.items()},
        "model_identity": identity,
        "scheduler_identity": scheduler_identity,
        "bag": str(bag_path),
        "bag_sha256": sha256(bag_path),
        "scientific_boundary": (
            "This smoke checks online safety, reference transport, and the executed "
            "regressor only. Estimator convergence, payload reconstruction, baseline "
            "training, and tracking benefit are intentionally not evaluated."),
    }
    (run / (artifact_stem + "_summary.json")).write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    if not acquisition_valid:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
