#!/usr/bin/env python3
"""Validate one D5H1 applied estimator-update smoke bag."""

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
V3 = "/task_d/payload_inertial_10d_v3_sample"
FLOW = "/task_d/equation10_full_flow_sample"
OBSERVATION = "/legged_robot_mpc_observation"
TARGET = "/legged_robot_mpc_target"
STATUS = "/task_c/locomotion_scheduler_status"
D5H1_LABEL = "payload_inertial_d5h1_applied_v1_71_fields"
V3_LABEL = "payload_inertial_10d_v3_118_fields"
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz",
              "Jyy", "Jyz", "Jzz")


def finite(values):
    return bool(np.all(np.isfinite(np.asarray(values, dtype=float))))


def match_rows(left, right, tolerance=1.0e-9):
    left = np.asarray(left, dtype=float)
    right = np.asarray(right, dtype=float)
    matched_left, matched_right = [], []
    index = 0
    for row in left:
        while index + 1 < len(right) and right[index, 0] < row[0] - tolerance:
            index += 1
        candidates = [index]
        if index + 1 < len(right):
            candidates.append(index + 1)
        best = min(candidates, key=lambda item: abs(right[item, 0] - row[0]))
        if abs(right[best, 0] - row[0]) <= tolerance:
            matched_left.append(row)
            matched_right.append(right[best])
    return np.asarray(matched_left), np.asarray(matched_right)


def physical_scales(config):
    payload = config["payload"]
    mass = float(payload["mass_kg"])
    com = np.asarray(payload["com_nominal_c0_base_m"], dtype=float)
    length = float(np.linalg.norm(com))
    inertia_scale = max(float(value) for value in
                        payload["inertia_com_base_kg_m2"])
    return np.asarray([mass, mass * length, mass * length, mass * length] +
                      [inertia_scale] * 6)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_task_d_d5h1_smoke.py RUN_DIR")
    run = Path(sys.argv[1]).resolve()
    config = json.loads((run / "effective_config.json").read_text())
    expected_correction = bool(config["enableGaitPhaseBaselineCorrection"])
    bag_path = run / "task_d_d5h1.bag"
    d5_rows, v3_rows, flows, observations, targets, statuses = [], [], [], [], [], []
    labels = {D5H1: set(), V3: set()}
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _ in bag.read_messages(
                topics=[D5H1, V3, FLOW, OBSERVATION, TARGET, STATUS]):
            if topic in (D5H1, V3):
                labels[topic].add(message.layout.dim[0].label
                                  if message.layout.dim else "")
                {D5H1: d5_rows, V3: v3_rows}[topic].append(
                    [float(value) for value in message.data])
            elif topic == FLOW:
                flows.append([float(value) for value in message.data])
            elif topic == OBSERVATION:
                observations.append((
                    float(message.time), int(message.mode),
                    [float(value) for value in message.state.value],
                    [float(value) for value in message.input.value]))
            elif topic == TARGET:
                targets.append((
                    np.asarray(message.timeTrajectory, dtype=float),
                    np.asarray([value.value for value in message.stateTrajectory],
                               dtype=float)))
            else:
                statuses.append(json.loads(message.data))

    d5_rows.sort(key=lambda row: row[0])
    v3_rows.sort(key=lambda row: row[0])
    observations.sort(key=lambda row: row[0])
    schema_ok = bool(
        len(d5_rows) >= 100 and len(v3_rows) >= 100 and len(flows) >= 100
        and all(len(row) == 71 for row in d5_rows)
        and all(len(row) == 118 for row in v3_rows)
        and all(len(row) == 80 for row in flows)
        and labels[D5H1] == {D5H1_LABEL}
        and labels[V3] == {V3_LABEL})
    all_finite = bool(schema_ok and
                      all(finite(row) for row in d5_rows + v3_rows + flows) and
                      all(finite(row[2]) and finite(row[3])
                          for row in observations))
    d5 = np.asarray(d5_rows, dtype=float)
    v3 = np.asarray(v3_rows, dtype=float)
    matched_d5, matched_v3 = match_rows(d5, v3)
    match_fraction = len(matched_d5) / max(len(d5), len(v3), 1)

    raw = matched_d5[:, 15:25]
    baseline = matched_d5[:, 25:35]
    applied = matched_d5[:, 35:45]
    derivative = matched_d5[:, 45:55]
    pi_hat = matched_d5[:, 55:65]
    sigma = matched_d5[:, 65:71]
    reconstructed_raw = np.einsum(
        "nij,ni->nj", matched_v3[:, 48:108].reshape(-1, 6, 10),
        matched_v3[:, 42:48])
    raw_closure = float(np.max(np.abs(raw - reconstructed_raw)))
    applied_closure = float(np.max(np.abs(applied - (raw - baseline))))
    derivative_closure = float(np.max(np.abs(derivative - applied)))
    v3_derivative_closure = float(np.max(np.abs(
        derivative - matched_v3[:, 32:42])))
    pi_closure = float(np.max(np.abs(pi_hat - matched_v3[:, 22:32])))
    sigma_closure = float(np.max(np.abs(sigma - matched_v3[:, 42:48])))

    time = d5[:, 0]
    versions = d5[:, 3]
    dt = np.diff(time)
    euler_error = pi_hat[1:] - pi_hat[:-1] - dt[:, None] * derivative[1:]
    euler_closure = float(np.max(np.abs(euler_error)))
    time_sync = float(np.max(np.abs(d5[:, 2])))
    correction_flags = d5[:, 7] == 1.0
    configured_flags = d5[:, 5] == 1.0
    if expected_correction:
        mode_contract = bool(np.all(configured_flags) and np.any(correction_flags)
                             and np.max(np.linalg.norm(
                                 baseline[correction_flags], axis=1)) > 0.0)
    else:
        mode_contract = bool(not np.any(configured_flags)
                             and not np.any(correction_flags)
                             and np.max(np.abs(baseline)) == 0.0)
    phase_contract = bool(
        np.all((d5[correction_flags, 9] >= 0) &
               (d5[correction_flags, 9] < 24))
        if np.any(correction_flags) else not expected_correction)

    versions_ok = bool(np.all(np.diff(versions) > 0.0))
    closures_ok = bool(
        raw_closure <= 1.0e-12 and applied_closure <= 1.0e-12
        and derivative_closure <= 1.0e-12
        and v3_derivative_closure <= 1.0e-12
        and pi_closure <= 1.0e-12 and sigma_closure <= 1.0e-12
        and euler_closure <= 1.0e-10)
    target_status = [item for item in statuses
                     if item.get("stage") == "target_published"]
    schedule_ok = bool(target_status and targets and
                       abs(target_status[-1].get("target_start_time", -1.0) -
                           1.2) <= 1.0e-12)
    if schedule_ok:
        target_time, target_state = targets[-1]
        schedule_ok = bool(
            len(target_time) == 2 and target_state.shape == (2, 24)
            and abs(target_time[0] - 1.2) <= 1.0e-9
            and abs(target_time[1] - 1.4) <= 1.0e-9
            and abs(target_state[1, 6] - target_state[0, 6] - 0.1) <= 1.0e-6)
    observation_ok = bool(
        len(observations) >= 100
        and all(len(row[2]) == 24 and len(row[3]) == 24
                for row in observations)
        and any(row[1] != 15 for row in observations))

    scales = physical_scales(config)
    terminal_scaled_norm = float(np.linalg.norm(pi_hat[-1] / scales))
    peak_scaled_norm = float(np.max(np.linalg.norm(pi_hat / scales, axis=1)))
    correction_duration = float(np.sum(
        np.where(correction_flags[1:], dt, 0.0)))

    observation_time = np.asarray([row[0] for row in observations])
    observation_state = np.asarray([row[2] for row in observations])
    desired_x = np.interp(observation_time, targets[-1][0],
                          targets[-1][1][:, 6]) if targets else np.zeros(len(observations))
    actual_x = observation_state[:, 6]
    x_error = desired_x - actual_x
    x_rmse = float(np.sqrt(np.mean(x_error ** 2)))

    csv_path = run / "task_d_d5h1_timeseries.csv"
    names = (["time", "mode", "phase_bin", "correction_applied"] +
             ["raw_" + name for name in PARAMETERS] +
             ["baseline_" + name for name in PARAMETERS] +
             ["applied_" + name for name in PARAMETERS] +
             ["pi_dot_" + name for name in PARAMETERS] +
             ["pi_hat_" + name for name in PARAMETERS])
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        for index in range(len(d5)):
            writer.writerow([d5[index, 0], int(round(d5[index, 10])),
                             int(round(d5[index, 9])), d5[index, 7]] +
                            d5[index, 15:65].tolist())

    figure_path = run / "task_d_d5h1_smoke.png"
    figure, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=False)
    axes[0].plot(time, np.linalg.norm(raw, axis=1), label="raw drive")
    axes[0].plot(time, np.linalg.norm(applied, axis=1), label="applied drive")
    axes[0].step(time, correction_flags.astype(float), where="post",
                 label="correction active")
    for parameter in (3, 4, 7):
        axes[1].plot(time, pi_hat[:, parameter], label=PARAMETERS[parameter])
    axes[2].plot(observation_time, desired_x, "k--", label="desired x")
    axes[2].plot(observation_time, actual_x, label="actual x")
    axes[3].plot(time, np.linalg.norm(sigma, axis=1), label="||sigma||")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[0].set_xlabel("MPC time [s]")
    axes[1].set_xlabel("MPC time [s]")
    axes[2].set_xlabel("MPC time [s]")
    axes[3].set_xlabel("MPC time [s]")
    figure.suptitle("D5H1 applied estimator update: " + config["group"])
    figure.tight_layout()
    figure.savefig(figure_path, dpi=170)
    plt.close(figure)

    checks = {
        "schema": schema_ok,
        "all_finite": all_finite,
        "cross_topic_match_at_least_99_percent": match_fraction >= 0.99,
        "snapshot_time_sync": time_sync <= 1.0e-6,
        "snapshot_versions_strictly_increase": versions_ok,
        "raw_applied_derivative_euler_closures": closures_ok,
        "configured_mode_and_active_baseline": mode_contract,
        "phase_bins_valid_when_applied": phase_contract,
        "deterministic_target_and_walk": schedule_ok and observation_ok,
    }
    passed = all(checks.values())
    summary = {
        "schema_version": 1,
        "stage": "D5H1_applied_update_smoke",
        "group": config["group"],
        "passed": passed,
        "formal_eligibility": False,
        "checks": checks,
        "sample_count": len(d5),
        "cross_topic_match_fraction": match_fraction,
        "correction_active_sample_count": int(np.sum(correction_flags)),
        "correction_active_duration_s": correction_duration,
        "closures": {
            "raw_drive_max_abs_error": raw_closure,
            "applied_drive_max_abs_error": applied_closure,
            "actual_derivative_max_abs_error": derivative_closure,
            "v3_derivative_max_abs_error": v3_derivative_closure,
            "pi_hat_cross_topic_max_abs_error": pi_closure,
            "sigma_cross_topic_max_abs_error": sigma_closure,
            "explicit_euler_max_abs_error": euler_closure,
            "snapshot_time_max_abs_error_s": time_sync,
        },
        "metrics": {
            "terminal_pi_hat_scaled_norm": terminal_scaled_norm,
            "peak_pi_hat_scaled_norm": peak_scaled_norm,
            "x_rmse_m_descriptive_only": x_rmse,
        },
        "boundary": (
            "Smoke validity only. One short run cannot establish robustness, "
            "payload identifiability, tracking improvement, or stability."),
        "artifacts": {"csv": str(csv_path), "plot": str(figure_path)},
    }
    (run / "task_d_d5h1_smoke_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    report = """# D5H1 applied-update smoke — {group}

- Result: **{result}**
- Correction configured: `{configured}`
- Correction active samples/duration: `{samples}` / `{duration:.6g} s`
- Terminal/peak physical-scaled pi norm: `{terminal:.8g}` / `{peak:.8g}`
- Descriptive x RMSE: `{xrmse:.8g} m`
- Maximum explicit-Euler closure error: `{euler:.3g}`

This is a short closed-loop validity run, not a formal performance or
parameter-convergence result.
""".format(
        group=config["group"], result="PASS" if passed else "FAIL",
        configured=expected_correction, samples=int(np.sum(correction_flags)),
        duration=correction_duration, terminal=terminal_scaled_norm,
        peak=peak_scaled_norm, xrmse=x_rmse, euler=euler_closure)
    (run / "stage_d5h1_smoke_report.md").write_text(report)
    print(json.dumps(summary, indent=2, sort_keys=True))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
