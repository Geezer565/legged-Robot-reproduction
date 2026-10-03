#!/usr/bin/env python3
"""Validate one D3 group sanity run; no cross-group performance inference."""

import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag


ESTIMATE = "/task_d/payload_inertial_10d_v3_sample"
FLOW = "/task_d/equation10_full_flow_sample"
PLANT = "/task_d/payload_inertial_plant_sample"
IDENTITY = "/task_d/model_identity"
OBSERVATION = "/legged_robot_mpc_observation"
TARGET = "/legged_robot_mpc_target"
STATUS = "/task_c/locomotion_scheduler_status"
POLICY = "/task_b/policy_input_sample"
LEGACY = ["/adaptive_update/sample", "/task_b/equation10_sample"]


def finite(values):
    return all(math.isfinite(float(value)) for value in values)


def norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def trapz(times, values):
    return sum(0.5 * (values[i] + values[i - 1]) *
               (times[i] - times[i - 1]) for i in range(1, len(times)))


def interpolate(time, times, values):
    if time <= times[0]:
        return values[0]
    if time >= times[-1]:
        return values[-1]
    for index in range(1, len(times)):
        if time <= times[index]:
            alpha = ((time - times[index - 1]) /
                     (times[index] - times[index - 1]))
            return [(1.0 - alpha) * left + alpha * right
                    for left, right in zip(values[index - 1], values[index])]
    return values[-1]


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_task_d_d3_sanity.py RUN_DIR")
    run = Path(sys.argv[1]).resolve()
    config = json.loads((run / "effective_config.json").read_text())
    group = config["group"]
    expected_update = bool(config["enableParameterUpdate"])
    expected_dynamics = bool(config["enableAdaptiveDynamics"])
    expected_payload = bool(config["truePlantPayloadInertialEnabled"])

    estimates, flows, plants, observations, targets = [], [], [], [], []
    statuses, policies, identity = [], [], None
    labels = {ESTIMATE: set(), FLOW: set(), PLANT: set()}
    legacy_counts = {topic: 0 for topic in LEGACY}
    with rosbag.Bag(str(run / "d3_sanity.bag")) as bag:
        topics = [ESTIMATE, FLOW, PLANT, IDENTITY, OBSERVATION, TARGET,
                  STATUS, POLICY] + LEGACY
        for topic, message, stamp in bag.read_messages(topics=topics):
            if topic in (ESTIMATE, FLOW, PLANT):
                labels[topic].add(message.layout.dim[0].label
                                  if message.layout.dim else "")
                row = [float(value) for value in message.data]
                {ESTIMATE: estimates, FLOW: flows, PLANT: plants}[topic].append(row)
            elif topic == IDENTITY:
                identity = json.loads(message.data)
            elif topic == OBSERVATION:
                state = [float(value) for value in message.state.value]
                control = [float(value) for value in message.input.value]
                observations.append((float(message.time), int(message.mode), state, control))
            elif topic == TARGET:
                targets.append((list(message.timeTrajectory),
                                [list(value.value) for value in message.stateTrajectory]))
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
        len(estimates) >= 5 and len(flows) >= 5
        and all(len(row) == 118 for row in estimates)
        and all(len(row) == 80 for row in flows)
        and labels[ESTIMATE] == {"payload_inertial_10d_v3_118_fields"}
        and labels[FLOW] == {"payload_inertial_eq10_full_flow_v2_80_fields"}
    )
    if expected_payload:
        schema_ok = (schema_ok and len(plants) >= 10
                     and all(len(row) == 74 for row in plants)
                     and labels[PLANT] == {"payload_inertial_plant_v2_74_fields"})
    else:
        schema_ok = schema_ok and not plants
    all_finite = (schema_ok and all(finite(row) for row in estimates + flows + plants)
                  and all(len(row[2]) == 24 and finite(row[2]) and finite(row[3])
                          for row in observations))
    versions = [int(round(row[3])) for row in estimates]
    synchronization_ok = (
        max((abs(row[2]) for row in estimates), default=math.inf) <= 1e-6
        and all(right > left for left, right in zip(versions, versions[1:]))
    )
    flags_ok = all(
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
    applied_error = max((max(abs(row[32 + index] - row[56 + index])
                                 for index in range(24)) for row in flows),
                        default=math.inf) if expected_dynamics else max(
                            (max(abs(row[56 + index]) for index in range(24))
                             for row in flows), default=math.inf)
    pi_peak = max((norm(row[22:32]) for row in estimates), default=0.0)
    candidate_peak = max((row[116] for row in estimates), default=0.0)
    applied_peak = max((row[117] for row in estimates), default=0.0)
    mode_path_ok = (
        (expected_update and pi_peak > 1e-12 or
         not expected_update and pi_peak <= 1e-12)
        and (expected_dynamics and candidate_peak > 1e-12 and applied_peak > 1e-12
             or not expected_dynamics and candidate_peak <= 1e-12
             and applied_peak <= 1e-12)
        and applied_error <= 1e-12
    )

    nominal_mass = identity.get("controller_nominal_mass") if identity else None
    identity_ok = bool(identity) and identity.get("schema_version") == 2
    if expected_payload:
        identity_ok = (identity_ok
                       and identity.get("payload_inertial_override_active") is True
                       and identity.get("plant_model_id") ==
                           "srbd_payload_inertial_10d_nominal_coordinates_v2"
                       and identity.get("plant_true_total_mass", 0.0) > nominal_mass)
        truth = plants[0][14:24] if plants else [math.nan] * 10
        truth_constant_error = max((max(abs(row[14 + index] - truth[index])
                                        for index in range(10)) for row in plants),
                                   default=math.inf)
        roundtrip_peak = max((row[24] for row in plants), default=math.inf)
        residual_peak = max((row[25] for row in plants), default=0.0)
        plant_ok = (truth_constant_error <= 1e-12 and roundtrip_peak <= 1e-10
                    and residual_peak > 1e-10)
    else:
        identity_ok = (identity_ok
                       and identity.get("payload_inertial_override_active") is False
                       and identity.get("plant_model_id") == "nominal_ocs2_rollout_v1"
                       and abs(identity.get("plant_true_total_mass", math.inf) -
                               nominal_mass) <= 1e-12)
        truth = [0.0] * 10
        truth_constant_error, roundtrip_peak, residual_peak = 0.0, 0.0, 0.0
        plant_ok = not plants

    target_status = [value for value in statuses
                     if value.get("stage") == "target_published"]
    schedule_ok = bool(target_status) and abs(
        target_status[-1].get("target_start_time", 0.0) - 1.2) <= 1e-12
    target_times, target_states = targets[-1] if targets else ([], [])
    if len(target_times) != 2 or len(target_states) != 2:
        schedule_ok = False
    times = [row[0] for row in observations]
    desired = [interpolate(time, target_times, target_states)
               for time in times] if schedule_ok else []
    actual_x = [row[2][6] for row in observations]
    desired_x = [row[6] for row in desired]
    x_error = [wanted - actual for wanted, actual in zip(desired_x, actual_x)]
    position_error = [norm(wanted[6 + index] - actual[2][6 + index]
                           for index in range(3))
                      for wanted, actual in zip(desired, observations)]
    tracking = {
        "x_rmse_m": math.sqrt(sum(value * value for value in x_error) /
                              len(x_error)) if x_error else None,
        "x_iae_m_s": trapz(times, [abs(value) for value in x_error])
                     if x_error else None,
        "peak_abs_x_error_m": max(map(abs, x_error)) if x_error else None,
        "position_rmse_m": math.sqrt(sum(value * value for value in position_error) /
                                     len(position_error)) if position_error else None,
        "peak_position_error_m": max(position_error) if position_error else None,
        "peak_sigma_norm": max((norm(row[42:48]) for row in estimates), default=None),
        "mean_sigma_norm": (sum(norm(row[42:48]) for row in estimates) /
                            len(estimates)) if estimates else None,
    }
    observation_ok = (schedule_ok and len(observations) >= 20
                      and times[-1] >= 3.0 and all(right > left
                                                  for left, right in zip(times, times[1:])))
    legacy_ok = all(value == 0 for value in legacy_counts.values())
    passed = (schema_ok and all_finite and synchronization_ok and flags_ok
              and flow_flags_ok and mode_path_ok and identity_ok and plant_ok
              and observation_ok and legacy_ok)

    csv_path = run / "d3_sanity.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time", "desired_x", "actual_x", "x_error",
                         "sigma_norm"] + ["pi_hat_{}".format(index)
                                           for index in range(10)]
                        + ["pi_star_{}".format(index) for index in range(10)]
                        + ["candidate_flow_norm", "applied_flow_norm"])
        for row in estimates:
            nearest = min(range(len(times)), key=lambda index: abs(times[index] - row[0]))
            writer.writerow([row[0], desired_x[nearest], actual_x[nearest],
                             x_error[nearest], norm(row[42:48])] + row[22:32]
                            + truth + [row[116], row[117]])

    figure_path = run / "d3_sanity.png"
    estimate_times = [row[0] for row in estimates]
    error_norm = [norm(row[22 + index] - truth[index]
                       for index in range(10)) for row in estimates]
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    axes[0].plot(times, desired_x, "k--", label="desired x")
    axes[0].plot(times, actual_x, label="actual x")
    axes[1].plot(estimate_times, [row[22] for row in estimates], label="pi_hat mass")
    axes[1].axhline(truth[0], color="k", ls="--", label="pi_star mass")
    axes[2].plot(estimate_times, error_norm, label="||pi_hat-pi_star||")
    axes[2].plot(estimate_times, [norm(row[42:48]) for row in estimates],
                 label="||sigma||")
    axes[3].plot(estimate_times, [row[116] for row in estimates], label="candidate")
    axes[3].plot(estimate_times, [row[117] for row in estimates], "--", label="applied")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[-1].set_xlabel("MPC observation time [s]")
    fig.suptitle("D3 {} sanity (not formal inference)".format(group))
    fig.tight_layout()
    fig.savefig(figure_path, dpi=160)
    plt.close(fig)

    summary = {
        "schema_version": 3,
        "group": group,
        "pass": passed,
        "scientific_scope": "single short D3 sanity; not formal performance inference",
        "expected": {"parameter_update": expected_update,
                     "adaptive_dynamics": expected_dynamics,
                     "payload_plant": expected_payload},
        "sample_counts": {"estimate": len(estimates), "flow": len(flows),
                          "plant": len(plants), "observation": len(observations),
                          "policy": len(policies)},
        "schema_ok": schema_ok,
        "all_finite": all_finite,
        "synchronization_ok": synchronization_ok,
        "mode_flags_ok": flags_ok and flow_flags_ok,
        "mode_path_ok": mode_path_ok,
        "peak_pi_hat_norm": pi_peak,
        "peak_candidate_full_flow_norm": candidate_peak,
        "peak_applied_full_flow_norm": applied_peak,
        "max_candidate_applied_path_error": applied_error,
        "model_identity_ok": identity_ok,
        "model_identity": identity,
        "plant_ok": plant_ok,
        "plant_pi_star_10d": truth,
        "plant_truth_constant_error": truth_constant_error,
        "plant_coordinate_roundtrip_peak_error": roundtrip_peak,
        "plant_full_flow_residual_peak_norm": residual_peak,
        "schedule_and_observation_ok": observation_ok,
        "legacy_6d_topics_suppressed": legacy_ok,
        "legacy_topic_counts": legacy_counts,
        "tracking_descriptive_only": tracking,
        "final_parameter_error_norm": error_norm[-1] if error_norm else None,
        "artifacts": {"csv": str(csv_path), "plot": str(figure_path)},
    }
    (run / "d3_sanity_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
