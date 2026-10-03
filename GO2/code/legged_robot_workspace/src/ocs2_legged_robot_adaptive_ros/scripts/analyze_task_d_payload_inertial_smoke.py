#!/usr/bin/env python3
"""Structural D2 smoke audit; not a scientific convergence analysis."""

import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag


ESTIMATE_TOPIC = "/task_d/payload_inertial_10d_sample"
FLOW_TOPIC = "/task_d/equation10_full_flow_sample"
PLANT_TOPIC = "/task_d/payload_inertial_plant_sample"
IDENTITY_TOPIC = "/task_d/model_identity"
LEGACY_TOPICS = ["/adaptive_update/sample", "/task_b/equation10_sample"]


def finite(row):
    return all(math.isfinite(float(value)) for value in row)


def norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def max_abs(values):
    return max((abs(float(value)) for value in values), default=0.0)


def read_bag(path):
    result = {ESTIMATE_TOPIC: [], FLOW_TOPIC: [], PLANT_TOPIC: []}
    identity = None
    legacy_counts = {topic: 0 for topic in LEGACY_TOPICS}
    scheduler = []
    observations = []
    with rosbag.Bag(str(path)) as bag:
        topics = list(result) + [IDENTITY_TOPIC, "/task_c/locomotion_scheduler_status",
                                 "/legged_robot_mpc_observation"] + LEGACY_TOPICS
        for topic, message, _ in bag.read_messages(topics=topics):
            if topic in result:
                label = message.layout.dim[0].label if message.layout.dim else ""
                result[topic].append((label, [float(value) for value in message.data]))
            elif topic == IDENTITY_TOPIC:
                identity = json.loads(message.data)
            elif topic == "/task_c/locomotion_scheduler_status":
                scheduler.append(json.loads(message.data))
            elif topic == "/legged_robot_mpc_observation":
                observations.append(float(message.time))
            elif topic in legacy_counts:
                legacy_counts[topic] += 1
    return result, identity, scheduler, observations, legacy_counts


def nearest_plant(plants, time):
    return min(plants, key=lambda row: abs(row[0] - time))


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_task_d_payload_inertial_smoke.py RUN_DIR")
    run_dir = Path(sys.argv[1]).resolve()
    bag_path = run_dir / "payload_inertial_smoke.bag"
    data, identity, scheduler, observation_times, legacy_counts = read_bag(bag_path)
    estimates = sorted((row for _, row in data[ESTIMATE_TOPIC]), key=lambda row: row[0])
    flows = sorted((row for _, row in data[FLOW_TOPIC]), key=lambda row: row[0])
    plants = sorted((row for _, row in data[PLANT_TOPIC]), key=lambda row: row[0])
    labels = {
        topic: sorted(set(label for label, _ in values))
        for topic, values in data.items()
    }

    shape_ok = (
        bool(estimates) and bool(flows) and bool(plants)
        and all(len(row) == 112 for row in estimates)
        and all(len(row) == 80 for row in flows)
        and all(len(row) == 74 for row in plants)
        and labels[ESTIMATE_TOPIC] == ["payload_inertial_10d_v2_112_fields"]
        and labels[FLOW_TOPIC] == ["payload_inertial_eq10_full_flow_v2_80_fields"]
        and labels[PLANT_TOPIC] == ["payload_inertial_plant_v2_74_fields"]
    )
    all_finite = shape_ok and all(finite(row) for row in estimates + flows + plants)
    max_time_error = max_abs(row[2] for row in estimates) if estimates else math.inf
    versions = [int(round(row[3])) for row in estimates]
    version_ok = bool(versions) and all(b > a for a, b in zip(versions, versions[1:]))
    flags_ok = bool(estimates) and all(
        row[4] == 1.0 and row[5] == 1.0 and row[6] == 1.0 and row[7] == 10.0
        and row[8] == 1.0 and row[9] == 1.0 and row[10] == 0.0
        for row in estimates
    )
    pi0 = estimates[0][22:32] if estimates else [0.0] * 10
    peak_pi_change = max((norm(row[22 + i] - pi0[i] for i in range(10))
                          for row in estimates), default=0.0)
    peak_candidate = max((row[110] for row in estimates), default=0.0)
    peak_applied = max((row[111] for row in estimates), default=0.0)
    flow_path_error = max((max_abs(
        row[32 + i] - row[56 + i] for i in range(24)) for row in flows),
        default=math.inf)
    flow_flags_ok = bool(flows) and all(
        row[4] == 1.0 and row[5] == 1.0 and row[6] == 1.0 and row[7] == 1.0
        for row in flows
    )

    plant_roundtrip_peak = max((row[24] for row in plants), default=math.inf)
    plant_residual_peak = max((row[25] for row in plants), default=0.0)
    plant_truth = plants[0][14:24] if plants else [math.nan] * 10
    plant_truth_constant_error = max((max_abs(
        row[14 + i] - plant_truth[i] for i in range(10)) for row in plants),
        default=math.inf)
    identity_ok = bool(identity) and (
        identity.get("schema_version") == 2
        and identity.get("controller_model_id") == "nominal_urdf_srbd_v1"
        and identity.get("plant_model_id") ==
            "srbd_payload_inertial_10d_nominal_coordinates_v2"
        and identity.get("payload_inertial_override_active") is True
        and identity.get("plant_true_total_mass", 0.0) >
            identity.get("controller_nominal_mass", math.inf)
    )
    target_events = [item for item in scheduler if item.get("stage") == "target_published"]
    schedule_ok = bool(target_events) and all(
        abs(item.get("relative_x_step", 0.0) - 0.1) <= 1e-12
        and abs(item.get("target_start_time", 0.0) - 1.2) <= 1e-12
        for item in target_events
    )
    legacy_suppressed = all(count == 0 for count in legacy_counts.values())

    structural_pass = (
        shape_ok and all_finite and len(estimates) >= 5 and len(flows) >= 5
        and len(plants) >= 10 and max_time_error <= 1e-6 and version_ok
        and flags_ok and flow_flags_ok and peak_pi_change > 1e-12
        and peak_candidate > 1e-12 and peak_applied > 1e-12
        and flow_path_error <= 1e-12 and plant_roundtrip_peak <= 1e-10
        and plant_residual_peak > 1e-10 and plant_truth_constant_error <= 1e-12
        and identity_ok and schedule_ok and legacy_suppressed
        and observation_times and max(observation_times) >= 2.0
    )

    csv_path = run_dir / "payload_inertial_smoke.csv"
    header = ["observation_time", "snapshot_time", "snapshot_version"]
    header += [f"pi_hat_{name}" for name in
               ["m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy", "Jyz", "Jzz"]]
    header += [f"pi_star_{name}" for name in
               ["m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy", "Jyz", "Jzz"]]
    header += ["pi_dot_norm", "candidate_flow_norm", "applied_flow_norm",
               "plant_flow_residual_norm", "plant_roundtrip_error"]
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for row in estimates:
            plant = nearest_plant(plants, row[0]) if plants else [math.nan] * 74
            writer.writerow([row[0], row[1], int(round(row[3]))] + row[22:32]
                            + plant[14:24] + [row[109], row[110], row[111],
                                              plant[25], plant[24]])

    figure_path = run_dir / "payload_inertial_smoke.png"
    times = [row[0] for row in estimates]
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    axes[0].plot(times, [row[22] for row in estimates], label="pi_hat m")
    axes[0].axhline(plant_truth[0], color="k", ls="--", label="pi_star m")
    for i, name in enumerate(["hx", "hy", "hz"], start=1):
        axes[1].plot(times, [row[22 + i] for row in estimates], label=f"hat {name}")
        axes[1].axhline(plant_truth[i], ls="--", label=f"star {name}")
    for i, name in enumerate(["Jxx", "Jxy", "Jxz", "Jyy", "Jyz", "Jzz"], start=4):
        axes[2].plot(times, [row[22 + i] for row in estimates], label=f"hat {name}")
        axes[2].axhline(plant_truth[i], ls="--", alpha=0.7)
    axes[3].plot(times, [row[110] for row in estimates], label="candidate ||delta f||")
    axes[3].plot(times, [row[111] for row in estimates], ls="--", label="applied ||delta f||")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend(ncol=3, fontsize=8)
    axes[-1].set_xlabel("MPC observation time [s]")
    fig.suptitle("Task D2 10D structural smoke (not convergence evidence)")
    fig.tight_layout()
    fig.savefig(figure_path, dpi=160)
    plt.close(fig)

    summary = {
        "schema_version": 2,
        "scope": "D2 structural smoke only; not a formal convergence result",
        "pass": structural_pass,
        "sample_counts": {"estimate": len(estimates), "flow": len(flows),
                          "plant": len(plants)},
        "schema_labels": labels,
        "all_samples_finite": all_finite,
        "max_snapshot_time_error_s": max_time_error,
        "strictly_increasing_estimate_versions": version_ok,
        "all_runtime_flags_valid": flags_ok and flow_flags_ok,
        "peak_pi_hat_change_norm": peak_pi_change,
        "peak_candidate_full_flow_correction_norm": peak_candidate,
        "peak_applied_full_flow_correction_norm": peak_applied,
        "max_candidate_applied_component_error": flow_path_error,
        "plant_coordinate_roundtrip_peak_error": plant_roundtrip_peak,
        "plant_full_flow_residual_peak_norm": plant_residual_peak,
        "plant_truth_constant_max_error": plant_truth_constant_error,
        "plant_pi_star_10d": plant_truth,
        "model_identity": identity,
        "model_identity_ok": identity_ok,
        "deterministic_target_schedule_ok": schedule_ok,
        "legacy_6d_topics_suppressed": legacy_suppressed,
        "legacy_topic_counts": legacy_counts,
        "observation_time_max": max(observation_times) if observation_times else None,
        "artifacts": {"csv": str(csv_path), "plot": str(figure_path)},
    }
    (run_dir / "payload_inertial_smoke_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    raise SystemExit(0 if structural_pass else 1)


if __name__ == "__main__":
    main()
