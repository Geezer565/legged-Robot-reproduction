#!/usr/bin/env python3
"""Validate one 10 cm deterministic static-walk animal_dummy smoke run."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import rosbag


OBSERVATION = "/legged_robot_mpc_observation"
TARGET = "/legged_robot_mpc_target"
MODE_SCHEDULE = "/legged_robot_mpc_mode_schedule"
STATUS = "/task_c/locomotion_scheduler_status"
MARKERS = "/legged_robot/currentState"
STANCE = 15
FOOT_NAMES = ("LF", "RF", "LH", "RH")


def finite(values):
    return np.all(np.isfinite(np.asarray(values, dtype=float)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--expected-target-start", type=float, default=None)
    parser.add_argument("--require-locomotion", action="store_true")
    args = parser.parse_args()
    run = args.run_directory.resolve()
    bag_path = run / "locomotion_smoke.bag"
    if not bag_path.is_file():
        bag_path = run / "payload_mass_smoke.bag"
    if not bag_path.is_file():
        raise RuntimeError("missing locomotion bag: {}".format(bag_path))

    observations, targets, schedules, statuses, marker_rows = [], [], [], [], []
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, stamp in bag.read_messages(
                topics=[OBSERVATION, TARGET, MODE_SCHEDULE, STATUS, MARKERS]):
            bag_time = stamp.to_sec()
            if topic == OBSERVATION:
                state = np.asarray(message.state.value, dtype=float)
                control = np.asarray(message.input.value, dtype=float)
                if len(state) != 24 or not finite(state) or not finite(control):
                    raise RuntimeError("invalid observation")
                observations.append((bag_time, float(message.time), int(message.mode), state, control))
            elif topic == TARGET:
                targets.append(message)
            elif topic == MODE_SCHEDULE:
                schedules.append({"switching_times": list(message.eventTimes),
                                  "mode_sequence": list(message.modeSequence)})
            elif topic == STATUS:
                statuses.append(json.loads(message.data))
            elif len(message.markers) >= 8:
                feet = []
                for index in (0, 2, 4, 6):
                    position = message.markers[index].pose.position
                    feet.append([position.x, position.y, position.z])
                if finite(feet):
                    marker_rows.append((bag_time, np.asarray(feet, dtype=float)))

    if not observations or not targets or not schedules or not statuses or not marker_rows:
        raise RuntimeError("bag is missing observation/target/gait/status/foot markers")
    target_statuses = [value for value in statuses if value.get("stage") == "target_published"]
    if not target_statuses:
        raise RuntimeError("scheduler never reported target_published")
    status = target_statuses[-1]
    target = targets[-1]
    if len(target.timeTrajectory) != 2 or len(target.stateTrajectory) != 2:
        raise RuntimeError("expected deterministic two-knot target")
    target_time = np.asarray(target.timeTrajectory, dtype=float)
    target_x = np.asarray([value.value[6] for value in target.stateTrajectory], dtype=float)
    relative_step = float(target_x[-1] - target_x[0])
    if abs(relative_step - 0.10) > 1.0e-6 or abs((target_time[1] - target_time[0]) - 0.20) > 1.0e-6:
        raise RuntimeError("target is not the frozen 0.10 m / 0.20 s smoke target")
    if args.expected_target_start is not None and \
            abs(float(target_time[0]) - args.expected_target_start) > 1.0e-9:
        raise RuntimeError("target start time is not the fixed campaign schedule")

    observations.sort(key=lambda row: row[1])
    deduplicated = {row[1]: row for row in observations}
    observations = [deduplicated[key] for key in sorted(deduplicated)]
    bag_time = np.asarray([row[0] for row in observations])
    sim_time = np.asarray([row[1] for row in observations])
    modes = np.asarray([row[2] for row in observations], dtype=int)
    states = np.vstack([row[3] for row in observations])
    controls = np.vstack([row[4] for row in observations])
    if not np.all(np.diff(sim_time) > 0.0):
        raise RuntimeError("observation times are not strictly increasing")

    marker_rows.sort(key=lambda row: row[0])
    marker_bag_time = np.asarray([row[0] for row in marker_rows])
    marker_sim_time = np.interp(marker_bag_time, bag_time, sim_time)
    feet = np.stack([row[1] for row in marker_rows])
    non_stance = modes != STANCE
    non_stance_modes = sorted(int(value) for value in np.unique(modes[non_stance]))
    transition_count = int(np.count_nonzero(np.diff(modes)))
    first_non_stance = float(sim_time[np.flatnonzero(non_stance)[0]]) if np.any(non_stance) else None

    desired_x = np.interp(sim_time, target_time, target_x)
    post = sim_time >= max(float(target_time[-1]), float(sim_time[-1] - 2.0))
    if not np.any(post):
        raise RuntimeError("no post-target analysis samples")
    actual_x = states[:, 6]
    actual_z = states[:, 8]
    final_progress = float(np.mean(actual_x[post]) - target_x[0])
    final_error = float(np.mean(actual_x[post] - desired_x[post]))

    foot_metrics = {}
    swinging_feet = 0
    forward_moving_feet = 0
    for index, name in enumerate(FOOT_NAMES):
        x_values = feet[:, index, 0]
        z_values = feet[:, index, 2]
        x_range = float(np.ptp(x_values))
        z_range = float(np.ptp(z_values))
        if z_range > 0.02:
            swinging_feet += 1
        if x_range > 0.02:
            forward_moving_feet += 1
        foot_metrics[name] = {
            "x_range_m": x_range,
            "z_range_m": z_range,
            "first_x_m": float(x_values[0]),
            "last_x_m": float(x_values[-1]),
            "net_x_m": float(x_values[-1] - x_values[0]),
        }

    schedule_valid = any(
        value["mode_sequence"] == [13, 7, 14, 11] and
        np.allclose(value["switching_times"], [0.0, 0.3, 0.6, 0.9, 1.2], atol=1.0e-12)
        for value in schedules)
    locomotion_confirmed = bool(
        schedule_valid and len(non_stance_modes) >= 3 and transition_count >= 4 and
        swinging_feet == 4 and forward_moving_feet >= 3)
    tracking_reached_80_percent = final_progress >= 0.08
    structurally_valid = bool(
        finite(states) and finite(controls) and schedule_valid and np.any(non_stance) and
        float(np.min(actual_z)) > 0.20)

    with (run / "locomotion_smoke_observation.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["observation_time", "mode", "desired_x", "actual_x", "x_error",
                         "actual_z", "vcom_x"])
        for index in range(len(sim_time)):
            writer.writerow([sim_time[index], modes[index], desired_x[index], actual_x[index],
                             actual_x[index] - desired_x[index], actual_z[index], states[index, 0]])

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(5, 1, figsize=(11, 14), sharex=True)
    axes[0].plot(sim_time, desired_x, "k--", label="desired x")
    axes[0].plot(sim_time, actual_x, label="actual x")
    axes[0].set_ylabel("base x (m)")
    axes[0].legend()
    axes[1].plot(sim_time, actual_z)
    axes[1].set_ylabel("base z (m)")
    axes[2].step(sim_time, modes, where="post")
    axes[2].set_ylabel("mode")
    for index, name in enumerate(FOOT_NAMES):
        axes[3].plot(marker_sim_time, feet[:, index, 0], label=name)
        axes[4].plot(marker_sim_time, feet[:, index, 2], label=name)
    axes[3].set_ylabel("foot x (m)")
    axes[4].set_ylabel("foot z (m)")
    axes[4].set_xlabel("MPC / simulation time (s)")
    axes[3].legend(ncol=4)
    for axis in axes:
        axis.grid(True, alpha=0.3)
    figure.suptitle("0.10 m deterministic static-walk smoke")
    figure.tight_layout(rect=(0.02, 0.02, 1.0, 0.97))
    figure.savefig(run / "locomotion_smoke.png", dpi=180)
    plt.close(figure)

    summary = {
        "passed_structural_validation": structurally_valid,
        "locomotion_confirmed": locomotion_confirmed,
        "tracking_reached_80_percent": bool(tracking_reached_80_percent),
        "scientific_scope": "single-run static-walk validation; campaign inference requires paired repeated analysis",
        "time_range_s": [float(sim_time[0]), float(sim_time[-1])],
        "target": {"start_time_s": float(target_time[0]), "end_time_s": float(target_time[1]),
                   "start_x_m": float(target_x[0]), "final_x_m": float(target_x[-1]),
                   "relative_step_m": relative_step},
        "scheduler_status": status,
        "mode_counts": {str(int(value)): int(np.sum(modes == value)) for value in np.unique(modes)},
        "non_stance_modes": non_stance_modes,
        "first_non_stance_time_s": first_non_stance,
        "mode_transition_count": transition_count,
        "non_stance_sample_fraction": float(np.mean(non_stance)),
        "last_2s_actual_x_mean_m": float(np.mean(actual_x[post])),
        "last_2s_x_progress_m": final_progress,
        "last_2s_x_error_mean_m": final_error,
        "peak_actual_x_m": float(np.max(actual_x)),
        "base_z_min_m": float(np.min(actual_z)),
        "base_z_max_m": float(np.max(actual_z)),
        "last_2s_vcom_x_rms_m_s": float(np.sqrt(np.mean(states[post, 0] ** 2))),
        "swinging_foot_count": swinging_feet,
        "forward_moving_foot_count": forward_moving_feet,
        "foot_metrics": foot_metrics,
        "interpretation": (
            "locomotion is confirmed only when executed contact modes cycle and all feet show swing motion; "
            "tracking quality is reported separately and is not required to make a negative smoke result disappear"),
    }
    (run / "locomotion_smoke_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    if not structurally_valid:
        raise RuntimeError("locomotion smoke structural validation failed")
    if args.require_locomotion and not locomotion_confirmed:
        raise RuntimeError("executed gait/foot motion did not confirm locomotion")


if __name__ == "__main__":
    main()
