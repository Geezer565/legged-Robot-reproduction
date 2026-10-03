#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path
import threading

from legged_controllers.myrbf_campaign_timing import (
    CAMPAIGN_START_TOPIC,
    campaign_elapsed,
)


FIELDS = (
    "message_index",
    "knot_index",
    "received_ros_time",
    "trajectory_time_s",
    "vx_mps",
    "vy_mps",
    "vz_mps",
    "normalized_angular_momentum_x",
    "normalized_angular_momentum_y",
    "normalized_angular_momentum_z",
    "x_m",
    "y_m",
    "z_m",
    "yaw_rad",
    "pitch_rad",
    "roll_rad",
    "state_size",
    "input_size",
)


def target_rows(message, received_ros_time, message_index):
    rows = []
    knot_count = min(
        len(message.timeTrajectory),
        len(message.stateTrajectory),
        len(message.inputTrajectory),
    )
    for index in range(knot_count):
        state = list(message.stateTrajectory[index].value)
        input_values = list(message.inputTrajectory[index].value)

        def state_value(position):
            return state[position] if position < len(state) else math.nan

        rows.append({
            "message_index": message_index,
            "knot_index": index,
            "received_ros_time": received_ros_time,
            "trajectory_time_s": message.timeTrajectory[index],
            "vx_mps": state_value(0),
            "vy_mps": state_value(1),
            "vz_mps": state_value(2),
            "normalized_angular_momentum_x": state_value(3),
            "normalized_angular_momentum_y": state_value(4),
            "normalized_angular_momentum_z": state_value(5),
            "x_m": state_value(6),
            "y_m": state_value(7),
            "z_m": state_value(8),
            "yaw_rad": state_value(9),
            "pitch_rad": state_value(10),
            "roll_rad": state_value(11),
            "state_size": len(state),
            "input_size": len(input_values),
        })
    return rows


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Record TP01 planner output and MPC target input.")
    parser.add_argument("--planner-output", required=True)
    parser.add_argument("--mpc-output", required=True)
    parser.add_argument("--duration", type=float, default=17.0)
    return parser.parse_args(argv)


def main():
    import rospy
    from ocs2_msgs.msg import mpc_target_trajectories
    from std_msgs.msg import Float64

    args = parse_args(rospy.myargv()[1:])
    if args.duration <= 0.0:
        raise ValueError("duration must be positive")
    rospy.init_node("tp01_target_recorder")

    handles = {}
    writers = {}
    counters = {"planner": 0, "mpc": 0}
    lock = threading.Lock()
    for name, path in (
            ("planner", args.planner_output),
            ("mpc", args.mpc_output)):
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        handle = output.open("w", encoding="utf-8", newline="")
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        handle.flush()
        handles[name] = handle
        writers[name] = writer

    def callback(name):
        def record(message):
            with lock:
                if handles[name].closed:
                    return
                index = counters[name]
                counters[name] += 1
                rows = target_rows(message, rospy.get_time(), index)
                writers[name].writerows(rows)
                handles[name].flush()
        return record

    rospy.Subscriber(
        "/tp01/planned_reference",
        mpc_target_trajectories,
        callback("planner"),
        queue_size=10,
    )
    rospy.Subscriber(
        "/legged_robot_mpc_target",
        mpc_target_trajectories,
        callback("mpc"),
        queue_size=10,
    )
    campaign_start_ros = rospy.wait_for_message(
        CAMPAIGN_START_TOPIC, Float64, timeout=240.0).data
    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            if campaign_elapsed(
                    rospy.get_time(), campaign_start_ros) >= args.duration:
                break
            rate.sleep()
    finally:
        with lock:
            for handle in handles.values():
                handle.close()
    rospy.loginfo(
        "TP01 target records: planner=%d messages, mpc=%d messages",
        counters["planner"],
        counters["mpc"],
    )


if __name__ == "__main__":
    main()
