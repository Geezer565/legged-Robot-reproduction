#!/usr/bin/env python3

import argparse
import csv
import math
import os
import threading

import rospy
from std_msgs.msg import Float64, String

from myrbf_campaign_timing import CAMPAIGN_START_TOPIC, campaign_elapsed


LINK_NAMES = ("base", "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT")


class Stage8cRecorder:
    def __init__(self):
        from gazebo_msgs.msg import LinkStates
        from ocs2_msgs.msg import mode_schedule, mpc_observation

        self._lock = threading.Lock()
        self._link_states = None
        self._observation = None
        self._schedule = None
        self._event = "startup"

        rospy.Subscriber("/gazebo/link_states", LinkStates, self._link_states_callback, queue_size=1)
        rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation, self._observation_callback, queue_size=1)
        rospy.Subscriber("/legged_robot_mpc_mode_schedule", mode_schedule, self._schedule_callback, queue_size=1)
        rospy.Subscriber("/aclf_stage8c_event", String, self._event_callback, queue_size=10)

    def _link_states_callback(self, msg):
        with self._lock:
            self._link_states = msg

    def _observation_callback(self, msg):
        with self._lock:
            self._observation = msg

    def _schedule_callback(self, msg):
        with self._lock:
            self._schedule = msg

    def _event_callback(self, msg):
        with self._lock:
            self._event = msg.data

    @staticmethod
    def _link_index(names, link_name):
        suffix = "::" + link_name
        for index, name in enumerate(names):
            if name == link_name or name.endswith(suffix):
                return index
        return None

    def row(self, campaign_start_ros):
        with self._lock:
            links = self._link_states
            observation = self._observation
            schedule = self._schedule
            event = self._event

        ros_now = rospy.get_time()
        values = [
            ros_now,
            campaign_start_ros,
            campaign_elapsed(ros_now, campaign_start_ros),
            event,
        ]
        if observation is None:
            values.extend([math.nan] * 8)
        else:
            state = observation.state.value
            base_state = list(state[6:12]) if len(state) >= 12 else [math.nan] * 6
            values.extend([observation.time, observation.mode] + base_state)

        if schedule is None:
            values.append("")
        else:
            values.append(";".join(str(mode) for mode in schedule.modeSequence))

        for link_name in LINK_NAMES:
            index = None if links is None else self._link_index(links.name, link_name)
            if index is None:
                values.extend([math.nan] * 6)
                continue
            pose = links.pose[index]
            twist = links.twist[index]
            values.extend(
                [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    twist.linear.x,
                    twist.linear.y,
                    twist.linear.z,
                ]
            )
        return values


def header():
    columns = [
        "ros_time",
        "campaign_start_ros",
        "wall_elapsed",
        "event",
        "observation_time",
        "observation_mode",
        "observation_base_x",
        "observation_base_y",
        "observation_base_z",
        "observation_base_yaw",
        "observation_base_pitch",
        "observation_base_roll",
        "scheduled_modes",
    ]
    for link_name in LINK_NAMES:
        columns.extend(
            [
                link_name + "_x",
                link_name + "_y",
                link_name + "_z",
                link_name + "_vx",
                link_name + "_vy",
                link_name + "_vz",
            ]
        )
    return columns


def parse_args():
    parser = argparse.ArgumentParser(description="Record Stage 8C COM-motion and contact-switch data.")
    parser.add_argument("--output", required=True, help="Output CSV path.")
    parser.add_argument("--duration", type=float, default=0.0, help="Wall-clock duration in seconds; zero runs until shutdown.")
    parser.add_argument("--rate", type=float, default=50.0, help="CSV sampling rate in Hz.")
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    if args.rate <= 0.0:
        raise ValueError("--rate must be positive")
    if args.duration < 0.0:
        raise ValueError("--duration must be non-negative")

    output_dir = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(output_dir, exist_ok=True)

    rospy.init_node("aclf_stage8c_recorder", anonymous=True)
    recorder = Stage8cRecorder()
    campaign_start_ros = rospy.wait_for_message(
        CAMPAIGN_START_TOPIC, Float64, timeout=60.0
    ).data
    rate = rospy.Rate(args.rate)

    with open(args.output, "w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(header())
        while not rospy.is_shutdown():
            elapsed = campaign_elapsed(rospy.get_time(), campaign_start_ros)
            if args.duration > 0.0 and elapsed >= args.duration:
                break
            writer.writerow(recorder.row(campaign_start_ros))
            if elapsed < 1.0 / args.rate or int(elapsed * args.rate) % int(max(args.rate, 1.0)) == 0:
                output.flush()
            rate.sleep()


if __name__ == "__main__":
    main()
