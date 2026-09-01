#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

from legged_controllers.myrbf_campaign_timing import (
    CAMPAIGN_START_TOPIC,
    campaign_elapsed,
)


GAIT_TEMPLATES = {
    "stance": ([0.0, 0.5], [15]),
    "trot": ([0.0, 0.3, 0.6], [9, 6]),
    "standing_trot": ([0.0, 0.25, 0.3, 0.55, 0.6], [9, 15, 6, 15]),
    "static_walk": ([0.0, 0.3, 0.6, 0.9, 1.2], [13, 7, 14, 11]),
}


def gait_template(name):
    if name not in GAIT_TEMPLATES:
        raise ValueError("unknown gait")
    event_times, mode_sequence = GAIT_TEMPLATES[name]
    return list(event_times), list(mode_sequence)


def motion_is_active(elapsed, delay, duration):
    return delay <= elapsed < delay + duration


def validate_motion(delay, duration, rate, vx, vy, yaw_rate):
    values = (delay, duration, rate, vx, vy, yaw_rate)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("motion arguments must be finite")
    if delay < 0.0:
        raise ValueError("motion delay must be nonnegative")
    if duration <= 0.0:
        raise ValueError("motion duration must be positive")
    if rate <= 0.0:
        raise ValueError("motion rate must be positive")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Publish a repeatable Go2 gait and velocity command.")
    parser.add_argument("--gait", choices=tuple(GAIT_TEMPLATES), default="static_walk")
    parser.add_argument("--vx", type=float, default=0.08)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--delay", type=float, default=4.0)
    parser.add_argument("--duration", type=float, default=7.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--output", required=True)
    return parser.parse_args(argv)


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from ocs2_msgs.msg import mode_schedule, mpc_observation
    from std_msgs.msg import Float64

    args = parse_args(rospy.myargv()[1:])
    validate_motion(args.delay, args.duration, args.rate, args.vx, args.vy, args.yaw_rate)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    rospy.init_node("myrbf_motion_publisher", anonymous=True)
    rospy.wait_for_message("/legged_robot_mpc_observation", mpc_observation, timeout=60.0)
    campaign_start_ros = rospy.wait_for_message(
        CAMPAIGN_START_TOPIC, Float64, timeout=60.0
    ).data
    gait_publisher = rospy.Publisher(
        "/legged_robot_mpc_mode_schedule", mode_schedule, queue_size=1, latch=True)
    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.sleep(0.25)

    def publish_gait(name):
        event_times, mode_sequence = gait_template(name)
        message = mode_schedule()
        message.eventTimes = event_times
        message.modeSequence = mode_sequence
        gait_publisher.publish(message)

    zero = Twist()
    command = Twist()
    command.linear.x = args.vx
    command.linear.y = args.vy
    command.angular.z = args.yaw_rate
    publish_gait("stance")

    end = args.delay + args.duration + 1.0
    rate = rospy.Rate(args.rate)
    gait_started = False
    gait_stopped = False
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("ros_time", "campaign_start_ros", "wall_elapsed", "active", "gait", "vx", "vy", "yaw_rate"))
        while not rospy.is_shutdown():
            ros_now = rospy.get_time()
            elapsed = campaign_elapsed(ros_now, campaign_start_ros)
            if elapsed >= end:
                break

            active = motion_is_active(elapsed, args.delay, args.duration)
            if active and not gait_started:
                publish_gait(args.gait)
                gait_started = True
                rospy.loginfo("Starting %s command at vx=%.3f m/s", args.gait, args.vx)
            if gait_started and not active and not gait_stopped:
                publish_gait("stance")
                gait_stopped = True
                rospy.loginfo("Stopping motion and returning to stance")
            velocity_publisher.publish(command if active else zero)
            writer.writerow((
                ros_now, campaign_start_ros, elapsed, int(active), args.gait if active else "stance",
                args.vx if active else 0.0, args.vy if active else 0.0,
                args.yaw_rate if active else 0.0))
            handle.flush()
            rate.sleep()

    velocity_publisher.publish(zero)
    publish_gait("stance")
    rospy.loginfo("Finished motion profile; commands written to %s", output)


if __name__ == "__main__":
    main()
