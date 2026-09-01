#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

from legged_controllers.myrbf_campaign_timing import (
    CAMPAIGN_START_TOPIC,
    campaign_elapsed,
)


AXES = ("fx", "fy", "fz", "tx", "ty", "tz")
PROFILES = ("constant", "sine", "multisine", "irregular")
IRREGULAR_LEVELS = (0.0, 1.0, -0.45, 0.75, -1.0, 0.3, -0.8, 0.55)
IRREGULAR_DURATIONS = (0.35, 0.55, 0.25, 0.65, 0.4, 0.3, 0.5, 0.45)


def validate_profile(profile, delay, duration, value):
    if profile not in PROFILES:
        raise ValueError("unknown wrench profile")
    if not math.isfinite(delay) or delay < 0.0:
        raise ValueError("delay must be finite and nonnegative")
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration must be finite and positive")
    if not math.isfinite(value):
        raise ValueError("wrench value must be finite")


def profile_is_active(elapsed, delay, duration):
    return delay <= elapsed < delay + duration


def irregular_level(active_time):
    cycle_duration = sum(IRREGULAR_DURATIONS)
    phase = active_time % cycle_duration
    boundary = 0.0
    for level, segment_duration in zip(IRREGULAR_LEVELS, IRREGULAR_DURATIONS):
        boundary += segment_duration
        if phase < boundary:
            return level
    return IRREGULAR_LEVELS[-1]


def profile_value(elapsed, profile, delay, duration, value, amplitude, frequency,
                  secondary_amplitude=0.0, secondary_frequency=1.0):
    validate_profile(profile, delay, duration, value)
    if not profile_is_active(elapsed, delay, duration):
        return 0.0
    if profile == "constant":
        return value
    if not math.isfinite(amplitude):
        raise ValueError("wrench amplitude must be finite")
    active_time = elapsed - delay
    if profile == "irregular":
        return value + amplitude * irregular_level(active_time)
    if not math.isfinite(frequency) or frequency <= 0.0:
        raise ValueError("frequency must be finite and positive")
    primary = amplitude * math.sin(2.0 * math.pi * frequency * active_time)
    if profile == "sine":
        return value + primary
    if (not math.isfinite(secondary_amplitude) or
            not math.isfinite(secondary_frequency) or secondary_frequency <= 0.0):
        raise ValueError("secondary amplitude must be finite and frequency must be positive")
    secondary = secondary_amplitude * math.sin(
        2.0 * math.pi * secondary_frequency * active_time + math.pi / 6.0)
    return value + primary + secondary


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Apply a repeatable wrench to the Go2 base in Gazebo.")
    parser.add_argument("--body", default="go2::base")
    parser.add_argument("--axis", choices=AXES, default="fy")
    parser.add_argument("--profile", choices=PROFILES, default="constant")
    parser.add_argument("--value", type=float, default=20.0)
    parser.add_argument("--amplitude", type=float, default=0.0)
    parser.add_argument("--frequency", type=float, default=0.5)
    parser.add_argument("--secondary-amplitude", type=float, default=0.0)
    parser.add_argument("--secondary-frequency", type=float, default=1.0)
    parser.add_argument("--delay", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--output", required=True)
    return parser.parse_args(argv)


def main():
    import rospy
    from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, GetLinkState
    from std_msgs.msg import Float64

    args = parse_args(rospy.myargv()[1:])
    validate_profile(args.profile, args.delay, args.duration, args.value)
    if not math.isfinite(args.rate) or args.rate <= 0.0:
        raise ValueError("rate must be finite and positive")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rospy.init_node("myrbf_wrench_injector", anonymous=True)
    rospy.wait_for_service("/gazebo/apply_body_wrench", timeout=30.0)
    rospy.wait_for_service("/gazebo/get_link_state", timeout=30.0)
    apply_wrench = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
    get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)

    while not rospy.is_shutdown():
        try:
            if get_link_state(args.body, "world").success:
                break
        except rospy.ServiceException:
            pass
        rospy.sleep(0.05)
    if rospy.is_shutdown():
        return
    rospy.loginfo("Gazebo body %s is ready; starting wrench schedule.", args.body)

    campaign_start_ros = rospy.wait_for_message(
        CAMPAIGN_START_TOPIC, Float64, timeout=60.0
    ).data
    end = args.delay + args.duration + 1.0
    period = 1.0 / args.rate
    rate = rospy.Rate(args.rate)
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("ros_time", "campaign_start_ros", "wall_elapsed", *AXES, "service_called", "service_success"))
        while not rospy.is_shutdown():
            ros_now = rospy.get_time()
            elapsed = campaign_elapsed(ros_now, campaign_start_ros)
            if elapsed >= end:
                break

            scalar = profile_value(
                elapsed, args.profile, args.delay, args.duration,
                args.value, args.amplitude, args.frequency,
                args.secondary_amplitude, args.secondary_frequency)
            wrench_values = [0.0] * 6
            wrench_values[AXES.index(args.axis)] = scalar
            active = profile_is_active(elapsed, args.delay, args.duration)
            success = True
            if active:
                request = ApplyBodyWrenchRequest()
                request.body_name = args.body
                request.reference_frame = "world"
                request.reference_point.x = 0.0
                request.reference_point.y = 0.0
                request.reference_point.z = 0.0
                request.wrench.force.x, request.wrench.force.y, request.wrench.force.z = wrench_values[:3]
                request.wrench.torque.x, request.wrench.torque.y, request.wrench.torque.z = wrench_values[3:]
                request.start_time = rospy.Time(0)
                request.duration = rospy.Duration.from_sec(1.5 * period)
                try:
                    success = bool(apply_wrench(request).success)
                except rospy.ServiceException as error:
                    rospy.logerr("Gazebo wrench call failed: %s", error)
                    success = False
            writer.writerow((
                ros_now, campaign_start_ros, elapsed,
                *wrench_values, int(active), int(success)))
            handle.flush()
            rate.sleep()

    rospy.loginfo("Finished MyRBF %s wrench profile; truth written to %s", args.profile, output)


if __name__ == "__main__":
    main()
