#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path
import time

from legged_controllers.myrbf_campaign_timing import (
    CAMPAIGN_START_TOPIC,
    campaign_elapsed,
)


AXES = ("fx", "fy", "fz", "tx", "ty", "tz")
PROFILES = ("none", "constant", "sine", "multisine", "irregular", "sixd")
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
                  secondary_amplitude=0.0, secondary_frequency=1.0,
                  phase_deg=0.0, secondary_phase_deg=30.0):
    validate_profile(profile, delay, duration, value)
    if profile == "none":
        return 0.0
    if not profile_is_active(elapsed, delay, duration):
        return 0.0
    if profile == "constant":
        return value
    if not math.isfinite(amplitude):
        raise ValueError("wrench amplitude must be finite")
    if not math.isfinite(phase_deg) or not math.isfinite(secondary_phase_deg):
        raise ValueError("wrench phases must be finite")
    active_time = elapsed - delay
    if profile == "irregular":
        return value + amplitude * irregular_level(active_time)
    if profile == "sixd":
        raise ValueError("sixd profile must be evaluated with wrench_values")
    if not math.isfinite(frequency) or frequency <= 0.0:
        raise ValueError("frequency must be finite and positive")
    primary = amplitude * math.sin(
        2.0 * math.pi * frequency * active_time + math.radians(phase_deg))
    if profile == "sine":
        return value + primary
    if (not math.isfinite(secondary_amplitude) or
            not math.isfinite(secondary_frequency) or secondary_frequency <= 0.0):
        raise ValueError("secondary amplitude must be finite and frequency must be positive")
    secondary = secondary_amplitude * math.sin(
        2.0 * math.pi * secondary_frequency * active_time
        + math.radians(secondary_phase_deg))
    return value + primary + secondary


def coupled_sixd(active_time, scale=1.0):
    if not math.isfinite(scale) or scale < 0.0:
        raise ValueError("six-dimensional wrench scale must be finite and nonnegative")
    return [
        scale * 10.0 * math.sin(2.0 * math.pi * 0.20 * active_time),
        scale * (15.0 * math.sin(2.0 * math.pi * 0.35 * active_time + math.pi / 4.0)
                 + 5.0 * math.sin(2.0 * math.pi * 0.90 * active_time)),
        scale * 8.0 * math.sin(2.0 * math.pi * 0.17 * active_time + math.pi / 2.0),
        scale * 1.0 * math.sin(2.0 * math.pi * 0.30 * active_time),
        scale * 0.8 * math.sin(2.0 * math.pi * 0.43 * active_time + math.pi / 3.0),
        scale * 0.6 * math.sin(2.0 * math.pi * 0.57 * active_time + math.pi / 5.0),
    ]


def wrench_values(elapsed, profile, axis, delay, duration, value, amplitude, frequency,
                  secondary_amplitude=0.0, secondary_frequency=1.0,
                  phase_deg=0.0, secondary_phase_deg=30.0):
    validate_profile(profile, delay, duration, value)
    if profile == "none":
        return [0.0] * 6
    if not profile_is_active(elapsed, delay, duration):
        return [0.0] * 6
    if profile == "sixd":
        return coupled_sixd(elapsed - delay, amplitude)
    scalar = profile_value(
        elapsed, profile, delay, duration, value, amplitude, frequency,
        secondary_amplitude, secondary_frequency, phase_deg, secondary_phase_deg)
    values = [0.0] * 6
    values[AXES.index(axis)] = scalar
    return values


def world_reference_point():
    # Gazebo Classic's world branch adds point x force to the requested torque
    # without subtracting the body's position, then calls SetForce/SetTorque.
    # Zero requests the stated net world wrench about the link CoM; passing
    # WorldPose().Pos() would add a position-dependent, unrequested moment.
    return (0.0, 0.0, 0.0)


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
    parser.add_argument("--phase-deg", type=float, default=0.0)
    parser.add_argument("--secondary-phase-deg", type=float, default=30.0)
    parser.add_argument("--delay", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--hold-duration", type=float, default=0.25)
    parser.add_argument("--output", required=True)
    return parser.parse_args(argv)


def main():
    import rospy
    from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, BodyRequest, GetLinkState
    from std_msgs.msg import Float64

    args = parse_args(rospy.myargv()[1:])
    validate_profile(args.profile, args.delay, args.duration, args.value)
    if not math.isfinite(args.rate) or args.rate <= 0.0:
        raise ValueError("rate must be finite and positive")
    if not math.isfinite(args.hold_duration) or args.hold_duration <= 0.0:
        raise ValueError("hold duration must be finite and positive")
    if not math.isfinite(args.phase_deg) or not math.isfinite(args.secondary_phase_deg):
        raise ValueError("wrench phases must be finite")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rospy.init_node("myrbf_wrench_injector", anonymous=True)
    rospy.wait_for_service("/gazebo/apply_body_wrench", timeout=180.0)
    rospy.wait_for_service("/gazebo/get_link_state", timeout=180.0)
    rospy.wait_for_service("/gazebo/clear_body_wrenches", timeout=180.0)
    apply_wrench = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
    clear_wrenches = rospy.ServiceProxy("/gazebo/clear_body_wrenches", BodyRequest)
    get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)

    while not rospy.is_shutdown():
        try:
            if get_link_state(args.body, "world").success:
                break
        except rospy.ServiceException:
            pass
        time.sleep(0.05)
    if rospy.is_shutdown():
        return
    rospy.loginfo("Gazebo body %s is ready; starting wrench schedule.", args.body)

    campaign_start_ros = rospy.wait_for_message(
        CAMPAIGN_START_TOPIC, Float64, timeout=240.0
    ).data
    last_reference_point = world_reference_point()
    end = args.delay + args.duration + 1.0
    period = 1.0 / args.rate
    rate = rospy.Rate(args.rate)
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow((
            "ros_time", "campaign_start_ros", "wall_elapsed", *AXES,
            "reference_point_x", "reference_point_y", "reference_point_z",
            "service_called", "clear_success", "service_success",
        ))
        was_active = False
        while not rospy.is_shutdown():
            ros_now = rospy.get_time()
            elapsed = campaign_elapsed(ros_now, campaign_start_ros)
            if elapsed >= end:
                break

            values = wrench_values(
                elapsed, args.profile, args.axis, args.delay, args.duration,
                args.value, args.amplitude, args.frequency,
                args.secondary_amplitude, args.secondary_frequency,
                args.phase_deg, args.secondary_phase_deg)
            active = args.profile != "none" and profile_is_active(
                elapsed, args.delay, args.duration)
            success = True
            clear_success = True
            reference_point = last_reference_point
            if active:
                try:
                    clear_wrenches(args.body)
                except rospy.ServiceException as error:
                    rospy.logerr("Gazebo wrench clear failed: %s", error)
                    clear_success = False
                try:
                    reference_point = world_reference_point()
                    last_reference_point = reference_point
                    request = ApplyBodyWrenchRequest()
                    request.body_name = args.body
                    request.reference_frame = "world"
                    (request.reference_point.x, request.reference_point.y,
                     request.reference_point.z) = reference_point
                    (request.wrench.force.x, request.wrench.force.y,
                     request.wrench.force.z) = values[:3]
                    (request.wrench.torque.x, request.wrench.torque.y,
                     request.wrench.torque.z) = values[3:]
                    request.start_time = rospy.Time(0)
                    request.duration = rospy.Duration.from_sec(
                        args.hold_duration)
                    success = clear_success and bool(
                        apply_wrench(request).success)
                except rospy.ServiceException as error:
                    rospy.logerr("Gazebo wrench call failed: %s", error)
                    success = False
                except ValueError as error:
                    rospy.logerr("Gazebo body-origin lookup failed: %s", error)
                    success = False
            elif was_active:
                try:
                    clear_wrenches(args.body)
                except rospy.ServiceException as error:
                    rospy.logerr("Gazebo final wrench clear failed: %s", error)
                    clear_success = False
            writer.writerow((
                ros_now, campaign_start_ros, elapsed, *values,
                *reference_point, int(active), int(clear_success),
                int(success)))
            handle.flush()
            was_active = active
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    try:
        clear_wrenches(args.body)
    except rospy.ServiceException as error:
        rospy.logerr("Gazebo shutdown wrench clear failed: %s", error)
    rospy.loginfo("Finished MyRBF %s wrench profile; truth written to %s", args.profile, output)


if __name__ == "__main__":
    main()
