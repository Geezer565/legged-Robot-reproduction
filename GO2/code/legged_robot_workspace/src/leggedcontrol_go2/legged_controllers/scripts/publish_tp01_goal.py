#!/usr/bin/env python3

import argparse
import csv
import json
import math
from pathlib import Path
import threading

from legged_controllers.myrbf_campaign_timing import (
    CAMPAIGN_START_TOPIC,
    campaign_elapsed,
)
from legged_controllers.tp01_trajectory import (
    append_terminal_hold,
    build_cruise_reference,
    build_single_bend_reference,
    build_target_payload,
    make_relative_goal,
    planner_rows,
)


PLANNER_FIELDS = (
    "trajectory_time_s",
    "campaign_elapsed_s",
    "x_m",
    "y_m",
    "z_m",
    "yaw_rad",
    "pitch_rad",
    "roll_rad",
    "vx_mps",
    "vy_mps",
    "vz_mps",
    "yaw_rate_radps",
    "pitch_rate_radps",
    "roll_rate_radps",
    "ax_mps2",
    "ay_mps2",
    "az_mps2",
    "yaw_accel_radps2",
    "pitch_accel_radps2",
    "roll_accel_radps2",
)

GO2_DEFAULT_JOINT_STATE = (
    -0.00, 0.72, -1.44,
    -0.00, 0.72, -1.44,
    0.25, 0.72, -1.44,
    0.25, 0.72, -1.44,
)


def build_target_message(payload, target_type, state_type, input_type):
    message = target_type()
    message.timeTrajectory = list(payload["times"])
    for values in payload["states"]:
        item = state_type()
        item.value = list(values)
        message.stateTrajectory.append(item)
    for values in payload["inputs"]:
        item = input_type()
        item.value = list(values)
        message.inputTrajectory.append(item)
    return message


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Publish a TP01 goal and its smooth cruise MPC reference.")
    parser.add_argument("--goal-output", required=True)
    parser.add_argument("--planner-output", required=True)
    parser.add_argument("--motion-output", required=True)
    parser.add_argument("--events-output", required=True)
    parser.add_argument("--goal-dx", type=float, default=0.6)
    parser.add_argument("--goal-dy", type=float, default=0.0)
    parser.add_argument("--goal-dyaw", type=float, default=0.0)
    parser.add_argument("--com-height", type=float, default=0.33)
    parser.add_argument("--goal-delay", type=float, default=3.0)
    parser.add_argument("--trajectory-duration", type=float, default=20.0)
    parser.add_argument("--accel-duration", type=float, default=3.0)
    parser.add_argument("--trajectory-id", choices=("t01", "t02"), default="t01")
    parser.add_argument("--forward-length", type=float, default=0.6)
    parser.add_argument("--lateral-amplitude", type=float, default=0.15)
    parser.add_argument("--terminal-hold", type=float, default=5.0)
    parser.add_argument("--total-duration", type=float, default=26.0)
    parser.add_argument("--rate", type=float, default=20.0)
    return parser.parse_args(argv)


def _validate(args):
    values = (
        args.goal_dx,
        args.goal_dy,
        args.goal_dyaw,
        args.com_height,
        args.goal_delay,
        args.trajectory_duration,
        args.accel_duration,
        args.forward_length,
        args.lateral_amplitude,
        args.terminal_hold,
        args.total_duration,
        args.rate,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("all TP01 settings must be finite")
    if args.goal_delay < 0.0:
        raise ValueError("goal delay must be nonnegative")
    if args.trajectory_duration <= 0.0:
        raise ValueError("trajectory duration must be positive")
    if args.accel_duration <= 0.0 or 2.0 * args.accel_duration >= args.trajectory_duration:
        raise ValueError("acceleration duration must fit twice inside trajectory duration")
    if args.forward_length <= 0.0 or args.lateral_amplitude < 0.0:
        raise ValueError("forward length must be positive and lateral amplitude nonnegative")
    if args.terminal_hold < 0.0:
        raise ValueError("terminal hold must be nonnegative")
    if args.rate <= 0.0:
        raise ValueError("rate must be positive")
    if args.total_duration < args.goal_delay + args.trajectory_duration + args.terminal_hold:
        raise ValueError("total duration must include trajectory and terminal hold")


def _write_csv(path, fieldnames, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _publish_gait(publisher, message_type, name):
    templates = {
        "stance": ([0.0, 0.5], [15]),
        "static_walk": ([0.0, 0.3, 0.6, 0.9, 1.2], [13, 7, 14, 11]),
    }
    event_times, mode_sequence = templates[name]
    message = message_type()
    message.eventTimes = event_times
    message.modeSequence = mode_sequence
    publisher.publish(message)


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped, Twist
    from ocs2_msgs.msg import (
        mode_schedule,
        mpc_input,
        mpc_observation,
        mpc_state,
        mpc_target_trajectories,
    )
    from std_msgs.msg import Float64
    from tf.transformations import quaternion_from_euler

    args = parse_args(rospy.myargv()[1:])
    _validate(args)
    output_paths = (
        args.goal_output,
        args.planner_output,
        args.motion_output,
        args.events_output,
    )
    for path in output_paths:
        Path(path).parent.mkdir(parents=True, exist_ok=True)

    rospy.init_node("tp01_goal_publisher")
    lock = threading.Lock()
    latest_observation = {"message": None}

    def observation_callback(message):
        with lock:
            latest_observation["message"] = message

    rospy.Subscriber(
        "/legged_robot_mpc_observation",
        mpc_observation,
        observation_callback,
        queue_size=1,
    )
    rospy.wait_for_message(
        "/legged_robot_mpc_observation", mpc_observation, timeout=240.0)
    campaign_start_ros = rospy.wait_for_message(
        CAMPAIGN_START_TOPIC, Float64, timeout=240.0).data

    goal_publisher = rospy.Publisher(
        "/tp01/goal", PoseStamped, queue_size=1, latch=True)
    planner_publisher = rospy.Publisher(
        "/tp01/planned_reference",
        mpc_target_trajectories,
        queue_size=1,
        latch=True,
    )
    target_publisher = rospy.Publisher(
        "/legged_robot_mpc_target",
        mpc_target_trajectories,
        queue_size=1,
        latch=True,
    )
    gait_publisher = rospy.Publisher(
        "/legged_robot_mpc_mode_schedule",
        mode_schedule,
        queue_size=1,
        latch=True,
    )
    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.sleep(0.25)
    _publish_gait(gait_publisher, mode_schedule, "stance")
    velocity_publisher.publish(Twist())

    motion_fields = (
        "ros_time",
        "campaign_start_ros",
        "wall_elapsed",
        "active",
        "gait",
        "command_source",
    )
    event_fields = ("ros_time", "campaign_start_ros", "wall_elapsed", "event")
    motion_handle = Path(args.motion_output).open(
        "w", encoding="utf-8", newline="")
    event_handle = Path(args.events_output).open(
        "w", encoding="utf-8", newline="")
    motion_writer = csv.DictWriter(motion_handle, fieldnames=motion_fields)
    event_writer = csv.DictWriter(event_handle, fieldnames=event_fields)
    motion_writer.writeheader()
    event_writer.writeheader()

    def write_event(event):
        ros_now = rospy.get_time()
        event_writer.writerow({
            "ros_time": ros_now,
            "campaign_start_ros": campaign_start_ros,
            "wall_elapsed": campaign_elapsed(ros_now, campaign_start_ros),
            "event": event,
        })
        event_handle.flush()

    write_event("startup")
    rate = rospy.Rate(args.rate)
    goal_sent = False
    gait_stopped = False
    try:
        while not rospy.is_shutdown():
            ros_now = rospy.get_time()
            elapsed = campaign_elapsed(ros_now, campaign_start_ros)
            if elapsed >= args.total_duration:
                break

            if not goal_sent and elapsed >= args.goal_delay:
                with lock:
                    observation = latest_observation["message"]
                if observation is None or len(observation.state.value) < 12:
                    raise RuntimeError("valid MPC observation is unavailable")
                measured_start = tuple(
                    float(value) for value in observation.state.value[6:12])
                planned_start = (
                    measured_start[0],
                    measured_start[1],
                    args.com_height,
                    measured_start[3],
                    0.0,
                    0.0,
                )
                if args.trajectory_id == "t01":
                    heading = planned_start[3]
                    goal_pose = make_relative_goal(
                        planned_start,
                        args.forward_length * math.cos(heading),
                        args.forward_length * math.sin(heading),
                        0.0,
                        args.com_height,
                    )
                    motion_reference = build_cruise_reference(
                        planned_start, goal_pose, float(observation.time),
                        args.trajectory_duration, args.rate, args.accel_duration)
                else:
                    motion_reference = build_single_bend_reference(
                        planned_start, args.forward_length,
                        args.lateral_amplitude, float(observation.time),
                        args.trajectory_duration, args.rate, args.accel_duration)
                    goal_pose = motion_reference[-1].pose
                reference = append_terminal_hold(
                    motion_reference, args.terminal_hold)
                payload = build_target_payload(
                    reference,
                    tuple(float(value) for value in observation.state.value),
                    len(observation.input.value),
                    GO2_DEFAULT_JOINT_STATE,
                )
                target_message = build_target_message(
                    payload,
                    mpc_target_trajectories,
                    mpc_state,
                    mpc_input,
                )

                goal_message = PoseStamped()
                goal_message.header.stamp = rospy.Time.now()
                goal_message.header.frame_id = "odom"
                goal_message.pose.position.x = goal_pose[0]
                goal_message.pose.position.y = goal_pose[1]
                goal_message.pose.position.z = goal_pose[2]
                quaternion = quaternion_from_euler(0.0, 0.0, goal_pose[3])
                goal_message.pose.orientation.x = quaternion[0]
                goal_message.pose.orientation.y = quaternion[1]
                goal_message.pose.orientation.z = quaternion[2]
                goal_message.pose.orientation.w = quaternion[3]

                Path(args.goal_output).write_text(json.dumps({
                    "frame": "odom",
                    "relative_goal": {
                        "dx_m": goal_pose[0] - planned_start[0],
                        "dy_m": goal_pose[1] - planned_start[1],
                        "dyaw_rad": goal_pose[3] - planned_start[3],
                    },
                    "trajectory_id": args.trajectory_id,
                    "forward_length_m": args.forward_length,
                    "lateral_amplitude_m": (args.lateral_amplitude if args.trajectory_id == "t02" else 0.0),
                    "terminal_hold_s": args.terminal_hold,
                    "measured_start_pose": list(measured_start),
                    "planned_start_pose": list(planned_start),
                    "absolute_goal_pose": list(goal_pose),
                    "campaign_start_ros": campaign_start_ros,
                    "published_ros_time": ros_now,
                    "published_elapsed_s": elapsed,
                    "observation_time": float(observation.time),
                    "trajectory_duration_s": args.trajectory_duration,
                    "accel_duration_s": args.accel_duration,
                    "reference_profile": "smooth_accel_cruise_brake",
                    "trajectory_rate_hz": args.rate,
                    "source": "go2_reference_trajectory",
                }, indent=2) + "\n", encoding="utf-8")
                _write_csv(
                    args.planner_output,
                    PLANNER_FIELDS,
                    planner_rows(reference, elapsed),
                )
                goal_publisher.publish(goal_message)
                planner_publisher.publish(target_message)
                target_publisher.publish(target_message)
                _publish_gait(
                    gait_publisher, mode_schedule, "static_walk")
                write_event("goal_and_planned_reference_published")
                goal_sent = True
                rospy.loginfo(
                    "%s published %.3f m path with %d reference knots",
                    args.trajectory_id.upper(), args.forward_length,
                    len(reference),
                )

            active = goal_sent
            in_terminal_hold = (
                goal_sent and
                elapsed >= args.goal_delay + args.trajectory_duration)

            motion_writer.writerow({
                "ros_time": ros_now,
                "campaign_start_ros": campaign_start_ros,
                "wall_elapsed": elapsed,
                "active": int(active),
                "gait": "static_walk" if active else "stance",
                "command_source": ("terminal_hold" if in_terminal_hold else
                                   "planned_reference" if active else "hold"),
            })
            motion_handle.flush()
            rate.sleep()
    finally:
        _publish_gait(gait_publisher, mode_schedule, "stance")
        velocity_publisher.publish(Twist())
        write_event("finished")
        motion_handle.close()
        event_handle.close()


if __name__ == "__main__":
    main()
