#!/usr/bin/env python3
"""向原生 OCS2 target 接口发送一次安全、可按 MPC time 调度的 base-x 目标。"""

import argparse
import math
import re
import sys
import time
from pathlib import Path

import rospy
from ocs2_msgs.msg import mpc_input, mpc_observation, mpc_state, mpc_target_trajectories


DEFAULT_REFERENCE_FILE = (Path(__file__).resolve().parents[2] / "ocs2" /
                          "ocs2_robotic_examples/ocs2_legged_robot/config/command/reference.info")


class SmallStepPublisher:
    def __init__(self, observation_topic):
        self.latest_observation = None
        self.subscriber = rospy.Subscriber(observation_topic, mpc_observation, self._observation_callback, queue_size=1)

    def _observation_callback(self, message):
        self.latest_observation = message

    def wait_for_observation(self, timeout):
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.latest_observation is None:
            if time.monotonic() > deadline:
                raise RuntimeError("未收到真实 MPC observation，拒绝猜测 standing target")
            rate.sleep()
        return self.latest_observation


def load_native_reference(reference_file):
    """读取原生 LeggedRobotPoseCommandNode 同一份 reference.info 的三个 target 参数。"""
    text = Path(reference_file).read_text()
    def scalar(name):
        match = re.search(r"^\s*" + re.escape(name) + r"\s+([-+0-9.eE]+)", text, re.MULTILINE)
        if not match:
            raise RuntimeError(f"reference.info 中找不到 {name}")
        return float(match.group(1))
    block = re.search(r"\bdefaultJointState\s*\{(.*?)\n\s*\}", text, re.DOTALL)
    if not block:
        raise RuntimeError("reference.info 中找不到 defaultJointState")
    joint_entries = {(int(i), int(j)): float(value) for i, j, value in re.findall(
        r"\((\d+),(\d+)\)\s+([-+0-9.eE]+)", block.group(1))}
    default_joint_state = [joint_entries[(i, 0)] for i in range(12)]
    return scalar("comHeight"), scalar("targetDisplacementVelocity"), scalar("targetRotationVelocity"), default_joint_state


def make_target(observation, dx, com_height, target_displacement_velocity, default_joint_state,
                start_time=None):
    """
    逐项复用 LeggedRobotPoseCommandNode::commandLineToTargetTrajectories()：
    state[0:6]=0（normalized centroidal momentum reference）；state[6:12] 为
    [base_x, base_y, base_z, yaw, pitch, roll]。x 相对当前 pose，z 使用
    reference.info 的 comHeight，joint target 使用同一份 defaultJointState。
    """
    actual = list(observation.state.value)
    if len(actual) != 24:
        raise RuntimeError("原生 animal_dummy pose command 要求 24 维 centroidal state")
    current_pose = actual[6:12]
    target_pose = list(current_pose)
    target_pose[0] = current_pose[0] + dx
    target_pose[2] = com_height  # deltaZ=0 时仍严格采用原生逻辑，而非复制 observation z。

    desired_start = [0.0] * 6 + current_pose + default_joint_state
    desired_final = [0.0] * 6 + target_pose + default_joint_state
    # 原生 estimateTimeToTarget() 只使用 dx、dy、dyaw；本实验 dy=dyaw=0。
    duration = abs(dx) / target_displacement_velocity

    zero_input = [0.0] * len(observation.input.value)
    message = mpc_target_trajectories()
    target_start_time = observation.time if start_time is None else start_time
    message.timeTrajectory = [target_start_time, target_start_time + duration]
    start_state, final_state = mpc_state(), mpc_state()
    start_state.value, final_state.value = desired_start, desired_final
    start_input, final_input = mpc_input(), mpc_input()
    start_input.value, final_input.value = zero_input, zero_input
    message.stateTrajectory = [start_state, final_state]
    message.inputTrajectory = [start_input, final_input]
    return message


def main():
    parser = argparse.ArgumentParser(description="发送一次原生 OCS2 +x relative target")
    parser.add_argument("--step", type=float, default=0.05, help="base-x relative displacement [m]")
    parser.add_argument("--settle-before-step", type=float, default=5.0, help="发送前 standing 保持时间 [s]")
    parser.add_argument("--start-at-mpc-time", type=float, default=None,
                        help="提前发布并在这个确定的 MPC/simulation time 开始 target；Task B 使用 0.50 s")
    parser.add_argument("--observation-topic", default="/legged_robot_mpc_observation")
    parser.add_argument("--target-topic", default="/legged_robot_mpc_target")
    parser.add_argument("--reference-file", default=None, help="默认读取 launch 写入的 /referenceFile 参数")
    parser.add_argument("--observation-timeout", type=float, default=180.0,
                        help="仅用于等待 MPC 首条 observation 的墙钟启动超时 [s]；不改变 target MPC time")
    # roslaunch appends ROS remapping arguments such as ``__name:=...`` and
    # ``__log:=...``.  argparse must only see the application arguments.
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    # 仅扩大 experiment safety 上限；duration 仍严格由原生
    # targetDisplacementVelocity 计算，不改变任何 target 数学语义。
    if not (0.0 < args.step <= 0.10):
        raise RuntimeError("本安全实验只允许 0 < step <= 0.10")

    rospy.init_node("aclf_small_step_target", anonymous=True)
    helper = SmallStepPublisher(args.observation_topic)
    publisher = rospy.Publisher(args.target_topic, mpc_target_trajectories, queue_size=1, latch=False)
    reference_file = args.reference_file or rospy.get_param("/referenceFile", str(DEFAULT_REFERENCE_FILE))
    com_height, target_displacement_velocity, _, default_joint_state = load_native_reference(reference_file)
    if target_displacement_velocity <= 0.0:
        raise RuntimeError("targetDisplacementVelocity 必须为正")
    if not math.isfinite(args.observation_timeout) or args.observation_timeout <= 0.0:
        raise RuntimeError("--observation-timeout 必须为正有限值")
    observation = helper.wait_for_observation(args.observation_timeout)
    connection_deadline = time.monotonic() + 5.0
    while publisher.get_num_connections() == 0 and not rospy.is_shutdown():
        if time.monotonic() > connection_deadline:
            raise RuntimeError("等待 /legged_robot_mpc_target subscriber 5 s 超时，拒绝丢失实验目标")
        rospy.sleep(0.05)

    if args.start_at_mpc_time is None:
        rospy.loginfo("已收到 observation；保持 standing %.1f s 后发送 +%.3f m x step",
                      args.settle_before_step, args.step)
        rospy.sleep(args.settle_before_step)
        observation = helper.wait_for_observation(1.0)
        target_start_time = None
    else:
        if not math.isfinite(args.start_at_mpc_time) or args.start_at_mpc_time < 0.0:
            raise RuntimeError("--start-at-mpc-time 必须为非负有限 MPC time")
        observation = helper.wait_for_observation(1.0)
        minimum_lead_time = 0.05
        if args.start_at_mpc_time < observation.time + minimum_lead_time:
            raise RuntimeError(
                "确定性 target 调度已错过：当前 observation time={:.6f}，要求 start={:.6f}，至少需提前 {:.3f} s".format(
                    observation.time, args.start_at_mpc_time, minimum_lead_time))
        target_start_time = args.start_at_mpc_time
        rospy.loginfo("提前调度 +%.3f m x step：固定 MPC time %.3f s 生效",
                      args.step, target_start_time)

    target = make_target(observation, args.step, com_height, target_displacement_velocity,
                         default_joint_state, target_start_time)
    publisher.publish(target)
    rospy.loginfo("已向 %s 发送一次原生 relative x step：%.3f -> %.3f m，duration=%.3f s", args.target_topic,
                  target.stateTrajectory[0].value[6], target.stateTrajectory[1].value[6],
                  target.timeTrajectory[1] - target.timeTrajectory[0])
    rospy.sleep(0.2)  # 留出 ROS transport 发送窗口；不重复发布以免改变原生一次命令语义。


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        rospy.logerr("aclf_small_step_target.py: %s", error)
        sys.exit(1)
