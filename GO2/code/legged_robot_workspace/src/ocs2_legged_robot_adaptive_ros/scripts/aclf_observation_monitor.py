#!/usr/bin/env python3
"""只读 actual observation 与原生 target，发布 ON/OFF 共用的实验 tracking 指标。"""

import math
import threading

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from ocs2_msgs.msg import mpc_observation, mpc_target_trajectories


def interpolate(time, times, states):
    """匹配 OCS2 TargetTrajectories 的线性插值和零阶外推语义。"""
    if time <= times[0]:
        return states[0]
    if time >= times[-1]:
        return states[-1]
    for index in range(len(times) - 1):
        if times[index] < time <= times[index + 1]:
            alpha = (times[index + 1] - time) / (times[index + 1] - times[index])
            return [alpha * a + (1.0 - alpha) * b for a, b in zip(states[index], states[index + 1])]
    return states[-1]


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ObservationMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.target_times = None
        self.target_states = None
        self.observation_time_pub = rospy.Publisher("/task_b/observation_time", Float64, queue_size=10)
        self.desired_x_pub = rospy.Publisher("/task_b/desired_x", Float64, queue_size=10)
        self.actual_x_pub = rospy.Publisher("/task_b/actual_x", Float64, queue_size=10)
        self.x_error_pub = rospy.Publisher("/aclf_experiment/x_error", Float64, queue_size=10)
        self.desired_position_pub = rospy.Publisher("/task_b/desired_position", Float64MultiArray, queue_size=10)
        self.actual_position_pub = rospy.Publisher("/task_b/actual_position", Float64MultiArray, queue_size=10)
        self.pitch_error_pub = rospy.Publisher("/aclf_experiment/pitch_error", Float64, queue_size=10)
        self.roll_error_pub = rospy.Publisher("/task_b/roll_error", Float64, queue_size=10)
        self.yaw_error_pub = rospy.Publisher("/task_b/yaw_error", Float64, queue_size=10)
        self.desired_rpy_pub = rospy.Publisher("/task_b/desired_rpy", Float64MultiArray, queue_size=10)
        self.actual_rpy_pub = rospy.Publisher("/task_b/actual_rpy", Float64MultiArray, queue_size=10)
        self.orientation_error_rpy_pub = rospy.Publisher("/task_b/orientation_error_rpy", Float64MultiArray, queue_size=10)
        self.orientation_error_norm_pub = rospy.Publisher("/task_b/orientation_error_norm", Float64, queue_size=10)
        self.position_norm_pub = rospy.Publisher("/aclf_experiment/position_error_norm", Float64, queue_size=10)
        self.policy_input_pub = rospy.Publisher("/task_b/policy_input_sample", Float64MultiArray, queue_size=10)

    def target_callback(self, message):
        if len(message.timeTrajectory) != len(message.stateTrajectory) or not message.timeTrajectory:
            rospy.logwarn("忽略不完整的 mpc_target_trajectories")
            return
        with self.lock:
            self.target_times = list(message.timeTrajectory)
            self.target_states = [list(state.value) for state in message.stateTrajectory]

    def observation_callback(self, message):
        actual = list(message.state.value)
        if len(actual) < 12:
            rospy.logwarn_throttle(1.0, "observation state 维度不足，无法发布 base tracking error")
            return
        self.observation_time_pub.publish(Float64(message.time))
        policy_sample = Float64MultiArray()
        policy_sample.data = [message.time, float(message.mode)] + list(message.input.value)
        self.policy_input_pub.publish(policy_sample)
        with self.lock:
            if self.target_times is None:
                return
            desired = interpolate(message.time, self.target_times, self.target_states)
        if len(desired) < 12:
            return

        # 所有量均为真实 observation 的 base pose [x,y,z,yaw,pitch,roll]，
        # 绝不读取 solver 内预测状态；误差方向与 ACLF 一致，为 desired - actual。
        x_error = desired[6] - actual[6]
        desired_rpy = [desired[11], desired[10], desired[9]]
        actual_rpy = [actual[11], actual[10], actual[9]]
        orientation_error_rpy = [wrap_angle(wanted - measured)
                                 for wanted, measured in zip(desired_rpy, actual_rpy)]
        roll_error, pitch_error, yaw_error = orientation_error_rpy
        position_error_norm = math.sqrt(sum((desired[i] - actual[i]) ** 2 for i in (6, 7, 8)))
        orientation_error_norm = math.sqrt(sum(value * value for value in orientation_error_rpy))
        self.desired_x_pub.publish(Float64(desired[6]))
        self.actual_x_pub.publish(Float64(actual[6]))
        self.x_error_pub.publish(Float64(x_error))
        desired_position = Float64MultiArray()
        desired_position.data = desired[6:9]
        self.desired_position_pub.publish(desired_position)
        actual_position = Float64MultiArray()
        actual_position.data = actual[6:9]
        self.actual_position_pub.publish(actual_position)
        self.roll_error_pub.publish(Float64(roll_error))
        self.pitch_error_pub.publish(Float64(pitch_error))
        self.yaw_error_pub.publish(Float64(yaw_error))
        desired_rpy_message = Float64MultiArray()
        desired_rpy_message.data = desired_rpy
        self.desired_rpy_pub.publish(desired_rpy_message)
        actual_rpy_message = Float64MultiArray()
        actual_rpy_message.data = actual_rpy
        self.actual_rpy_pub.publish(actual_rpy_message)
        orientation_error_message = Float64MultiArray()
        orientation_error_message.data = orientation_error_rpy
        self.orientation_error_rpy_pub.publish(orientation_error_message)
        self.orientation_error_norm_pub.publish(Float64(orientation_error_norm))
        self.position_norm_pub.publish(Float64(position_error_norm))


def main():
    rospy.init_node("aclf_observation_monitor")
    monitor = ObservationMonitor()
    rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation, monitor.observation_callback, queue_size=10)
    rospy.Subscriber("/legged_robot_mpc_target", mpc_target_trajectories, monitor.target_callback, queue_size=10)
    # 不发布 sigma_norm：仅从 mpc_observation 无法无歧义恢复系统 COM 与 world angular velocity。
    # 发布伪 sigma 会破坏 ON/OFF 对照的科学性；离线分析会明确报告该指标 unavailable。
    rospy.spin()


if __name__ == "__main__":
    main()
