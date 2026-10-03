#!/usr/bin/env python3
"""Exit successfully when the observed MPC clock reaches a requested time."""
import sys
import rospy
from ocs2_msgs.msg import mpc_observation

TARGET = float(sys.argv[1])

def callback(message):
    if message.time >= TARGET:
        rospy.signal_shutdown("requested MPC duration reached")

rospy.init_node("joint_uncertainty_pilot_waiter", anonymous=True)
rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation, callback, queue_size=1)
rospy.spin()
