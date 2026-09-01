#!/usr/bin/env python3

import time

import rospy
from controller_manager_msgs.srv import (
    ListControllers,
    LoadController,
    SwitchController,
    SwitchControllerRequest,
)
from gazebo_msgs.srv import GetJointProperties
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from myrbf_campaign_timing import (
    CAMPAIGN_START_TOPIC,
    publish_campaign_start_when_running,
)


def main():
    rospy.init_node("myrbf_unpause_when_controller_ready")
    for service in (
        "/controller_manager/list_controllers",
        "/controller_manager/load_controller",
        "/controller_manager/switch_controller",
        "/gazebo/get_joint_properties",
        "/gazebo/unpause_physics",
    ):
        rospy.wait_for_service(service, timeout=30.0)
    list_controllers = rospy.ServiceProxy(
        "/controller_manager/list_controllers", ListControllers
    )
    load_controller = rospy.ServiceProxy(
        "/controller_manager/load_controller", LoadController
    )
    switch_controller = rospy.ServiceProxy(
        "/controller_manager/switch_controller", SwitchController
    )
    get_joint = rospy.ServiceProxy(
        "/gazebo/get_joint_properties", GetJointProperties
    )
    campaign_start_publisher = rospy.Publisher(
        CAMPAIGN_START_TOPIC, Float64, queue_size=1, latch=True
    )

    # spawn_model applies -J positions after its own one-second workaround.
    # Do not load the controller until Gazebo confirms those positions.
    pose_deadline = time.monotonic() + 10.0
    pose_ready = False
    while not rospy.is_shutdown() and time.monotonic() < pose_deadline:
        hip = get_joint("go2::LF_HFE")
        knee = get_joint("go2::LF_KFE")
        if (
            hip.success and knee.success
            and hip.position and knee.position
            and abs(hip.position[0] - 0.72) < 0.02
            and abs(knee.position[0] + 1.44) < 0.02
        ):
            pose_ready = True
            break
        time.sleep(0.05)
    if not pose_ready:
        raise RuntimeError("Go2 standing joint pose was not applied")

    controllers = (
        "controllers/joint_state_controller",
        "controllers/legged_controller",
    )
    for controller in controllers:
        if not load_controller(controller).ok:
            raise RuntimeError("Failed to load %s" % controller)

    # A paused Gazebo cannot complete a controller switch because its update
    # loop is stopped. Load first, release physics, then atomically start both.
    rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
    request = SwitchControllerRequest()
    request.start_controllers = list(controllers)
    request.strictness = SwitchControllerRequest.STRICT
    request.start_asap = True
    request.timeout = 5.0
    if not switch_controller(request).ok:
        raise RuntimeError("Failed to start the Go2 controllers")

    def controller_state():
        states = {item.name: item.state for item in list_controllers().controller}
        return states.get("controllers/legged_controller")

    campaign_start = publish_campaign_start_when_running(
        list_controller_state=controller_state,
        publish_start=lambda value: campaign_start_publisher.publish(Float64(value)),
        ros_now=rospy.get_time,
        sleep=rospy.sleep,
        is_shutdown=rospy.is_shutdown,
        timeout_s=30.0,
        monotonic_now=time.monotonic,
    )
    rospy.loginfo(
        "Go2 controller is running; shared campaign start %.9f published.",
        campaign_start,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
