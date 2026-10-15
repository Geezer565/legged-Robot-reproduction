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
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import Empty

from legged_controllers.myrbf_campaign_timing import (
    CAMPAIGN_START_TOPIC,
    DiagnosticReadinessWindow,
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
        rospy.wait_for_service(service, timeout=180.0)
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
    diagnostic_readiness = DiagnosticReadinessWindow()
    require_v2_feasible = rospy.get_param("~require_v2_feasible", False)
    require_v11_ready = rospy.get_param("~require_v11_ready", False)
    require_adaptive_diagnostic = rospy.get_param("~require_adaptive_diagnostic", True)

    def observe_diagnostic_arrival(message):
        if require_v11_ready:
            columns = rospy.get_param("/myrbf_v11_columns", [])
            sample = dict(zip(columns, message.data))
            # Campaign readiness must NOT depend on learning_enabled.
            # The adaptive law is intentionally allowed to remain disabled
            # during activation_warmup while the experiment clock is already
            # running.  We only require a healthy V1.1 WBC solution and a
            # consistent MPC/WBC snapshot before publishing campaign start.
            if (sample.get("wbc_status") != 2
                    or sample.get("mpc_snapshot_id") != sample.get("wbc_snapshot_id")):
                return
        # Start a recorded campaign only after V2 itself has produced a valid
        # hard-constrained solution. Warmup diagnostics intentionally do not
        # advance the readiness window.
        if (require_v2_feasible and
                (len(message.data) <= 12 or message.data[11] < 0.5 or
                 int(message.data[12]) != 2)):
            return
        arrival = rospy.get_time()
        # Several controller publications can be delivered within one Gazebo
        # clock tick.  They are duplicate evidence, not a clock reset.  Ignore
        # exact duplicates while still forwarding backward time to the shared
        # readiness window, where it resets the window as before.
        previous = getattr(observe_diagnostic_arrival, "last_ros_time", None)
        if previous is not None and arrival == previous:
            return
        observe_diagnostic_arrival.last_ros_time = arrival
        try:
            diagnostic_readiness.observe(arrival)
        except ValueError as error:
            rospy.logwarn("MyRBF diagnostic readiness reset: %s", error)

    diagnostic_subscriber = None
    if require_adaptive_diagnostic:
        diagnostic_subscriber = rospy.Subscriber(
            "/myrbf_shadow_diagnostics",
            Float64MultiArray,
            observe_diagnostic_arrival,
            queue_size=1,
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
    request.timeout = 30.0
    if not switch_controller(request).ok:
        raise RuntimeError("Failed to start the Go2 controllers")

    def controller_state():
        states = {item.name: item.state for item in list_controllers().controller}
        return states.get("controllers/legged_controller")

    if require_adaptive_diagnostic:
        campaign_start = publish_campaign_start_when_running(
            list_controller_state=controller_state,
            diagnostic_readiness=diagnostic_readiness,
            publish_start=lambda value: campaign_start_publisher.publish(Float64(value)),
            ros_now=rospy.get_time,
            sleep=time.sleep,
            is_shutdown=rospy.is_shutdown,
            timeout_s=90.0,
            monotonic_now=time.monotonic,
        )
    else:
        # Pure MPC has no adaptive publisher by design.  Its campaign start
        # depends only on controller-manager state, never a diagnostic topic.
        deadline = time.monotonic() + 90.0
        campaign_start = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if controller_state() == "running":
                campaign_start = rospy.get_time()
                campaign_start_publisher.publish(Float64(campaign_start))
                break
            time.sleep(0.05)
        if campaign_start is None:
            raise RuntimeError("Pure MPC controller did not reach running state before timeout")
    rospy.loginfo(
        "Go2 controller and diagnostics are ready; shared campaign start %.9f published.",
        campaign_start,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
