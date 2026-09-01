import math


CAMPAIGN_START_TOPIC = "/myrbf_campaign_start"


def campaign_elapsed(ros_now, campaign_start_ros):
    ros_now = float(ros_now)
    campaign_start_ros = float(campaign_start_ros)
    if not math.isfinite(ros_now) or not math.isfinite(campaign_start_ros):
        raise ValueError("campaign ROS times must be finite")
    return ros_now - campaign_start_ros


def publish_campaign_start_when_running(
        list_controller_state, publish_start, ros_now, sleep, is_shutdown,
        timeout_s, monotonic_now):
    deadline = monotonic_now() + timeout_s
    while not is_shutdown() and monotonic_now() < deadline:
        if list_controller_state() == "running":
            candidate_start = float(ros_now())
            publish_start(candidate_start)
            return candidate_start
        sleep(0.05)
    raise RuntimeError("Go2 controller did not become ready before timeout")
