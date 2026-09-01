import math


CAMPAIGN_START_TOPIC = "/myrbf_campaign_start"
DIAGNOSTIC_READINESS_WINDOW_S = 0.5
DIAGNOSTIC_READINESS_MAX_GAP_S = 0.05


class DiagnosticReadinessWindow:
    def __init__(self):
        self._window_start = None
        self._last_arrival = None
        self._ready = False

    def _reset(self):
        self._window_start = None
        self._last_arrival = None
        self._ready = False

    def observe(self, ros_time):
        ros_time = float(ros_time)
        if not math.isfinite(ros_time):
            self._reset()
            raise ValueError("diagnostic ROS arrival time must be finite")
        if self._last_arrival is not None and ros_time <= self._last_arrival:
            self._reset()
            raise ValueError(
                "diagnostic ROS arrival times must be strictly increasing")
        if (self._last_arrival is None or
                ros_time - self._last_arrival >
                DIAGNOSTIC_READINESS_MAX_GAP_S):
            self._window_start = ros_time
            self._ready = False
        self._last_arrival = ros_time
        if (ros_time - self._window_start + 1e-12 >=
                DIAGNOSTIC_READINESS_WINDOW_S):
            self._ready = True
        return self._ready

    def ready(self):
        return self._ready


def campaign_elapsed(ros_now, campaign_start_ros):
    ros_now = float(ros_now)
    campaign_start_ros = float(campaign_start_ros)
    if not math.isfinite(ros_now) or not math.isfinite(campaign_start_ros):
        raise ValueError("campaign ROS times must be finite")
    return ros_now - campaign_start_ros


def publish_campaign_start_when_running(
        list_controller_state, diagnostic_ready, publish_start, ros_now, sleep,
        is_shutdown, timeout_s, monotonic_now):
    deadline = monotonic_now() + timeout_s
    while not is_shutdown() and monotonic_now() < deadline:
        if (list_controller_state() == "running" and diagnostic_ready()):
            candidate_start = float(ros_now())
            publish_start(candidate_start)
            return candidate_start
        sleep(0.05)
    raise RuntimeError(
        "Go2 controller and diagnostic readiness did not complete before timeout")
