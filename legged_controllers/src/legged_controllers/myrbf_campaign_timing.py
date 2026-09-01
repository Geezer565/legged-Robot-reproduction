import math
import threading


CAMPAIGN_START_TOPIC = "/myrbf_campaign_start"
DIAGNOSTIC_READINESS_WINDOW_S = 0.5
DIAGNOSTIC_READINESS_MAX_GAP_S = 0.05


class DiagnosticReadinessWindow:
    def __init__(self):
        self._lock = threading.Lock()
        self._controller_running = False
        self._window_start = None
        self._last_arrival = None
        self._ready = False
        self._start_claimed = False

    def _reset_window(self):
        self._window_start = None
        self._last_arrival = None
        self._ready = False

    def set_controller_running(self, running):
        running = bool(running)
        with self._lock:
            if running != self._controller_running:
                self._reset_window()
                self._controller_running = running

    def observe(self, ros_time):
        with self._lock:
            if not self._controller_running:
                return False
            ros_time = float(ros_time)
            if not math.isfinite(ros_time):
                self._reset_window()
                raise ValueError("diagnostic ROS arrival time must be finite")
            if (self._last_arrival is not None and
                    ros_time <= self._last_arrival):
                self._reset_window()
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
        with self._lock:
            return self._ready

    def claim_start(self, ros_now, publish_start):
        with self._lock:
            if self._start_claimed or not self._controller_running:
                return None
            if not self._ready or self._last_arrival is None:
                return None
            candidate_start = float(ros_now())
            if (not math.isfinite(candidate_start) or
                    candidate_start < self._last_arrival or
                    candidate_start - self._last_arrival >
                    DIAGNOSTIC_READINESS_MAX_GAP_S + 1e-12):
                self._reset_window()
                return None
            publish_start(candidate_start)
            self._start_claimed = True
            return candidate_start


def campaign_elapsed(ros_now, campaign_start_ros):
    ros_now = float(ros_now)
    campaign_start_ros = float(campaign_start_ros)
    if not math.isfinite(ros_now) or not math.isfinite(campaign_start_ros):
        raise ValueError("campaign ROS times must be finite")
    return ros_now - campaign_start_ros


def publish_campaign_start_when_running(
        list_controller_state, diagnostic_readiness, publish_start, ros_now,
        sleep, is_shutdown, timeout_s, monotonic_now):
    deadline = monotonic_now() + timeout_s
    while not is_shutdown() and monotonic_now() < deadline:
        controller_running = list_controller_state() == "running"
        diagnostic_readiness.set_controller_running(controller_running)
        if controller_running:
            candidate_start = diagnostic_readiness.claim_start(
                ros_now, publish_start)
        else:
            candidate_start = None
        if candidate_start is not None:
            return candidate_start
        sleep(0.05)
    raise RuntimeError(
        "Go2 controller and diagnostic readiness did not complete before timeout")
