#!/usr/bin/env python3

import importlib
import importlib.util
import inspect
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest import mock


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def load_script(name):
    path = SCRIPTS / (name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def import_installed_timing():
    return importlib.import_module(
        "legged_controllers.myrbf_campaign_timing")


class MyRbfCampaignTimingTest(unittest.TestCase):
    def make_readiness_window(self, timing):
        window_type = getattr(timing, "DiagnosticReadinessWindow", None)
        self.assertIsNotNone(
            window_type,
            "stable diagnostic-readiness window contract is missing",
        )
        readiness = window_type()
        for method in ("set_controller_running", "claim_start"):
            self.assertTrue(
                callable(getattr(readiness, method, None)),
                "atomic diagnostic-readiness contract is missing %s" % method,
            )
        return readiness

    def assert_atomic_helper_contract(self, helper):
        self.assertIn(
            "diagnostic_readiness",
            inspect.signature(helper).parameters,
            "campaign-start helper lacks the synchronized readiness contract",
        )

    # Production mutation caught: publishing on controller-manager running
    # alone lets the preserved 0.024..0.248 startup burst enter the campaign
    # before the deterministic 0.749-second diagnostic pause.
    def test_preserved_startup_burst_cannot_pass_stable_readiness_window(self):
        timing = import_installed_timing()
        readiness = self.make_readiness_window(timing)
        readiness.set_controller_running(True)
        startup_arrivals = (
            0.024, 0.031, 0.038, 0.045, 0.052, 0.059, 0.073,
            0.080, 0.087, 0.094, 0.101, 0.108, 0.115, 0.122,
            0.248,
        )

        for arrival in startup_arrivals:
            self.assertFalse(readiness.observe(arrival))
        self.assertFalse(readiness.observe(0.997))
        for step in range(1, 50):
            self.assertFalse(readiness.observe(0.997 + 0.01 * step))
        self.assertTrue(readiness.observe(1.497))
        self.assertTrue(readiness.ready())

    # Production mutation caught: retaining the old window origin after a
    # large diagnostic gap lets disconnected bursts add up to readiness.
    def test_large_gap_resets_continuous_readiness_window(self):
        timing = import_installed_timing()
        readiness = self.make_readiness_window(timing)
        readiness.set_controller_running(True)

        for step in range(21):
            self.assertFalse(readiness.observe(2.0 + 0.01 * step))
        self.assertFalse(readiness.observe(2.30))
        for step in range(1, 50):
            self.assertFalse(readiness.observe(2.30 + 0.01 * step))
        self.assertTrue(readiness.observe(2.80))

    # Production mutation caught: accepting NaN/Inf arrivals can make an
    # invalid ROS clock satisfy or permanently poison the readiness state.
    def test_nonfinite_arrival_is_rejected_and_resets_window(self):
        timing = import_installed_timing()
        for invalid in (math.nan, math.inf, -math.inf):
            readiness = self.make_readiness_window(timing)
            readiness.set_controller_running(True)
            self.assertFalse(readiness.observe(3.0))
            with self.assertRaises(ValueError):
                readiness.observe(invalid)
            for step in range(50):
                self.assertFalse(readiness.observe(4.0 + 0.01 * step))
            self.assertTrue(readiness.observe(4.50))

    # Production mutation caught: accepting equal or backward ROS arrivals
    # hides a clock reset and incorrectly preserves the prior stable span.
    def test_nonmonotonic_arrival_is_rejected_and_resets_window(self):
        timing = import_installed_timing()
        for invalid in (5.10, 5.05):
            readiness = self.make_readiness_window(timing)
            readiness.set_controller_running(True)
            self.assertFalse(readiness.observe(5.0))
            self.assertFalse(readiness.observe(5.10))
            with self.assertRaises(ValueError):
                readiness.observe(invalid)
            for step in range(50):
                self.assertFalse(readiness.observe(6.0 + 0.01 * step))
            self.assertTrue(readiness.observe(6.50))

    # Production mutation caught: installing the helper only through
    # catkin_install_python leaves no package-qualified module for fresh
    # devel-space Python processes or installed timing consumers to import.
    def test_installed_package_exports_complete_timing_contract(self):
        try:
            timing = import_installed_timing()
        except ImportError as error:
            self.fail("installed timing package is not importable: %s" % error)

        published = []
        readiness = self.make_readiness_window(timing)
        readiness.set_controller_running(True)
        for step in range(51):
            readiness.observe(10.0 + 0.01 * step)
        start = timing.publish_campaign_start_when_running(
            list_controller_state=lambda: "running",
            diagnostic_readiness=readiness,
            publish_start=published.append,
            ros_now=lambda: 10.50,
            sleep=lambda _seconds: None,
            is_shutdown=lambda: False,
            timeout_s=30.0,
            monotonic_now=iter((0.0, 1.0)).__next__,
        )

        self.assertEqual(timing.CAMPAIGN_START_TOPIC,
                         "/myrbf_campaign_start")
        self.assertEqual(start, 10.50)
        self.assertEqual(published, [10.50])
        self.assertEqual(timing.campaign_elapsed(106.0, 100.0), 6.0)

    def test_delayed_controller_readiness_publishes_one_shared_ros_start(self):
        unpause = load_script("unpause_when_controller_ready")
        self.assert_atomic_helper_contract(
            unpause.publish_campaign_start_when_running)
        states = iter(("initialized", "initialized", "running"))
        published = []
        clock_reads = []

        class ClaimOnFirstRunningPoll:
            def __init__(self):
                self.running = False

            def set_controller_running(self, running):
                self.running = running

            def claim_start(self, ros_now, publish_start):
                if not self.running:
                    return None
                candidate = float(ros_now())
                publish_start(candidate)
                return candidate

        readiness = ClaimOnFirstRunningPoll()

        def ros_now():
            clock_reads.append("read")
            return 100.0

        start = unpause.publish_campaign_start_when_running(
            list_controller_state=lambda: next(states),
            diagnostic_readiness=readiness,
            publish_start=published.append,
            ros_now=ros_now,
            sleep=lambda _seconds: None,
            is_shutdown=lambda: False,
            timeout_s=30.0,
            monotonic_now=iter((0.0, 1.0, 2.0, 3.0)).__next__,
        )

        self.assertEqual(start, 100.0)
        self.assertEqual(published, [100.0])
        self.assertEqual(clock_reads, ["read"])

    # Production mutation caught: treating controller running as sufficient
    # publishes even though diagnostics never establish a continuous window.
    def test_diagnostic_readiness_timeout_publishes_nothing(self):
        timing = import_installed_timing()
        published = []

        with self.assertRaisesRegex(RuntimeError, "diagnostic readiness"):
            timing.publish_campaign_start_when_running(
                list_controller_state=lambda: "running",
                diagnostic_readiness=self.make_readiness_window(timing),
                publish_start=published.append,
                ros_now=lambda: 100.0,
                sleep=lambda _seconds: None,
                is_shutdown=lambda: False,
                timeout_s=1.0,
                monotonic_now=iter((0.0, 0.0, 0.4, 0.8, 1.2)).__next__,
            )

        self.assertEqual(published, [])

    # Production mutation caught: continuing the wait loop after success can
    # publish more than one campaign origin to the shared latched topic.
    def test_controller_and_diagnostics_publish_exactly_once(self):
        timing = import_installed_timing()
        self.assert_atomic_helper_contract(
            timing.publish_campaign_start_when_running)
        published = []

        class ClaimOnThirdAttempt:
            def __init__(self):
                self.attempts = 0

            def set_controller_running(self, _running):
                return None

            def claim_start(self, ros_now, publish_start):
                self.attempts += 1
                if self.attempts < 3:
                    return None
                candidate = float(ros_now())
                publish_start(candidate)
                return candidate

        start = timing.publish_campaign_start_when_running(
            list_controller_state=lambda: "running",
            diagnostic_readiness=ClaimOnThirdAttempt(),
            publish_start=published.append,
            ros_now=lambda: 100.0,
            sleep=lambda _seconds: None,
            is_shutdown=lambda: False,
            timeout_s=30.0,
            monotonic_now=iter((0.0, 1.0, 2.0, 3.0, 4.0)).__next__,
        )

        self.assertEqual(start, 100.0)
        self.assertEqual(published, [100.0])

    # Production mutation caught: returning from the actual unpause helper
    # immediately after the latched publication, before a late subscriber can
    # connect and receive the retained campaign start.
    def test_unpause_main_keeps_latched_start_alive_for_late_subscriber(self):
        unpause = load_script("unpause_when_controller_ready")
        published = []
        late_received = []

        class LatchedPublisher:
            def publish(self, message):
                published.append(message.data)

        def service_proxy(name, _service_type):
            if name == "/controller_manager/list_controllers":
                return lambda: SimpleNamespace(controller=[
                    SimpleNamespace(
                        name="controllers/legged_controller",
                        state="running",
                    ),
                ])
            if name == "/controller_manager/load_controller":
                return lambda _controller: SimpleNamespace(ok=True)
            if name == "/controller_manager/switch_controller":
                return lambda _request: SimpleNamespace(ok=True)
            if name == "/gazebo/get_joint_properties":
                return lambda joint: SimpleNamespace(
                    success=True,
                    position=[0.72 if joint.endswith("LF_HFE") else -1.44],
                )
            if name == "/gazebo/unpause_physics":
                return lambda: SimpleNamespace()
            raise AssertionError("unexpected service: %s" % name)

        def spin_until_late_subscriber_connects():
            late_received.append(published[-1])

        diagnostic_callback = []
        ros_clock = {"now": 10.0}

        def subscribe(_topic, _message_type, callback, queue_size):
            self.assertEqual(queue_size, 1)
            diagnostic_callback.append(callback)
            for _step in range(51):
                callback(SimpleNamespace(data=[]))
                ros_clock["now"] += 0.01
            ros_clock["now"] = 20.0
            return SimpleNamespace()

        def advance_diagnostics(_seconds):
            diagnostic_callback[0](SimpleNamespace(data=[]))
            ros_clock["now"] += 0.01

        with mock.patch.multiple(
                unpause.rospy,
                init_node=mock.DEFAULT,
                wait_for_service=mock.DEFAULT,
                ServiceProxy=mock.Mock(side_effect=service_proxy),
                Publisher=mock.Mock(return_value=LatchedPublisher()),
                Subscriber=mock.Mock(side_effect=subscribe),
                is_shutdown=mock.Mock(return_value=False),
                get_time=mock.Mock(side_effect=lambda: ros_clock["now"]),
                sleep=mock.Mock(side_effect=advance_diagnostics),
                loginfo=mock.DEFAULT,
                spin=mock.Mock(side_effect=spin_until_late_subscriber_connects),
        ), mock.patch.object(unpause.time, "monotonic", return_value=0.0):
            unpause.main()

        self.assertEqual(len(published), 1)
        self.assertAlmostEqual(published[0], 20.51)
        self.assertEqual(late_received, published)

    # Production mutation caught: accepting diagnostics before controller
    # running lets an already-ready startup burst satisfy the start gate.
    def test_pre_controller_arrivals_do_not_count_toward_full_window(self):
        timing = import_installed_timing()
        readiness = self.make_readiness_window(timing)
        published = []

        for step in range(51):
            self.assertFalse(readiness.observe(10.0 + 0.01 * step))
        readiness.set_controller_running(True)
        for step in range(50):
            self.assertFalse(readiness.observe(20.0 + 0.01 * step))
        self.assertIsNone(
            readiness.claim_start(lambda: 20.49, published.append))
        self.assertTrue(readiness.observe(20.50))
        self.assertEqual(
            readiness.claim_start(lambda: 20.50, published.append), 20.50)

        self.assertEqual(published, [20.50])

    # Production mutation caught: a latched ready boolean can publish after
    # diagnostic silence even when the selected start exceeds the max gap.
    def test_ready_window_is_rejected_after_diagnostic_silence(self):
        timing = import_installed_timing()
        readiness = self.make_readiness_window(timing)
        published = []

        readiness.set_controller_running(True)
        for step in range(51):
            readiness.observe(30.0 + 0.01 * step)
        self.assertTrue(readiness.ready())

        self.assertIsNone(
            readiness.claim_start(lambda: 30.56, published.append))
        self.assertFalse(readiness.ready())
        self.assertEqual(published, [])

    # Production mutation caught: without checking the exact publication
    # candidate, a backward ROS reset remains ready until another callback.
    def test_backward_publication_candidate_resets_without_later_callback(self):
        timing = import_installed_timing()
        readiness = self.make_readiness_window(timing)
        published = []

        readiness.set_controller_running(True)
        for step in range(51):
            readiness.observe(40.0 + 0.01 * step)
        self.assertTrue(readiness.ready())

        self.assertIsNone(
            readiness.claim_start(lambda: 1.0, published.append))
        self.assertFalse(readiness.ready())
        self.assertEqual(published, [])

    # Production mutation caught: checking ready outside the observation
    # lock lets a reset callback interleave before the final publication.
    def test_observation_cannot_interleave_with_atomic_publication_claim(self):
        timing = import_installed_timing()
        readiness = self.make_readiness_window(timing)
        published = []
        callback_started = threading.Event()
        callback_finished = threading.Event()
        callback_thread = []

        readiness.set_controller_running(True)
        for step in range(51):
            readiness.observe(50.0 + 0.01 * step)

        def reset_from_callback():
            callback_started.set()
            with self.assertRaises(ValueError):
                readiness.observe(1.0)
            callback_finished.set()

        def publish_while_callback_waits(candidate):
            thread = threading.Thread(target=reset_from_callback)
            callback_thread.append(thread)
            thread.start()
            self.assertTrue(callback_started.wait(timeout=1.0))
            self.assertFalse(callback_finished.is_set())
            published.append(candidate)

        self.assertEqual(
            readiness.claim_start(lambda: 50.50, publish_while_callback_waits),
            50.50,
        )
        callback_thread[0].join(timeout=1.0)

        self.assertFalse(callback_thread[0].is_alive())
        self.assertTrue(callback_finished.is_set())
        self.assertEqual(published, [50.50])

    def test_all_timing_scripts_use_shared_ros_elapsed_after_delayed_start(self):
        modules = [
            load_script("record_stage8c"),
            load_script("record_myrbf_shadow"),
            load_script("publish_myrbf_motion"),
            load_script("apply_myrbf_wrench"),
        ]
        timing = import_installed_timing()
        campaign_start_ros = 100.0

        for module in modules:
            self.assertIs(module.campaign_elapsed, timing.campaign_elapsed)
            self.assertEqual(
                module.campaign_elapsed(campaign_start_ros, campaign_start_ros),
                0.0,
            )
            self.assertEqual(
                module.campaign_elapsed(106.0, campaign_start_ros),
                6.0,
            )

        motion = modules[2]
        wrench = modules[3]
        self.assertTrue(motion.motion_is_active(6.0, 4.0, 9.0))
        self.assertTrue(wrench.profile_is_active(6.0, 6.0, 6.0))


if __name__ == "__main__":
    unittest.main()
