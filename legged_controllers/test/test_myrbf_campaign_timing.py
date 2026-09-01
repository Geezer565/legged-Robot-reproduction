#!/usr/bin/env python3

import importlib
import importlib.util
from pathlib import Path
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
    # Production mutation caught: installing the helper only through
    # catkin_install_python leaves no package-qualified module for fresh
    # devel-space Python processes or installed timing consumers to import.
    def test_installed_package_exports_complete_timing_contract(self):
        try:
            timing = import_installed_timing()
        except ImportError as error:
            self.fail("installed timing package is not importable: %s" % error)

        published = []
        start = timing.publish_campaign_start_when_running(
            list_controller_state=lambda: "running",
            publish_start=published.append,
            ros_now=lambda: 100.0,
            sleep=lambda _seconds: None,
            is_shutdown=lambda: False,
            timeout_s=30.0,
            monotonic_now=iter((0.0, 1.0)).__next__,
        )

        self.assertEqual(timing.CAMPAIGN_START_TOPIC,
                         "/myrbf_campaign_start")
        self.assertEqual(start, 100.0)
        self.assertEqual(published, [100.0])
        self.assertEqual(timing.campaign_elapsed(106.0, 100.0), 6.0)

    def test_delayed_controller_readiness_publishes_one_shared_ros_start(self):
        unpause = load_script("unpause_when_controller_ready")
        states = iter(("initialized", "initialized", "running"))
        published = []
        clock_reads = []

        def ros_now():
            clock_reads.append("read")
            return 100.0

        start = unpause.publish_campaign_start_when_running(
            list_controller_state=lambda: next(states),
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

        with mock.patch.multiple(
                unpause.rospy,
                init_node=mock.DEFAULT,
                wait_for_service=mock.DEFAULT,
                ServiceProxy=mock.Mock(side_effect=service_proxy),
                Publisher=mock.Mock(return_value=LatchedPublisher()),
                is_shutdown=mock.Mock(return_value=False),
                get_time=mock.Mock(return_value=100.0),
                sleep=mock.DEFAULT,
                loginfo=mock.DEFAULT,
                spin=mock.Mock(side_effect=spin_until_late_subscriber_connects),
        ), mock.patch.object(unpause.time, "monotonic", return_value=0.0):
            unpause.main()

        self.assertEqual(published, [100.0])
        self.assertEqual(late_received, [100.0])

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
