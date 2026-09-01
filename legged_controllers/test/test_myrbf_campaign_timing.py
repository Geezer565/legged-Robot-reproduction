#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))


def load_script(name):
    path = SCRIPTS / (name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class MyRbfCampaignTimingTest(unittest.TestCase):
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

    def test_all_timing_scripts_use_shared_ros_elapsed_after_delayed_start(self):
        modules = [
            load_script("record_stage8c"),
            load_script("record_myrbf_shadow"),
            load_script("publish_myrbf_motion"),
            load_script("apply_myrbf_wrench"),
        ]
        timing = sys.modules["myrbf_campaign_timing"]
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
