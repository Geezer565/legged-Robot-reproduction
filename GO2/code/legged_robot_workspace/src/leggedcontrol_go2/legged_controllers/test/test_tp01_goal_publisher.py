#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "publish_tp01_goal.py"
)


class VectorMessage:
    def __init__(self):
        self.value = []


class TargetMessage:
    def __init__(self):
        self.timeTrajectory = []
        self.stateTrajectory = []
        self.inputTrajectory = []


class Tp01GoalPublisherTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not SCRIPT.is_file():
            cls.module = None
            return
        spec = importlib.util.spec_from_file_location(
            "publish_tp01_goal", SCRIPT)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_build_target_message_preserves_every_payload_value(self):
        self.assertIsNotNone(self.module, "TP01 goal publisher is missing")
        self.assertTrue(hasattr(self.module, "build_target_message"))
        payload = {
            "times": (3.0, 3.05),
            "states": ((1.0, 2.0), (3.0, 4.0)),
            "inputs": ((5.0,), (6.0,)),
        }
        message = self.module.build_target_message(
            payload, TargetMessage, VectorMessage, VectorMessage)
        self.assertEqual(message.timeTrajectory, [3.0, 3.05])
        self.assertEqual(
            [item.value for item in message.stateTrajectory],
            [[1.0, 2.0], [3.0, 4.0]],
        )
        self.assertEqual(
            [item.value for item in message.inputTrajectory],
            [[5.0], [6.0]],
        )

    def test_cli_requires_distinct_goal_planner_motion_and_event_outputs(self):
        self.assertIsNotNone(self.module, "TP01 goal publisher is missing")
        args = self.module.parse_args([
            "--goal-output", "goal.json",
            "--planner-output", "planner.csv",
            "--motion-output", "motion.csv",
            "--events-output", "events.csv",
        ])
        self.assertEqual(args.goal_dx, 0.6)
        self.assertEqual(args.goal_delay, 3.0)
        self.assertEqual(args.trajectory_duration, 20.0)
        self.assertEqual(args.total_duration, 26.0)
        self.assertEqual(args.rate, 20.0)
        self.assertEqual(args.com_height, 0.33)


if __name__ == "__main__":
    unittest.main()
