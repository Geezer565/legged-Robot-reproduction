#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "publish_myrbf_motion.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("publish_myrbf_motion", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MyRbfMotionProfileTest(unittest.TestCase):
    def test_forward_command_reaches_requested_speed_without_a_start_step(self):
        self.assertTrue(hasattr(MODULE, "speed_scale"), "start command must ramp rather than jump")
        for elapsed, expected in ((2.9, 0.0), (3.0, 0.0), (4.0, 0.5), (5.0, 1.0), (32.9, 1.0)):
            self.assertAlmostEqual(MODULE.speed_scale(elapsed, 3.0, 2.0), expected)
        values = [MODULE.speed_scale(3+i*.05, 3.0, 2.0) for i in range(41)]
        self.assertTrue(all(0 <= b-a < .04 for a,b in zip(values, values[1:])))

    def test_motion_window_has_clean_start_and_stop(self):
        self.assertFalse(MODULE.motion_is_active(1.99, 2.0, 5.0))
        self.assertTrue(MODULE.motion_is_active(2.0, 2.0, 5.0))
        self.assertTrue(MODULE.motion_is_active(6.99, 2.0, 5.0))
        self.assertFalse(MODULE.motion_is_active(7.0, 2.0, 5.0))

    def test_static_walk_template_matches_go2_configuration(self):
        event_times, mode_sequence = MODULE.gait_template("static_walk")
        self.assertEqual(event_times, [0.0, 0.3, 0.6, 0.9, 1.2])
        self.assertEqual(mode_sequence, [13, 7, 14, 11])

    def test_trot_template_matches_go2_configuration(self):
        event_times, mode_sequence = MODULE.gait_template("trot")
        self.assertEqual(event_times, [0.0, 0.3, 0.6])
        self.assertEqual(mode_sequence, [9, 6])

    def test_unknown_gait_is_rejected(self):
        with self.assertRaises(ValueError):
            MODULE.gait_template("moonwalk")

    def test_motion_arguments_are_validated(self):
        MODULE.validate_motion(2.0, 5.0, 20.0, 0.1, 0.0, 0.0)
        with self.assertRaises(ValueError):
            MODULE.validate_motion(-1.0, 5.0, 20.0, 0.1, 0.0, 0.0)
        with self.assertRaises(ValueError):
            MODULE.validate_motion(1.0, 0.0, 20.0, 0.1, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
