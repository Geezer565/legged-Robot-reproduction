#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import sys
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "apply_myrbf_wrench.py"
sys.path.insert(0, str(SCRIPT.parents[1] / "src"))
SPEC = importlib.util.spec_from_file_location("apply_myrbf_wrench", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MyRbfWrenchPhaseTest(unittest.TestCase):
    def test_phase_controls_make_low_frequency_profiles_start_at_ten_newtons(self):
        try:
            sine = MODULE.profile_value(
                0.0, "sine", 0.0, 80.0, 0.0, 10.0, 0.05,
                phase_deg=90.0,
            )
            multisine = MODULE.profile_value(
                0.0, "multisine", 0.0, 80.0, 0.0, 5.0, 0.05,
                secondary_amplitude=5.0,
                secondary_frequency=0.10,
                phase_deg=90.0,
                secondary_phase_deg=90.0,
            )
        except TypeError as error:
            self.fail("wrench profiles do not expose phase controls: %s" % error)
        self.assertAlmostEqual(sine, 10.0, places=12)
        self.assertAlmostEqual(multisine, 10.0, places=12)

    def test_shutdown_during_rate_sleep_is_handled_cleanly(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("except rospy.ROSInterruptException:", source)

    def test_world_wrench_has_no_unrequested_moment(self):
        # gazebo_ros applyBodyWrench adds reference_point x force even in
        # its world-frame branch; the point is NOT a world application pose.
        point = MODULE.world_reference_point()
        self.assertEqual(point, (0.0, 0.0, 0.0))
        force = (0.0, 5.0, 0.0)
        added_torque = (point[1]*force[2]-point[2]*force[1],
                        point[2]*force[0]-point[0]*force[2],
                        point[0]*force[1]-point[1]*force[0])
        self.assertEqual(added_torque, (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
