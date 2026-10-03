#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "record_tp01_target.py"
)


class Vector:
    def __init__(self, values):
        self.value = values


class Target:
    timeTrajectory = [3.0, 3.05]
    stateTrajectory = [
        Vector([0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                1.0, 2.0, 0.33, 0.2, 0.0, 0.0]),
        Vector([0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
                1.1, 2.0, 0.33, 0.2, 0.0, 0.0]),
    ]
    inputTrajectory = [Vector([4.0]), Vector([5.0])]


class Tp01TargetRecorderTest(unittest.TestCase):
    def test_rows_preserve_received_target_knots_and_base_reference(self):
        self.assertTrue(SCRIPT.is_file(), "TP01 target recorder is missing")
        spec = importlib.util.spec_from_file_location(
            "record_tp01_target", SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        rows = module.target_rows(Target(), received_ros_time=9.0, message_index=2)
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0], {
            "message_index": 2,
            "knot_index": 0,
            "received_ros_time": 9.0,
            "trajectory_time_s": 3.0,
            "vx_mps": 0.1,
            "vy_mps": 0.0,
            "vz_mps": 0.0,
            "x_m": 1.0,
            "y_m": 2.0,
            "z_m": 0.33,
            "yaw_rad": 0.2,
            "pitch_rad": 0.0,
            "roll_rad": 0.0,
            "state_size": 12,
            "input_size": 1,
        })
        self.assertEqual(rows[1]["x_m"], 1.1)


if __name__ == "__main__":
    unittest.main()
