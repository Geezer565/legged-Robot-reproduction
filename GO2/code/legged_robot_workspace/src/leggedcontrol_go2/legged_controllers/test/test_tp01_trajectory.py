#!/usr/bin/env python3

import importlib
import math
import unittest


class Tp01TrajectoryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.module = importlib.import_module(
                "legged_controllers.tp01_trajectory")
        except ModuleNotFoundError:
            cls.module = None

    def test_builds_241_knot_gate_reference_with_bounded_speed(self):
        self.assertIsNotNone(
            self.module, "TP01 trajectory module has not been implemented")
        reference = self.module.build_quintic_reference(
            start_pose=(1.0, -0.2, 0.33, 0.1, 0.0, 0.0),
            goal_pose=(1.6, -0.2, 0.33, 0.1, 0.0, 0.0),
            start_time=7.0,
            duration=12.0,
            rate_hz=20.0,
        )

        self.assertEqual(len(reference), 241)
        self.assertEqual(reference[0].time, 7.0)
        self.assertAlmostEqual(reference[-1].time, 19.0, places=12)
        self.assertEqual(reference[0].pose, (1.0, -0.2, 0.33, 0.1, 0.0, 0.0))
        self.assertEqual(reference[-1].pose, (1.6, -0.2, 0.33, 0.1, 0.0, 0.0))
        self.assertAlmostEqual(reference[120].pose[0], 1.3, places=12)
        self.assertAlmostEqual(
            max(math.hypot(point.velocity[0], point.velocity[1])
                for point in reference),
            0.09375,
            places=10,
        )
        self.assertTrue(all(
            left.time < right.time
            for left, right in zip(reference, reference[1:])))

    def test_endpoint_velocity_and_acceleration_are_zero(self):
        self.assertIsNotNone(
            self.module, "TP01 trajectory module has not been implemented")
        reference = self.module.build_quintic_reference(
            start_pose=(0.0, 0.0, 0.33, 0.0, 0.0, 0.0),
            goal_pose=(0.6, 0.2, 0.33, 0.4, 0.0, 0.0),
            start_time=0.0,
            duration=12.0,
            rate_hz=20.0,
        )
        for endpoint in (reference[0], reference[-1]):
            self.assertTrue(all(abs(value) < 1e-12 for value in endpoint.velocity))
            self.assertTrue(all(abs(value) < 1e-12 for value in endpoint.acceleration))

    def test_yaw_uses_shortest_wrapped_displacement(self):
        self.assertIsNotNone(
            self.module, "TP01 trajectory module has not been implemented")
        reference = self.module.build_quintic_reference(
            start_pose=(0.0, 0.0, 0.33, 3.10, 0.0, 0.0),
            goal_pose=(0.0, 0.0, 0.33, -3.10, 0.0, 0.0),
            start_time=0.0,
            duration=2.0,
            rate_hz=20.0,
        )
        travelled = reference[-1].pose[3] - reference[0].pose[3]
        self.assertAlmostEqual(travelled, 2.0 * math.pi - 6.20, places=12)
        self.assertLess(abs(travelled), 0.1)

    def test_rejects_nonpositive_duration_or_rate(self):
        self.assertIsNotNone(
            self.module, "TP01 trajectory module has not been implemented")
        kwargs = dict(
            start_pose=(0.0,) * 6,
            goal_pose=(1.0,) + (0.0,) * 5,
            start_time=0.0,
            duration=12.0,
            rate_hz=20.0,
        )
        for name, value in (("duration", 0.0), ("rate_hz", 0.0)):
            invalid = dict(kwargs)
            invalid[name] = value
            with self.subTest(name=name):
                with self.assertRaises(ValueError):
                    self.module.build_quintic_reference(**invalid)

    def test_relative_goal_uses_world_xy_and_preserves_safe_body_pose(self):
        self.assertIsNotNone(
            self.module, "TP01 trajectory module has not been implemented")
        self.assertTrue(hasattr(self.module, "make_relative_goal"))
        goal = self.module.make_relative_goal(
            start_pose=(1.2, -0.4, 0.31, 0.25, 0.03, -0.02),
            dx=0.6,
            dy=0.1,
            dyaw=-0.2,
            com_height=0.33,
        )
        for actual, expected in zip(
                goal, (1.8, -0.3, 0.33, 0.05, 0.0, 0.0)):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_reference_state_has_commanded_world_velocity_pose_and_joint_targets(self):
        self.assertIsNotNone(
            self.module, "TP01 trajectory module has not been implemented")
        self.assertTrue(hasattr(self.module, "reference_state_values"))
        point = self.module.ReferencePoint(
            time=2.0,
            pose=(0.1, 0.2, 0.33, 0.4, 0.0, 0.0),
            velocity=(0.05, -0.02, 0.0, 0.1, 0.0, 0.0),
            acceleration=(0.0,) * 6,
        )
        template = tuple(float(index) for index in range(24))
        state = self.module.reference_state_values(point, template)
        self.assertEqual(len(state), 24)
        self.assertEqual(state[0:6], (0.05, -0.02, 0.0, 0.0, 0.0, 0.0))
        self.assertEqual(state[6:12], point.pose)
        self.assertEqual(state[12:24], template[12:24])

    def test_target_payload_keeps_every_planner_knot_and_input_dimension(self):
        self.assertTrue(hasattr(self.module, "build_target_payload"))
        reference = self.module.build_quintic_reference(
            start_pose=(0.0, 0.0, 0.33, 0.0, 0.0, 0.0),
            goal_pose=(0.6, 0.0, 0.33, 0.0, 0.0, 0.0),
            start_time=3.0,
            duration=12.0,
            rate_hz=20.0,
        )
        payload = self.module.build_target_payload(
            reference=reference,
            template_state=tuple(float(index) for index in range(24)),
            input_size=24,
        )
        self.assertEqual(len(payload["times"]), 241)
        self.assertEqual(len(payload["states"]), 241)
        self.assertEqual(len(payload["inputs"]), 241)
        self.assertEqual(payload["times"][0], 3.0)
        self.assertEqual(payload["times"][-1], 15.0)
        self.assertEqual(payload["states"][120][6], 0.3)
        self.assertAlmostEqual(payload["states"][120][0], 0.09375)
        self.assertEqual(payload["states"][120][1:6], (0.0,) * 5)
        self.assertEqual(payload["inputs"][0], (0.0,) * 24)

    def test_planner_rows_include_reference_derivatives_not_integrated_targets(self):
        self.assertTrue(hasattr(self.module, "planner_rows"))
        reference = self.module.build_quintic_reference(
            start_pose=(0.0, 0.0, 0.33, 0.0, 0.0, 0.0),
            goal_pose=(0.6, 0.0, 0.33, 0.0, 0.0, 0.0),
            start_time=3.0,
            duration=12.0,
            rate_hz=20.0,
        )
        rows = self.module.planner_rows(reference, start_elapsed_s=3.0)
        self.assertEqual(rows[0]["trajectory_time_s"], 3.0)
        self.assertEqual(rows[0]["campaign_elapsed_s"], 3.0)
        self.assertEqual(rows[120]["x_m"], 0.3)
        self.assertAlmostEqual(rows[120]["vx_mps"], 0.09375, places=10)
        self.assertIn("ax_mps2", rows[120])
        self.assertIn("yaw_rate_radps", rows[120])

    def test_single_bend_has_requested_extent_and_smooth_endpoints(self):
        reference = self.module.build_single_bend_reference(
            start_pose=(1.0, -0.2, 0.33, 0.0, 0.0, 0.0),
            forward_length=1.2, lateral_amplitude=0.15,
            start_time=3.0, duration=30.0, rate_hz=20.0)
        self.assertEqual(len(reference), 601)
        self.assertAlmostEqual(reference[0].pose[0], 1.0)
        self.assertAlmostEqual(reference[-1].pose[0], 2.2)
        self.assertAlmostEqual(reference[0].pose[1], -0.2)
        self.assertAlmostEqual(reference[-1].pose[1], -0.2)
        self.assertAlmostEqual(max(point.pose[1] for point in reference), -0.05, places=4)
        self.assertAlmostEqual(min(point.pose[1] for point in reference), -0.2, places=4)
        self.assertAlmostEqual(reference[len(reference) // 2].pose[1], -0.05)
        self.assertTrue(all(point.pose[1] >= -0.2 - 1e-10 for point in reference))
        for endpoint in (reference[0], reference[-1]):
            self.assertTrue(all(abs(value) < 1e-10 for value in endpoint.velocity))
            self.assertAlmostEqual(endpoint.pose[3], 0.0, places=10)

    def test_two_metre_cruise_has_smooth_three_second_ramps(self):
        reference = self.module.build_cruise_reference(
            start_pose=(0.0, 0.0, 0.33, 0.0, 0.0, 0.0),
            goal_pose=(2.0, 0.0, 0.33, 0.0, 0.0, 0.0),
            start_time=3.0, duration=25.0, rate_hz=20.0,
            accel_duration=3.0)
        self.assertEqual(len(reference), 501)
        self.assertAlmostEqual(reference[250].pose[0], 1.0)
        self.assertAlmostEqual(reference[250].velocity[0], 2.0 / 22.0)
        self.assertAlmostEqual(reference[60].velocity[0], 2.0 / 22.0)
        self.assertAlmostEqual(reference[440].velocity[0], 2.0 / 22.0)
        self.assertAlmostEqual(reference[0].velocity[0], 0.0)
        self.assertAlmostEqual(reference[-1].velocity[0], 0.0)
        self.assertAlmostEqual(reference[-1].pose[0], 2.0)
        self.assertTrue(all(left.pose[0] <= right.pose[0] for left, right in
                            zip(reference, reference[1:])))

    def test_long_single_bend_uses_same_cruise_speed(self):
        reference = self.module.build_single_bend_reference(
            start_pose=(0.0, 0.0, 0.33, 0.0, 0.0, 0.0),
            forward_length=2.0, lateral_amplitude=0.08,
            start_time=3.0, duration=25.0, rate_hz=20.0,
            accel_duration=3.0)
        self.assertAlmostEqual(reference[250].pose[0], 1.0)
        self.assertAlmostEqual(reference[250].pose[1], 0.08)
        self.assertAlmostEqual(reference[250].velocity[0], 2.0 / 22.0)
        self.assertAlmostEqual(reference[0].pose[1], 0.0)
        self.assertAlmostEqual(reference[-1].pose[1], 0.0, places=12)

    def test_terminal_hold_extends_target_support(self):
        reference = self.module.build_quintic_reference(
            start_pose=(0.0, 0.0, 0.33, 0.0, 0.0, 0.0),
            goal_pose=(0.6, 0.0, 0.33, 0.0, 0.0, 0.0),
            start_time=3.0, duration=20.0, rate_hz=20.0)
        held = self.module.append_terminal_hold(reference, 5.0)
        self.assertAlmostEqual(held[-1].time, 28.0)
        self.assertEqual(held[-1].pose, reference[-1].pose)
        self.assertEqual(held[-1].velocity, (0.0,) * 6)


if __name__ == "__main__":
    unittest.main()
