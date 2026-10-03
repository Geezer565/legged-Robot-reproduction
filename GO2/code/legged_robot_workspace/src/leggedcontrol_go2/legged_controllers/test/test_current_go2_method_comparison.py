#!/usr/bin/env python3
"""Contract tests for the current T01/T02 Go2 comparison campaign."""
from pathlib import Path
import math
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "legged_controllers/scripts"
sys.path.insert(0, str(SCRIPTS))

import run_go2_method_comparison as runner
import plot_go2_method_comparison as plotter


class CurrentComparisonContractTests(unittest.TestCase):
    def test_formal_method_set_and_public_names_are_frozen(self):
        self.assertEqual(
            runner.METHODS,
            ("v5_residual_dynamics", "eth", "liu_van_mpc_port", "zhou_ssi"),
        )
        self.assertEqual(
            {method: plotter.METHOD_LABELS[method] for method in runner.METHODS},
            {
                "v5_residual_dynamics": "MyRBF V5",
                "eth": "ETH",
                "liu_van_mpc_port": "Liu-VAN",
                "zhou_ssi": "Zhou-RFF",
            },
        )

    def test_full_plan_is_four_methods_by_eight_trajectory_conditions(self):
        rows = runner.run_matrix(runner.parse_args(["--study", "full"]))
        self.assertEqual(len(rows), 32)
        self.assertEqual({row["method"] for row in rows}, set(runner.METHODS))
        self.assertEqual({row["disturbance"]["profile"] for row in rows},
                         {"none", "constant", "sine", "multisine"})
        self.assertEqual({row["experiment"] for row in rows},
                         {"GO2-METHOD-COMPARISON"})
        conditions = {(row["planner"]["trajectory_id"],
                       row["disturbance"]["profile"]) for row in rows}
        self.assertEqual(conditions, {
            (trajectory, profile)
            for trajectory in ("t01", "t02")
            for profile in ("none", "constant", "sine", "multisine")})
        self.assertEqual({row["planner"]["relative_dx_m"] for row in rows}, {2.0})
        self.assertEqual({row["timing_s"]["trajectory"] for row in rows}, {25.0})
        self.assertEqual({row["planner"]["accel_duration_s"] for row in rows}, {3.0})

    def test_each_condition_runs_exactly_once(self):
        with self.assertRaises(SystemExit):
            runner.parse_args(["--repetitions", "2"])

    def test_removed_methods_are_not_accepted_by_new_entry(self):
        for removed in ("v2_lyapunov", "v3_teacher", "v4_convergence",
                        "neural_fly_constant"):
            with self.assertRaises(SystemExit):
                runner.parse_args(["--method", removed])

    def test_new_launch_path_is_used(self):
        case = runner.run_matrix(runner.parse_args([
            "--method", "v5_residual_dynamics", "--trajectory", "t02",
            "--profile", "sine"]))[0]
        command = runner.command_for(case, Path("/case"), Path("/config.yaml"))
        self.assertEqual(command[2], "go2_method_comparison.launch")
        self.assertIn("trajectory_id:=t02", command)
        self.assertIn("accel_duration:=3.0", command)
        self.assertTrue(any(part.endswith("myrbf_flat_control.world") for part in command))

    def test_v5_is_selectable_with_the_residual_dynamics_config(self):
        args = runner.parse_args([
            "--method", "v5_residual_dynamics", "--profile", "sine"])
        case = runner.run_matrix(args)[0]
        source = runner.config_source_for_case(args, case)
        frozen = runner.frozen_config(source, case["method"], "smooth")
        self.assertEqual(frozen["algorithm"], "v5_residual_dynamics")
        self.assertEqual(frozen["learning_target"],
                         "filtered_compensation_aware_rate_residual")
        self.assertEqual(frozen["representation"],
                         "factorized_motion_history_contact_rbf")
        self.assertEqual(case["method_definition"]["representation"],
                         frozen["representation"])
        self.assertEqual(case["method_definition"]["update"],
                         frozen["learning_target"])
        self.assertEqual(len(frozen["v5_history_scale"]), 12)
        self.assertGreater(frozen["v5_residual_filter_tau"], 0.0)

    def test_every_formal_method_has_a_valid_current_config(self):
        args = runner.parse_args([])
        for method in runner.METHODS:
            case = runner.run_matrix(runner.parse_args([
                "--method", method, "--profile", "constant"]))[0]
            source = runner.config_source_for_case(args, case)
            self.assertTrue(source.is_file(), source)
            frozen = runner.frozen_config(source, method, "smooth")
            self.assertEqual(runner.METHOD_DEFINITIONS[method]["public_name"],
                             plotter.METHOD_LABELS[method])
            self.assertIsInstance(frozen, dict)

    def test_plotter_outputs_only_the_four_required_figure_groups(self):
        with tempfile.TemporaryDirectory(prefix="go2_v4_comparison_plot_") as tmp:
            root = Path(tmp)
            output = root / "figures"
            output.mkdir()
            cases = []
            times = [i * 33.0 / 60.0 for i in range(61)]
            for method in runner.METHODS:
                path = root / method
                path.mkdir()
                (path / "frozen").mkdir()
                (path / "frozen/source_sha256.json").write_text("{}", encoding="utf-8")
                manifest = runner.run_matrix(runner.parse_args([
                    "--method", method, "--profile", "constant"]))[0]
                manifest["data_kind"] = "synthetic_test_fixture"
                state = []
                wrench = []
                estimates = []
                targets = []
                for index, elapsed in enumerate(times):
                    x_target = 2.0 * min(1.0, max(0.0, (elapsed - 3.0) / 25.0))
                    state.append({
                        "wall_elapsed": str(elapsed), "ros_time": str(100.0 + elapsed),
                        "observation_time": str(100.0 + elapsed),
                        "observation_base_x": str(x_target + 0.002 * math.sin(elapsed)),
                        "observation_base_y": str(0.001 * math.cos(elapsed)),
                        "observation_base_z": "0.33", "observation_base_yaw": "0",
                        "observation_base_pitch": "0", "observation_base_roll": "0",
                        "base_vx": "0.08", "base_vy": "0", "base_vz": "0",
                    })
                    wrench.append({"wall_elapsed": str(elapsed), "fx": "0", "fy": "5",
                                   "fz": "0", "tx": "0", "ty": "0", "tz": "0"})
                    estimate_values = [0.0, 4.9, 0.0, 0.0, 0.0, 0.0]
                    if method in plotter.ADAPTIVE_METHODS:
                        estimates.append(dict(
                            wall_elapsed=str(elapsed),
                            **{field: str(value) for field, value in zip(
                                plotter.ADAPTIVE_EXTERNAL_FIELDS, estimate_values)}))
                    else:
                        estimates.append(dict(
                            wall_elapsed=str(elapsed),
                            **{field: str(-value) for field, value in zip(
                                plotter.SHADOW_ESTIMATE_FIELDS, estimate_values)}))
                    targets.append({
                        "message_index": "0", "received_ros_time": "100",
                        "trajectory_time_s": str(100.0 + elapsed),
                        "x_m": str(x_target), "y_m": "0", "z_m": "0.33",
                        "yaw_rad": "0", "pitch_rad": "0", "roll_rad": "0",
                        "vx_mps": "0.08", "vy_mps": "0", "vz_mps": "0",
                    })
                stream = method if method in plotter.ADAPTIVE_METHODS else "shadow"
                cases.append({
                    "path": path, "manifest": manifest,
                    "validation": {"valid": True, "errors": []},
                    "data": {
                        "state": {"rows": state}, "wrench": {"rows": wrench},
                        stream: {"rows": estimates}, "mpc_target": {"rows": targets},
                    },
                })
            stems, metrics = plotter.plot_group(cases, output, "E01", dpi=72)
            self.assertEqual(len(stems), 4)
            self.assertTrue(any(item.endswith("_top_view") for item in stems))
            self.assertTrue(any(item.endswith("_base_state") for item in stems))
            self.assertTrue(any(item.endswith("_base_rates") for item in stems))
            self.assertTrue(any(item.endswith("_path_error") for item in stems))
            self.assertEqual(len(metrics), 4)
            self.assertEqual(len(list(output.glob("*.png"))), 4)
            self.assertEqual(len(list(output.glob("*.svg"))), 4)


if __name__ == "__main__":
    unittest.main()
