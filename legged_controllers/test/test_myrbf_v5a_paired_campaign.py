#!/usr/bin/env python3

import dataclasses
import contextlib
import csv
import importlib.util
import io
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

from legged_controllers.test.test_myrbf_v5a_paired_analysis import (
    PROTOCOL_TEXT,
    load_analyzer,
)


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "run_myrbf_v5a_paired_campaign.py"
)
MYRBF_LAUNCH = SCRIPT.parents[1] / "launch" / "myrbf_shadow_gazebo.launch"
UNITREE_EMPTY_WORLD = (
    SCRIPT.parents[2]
    / "legged_examples"
    / "legged_unitree"
    / "legged_unitree_description"
    / "launch"
    / "empty_world.launch"
)


def command_arguments(command):
    return dict(argument.split(":=", 1)
                for argument in command[3:])


def write_csv(path, fieldnames, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path):
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_portable_source_lock(module, source_root, path, source_commit):
    required_files = module.campaign_schema.SOURCE_LOCK_SENTINELS
    files = {
        relative: hashlib.sha256(
            (source_root / relative).read_bytes()).hexdigest()
        for relative in required_files
    }
    lock = {
        "schema_version": module.campaign_schema.SOURCE_LOCK_VERSION,
        "source_commit": source_commit,
        "files": files,
    }
    path.write_text(json.dumps(lock, sort_keys=True) + "\n", encoding="utf-8")
    return lock


def write_valid_evidence(command, log_path):
    arguments = command_arguments(command)
    duration = float(arguments["duration"])
    elapsed_values = [0.25 * index
                      for index in range(int(duration / 0.25) + 1)]
    shadow_rows = []
    state_rows = []
    motion_rows = []
    wrench_rows = []
    for elapsed in elapsed_values:
        ros_time = 100.0 + elapsed
        motion_active = 4.0 <= elapsed < 13.0
        force_active = (
            arguments["enable_wrench"] == "true" and
            6.0 <= elapsed < 12.0
        )
        if not force_active:
            force = 0.0
        elif arguments["wrench_profile"] == "constant":
            force = 20.0
        else:
            force = 15.0 * math.sin(2.0 * math.pi * 0.5 * (elapsed - 6.0))
        shadow_rows.append({
            "campaign_start_ros": 100.0,
            "wall_elapsed": elapsed,
            "time_s": ros_time,
            "estimate_after_fy_n": -force,
            "actual_base_y_m": 0.0,
            "actual_base_roll_rad": 0.0,
            "requested_control_fy_n": -force,
            "achieved_control_fy_n": -force,
            "allocator_scale": 1.0,
            "update_frozen": 0,
            "projection_active_fx": 0, "projection_active_fy": 0,
            "projection_active_fz": 0, "projection_active_tx": 0,
            "projection_active_ty": 0, "projection_active_tz": 0,
        })
        state_rows.append({
            "ros_time": ros_time, "campaign_start_ros": 100.0,
            "wall_elapsed": elapsed,
            "base_x": 0.0, "base_y": 0.0, "base_z": 0.34,
            "observation_base_yaw": 0.0,
            "observation_base_pitch": 0.0,
            "observation_base_roll": 0.0,
            "base_vx": 0.0, "base_vy": 0.0, "base_vz": 0.0,
        })
        motion_rows.append({
            "ros_time": ros_time, "campaign_start_ros": 100.0,
            "wall_elapsed": elapsed,
            "active": 1 if motion_active else 0,
            "gait": "static_walk" if motion_active else "stance",
            "vx": 0.08 if motion_active else 0.0,
            "vy": 0.0, "yaw_rate": 0.0,
        })
        wrench_rows.append({
            "ros_time": ros_time, "campaign_start_ros": 100.0,
            "wall_elapsed": elapsed,
            "fx": 0.0, "fy": force, "fz": 0.0,
            "tx": 0.0, "ty": 0.0, "tz": 0.0,
            "service_called": 1 if force_active else 0,
            "service_success": 1,
        })
    write_csv(Path(arguments["shadow_output"]), tuple(shadow_rows[0]),
              shadow_rows)
    write_csv(Path(arguments["state_output"]), tuple(state_rows[0]), state_rows)
    write_csv(Path(arguments["motion_output"]), tuple(motion_rows[0]),
              motion_rows)
    if arguments["enable_wrench"] == "true":
        write_csv(Path(arguments["wrench_output"]), tuple(wrench_rows[0]),
                  wrench_rows)
    log_path.write_text("campaign process completed\n", encoding="utf-8")


class SuccessfulEvidenceProcess:
    def __init__(self):
        self.commands = []

    def __call__(self, command, cwd, log_path, timeout_s):
        self.commands.append(list(command))
        write_valid_evidence(command, log_path)
        return {"exit_code": 0, "timed_out": False}


class FailingProcess:
    def __init__(self):
        self.commands = []

    def __call__(self, command, cwd, log_path, timeout_s):
        self.commands.append(list(command))
        log_path.write_text("intentional incomplete attempt\n", encoding="utf-8")
        return {"exit_code": 7, "timed_out": False}


class SuccessfulThenFailingProcess:
    def __init__(self):
        self.commands = []

    def __call__(self, command, cwd, log_path, timeout_s):
        self.commands.append(list(command))
        if len(self.commands) == 1:
            write_valid_evidence(command, log_path)
            return {"exit_code": 0, "timed_out": False}
        log_path.write_text("intentional second-run failure\n", encoding="utf-8")
        return {"exit_code": 7, "timed_out": False}


class MustNotRunProcess:
    def __init__(self):
        self.commands = []

    def __call__(self, command, cwd, log_path, timeout_s):
        self.commands.append(list(command))
        raise AssertionError("process must not be launched")


class ManifestCheckingProcess:
    def __init__(self, output_root):
        self.output_root = output_root

    def __call__(self, command, cwd, log_path, timeout_s):
        manifest = json.loads(
            (self.output_root / "manifest.json").read_text(encoding="utf-8")
        )
        attempt = manifest["runs"][0]["attempt_history"][0]
        if (attempt["status"] != "in_progress" or attempt["complete"] or
                attempt["finished_at"] is not None or
                attempt["exit_code"] is not None):
            raise AssertionError("attempt was not persisted before launch")
        return {"exit_code": 7, "timed_out": False}


def load_runner():
    sys.path.insert(0, str(SCRIPT.parent))
    spec = importlib.util.spec_from_file_location(
        "run_myrbf_v5a_paired_campaign", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RunMatrixTest(unittest.TestCase):
    # Production mutation caught: omitting/duplicating a run, changing a seed,
    # or failing to alternate which method goes first on even repeats.
    def test_build_run_matrix_is_the_exact_frozen_30_run_order(self):
        module = load_runner()
        matrix = module.build_run_matrix()

        identities = [
            (item.method, item.scenario, item.repeat, item.seed, item.order)
            for item in matrix
        ]
        self.assertEqual(len(identities), 30)
        self.assertEqual(len(set(identities)), 30)
        self.assertEqual(identities[:6], [
            ("V4", "no_disturbance", 1, 4101, 1),
            ("V5-A", "no_disturbance", 1, 4101, 2),
            ("V5-A", "no_disturbance", 2, 4102, 1),
            ("V4", "no_disturbance", 2, 4102, 2),
            ("V4", "no_disturbance", 3, 4103, 1),
            ("V5-A", "no_disturbance", 3, 4103, 2),
        ])
        self.assertEqual(identities[-2:], [
            ("V4", "sine_fy", 5, 4105, 1),
            ("V5-A", "sine_fy", 5, 4105, 2),
        ])
        self.assertEqual(
            {item.config_path for item in matrix if item.method == "V4"},
            {"legged_controllers/config/go2/myrbf_shadow_v4_go2.yaml"},
        )
        self.assertEqual(
            {item.config_path for item in matrix if item.method == "V5-A"},
            {"legged_controllers/config/go2/myrbf_shadow_v5a_go2.yaml"},
        )
        with self.assertRaises(dataclasses.FrozenInstanceError):
            matrix[0].seed = 9999


class CommandContractTest(unittest.TestCase):
    # Production mutation caught: omitting or changing any launch setting,
    # using the wrong config/seed, or returning a shell command string.
    def test_command_for_is_the_exact_argument_list(self):
        module = load_runner()
        spec = next(
            item for item in module.build_run_matrix()
            if (item.method, item.scenario, item.repeat) ==
            ("V5-A", "sine_fy", 2)
        )
        output_dir = Path("/tmp/campaign/sine_fy_r02_01_v5a/attempt-001")

        self.assertEqual(module.command_for(spec, output_dir), [
            "roslaunch", "legged_controllers", "myrbf_shadow_gazebo.launch",
            "robot_type:=go2", "gui:=false", "gazebo_seed:=4102",
            "duration:=18.0",
            "state_output:=/tmp/campaign/sine_fy_r02_01_v5a/attempt-001/state.csv",
            "shadow_output:=/tmp/campaign/sine_fy_r02_01_v5a/attempt-001/shadow.csv",
            "wrench_output:=/tmp/campaign/sine_fy_r02_01_v5a/attempt-001/wrench.csv",
            "motion_output:=/tmp/campaign/sine_fy_r02_01_v5a/attempt-001/motion.csv",
            "enable_shadow:=true", "enable_control_injection:=true",
            "control_gain:=0.1", "control_force_limit:=3.0",
            "control_rate_limit:=5.0",
            "myrbf_config:=myrbf_shadow_v5a_go2.yaml",
            "enable_motion:=true", "motion_gait:=static_walk",
            "motion_vx:=0.08", "motion_vy:=0.0",
            "motion_yaw_rate:=0.0", "motion_delay:=4.0",
            "motion_duration:=9.0", "enable_wrench:=true",
            "wrench_profile:=sine", "wrench_axis:=fy",
            "wrench_value:=0.0", "wrench_amplitude:=15.0",
            "wrench_frequency:=0.5",
            "wrench_secondary_amplitude:=0.0",
            "wrench_secondary_frequency:=0.0",
            "wrench_delay:=6.0", "wrench_duration:=6.0",
        ])


class LaunchForwardingTest(unittest.TestCase):
    # Production mutation caught: declaring gazebo_seed without forwarding it,
    # or dropping extra_gazebo_args before gazebo_ros receives --seed.
    def test_actual_launch_chain_forwards_gazebo_seed(self):
        outer = ET.parse(str(MYRBF_LAUNCH)).getroot()
        outer_args = {item.attrib["name"]: item.attrib
                      for item in outer.findall("arg")}
        self.assertEqual(outer_args["gazebo_seed"]["default"], "0")
        unitree_include = next(
            item for item in outer.findall("include")
            if "legged_unitree_description" in item.attrib["file"]
        )
        forwarded = {item.attrib["name"]: item.attrib["value"]
                     for item in unitree_include.findall("arg")}
        self.assertEqual(forwarded["extra_gazebo_args"],
                         "--seed $(arg gazebo_seed)")

        inner = ET.parse(str(UNITREE_EMPTY_WORLD)).getroot()
        inner_args = {item.attrib["name"]: item.attrib
                      for item in inner.findall("arg")}
        self.assertEqual(inner_args["extra_gazebo_args"]["default"], "")
        gazebo_include = next(
            item for item in inner.findall("include")
            if "gazebo_ros" in item.attrib["file"]
        )
        gazebo_forwarded = {item.attrib["name"]: item.attrib["value"]
                            for item in gazebo_include.findall("arg")}
        self.assertEqual(gazebo_forwarded["extra_gazebo_args"],
                         "$(arg extra_gazebo_args)")


class DryRunTest(unittest.TestCase):
    # Production mutation caught: launching/creating evidence during dry-run,
    # or printing fewer, duplicated, or reordered campaign commands.
    def test_dry_run_prints_exactly_30_commands_without_writes(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            output_root = Path(directory) / "campaign-must-not-exist"
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                result = module.run_campaign(output_root, execute=False)

            self.assertEqual(result, 0)
            self.assertFalse(output_root.exists())

        commands = [shlex.split(line)
                    for line in stdout.getvalue().splitlines()]
        self.assertEqual(len(commands), 30)
        self.assertEqual(len({tuple(command) for command in commands}), 30)
        self.assertEqual(
            [next(argument for argument in command
                  if argument.startswith("gazebo_seed:="))
             for command in commands[:6]],
            ["gazebo_seed:=4101", "gazebo_seed:=4101",
             "gazebo_seed:=4102", "gazebo_seed:=4102",
             "gazebo_seed:=4103", "gazebo_seed:=4103"],
        )
        self.assertIn("myrbf_config:=myrbf_shadow_v4_go2.yaml", commands[0])
        self.assertIn("myrbf_config:=myrbf_shadow_v5a_go2.yaml", commands[1])
        self.assertIn("myrbf_config:=myrbf_shadow_v5a_go2.yaml", commands[2])
        self.assertIn("myrbf_config:=myrbf_shadow_v4_go2.yaml", commands[3])
        self.assertTrue(all(command[:3] == [
            "roslaunch", "legged_controllers", "myrbf_shadow_gazebo.launch",
        ] for command in commands))


class SuccessfulExecutionTest(unittest.TestCase):
    # Production mutation caught: executing without frozen provenance, losing
    # attempt history, or writing a manifest not independently analyzable.
    def test_execute_builds_a_complete_auditable_campaign(self):
        module = load_runner()
        source_commit = "8e7dd40b0eb085bc91b6c20fd660b0171774c8b9"
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            protocol = workspace / "source-PROTOCOL.md"
            protocol.write_text(PROTOCOL_TEXT, encoding="utf-8")
            output_root = workspace / "campaign"
            process = SuccessfulEvidenceProcess()
            source_lock = workspace / "source-lock.json"
            write_portable_source_lock(
                module, module.SOURCE_ROOT, source_lock, source_commit)

            result = module.run_campaign(
                output_root,
                execute=True,
                protocol=protocol,
                source_commit=source_commit,
                source_lock=source_lock,
                process_runner=process,
            )

            self.assertEqual(result, 0)
            self.assertEqual(len(process.commands), 30)
            manifest = json.loads(
                (output_root / "manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["schema_version"], 1)
            self.assertEqual(
                manifest["protocol_sha256"],
                "e6004dbbbc69b81fc58a8567196d46adf84c14dd2eee1b1acee656cca924c484",
            )
            self.assertEqual(len(manifest["runs"]), 30)
            self.assertEqual(
                (output_root / "PROTOCOL.md").read_text(encoding="utf-8"),
                PROTOCOL_TEXT,
            )
            first = manifest["runs"][0]
            self.assertEqual(first["git_commit"], source_commit)
            self.assertEqual(first["output_dir"],
                             "no_disturbance_r01_01_v4/attempt-001")
            self.assertEqual(len(first["attempt_history"]), 1)
            self.assertEqual(first["attempt_history"][0]["attempt"], 1)
            self.assertEqual(first["attempt_history"][0]["output_dir"],
                             first["output_dir"])
            self.assertEqual(first["exit_code"], 0)
            self.assertTrue(first["finite"])
            self.assertFalse(first["safety_stop_detected"])
            self.assertTrue(first["complete"])
            self.assertGreater(first["controller_rows"], 0)
            self.assertGreater(first["estimator_rows"], 0)
            self.assertEqual(first["observed_spans"]["controller"]["span_s"],
                             15.0)
            self.assertEqual(first["campaign_start_ros"], 100.0)
            self.assertIsNone(first["observed_spans"]["wrench"])
            self.assertTrue((output_root / first["output_dir"] /
                             "run.log").is_file())

            analyzer = load_analyzer()
            analyzer.validate_campaign(manifest, output_root)


class RunnerCompletenessTest(unittest.TestCase):
    def make_attempt(self, module, root, scenario="constant_fy"):
        spec = next(item for item in module.build_run_matrix()
                    if item.method == "V4" and
                    item.scenario == scenario and item.repeat == 1)
        attempt_dir = root / "attempt-001"
        attempt_dir.mkdir()
        log_path = attempt_dir / "run.log"
        command = module.command_for(spec, attempt_dir)
        write_valid_evidence(command, log_path)
        return spec, attempt_dir, log_path

    # Production mutation caught: checking only the force/motion window or
    # first/last samples while accepting truncated or internally gapped files.
    def test_runner_rejects_truncated_and_gap_broken_primary_streams(self):
        module = load_runner()
        mutations = (
            ("shadow.csv", "time_s", "start"),
            ("shadow.csv", "time_s", "end"),
            ("state.csv", "ros_time", "start"),
            ("state.csv", "ros_time", "end"),
            ("shadow.csv", "time_s", "gap"),
            ("state.csv", "ros_time", "gap"),
        )
        for filename, field, mutation in mutations:
            with self.subTest(filename=filename, mutation=mutation):
                with tempfile.TemporaryDirectory() as directory:
                    spec, attempt_dir, log_path = self.make_attempt(
                        module, Path(directory))
                    path = attempt_dir / filename
                    rows = read_csv(path)
                    times = [float(row[field]) for row in rows]
                    if mutation == "start":
                        boundary = times[0] + 1.25
                        rows = [row for row in rows
                                if float(row[field]) >= boundary]
                    elif mutation == "end":
                        boundary = times[-1] - 1.25
                        rows = [row for row in rows
                                if float(row[field]) <= boundary]
                    else:
                        lower = times[0] + 8.0
                        rows = [row for row in rows
                                if not lower <= float(row[field]) < lower + 2.5]
                    write_csv(path, tuple(rows[0]), rows)

                    with self.assertRaisesRegex(
                            ValueError, "full-run coverage|excessive gap"):
                        module._inspect_attempt(spec, attempt_dir, log_path)

    def test_runner_rejects_mismatched_shared_campaign_start(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            spec, attempt_dir, log_path = self.make_attempt(
                module, Path(directory))
            motion_path = attempt_dir / "motion.csv"
            rows = read_csv(motion_path)
            for row in rows:
                row["campaign_start_ros"] = "101.0"
            write_csv(motion_path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError, "campaign start"):
                module._inspect_attempt(spec, attempt_dir, log_path)

    def test_runner_rejects_nonzero_nonselected_wrench_axis(self):
        module = load_runner()
        for axis in ("fx", "fz", "tx", "ty", "tz"):
            with self.subTest(axis=axis):
                with tempfile.TemporaryDirectory() as directory:
                    spec, attempt_dir, log_path = self.make_attempt(
                        module, Path(directory))
                    wrench_path = attempt_dir / "wrench.csv"
                    rows = read_csv(wrench_path)
                    rows[len(rows) // 2][axis] = "0.25"
                    write_csv(wrench_path, tuple(rows[0]), rows)

                    with self.assertRaisesRegex(
                            ValueError, "non-selected wrench axis"):
                        module._inspect_attempt(spec, attempt_dir, log_path)


class ResumeTest(unittest.TestCase):
    SOURCE_COMMIT = "8e7dd40b0eb085bc91b6c20fd660b0171774c8b9"
    make_attempt = RunnerCompletenessTest.make_attempt

    def execution_options(self, protocol, process):
        source_lock = protocol.parent / "source-lock.json"
        module = load_runner()
        write_portable_source_lock(
            module, module.SOURCE_ROOT, source_lock, self.SOURCE_COMMIT)
        return {
            "protocol": protocol,
            "source_commit": self.SOURCE_COMMIT,
            "source_lock": source_lock,
            "process_runner": process,
        }

    # Production mutation caught: relaunching already complete runs instead of
    # safely skipping all 30 after provenance and evidence still match.
    def test_resume_skips_complete_matching_campaign(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            protocol = workspace / "PROTOCOL-source.md"
            protocol.write_text(PROTOCOL_TEXT, encoding="utf-8")
            output_root = workspace / "campaign"
            first_process = SuccessfulEvidenceProcess()
            self.assertEqual(module.run_campaign(
                output_root, True,
                **self.execution_options(protocol, first_process)), 0)
            manifest_before = (output_root / "manifest.json").read_bytes()
            second_process = MustNotRunProcess()

            result = module.run_campaign(
                output_root, True, resume=True,
                **self.execution_options(protocol, second_process))

            self.assertEqual(result, 0)
            self.assertEqual(second_process.commands, [])
            self.assertEqual((output_root / "manifest.json").read_bytes(),
                             manifest_before)

    # Production mutation caught: overwriting attempt-001 or discarding its
    # history when retrying the first incomplete run.
    def test_resume_incomplete_run_creates_attempt_002_and_preserves_old_files(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            protocol = workspace / "PROTOCOL-source.md"
            protocol.write_text(PROTOCOL_TEXT, encoding="utf-8")
            output_root = workspace / "campaign"
            failed = FailingProcess()
            self.assertEqual(module.run_campaign(
                output_root, True,
                **self.execution_options(protocol, failed)), 1)
            manifest = json.loads(
                (output_root / "manifest.json").read_text(encoding="utf-8"))
            first_attempt = output_root / manifest["runs"][0]["output_dir"]
            sentinel = first_attempt / "preserve-me.txt"
            sentinel.write_text("old evidence", encoding="utf-8")
            old_log = (first_attempt / "run.log").read_text(encoding="utf-8")
            successful = SuccessfulEvidenceProcess()

            result = module.run_campaign(
                output_root, True, resume=True,
                **self.execution_options(protocol, successful))

            self.assertEqual(result, 0)
            self.assertEqual(len(successful.commands), 30)
            self.assertEqual(sentinel.read_text(encoding="utf-8"),
                             "old evidence")
            self.assertEqual((first_attempt / "run.log").read_text(
                encoding="utf-8"), old_log)
            manifest = json.loads(
                (output_root / "manifest.json").read_text(encoding="utf-8"))
            first = manifest["runs"][0]
            self.assertEqual(first["output_dir"],
                             "no_disturbance_r01_01_v4/attempt-002")
            self.assertEqual([item["attempt"]
                              for item in first["attempt_history"]], [1, 2])
            self.assertTrue(first["complete"])
            self.assertTrue((output_root / first["output_dir"] /
                             "run.log").is_file())

    def test_resume_preserves_orphan_attempt_and_uses_next_unused_number(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            protocol = workspace / "PROTOCOL-source.md"
            protocol.write_text(PROTOCOL_TEXT, encoding="utf-8")
            output_root = workspace / "campaign"
            self.assertEqual(module.run_campaign(
                output_root, True,
                **self.execution_options(protocol, FailingProcess())), 1)
            run_dir = output_root / "no_disturbance_r01_01_v4"
            orphan = run_dir / "attempt-007"
            orphan.mkdir()
            sentinel = orphan / "orphan.txt"
            sentinel.write_text("preserve orphan", encoding="utf-8")

            self.assertEqual(module.run_campaign(
                output_root, True, resume=True,
                **self.execution_options(protocol, FailingProcess())), 1)

            manifest = json.loads(
                (output_root / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(
                manifest["runs"][0]["output_dir"],
                "no_disturbance_r01_01_v4/attempt-008",
            )
            self.assertEqual(sentinel.read_text(encoding="utf-8"),
                             "preserve orphan")

    def test_resume_adopts_orphan_attempt_directory_for_unstarted_run(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            protocol = workspace / "PROTOCOL-source.md"
            protocol.write_text(PROTOCOL_TEXT, encoding="utf-8")
            output_root = workspace / "campaign"
            self.assertEqual(module.run_campaign(
                output_root, True,
                **self.execution_options(protocol, FailingProcess())), 1)
            orphan_run = output_root / "no_disturbance_r01_02_v5a"
            orphan_attempt = orphan_run / "attempt-004"
            orphan_attempt.mkdir(parents=True)
            sentinel = orphan_attempt / "orphan.txt"
            sentinel.write_text("preserve orphan", encoding="utf-8")
            process = SuccessfulThenFailingProcess()

            result = module.run_campaign(
                output_root, True, resume=True,
                **self.execution_options(protocol, process))

            self.assertEqual(result, 1)
            manifest = json.loads(
                (output_root / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(
                manifest["runs"][1]["output_dir"],
                "no_disturbance_r01_02_v5a/attempt-005",
            )
            self.assertEqual(sentinel.read_text(encoding="utf-8"),
                             "preserve orphan")

    def test_attempt_is_written_in_progress_before_process_launch(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            protocol = workspace / "PROTOCOL-source.md"
            protocol.write_text(PROTOCOL_TEXT, encoding="utf-8")
            output_root = workspace / "campaign"

            result = module.run_campaign(
                output_root, True,
                **self.execution_options(
                    protocol, ManifestCheckingProcess(output_root)))

            self.assertEqual(result, 1)

    # Production mutation caught: launching a resume before checking all
    # protocol/source/config provenance recorded by the existing campaign.
    def test_resume_drift_aborts_before_launch(self):
        module = load_runner()
        for mutation in ("protocol_snapshot", "run_commit", "config_hash"):
            with self.subTest(mutation=mutation):
                with tempfile.TemporaryDirectory() as directory:
                    workspace = Path(directory)
                    protocol = workspace / "PROTOCOL-source.md"
                    protocol.write_text(PROTOCOL_TEXT, encoding="utf-8")
                    output_root = workspace / "campaign"
                    self.assertEqual(module.run_campaign(
                        output_root, True,
                        **self.execution_options(protocol, FailingProcess())), 1)
                    manifest_path = output_root / "manifest.json"
                    manifest = json.loads(
                        manifest_path.read_text(encoding="utf-8"))
                    if mutation == "protocol_snapshot":
                        manifest["protocol_snapshot"]["schema_version"] = 999
                    elif mutation == "run_commit":
                        manifest["runs"][0]["git_commit"] = "1" * 40
                    else:
                        manifest["runs"][0]["config_sha256"] = "0" * 64
                    manifest_path.write_text(
                        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
                    process = MustNotRunProcess()

                    with self.assertRaisesRegex(
                            ValueError, "drift|commit|configuration"):
                        module.run_campaign(
                            output_root, True, resume=True,
                            **self.execution_options(protocol, process))
                    self.assertEqual(process.commands, [])

    # Production mutation caught: treating a nonempty unrelated directory as
    # a fresh campaign root and overwriting user files.
    def test_first_execution_refuses_nonempty_root_without_overwrite(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            protocol = workspace / "PROTOCOL-source.md"
            protocol.write_text(PROTOCOL_TEXT, encoding="utf-8")
            output_root = workspace / "campaign"
            output_root.mkdir()
            sentinel = output_root / "user-file.txt"
            sentinel.write_text("preserve", encoding="utf-8")
            process = MustNotRunProcess()

            with self.assertRaisesRegex(ValueError, "nonempty"):
                module.run_campaign(
                    output_root, True,
                    **self.execution_options(protocol, process))

            self.assertEqual(process.commands, [])
            self.assertEqual(sentinel.read_text(encoding="utf-8"), "preserve")
            self.assertEqual(list(output_root.iterdir()), [sentinel])

    # Production mutation caught: accepting nonfinite controller evidence at
    # the first or last row instead of rejecting malformed usable data.
    def test_runner_rejects_malformed_csv_at_both_edges(self):
        module = load_runner()
        for edge in (0, -1):
            with self.subTest(edge=edge):
                with tempfile.TemporaryDirectory() as directory:
                    spec, attempt_dir, log_path = self.make_attempt(
                        module, Path(directory))
                    path = attempt_dir / "shadow.csv"
                    rows = read_csv(path)
                    rows[edge]["estimate_after_fy_n"] = "nan"
                    write_csv(path, tuple(rows[0]), rows)

                    with self.assertRaisesRegex(ValueError, "nonfinite"):
                        module._inspect_attempt(spec, attempt_dir, log_path)


class ProcessSupervisionTest(unittest.TestCase):
    make_attempt = RunnerCompletenessTest.make_attempt

    # Production mutation caught: terminating only roslaunch's parent process
    # and leaving a spawned descendant alive after the campaign timeout.
    def test_timeout_terminates_the_new_process_group_and_preserves_log(self):
        module = load_runner()
        script = (
            "import os,subprocess,sys,time; "
            "child=subprocess.Popen([sys.executable,'-c',"
            "'import time; time.sleep(30)']); "
            "print(os.getpid(), child.pid, flush=True); time.sleep(30)"
        )
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "run.log"
            parent_pid = None
            child_pid = None
            started = time.monotonic()
            try:
                result = module._supervise(
                    [sys.executable, "-c", script],
                    Path(directory), log_path, 0.2,
                )
                elapsed = time.monotonic() - started
                pids = log_path.read_text(encoding="utf-8").strip().split()
                parent_pid, child_pid = (int(value) for value in pids[-2:])
                status = subprocess.run(
                    ["ps", "-o", "stat=", "-p", str(child_pid)],
                    text=True, capture_output=True, check=False,
                ).stdout.strip()

                self.assertTrue(result["timed_out"])
                self.assertNotEqual(result["exit_code"], 0)
                self.assertLess(elapsed, 3.0)
                self.assertTrue(log_path.is_file())
                self.assertTrue(not status or status.startswith("Z"), status)
            finally:
                if parent_pid is not None:
                    try:
                        os.killpg(parent_pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass

    def test_interrupt_and_unexpected_error_terminate_process_group(self):
        module = load_runner()
        for failure in (KeyboardInterrupt(), RuntimeError("unexpected")):
            with self.subTest(failure=type(failure).__name__):
                process = mock.Mock(pid=4321, returncode=-15)
                process.wait.side_effect = [failure, -15]
                with tempfile.TemporaryDirectory() as directory:
                    log_path = Path(directory) / "run.log"
                    with mock.patch.object(
                            module.subprocess, "Popen", return_value=process), \
                            mock.patch.object(module.os, "killpg") as killpg:
                        with self.assertRaises(type(failure)):
                            module._supervise(
                                ["unused"], Path(directory), log_path, 10.0)
                    killpg.assert_called_once_with(4321, signal.SIGTERM)

    # Production mutation caught: ignoring either a low base-height sample or
    # a case-varied recognized safety-stop marker in the preserved log.
    def test_runner_rejects_fall_and_case_insensitive_safety_markers(self):
        module = load_runner()
        for mutation in ("fall", "orientation_marker", "controller_marker"):
            with self.subTest(mutation=mutation):
                with tempfile.TemporaryDirectory() as directory:
                    spec, attempt_dir, log_path = self.make_attempt(
                        module, Path(directory))
                    if mutation == "fall":
                        path = attempt_dir / "state.csv"
                        rows = read_csv(path)
                        rows[len(rows) // 2]["base_z"] = "0.249"
                        write_csv(path, tuple(rows[0]), rows)
                    elif mutation == "orientation_marker":
                        log_path.write_text(
                            "[sAfEtYcHeCkEr] oRiEnTaTiOn SaFeTy ChEcK FaIlEd!\n",
                            encoding="utf-8",
                        )
                    else:
                        log_path.write_text(
                            "[lEgGeD cOnTrOlLeR] sAfEtY cHeCk FaIlEd, "
                            "sToPpInG tHe CoNtRoLlEr.\n",
                            encoding="utf-8",
                        )

                    with self.assertRaisesRegex(
                            ValueError, "base height|safety-stop"):
                        module._inspect_attempt(spec, attempt_dir, log_path)


class PortableSourceLockTest(unittest.TestCase):
    SOURCE_COMMIT = "8e7dd40b0eb085bc91b6c20fd660b0171774c8b9"

    PREVIOUSLY_OMITTED = (
        "legged_controllers/include/legged_controllers/SafetyChecker.h",
        "legged_controllers/legged_controllers_plugins.xml",
        "legged_common/scripts/generate_urdf.sh",
        "README.md",
    )

    def make_tracked_repository(self, module, source_root):
        required = module.campaign_schema.SOURCE_LOCK_SENTINELS
        tracked = tuple(dict.fromkeys((*required, *self.PREVIOUSLY_OMITTED)))
        for relative in tracked:
            path = source_root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("tracked: %s\n" % relative, encoding="utf-8")
        subprocess.run(["git", "init"], cwd=source_root, check=True,
                       capture_output=True)
        subprocess.run(["git", "add", "."], cwd=source_root, check=True)
        subprocess.run([
            "git", "-c", "user.name=Test", "-c",
            "user.email=test@example.com", "commit", "-m", "initial",
        ], cwd=source_root, check=True, capture_output=True)
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=source_root,
            text=True).strip()
        return commit, tracked

    def test_required_sentinels_cover_every_execution_category(self):
        module = load_runner()
        expected = {
            "legged_controllers/CMakeLists.txt",
            "legged_controllers/setup.py",
            "legged_controllers/src/legged_controllers/__init__.py",
            "legged_controllers/src/legged_controllers/myrbf_campaign_timing.py",
            "legged_controllers/src/LeggedController.cpp",
            "legged_controllers/include/legged_controllers/SafetyChecker.h",
            "legged_controllers/legged_controllers_plugins.xml",
            "legged_common/scripts/generate_urdf.sh",
            "legged_controllers/launch/myrbf_shadow_gazebo.launch",
            "legged_controllers/scripts/record_stage8c.py",
            "legged_controllers/scripts/record_myrbf_shadow.py",
            "legged_controllers/scripts/run_myrbf_v5a_paired_campaign.py",
            "legged_controllers/scripts/analyze_myrbf_v5a_paired.py",
            "legged_controllers/config/go2/myrbf_shadow_v5a_go2.yaml",
        }
        self.assertTrue(expected.issubset(
            set(module.campaign_schema.SOURCE_LOCK_SENTINELS)))

    def test_git_lock_creation_hashes_exact_full_tracked_inventory(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            source_root = Path(directory)
            commit, tracked = self.make_tracked_repository(
                module, source_root)

            source_lock = module.create_source_lock(source_root, commit)

            self.assertEqual(set(source_lock["files"]), set(tracked))
            for relative in self.PREVIOUSLY_OMITTED:
                self.assertIn(relative, source_lock["files"])

    def test_git_lock_creation_rejects_dirty_previously_omitted_tracked_file(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            source_root = Path(directory)
            commit, _tracked = self.make_tracked_repository(
                module, source_root)
            dirty = source_root / self.PREVIOUSLY_OMITTED[0]
            dirty.write_text("dirty\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "dirty tracked"):
                module.create_source_lock(source_root, commit)

    def test_portable_lock_rejects_nonmapping_file_table_cleanly(self):
        module = load_runner()
        malformed = {
            "schema_version": module.campaign_schema.SOURCE_LOCK_VERSION,
            "source_commit": self.SOURCE_COMMIT,
            "files": list(module.campaign_schema.SOURCE_LOCK_SENTINELS),
        }
        with self.assertRaisesRegex(ValueError, "source-lock schema"):
            module._verify_source_lock(module.SOURCE_ROOT, malformed)

    def test_portable_lock_requires_every_sentinel(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            source_root = Path(directory)
            for relative in module.campaign_schema.SOURCE_LOCK_SENTINELS:
                path = source_root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("locked: %s\n" % relative, encoding="utf-8")
            lock_path = source_root / "source-lock.json"
            source_lock = write_portable_source_lock(
                module, source_root, lock_path, self.SOURCE_COMMIT)
            missing = module.campaign_schema.SOURCE_LOCK_SENTINELS[-1]
            del source_lock["files"][missing]

            with self.assertRaisesRegex(ValueError, "source-lock schema"):
                module._verify_source_lock(source_root, source_lock)

    def test_execution_uses_portable_lock_without_resolving_git(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            source_root = workspace / "portable-source"
            for relative in module.campaign_schema.SOURCE_LOCK_SENTINELS:
                path = source_root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("locked: %s\n" % relative, encoding="utf-8")
            protocol = workspace / "PROTOCOL.md"
            protocol.write_text(PROTOCOL_TEXT, encoding="utf-8")
            source_lock = workspace / "source-lock.json"
            write_portable_source_lock(
                module, source_root, source_lock, self.SOURCE_COMMIT)
            with mock.patch.object(
                    module.subprocess, "check_output",
                    side_effect=AssertionError("Git must not be resolved")):
                result = module.run_campaign(
                    workspace / "campaign", True,
                    protocol=protocol,
                    source_commit=self.SOURCE_COMMIT,
                    source_lock=source_lock,
                    source_root=source_root,
                    process_runner=SuccessfulEvidenceProcess(),
                )
            self.assertEqual(result, 0)
            manifest = json.loads(
                (workspace / "campaign" / "manifest.json").read_text(
                    encoding="utf-8"))
            self.assertEqual(manifest["source_lock"]["source_commit"],
                             self.SOURCE_COMMIT)
            self.assertTrue(set(
                module.campaign_schema.SOURCE_LOCK_SENTINELS
            ).issubset(manifest["source_lock"]["files"]))

    def test_execution_rechecks_every_supplied_hash_before_each_attempt(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            source_root = workspace / "portable-source"
            source_root.mkdir()
            critical = source_root / "critical.txt"
            critical.write_text("locked\n", encoding="utf-8")
            extra = source_root / "extra.txt"
            extra.write_text("also locked\n", encoding="utf-8")
            protocol = workspace / "PROTOCOL.md"
            protocol.write_text(PROTOCOL_TEXT, encoding="utf-8")
            source_lock = workspace / "source-lock.json"

            class DriftAfterFirstAttempt(SuccessfulEvidenceProcess):
                def __call__(self, command, cwd, log_path, timeout_s):
                    result = super().__call__(command, cwd, log_path, timeout_s)
                    extra.write_text("drifted\n", encoding="utf-8")
                    return result

            with mock.patch.object(
                    module.campaign_schema, "SOURCE_LOCK_SENTINELS",
                    ("critical.txt",)):
                write_portable_source_lock(
                    module, source_root, source_lock, self.SOURCE_COMMIT)
                source_lock_data = json.loads(source_lock.read_text(
                    encoding="utf-8"))
                source_lock_data["files"]["extra.txt"] = hashlib.sha256(
                    extra.read_bytes()).hexdigest()
                source_lock.write_text(
                    json.dumps(source_lock_data, sort_keys=True) + "\n",
                    encoding="utf-8",
                )
                process = DriftAfterFirstAttempt()
                with self.assertRaisesRegex(ValueError, "source-lock drift"):
                    module.run_campaign(
                        workspace / "campaign", True,
                        protocol=protocol,
                        source_commit=self.SOURCE_COMMIT,
                        source_lock=source_lock,
                        source_root=source_root,
                        process_runner=process,
                    )
            self.assertEqual(len(process.commands), 1)


class CommandLineTest(unittest.TestCase):
    # Production mutation caught: allowing ambiguous execution mode, implicit
    # execution provenance, or resume on a dry-run.
    def test_cli_requires_explicit_safe_execution_arguments(self):
        module = load_runner()
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                module.parse_args(["--execute"])
            with self.assertRaises(SystemExit):
                module.parse_args(["--execute", "--protocol", "PROTOCOL.md"])
            with self.assertRaises(SystemExit):
                module.parse_args([
                    "--execute", "--source-commit", "1" * 40,
                ])
            with self.assertRaises(SystemExit):
                module.parse_args([
                    "--execute", "--protocol", "PROTOCOL.md",
                    "--source-commit", "1" * 40,
                ])
            with self.assertRaises(SystemExit):
                module.parse_args(["--dry-run", "--resume"])
            with self.assertRaises(SystemExit):
                module.parse_args(["--dry-run", "--execute"])

        args = module.parse_args([
            "--execute", "--resume", "--protocol", "PROTOCOL.md",
            "--source-commit", "8e7dd40b0eb085bc91b6c20fd660b0171774c8b9",
            "--source-lock", "source-lock.json",
        ])
        self.assertTrue(args.execute)
        self.assertTrue(args.resume)
        self.assertEqual(args.output_root,
                         Path("/tmp/myrbf_v5a_repeat_20260901"))

    # Production mutation caught: the CLI dry-run writing files or printing a
    # different command count than the direct run_campaign interface.
    def test_cli_dry_run_prints_30_commands_without_files(self):
        module = load_runner()
        with tempfile.TemporaryDirectory() as directory:
            output_root = Path(directory) / "cli-campaign"
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                result = module.main([
                    "--dry-run", "--output-root", str(output_root),
                ])
            self.assertEqual(result, 0)
            self.assertEqual(len(stdout.getvalue().splitlines()), 30)
            self.assertFalse(output_root.exists())


if __name__ == "__main__":
    unittest.main()
