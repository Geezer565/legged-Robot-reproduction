#!/usr/bin/env python3

import csv
import contextlib
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "analyze_myrbf_v5a_paired.py"
)
SCHEMA_SCRIPT = SCRIPT.with_name("myrbf_v5a_campaign_schema.py")


def load_analyzer():
    sys.path.insert(0, str(SCRIPT.parent))
    spec = importlib.util.spec_from_file_location(
        "analyze_myrbf_v5a_paired", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_schema():
    spec = importlib.util.spec_from_file_location(
        "myrbf_v5a_campaign_schema", SCHEMA_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PROTOCOL_TEXT = """# MyRBF V5-A Repeat Evidence Protocol

## Frozen execution identity

- Source worktree: `C:/Users/lenovo/Documents/GO2/work/myrbf_v5a_repeat_20260901`
- Frozen candidate: `codex/myrbf-v5a-safe-update @ 51b4041`
- Execution branch (verified): `codex/myrbf-v5a-repeat-gate @ 51b4041`
- Evidence root: `C:/Users/lenovo/Documents/GO2/results/myrbf_v5a_repeat_20260901`
- Locked V4 config: `legged_controllers/config/go2/myrbf_shadow_v4_go2.yaml`
- Locked V5-A config: `legged_controllers/config/go2/myrbf_shadow_v5a_go2.yaml`
- Reference single-run report: `C:/Users/lenovo/Documents/GO2/results/myrbf_v5a_20260901/RESULT_SUMMARY.md`

The candidate, protocol, and acceptance gates are frozen before any new data is run. No parameter changes are allowed during the campaign. The two locked configuration files are identified by the SHA-256 values in `locked_inputs.sha256`.

## Experiment matrix

Each scenario has five paired repeats. Use one seed per pair:

```text
no_disturbance: duration=15 s, vx=0.08 m/s, force=0 N
constant_fy:    duration=18 s, vx=0.08 m/s, Fy=+20 N, onset=6 s, force_duration=6 s
sine_fy:        duration=18 s, vx=0.08 m/s, Fy_amplitude=15 N, frequency=0.5 Hz, onset=6 s, force_duration=6 s
```

For repeats 1, 3, and 5 run `V4 → V5-A`; for repeats 2 and 4 run `V5-A → V4`. Use seeds `4101` through `4105`, one seed per pair.

### Paired-repeat fairness lock

For every pair, the V4 run and the V5-A run must use the same initial state, applied external force, duration, control settings, and random seed. These values must remain identical within the pair; only the controller under test and the prescribed execution order may differ. The scenario parameters above and the per-pair seed define the shared conditions for both runs.

### V5-B blocking condition

V5-A repeat gate must pass before V5-B may be enabled. V5-B is prohibited for the campaign until the V5-A repeat acceptance gates below have passed.

## Acceptance gates

### Safety gate, every run

- process exits successfully; required CSV files are complete and finite
- no fall or safety stop; minimum base height >= 0.25 m
- estimator freeze rate < 5%; no sustained allocator saturation longer than 0.25 s

### No-disturbance gate, V5-A median relative to V4

- onset-relative lateral RMS <= 1.05 x V4
- roll RMS <= 1.05 x V4
- applied compensation RMS <= 3.0 N

### Sinusoidal-force gate, paired median

- estimation RMSE <= 1.10 x V4
- onset-relative lateral RMS improves by at least 10%
- roll RMS <= 1.05 x V4

### Constant-force gate, paired median

- estimation RMSE <= 1.10 x V4
- onset-relative lateral RMS <= 1.05 x V4
- roll RMS <= 1.05 x V4

Five repeats are a development gate, not a paper-level statistical claim.
"""


def write_csv(path, fieldnames, rows):
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path):
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def paired_conditions_fixture(scenario, seed):
    scenarios = {
        "no_disturbance": {
            "duration_s": 15.0, "gait": "static_walk",
            "motion_delay_s": 4.0, "motion_duration_s": 9.0,
            "vx_mps": 0.08, "vy_mps": 0.0, "yaw_rate_rps": 0.0,
            "wrench_required": False, "force_axis": "fy",
            "force_profile": "none", "force_value_n": 0.0,
            "force_amplitude_n": 0.0, "force_frequency_hz": 0.0,
            "force_onset_s": None, "force_duration_s": 0.0,
        },
        "constant_fy": {
            "duration_s": 18.0, "gait": "static_walk",
            "motion_delay_s": 4.0, "motion_duration_s": 9.0,
            "vx_mps": 0.08, "vy_mps": 0.0, "yaw_rate_rps": 0.0,
            "wrench_required": True, "force_axis": "fy",
            "force_profile": "constant", "force_value_n": 20.0,
            "force_amplitude_n": 0.0, "force_frequency_hz": 0.0,
            "force_onset_s": 6.0, "force_duration_s": 6.0,
        },
        "sine_fy": {
            "duration_s": 18.0, "gait": "static_walk",
            "motion_delay_s": 4.0, "motion_duration_s": 9.0,
            "vx_mps": 0.08, "vy_mps": 0.0, "yaw_rate_rps": 0.0,
            "wrench_required": True, "force_axis": "fy",
            "force_profile": "sine", "force_value_n": 0.0,
            "force_amplitude_n": 15.0, "force_frequency_hz": 0.5,
            "force_onset_s": 6.0, "force_duration_s": 6.0,
        },
    }
    result = {"scenario": scenario, "seed": seed}
    result.update(scenarios[scenario])
    return result


def launch_command_fixture(method, scenario, seed, run_dir):
    scenarios = {
        "no_disturbance": {
            "duration": "15.0", "enable_wrench": "false",
            "wrench_profile": "none", "wrench_value": "0.0",
            "wrench_amplitude": "0.0", "wrench_frequency": "0.0",
            "wrench_delay": "0.0", "wrench_duration": "0.0",
        },
        "constant_fy": {
            "duration": "18.0", "enable_wrench": "true",
            "wrench_profile": "constant", "wrench_value": "20.0",
            "wrench_amplitude": "0.0", "wrench_frequency": "0.0",
            "wrench_delay": "6.0", "wrench_duration": "6.0",
        },
        "sine_fy": {
            "duration": "18.0", "enable_wrench": "true",
            "wrench_profile": "sine", "wrench_value": "0.0",
            "wrench_amplitude": "15.0", "wrench_frequency": "0.5",
            "wrench_delay": "6.0", "wrench_duration": "6.0",
        },
    }
    scenario_args = scenarios[scenario]
    config = {
        "V4": "myrbf_shadow_v4_go2.yaml",
        "V5-A": "myrbf_shadow_v5a_go2.yaml",
    }[method]
    return [
        "roslaunch", "legged_controllers", "myrbf_shadow_gazebo.launch",
        "robot_type:=go2", "gui:=false", "gazebo_seed:=%d" % seed,
        "duration:=" + scenario_args["duration"],
        "state_output:=" + str(run_dir / "state.csv"),
        "shadow_output:=" + str(run_dir / "shadow.csv"),
        "wrench_output:=" + str(run_dir / "wrench.csv"),
        "motion_output:=" + str(run_dir / "motion.csv"),
        "enable_shadow:=true", "enable_control_injection:=true",
        "control_gain:=0.1", "control_force_limit:=3.0",
        "control_rate_limit:=5.0", "myrbf_config:=" + config,
        "enable_motion:=true", "motion_gait:=static_walk",
        "motion_vx:=0.08", "motion_vy:=0.0",
        "motion_yaw_rate:=0.0", "motion_delay:=4.0",
        "motion_duration:=9.0",
        "enable_wrench:=" + scenario_args["enable_wrench"],
        "wrench_profile:=" + scenario_args["wrench_profile"],
        "wrench_axis:=fy", "wrench_value:=" + scenario_args["wrench_value"],
        "wrench_amplitude:=" + scenario_args["wrench_amplitude"],
        "wrench_frequency:=" + scenario_args["wrench_frequency"],
        "wrench_secondary_amplitude:=0.0",
        "wrench_secondary_frequency:=0.0",
        "wrench_delay:=" + scenario_args["wrench_delay"],
        "wrench_duration:=" + scenario_args["wrench_duration"],
    ]


def build_complete_campaign(root):
    schema = load_schema()
    root = Path(root)
    (root / "PROTOCOL.md").write_text(PROTOCOL_TEXT, encoding="utf-8")
    runs = []
    for method, scenario, repeat, seed, order in schema.expected_run_identities():
        run_id = "%s_r%d_%s" % (
            scenario, repeat, method.lower().replace("-", ""))
        run_dir = root / run_id
        run_dir.mkdir()
        duration = schema.SCENARIOS[scenario]["duration_s"]
        elapsed_values = [0.25 * index
                          for index in range(int(duration / 0.25) + 1)]
        start_ros = 100.0
        shadow_rows = []
        state_rows = [{
            "ros_time": "99.75", "campaign_start_ros": "100.0",
            "wall_elapsed": "-0.25",
            "base_x": "nan", "base_y": "nan", "base_z": "nan",
            "observation_base_yaw": "nan",
            "observation_base_pitch": "nan",
            "observation_base_roll": "nan",
            "base_vx": "nan", "base_vy": "nan", "base_vz": "nan",
            "startup_extra": "not-ready",
        }]
        wrench_rows = []
        motion_rows = []
        for elapsed in elapsed_values:
            ros_time = start_ros + elapsed
            motion_active = 4.0 <= elapsed < 13.0
            if scenario == "constant_fy" and 6.0 <= elapsed < 12.0:
                external = 20.0
            elif scenario == "sine_fy" and 6.0 <= elapsed < 12.0:
                external = 15.0 * math.sin(
                    2.0 * math.pi * 0.5 * (elapsed - 6.0))
            else:
                external = 0.0
            onset = 4.0 if scenario == "no_disturbance" else 6.0
            response = 0.0 if elapsed <= onset else 0.03
            target = -external
            shadow_rows.append({
                "campaign_start_ros": "100.0",
                "wall_elapsed": "%.2f" % elapsed,
                "time_s": "%.2f" % ros_time,
                "estimate_after_fy_n": "%.12f" % target,
                "actual_base_y_m": "%.6f" % (2.0 + response),
                "actual_base_roll_rad": "%.6f" % (0.01 + response / 3.0),
                "requested_control_fy_n": "%.12f" % target,
                "achieved_control_fy_n": "%.12f" % (0.8 * target),
                "allocator_scale": "1.0",
                "update_frozen": "0",
                "projection_active_fx": "0",
                "projection_active_fy": "0",
                "projection_active_fz": "0",
                "projection_active_tx": "0",
                "projection_active_ty": "0",
                "projection_active_tz": "0",
                "retained_extra_column": "kept",
            })
            state_rows.append({
                "ros_time": "%.2f" % ros_time,
                "campaign_start_ros": "100.0",
                "wall_elapsed": "%.2f" % elapsed,
                "base_x": "0.0", "base_y": "%.6f" % (2.0 + response),
                "base_z": "0.34",
                "observation_base_yaw": "0.0",
                "observation_base_pitch": "0.0",
                "observation_base_roll": "0.01",
                "base_vx": "0.0", "base_vy": "0.0", "base_vz": "0.0",
                "startup_extra": "ready",
            })
            wrench_rows.append({
                "ros_time": "%.2f" % ros_time,
                "campaign_start_ros": "100.0",
                "wall_elapsed": "%.2f" % elapsed,
                "fx": "0", "fy": "%.12f" % external, "fz": "0",
                "tx": "0", "ty": "0", "tz": "0",
                "service_called": "1" if 6.0 <= elapsed < 12.0 else "0",
                "service_success": "1",
                "clear_success": "1",
            })
            motion_rows.append({
                "ros_time": "%.2f" % ros_time,
                "campaign_start_ros": "100.0",
                "wall_elapsed": "%.2f" % elapsed,
                "active": "1" if motion_active else "0",
                "gait": "static_walk" if motion_active else "stance",
                "vx": "0.08" if motion_active else "0",
                "vy": "0", "yaw_rate": "0", "publish_count": "1",
            })
        write_csv(run_dir / "shadow.csv", tuple(shadow_rows[0]), shadow_rows)
        write_csv(run_dir / "state.csv", tuple(state_rows[0]), state_rows)
        write_csv(run_dir / "wrench.csv", tuple(wrench_rows[0]), wrench_rows)
        write_csv(run_dir / "motion.csv", tuple(motion_rows[0]), motion_rows)
        (run_dir / "run.log").write_text(
            "campaign process completed\n", encoding="utf-8")
        runs.append({
            "run_id": run_id,
            "method": method,
            "scenario": scenario,
            "repeat": repeat,
            "seed": seed,
            "order": order,
            "paired_conditions": paired_conditions_fixture(scenario, seed),
            "git_commit": "0b0544906739e33341075725b92ebe2c34da7eb0",
            "config_path": schema.METHOD_CONFIGS[method],
            "config_sha256": schema.METHOD_CONFIG_SHA256[method],
            "command": launch_command_fixture(
                method, scenario, seed, run_dir),
            "started_at": "2026-09-01T00:00:00Z",
            "finished_at": "2026-09-01T00:00:18Z",
            "exit_code": 0,
            "controller_rows": len(shadow_rows),
            "estimator_rows": len(state_rows) - 1,
            "finite": True,
            "safety_stop_detected": False,
            "complete": True,
            "output_dir": run_id,
            "attempt_history": [],
            "observed_spans": {
                "controller": {
                    "rows": len(shadow_rows), "start_s": 100.0,
                    "stop_s": 100.0 + duration, "span_s": duration,
                },
                "estimator": {
                    "rows": len(state_rows) - 1, "start_s": 0.0,
                    "stop_s": duration, "span_s": duration,
                },
                "motion": {
                    "rows": len(motion_rows), "start_s": 0.0,
                    "stop_s": duration, "span_s": duration,
                },
                "wrench": {
                    "rows": len(wrench_rows), "start_s": 0.0,
                    "stop_s": duration, "span_s": duration,
                },
            },
            "campaign_start_ros": 100.0,
        })
    source_lock = {
        "schema_version": schema.SOURCE_LOCK_VERSION,
        "source_commit": "0b0544906739e33341075725b92ebe2c34da7eb0",
        "files": {
            relative: "0" * 64
            for relative in schema.EXECUTION_CRITICAL_FILES
        },
    }
    source_lock_sha256 = hashlib.sha256(json.dumps(
        source_lock, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")).hexdigest()
    manifest = {
        "schema_version": schema.SCHEMA_VERSION,
        "protocol_sha256": schema.FROZEN_PROTOCOL_SHA256,
        "protocol_snapshot": schema.protocol_snapshot(),
        "source_lock": source_lock,
        "source_lock_sha256": source_lock_sha256,
        "runs": runs,
    }
    (root / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


class CampaignSchemaTest(unittest.TestCase):
    # Production mutation caught: a runner and analyzer silently using different
    # methods, scenario timings, seeds, pair order, hashes, gates, or field names.
    def test_protocol_snapshot_and_run_identities_are_exact(self):
        schema = load_schema()

        self.assertEqual(schema.METHODS, ("V4", "V5-A"))
        self.assertEqual(
            schema.SCENARIOS,
            {
                "no_disturbance": {
                    "duration_s": 15.0,
                    "gait": "static_walk",
                    "motion_delay_s": 4.0,
                    "motion_duration_s": 9.0,
                    "vx_mps": 0.08,
                    "vy_mps": 0.0,
                    "yaw_rate_rps": 0.0,
                    "wrench_required": False,
                    "force_axis": "fy",
                    "force_profile": "none",
                    "force_value_n": 0.0,
                    "force_amplitude_n": 0.0,
                    "force_frequency_hz": 0.0,
                    "force_onset_s": None,
                    "force_duration_s": 0.0,
                },
                "constant_fy": {
                    "duration_s": 18.0,
                    "gait": "static_walk",
                    "motion_delay_s": 4.0,
                    "motion_duration_s": 9.0,
                    "vx_mps": 0.08,
                    "vy_mps": 0.0,
                    "yaw_rate_rps": 0.0,
                    "wrench_required": True,
                    "force_axis": "fy",
                    "force_profile": "constant",
                    "force_value_n": 20.0,
                    "force_amplitude_n": 0.0,
                    "force_frequency_hz": 0.0,
                    "force_onset_s": 6.0,
                    "force_duration_s": 6.0,
                },
                "sine_fy": {
                    "duration_s": 18.0,
                    "gait": "static_walk",
                    "motion_delay_s": 4.0,
                    "motion_duration_s": 9.0,
                    "vx_mps": 0.08,
                    "vy_mps": 0.0,
                    "yaw_rate_rps": 0.0,
                    "wrench_required": True,
                    "force_axis": "fy",
                    "force_profile": "sine",
                    "force_value_n": 0.0,
                    "force_amplitude_n": 15.0,
                    "force_frequency_hz": 0.5,
                    "force_onset_s": 6.0,
                    "force_duration_s": 6.0,
                },
            },
        )
        identities = schema.expected_run_identities()
        self.assertEqual(len(identities), 30)
        self.assertEqual(len(set(identities)), 30)
        self.assertEqual(identities[:4], [
            ("V4", "no_disturbance", 1, 4101, 1),
            ("V5-A", "no_disturbance", 1, 4101, 2),
            ("V5-A", "no_disturbance", 2, 4102, 1),
            ("V4", "no_disturbance", 2, 4102, 2),
        ])
        snapshot = schema.protocol_snapshot()
        self.assertEqual(
            snapshot["protocol_sha256"],
            "e6004dbbbc69b81fc58a8567196d46adf84c14dd2eee1b1acee656cca924c484",
        )
        self.assertEqual(snapshot["methods"], {
            "V4": {
                "config_path": "legged_controllers/config/go2/myrbf_shadow_v4_go2.yaml",
                "config_sha256": "c99e28eb6ff8157e2097132710ce8a044bd9e54adc209e39308bc797e1ad43fd",
            },
            "V5-A": {
                "config_path": "legged_controllers/config/go2/myrbf_shadow_v5a_go2.yaml",
                "config_sha256": "e00afb30644f08e5d859262d55d103ff69525231997b50fdca7007e38918f70c",
            },
        })
        self.assertEqual(snapshot["gates"], {
            "safety": {
                "minimum_base_height_m": 0.25,
                "maximum_freeze_rate": 0.05,
                "maximum_saturation_duration_s": 0.25,
            },
            "no_disturbance": {
                "lateral_rms_ratio": 1.05,
                "roll_rms_ratio": 1.05,
                "maximum_achieved_compensation_rms_n": 3.0,
            },
            "sine_fy": {
                "estimate_rmse_ratio": 1.10,
                "lateral_rms_ratio": 0.90,
                "roll_rms_ratio": 1.05,
            },
            "constant_fy": {
                "estimate_rmse_ratio": 1.10,
                "lateral_rms_ratio": 1.05,
                "roll_rms_ratio": 1.05,
            },
        })
        self.assertEqual(schema.REQUIRED_MANIFEST_FIELDS, (
            "schema_version", "protocol_sha256", "protocol_snapshot",
            "source_lock", "source_lock_sha256", "runs",
        ))
        self.assertEqual(schema.REQUIRED_RUN_FIELDS, (
            "run_id", "method", "scenario", "repeat", "seed", "order",
            "paired_conditions", "git_commit", "config_path",
            "config_sha256", "command",
            "started_at", "finished_at", "exit_code", "controller_rows",
            "estimator_rows", "finite", "safety_stop_detected", "complete",
            "output_dir", "attempt_history", "observed_spans",
            "campaign_start_ros",
        ))
        self.assertEqual(schema.paired_conditions("sine_fy", 4103), {
            "scenario": "sine_fy", "seed": 4103, "duration_s": 18.0,
            "gait": "static_walk", "motion_delay_s": 4.0,
            "motion_duration_s": 9.0, "vx_mps": 0.08, "vy_mps": 0.0,
            "yaw_rate_rps": 0.0, "wrench_required": True,
            "force_axis": "fy", "force_profile": "sine",
            "force_value_n": 0.0, "force_amplitude_n": 15.0,
            "force_frequency_hz": 0.5, "force_onset_s": 6.0,
            "force_duration_s": 6.0,
        })
        self.assertEqual(schema.REQUIRED_CSV_FIELDS["state.csv"],
                         (
                             "ros_time", "campaign_start_ros", "wall_elapsed",
                             "base_x", "base_y", "base_z",
                             "observation_base_yaw",
                             "observation_base_pitch",
                             "observation_base_roll",
                             "base_vx", "base_vy", "base_vz",
                         ))
        self.assertEqual(schema.REQUIRED_CSV_FIELDS["motion.csv"], (
            "ros_time", "campaign_start_ros", "wall_elapsed",
            "active", "gait", "vx", "vy",
            "yaw_rate",
        ))

    # Production mutation caught: Task 4 and the analyzer drifting on launch
    # arguments or on the numeric limits for recorded pair initial states.
    def test_launch_and_initial_state_fairness_contract_is_exact(self):
        schema = load_schema()

        self.assertEqual(getattr(schema, "INITIAL_STATE_TOLERANCES", None), {
            "base_x": 0.01, "base_y": 0.01, "base_z": 0.01,
            "observation_base_yaw": 0.01,
            "observation_base_pitch": 0.01,
            "observation_base_roll": 0.01,
            "base_vx": 0.02, "base_vy": 0.02, "base_vz": 0.02,
        })
        self.assertEqual(
            schema.expected_launch_arguments("V5-A", "sine_fy", 4103),
            {
                "robot_type": "go2", "gui": "false",
                "gazebo_seed": "4103",
                "duration": "18.0", "enable_shadow": "true",
                "enable_control_injection": "true", "control_gain": "0.1",
                "control_force_limit": "3.0", "control_rate_limit": "5.0",
                "myrbf_config": "myrbf_shadow_v5a_go2.yaml",
                "enable_motion": "true", "motion_gait": "static_walk",
                "motion_vx": "0.08", "motion_vy": "0.0",
                "motion_yaw_rate": "0.0", "motion_delay": "4.0",
                "motion_duration": "9.0", "enable_wrench": "true",
                "wrench_profile": "sine", "wrench_axis": "fy",
                "wrench_value": "0.0", "wrench_amplitude": "15.0",
                "wrench_frequency": "0.5",
                "wrench_secondary_amplitude": "0.0",
                "wrench_secondary_frequency": "0.0",
                "wrench_delay": "6.0", "wrench_duration": "6.0",
            },
        )
        snapshot = schema.protocol_snapshot()
        self.assertEqual(snapshot["launch"]["prefix"], [
            "roslaunch", "legged_controllers", "myrbf_shadow_gazebo.launch",
        ])
        self.assertEqual(snapshot["launch"]["output_arguments"], [
            "state_output", "shadow_output", "wrench_output", "motion_output",
        ])
        self.assertEqual(snapshot["initial_state_tolerances"],
                         schema.INITIAL_STATE_TOLERANCES)

    # Production mutation caught: runner/analyzer using a shorter accepted
    # recording span or recognizing different safety-stop messages.
    def test_full_run_coverage_and_safety_markers_are_shared_exactly(self):
        schema = load_schema()

        self.assertEqual(schema.FULL_RUN_COVERAGE, {
            "start_margin_fraction": 0.05,
            "minimum_span_fraction": 0.95,
        })
        self.assertEqual(schema.SAFETY_STOP_MARKERS, (
            "[SafetyChecker] Orientation safety check failed!",
            "[Legged Controller] Safety check failed, stopping the controller.",
        ))


class OnsetRelativeMotionTest(unittest.TestCase):
    # Production mutation caught: computing RMS from absolute lateral position
    # instead of subtracting the position at the force-window onset.
    def test_lateral_motion_is_relative_to_force_onset(self):
        module = load_analyzer()
        metric = module.onset_relative_rms(
            [5.0, 6.0, 7.0, 8.0],
            [1.8, 2.0, 2.03, 1.97],
            onset=6.0,
            stop=8.0,
        )

        self.assertAlmostEqual(metric, math.sqrt(0.0006), places=12)

    # Production mutation caught: subtracting wrapped roll angles before
    # unwrapping, which turns a small crossing of pi into a full revolution.
    def test_roll_motion_is_unwrapped_before_onset_subtraction(self):
        module = load_analyzer()
        metric = module.onset_relative_rms(
            [6.0, 7.0, 8.0],
            [3.13, -3.13, -3.12],
            onset=6.0,
            stop=8.0,
            unwrap=True,
        )

        self.assertAlmostEqual(metric, 0.0233725129273348, places=12)


class ForceResponseMetricsTest(unittest.TestCase):
    # Production mutation caught: comparing estimates or achieved compensation
    # with +external force, reversing lag, or subtracting aggregate RMS values.
    def test_delayed_sinusoid_has_expected_estimation_and_compensation_metrics(self):
        module = load_analyzer()
        sample_period = 0.04
        target = [
            10.0 * math.sin(2.0 * math.pi * index / 400.0)
            for index in range(400)
        ]
        estimate = target[-2:] + target[:-2]
        external = [-value for value in target]
        achieved = [0.8 * value for value in target]
        metrics = module.force_response_metrics(
            [sample_period * index for index in range(400)],
            external,
            estimate,
            target,
            achieved,
        )

        self.assertAlmostEqual(metrics["estimate_rmse_n"],
                               0.222135011708735, places=12)
        self.assertGreater(metrics["estimate_correlation"], 0.999)
        self.assertAlmostEqual(metrics["estimate_bias_n"], 0.0, places=12)
        self.assertLessEqual(abs(metrics["response_lag_s"] - 0.08),
                             sample_period)
        self.assertEqual(metrics["compensation_sign_agreement"], 1.0)
        self.assertAlmostEqual(metrics["requested_achieved_rms_gap_n"],
                               1.4142135623731, places=12)


class CampaignValidationTest(unittest.TestCase):
    # Production mutation caught: accepting an empty, abbreviated, or differing
    # source revision instead of one exact 40-character commit for all 30 runs.
    def test_source_commit_must_be_full_and_shared_across_campaign(self):
        module = load_analyzer()
        mutations = (
            ("empty", "", True),
            ("abbreviated", "0b05449", True),
            ("different", "1" * 40, False),
        )
        for name, value, change_all in mutations:
            with self.subTest(name=name):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    manifest = build_complete_campaign(root)
                    targets = (manifest["runs"] if change_all
                               else manifest["runs"][:1])
                    for run in targets:
                        run["git_commit"] = value

                    with self.assertRaisesRegex(ValueError,
                                                "source commit"):
                        module.validate_campaign(manifest, root)

    # Production mutation caught: accepting a missing/wrong per-run fairness
    # map or allowing the V4/V5-A members of one pair to carry different maps.
    def test_paired_conditions_match_frozen_scenario_seed_and_pair(self):
        module = load_analyzer()
        for mutation in ("not_mapping", "wrong_seed", "pair_mismatch"):
            with self.subTest(mutation=mutation):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    manifest = build_complete_campaign(root)
                    if mutation == "not_mapping":
                        manifest["runs"][0]["paired_conditions"] = (
                            "no_disturbance/4101")
                    elif mutation == "wrong_seed":
                        manifest["runs"][0]["paired_conditions"]["seed"] = 9999
                    else:
                        manifest["runs"][1]["paired_conditions"]["vx_mps"] = 0.09

                    with self.assertRaisesRegex(ValueError,
                                                "paired conditions"):
                        module.validate_campaign(manifest, root)

    # Production mutation caught: accepting a shell-like command string when
    # the provenance contract requires the exact subprocess argument list.
    def test_manifest_command_must_be_nonempty_argument_list(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            manifest["runs"][0]["command"] = "synthetic V4 no_disturbance"

            with self.assertRaisesRegex(ValueError, "command argument list"):
                module.validate_campaign(manifest, root)

    # Production mutation caught: treating any string-list command as fair and
    # accepting a wrong package, seed, source config, or shared control setting.
    def test_manifest_command_must_match_actual_frozen_launch(self):
        module = load_analyzer()
        mutations = (
            ("legged_controllers", "wrong_package"),
            ("gazebo_seed:=4101", "gazebo_seed:=9999"),
            ("myrbf_config:=myrbf_shadow_v4_go2.yaml",
             "myrbf_config:=myrbf_shadow_v5a_go2.yaml"),
            ("control_gain:=0.1", "control_gain:=0.2"),
        )
        for original, replacement in mutations:
            with self.subTest(replacement=replacement):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    manifest = build_complete_campaign(root)
                    command = manifest["runs"][0]["command"]
                    command[command.index(original)] = replacement

                    with self.assertRaisesRegex(ValueError,
                                                "launch command"):
                        module.validate_campaign(manifest, root)

    def test_manifest_command_requires_exact_order_and_attempt_outputs(self):
        module = load_analyzer()
        for mutation in ("order", "output"):
            with self.subTest(mutation=mutation):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    manifest = build_complete_campaign(root)
                    command = manifest["runs"][0]["command"]
                    if mutation == "order":
                        command[3], command[4] = command[4], command[3]
                    else:
                        index = next(
                            i for i, value in enumerate(command)
                            if value.startswith("state_output:=")
                        )
                        command[index] = "state_output:=/tmp/wrong/state.csv"

                    with self.assertRaisesRegex(ValueError, "launch command"):
                        module.validate_campaign(manifest, root)

    # Production mutation caught: validating the seed under the unused
    # seed:= name instead of the gazebo_seed:= argument forwarded to Gazebo.
    def test_launch_command_requires_gazebo_seed_argument(self):
        module = load_analyzer()
        cases = (("gazebo_seed:=4101", True), ("seed:=4101", False))
        for seed_argument, accepted in cases:
            with self.subTest(seed_argument=seed_argument):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    manifest = build_complete_campaign(root)
                    command = manifest["runs"][0]["command"]
                    command[command.index("gazebo_seed:=4101")] = seed_argument

                    if accepted:
                        try:
                            module.validate_campaign(manifest, root)
                        except ValueError as error:
                            self.fail("gazebo_seed launch argument rejected: %s" %
                                      error)
                    else:
                        with self.assertRaisesRegex(ValueError,
                                                    "launch command"):
                            module.validate_campaign(manifest, root)

    # Production mutation caught: comparing only declared pair settings and
    # accepting position, orientation, or velocity differences in recorded data.
    def test_recorded_initial_state_must_match_within_tolerances(self):
        module = load_analyzer()
        mutations = (
            ("base_x", "0.010001"),
            ("observation_base_roll", "0.020001"),
            ("base_vx", "0.020001"),
        )
        for field, value in mutations:
            with self.subTest(field=field):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    manifest = build_complete_campaign(root)
                    run = manifest["runs"][1]
                    path = root / run["run_id"] / "state.csv"
                    rows = read_csv(path)
                    rows[1][field] = value
                    write_csv(path, tuple(rows[0]), rows)

                    with self.assertRaisesRegex(ValueError,
                                                "initial state"):
                        module.validate_campaign(manifest, root)

    # Production mutation caught: changing an inclusive documented initial
    # state tolerance into a strict comparison that rejects its exact boundary.
    def test_recorded_initial_state_exact_tolerance_is_accepted(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = manifest["runs"][1]
            path = root / run["run_id"] / "state.csv"
            rows = read_csv(path)
            rows[1]["base_x"] = "0.01"
            rows[1]["observation_base_roll"] = "0.02"
            rows[1]["base_vx"] = "0.02"
            write_csv(path, tuple(rows[0]), rows)

            module.validate_campaign(manifest, root)

    # Production mutation caught: trusting complete=true even when the runner's
    # independent log scan recorded that a safety stop occurred.
    def test_safety_stop_is_rejected_even_when_run_claims_complete(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            manifest["runs"][0]["safety_stop_detected"] = True
            self.assertTrue(manifest["runs"][0]["complete"])

            with self.assertRaisesRegex(ValueError, "safety stop"):
                module.validate_campaign(manifest, root)

    # Production mutation caught: accepting 0/1 or another truthy value in a
    # field whose manifest contract requires an actual JSON boolean.
    def test_safety_stop_detected_must_be_boolean(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            manifest["runs"][0]["safety_stop_detected"] = 0

            with self.assertRaisesRegex(ValueError,
                                        "safety_stop_detected.*boolean"):
                module.validate_campaign(manifest, root)

    # Production mutation caught: trusting safety_stop_detected=false instead
    # of independently scanning the preserved process log case-insensitively.
    def test_analyzer_rejects_safety_marker_hidden_by_manifest(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = manifest["runs"][0]
            self.assertFalse(run["safety_stop_detected"])
            (root / run["output_dir"] / "run.log").write_text(
                "[lEgGeD cOnTrOlLeR] sAfEtY cHeCk FaIlEd, "
                "sToPpInG tHe CoNtRoLlEr.\n",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "safety-stop marker"):
                module.validate_campaign(manifest, root)

    # Production mutation caught: rejecting valid retained extra columns or
    # a present zero-force no-disturbance file, or treating leading startup nan
    # state rows as usable estimator samples.
    def test_present_zero_no_disturbance_wrench_and_startup_rows_are_accepted(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            self.assertTrue((root / manifest["runs"][0]["run_id"] /
                             "wrench.csv").is_file())

            module.validate_campaign(manifest, root)
            metrics = module.analyze_run(manifest["runs"][0], root)

        self.assertEqual(metrics["scenario"], "no_disturbance")
        self.assertEqual(metrics["analysis_onset_s"], 104.0)
        self.assertEqual(metrics["analysis_stop_s"], 113.0)
        self.assertEqual(metrics["minimum_base_height_m"], 0.34)
        self.assertEqual(metrics["estimate_rmse_n"], 0.0)
        self.assertEqual(metrics["freeze_rate"], 0.0)
        self.assertEqual(metrics["projection_rate"], 0.0)

    # Production mutation caught: requiring wrench.csv for no_disturbance even
    # though Task 4's completeness contract omits it for zero-force runs.
    def test_missing_no_disturbance_wrench_uses_explicit_zero_truth(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = manifest["runs"][0]
            (root / run["run_id"] / "wrench.csv").unlink()

            metrics = module.analyze_run(run, root)

        self.assertEqual(metrics["analysis_onset_s"], 104.0)
        self.assertEqual(metrics["analysis_stop_s"], 113.0)
        self.assertEqual(metrics["disturbance_sample_count"], 0)
        self.assertEqual(metrics["estimate_rmse_n"], 0.0)

    # Production mutation caught: accepting a 29-run campaign and therefore
    # computing medians from an unpaired or incomplete experiment matrix.
    def test_missing_run_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            manifest["runs"].pop()

            with self.assertRaisesRegex(ValueError, "incomplete campaign"):
                module.validate_campaign(manifest, root)

    # Production mutation caught: collapsing duplicate method/scenario/repeat/
    # seed identities into one dictionary entry and hiding a missing pair.
    def test_duplicate_run_identity_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            manifest["runs"][-1] = dict(manifest["runs"][0])

            with self.assertRaisesRegex(ValueError, "duplicate run identity"):
                module.validate_campaign(manifest, root)

    # Production mutation caught: trusting a run's claimed configuration hash
    # without comparing it with the top-level frozen method snapshot.
    def test_configuration_hash_mismatch_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            manifest["runs"][0]["config_sha256"] = "0" * 64

            with self.assertRaisesRegex(ValueError, "configuration hash"):
                module.validate_campaign(manifest, root)

    # Production mutation caught: accepting a top-level scenario snapshot that
    # no longer matches the analyzer/runner's frozen protocol contract.
    def test_protocol_snapshot_drift_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            manifest["protocol_snapshot"]["scenarios"]["sine_fy"][
                "force_duration_s"] = 5.0

            with self.assertRaisesRegex(ValueError, "protocol snapshot"):
                module.validate_campaign(manifest, root)

    # Production mutation caught: allowing a nan in a controller row that is
    # otherwise inside the usable recorded stream.
    def test_nonfinite_usable_csv_value_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = manifest["runs"][0]
            path = root / run["run_id"] / "shadow.csv"
            rows = read_csv(path)
            rows[1]["estimate_after_fy_n"] = "nan"
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError, "nonfinite"):
                module.analyze_run(run, root)

    # Production mutation caught: validating only analysis-window controller
    # rows and allowing an earlier duplicate time into later lag/recovery logic.
    def test_controller_times_must_be_strictly_increasing(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = manifest["runs"][0]
            path = root / run["run_id"] / "shadow.csv"
            rows = read_csv(path)
            rows[1]["time_s"] = rows[0]["time_s"]
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError,
                                        "controller times.*strictly increasing"):
                module.analyze_run(run, root)

    # Production mutation caught: checking ordering before startup nan rows are
    # skipped, or allowing duplicate timestamps among the usable state rows.
    def test_usable_state_times_must_be_strictly_increasing(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = manifest["runs"][0]
            path = root / run["run_id"] / "state.csv"
            rows = read_csv(path)
            rows[2]["ros_time"] = rows[1]["ros_time"]
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError,
                                        "state times.*strictly increasing"):
                module.analyze_run(run, root)

    # Production mutation caught: silently filling a missing diagnostic field
    # with a default and reporting a false zero projection rate.
    def test_missing_required_csv_field_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = manifest["runs"][0]
            path = root / run["run_id"] / "shadow.csv"
            rows = read_csv(path)
            for row in rows:
                row.pop("projection_active_tz")
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError, "missing required fields"):
                module.analyze_run(run, root)

    # Production mutation caught: deriving a force window from one or two noisy
    # nonzero truth samples and presenting it as a disturbance response.
    def test_too_few_disturbance_samples_are_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "constant_fy")
            path = root / run["run_id"] / "wrench.csv"
            rows = read_csv(path)
            for row in rows:
                retained = row["wall_elapsed"] in ("6.25", "6.50")
                row["service_called"] = "1" if retained else "0"
                row["fy"] = "20.0" if retained else "0.0"
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError,
                                        "too few disturbance samples"):
                module.analyze_run(run, root)

    # Production mutation caught: deriving sine onset from nonzero magnitude
    # and dropping the successfully applied zero-valued first sine sample.
    def test_sine_zero_valued_successful_onset_is_included(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "sine_fy")

            metrics = module.analyze_run(run, root)

        self.assertEqual(metrics["analysis_onset_s"], 106.0)
        self.assertEqual(metrics["analysis_stop_s"], 112.0)
        self.assertEqual(metrics["disturbance_sample_count"], 24)

    # Production mutation caught: treating a scheduled force row as valid even
    # when Gazebo reported that the wrench service call failed.
    def test_failed_force_application_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "sine_fy")
            path = root / run["run_id"] / "wrench.csv"
            rows = read_csv(path)
            next(row for row in rows
                 if row["wall_elapsed"] == "6.00")["service_success"] = "0"
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError,
                                        "wrench service call failed"):
                module.analyze_run(run, root)

    # Production mutation caught: accepting a recorded force value that does
    # not match the frozen constant-force waveform.
    def test_force_waveform_mismatch_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "constant_fy")
            path = root / run["run_id"] / "wrench.csv"
            rows = read_csv(path)
            next(row for row in rows
                 if row["wall_elapsed"] == "7.00")["fy"] = "19.0"
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError, "force waveform"):
                module.analyze_run(run, root)

    # Production mutation caught: allowing the successful-call window to end
    # one sample before the frozen six-second force duration.
    def test_force_duration_mismatch_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "constant_fy")
            path = root / run["run_id"] / "wrench.csv"
            rows = read_csv(path)
            next(row for row in rows
                 if row["wall_elapsed"] == "11.75")["service_called"] = "0"
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError, "force schedule"):
                module.analyze_run(run, root)

    # Production mutation caught: ignoring an active gait that differs from
    # the frozen static_walk command.
    def test_motion_gait_mismatch_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = manifest["runs"][0]
            path = root / run["run_id"] / "motion.csv"
            rows = read_csv(path)
            next(row for row in rows
                 if row["wall_elapsed"] == "5.00")["gait"] = "trot"
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError, "motion gait"):
                module.analyze_run(run, root)

    # Production mutation caught: validating vx but silently accepting drift
    # in vx, vy, or yaw_rate from the frozen motion command.
    def test_motion_velocity_components_are_all_validated(self):
        module = load_analyzer()
        mutations = (("vx", "0.09"), ("vy", "0.01"),
                     ("yaw_rate", "0.02"))
        for field, value in mutations:
            with self.subTest(field=field):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    manifest = build_complete_campaign(root)
                    run = manifest["runs"][0]
                    path = root / run["run_id"] / "motion.csv"
                    rows = read_csv(path)
                    next(row for row in rows
                         if row["wall_elapsed"] == "5.00")[field] = value
                    write_csv(path, tuple(rows[0]), rows)

                    with self.assertRaisesRegex(ValueError,
                                                "motion command"):
                        module.analyze_run(run, root)

    # Production mutation caught: trusting active flags instead of checking the
    # frozen four-second delay and nine-second motion duration.
    def test_motion_timing_mismatch_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = manifest["runs"][0]
            path = root / run["run_id"] / "motion.csv"
            rows = read_csv(path)
            next(row for row in rows
                 if row["wall_elapsed"] == "4.00")["active"] = "0"
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError, "motion timing"):
                module.analyze_run(run, root)

    # Production mutation caught: shrinking the apparent force window to the
    # surviving rows when the beginning of wrench truth is truncated.
    def test_truncated_wrench_truth_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "constant_fy")
            path = root / run["run_id"] / "wrench.csv"
            rows = [row for row in read_csv(path)
                    if float(row["wall_elapsed"]) >= 6.5]
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError,
                                        "wrench truth coverage"):
                module.analyze_run(run, root)

    # Production mutation caught: allowing interpolation to bridge a multi-
    # second hole in wrench truth and fabricate force values in the gap.
    def test_disjoint_wrench_truth_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "constant_fy")
            path = root / run["run_id"] / "wrench.csv"
            rows = [row for row in read_csv(path)
                    if not 8.0 <= float(row["wall_elapsed"]) < 10.5]
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError,
                                        "wrench truth coverage"):
                module.analyze_run(run, root)

    # Production mutation caught: checking only wall-time spacing and allowing
    # ROS-time interpolation to bridge a large monotonic hole in force truth.
    def test_excessive_ros_time_gap_in_schedule_is_rejected(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "constant_fy")
            path = root / run["run_id"] / "wrench.csv"
            rows = read_csv(path)
            for row in rows:
                elapsed = float(row["wall_elapsed"])
                if 8.0 <= elapsed <= 12.0:
                    row["ros_time"] = "%.3f" % (
                        110.0 + 0.5 * (elapsed - 8.0))
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError, "ROS-time gap"):
                module.analyze_run(run, root)

    # Production mutation caught: treating equal consecutive ROS timestamps as
    # monotonic and silently deduplicating one applied-force observation.
    def test_schedule_ros_times_must_be_strictly_increasing(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "constant_fy")
            path = root / run["run_id"] / "wrench.csv"
            rows = read_csv(path)
            prior = next(row for row in rows
                         if row["wall_elapsed"] == "7.00")
            duplicate = next(row for row in rows
                             if row["wall_elapsed"] == "7.25")
            duplicate["ros_time"] = prior["ros_time"]
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError,
                                        "strictly increasing ROS times"):
                module.analyze_run(run, root)

    # Production mutation caught: trusting a no-disturbance label while its
    # recorded wrench truth contains a real nonzero lateral force.
    def test_no_disturbance_rejects_nonzero_force_truth(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = manifest["runs"][0]
            path = root / run["run_id"] / "wrench.csv"
            rows = read_csv(path)
            rows[20]["fy"] = "1.0"
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError, "nonzero force truth"):
                module.analyze_run(run, root)

    # Production mutation caught: pairing controller and estimator streams that
    # share only a short fragment because one full-run record is incomplete.
    def test_short_controller_estimator_overlap_is_rejected_as_incomplete(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "constant_fy")
            path = root / run["run_id"] / "state.csv"
            rows = read_csv(path)
            rows = [row for row in rows
                    if row["ros_time"] == "99.75"
                    or float(row["ros_time"]) <= 110.0]
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError, "full-run coverage"):
                module.analyze_run(run, root)

    # Production mutation caught: using endpoint extrapolation when requested
    # times lie outside the source series instead of rejecting absent coverage.
    def test_interpolation_rejects_endpoint_extrapolation(self):
        module = load_analyzer()
        source = [
            {"ros_time": 1.0, "fy": 10.0},
            {"ros_time": 2.0, "fy": 20.0},
        ]

        with self.assertRaisesRegex(ValueError, "source coverage"):
            module._nearest_values([0.5, 1.5], source, "ros_time", "fy")

    # Production mutation caught: helper-only motion tests passing while the
    # full run path reports absolute y and wrapped roll instead of onset deltas.
    def test_analyze_run_reports_onset_relative_lateral_and_unwrapped_roll(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "sine_fy")
            path = root / run["run_id"] / "shadow.csv"
            rows = read_csv(path)
            for row in rows:
                if float(row["time_s"]) <= 106.0:
                    row["actual_base_y_m"] = "2.0"
                    row["actual_base_roll_rad"] = "3.13"
                else:
                    row["actual_base_y_m"] = "2.03"
                    row["actual_base_roll_rad"] = "-3.13"
            write_csv(path, tuple(rows[0]), rows)

            metrics = module.analyze_run(run, root)

        self.assertAlmostEqual(metrics["lateral_rms_m"],
                               0.0293938769133981, places=12)
        self.assertAlmostEqual(metrics["lateral_max_m"], 0.03, places=12)
        self.assertAlmostEqual(metrics["roll_rms_deg"],
                               1.30158070873515, places=12)
        self.assertAlmostEqual(metrics["roll_max_deg"],
                               1.32842024810467, places=12)

    # Production mutation caught: dropping the controller sample before onset
    # and using the first post-onset lateral/roll values as a zero baseline.
    def test_analyze_run_interpolates_exact_onset_motion_baseline(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "sine_fy")
            for filename in ("motion.csv", "wrench.csv"):
                path = root / run["run_id"] / filename
                rows = read_csv(path)
                for row in rows:
                    row["ros_time"] = "%.3f" % (
                        float(row["ros_time"]) + 0.125)
                write_csv(path, tuple(rows[0]), rows)

            path = root / run["run_id"] / "shadow.csv"
            rows = read_csv(path)
            for row in rows:
                if float(row["time_s"]) <= 106.0:
                    row["actual_base_y_m"] = "2.0"
                    row["actual_base_roll_rad"] = "3.13"
                else:
                    row["actual_base_y_m"] = "2.04"
                    row["actual_base_roll_rad"] = "-3.13"
            write_csv(path, tuple(rows[0]), rows)

            metrics = module.analyze_run(run, root)

        self.assertAlmostEqual(metrics["analysis_onset_s"], 106.125)
        self.assertAlmostEqual(metrics["lateral_rms_m"], 0.02, places=12)
        self.assertAlmostEqual(metrics["lateral_max_m"], 0.02, places=12)
        self.assertAlmostEqual(metrics["roll_rms_deg"],
                               0.6642101240523348, places=12)
        self.assertAlmostEqual(metrics["roll_max_deg"],
                               0.6642101240523348, places=12)

    # Production mutation caught: checking only controller first/last coverage
    # and accepting a missing interior block before computing all metrics.
    def test_analyze_run_rejects_interior_controller_gap(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "constant_fy")
            path = root / run["run_id"] / "shadow.csv"
            rows = read_csv(path)
            rows = [row for row in rows
                    if not 108.0 <= float(row["time_s"]) < 110.5]
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError,
                                        "controller times.*excessive gap"):
                module.analyze_run(run, root)

    # Production mutation caught: computing minimum height from state evidence
    # whose first/last timestamps cover the window but omit an interior block.
    def test_analyze_run_rejects_interior_usable_state_gap(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "constant_fy")
            path = root / run["run_id"] / "state.csv"
            rows = read_csv(path)
            rows = [row for row in rows
                    if row["base_z"] == "nan"
                    or not 108.0 <= float(row["ros_time"]) < 110.5]
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError,
                                        "state times.*excessive gap"):
                module.analyze_run(run, root)

    # Production mutation caught: validating only the force/motion analysis
    # window and accepting controller/state evidence truncated at either end.
    def test_analyze_run_rejects_truncated_full_duration_streams(self):
        module = load_analyzer()
        mutations = (
            ("shadow.csv", "time_s", "start"),
            ("shadow.csv", "time_s", "end"),
            ("state.csv", "ros_time", "start"),
            ("state.csv", "ros_time", "end"),
        )
        for filename, time_field, edge in mutations:
            with self.subTest(filename=filename, edge=edge):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    manifest = build_complete_campaign(root)
                    run = next(item for item in manifest["runs"]
                               if item["scenario"] == "constant_fy")
                    path = root / run["run_id"] / filename
                    rows = read_csv(path)
                    finite_rows = [row for row in rows
                                   if row[time_field].lower() != "nan"]
                    boundary = (
                        float(finite_rows[0][time_field]) + 1.25
                        if edge == "start"
                        else float(finite_rows[-1][time_field]) - 1.25
                    )
                    retained = []
                    for row in rows:
                        try:
                            sample_time = float(row[time_field])
                        except ValueError:
                            retained.append(row)
                            continue
                        if ((edge == "start" and sample_time >= boundary) or
                                (edge == "end" and sample_time <= boundary)):
                            retained.append(row)
                    write_csv(path, tuple(rows[0]), retained)

                    with self.assertRaisesRegex(
                            ValueError, "full-run coverage"):
                        module.analyze_run(run, root)

    # Production mutation caught: trusting claimed row counts/spans instead of
    # independently recomputing them from the evidence files.
    def test_manifest_row_and_span_claims_must_match_evidence(self):
        module = load_analyzer()
        mutations = (
            ("controller_rows", 999),
            ("estimator_rows", 999),
            ("controller_span_s", 99.0),
            ("motion_span_s", 99.0),
        )
        for field, value in mutations:
            with self.subTest(field=field):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    manifest = build_complete_campaign(root)
                    run = manifest["runs"][0]
                    if field.endswith("_rows"):
                        run[field] = value
                    else:
                        stream = field.split("_", 1)[0]
                        run["observed_spans"][stream]["span_s"] = value

                    with self.assertRaisesRegex(ValueError,
                                                "row/span claims"):
                        module.validate_campaign(manifest, root)

    def test_manifest_span_claims_reject_nonfinite_values(self):
        module = load_analyzer()
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.subTest(value=value):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    manifest = build_complete_campaign(root)
                    manifest["runs"][0]["observed_spans"][
                        "controller"]["span_s"] = value

                    with self.assertRaisesRegex(ValueError,
                                                "row/span claims"):
                        module.validate_campaign(manifest, root)

    def test_analyzer_rejects_evidence_with_a_different_campaign_start(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = manifest["runs"][0]
            path = root / run["run_id"] / "motion.csv"
            rows = read_csv(path)
            for row in rows:
                row["campaign_start_ros"] = "101.0"
            write_csv(path, tuple(rows[0]), rows)

            with self.assertRaisesRegex(ValueError, "campaign start"):
                module.validate_campaign(manifest, root)

    # Production mutation caught: checking span only and accepting a complete-
    # length stream that starts after the shared 5% startup allowance.
    def test_analyze_run_rejects_streams_starting_after_margin(self):
        module = load_analyzer()
        mutations = (("shadow.csv", "time_s"),
                     ("state.csv", "wall_elapsed"))
        for filename, field in mutations:
            with self.subTest(filename=filename):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    manifest = build_complete_campaign(root)
                    run = next(item for item in manifest["runs"]
                               if item["scenario"] == "constant_fy")
                    path = root / run["run_id"] / filename
                    rows = read_csv(path)
                    for row in rows:
                        try:
                            row[field] = "%.2f" % (float(row[field]) + 1.0)
                        except ValueError:
                            pass
                    write_csv(path, tuple(rows[0]), rows)

                    with self.assertRaisesRegex(ValueError, "start margin"):
                        module.analyze_run(run, root)

    # Production mutation caught: computing the gate-facing freeze rate only
    # inside the disturbance window and hiding frozen updates earlier in a run.
    def test_analyze_run_freeze_rate_covers_complete_controller_stream(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "constant_fy")
            path = root / run["run_id"] / "shadow.csv"
            rows = read_csv(path)
            for row in rows[:4]:
                row["update_frozen"] = "1"
            write_csv(path, tuple(rows[0]), rows)

            metrics = module.analyze_run(run, root)

        self.assertAlmostEqual(metrics["freeze_rate"],
                               0.0547945205479452, places=12)

    # Production mutation caught: measuring longest allocator saturation only
    # inside the disturbance window and hiding an earlier unsafe interval.
    def test_analyze_run_saturation_covers_complete_controller_stream(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = build_complete_campaign(root)
            run = next(item for item in manifest["runs"]
                       if item["scenario"] == "constant_fy")
            path = root / run["run_id"] / "shadow.csv"
            rows = read_csv(path)
            for row in rows[:2]:
                row["allocator_scale"] = "0.5"
            write_csv(path, tuple(rows[0]), rows)

            metrics = module.analyze_run(run, root)

        self.assertAlmostEqual(metrics["saturation_duration_s"],
                               0.5, places=12)


def paired_metric_run(method, repeat, lateral_rms):
    values = {
        "run_id": "constant_fy_r%d_%s" % (repeat, method),
        "method": method,
        "scenario": "constant_fy",
        "repeat": repeat,
        "seed": 4100 + repeat,
        "order": 1 if method == ("V4" if repeat % 2 else "V5-A") else 2,
        "estimate_rmse_n": 1.0,
        "estimate_correlation": 0.9,
        "estimate_bias_n": 0.1,
        "response_lag_s": 0.08,
        "requested_compensation_rms_n": 2.0,
        "achieved_compensation_rms_n": 1.5,
        "requested_achieved_rms_gap_n": 0.5,
        "compensation_sign_agreement": 1.0,
        "lateral_rms_m": lateral_rms,
        "lateral_max_m": 2.0 * lateral_rms,
        "roll_rms_deg": 1.0,
        "roll_max_deg": 2.0,
        "allocator_scale_mean": 1.0,
        "allocator_scale_min": 1.0,
        "saturation_duration_s": 0.0,
        "freeze_rate": 0.0,
        "projection_rate": 0.0,
        "minimum_base_height_m": 0.34,
        "recovery_2s_lateral_m": 0.01,
    }
    return values


def gate_summary_row(scenario, metric, v4, v5a):
    return {
        "scenario": scenario,
        "metric": metric,
        "v4_median": v4,
        "v5a_median": v5a,
        "median_difference": v5a - v4,
        "median_percent_difference": (
            100.0 * (v5a - v4) / v4 if v4 != 0.0 else None),
        "mad_difference": 0.0,
        "pairs": [
            {
                "repeat": repeat,
                "seed": 4100 + repeat,
                "v4": v4,
                "v5a": v5a,
                "difference": v5a - v4,
                "percent_difference": (
                    100.0 * (v5a - v4) / v4 if v4 != 0.0 else None),
            }
            for repeat in range(1, 6)
        ],
    }


def passing_gate_summary():
    rows = []
    for scenario in ("no_disturbance", "constant_fy", "sine_fy"):
        rows.extend([
            gate_summary_row(scenario, "minimum_base_height_m", 0.30, 0.30),
            gate_summary_row(scenario, "freeze_rate", 0.01, 0.01),
            gate_summary_row(scenario, "saturation_duration_s", 0.25, 0.25),
        ])
    rows.extend([
        gate_summary_row("no_disturbance", "lateral_rms_m", 1.0, 1.05),
        gate_summary_row("no_disturbance", "roll_rms_deg", 2.0, 2.10),
        gate_summary_row("no_disturbance",
                         "achieved_compensation_rms_n", 2.5, 3.0),
        gate_summary_row("sine_fy", "estimate_rmse_n", 10.0, 11.0),
        gate_summary_row("sine_fy", "lateral_rms_m", 10.0, 9.0),
        gate_summary_row("sine_fy", "roll_rms_deg", 2.0, 2.10),
        gate_summary_row("constant_fy", "estimate_rmse_n", 10.0, 11.0),
        gate_summary_row("constant_fy", "lateral_rms_m", 1.0, 1.05),
        gate_summary_row("constant_fy", "roll_rms_deg", 2.0, 2.10),
    ])
    return rows


class PairedSummaryAndGateTest(unittest.TestCase):
    # Production mutation caught: aggregating runs before pairing, dropping a
    # repeat, or computing percent change and MAD from the wrong quantities.
    def test_five_pairs_keep_literal_points_and_hand_derived_summary(self):
        module = load_analyzer()
        metrics = []
        for repeat, (v4, v5a) in enumerate(
                zip((1.0, 2.0, 3.0, 4.0, 5.0),
                    (0.5, 1.5, 2.5, 3.5, 4.5)), start=1):
            metrics.append(paired_metric_run("V4", repeat, v4))
            metrics.append(paired_metric_run("V5-A", repeat, v5a))

        pairs = module.paired_differences(metrics)
        summary = module.summarize_pairs(pairs)
        lateral_pairs = [row for row in pairs
                         if row["metric"] == "lateral_rms_m"]
        lateral_summary = next(row for row in summary
                               if row["metric"] == "lateral_rms_m")

        self.assertEqual(len(lateral_pairs), 5)
        self.assertEqual(lateral_pairs[0], {
            "scenario": "constant_fy", "repeat": 1, "seed": 4101,
            "metric": "lateral_rms_m", "v4": 1.0, "v5a": 0.5,
            "difference": -0.5, "percent_difference": -50.0,
        })
        self.assertEqual(lateral_summary["v4_median"], 3.0)
        self.assertEqual(lateral_summary["v5a_median"], 2.5)
        self.assertEqual(lateral_summary["median_difference"], -0.5)
        self.assertEqual(lateral_summary["mad_difference"], 0.0)
        self.assertEqual(len(lateral_summary["pairs"]), 5)

    # Production mutation caught: changing <= acceptance boundaries to strict
    # comparisons or implementing the 10% sine improvement as a 10% increase.
    def test_exact_gate_boundaries_pass(self):
        module = load_analyzer()
        schema = load_schema()

        results = module.evaluate_gates(
            passing_gate_summary(), schema.protocol_snapshot())

        self.assertTrue(results["passed"])
        self.assertEqual(results["decision"], "PASS_ALL")
        self.assertTrue(results["safety"]["passed"])
        self.assertTrue(all(item["passed"]
                            for item in results["scenarios"].values()))

    # Production mutation caught: treating the protocol's freeze-rate '< 5%'
    # rule as '<= 5%' and allowing an unsafe run through the global gate.
    def test_freeze_rate_equal_to_five_percent_fails(self):
        module = load_analyzer()
        schema = load_schema()
        summary = passing_gate_summary()
        row = next(item for item in summary
                   if item["scenario"] == "constant_fy"
                   and item["metric"] == "freeze_rate")
        row["pairs"][2]["v5a"] = 0.05

        results = module.evaluate_gates(summary, schema.protocol_snapshot())

        self.assertFalse(results["passed"])
        self.assertEqual(results["decision"], "GENERAL_FAIL")
        self.assertFalse(results["safety"]["passed"])

    # Production mutation caught: collapsing a campaign where only the
    # constant-force gate fails into GENERAL_FAIL instead of its frozen route.
    def test_constant_only_failure_has_distinct_decision(self):
        module = load_analyzer()
        schema = load_schema()
        summary = passing_gate_summary()
        row = next(item for item in summary
                   if item["scenario"] == "constant_fy"
                   and item["metric"] == "lateral_rms_m")
        row["v5a_median"] = 1.050000000002

        results = module.evaluate_gates(summary, schema.protocol_snapshot())

        self.assertFalse(results["passed"])
        self.assertEqual(results["decision"], "CONSTANT_ONLY_FAIL")
        self.assertTrue(results["safety"]["passed"])
        self.assertTrue(results["scenarios"]["no_disturbance"]["passed"])
        self.assertFalse(results["scenarios"]["constant_fy"]["passed"])
        self.assertTrue(results["scenarios"]["sine_fy"]["passed"])


class CampaignOutputTest(unittest.TestCase):
    # Production mutation caught: returning in-memory metrics without writing
    # the JSON, five-point CSV summary, or one paired-line figure per scenario.
    def test_analyze_campaign_writes_all_required_artifacts(self):
        module = load_analyzer()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            build_complete_campaign(root)

            result = module.analyze_campaign(root)

            self.assertTrue((root / "metrics.json").is_file())
            self.assertTrue((root / "paired_summary.csv").is_file())
            self.assertEqual(
                sorted(path.name for path in root.glob("paired_*.png")),
                [
                    "paired_constant_fy.png",
                    "paired_no_disturbance.png",
                    "paired_sine_fy.png",
                ],
            )
            self.assertEqual(len(result["run_metrics"]), 30)
            self.assertEqual(len(result["paired_differences"]),
                             15 * len(module.PAIRED_METRICS))
            self.assertIn("gate_results", result)
            with (root / "paired_summary.csv").open(
                    "r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows),
                             3 * len(module.PAIRED_METRICS))
            self.assertEqual(rows[0]["pair_1_repeat"], "1")

    # Production mutation caught: retaining only the positional campaign path
    # and rejecting the plan's --root form before analysis can start.
    def test_cli_accepts_positional_and_root_option_forms(self):
        module = load_analyzer()
        forms = (("positional", lambda root: [str(root)]),
                 ("option", lambda root: ["--root", str(root)]))
        for name, arguments in forms:
            with self.subTest(name=name):
                with tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    build_complete_campaign(root)
                    with contextlib.redirect_stdout(io.StringIO()):
                        with contextlib.redirect_stderr(io.StringIO()):
                            try:
                                result = module.main(arguments(root))
                            except SystemExit as error:
                                result = error.code

                    self.assertEqual(result, 0)
                    self.assertTrue((root / "metrics.json").is_file())

    # Production mutation caught: letting argparse choose an ambiguous root or
    # emit unrelated errors when both or neither CLI root forms are supplied.
    def test_cli_rejects_both_or_neither_root_with_clear_error(self):
        module = load_analyzer()
        cases = ((), ("campaign-a", "--root", "campaign-b"))
        for arguments in cases:
            with self.subTest(arguments=arguments):
                error_output = io.StringIO()
                with contextlib.redirect_stderr(error_output):
                    with self.assertRaises(SystemExit) as raised:
                        module.main(list(arguments))

                self.assertEqual(raised.exception.code, 2)
                self.assertIn("provide exactly one campaign root",
                              error_output.getvalue())


if __name__ == "__main__":
    unittest.main()
