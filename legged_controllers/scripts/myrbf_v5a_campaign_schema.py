#!/usr/bin/env python3

import copy


SCHEMA_VERSION = 1
PROTOCOL_FILENAME = "PROTOCOL.md"
FROZEN_PROTOCOL_SHA256 = (
    "e6004dbbbc69b81fc58a8567196d46adf84c14dd2eee1b1acee656cca924c484"
)

METHODS = ("V4", "V5-A")
METHOD_CONFIGS = {
    "V4": "legged_controllers/config/go2/myrbf_shadow_v4_go2.yaml",
    "V5-A": "legged_controllers/config/go2/myrbf_shadow_v5a_go2.yaml",
}
METHOD_CONFIG_SHA256 = {
    "V4": "c99e28eb6ff8157e2097132710ce8a044bd9e54adc209e39308bc797e1ad43fd",
    "V5-A": "e00afb30644f08e5d859262d55d103ff69525231997b50fdca7007e38918f70c",
}

LAUNCH_PREFIX = (
    "roslaunch", "legged_controllers", "myrbf_shadow_gazebo.launch",
)
LAUNCH_OUTPUT_ARGUMENTS = (
    "state_output", "shadow_output", "wrench_output", "motion_output",
)
COMMON_LAUNCH_ARGUMENTS = {
    "robot_type": "go2",
    "gui": "false",
    "enable_shadow": "true",
    "enable_control_injection": "true",
    "control_gain": "0.1",
    "control_force_limit": "3.0",
    "control_rate_limit": "5.0",
    "enable_motion": "true",
    "wrench_secondary_amplitude": "0.0",
    "wrench_secondary_frequency": "0.0",
}
INITIAL_STATE_TOLERANCES = {
    "base_x": 0.01,
    "base_y": 0.01,
    "base_z": 0.01,
    "observation_base_yaw": 0.01,
    "observation_base_pitch": 0.01,
    "observation_base_roll": 0.01,
    "base_vx": 0.02,
    "base_vy": 0.02,
    "base_vz": 0.02,
}
INITIAL_ORIENTATION_FIELDS = (
    "observation_base_yaw",
    "observation_base_pitch",
    "observation_base_roll",
)

SCENARIOS = {
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
}

REPEAT_SEEDS = {1: 4101, 2: 4102, 3: 4103, 4: 4104, 5: 4105}
PAIR_ORDER = {
    1: ("V4", "V5-A"),
    2: ("V5-A", "V4"),
    3: ("V4", "V5-A"),
    4: ("V5-A", "V4"),
    5: ("V4", "V5-A"),
}

GATE_THRESHOLDS = {
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
}

REQUIRED_MANIFEST_FIELDS = ("schema_version", "protocol_snapshot", "runs")
REQUIRED_RUN_FIELDS = (
    "run_id",
    "method",
    "scenario",
    "repeat",
    "seed",
    "order",
    "paired_conditions",
    "git_commit",
    "config_path",
    "config_sha256",
    "command",
    "started_at",
    "finished_at",
    "exit_code",
    "controller_rows",
    "estimator_rows",
    "finite",
    "safety_stop_detected",
    "complete",
)

REQUIRED_CSV_FIELDS = {
    "shadow.csv": (
        "time_s",
        "estimate_after_fy_n",
        "actual_base_y_m",
        "actual_base_roll_rad",
        "requested_control_fy_n",
        "achieved_control_fy_n",
        "allocator_scale",
        "update_frozen",
        "projection_active_fx",
        "projection_active_fy",
        "projection_active_fz",
        "projection_active_tx",
        "projection_active_ty",
        "projection_active_tz",
    ),
    "state.csv": (
        "ros_time",
        "base_x",
        "base_y",
        "base_z",
        "observation_base_yaw",
        "observation_base_pitch",
        "observation_base_roll",
        "base_vx",
        "base_vy",
        "base_vz",
    ),
    "wrench.csv": (
        "ros_time",
        "wall_elapsed",
        "fx",
        "fy",
        "fz",
        "tx",
        "ty",
        "tz",
        "service_called",
        "service_success",
    ),
    "motion.csv": (
        "ros_time",
        "wall_elapsed",
        "active",
        "gait",
        "vx",
        "vy",
        "yaw_rate",
    ),
}


def expected_run_identities():
    identities = []
    for scenario in SCENARIOS:
        for repeat, seed in REPEAT_SEEDS.items():
            for order, method in enumerate(PAIR_ORDER[repeat], start=1):
                identities.append((method, scenario, repeat, seed, order))
    return identities


def paired_conditions(scenario, seed):
    if scenario not in SCENARIOS:
        raise ValueError("unknown paired scenario")
    conditions = {"scenario": scenario, "seed": seed}
    conditions.update(SCENARIOS[scenario])
    return copy.deepcopy(conditions)


def expected_launch_arguments(method, scenario, seed):
    if method not in METHODS:
        raise ValueError("unknown paired method")
    if scenario not in SCENARIOS:
        raise ValueError("unknown paired scenario")
    spec = SCENARIOS[scenario]
    arguments = copy.deepcopy(COMMON_LAUNCH_ARGUMENTS)
    arguments.update({
        "gazebo_seed": str(seed),
        "duration": str(spec["duration_s"]),
        "myrbf_config": METHOD_CONFIGS[method].rsplit("/", 1)[-1],
        "motion_gait": spec["gait"],
        "motion_vx": str(spec["vx_mps"]),
        "motion_vy": str(spec["vy_mps"]),
        "motion_yaw_rate": str(spec["yaw_rate_rps"]),
        "motion_delay": str(spec["motion_delay_s"]),
        "motion_duration": str(spec["motion_duration_s"]),
        "enable_wrench": str(spec["wrench_required"]).lower(),
        "wrench_profile": spec["force_profile"],
        "wrench_axis": spec["force_axis"],
        "wrench_value": str(spec["force_value_n"]),
        "wrench_amplitude": str(spec["force_amplitude_n"]),
        "wrench_frequency": str(spec["force_frequency_hz"]),
        "wrench_delay": str(
            0.0 if spec["force_onset_s"] is None
            else spec["force_onset_s"]),
        "wrench_duration": str(spec["force_duration_s"]),
    })
    return arguments


def protocol_snapshot():
    methods = {
        method: {
            "config_path": METHOD_CONFIGS[method],
            "config_sha256": METHOD_CONFIG_SHA256[method],
        }
        for method in METHODS
    }
    repeats = [
        {
            "repeat": repeat,
            "seed": seed,
            "method_order": list(PAIR_ORDER[repeat]),
        }
        for repeat, seed in REPEAT_SEEDS.items()
    ]
    return copy.deepcopy({
        "schema_version": SCHEMA_VERSION,
        "protocol_file": PROTOCOL_FILENAME,
        "protocol_sha256": FROZEN_PROTOCOL_SHA256,
        "methods": methods,
        "scenarios": SCENARIOS,
        "repeats": repeats,
        "gates": GATE_THRESHOLDS,
        "launch": {
            "prefix": list(LAUNCH_PREFIX),
            "output_arguments": list(LAUNCH_OUTPUT_ARGUMENTS),
            "common_arguments": COMMON_LAUNCH_ARGUMENTS,
        },
        "initial_state_tolerances": INITIAL_STATE_TOLERANCES,
    })
