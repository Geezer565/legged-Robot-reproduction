#!/usr/bin/env python3
"""Current Go2 method comparison. Planning is the default; execution is explicit.

Run inside the sourced ROS Noetic container. Never starts/restarts Docker, never
kills another experiment. Uses the existing wrench/goal/readiness/CSV interfaces.
"""
import argparse
import copy
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import re
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_EXPERIMENT_ROOT = next(
    (path / "experi" for path in (ROOT, *ROOT.parents) if path.name.lower() == "go2"),
    ROOT.parents[1] / "experi",
)
DEFAULT_EXPERIMENT_ROOT = Path(os.environ.get(
    "GO2_EXPERIMENT_ROOT", str(WORKSPACE_EXPERIMENT_ROOT)))
DEFAULT_OUTPUT_ROOT = DEFAULT_EXPERIMENT_ROOT / "results/v1_1"
METHODS = ("pure_mpc", "eth_adaptive_clf_mpc", "eth_reference_faithful", "v11_equivalent_wrench", "direct_wrench", "liu_van_mpc_port", "zhou_ssi")
# `eth` is retained only as a command-line compatibility alias.  New manifests
# always use the technically accurate direct_wrench method identity.
LEGACY_METHOD_ALIASES = {"eth": "direct_wrench"}
ADAPTIVE_METHODS = ("v5_residual_dynamics", "v6_variable_gain")
METHOD_DEFINITIONS = {
    "pure_mpc": {
        "public_name": "Pure MPC",
        "representation": "none",
        "fidelity": "nominal_mpc_nominal_wbc_no_adaptive_architecture",
        "update": "none",
        "adaptive_components": "disabled",
    },
    "v11_equivalent_wrench": {
        "public_name": "MyRBF V1.1",
        "representation": "six_independent_scalar_gaussian_rbf_25_plus_bias",
        "fidelity": "word_v1_1_fixed_gain_with_user_approved_soft_clf",
        "clf_mode": "soft_squared_hinge",
        "clf_penalty_weight": 0.01,
        "update": "negative_gamma_h_G_transpose_composite_velocity_error",
    },
    "eth_adaptive_clf_mpc": {
        "public_name": "ETH Adaptive CLF-MPC",
        "representation": "payload_inertial_10d_plus_wrench_6d",
        "fidelity": "repository_paper_regressor_16d_with_eq8_eq10_eq11",
        "clf_mode": "soft_relaxed_barrier",
        "update": "gamma_yu_transpose_sigma",
    },
    "eth_reference_faithful": {
        "public_name": "ETH Adaptive CLF-MPC (reference-faithful Stage 1)",
        "representation": "payload_inertial_10d_plus_wrench_6d",
        "fidelity": "ocs2_ws_reference_legacy_estimated_srbd_yin_i6_no_wbc_bridge",
        "clf_mode": "soft_relaxed_barrier",
        "update": "gamma_yu_transpose_sigma",
    },
    "v5_residual_dynamics": {
        "public_name": "MyRBF V5",
        "representation": "factorized_motion_history_contact_rbf",
        "fidelity": "2026_09_20_v5_dynamic_residual_revision",
        "update": "filtered_compensation_aware_rate_residual",
    },
    "v6_variable_gain": {
        "public_name": "MyRBF V6",
        "representation": "factorized_motion_history_contact_rbf",
        "fidelity": "v5_core_with_fy_uncertainty_gain_schedule_20260928",
        "update": "filtered_compensation_aware_rate_residual_variable_gain",
    },
    "v4_convergence": {
        "public_name": "MyRBF V4",
        "representation": "calibrated_residual_gaussian_rbf",
        "fidelity": "empirical_convergence_without_stability_claim",
        "update": "normalized_residual_prediction_with_forgetting",
    },
    "direct_wrench": {
        "public_name": "Direct Wrench",
        "representation": "direct_parameter",
        "fidelity": "6D constant-wrench direct adaptation comparator; not full Minniti ACLF-MPC",
        "update": "identity_regressor_tracking_adaptation",
    },
    "liu_van_mpc_port": {
        "public_name": "Liu-VAN",
        "representation": "dual_gaussian_rbf",
        "fidelity": "architecture_preserving_go2_port",
        "update": "composite_error_variable_step",
    },
    "zhou_ssi": {
        "public_name": "Zhou-RFF",
        "representation": "random_fourier_features",
        "fidelity": "interface_matched_rff_ogd",
        "update": "self_supervised_online_gradient_descent",
    },
}
MAIN_PROFILES = ("constant", "sine", "multisine")
PROFILES = ("none",) + MAIN_PROFILES
STUDIES = ("main", "full", "suite", "no_force", "torque", "advisor_force")
TRAJECTORIES = {
    "velocity": {"name": "constant_body_forward_velocity", "profiles": MAIN_PROFILES},
}
TERRAINS = {
    "flat": {"purpose": "flat_control_trial", "spawn": [0, 0, 0.35, 0],
             "motion": True, "surface_height_m": 0.0, "surface_normal_world": [0, 0, 1]},
    "slope": {"purpose": "stationary_terrain_feasibility", "spawn": [3, 0, 0.49, -math.radians(5)],
              "motion": False, "surface_height_m": 1.5 * math.tan(math.radians(5)),
              "surface_normal_world": [-math.sin(math.radians(5)), 0, math.cos(math.radians(5))]},
    "steps": {"purpose": "stationary_terrain_feasibility", "spawn": [6.8, 0, 0.39, 0],
              "motion": False, "surface_height_m": 0.04, "surface_normal_world": [0, 0, 1]},
}
CONFIG_DIR = ROOT / "legged_controllers/config/go2"
CAMERA_TOPIC = "/myrbf_scene/overview/image_raw"


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def profile_settings(profile, scale=1.0, axis="fy", amplitude_override=None):
    if profile not in PROFILES or axis not in ("fx", "fy", "tx") or not math.isfinite(scale) or scale < 0:
        raise ValueError("unknown force profile or invalid scale")
    if amplitude_override is not None and (profile != "sine" or axis != "tx" or
            not math.isfinite(amplitude_override) or amplitude_override <= 0):
        raise ValueError("torque amplitude must be finite and positive")
    return dict(profile=profile, axis=axis, body="go2::base", reference_frame="world",
                reference_point=[0.0, 0.0, 0.0],
                reference_point_source="gazebo_world_net_wrench_zero_offset",
                torque_about="go2::base link center of mass", delay_s=0.0,
                # A shared 5 N peak keeps every method inside the verified Go2
                # operating envelope.  The multisine splits that peak 3+2 N.
                value=5.0 * scale if profile == "constant" else 0.0,
                amplitude=(amplitude_override if amplitude_override is not None else
                           (5.0 if profile == "sine" else 3.0 if profile == "multisine" else 0.0) * scale),
                frequency=0.15, secondary_amplitude=2.0 * scale if profile == "multisine" else 0.0,
                secondary_frequency=0.30, phase_deg=90.0, secondary_phase_deg=90.0,
                # 20 Hz gives more than 130 samples per 0.15 Hz cycle. Each command
                # remains active for 0.25 s so a brief service delay cannot
                # silently withdraw the external force.
                rate_hz=20.0, command_hold_s=0.25, withdrawn_during_recording=False)


def run_matrix(args):
    if args.study == "suite":
        # One forward command x four disturbances x four methods.
        selection = copy.copy(args)
        selection.study, selection.terrain = "full", "flat"
        selection.trajectory = selection.profile = selection.method = "all"
        return run_matrix(selection)
    studies = (("main", "no_force") if args.study == "full" else
               ("no_force", "ablation") if args.study == "additions" else
               (args.study,))
    rows = []
    for study in studies:
        terrains = tuple(TERRAINS) if args.terrain == "all" else (args.terrain,)
        trajectory_ids = tuple(TRAJECTORIES) if args.trajectory == "all" else (args.trajectory,)
        for terrain in terrains:
            for trajectory_id in trajectory_ids:
                # Non-flat cases stand in place; T01/T02 must not duplicate them.
                if terrain != "flat" and trajectory_id != trajectory_ids[0]:
                    continue
                trajectory = TRAJECTORIES[trajectory_id]
                if study == "main":
                    profiles = trajectory["profiles"] if args.profile == "all" else (args.profile,)
                    methods = METHODS if args.method == "all" else (LEGACY_METHOD_ALIASES.get(args.method, args.method),)
                    variants = [(method, "full" if method in ADAPTIVE_METHODS else "standard")
                                for method in methods]
                elif study == "no_force":
                    profiles = ("none",)
                    variants = [(method, "full" if method in ADAPTIVE_METHODS else "standard")
                                for method in (METHODS if args.method == "all" else (LEGACY_METHOD_ALIASES.get(args.method, args.method),))]
                elif study == "advisor_force":
                    # Advisor generalization protocol: a fixed world-frame
                    # horizontal 20 N force, applied from t=6 s to experiment end.
                    profiles = ("constant",)
                    variants = [(method, "full" if method in ADAPTIVE_METHODS else "standard")
                                for method in (METHODS if args.method == "all" else (LEGACY_METHOD_ALIASES.get(args.method, args.method),))]
                elif study == "ablation":
                    if trajectory_id != "velocity":
                        continue
                    profiles = ("sine",)
                    variants = [("v5_residual_dynamics", "without_rbf")]
                else:
                    if trajectory_id != "velocity":
                        continue
                    profiles = ("sine",)
                    variants = [(method, "full" if method in ADAPTIVE_METHODS else "standard")
                                for method in METHODS]
                for profile in profiles:
                    if study == "main" and profile not in trajectory["profiles"]:
                        continue
                    for method, variant in variants:
                        mode = args.robust_mode if method in ADAPTIVE_METHODS else "none"
                        if study == "advisor_force":
                            disturbance = profile_settings(
                                "constant", 1.0, axis=args.advisor_axis)
                            disturbance["value"] = -10.0
                            disturbance["amplitude"] = 0.0
                            disturbance["delay_s"] = 6.0
                        else:
                            disturbance = profile_settings(
                                profile, args.force_scale,
                                axis="tx" if study == "torque" else "fy",
                                amplitude_override=args.torque_amplitude if study == "torque" else None)

                        total_duration = args.duration
                        move_duration = total_duration - args.motion_delay
                        terminal_hold = 0.0

                        # 外力从 delay_s 开始，并恰好持续到记录结束。
                        disturbance["duration_s"] = (
                            total_duration - disturbance["delay_s"]
                        )
                        if disturbance["duration_s"] <= 0:
                            raise ValueError(
                                "experiment must end after disturbance onset"
                            )
                        method_definition = dict(METHOD_DEFINITIONS[method])
                        if method == "v11_equivalent_wrench":
                            block = re.search(r"v11Feasibility\s*\{([^}]+)\}",
                                              (CONFIG_DIR / "task.info").read_text(encoding="utf-8"))
                            match = re.search(r"clfPenaltyWeight\s+([^\s;]+)", block.group(1)) if block else None
                            if not match or not math.isfinite(float(match.group(1))) or float(match.group(1)) <= 0:
                                raise ValueError("missing or invalid V1.1 CLF penalty weight in task.info")
                            method_definition["clf_penalty_weight"] = float(match.group(1))
                        rows.append({
                            "schema_version": 2, "experiment": "GO2-METHOD-COMPARISON",
                            "study": study, "variant": variant,
                            "method": method, "method_definition": method_definition,
                            "legacy_method_alias": (
                                args.method if args.method in LEGACY_METHOD_ALIASES else None),
                            "robust_mode": mode, "repetition": args.repetition_start,
                            "seed": args.seed, "terrain": terrain,
                            "terrain_conditions": TERRAINS[terrain],
                            "constraint_surface_normal_world": (
                                TERRAINS[terrain]["surface_normal_world"]
                                if method in ADAPTIVE_METHODS else [0, 0, 1]),
                            "constraint_comparison_note": "Terrain traversal is not claimed; controller structural differences remain documented.",
                            "robot": "go2", "simulator": "Gazebo Classic",
                            "physics_step_s": 0.001, "disturbance": disturbance,
                            "timing_s": {"total": total_duration,
                                         "goal_delay": args.motion_delay,
                                         "trajectory": move_duration,
                                         "final_hold": terminal_hold},
                            "gait": "trot" if terrain == "flat" else "initial_stance",
                            "planner": {
                                "source": "cmd_vel" if terrain == "flat" else "none",
                                "trajectory_id": trajectory_id,
                                "trajectory_name": trajectory["name"],
                                "frame": "body",
                                "command_vx_mps": args.vx if terrain == "flat" else 0.0,
                                "command_vy_mps": 0.0,
                                "command_yaw_rate_rps": 0.0,
                                "heading_reference": "hold_initial_heading" if terrain == "flat" else "none",
                                "terminal_hold_s": terminal_hold,
                                "com_height_m": 0.33 if terrain == "flat" else None,
                                "rate_hz": 20.0 if terrain == "flat" else None,
                                "reference_profile": "smooth_start_constant_velocity" if terrain == "flat" else None,
                                "ramp_duration_s": 2.0 if terrain == "flat" else 0.0,
                                "gait_lead_s": 0.6 if terrain == "flat" else 0.0,
                                "linear_velocity_target": "world_normalized_momentum_first3" if terrain == "flat" else None},
                            "force_estimate_convention": (
                                "none; pure nominal MPC/WBC has no adaptive wrench"
                                if method == "pure_mpc" else
                                "body_equivalent_uncertainty_about_com_left_hand"
                                if method == "v11_equivalent_wrench" else
                                "world_yupi_about_com_with_physical_wbc_translation_to_base_origin"
                                if method == "eth_adaptive_clf_mpc" else
                                "world_compensation_wrench_about_base_origin"),
                            "data_kind": "simulation", "status": "planned"})
    return rows


def command_for(case, case_dir, config_path):
    d = case["disturbance"]
    spawn = case["terrain_conditions"]["spawn"]
    values = {"case_dir": str(case_dir), "controller_config": str(config_path),
              "method": case["method"], "gui": str(case.get("gui", False)).lower(),
              "gazebo_seed": case["seed"], "total_duration": case["timing_s"]["total"],
              "motion_delay": case["timing_s"]["goal_delay"],
              "motion_duration": case["timing_s"]["trajectory"],
              "vx": case["planner"]["command_vx_mps"],
              "flat_motion": str(case["terrain_conditions"]["motion"]).lower(),
              "spawn_x": spawn[0], "spawn_y": spawn[1], "spawn_z": spawn[2], "spawn_pitch": spawn[3],
              "world": str(ROOT / "legged_gazebo/worlds" /
                           ("myrbf_flat_control.world" if case["terrain"] == "flat"
                            else "myrbf_multi_terrain.world")),
              "wrench_rate": d["rate_hz"], "wrench_hold": d["command_hold_s"],
              "wrench_axis": d["axis"],
              "wrench_delay": d["delay_s"],
              "wrench_duration": d["duration_s"]}
    for key in ("profile", "value", "amplitude", "frequency", "secondary_amplitude",
                "secondary_frequency", "phase_deg", "secondary_phase_deg"):
        values["wrench_" + key] = d[key]
    # Invoke the repository launch file by absolute path.  This avoids an
    # ambiguous duplicate discovery when the bind-mounted workspace is also
    # present in ROS_PACKAGE_PATH; nested $(find ...) remain package-resolved.
    return ["roslaunch", str(ROOT / "legged_controllers/launch/go2_method_comparison.launch")] + [
        "%s:=%s" % (k, v) for k, v in values.items()]


def frozen_config(source, method, robust_mode):
    import yaml
    if method not in METHODS:
        raise ValueError("Archived methods are not supported by the current entry")
    config = yaml.safe_load(Path(source).read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("controller config must be a YAML mapping")
    if method == "pure_mpc":
        if config:
            raise ValueError("pure_mpc config must not load adaptive parameters")
    elif method == "eth_adaptive_clf_mpc":
        expected = ("eth_adaptive_clf_mpc", "payload_inertial_10d_plus_wrench_6d",
                    "paper_regressor_compensation", "soft_relaxed_barrier")
        actual = (config.get("algorithm"), config.get("representation"),
                  config.get("adaptive_dynamics"), config.get("adaptive_clf"))
        if actual != expected:
            raise ValueError("ETH config must select the repository 16D paper implementation")
    elif method == "eth_reference_faithful":
        expected = ("eth_reference_faithful", "payload_inertial_10d_plus_wrench_6d",
                    "reference_legacy_estimated_srbd_rebuild", "[Y_in, I_6]",
                    "soft_relaxed_barrier", "disabled_stage1")
        actual = (config.get("algorithm"), config.get("representation"),
                  config.get("adaptive_dynamics"), config.get("regressor"),
                  config.get("adaptive_clf"), config.get("wbc_compensation"))
        if actual != expected:
            raise ValueError("reference-faithful ETH config must preserve legacy Y=[Y_in,I6], estimated-SRBD dynamics, and no WBC bridge")
    elif method == "v11_equivalent_wrench":
        if (config.get("algorithm") != method or
                config.get("learning_target") != "equivalent_uncertainty_wrench_body_com_left_hand" or
                config.get("control_interface") != "model_and_wbc"):
            raise ValueError("V1.1 requires the Word-defined equivalent wrench and model/WBC injection")
        if not .5 < config.get("v11_composite_ratio", 0) < 1:
            raise ValueError("V1.1 composite ratio must lie in (0.5,1)")
        for key, size in (("v11_input_scale",6),("v11_gamma",6),("v11_weight_limit",6),
                          ("v11_clf_decay",6),("v11_centers",25),("v11_widths",25)):
            values=config.get(key,[])
            if len(values)!=size or any(not isinstance(x,(int,float)) or not math.isfinite(x) for x in values):
                raise ValueError("invalid V1.1 parameter: "+key)
            if key != "v11_centers" and any(x<=0 for x in values):
                raise ValueError("nonpositive V1.1 parameter: "+key)
        if set(config).intersection({"prediction_gain","leakage","output_scale","v5_gamma","v6_gain_multiplier_max"}):
            raise ValueError("V1.1 does not accept legacy update terms")
    elif method in ADAPTIVE_METHODS:
        if config.get("algorithm") != method:
            raise ValueError("adaptive config algorithm does not match selected method")
        if config.get("learning_target") != "filtered_compensation_aware_rate_residual":
            raise ValueError("V5 must learn the filtered compensation-aware rate residual")
        if config.get("representation") != "factorized_motion_history_contact_rbf":
            raise ValueError("V5 requires factorized motion, history and contact RBF inputs")
        if config.get("control_interface") != "wrench_allocator":
            raise ValueError("V5 requires the common wrench allocator")
        if not config.get("compensation_aware_observer"):
            raise ValueError("V5 must subtract its known applied compensation")
        if config.get("v5_rbf_enabled") is not True:
            raise ValueError("the frozen source V5 config must enable RBF before a per-case ablation")
        for key, length in (("v5_input_scale", 12),
                            ("v5_history_scale", 12), ("v5_gamma", 6),
                            ("v5_leakage", 6), ("v5_weight_limit", 6),
                            ("v5_output_limit", 6)):
            values = config.get(key)
            if (not isinstance(values, list) or len(values) != length or
                    not all(isinstance(value, (int, float)) and
                            math.isfinite(value) for value in values)):
                raise ValueError("V5 %s must contain %d finite values" %
                                 (key, length))
        if any(value <= 0 for value in
               config["v5_input_scale"] + config["v5_history_scale"] +
               config["v5_gamma"] +
               config["v5_weight_limit"] + config["v5_output_limit"]):
            raise ValueError("V5 scales, gains and limits must be positive")
        if any(value < 0 for value in config["v5_leakage"]):
            raise ValueError("V5 leakage must be nonnegative")
        for key in ("v5_motion_width", "v5_history_width",
                    "v5_contact_width", "v5_input_clip",
                    "v5_residual_filter_tau"):
            value = config.get(key)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError("V5 %s must be finite and positive" % key)
        if method == "v6_variable_gain":
            multipliers = config.get("v6_gain_multiplier_max")
            scales = config.get("v6_uncertainty_scale")
            if (not isinstance(multipliers, list) or len(multipliers) != 6 or
                    not all(isinstance(x, (int, float)) and math.isfinite(x) and x >= 1 for x in multipliers)):
                raise ValueError("V6 multipliers must be six finite values >= 1")
            if (not isinstance(scales, list) or len(scales) != 6 or
                    not all(isinstance(x, (int, float)) and math.isfinite(x) and x > 0 for x in scales)):
                raise ValueError("V6 uncertainty scales must be six positive finite values")
            for key in ("v6_gain_attack_tau", "v6_gain_release_tau"):
                x = config.get(key)
                if not isinstance(x, (int, float)) or not math.isfinite(x) or x <= 0:
                    raise ValueError("V6 gain time constants must be positive")
    elif method == "v4_convergence":
        if config.get("algorithm") != "v4_convergence":
            raise ValueError("V4 requires the independent convergence config")
        if config.get("representation") != "calibrated_residual_gaussian_rbf":
            raise ValueError("V4 must use calibrated residual Gaussian RBF networks")
        if config.get("feature_mode") != "residual_only":
            raise ValueError("V4 must use residual-only RBF inputs")
        if not config.get("normalize_update"):
            raise ValueError("V4 requires a normalized update")
        if config.get("mpc_model_injection") or not config.get("v3_shared_allocator"):
            raise ValueError("V4 requires exactly the shared allocator delivery path")
        if config.get("control_interface") != "wrench_allocator":
            raise ValueError("V4 requires the wrench allocator")
        if config.get("clf_enabled") or config.get("clf_hard_constraint"):
            raise ValueError("V4 must not inherit the V2 hard CLF constraint")
        for key in ("residual_bias", "residual_deadzone", "leakage"):
            values = config.get(key)
            if (not isinstance(values, list) or len(values) != 6 or
                    not all(isinstance(value, (int, float)) and
                            math.isfinite(value) for value in values)):
                raise ValueError("V4 %s must contain six finite values" % key)
        if any(value < 0 for value in config["residual_deadzone"] + config["leakage"]):
            raise ValueError("V4 deadzones and leakage must be nonnegative")
        if any(config.get("prediction_gain", [])):
            raise ValueError("V4 must not use V3 concurrent-learning history")
        if robust_mode not in ("sign", "smooth"):
            raise ValueError("v4 robust mode must be sign or smooth")
        config["robust_mode"] = robust_mode
    elif method == "direct_wrench":
        if config.get("algorithm") != "v1":
            raise ValueError("direct-wrench comparator requires the shared V1-compatible adapter")
        if (config.get("estimator_version"), config.get("comparison_method"),
                config.get("representation")) != (
                    "eth_direct", "eth", "direct_parameter"):
            raise ValueError("direct-wrench comparator must use direct adaptation with no RBF representation")
        forbidden = {"linear_extent", "angular_extent", "width_ratio",
                     "tracking_gain", "prediction_gain", "leakage"}
        if forbidden.intersection(config):
            raise ValueError("direct-wrench config contains RBF-only or legacy update parameters")
    elif method in ("liu_van_mpc_port", "zhou_ssi"):
        if config.get("algorithm", "v1") != "v1":
            raise ValueError(method + " requires the V1-compatible comparison interface")
        expected = {
            "liu_van_mpc_port": ("liu_dual_rbf", "dual_gaussian_rbf"),
            "zhou_ssi": ("rff", "random_fourier_features"),
        }[method]
        if (config.get("estimator_version"), config.get("representation")) != expected:
            raise ValueError(method + " has the wrong estimator or representation")
        if config.get("comparison_method") != method:
            raise ValueError(method + " config identity is missing or mismatched")
    else:
        raise ValueError("unsupported method")
    return config


def config_source_for_case(args, case):
    return {"pure_mpc": args.pure_mpc_config,
            "eth_adaptive_clf_mpc": args.eth_config,
            "eth_reference_faithful": args.eth_reference_config,
            "v11_equivalent_wrench": args.v11_config,
            "direct_wrench": args.direct_wrench_config,
            "liu_van_mpc_port": args.liu_config,
            "zhou_ssi": args.zhou_config}[case["method"]]


def isolation_environment(case_dir, ros_port, gazebo_port):
    if ros_port == gazebo_port or not all(1024 <= p <= 65535 for p in (ros_port, gazebo_port)):
        raise ValueError("ROS/Gazebo need different unprivileged ports")
    env = os.environ.copy()
    # rosgraph's fallback scans every ROS package before opening the master
    # socket. On the Windows-mounted workspace this can exceed startup time.
    env.setdefault("ROS_PYTHON_LOG_CONFIG_FILE", "/opt/ros/noetic/etc/ros/python_logging.conf")
    # Do not inherit a mixed overlay from the caller.  roslaunch launches
    # xacro/rosout as children, which must see the same package roots and
    # Python modules as the parent runner.
    ros_root = Path("/opt/ros/noetic")
    workspace_src = ROOT.parent
    ros_share = ros_root / "share"
    ros_lib_python = ros_root / "lib/python3/dist-packages"
    # A source-space package exports the Gazebo hardware plugin XML, but its
    # compiled .so lives in an overlay.  A caller may provide an installed
    # *matching* overlay through this narrowly scoped environment variable.
    # It is intentionally optional: controller selection and all experiment
    # settings remain entirely in the frozen case configuration.
    gazebo_plugin_prefix = Path(env.get("GO2_GAZEBO_PLUGIN_PREFIX", ""))
    has_gazebo_plugin_prefix = (
        gazebo_plugin_prefix.is_dir()
        and (gazebo_plugin_prefix / "share/legged_gazebo/legged_hw_sim_plugins.xml").is_file()
        and (gazebo_plugin_prefix / "lib/liblegged_hw_sim.so").is_file())
    env["PATH"] = os.pathsep.join((str(ros_root / "bin"), env.get("PATH", "")))
    env["ROS_ROOT"] = str(ros_share / "ros")
    ros_package_roots = [str(workspace_src), str(ros_share)]
    if has_gazebo_plugin_prefix:
        ros_package_roots.insert(0, str(gazebo_plugin_prefix / "share"))
        env["GAZEBO_PLUGIN_PATH"] = os.pathsep.join(
            (str(gazebo_plugin_prefix / "lib"), env.get("GAZEBO_PLUGIN_PATH", "")))
        env["LD_LIBRARY_PATH"] = os.pathsep.join(
            (str(gazebo_plugin_prefix / "lib"), env.get("LD_LIBRARY_PATH", "")))
    env["ROS_PACKAGE_PATH"] = os.pathsep.join(ros_package_roots)
    # The ad-hoc Go2/ETH overlays predate the system Noetic prefix and their
    # setup scripts may omit it.  rospack can still resolve package.xml from
    # ROS_PACKAGE_PATH in that state, but roslaunch cannot resolve installed
    # node executables (notably rosout and rosbag) below /opt/ros/noetic/lib.
    # Keep the system prefix in every child process; this is environment
    # plumbing only and does not select or alter a controller implementation.
    prefixes = [path for path in env.get("CMAKE_PREFIX_PATH", "").split(os.pathsep) if path]
    if str(ros_root) not in prefixes:
        prefixes.append(str(ros_root))
    env["CMAKE_PREFIX_PATH"] = os.pathsep.join(prefixes)
    env["PYTHONPATH"] = os.pathsep.join(
        path for path in (str(ros_lib_python), env.get("PYTHONPATH", "")) if path)
    env.update(ROS_MASTER_URI="http://127.0.0.1:%d" % ros_port,
               GAZEBO_MASTER_URI="http://127.0.0.1:%d" % gazebo_port,
               ROS_IP="127.0.0.1", ROS_HOSTNAME="127.0.0.1",
               ROS_HOME=str(case_dir / "ros_home"), ROS_LOG_DIR=str(case_dir / "ros_logs"),
               GAZEBO_LOG_PATH=str(case_dir / "gazebo_logs"), GAZEBO_MODEL_DATABASE_URI="", ROBOT_TYPE="go2")
    return env


def config_for_case(shared_config, case):
    """Deep-copy per case: never let a slope normal contaminate a later flat run."""
    config = copy.deepcopy(shared_config)
    if case["method"] in ADAPTIVE_METHODS:
        config["surface_normal_world"] = list(case["terrain_conditions"]["surface_normal_world"])
    if case["method"] in ADAPTIVE_METHODS:
        config["v5_rbf_enabled"] = case.get("variant", "full") != "without_rbf"
    return config


def choose_ports():
    sockets = []
    try:
        for _ in range(2):
            sock = socket.socket()
            sock.bind(("127.0.0.1", 0))
            sockets.append(sock)
        return [sock.getsockname()[1] for sock in sockets]
    finally:
        for sock in sockets:
            sock.close()


@contextmanager
def experiment_lock():
    import fcntl
    # The existing generator writes /tmp/legged_control/go2.urdf. Serialise here
    # and refuse any already-running gzserver; never kill a foreign process.
    with open("/tmp/go2_method_comparison.lock", "a+") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("another Go2 comparison holds the experiment lock")
        for proc in Path("/proc").glob("[0-9]*/comm"):
            try:
                if proc.read_text().strip() == "gzserver":
                    raise RuntimeError("an existing gzserver is active; shared URDF generation prevents concurrent runs")
            except (FileNotFoundError, PermissionError):
                pass
        yield


def stop_group(process):
    # Also signal descendants when the group leader has already exited.
    # Large all-topic bags on a Windows-mounted result drive can need tens of
    # seconds to write their index and rename .active -> .bag after SIGINT.
    for sig, timeout in ((signal.SIGINT, 60), (signal.SIGTERM, 10), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.1)


def recover_raw_bag(case_dir):
    """Create a readable bag without altering the retained .active original."""
    finished = case_dir / "raw.bag"
    active = case_dir / "raw.bag.active"
    if finished.is_file() and finished.stat().st_size:
        return False
    if not active.is_file() or not active.stat().st_size:
        return False
    with tempfile.TemporaryDirectory(prefix="go2_rosbag_recovery_") as tmp:
        temporary = Path(tmp) / "raw.bag"
        shutil.copy2(active, temporary)
        result = subprocess.run(["rosbag", "reindex", str(temporary)], capture_output=True, text=True, timeout=180)
        if result.returncode or not temporary.is_file() or not temporary.stat().st_size:
            raise RuntimeError("rosbag recovery failed: " + (result.stderr or result.stdout).strip())
        check = subprocess.run(["rosbag", "info", "--yaml", str(temporary)], capture_output=True, text=True, timeout=60)
        if check.returncode or "messages:" not in check.stdout:
            raise RuntimeError("recovered rosbag failed readability check")
        shutil.copy2(temporary, finished)
    return True


def source_snapshot(destination):
    destination.mkdir()
    inventory = {}
    for directory in ("legged_controllers", "legged_interface", "legged_wbc", "legged_gazebo", "legged_common", "legged_estimation"):
        for path in sorted((ROOT / directory).rglob("*")):
            if path.is_file() and path.suffix in (".h", ".cpp", ".py", ".yaml", ".info", ".launch", ".world"):
                inventory[str(path.relative_to(ROOT)).replace("\\", "/")] = sha256(path)
    write_json(destination / "source_sha256.json", inventory)
    for relative in ("legged_controllers/launch/go2_method_comparison.launch",
                     "legged_gazebo/worlds/myrbf_flat_control.world",
                     "legged_gazebo/worlds/myrbf_multi_terrain.world",
                     "legged_controllers/scripts/run_go2_method_comparison.py",
                     "legged_controllers/scripts/plot_go2_method_comparison.py",
                     "legged_controllers/scripts/record_adaptive_wrench.py",
                     "legged_controllers/config/go2/task.info",
                     "legged_controllers/config/go2/reference.info",
                     "legged_controllers/config/go2/gait.info"):
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / relative, target)
    for filename, cmd in (("git_commit.txt", ["git", "rev-parse", "HEAD"]),
                          ("git_status.txt", ["git", "status", "--short"]),
                          ("git_diff.patch", ["git", "diff", "--no-ext-diff", "--binary"])):
        result = subprocess.run(cmd, cwd=ROOT, capture_output=True, timeout=30)
        (destination / filename).write_bytes(result.stdout + result.stderr)
    return inventory


def reusable_identity(case):
    keys = ("method", "robust_mode", "terrain", "terrain_conditions",
            "disturbance", "timing_s", "planner", "gait", "seed",
            "physics_step_s", "data_kind")
    value = {key: case[key] for key in keys}
    value["study"] = case.get("study", "main")
    value["variant"] = case.get(
        "variant", "full" if case["method"] == "v5_residual_dynamics" else "standard")
    return json.dumps(value, sort_keys=True)


def runtime_inventory():
    """Ignore plotting/runner edits, but invalidate reuse after any runtime source edit."""
    excluded = {"legged_controllers/scripts/plot_go2_method_comparison.py",
                "legged_controllers/scripts/run_go2_method_comparison.py"}
    inventory = {}
    for directory in ("legged_controllers", "legged_interface", "legged_wbc",
                      "legged_gazebo", "legged_common", "legged_estimation"):
        for path in (ROOT / directory).rglob("*"):
            if path.is_file() and path.suffix in (
                    ".h", ".cpp", ".py", ".yaml", ".info", ".launch", ".world"):
                relative = str(path.relative_to(ROOT)).replace("\\", "/")
                if relative not in excluded:
                    inventory[relative] = sha256(path)
    return inventory


def reusable_record(output_root, case, config_digest, current_sources):
    if not output_root.is_dir():
        return None
    for manifest_path in sorted(output_root.glob("*/*/case_manifest.json"),
                                reverse=True):
        case_dir = manifest_path.parent
        try:
            old = json.loads(manifest_path.read_text(encoding="utf-8"))
            if (old.get("status") != "recorded_validated" or old.get("source_drift") or
                    reusable_identity(old) != reusable_identity(case) or
                    old.get("controller_config_sha256") != config_digest):
                continue
            if not json.loads((case_dir / "validation.json").read_text())["valid"]:
                continue
            frozen = case_dir / "controller_config.yaml"
            if sha256(frozen) != config_digest:
                continue
            old_sources = json.loads(
                (case_dir / "frozen/source_sha256.json").read_text())
            old_sources = {key: value for key, value in old_sources.items()
                           if key not in (
                               "legged_controllers/scripts/plot_go2_method_comparison.py",
                               "legged_controllers/scripts/run_go2_method_comparison.py")}
            if old_sources == current_sources:
                return case_dir
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return None


def failed_before_campaign_start(case_dir):
    """Only a timed-out start signal permits a setup retry, never a robot run."""
    evidence = case_dir / "evidence.log"
    return (not (case_dir / "goal.json").exists() and
            not (case_dir / "recording_finished.json").exists() and
            evidence.is_file() and
            "timeout exceeded while waiting for message on topic /myrbf_campaign_start" in
            evidence.read_text(encoding="utf-8", errors="replace"))


def ros_support(case_dir, duration):
    """Separate ROS child: freeze parameters and optionally save a sensor image."""
    import rospy
    import rosbag
    import cv2
    from cv_bridge import CvBridge
    from gazebo_msgs.srv import GetModelState
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Float64
    import yaml
    rospy.init_node("go2_comparison_evidence", disable_signals=True)
    def simulation_time(timeout=15):
        return rospy.wait_for_message("/clock", Clock, timeout=timeout).clock.to_sec()
    start = rospy.wait_for_message("/myrbf_campaign_start", Float64, timeout=240).data
    (case_dir / "effective_rosparams.yaml").write_text(yaml.safe_dump(rospy.get_param("/")), encoding="utf-8")
    rospy.wait_for_service("/gazebo/get_model_state", timeout=30)
    get_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    while simulation_time() - start < 1.0:
        time.sleep(0.05)
    model = get_model("go2", "world")
    if not model.success:
        raise RuntimeError("Go2 has not spawned; refusing an empty-scene screenshot")
    requested = simulation_time()
    try:
        msg = rospy.wait_for_message(CAMERA_TOPIC, Image, timeout=3)
        if msg.header.stamp.to_sec() < requested:
            raise RuntimeError("camera image is stale")
        image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if not cv2.imwrite(str(case_dir / "gazebo_scene.png"), image):
            raise RuntimeError("failed to save real Gazebo camera image")
        info = rospy.wait_for_message(CAMERA_TOPIC.replace("image_raw", "camera_info"), CameraInfo, timeout=3)
        with rosbag.Bag(str(case_dir / "gazebo_scene.bag"), "w") as bag:
            bag.write(CAMERA_TOPIC, msg, msg.header.stamp)
            bag.write(CAMERA_TOPIC.replace("image_raw", "camera_info"), info, info.header.stamp)
        write_json(case_dir / "gazebo_scene.json", {
            "source": "Gazebo ROS camera sensor (unmodified pixels)", "topic": CAMERA_TOPIC,
            "stamp_ros": msg.header.stamp.to_sec(), "campaign_start_ros": start,
            "encoding": msg.encoding, "width": msg.width, "height": msg.height,
            "go2_spawn_verified": True, "base_world_m": [model.pose.position.x, model.pose.position.y, model.pose.position.z],
            "K": list(info.K), "frame_id": msg.header.frame_id,
        })
    except Exception as error:
        # Headless numerical runs must not be discarded solely because OGRE did
        # not publish a frame. A separate, real Gazebo multi-terrain preview is
        # maintained in the result package; record this absence explicitly.
        write_json(case_dir / "gazebo_scene_unavailable.json", {
            "camera_optional": True, "reason": str(error), "topic": CAMERA_TOPIC,
            "campaign_start_ros": start, "go2_spawn_verified": True,
            "base_world_m": [model.pose.position.x, model.pose.position.y, model.pose.position.z],
        })
    finished = simulation_time()
    while not rospy.is_shutdown() and finished - start < duration + 1.5:
        finished = simulation_time()
    write_json(case_dir / "recording_finished.json", {"campaign_start_ros": start, "finished_ros": finished})
    return 0


def execute_case(case, case_dir, config, timeout_s):
    import yaml
    config = config_for_case(config, case)
    case_dir.mkdir(parents=True, exist_ok=False)
    frozen = case_dir / "controller_config.yaml"
    frozen.write_text(yaml.safe_dump(config, sort_keys=True), encoding="utf-8")
    case["controller_config_sha256"] = sha256(frozen)
    inventory = source_snapshot(case_dir / "frozen")
    case["source_inventory_sha256"] = sha256(case_dir / "frozen/source_sha256.json")
    ports = choose_ports()
    env = isolation_environment(case_dir, *ports)
    case["environment"] = {k: env[k] for k in ("ROS_MASTER_URI", "GAZEBO_MASTER_URI", "ROS_HOME", "ROS_LOG_DIR", "GAZEBO_LOG_PATH", "ROS_PYTHON_LOG_CONFIG_FILE", "DISPLAY") if k in env}
    command = command_for(case, case_dir, frozen)
    case.update(command=command, status="running", started_utc=datetime.now(timezone.utc).isoformat())
    manifest = case_dir / "case_manifest.json"
    write_json(manifest, case)
    processes, handles = [], []

    def start(cmd, name):
        handle = (case_dir / name).open("w", encoding="utf-8")
        handles.append(handle)
        process = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    try:
        core = start(["roscore", "-p", str(ports[0])], "roscore.log")
        deadline = time.monotonic() + 30
        while True:
            try:
                with socket.create_connection(("127.0.0.1", ports[0]), timeout=0.2):
                    break
            except OSError:
                if core.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("private ROS master did not start")
                time.sleep(0.1)
        bag = start(["rosbag", "record", "-a", "-x", "/myrbf_scene/.*", "-O", str(case_dir / "raw.bag")], "rosbag.log")
        helper = start([sys.executable, str(Path(__file__).resolve()), "--ros-support", str(case_dir), "--duration", str(case["timing_s"]["total"])], "evidence.log")
        launch = start(command, "run.log")
        deadline = time.monotonic() + timeout_s
        log_position, log_tail = 0, ""
        next_failure_check = time.monotonic()
        while helper.poll() is None:
            if launch.poll() is not None or core.poll() is not None or bag.poll() is not None:
                raise RuntimeError("launch/master/bag exited before recording finished")
            if time.monotonic() > deadline:
                raise TimeoutError("wall-time watchdog exceeded; all owned processes will stop")
            if time.monotonic() >= next_failure_check:
                with (case_dir / "run.log").open(encoding="utf-8", errors="replace") as log:
                    log.seek(log_position)
                    log_tail += log.read()
                    log_position = log.tell()
                for fatal in ("[V2] Control cycle exception:", "[V2] MPC solver failed;",
                              "[WBC] Solve failed;", "[WBC] Invalid primal solution;"):
                    if fatal in log_tail:
                        raise RuntimeError("controller stopped: " + fatal + " (see run.log)")
                log_tail = log_tail[-256:]
                next_failure_check = time.monotonic() + 1.0
            time.sleep(0.2)
        if helper.returncode:
            raise RuntimeError("evidence capture failed; inspect evidence.log")
        case["status"] = "recorded_pending_validation"
    except BaseException as error:
        case["status"] = "failed"
        case["error"] = str(error)
        raise
    finally:
        for process in reversed(processes):
            stop_group(process)
        for handle in handles:
            handle.close()
        try:
            case["raw_bag_recovered_from_active"] = recover_raw_bag(case_dir)
        except Exception as recovery_error:
            case["raw_bag_recovery_error"] = str(recovery_error)
        case["finished_utc"] = datetime.now(timezone.utc).isoformat()
        drift = [relative for relative, digest in inventory.items()
                 if not (ROOT / relative).is_file() or sha256(ROOT / relative) != digest]
        case["source_drift"] = drift
        if drift:
            case["status"] = "failed"
            case["error"] = "source changed while run was in progress"
        write_json(manifest, case)
    from plot_go2_method_comparison import validate_case
    try:
        result = validate_case(case_dir)
        if not result["valid"] or case["source_drift"]:
            raise RuntimeError("recorded evidence did not pass validation: " + "; ".join(result["errors"]))
        case["status"] = "recorded_validated"
        write_json(case_dir / "validation.json", result)
        write_json(manifest, case)
    except Exception as error:
        case.update(status="failed", error=str(error))
        write_json(manifest, case)
        raise


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--plan", "--dry-run", action="store_true")
    p.add_argument("--method", choices=("all",) + METHODS + tuple(LEGACY_METHOD_ALIASES), default="all")
    p.add_argument("--study", choices=STUDIES, default="main",
                   help="full/suite=16 flat constant-velocity conditions; no planned paths")
    p.add_argument("--robust-mode", choices=("sign", "smooth"), default="smooth")
    p.add_argument("--profile", choices=("all",) + PROFILES, default="all")
    p.add_argument("--trajectory", choices=("all",) + tuple(TRAJECTORIES), default="all")
    p.add_argument("--terrain", choices=("all",) + tuple(TERRAINS), default="flat")
    p.add_argument("--repetitions", type=int, default=1)
    p.add_argument("--repetition-start", type=int, default=1)
    p.add_argument("--seed", type=int, default=4102)
    p.add_argument("--duration", type=float, default=33.0)
    p.add_argument("--motion-delay", type=float, default=3.0)
    p.add_argument("--vx", type=float, default=0.45, help="constant forward BODY velocity in m/s")
    p.add_argument("--force-scale", type=float, default=1.0)
    p.add_argument("--advisor-axis", choices=("fx", "fy"), default="fx",
                   help="world-force axis for the fixed 20 N advisor protocol")
    p.add_argument("--torque-amplitude", type=float,
                   help="required positive Nm amplitude for the optional Tx sine study")
    p.add_argument("--wall-timeout", type=float, default=300.0)
    p.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    p.add_argument("--v11-config", type=Path, default=CONFIG_DIR / "myrbf_v11_equivalent_wrench_go2.yaml")
    p.add_argument("--pure-mpc-config", type=Path, default=CONFIG_DIR / "pure_mpc_go2.yaml")
    p.add_argument("--eth-config", type=Path, default=CONFIG_DIR / "eth_adaptive_clf_mpc_go2.yaml")
    p.add_argument("--eth-reference-config", type=Path, default=CONFIG_DIR / "eth_reference_faithful_go2.yaml")
    p.add_argument("--direct-wrench-config", dest="direct_wrench_config", type=Path,
                   default=CONFIG_DIR / "myrbf_shadow_eth_go2_closed_loop.yaml")
    p.add_argument("--liu-config", type=Path, default=CONFIG_DIR / "myrbf_shadow_liu_hybrid_go2.yaml")
    p.add_argument("--zhou-config", type=Path, default=CONFIG_DIR / "myrbf_shadow_zhou_ssi_rff_go2.yaml")
    p.add_argument("--gui", action="store_true")
    p.add_argument("--continue-on-error", action="store_true",
                   help="record a failed case and continue the remaining matrix")
    p.add_argument("--fresh", action="store_true",
                   help="run every selected condition once instead of reusing earlier results")
    p.add_argument("--ros-support", type=Path, help=argparse.SUPPRESS)
    args = p.parse_args(argv)
    if args.repetitions != 1 or args.repetition_start != 1:
        p.error("each experiment condition must run exactly once")
    if args.study == "suite" and args.trajectory != "all":
        p.error("suite runs all experiments; use a single-experiment entry to select a trajectory")
    if args.study not in ("main", "no_force", "advisor_force") and (
            args.terrain != "flat" or args.method != "all" or args.profile != "all"):
        p.error("added studies require --terrain flat --method all --profile all")
    if args.study == "advisor_force" and (
            args.terrain != "flat" or args.profile != "all"):
        p.error("advisor_force requires flat terrain and its fixed constant-force profile")
    if args.study == "no_force" and args.profile != "all":
        p.error("no-force study selects profile none; omit --profile")
    if args.study == "torque":
        if args.torque_amplitude is None or not math.isfinite(args.torque_amplitude) or args.torque_amplitude <= 0:
            p.error("--study torque requires an explicit positive --torque-amplitude in Nm")
    elif args.torque_amplitude is not None:
        p.error("--torque-amplitude is only valid with --study torque")
    if not math.isfinite(args.motion_delay) or args.motion_delay < 0:
        p.error("motion delay must be finite and nonnegative")
    if not math.isfinite(args.duration) or args.duration <= args.motion_delay:
        p.error("duration must exceed motion delay")
    if not math.isfinite(args.vx) or args.vx <= 0:
        p.error("forward velocity must be finite and positive")
    if not math.isfinite(args.force_scale) or args.force_scale < 0:
        p.error("force scale must be finite and nonnegative")
    if args.seed < 0 or args.seed + args.repetition_start + args.repetitions - 2 > 2**32 - 1:
        p.error("paired Gazebo seeds must fit an unsigned 32-bit integer")
    if not math.isfinite(args.wall_timeout) or args.wall_timeout < args.duration + 60:
        p.error("wall timeout must allow duration plus 60 s setup")
    return args


def main(argv=None):
    args = parse_args(argv)
    if args.ros_support:
        return ros_support(args.ros_support, args.duration)
    cases = run_matrix(args)
    if not cases:
        raise ValueError("the requested study/trajectory/profile combination has no cases")
    if not args.execute:
        print(json.dumps({"execution": False, "cases": cases}, indent=2))
        return 0
    if os.name != "posix":
        raise RuntimeError("execute inside the sourced ROS Noetic Linux container")
    # Rendering is required even with gui=false. User/main owns Xvfb provisioning.
    if not os.environ.get("DISPLAY"):
        raise RuntimeError("camera evidence requires DISPLAY (e.g. an existing Xvfb display)")
    actual_package = subprocess.check_output(["rospack", "find", "legged_controllers"], text=True).strip()
    if Path(actual_package).resolve() != (ROOT / "legged_controllers").resolve():
        raise RuntimeError("ROS overlay resolves another worktree; source the intended workspace")
    # Source execution of new ROS scripts needs executable permission; no CMake edit.
    recorder = ROOT / "legged_controllers/scripts/record_adaptive_wrench.py"
    if not os.access(recorder, os.X_OK):
        raise RuntimeError("make record_adaptive_wrench.py executable in the container before launch")
    configs = {}
    for case in cases:
        key = case["method"], case["robust_mode"]
        source = config_source_for_case(args, case)
        if key not in configs:
            configs[key] = frozen_config(source, *key)
        case.update(gui=args.gui, controller_config_source=str(source.resolve()),
                    controller_config_source_sha256=sha256(source))
    import yaml
    output_root = args.output_root.resolve()
    with experiment_lock():
        current_sources = runtime_inventory()
        pending, reused = [], []
        for case in cases:
            config = config_for_case(
                configs[case["method"], case["robust_mode"]], case)
            digest = hashlib.sha256(
                yaml.safe_dump(config, sort_keys=True).encode("utf-8")).hexdigest()
            previous = (None if args.fresh else
                        reusable_record(output_root, case, digest, current_sources))
            if previous is None:
                pending.append(case)
            else:
                reused.append({"path": str(previous), "study": case["study"],
                               "variant": case["variant"]})
        if not pending:
            print(json.dumps({"new_runs": 0, "reused_validated": reused}, indent=2))
            return 0
        campaign = output_root / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + "_" + uuid.uuid4().hex[:8])
        campaign.mkdir(parents=True, exist_ok=False)
        write_json(campaign / "campaign_plan.json", {"schema_version": 1, "cases": pending})
        print(str(campaign), flush=True)
        failures = []
        case_index = list(reused)
        for case in pending:
            name = "%s__%s_%s_%s_%s__%s_%s__r%03d" % (
                case["study"], case["terrain"], case["planner"]["trajectory_id"],
                case["disturbance"]["axis"], case["disturbance"]["profile"],
                case["method"], case["variant"], case["repetition"])
            case_dir = campaign / name
            case_index.append({"path": str(case_dir), "study": case["study"],
                               "variant": case["variant"]})
            try:
                # Exactly one attempt per condition, including startup failures.
                execute_case(case, case_dir,
                             configs[case["method"], case["robust_mode"]],
                             args.wall_timeout)
            except Exception as error:
                failures.append({"case": name, "error": str(error)})
                if not args.continue_on_error:
                    raise
        write_json(campaign / "campaign_case_index.json", {"cases": case_index})
        result = {"requested": len(cases), "new_runs": len(pending),
                  "reused_validated": len(reused), "failed": failures,
                  "validated": len(cases) - len(failures), "delivery_complete": False}
        write_json(campaign / "campaign_result.json", result)
        figures = campaign / "figures_final"
        with (campaign / "plot.log").open("w", encoding="utf-8") as log:
            plotted = subprocess.run([sys.executable, str(Path(__file__).with_name("plot_go2_method_comparison.py")),
                "--case-index", str(campaign / "campaign_case_index.json"), "--output", str(figures),
                "--dpi", "300"], stdout=log, stderr=subprocess.STDOUT)
        expected = 2 * len({(c["study"], c["terrain"], c["disturbance"]["profile"],
                             c["disturbance"]["axis"]) for c in cases})
        result.update(figure_directory=str(figures), plot_return_code=plotted.returncode,
                      expected_figure_pairs=expected, png_count=len(list(figures.glob("*.png"))),
                      svg_count=len(list(figures.glob("*.svg"))))
        result["delivery_complete"] = (not failures and plotted.returncode == 0 and
            result["png_count"] == expected and result["svg_count"] == expected)
        write_json(campaign / "campaign_result.json", result)
        if failures:
            print("Campaign completed with %d failed cases: %s" %
                  (len(failures), campaign), flush=True)
            return 1
        if not result["delivery_complete"]:
            print("Recordings saved but figure delivery incomplete: " + str(campaign), flush=True)
            return 1
    print("Recorded and validated campaign: " + str(campaign))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
