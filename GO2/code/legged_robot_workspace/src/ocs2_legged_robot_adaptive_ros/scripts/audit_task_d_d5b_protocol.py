#!/usr/bin/env python3
"""Freeze the controlled Stage D5B nominal-bias attribution protocol."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
GROUPS = {
    "nominal_stance_hold": {
        "source_mode": "nominal", "scheduler_condition": "stance_hold",
        "payload": False, "target": False, "walking": False,
    },
    "nominal_stance_target": {
        "source_mode": "nominal", "scheduler_condition": "stance_target",
        "payload": False, "target": True, "walking": False,
    },
    "nominal_walk_target": {
        "source_mode": "nominal", "scheduler_condition": "walk_target",
        "payload": False, "target": True, "walking": True,
    },
    "uncertain_walk_target": {
        "source_mode": "uncertain_adaptive", "scheduler_condition": "walk_target",
        "payload": True, "target": True, "walking": True,
    },
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_mode_module():
    spec = importlib.util.spec_from_file_location(
        "task_d_mode", PACKAGE / "scripts" / "set_task_d_mode.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_protocol(d5a_summary):
    mode = load_mode_module()
    base_text = mode.BASE_CONFIG.read_text()
    configs = {}
    adaptive_hashes = set()
    for group, role in GROUPS.items():
        text = mode.materialize_text(base_text, role["source_mode"])
        config = mode.effective_config(text, role["source_mode"])
        configs[group] = config
        adaptive_hashes.add(hashlib.sha256(text.encode()).hexdigest())
    fixed = {json.dumps(value["fixed_files"], sort_keys=True)
             for value in configs.values()}
    controller_flags_ok = all(
        value["enableParameterUpdate"] is True
        and value["enableAdaptiveDynamics"] is True
        and value["enableClfConstraint"] is True
        and value["parameterization"] == "payload_inertial_10d"
        for value in configs.values())
    passed = bool(d5a_summary.get("passed") and len(adaptive_hashes) == 1
                  and len(fixed) == 1 and controller_flags_ok)
    return {
        "schema_version": 1,
        "stage": "D5B_nominal_bias_attribution_protocol",
        "passed": passed,
        "groups": GROUPS,
        "controller_flags": {
            "enableParameterUpdate": True,
            "enableAdaptiveDynamics": True,
            "enableClfConstraint": True,
            "parameterization": "payload_inertial_10d",
            "gamma": "I10_untuned",
        },
        "fixed_schedule": {
            "target_start_mpc_time_s": 1.2,
            "target_transition_duration_s": 0.2,
            "target_relative_x_step_m": 0.1,
            "formal_analysis_window_s": [1.2, 16.2],
            "formal_acquisition_end_mpc_time_s": 16.4,
            "common_grid_dt_s": 0.02,
        },
        "controlled_contrasts": {
            "target_without_gait": "nominal_stance_target - nominal_stance_hold",
            "walking_given_target": "nominal_walk_target - nominal_stance_target",
            "payload_given_walking": "uncertain_walk_target - nominal_walk_target",
        },
        "interpretation_boundary": (
            "Closed-loop contrasts are not assumed additive. They attribute changes "
            "to controlled conditions but are not a linear causal decomposition."),
        "checks": {
            "d5a_passed": bool(d5a_summary.get("passed")),
            "one_identical_adaptive_config": len(adaptive_hashes) == 1,
            "fixed_controller_files_identical": len(fixed) == 1,
            "controller_flags_locked": controller_flags_ok,
        },
        "adaptive_config_sha256": sorted(adaptive_hashes),
        "fixed_files": next(iter(configs.values()))["fixed_files"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--d5a-summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    d5a = json.loads(args.d5a_summary.read_text())
    result = build_protocol(d5a)
    result["d5a_summary_path"] = str(args.d5a_summary.resolve())
    result["d5a_summary_sha256"] = digest(args.d5a_summary)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
