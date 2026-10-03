#!/usr/bin/env python3
"""Generate and audit the three Stage D3 payload-inertial groups."""

import argparse
import difflib
import hashlib
import json
import math
import re
import sys
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE_DIR.parents[1]
BASE_CONFIG = (PACKAGE_DIR.parent / "ocs2_legged_robot_adaptive" / "config" /
               "adaptive_settings_payload_inertial_10d_smoke.info")
MODE_KEYS = ("enableParameterUpdate", "enableAdaptiveDynamics")
PAYLOAD = {
    "mass_kg": 5.213485,
    "com_nominal_c0_base_m": [0.16, -0.03, 0.09],
    "inertia_com_base_kg_m2": [0.08, 0.0, 0.0, 0.10, 0.0, 0.12],
}
GROUPS = {
    "nominal": {
        "enableParameterUpdate": True,
        "enableAdaptiveDynamics": True,
        "truePlantPayloadInertialEnabled": False,
    },
    "uncertain_off": {
        "enableParameterUpdate": False,
        "enableAdaptiveDynamics": False,
        "truePlantPayloadInertialEnabled": True,
    },
    "uncertain_adaptive": {
        "enableParameterUpdate": True,
        "enableAdaptiveDynamics": True,
        "truePlantPayloadInertialEnabled": True,
    },
}
LOCKED_SCALARS = {
    "enableClfConstraint": True,
    "enableInputBiasCost": False,
    "useSoftConstraint": True,
    "softConstraintMu": 0.1,
    "softConstraintDelta": 5.0,
    "timestampTolerance": 1.0e-9,
    "parameterization": "payload_inertial_10d",
    "minimumEstimatedTotalMass": 1.0,
}


def sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def sha256_file(path):
    return sha256_bytes(Path(path).read_bytes())


def scalar_token(text, key):
    match = re.search(r"^\s*" + re.escape(key) + r"\s+(\S+)", text,
                      re.MULTILINE)
    if not match:
        raise RuntimeError("missing setting " + key)
    return match.group(1).rstrip(";")


def parse_scalar(token):
    if token in ("true", "false"):
        return token == "true"
    try:
        return float(token)
    except ValueError:
        return token


def matrix_entries(text, key):
    match = re.search(r"^\s*" + re.escape(key) + r"\s*\{", text,
                      re.MULTILINE)
    if not match:
        raise RuntimeError("missing matrix " + key)
    begin = text.find("{", match.start())
    end = text.find("}", begin + 1)
    if end < 0:
        raise RuntimeError("unterminated matrix " + key)
    result = {}
    for row, column, value in re.findall(
            r"\((\d+),(\d+)\)\s+([-+0-9.eE]+)", text[begin + 1:end]):
        result[(int(row), int(column))] = float(value)
    return result


def effective_matrix(text, key, rows, columns, identity_default=False):
    matrix = [[1.0 if identity_default and row == column else 0.0
               for column in range(columns)] for row in range(rows)]
    for (row, column), value in matrix_entries(text, key).items():
        if row >= rows or column >= columns:
            raise RuntimeError("matrix index out of range: " + key)
        matrix[row][column] = value
    return matrix


def close(left, right):
    return math.isclose(float(left), float(right), rel_tol=0.0,
                        abs_tol=1.0e-12)


def validate_base(text):
    errors = []
    for key, expected in LOCKED_SCALARS.items():
        actual = parse_scalar(scalar_token(text, key))
        if isinstance(expected, float):
            if not isinstance(actual, float) or not close(actual, expected):
                errors.append("{}={!r}, expected {!r}".format(key, actual, expected))
        elif actual != expected:
            errors.append("{}={!r}, expected {!r}".format(key, actual, expected))
    matrices = {
        "lambdaLinear": effective_matrix(text, "lambdaLinear", 3, 3, True),
        "lambdaAngular": effective_matrix(text, "lambdaAngular", 3, 3, True),
        "gamma": effective_matrix(text, "gamma", 10, 10, True),
        "kd": effective_matrix(text, "kd", 6, 6, True),
    }
    expected_diagonals = {
        "lambdaLinear": [2.0] * 3,
        "lambdaAngular": [1.5] * 3,
        "gamma": [1.0] * 10,
        "kd": [1.0] * 6,
    }
    for key, diagonal in expected_diagonals.items():
        matrix = matrices[key]
        for row in range(len(matrix)):
            for column in range(len(matrix[row])):
                expected = diagonal[row] if row == column else 0.0
                if not close(matrix[row][column], expected):
                    errors.append("{} is not the frozen D2 baseline".format(key))
                    break
    initial_entries = matrix_entries(text, "initialParameterEstimate")
    initial = [initial_entries.get((index, 0), 0.0) for index in range(10)]
    if any(not close(value, 0.0) for value in initial):
        errors.append("initialParameterEstimate must be zero")
    if errors:
        raise RuntimeError("invalid D3 baseline: " + "; ".join(errors))
    return matrices, initial


def materialize_text(base_text, group):
    validate_base(base_text)
    result = base_text
    for key in MODE_KEYS:
        replacement = "true" if GROUPS[group][key] else "false"
        pattern = r"^(\s*" + re.escape(key) + r"\s+)(true|false)(\s*(?:;.*)?$)"
        result, count = re.subn(pattern, r"\g<1>" + replacement + r"\g<3>",
                                result, flags=re.MULTILINE)
        if count != 1:
            raise RuntimeError("mode key must occur exactly once: " + key)
    validate_base(result)
    return result


def neutral_text(text):
    result = text
    for key in MODE_KEYS:
        pattern = r"^(\s*" + re.escape(key) + r"\s+)(true|false)(\s*(?:;.*)?$)"
        result, count = re.subn(pattern, r"\g<1><D3_MODE>\g<3>", result,
                                flags=re.MULTILINE)
        if count != 1:
            raise RuntimeError("cannot normalize " + key)
    return result


def fixed_files():
    paths = {
        "mpc_task": WORKSPACE / "src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/mpc/task.info",
        "reference": WORKSPACE / "src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/command/reference.info",
        "gait": WORKSPACE / "src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/command/gait.info",
        "urdf": WORKSPACE / "src/ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf",
    }
    return {name: {"path": str(path), "sha256": sha256_file(path)}
            for name, path in paths.items()}


def effective_config(text, group):
    matrices, initial = validate_base(text)
    config = {key: parse_scalar(scalar_token(text, key))
              for key in tuple(MODE_KEYS) + tuple(LOCKED_SCALARS)}
    config.update({
        "schema_version": 3,
        "group": group,
        "truePlantPayloadInertialEnabled":
            GROUPS[group]["truePlantPayloadInertialEnabled"],
        "payload": PAYLOAD,
        "lambdaLinear": matrices["lambdaLinear"],
        "lambdaAngular": matrices["lambdaAngular"],
        "gamma": matrices["gamma"],
        "kd": matrices["kd"],
        "initialParameterEstimate": initial,
        "adaptive_config_sha256": sha256_bytes(text.encode()),
        "mode_neutral_adaptive_config_sha256":
            sha256_bytes(neutral_text(text).encode()),
        "fixed_files": fixed_files(),
        "target": {"relative_x_step_m": 0.10, "start_mpc_time_s": 1.20,
                   "transition_duration_s": 0.20,
                   "target_displacement_velocity_m_s": 0.50},
        "gait": {"name": "static_walk", "mode_sequence": [13, 7, 14, 11],
                 "switching_times_s": [0.0, 0.3, 0.6, 0.9, 1.2]},
        "artificial_matched_wrench_enabled": False,
    })
    return config


def write(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)


def emit(args):
    text = materialize_text(args.base_config.read_text(), args.group)
    config = effective_config(text, args.group)
    write(args.output_config, text)
    write(args.output_json, json.dumps(config, indent=2, sort_keys=True) + "\n")
    print(json.dumps(config, indent=2, sort_keys=True))


def audit(args):
    base = args.base_config.read_text()
    texts = {group: materialize_text(base, group) for group in GROUPS}
    configs = {group: effective_config(text, group)
               for group, text in texts.items()}
    expected = {
        "nominal__uncertain_adaptive": ["truePlantPayloadInertialEnabled"],
        "uncertain_off__uncertain_adaptive":
            ["enableParameterUpdate", "enableAdaptiveDynamics"],
        "nominal__uncertain_off": ["enableParameterUpdate",
                                     "enableAdaptiveDynamics",
                                     "truePlantPayloadInertialEnabled"],
    }
    relevant = list(MODE_KEYS) + ["truePlantPayloadInertialEnabled"]
    pairwise = {}
    diff_text = []
    for name, expected_keys in expected.items():
        left, right = name.split("__")
        changed = [key for key in relevant
                   if configs[left][key] != configs[right][key]]
        pairwise[name] = {"changed_keys": changed,
                          "expected_changed_keys": expected_keys,
                          "passed": changed == expected_keys}
        diff_text.append("===== {} vs {} =====\n".format(left, right))
        diff_text.extend(difflib.unified_diff(
            texts[left].splitlines(True), texts[right].splitlines(True),
            fromfile=left + ".info", tofile=right + ".info"))
    neutral_hashes = {config["mode_neutral_adaptive_config_sha256"]
                      for config in configs.values()}
    fixed_hashes = {json.dumps(config["fixed_files"], sort_keys=True)
                    for config in configs.values()}
    payloads = {json.dumps(config["payload"], sort_keys=True)
                for config in configs.values()}
    passed = (len(neutral_hashes) == 1 and len(fixed_hashes) == 1
              and len(payloads) == 1
              and all(item["passed"] for item in pairwise.values()))
    output = {
        "schema_version": 3,
        "passed": passed,
        "groups": configs,
        "allowed_independent_variables": relevant,
        "pairwise": pairwise,
        "common_mode_neutral_config_sha256": next(iter(neutral_hashes)),
        "formal_campaign_started": False,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for group in GROUPS:
        write(args.output_dir / (group + ".info"), texts[group])
        write(args.output_dir / (group + ".json"),
              json.dumps(configs[group], indent=2, sort_keys=True) + "\n")
    write(args.output_dir / "effective_config_diff.txt", "".join(diff_text))
    write(args.output_dir / "fairness_audit.json",
          json.dumps(output, indent=2, sort_keys=True) + "\n")
    print(json.dumps(output, indent=2, sort_keys=True))
    return 0 if passed else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-config", type=Path, default=BASE_CONFIG)
    subparsers = parser.add_subparsers(dest="command", required=True)
    emit_parser = subparsers.add_parser("emit")
    emit_parser.add_argument("group", choices=tuple(GROUPS))
    emit_parser.add_argument("--output-config", type=Path, required=True)
    emit_parser.add_argument("--output-json", type=Path, required=True)
    emit_parser.set_defaults(function=emit)
    audit_parser = subparsers.add_parser("audit")
    audit_parser.add_argument("--output-dir", type=Path, required=True)
    audit_parser.set_defaults(function=audit)
    args = parser.parse_args()
    result = args.function(args)
    return 0 if result is None else result


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print("set_task_d_mode.py:", error, file=sys.stderr)
        raise SystemExit(1)
