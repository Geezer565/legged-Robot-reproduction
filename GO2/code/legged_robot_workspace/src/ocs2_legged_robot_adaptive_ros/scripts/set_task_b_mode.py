#!/usr/bin/env python3
"""Set and audit the three Task B modes without changing any other setting."""

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
DEFAULT_CONFIG = PACKAGE_DIR.parent / "ocs2_legged_robot_adaptive" / "config" / "adaptive_settings.info"
ACTIVE_SECTION = "adaptive_clf_mpc"
MODE_KEYS = ("enableParameterUpdate", "enableAdaptiveDynamics")
MODES = {
    "nominal": {"enableParameterUpdate": False, "enableAdaptiveDynamics": False},
    "estimate_only": {"enableParameterUpdate": True, "enableAdaptiveDynamics": False},
    "full_adaptive": {"enableParameterUpdate": True, "enableAdaptiveDynamics": True},
}
LOCKED_SCALARS = {
    "enableClfConstraint": True,
    "enableInputBiasCost": False,
    "useSoftConstraint": True,
    "softConstraintMu": 0.1,
    "softConstraintDelta": 5.0,
    "timestampTolerance": 1.0e-9,
    "parameterization": "constant_wrench_6d",
}


def active_section(text):
    """Return the exact INFO section loaded by AdaptiveLeggedRobotInterface."""
    begin = text.find(ACTIVE_SECTION)
    end = text.find("\nadaptive_estimator", begin)
    if begin < 0 or end < 0:
        raise RuntimeError("配置中找不到完整 adaptive_clf_mpc section")
    return begin, end, text[begin:end]


def scalar_token(section, key):
    match = re.search(r"^\s*" + re.escape(key) + r"\s+(\S+)", section, re.MULTILINE)
    if not match:
        raise RuntimeError("adaptive_clf_mpc 中找不到 " + key)
    return match.group(1).rstrip(";")


def parse_scalar(token):
    if token in ("true", "false"):
        return token == "true"
    try:
        return float(token)
    except ValueError:
        return token


def matrix_entries(section, key):
    match = re.search(r"^\s*" + re.escape(key) + r"\s*\{", section, re.MULTILINE)
    if not match:
        raise RuntimeError("adaptive_clf_mpc 中找不到矩阵 " + key)
    brace_start = section.find("{", match.start())
    brace_end = section.find("}", brace_start + 1)
    if brace_end < 0:
        raise RuntimeError("矩阵 {} 缺少结束括号".format(key))
    entries = {}
    for row, column, value in re.findall(r"\((\d+),(\d+)\)\s+([-+0-9.eE]+)",
                                         section[brace_start + 1:brace_end]):
        index = (int(row), int(column))
        if index in entries:
            raise RuntimeError("矩阵 {} 中条目 {} 重复".format(key, index))
        entries[index] = float(value)
    return entries


def effective_matrix(section, key, size, default):
    matrix = [[float(default(row, column)) for column in range(size)] for row in range(size)]
    for (row, column), value in matrix_entries(section, key).items():
        if row >= size or column >= size:
            raise RuntimeError("矩阵 {} 的索引越界".format(key))
        matrix[row][column] = value
    return matrix


def effective_vector(section, key, size):
    values = [0.0] * size
    for (row, column), value in matrix_entries(section, key).items():
        if row >= size or column != 0:
            raise RuntimeError("向量 {} 的索引非法".format(key))
        values[row] = value
    return values


def close(left, right, tolerance=1.0e-12):
    return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=tolerance)


def identity(size, diagonal):
    return [[float(diagonal if row == column else 0.0) for column in range(size)] for row in range(size)]


def validate_locked_settings(section):
    errors = []
    for key, expected in LOCKED_SCALARS.items():
        actual = parse_scalar(scalar_token(section, key))
        if isinstance(expected, float):
            if not isinstance(actual, float) or not close(actual, expected):
                errors.append("{}={}，要求 {}".format(key, actual, expected))
        elif actual != expected:
            errors.append("{}={}，要求 {}".format(key, actual, expected))

    expected_matrices = {
        "lambdaLinear": identity(3, 2.0),
        "lambdaAngular": identity(3, 1.5),
        "gamma": [[3.0 if row == column and row < 3 else
                   1.5 if row == column else 0.0 for column in range(6)] for row in range(6)],
        "kd": identity(6, 1.0),
    }
    effective_matrices = {
        "lambdaLinear": effective_matrix(section, "lambdaLinear", 3,
                                          lambda row, column: 1.0 if row == column else 0.0),
        "lambdaAngular": effective_matrix(section, "lambdaAngular", 3,
                                           lambda row, column: 1.0 if row == column else 0.0),
        "gamma": effective_matrix(section, "gamma", 6,
                                   lambda row, column: 1.0 if row == column else 0.0),
        "kd": effective_matrix(section, "kd", 6,
                                lambda row, column: 1.0 if row == column else 0.0),
    }
    for key, expected in expected_matrices.items():
        actual = effective_matrices[key]
        if any(not close(actual[row][column], expected[row][column])
               for row in range(len(expected)) for column in range(len(expected))):
            errors.append("{} 的有效矩阵不是固定 baseline".format(key))

    initial = effective_vector(section, "initialParameterEstimate", 6)
    if any(not close(value, 0.0) for value in initial):
        errors.append("initialParameterEstimate 必须为零")
    if errors:
        raise RuntimeError("拒绝 Task B 模式切换：" + "; ".join(errors))
    return effective_matrices, initial


def replace_mode(text, mode):
    begin, end, section = active_section(text)
    matrices, initial = validate_locked_settings(section)
    updated_section = section
    for key in MODE_KEYS:
        requested = "true" if MODES[mode][key] else "false"
        pattern = r"^(\s*" + re.escape(key) + r"\s+)(true|false)(\s*(?:;.*)?$)"
        updated_section, count = re.subn(pattern, r"\g<1>" + requested + r"\g<3>",
                                         updated_section, flags=re.MULTILINE)
        if count != 1:
            raise RuntimeError("{} 必须且只能出现一次".format(key))
    updated = text[:begin] + updated_section + text[end:]
    _, _, verified = active_section(updated)
    validate_locked_settings(verified)
    for key, expected in MODES[mode].items():
        if parse_scalar(scalar_token(verified, key)) != expected:
            raise RuntimeError("{} 写入后校验失败".format(key))
    return updated, matrices, initial


def normalized_mode_text(text):
    begin, end, section = active_section(text)
    for key in MODE_KEYS:
        pattern = r"^(\s*" + re.escape(key) + r"\s+)(true|false)(\s*(?:;.*)?$)"
        section, count = re.subn(pattern, r"\g<1><TASK_B_MODE>\g<3>", section, flags=re.MULTILINE)
        if count != 1:
            raise RuntimeError("无法规范化 {}".format(key))
    return text[:begin] + section + text[end:]


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    with Path(path).open("rb") as stream:
        return sha256_bytes(stream.read())


def fixed_file_hashes():
    files = {
        "mpc_task": WORKSPACE / "src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/mpc/task.info",
        "reference": WORKSPACE / "src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/command/reference.info",
        "gait": WORKSPACE / "src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/command/gait.info",
        "urdf": WORKSPACE / "src/ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf",
    }
    missing = [str(path) for path in files.values() if not path.is_file()]
    if missing:
        raise RuntimeError("找不到固定实验文件：" + ", ".join(missing))
    return {name: {"path": str(path), "sha256": sha256_file(path)} for name, path in files.items()}


def effective_config(text, mode, matrices, initial):
    _, _, section = active_section(text)
    return {
        "mode": mode,
        "enableParameterUpdate": parse_scalar(scalar_token(section, "enableParameterUpdate")),
        "enableAdaptiveDynamics": parse_scalar(scalar_token(section, "enableAdaptiveDynamics")),
        "enableClfConstraint": parse_scalar(scalar_token(section, "enableClfConstraint")),
        "enableInputBiasCost": parse_scalar(scalar_token(section, "enableInputBiasCost")),
        "useSoftConstraint": parse_scalar(scalar_token(section, "useSoftConstraint")),
        "softConstraintMu": parse_scalar(scalar_token(section, "softConstraintMu")),
        "softConstraintDelta": parse_scalar(scalar_token(section, "softConstraintDelta")),
        "timestampTolerance": parse_scalar(scalar_token(section, "timestampTolerance")),
        "parameterization": scalar_token(section, "parameterization"),
        "lambdaLinear": matrices["lambdaLinear"],
        "lambdaAngular": matrices["lambdaAngular"],
        "gamma": matrices["gamma"],
        "kd": matrices["kd"],
        "initialParameterEstimate": initial,
        "unknownRegressorYu": identity(6, 1.0),
        "config_sha256": sha256_bytes(text.encode("utf-8")),
        "mode_neutral_config_sha256": sha256_bytes(normalized_mode_text(text).encode("utf-8")),
    }


def atomic_write(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(content)
    temporary.replace(path)


def unified_diff(before, after, before_name, after_name):
    return "".join(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                        fromfile=before_name, tofile=after_name))


def set_mode(args):
    original = args.config.read_text()
    updated, matrices, initial = replace_mode(original, args.mode)
    report = effective_config(updated, args.mode, matrices, initial)
    report["fixed_files"] = fixed_file_hashes()
    difference = unified_diff(original, updated, "active_before.info", args.mode + ".info")
    if normalized_mode_text(original) != normalized_mode_text(updated):
        raise RuntimeError("模式切换修改了两个允许字段之外的内容")
    if not args.dry_run:
        atomic_write(args.config, updated)
    if args.snapshot_dir:
        snapshot_dir = args.snapshot_dir
        atomic_write(snapshot_dir / "effective_adaptive_settings.info", updated)
        atomic_write(snapshot_dir / "effective_config.json", json.dumps(report, indent=2, sort_keys=True) + "\n")
        atomic_write(snapshot_dir / "active_config_diff.txt", difference or "# active config already matched requested mode\n")
    print(difference, end="")
    print(json.dumps(report, indent=2, sort_keys=True))


def audit_all(args):
    original = args.config.read_text()
    generated = {}
    reports = {}
    for mode in MODES:
        text, matrices, initial = replace_mode(original, mode)
        generated[mode] = text
        reports[mode] = effective_config(text, mode, matrices, initial)

    neutral_hashes = {report["mode_neutral_config_sha256"] for report in reports.values()}
    pairwise = {}
    expected_pairwise = {
        "nominal__estimate_only": ["enableParameterUpdate"],
        "estimate_only__full_adaptive": ["enableAdaptiveDynamics"],
        "nominal__full_adaptive": ["enableParameterUpdate", "enableAdaptiveDynamics"],
    }
    diff_sections = []
    for pair, expected in expected_pairwise.items():
        left, right = pair.split("__")
        changed = [key for key in MODE_KEYS if MODES[left][key] != MODES[right][key]]
        pairwise[pair] = {"changed_keys": changed, "expected_changed_keys": expected,
                          "passed": changed == expected}
        diff_sections.append("===== {} vs {} =====\n".format(left, right))
        diff_sections.append(unified_diff(generated[left], generated[right], left + ".info", right + ".info"))

    passed = len(neutral_hashes) == 1 and all(item["passed"] for item in pairwise.values())
    audit = {
        "passed": passed,
        "allowed_mode_keys": list(MODE_KEYS),
        "pairwise_differences": pairwise,
        "mode_neutral_config_sha256": next(iter(neutral_hashes)) if neutral_hashes else None,
        "fixed_files": fixed_file_hashes(),
        "effective_configs": reports,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for mode, text in generated.items():
        atomic_write(args.output_dir / (mode + ".info"), text)
        atomic_write(args.output_dir / (mode + ".json"), json.dumps(reports[mode], indent=2, sort_keys=True) + "\n")
    atomic_write(args.output_dir / "effective_config_diff.txt", "".join(diff_sections))
    atomic_write(args.output_dir / "fairness_audit.json", json.dumps(audit, indent=2, sort_keys=True) + "\n")
    print("".join(diff_sections), end="")
    print(json.dumps(audit, indent=2, sort_keys=True))
    return 0 if passed else 2


def main():
    parser = argparse.ArgumentParser(description="Task B 三模式切换与公平性审计")
    subparsers = parser.add_subparsers(dest="command", required=True)

    set_parser = subparsers.add_parser("set", help="原子切换 active adaptive_settings.info")
    set_parser.add_argument("mode", choices=tuple(MODES))
    set_parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    set_parser.add_argument("--snapshot-dir", type=Path)
    set_parser.add_argument("--dry-run", action="store_true")
    set_parser.set_defaults(function=set_mode)

    audit_parser = subparsers.add_parser("audit", help="生成三组有效配置并证明仅两个 mode bit 不同")
    audit_parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    audit_parser.add_argument("--output-dir", type=Path,
                              default=WORKSPACE / "records/task_b/gate_b0")
    audit_parser.set_defaults(function=audit_all)

    args = parser.parse_args()
    result = args.function(args)
    return 0 if result is None else result


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print("set_task_b_mode.py:", error, file=sys.stderr)
        raise SystemExit(1)
