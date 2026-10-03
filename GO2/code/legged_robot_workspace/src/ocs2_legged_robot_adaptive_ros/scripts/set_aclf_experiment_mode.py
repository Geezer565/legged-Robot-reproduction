#!/usr/bin/env python3
"""仅切换 CLF ON/OFF 与受限的 softConstraintMu 单变量实验值。"""

import argparse
import re
import sys
from pathlib import Path


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "ocs2_legged_robot_adaptive" / "config" / "adaptive_settings.info"
ACTIVE_SECTION = "adaptive_clf_mpc"


def active_section(text):
    """仅取 AdaptiveLeggedRobotInterface 实际 load 的 adaptive_clf_mpc 段。"""
    begin = text.find(ACTIVE_SECTION)
    end = text.find("\nadaptive_estimator", begin)
    if begin < 0 or end < 0:
        raise RuntimeError("配置中找不到完整 adaptive_clf_mpc section")
    return begin, end, text[begin:end]


def value_of(text, key):
    _, _, section = active_section(text)
    match = re.search(r"^\s*" + re.escape(key) + r"\s+(\S+)", section, re.MULTILINE)
    if not match:
        raise RuntimeError("配置中找不到 " + key)
    return match.group(1)


def read_kd_diagonal(text):
    _, _, section = active_section(text)
    match = re.search(r"\bkd\s*\{(.*?)\n\s*\}", section, re.DOTALL)
    if not match:
        raise RuntimeError("配置中找不到 K_D 矩阵")
    entries = {(int(i), int(j)): float(value) for i, j, value in re.findall(
        r"\((\d+),(\d+)\)\s+([-+0-9.eE]+)", match.group(1))}
    if len(entries) != 36:
        raise RuntimeError("K_D 必须显式包含完整 6x6 条目")
    return [entries[(i, i)] for i in range(6)]


def main():
    parser = argparse.ArgumentParser(description="公平切换 Adaptive MPC 的 CLF ON/OFF 与 mu 单变量实验模式")
    parser.add_argument("mode", choices=("on", "off"))
    # 默认正式 baseline；当前实验只允许显式选择 0.2，不允许脚本演变为任意调参入口。
    parser.add_argument("--mu", type=float, choices=(0.1, 0.2), default=0.1,
                        help="softConstraintMu；默认正式 baseline 0.1，仅允许 0.1 或 0.2")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()

    text = args.config.read_text()
    begin, end, section = active_section(text)
    requested = "true" if args.mode == "on" else "false"
    pattern = r"^(\s*enableClfConstraint\s+)(true|false)(\s*(?:;.*)?$)"
    updated_section, count = re.subn(pattern, r"\g<1>" + requested + r"\g<3>", section, flags=re.MULTILINE)
    if count != 1:
        raise RuntimeError("enableClfConstraint 必须且只能出现一次")

    # 唯一允许随本实验改变的算法参数：soft penalty 的 mu。delta/K_D/Lambda/
    # Gamma 及 Eq.(11) 均不在这个脚本的写入范围内。
    mu_pattern = r"^(\s*softConstraintMu\s+)([-+0-9.eE]+)(\s*(?:;.*)?$)"
    updated_section, count = re.subn(mu_pattern, r"\g<1>" + f"{args.mu:g}" + r"\g<3>", updated_section, flags=re.MULTILINE)
    if count != 1:
        raise RuntimeError("softConstraintMu 必须且只能出现一次")
    updated = text[:begin] + updated_section + text[end:]

    # 写入前先检查不会悄悄改变对照实验必须固定的字段。
    for key, expected in (("enableParameterUpdate", "true"), ("enableAdaptiveDynamics", "true"),
                          ("enableInputBiasCost", "false"), ("softConstraintDelta", "5.0")):
        actual = value_of(updated, key)
        if actual != expected:
            raise RuntimeError(f"拒绝切换：{key}={actual}，实验要求为 {expected}")
    if float(value_of(updated, "softConstraintMu")) != args.mu:
        raise RuntimeError("softConstraintMu 写入后校验失败")
    kd = read_kd_diagonal(updated)
    if kd != [1.0] * 6:
        raise RuntimeError(f"拒绝切换：K_D 对角线为 {kd}，实验要求为 I6")

    # 仅替换 enableClfConstraint 和 softConstraintMu 两个 token；使用原子 replace
    # 避免节点启动时读到半写入配置。其余实验参数保持原样。
    temporary = args.config.with_suffix(args.config.suffix + ".tmp")
    temporary.write_text(updated)
    temporary.replace(args.config)

    print("config:", args.config)
    print("enableParameterUpdate =", value_of(updated, "enableParameterUpdate"))
    print("enableAdaptiveDynamics =", value_of(updated, "enableAdaptiveDynamics"))
    print("enableClfConstraint =", value_of(updated, "enableClfConstraint"))
    print("enableInputBiasCost =", value_of(updated, "enableInputBiasCost"))
    print("softConstraintMu =", value_of(updated, "softConstraintMu"))
    print("softConstraintDelta =", value_of(updated, "softConstraintDelta"))
    print("K_D diagonal =", kd)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("set_aclf_experiment_mode.py:", error, file=sys.stderr)
        sys.exit(1)
