#!/usr/bin/env python3

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--xacro", required=True)
    p.add_argument("--robot-type", default="go2")
    p.add_argument("--evidence", required=True)
    return p.parse_args()


def main():
    args = parse_args()

    delta_mass = float(
        os.environ.get("PAYLOAD_DELTA_MASS", "0.0")
    )

    if not math.isfinite(delta_mass):
        raise RuntimeError(
            "PAYLOAD_DELTA_MASS must be finite"
        )

    xacro_cmd = shutil.which("xacro")
    if xacro_cmd is None:
        raise RuntimeError("Cannot find xacro executable")

    # ----------------------------------------------------------
    # 先展开原始 nominal xacro。
    # 这份 nominal 模型仍然是 controller 使用的模型基础。
    # ----------------------------------------------------------
    result = subprocess.run(
        [
            xacro_cmd,
            args.xacro,
            f"robot_type:={args.robot_type}",
        ],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    root = ET.fromstring(result.stdout)

    # ----------------------------------------------------------
    # 统计 nominal 总质量，同时找到 base。
    # ----------------------------------------------------------
    nominal_total_mass = 0.0
    base_mass_node = None

    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue

        mass_node = inertial.find("mass")
        if mass_node is None:
            continue

        value = mass_node.attrib.get("value")
        if value is None:
            continue

        mass = float(value)
        nominal_total_mass += mass

        if link.attrib.get("name") == "base":
            base_mass_node = mass_node

    if base_mass_node is None:
        raise RuntimeError(
            "Could not find <link name='base'> inertial mass"
        )

    old_base_mass = float(
        base_mass_node.attrib["value"]
    )

    new_base_mass = old_base_mass + delta_mass

    if new_base_mass <= 0.0:
        raise RuntimeError(
            f"Invalid resulting base mass: {new_base_mass}"
        )

    # ----------------------------------------------------------
    # 只改 Gazebo plant 的 base mass。
    #
    # 不改：
    #   - CoM
    #   - inertia tensor
    #   - controller nominal URDF
    #
    # 这与此前“mass-only mismatch”实验定义一致，
    # 但现在是在 spawn 前修改，因此 ODE 初始化时真正看到它。
    # ----------------------------------------------------------
    base_mass_node.set(
        "value",
        f"{new_base_mass:.12g}"
    )

    plant_total_mass = nominal_total_mass + delta_mass

    evidence = {
        "experiment": "mass_only_uncertainty_pre_spawn",
        "body": "go2::base",
        "delta_mass_kg": delta_mass,
        "base_original_mass_kg": old_base_mass,
        "base_modified_mass_kg": new_base_mass,
        "nominal_total_mass_kg": nominal_total_mass,
        "plant_total_mass_kg": plant_total_mass,
        "com_preserved": True,
        "inertia_preserved": True,
        "controller_nominal_model_modified": False,
        "runtime_set_link_properties_used": False,
        "plant_mass_modified_before_spawn": True,
    }

    out = Path(args.evidence)
    out.parent.mkdir(parents=True, exist_ok=True)

    out.write_text(
        json.dumps(
            evidence,
            indent=2,
            sort_keys=True,
        ) + "\n"
    )

    # ----------------------------------------------------------
    # stdout 必须只输出 URDF。
    # roslaunch <param command="..."> 会直接捕获这个结果。
    # ----------------------------------------------------------
    sys.stdout.write(
        ET.tostring(
            root,
            encoding="unicode",
        )
    )


if __name__ == "__main__":
    main()
