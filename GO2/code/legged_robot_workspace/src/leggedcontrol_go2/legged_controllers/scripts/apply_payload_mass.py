#!/usr/bin/env python3

import argparse
import json
import math
import os
import time
from pathlib import Path

import rospy

from gazebo_msgs.srv import (
    GetLinkProperties,
    SetLinkProperties,
    SetLinkPropertiesRequest,
)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--body", default="go2::base")
    p.add_argument("--evidence", required=True)
    return p.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()

    delta_mass = float(
        os.environ.get("PAYLOAD_DELTA_MASS", "0.0")
    )

    if not math.isfinite(delta_mass):
        raise RuntimeError("PAYLOAD_DELTA_MASS must be finite")

    rospy.init_node("myrbf_payload_mass", anonymous=False)

    rospy.wait_for_service(
        "/gazebo/get_link_properties",
        timeout=180.0
    )

    rospy.wait_for_service(
        "/gazebo/set_link_properties",
        timeout=180.0
    )

    get_prop = rospy.ServiceProxy(
        "/gazebo/get_link_properties",
        GetLinkProperties
    )

    set_prop = rospy.ServiceProxy(
        "/gazebo/set_link_properties",
        SetLinkProperties
    )

    # 等待 go2::base 真正生成
    before = None

    deadline = time.monotonic() + 180.0

    while not rospy.is_shutdown():

        try:
            r = get_prop(args.body)

            if r.success:
                before = r
                break

        except rospy.ServiceException:
            pass

        if time.monotonic() > deadline:
            raise RuntimeError(
                "Timed out waiting for Gazebo link " + args.body
            )

        time.sleep(0.05)

    if before is None:
        raise RuntimeError("Gazebo link unavailable")

    old_mass = float(before.mass)
    new_mass = old_mass + delta_mass

    if new_mass <= 0.0:
        raise RuntimeError(
            "Invalid resulting mass %.9f" % new_mass
        )

    req = SetLinkPropertiesRequest()

    req.link_name = args.body

    # 纯质量失配实验：
    # 只改变 mass，保持 CoM 与惯量完全不变。
    req.com = before.com
    req.gravity_mode = before.gravity_mode

    req.mass = new_mass

    req.ixx = before.ixx
    req.ixy = before.ixy
    req.ixz = before.ixz
    req.iyy = before.iyy
    req.iyz = before.iyz
    req.izz = before.izz

    result = set_prop(req)

    if not result.success:
        raise RuntimeError(
            "set_link_properties failed: "
            + result.status_message
        )

    after = get_prop(args.body)

    if not after.success:
        raise RuntimeError(
            "Could not verify modified link properties"
        )

    if abs(float(after.mass) - new_mass) > 1e-6:
        raise RuntimeError(
            "Mass verification failed: expected %.9f got %.9f"
            % (new_mass, float(after.mass))
        )

    evidence = {
        "experiment": "mass_only_uncertainty",
        "body": args.body,
        "delta_mass_kg": delta_mass,
        "original_mass_kg": old_mass,
        "modified_mass_kg": float(after.mass),

        # 用于确认负载是在运动开始前就已经生效
        "ros_time_when_applied_s": float(rospy.get_time()),

        "com_preserved": True,
        "inertia_preserved": True,
        "external_wrench_applied": False,
        "controller_nominal_model_modified": False,
    }

    out = Path(args.evidence)
    out.parent.mkdir(parents=True, exist_ok=True)

    out.write_text(
        json.dumps(
            evidence,
            indent=2,
            sort_keys=True
        ) + "\n"
    )

    rospy.set_param(
        "/myrbf_payload_mass_applied",
        True
    )

    rospy.logwarn(
        "[PAYLOAD_MASS_APPLIED] %s %.6f -> %.6f kg, delta=%.6f kg, sim_time=%.6f",
        args.body,
        old_mass,
        float(after.mass),
        delta_mass,
        rospy.get_time(),
    )


if __name__ == "__main__":
    main()
