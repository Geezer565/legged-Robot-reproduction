#!/usr/bin/env python3
"""Read-only V1.1 BODY/CoM/left-hand wrench conversion contract check."""
import math
import numpy as np


def rotation_zyx(yaw, pitch, roll):
    cz, sz = math.cos(yaw), math.sin(yaw)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cx, sx = math.cos(roll), math.sin(roll)
    return np.array(((cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
                     (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
                     (-sy, cy * sx, cy * cx)))


def body_com_left_hand_to_world_base(d_body_com_left_hand, rotation_world_body,
                                     com_to_base_body):
    """Exact conversion used by MyRbfV11Control.cpp."""
    d = np.asarray(d_body_com_left_hand, dtype=float)
    r = np.asarray(rotation_world_body, dtype=float)
    c = np.asarray(com_to_base_body, dtype=float)
    force_world = -r @ d[:3]
    base_to_com_world = -r @ c
    torque_world_base = -r @ d[3:] + np.cross(base_to_com_world, force_world)
    return np.concatenate((force_world, torque_world_base))


def world_base_to_body_com_left_hand(wrench_world_base, rotation_world_body,
                                     com_to_base_body):
    """Inverse of the controller/WBC interface conversion."""
    w = np.asarray(wrench_world_base, dtype=float)
    r = np.asarray(rotation_world_body, dtype=float)
    c = np.asarray(com_to_base_body, dtype=float)
    force_body = -r.T @ w[:3]
    base_to_com_world = -r @ c
    torque_world_com = w[3:] - np.cross(base_to_com_world, w[:3])
    torque_body = -r.T @ torque_world_com
    return np.concatenate((force_body, torque_body))


def run():
    c = np.array((0.000229, -0.003089, 0.027360))
    cases = (
        ("identity_pure_fx", rotation_zyx(0, 0, 0), (20, 0, 0, 0, 0, 0)),
        ("identity_pure_fy", rotation_zyx(0, 0, 0), (0, 20, 0, 0, 0, 0)),
        ("identity_pure_torque", rotation_zyx(0, 0, 0), (0, 0, 0, .2, -.5, .7)),
        ("zyx_pure_fx", rotation_zyx(.7, -.25, .18), (20, 0, 0, 0, 0, 0)),
        ("zyx_pure_fy", rotation_zyx(-.4, .31, -.22), (0, 20, 0, 0, 0, 0)),
        ("zyx_mixed", rotation_zyx(.9, -.2, .35), (12, -8, 3, .4, -.7, .2)),
    )
    for name, rotation, d in cases:
        original = np.asarray(d, dtype=float)
        restored = world_base_to_body_com_left_hand(
            body_com_left_hand_to_world_base(original, rotation, c), rotation, c)
        if not np.allclose(restored, original, atol=1e-12, rtol=1e-12):
            raise AssertionError("%s failed: %s != %s" % (name, restored, original))
    print("PASS: BODY/CoM/left-hand <-> WORLD/base wrench contract (%d cases)" % len(cases))


if __name__ == "__main__":
    run()
