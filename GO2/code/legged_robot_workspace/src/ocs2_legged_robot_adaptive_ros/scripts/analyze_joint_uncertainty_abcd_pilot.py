#!/usr/bin/env python3
"""Compact offline analysis for one completed A/B/C/D quick-pilot set."""
import csv
import json
import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import rosbag

ROOT = sys.argv[1]
MODES = ("A", "B", "C", "D")
NAMES = {"A": "no_adaptation_clf", "B": "wrench_6d", "C": "inertial_10d", "D": "joint_16d"}


def desired_at(t, times, states):
    if t <= times[0]: return states[0]
    if t >= times[-1]: return states[-1]
    for i in range(len(times) - 1):
        if times[i] < t <= times[i + 1]:
            a = (times[i + 1] - t) / (times[i + 1] - times[i])
            return [a * x + (1.0 - a) * y for x, y in zip(states[i], states[i + 1])]
    return states[-1]


def wrap(x): return math.atan2(math.sin(x), math.cos(x))
def rms(x): return float(np.sqrt(np.mean(np.square(x)))) if x else float("nan")


all_series = {}
rows = []
for mode in MODES:
    path = os.path.join(ROOT, mode, "pilot.bag")
    target_times, target_states, observations = [], [], []
    s = {"t": [], "xerr": [], "zerr": [], "pos": [], "ori": [], "sigma": [], "mass": [], "fx": [], "joint": []}
    with rosbag.Bag(path) as bag:
        for topic, msg, stamp in bag.read_messages():
            if topic == "/legged_robot_mpc_target":
                target_times = list(msg.timeTrajectory)
                target_states = [list(q.value) for q in msg.stateTrajectory]
            elif topic == "/legged_robot_mpc_observation" and len(msg.state.value) >= 12:
                observations.append((msg.time, list(msg.state.value)))
            elif topic == "/adaptive_plot/sigma_norm": s["sigma"].append(msg.data)
            elif topic == "/adaptive_plot/mass_hat": s["mass"].append(msg.data)
            elif topic == "/adaptive_plot/Fx_hat": s["fx"].append(msg.data)
            elif topic == "/adaptive_update/payload_inertial_plus_wrench_6d_sample": s["joint"].append(list(msg.data))
    # The deterministic scheduler publishes its one-shot target before rosbag's
    # subscriber handshake can be guaranteed.  Its protocol is fixed and its
    # source records the exact construction: initial pose, a 0.10 m x ramp
    # over [1.20,1.40] s, final z=comHeight=0.575.  Reconstructing that same
    # target here avoids altering any controller or rerunning a valid pilot.
    if not target_times:
        if not observations:
            raise RuntimeError("missing observations for mode " + mode)
        initial = observations[0][1]
        start = list(initial); end = list(initial)
        end[6] += 0.10; end[8] = 0.575
        target_times, target_states = [1.20, 1.40], [start, end]
    for t, actual in observations:
        desired = desired_at(t, target_times, target_states)
        e = [desired[i] - actual[i] for i in (6, 7, 8)]
        rpy = [wrap(desired[i] - actual[i]) for i in (11, 10, 9)]
        s["t"].append(t); s["xerr"].append(e[0]); s["zerr"].append(e[2])
        s["pos"].append(math.sqrt(sum(v*v for v in e))); s["ori"].append(math.sqrt(sum(v*v for v in rpy)))
    rows.append({"mode": NAMES[mode], "samples": len(s["t"]), "duration_s": s["t"][-1] if s["t"] else None,
                 "x_rmse_m": rms(s["xerr"]), "z_rmse_m": rms(s["zerr"]), "position_rmse_m": rms(s["pos"]),
                 "orientation_rmse_rad": rms(s["ori"]), "mean_sigma_norm": float(np.mean(s["sigma"])) if s["sigma"] else None,
                 "max_sigma_norm": float(np.max(s["sigma"])) if s["sigma"] else None,
                 "mass_hat_final_kg": s["mass"][-1] if s["mass"] else None, "fx_hat_final_N": s["fx"][-1] if s["fx"] else None})
    all_series[mode] = s

with open(os.path.join(ROOT, "summary.json"), "w") as f: json.dump(rows, f, indent=2, allow_nan=True)
with open(os.path.join(ROOT, "summary.csv"), "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)

def save_metric(key, ylabel, filename):
    plt.figure(figsize=(8, 4.5))
    for mode in MODES:
        s = all_series[mode]
        plt.plot(s["t"], s[key], label=NAMES[mode])
    plt.xlabel("MPC time [s]"); plt.ylabel(ylabel); plt.grid(); plt.legend(); plt.tight_layout()
    plt.savefig(os.path.join(ROOT, filename), dpi=160); plt.close()

save_metric("pos", "position error norm [m]", "figure1_position_error_norm.png")
save_metric("zerr", "z error (desired - actual) [m]", "figure2_z_error.png")
plt.figure(figsize=(8, 4.5))
for mode in MODES:
    s = all_series[mode]
    if s["sigma"]: plt.plot(np.linspace(0, s["t"][-1], len(s["sigma"])), s["sigma"], label=NAMES[mode])
plt.xlabel("MPC time [s]"); plt.ylabel("sigma norm"); plt.grid(); plt.legend(); plt.tight_layout()
plt.savefig(os.path.join(ROOT, "figure3_sigma_norm.png"), dpi=160); plt.close()
plt.figure(figsize=(8, 4.5))
for mode in ("B", "D"):
    s = all_series[mode]
    if s["fx"]: plt.plot(np.linspace(0, s["t"][-1], len(s["fx"])), s["fx"], label=NAMES[mode] + " Fx_hat [N]")
for mode in ("C", "D"):
    s = all_series[mode]
    if s["mass"]: plt.plot(np.linspace(0, s["t"][-1], len(s["mass"])), s["mass"], linestyle="--", label=NAMES[mode] + " mass_hat [kg]")
plt.xlabel("MPC time [s]"); plt.ylabel("mixed units: N (solid), kg (dashed)"); plt.grid(); plt.legend(); plt.tight_layout()
plt.savefig(os.path.join(ROOT, "figure4_mass_and_fx_hat.png"), dpi=160); plt.close()
