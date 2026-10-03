#!/usr/bin/env python3
"""Read-only pre/active/post analysis for an already completed quick pilot."""
import csv
import json
import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import rosbag

ROOT = sys.argv[1]
OUT = os.path.join(ROOT, "windowed_reanalysis")
os.makedirs(OUT, exist_ok=True)
MODES = ("A", "B", "C", "D")
NAMES = {"A": "no_adaptation_clf", "B": "wrench_6d", "C": "inertial_10d", "D": "joint_16d"}
WINDOWS = {"pre": (0.0, 4.0, False), "active": (4.0, 16.0, True), "post": (16.0, 20.0, True)}

def wrap(x): return math.atan2(math.sin(x), math.cos(x))
def rms(x): return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")
def interpolate(t, ts, ys): return float(np.interp(t, ts, ys))
def nearest(values, target, before=None):
    values = [p for p in values if before is None or (p[0] < target if before else p[0] > target)]
    return min(values, key=lambda p: abs(p[0] - target))[1] if values else None
def selected(pairs, lo, hi, inclusive): return [v for t, v in pairs if lo <= t <= hi if inclusive] if inclusive else [v for t, v in pairs if lo <= t < hi]
def stats(pairs, lo, hi, inclusive=True):
    x = selected(pairs, lo, hi, inclusive)
    return {"n": len(x), "mean": float(np.mean(x)) if x else None, "rms": rms(x) if x else None, "max": float(np.max(x)) if x else None}

series = {}
metric_rows = []
for mode in MODES:
    path = os.path.join(ROOT, mode, "pilot.bag")
    obs_wall, obs_time, obs_state = [], [], []
    raw = {"sigma": [], "mass": [], "fx": [], "joint": []}
    with rosbag.Bag(path) as bag:
        for topic, msg, stamp in bag.read_messages():
            wall = stamp.to_sec()
            if topic == "/legged_robot_mpc_observation" and len(msg.state.value) >= 12:
                obs_wall.append(wall); obs_time.append(msg.time); obs_state.append(list(msg.state.value))
            elif topic == "/adaptive_plot/sigma_norm": raw["sigma"].append((wall, msg.data))
            elif topic == "/adaptive_plot/mass_hat": raw["mass"].append((wall, msg.data))
            elif topic == "/adaptive_plot/Fx_hat": raw["fx"].append((wall, msg.data))
            elif topic == "/adaptive_update/payload_inertial_plus_wrench_6d_sample": raw["joint"].append(list(msg.data))
    # Scheduler target is deterministic and its non-latched one-shot message was
    # not bagged. Reconstruct precisely from scheduler source: initial pose,
    # 0.10 m x ramp [1.2,1.4], final z=0.575.
    initial = obs_state[0]
    def desired(t):
        alpha = min(1.0, max(0.0, (t - 1.2) / 0.2))
        q = list(initial); q[6] += 0.10 * alpha; q[8] = initial[8] + alpha * (0.575 - initial[8]); return q
    errors = {"x": [], "z": [], "position": [], "orientation": []}
    for t, actual in zip(obs_time, obs_state):
        ref = desired(t); ep = [ref[i] - actual[i] for i in (6, 7, 8)]
        eo = [wrap(ref[i] - actual[i]) for i in (11, 10, 9)]
        errors["x"].append((t, ep[0])); errors["z"].append((t, ep[2]))
        errors["position"].append((t, math.sqrt(sum(v*v for v in ep))))
        errors["orientation"].append((t, math.sqrt(sum(v*v for v in eo))))
    def to_mpc(raw_pairs): return [(interpolate(w, obs_wall, obs_time), v) for w, v in raw_pairs]
    s = {"errors": errors, "sigma": to_mpc(raw["sigma"]), "mass": to_mpc(raw["mass"]), "fx": to_mpc(raw["fx"]), "joint": raw["joint"]}
    series[mode] = s
    for window, (lo, hi, inclusive) in WINDOWS.items():
        row = {"mode": NAMES[mode], "window": window}
        for k, pairs in errors.items(): row[k + "_rmse"] = rms(selected(pairs, lo, hi, inclusive))
        ss = stats(s["sigma"], lo, hi, inclusive)
        row.update({"sigma_mean": ss["mean"], "sigma_rms": ss["rms"], "sigma_max": ss["max"], "samples": len(selected(errors["x"], lo, hi, inclusive))})
        metric_rows.append(row)

active = [r for r in metric_rows if r["window"] == "active"]
def row(mode): return next(r for r in active if r["mode"] == NAMES[mode])
def reduction(new, old): return 100.0 * (old - new) / old
improvements=[]
for candidate, baseline in (("B","A"),("C","A"),("D","A"),("D","C")):
    a, b = row(candidate), row(baseline)
    improvements.append({"comparison": NAMES[candidate] + " vs " + NAMES[baseline], **{k + "_reduction_pct": reduction(a[k], b[k]) for k in ("x_rmse", "z_rmse", "position_rmse", "orientation_rmse", "sigma_mean", "sigma_rms")}})

estimate_report = {}
for mode in ("B", "D"):
    p = series[mode]["fx"]
    estimate_report[mode + "_fx"] = {"pre_4_minus": nearest(p, 4.0, True), "at_8": nearest(p,8.0), "at_12": nearest(p,12.0), "at_15_5": nearest(p,15.5), "post_16_plus": nearest(p,16.0, False), "active": stats(p,4.0,16.0)}
for mode in ("C", "D"):
    p = series[mode]["mass"]
    estimate_report[mode + "_mass"] = {"at_4": nearest(p,4.0), "at_8": nearest(p,8.0), "at_12": nearest(p,12.0), "at_16": nearest(p,16.0), "at_20": nearest(p,20.0), "mean_10_20": stats(p,10.0,20.0)["mean"]}

joint = series["D"]["joint"]
decomp = {}
for label, index in (("inertial_Fx",18),("inertial_Fz",20),("constant_Fx",24),("constant_Fz",26),("combined_Fx",30),("combined_Fz",32)):
    p = [(d[0],d[index]) for d in joint if len(d) == 44]
    decomp[label] = stats(p,4.0,16.0)

with open(os.path.join(OUT, "window_metrics.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=metric_rows[0].keys()); w.writeheader(); w.writerows(metric_rows)
with open(os.path.join(OUT, "active_improvements.json"), "w") as f: json.dump(improvements, f, indent=2)
with open(os.path.join(OUT, "estimates_and_decomposition.json"), "w") as f: json.dump({"estimates":estimate_report,"joint_active_decomposition":decomp},f,indent=2)

plt.figure(figsize=(9,6));
for key, title, axis in (("x","x error [m]",1),("position","position error norm [m]",2)):
    plt.subplot(2,1,axis)
    for m in MODES:
        p=series[m]["errors"][key]; plt.plot([x[0] for x in p if 4<=x[0]<=16],[x[1] for x in p if 4<=x[0]<=16],label=NAMES[m])
    plt.ylabel(title); plt.grid(); plt.legend()
plt.xlabel("MPC time [s]"); plt.tight_layout(); plt.savefig(os.path.join(OUT,"figure1_active_x_and_position_error.png"),dpi=160); plt.close()
plt.figure(figsize=(9,4.5));
for m in ("B","D"):
    p=series[m]["fx"]; plt.plot([x[0] for x in p],[x[1] for x in p],label=NAMES[m])
plt.axvspan(4,16,color="gray",alpha=.15,label="true Fx active"); plt.hlines(20,4,16,colors="k",linestyles="--",label="true Fx=+20 N"); plt.xlim(0,20); plt.xlabel("MPC time [s]"); plt.ylabel("Fx_hat [N]"); plt.grid(); plt.legend(); plt.tight_layout(); plt.savefig(os.path.join(OUT,"figure2_fx_hat_active_window.png"),dpi=160); plt.close()
plt.figure(figsize=(9,4.5));
for m in ("C","D"):
    p=series[m]["mass"]; plt.plot([x[0] for x in p],[x[1] for x in p],label=NAMES[m])
plt.hlines(5.213485,0,20,colors="k",linestyles="--",label="true payload mass"); plt.xlim(0,20); plt.xlabel("MPC time [s]"); plt.ylabel("mass_hat [kg]"); plt.grid(); plt.legend(); plt.tight_layout(); plt.savefig(os.path.join(OUT,"figure3_mass_hat.png"),dpi=160); plt.close()
