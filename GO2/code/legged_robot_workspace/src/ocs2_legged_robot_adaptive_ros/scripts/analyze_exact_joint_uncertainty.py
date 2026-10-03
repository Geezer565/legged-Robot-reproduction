#!/usr/bin/env python3
"""Analyze the recorded exact joint-uncertainty diagnostic without reconstruction."""
import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag

TOPIC = "/adaptive_diagnostics/exact_joint_uncertainty_wrench"
WINDOWS = (("pre", 0.0, 4.0, False), ("active", 4.0, 18.0, True),
           ("post", 18.0, 20.0, True), ("4_7", 4.0, 7.0, False),
           ("7_12", 7.0, 12.0, False), ("12_18", 12.0, 18.0, True))

def statistics(t, eps):
    result = {}
    for name, start, end, inclusive_end in WINDOWS:
        mask = (t >= start) & ((t <= end) if inclusive_end else (t < end))
        values = eps[mask]
        if not len(values):
            raise RuntimeError("no samples in " + name)
        result[name] = {"samples": int(len(values)), "mean": float(values.mean()),
                        "rms": float(np.sqrt(np.mean(values ** 2))),
                        "final": float(values[-1])}
    return result

def component_plot(t, estimate, truth, component, filename, ylabel):
    plt.figure(figsize=(9, 4.5))
    plt.plot(t, truth[:, component], "k--", linewidth=1.5, label="true")
    plt.plot(t, estimate[:, component], linewidth=1.2, label="hat")
    plt.axvspan(4, 18, color="0.7", alpha=.25, label="Fx active")
    plt.xlim(0, 20); plt.xlabel("MPC time [s]"); plt.ylabel(ylabel)
    plt.grid(True); plt.legend(); plt.tight_layout(); plt.savefig(filename, dpi=180); plt.close()

def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: analyze_exact_joint_uncertainty.py BAG OUTPUT_DIR")
    bag_path, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, _ in bag.read_messages(topics=[TOPIC]):
            if len(msg.data) != 44:
                raise RuntimeError("expected 44 diagnostic fields, got %d" % len(msg.data))
            rows.append(np.asarray(msg.data, dtype=float))
    if not rows:
        raise RuntimeError("diagnostic topic has no samples")
    x = np.vstack(rows)
    t = x[:, 0]
    # The diagnostics publisher starts after the controller's initialization
    # phase and can emit a final callback after the 20 s waiter releases.
    # Keep the requested absolute MPC-time windows; only require that they
    # are covered by the recorded stream.
    if np.any(np.diff(t) < 0) or t[0] >= 4.0 or t[-1] < 20.0:
        raise RuntimeError("invalid diagnostic time range: [%g, %g]" % (t[0], t[-1]))
    w_hat, w_true, epsilon, epsilon_norm = x[:, 25:31], x[:, 31:37], x[:, 37:43], x[:, 43]
    recomputed = np.linalg.norm(epsilon, axis=1)
    max_norm_error = float(np.max(np.abs(epsilon_norm - recomputed)))
    if max_norm_error > 1e-8:
        raise RuntimeError("epsilon_norm mismatch: %g" % max_norm_error)
    report = {"bag": str(bag_path), "topic": TOPIC, "samples": int(len(t)),
              "time_range_s": [float(t[0]), float(t[-1])], "epsilon_norm": statistics(t, epsilon_norm),
              "epsilon_norm_recomputed_max_abs_error": max_norm_error,
              "initial_to_final_ratio": float(epsilon_norm[-1] / epsilon_norm[0]) if epsilon_norm[0] else None,
              "active_first_3s_to_last_6s_mean_ratio": float(statistics(t, epsilon_norm)["12_18"]["mean"] / statistics(t, epsilon_norm)["4_7"]["mean"])}
    component_plot(t, w_hat, w_true, 0, out / "U1_exact_Fx_true_vs_hat.png", "Fx [N]")
    component_plot(t, w_hat, w_true, 2, out / "U2_exact_Fz_true_vs_hat.png", "Fz [N]")
    plt.figure(figsize=(9, 4.5)); plt.plot(t, epsilon_norm, linewidth=1.2)
    plt.axvspan(4, 18, color="0.7", alpha=.25); plt.xlim(0, 20)
    plt.xlabel("MPC time [s]"); plt.ylabel("epsilon norm [N,Nm]"); plt.grid(True); plt.tight_layout()
    plt.savefig(out / "U3_exact_epsilon_norm.png", dpi=180); plt.close()
    (out / "exact_joint_uncertainty_report.json").write_text(json.dumps(report, indent=2) + "\n")
    with (out / "epsilon_norm_statistics.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=("window", "samples", "mean", "rms", "final")); writer.writeheader()
        for name, values in report["epsilon_norm"].items(): writer.writerow({"window": name, **values})
    print(json.dumps(report, indent=2))

if __name__ == "__main__":
    main()
