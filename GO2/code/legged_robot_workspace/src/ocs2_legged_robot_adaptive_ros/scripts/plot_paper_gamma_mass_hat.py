#!/usr/bin/env python3
"""Offline report plot for the completed Paper-Gamma 16D bag."""
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag

TRUE_MASS = 5.213485
TOPIC = "/adaptive_plot/mass_hat"
DIAG_TOPIC = "/adaptive_diagnostics/exact_joint_uncertainty_wrench"
BAG = Path("/home/han/ocs2_ws/records/gamma_audit_paper_gain_16d/D_paper_gamma/pilot.bag")
OUT = Path("/home/han/ocs2_ws/records/gamma_audit_paper_gain_16d/analysis/G3_paperGamma_mass_hat_vs_true.png")

diagnostic_stamps, diagnostic_times, mass_stamps, mass_values = [], [], [], []
with rosbag.Bag(str(BAG)) as bag:
    for topic, message, stamp in bag.read_messages(topics=[TOPIC, DIAG_TOPIC]):
        if topic == DIAG_TOPIC:
            diagnostic_stamps.append(stamp.to_sec())
            diagnostic_times.append(float(message.data[0]))
        else:
            mass_stamps.append(stamp.to_sec())
            mass_values.append(float(message.data))

if not diagnostic_times or not mass_values:
    raise RuntimeError("required recorded mass/diagnostic topic is empty")
t = np.interp(np.asarray(mass_stamps), np.asarray(diagnostic_stamps), np.asarray(diagnostic_times))
m = np.asarray(mass_values)
last = (t >= 15.0) & (t <= 20.0)
if not np.any(last):
    raise RuntimeError("no mass samples in [15,20] s")
slope = float(np.polyfit(t[last], m[last], 1)[0])

OUT.parent.mkdir(parents=True, exist_ok=True)
plt.figure(figsize=(9, 4.5))
plt.plot(t, m, linewidth=1.5, label="estimated payload mass")
plt.axhline(TRUE_MASS, color="black", linestyle="--", linewidth=1.2, label="true payload mass")
plt.xlim(0, 20)
plt.ylim(bottom=0)
plt.xlabel("MPC time [s]")
plt.ylabel("payload mass estimate [kg]")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(OUT, dpi=180)
plt.close()

print("true mass: %.6f kg" % TRUE_MASS)
print("final mass_hat: %.6f kg" % m[-1])
print("absolute final error: %.6f kg" % abs(m[-1] - TRUE_MASS))
print("relative final error: %.6f %%" % (100.0 * abs(m[-1] - TRUE_MASS) / TRUE_MASS))
print("last 5 s mean: %.6f kg" % m[last].mean())
print("last 5 s std: %.6f kg" % m[last].std())
print("last 5 s slope: %.6f kg/s" % slope)
