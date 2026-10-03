#!/usr/bin/env python3
"""Validate and plot one isolated PayloadMass1D smoke run."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import rosbag


PAYLOAD_TOPIC = "/adaptive_update/payload_mass_sample"
UPDATE_TOPIC = "/adaptive_update/sample"
EQ10_TOPIC = "/task_b/equation10_sample"
MODEL_TOPIC = "/task_c/model_mismatch_sample"
IDENTITY_TOPIC = "/task_c/model_identity"


def finite(values):
    return all(math.isfinite(value) for value in values)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args()
    run_dir = args.run_directory.resolve()
    bag_path = run_dir / "payload_mass_smoke.bag"
    if not bag_path.is_file():
        raise RuntimeError("missing smoke bag: {}".format(bag_path))

    payload_rows, update_rows, eq10_rows, model_rows, identities = [], [], [], [], []
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _ in bag.read_messages(
                topics=[PAYLOAD_TOPIC, UPDATE_TOPIC, EQ10_TOPIC, MODEL_TOPIC, IDENTITY_TOPIC]):
            if topic == IDENTITY_TOPIC:
                identities.append(json.loads(message.data))
                continue
            values = list(message.data)
            if not finite(values):
                raise RuntimeError("non-finite values on {}".format(topic))
            if topic == PAYLOAD_TOPIC:
                if len(values) != 21:
                    raise RuntimeError("PayloadMass1D sample must have 21 fields")
                payload_rows.append(values)
            elif topic == UPDATE_TOPIC:
                if len(values) != 16:
                    raise RuntimeError("adaptive update sample must have 16 fields")
                update_rows.append(values)
            elif topic == EQ10_TOPIC:
                if len(values) != 24:
                    raise RuntimeError("Eq.(10) sample must have 24 fields")
                eq10_rows.append(values)
            elif topic == MODEL_TOPIC:
                if len(values) != 41:
                    raise RuntimeError("Task C model sample must have 41 fields")
                model_rows.append(values)

    if not all((payload_rows, update_rows, eq10_rows, model_rows, identities)):
        raise RuntimeError("smoke bag is missing required PayloadMass1D topics")
    identity = identities[-1]
    nominal_mass = payload_rows[0][4]
    true_mass = model_rows[-1][2]
    true_payload = model_rows[-1][3]
    if abs(nominal_mass - 52.13485) > 1.0e-8:
        raise RuntimeError("unexpected nominal mass")
    if abs(identity["nominal_mass"] - nominal_mass) > 1.0e-8 or \
            abs(identity["true_mass"] - true_mass) > 1.0e-8 or \
            abs(identity["payload_delta"] - true_payload) > 1.0e-8:
        raise RuntimeError("model identity disagrees with numeric diagnostics")

    max_time_error = max(abs(row[2]) for row in payload_rows)
    versions = [row[3] for row in payload_rows]
    if max_time_error > 1.0e-6 or any(b <= a for a, b in zip(versions, versions[1:])):
        raise RuntimeError("snapshot synchronization/version monotonicity failed")
    if any(abs(value) > 1.0e-12 for row in update_rows for value in row[5:10]):
        raise RuntimeError("PayloadMass1D inactive estimate elements are nonzero")
    mode_flags = {(row[4], row[5]) for row in eq10_rows}
    if len(mode_flags) != 1 or next(iter(mode_flags)) not in ((1.0, 1.0), (0.0, 0.0)):
        raise RuntimeError("PayloadMass1D update/dynamics flags are inconsistent or unsupported")
    update_enabled, dynamics_enabled = (bool(value) for value in next(iter(mode_flags)))

    total_mass_error = max(abs(row[6] - (row[4] + row[5])) for row in payload_rows)
    ratio_error = max(abs(row[7] - row[5] / row[6]) for row in payload_rows)
    # rosbag establishes the two topic subscriptions independently.  It may
    # therefore capture one member of the simultaneously published pair at
    # ROS startup.  Formal science begins at t=0.50 s; require exact pairing
    # for every sample in that window and retain a count of skipped startup
    # boundary rows instead of falsely rejecting an otherwise complete run.
    synchronized_payload_rows = []
    unmatched_startup_rows = []
    eq10_relation_error = 0.0
    for row in payload_rows:
        eq10 = min(eq10_rows, key=lambda value: abs(value[0] - row[0]))
        if abs(eq10[0] - row[0]) > 1.0e-6:
            if row[0] >= 0.50 - 1.0e-12:
                raise RuntimeError(
                    "PayloadMass1D and Eq.(10) samples are not synchronized in the analysis window")
            unmatched_startup_rows.append(row)
            continue
        synchronized_payload_rows.append(row)
        for index in range(6):
            equivalent_wrench = row[15 + index]
            eq10_relation_error = max(
                eq10_relation_error,
                abs(eq10[6 + index] - equivalent_wrench),
                abs(eq10[12 + index] + equivalent_wrench / nominal_mass),
                abs(eq10[18 + index] - eq10[12 + index]))
    if max(total_mass_error, ratio_error, eq10_relation_error) > 1.0e-10:
        raise RuntimeError("PayloadMass1D algebraic diagnostics are inconsistent")

    if not synchronized_payload_rows:
        raise RuntimeError("no synchronized PayloadMass1D/Eq.(10) samples")

    estimates = np.asarray([row[5] for row in synchronized_payload_rows])
    derivatives = np.asarray([row[8] for row in synchronized_payload_rows])
    times = np.asarray([row[0] for row in synchronized_payload_rows])
    if np.min(nominal_mass + estimates) < 1.0:
        raise RuntimeError("estimated total mass crossed its safety floor")
    applied_norm = np.asarray([np.linalg.norm(row[18:24]) for row in eq10_rows])
    if dynamics_enabled and np.max(applied_norm) <= 1.0e-12:
        raise RuntimeError("PayloadMass1D Eq.(10) applied correction stayed zero")
    if not dynamics_enabled and np.max(applied_norm) > 1.0e-12:
        raise RuntimeError("PayloadMass1D Adaptive-OFF applied correction is nonzero")
    if not update_enabled and np.max(np.abs(estimates)) > 1.0e-12:
        raise RuntimeError("PayloadMass1D Adaptive-OFF estimate is nonzero")

    csv_path = run_dir / "payload_mass_smoke.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["observation_time", "nominal_mass", "true_mass", "true_payload_delta",
                         "payload_mass_estimate", "estimated_total_mass", "estimated_mass_ratio",
                         "payload_mass_derivative", "regressor_Fx", "regressor_Fy", "regressor_Fz",
                         "eq10_equivalent_Fx", "eq10_equivalent_Fy", "eq10_equivalent_Fz"])
        for row in synchronized_payload_rows:
            writer.writerow([row[0], row[4], true_mass, true_payload, row[5], row[6], row[7], row[8],
                             row[9], row[10], row[11], row[15], row[16], row[17]])

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(times, estimates, linewidth=2.0, label=r"$\Delta\hat m$")
    axes[0].axhline(true_payload, color="black", linestyle="--", label=r"true $\Delta m$")
    axes[0].set_ylabel("Payload mass (kg)")
    axes[0].legend()
    axes[1].plot(times, derivatives, color="#ff7f0e", linewidth=1.7)
    axes[1].set_ylabel("Candidate Eq.(8) derivative (kg/s)")
    eq10_times = np.asarray([row[0] for row in eq10_rows])
    axes[2].plot(eq10_times, applied_norm, color="#2ca02c", linewidth=1.7)
    axes[2].set_ylabel("Applied correction norm")
    axes[2].set_xlabel("MPC / simulation time (s)")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    figure.suptitle("PayloadMass1D implementation diagnostic\n"
                    "Single run only; not formal convergence evidence", y=0.985)
    figure.tight_layout(rect=(0.025, 0.02, 1.0, 0.94))
    figure.savefig(run_dir / "payload_mass_smoke.png", dpi=180)
    plt.close(figure)

    summary = {
        "passed": True,
        "scope": "implementation characterization only; single-run data are not formal convergence evidence",
        "sample_count": len(synchronized_payload_rows),
        "unmatched_rosbag_startup_sample_count": len(unmatched_startup_rows),
        "unmatched_rosbag_startup_times_s": [float(row[0]) for row in unmatched_startup_rows],
        "analysis_window_pairing_complete": True,
        "enable_parameter_update": update_enabled,
        "enable_adaptive_dynamics": dynamics_enabled,
        "payload_mass_derivative_semantics": (
            "candidate Eq.(8) derivative; applied to the estimate only when update is enabled"),
        "time_range_s": [float(times[0]), float(times[-1])],
        "nominal_mass_kg": nominal_mass,
        "true_mass_kg": true_mass,
        "true_payload_delta_kg": true_payload,
        "payload_mass_estimate_first_kg": float(estimates[0]),
        "payload_mass_estimate_last_kg": float(estimates[-1]),
        "payload_mass_estimate_min_kg": float(np.min(estimates)),
        "payload_mass_estimate_max_kg": float(np.max(estimates)),
        "payload_mass_derivative_min_kg_s": float(np.min(derivatives)),
        "payload_mass_derivative_max_kg_s": float(np.max(derivatives)),
        "peak_applied_correction_norm": float(np.max(applied_norm)),
        "max_snapshot_time_error_s": max_time_error,
        "max_total_mass_identity_error": total_mass_error,
        "max_mass_ratio_identity_error": ratio_error,
        "max_eq10_mapping_error": eq10_relation_error,
        "model_identity": identity,
    }
    (run_dir / "payload_mass_smoke_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
