#!/usr/bin/env python3
"""Validate one D5H0 shadow run and write fixed-grid-independent metrics."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


SHADOW_TOPIC = "/task_d/d5h0_shadow_update_sample"
IDENTITY_TOPIC = "/task_d/d5h0_shadow_identity"
V3_TOPIC = "/task_d/payload_inertial_10d_v3_sample"
SHADOW_LABEL = "payload_inertial_d5h0_shadow_v1_65_fields"
V3_LABEL = "payload_inertial_10d_v3_118_fields"
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz",
              "Jyy", "Jyz", "Jzz")


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sample_std(values):
    values = np.asarray(values, dtype=float)
    return float(np.std(values, ddof=1)) if len(values) > 1 else 0.0


def physical_truth_and_scales(config):
    payload = config["payload"]
    mass = float(payload["mass_kg"])
    com = np.asarray(payload["com_nominal_c0_base_m"], dtype=float)
    compact_inertia = np.asarray(
        payload["inertia_com_base_kg_m2"], dtype=float)
    if compact_inertia.shape != (6,):
        raise ValueError("payload inertia must contain xx,xy,xz,yy,yz,zz")
    truth = np.asarray([
        mass, *(mass * com), *compact_inertia,
    ], dtype=float)
    length = np.linalg.norm(truth[1:4]) / abs(mass)
    scales = np.asarray([abs(mass)] + [abs(mass) * length] * 3 +
                        [abs(mass) * length * length] * 6)
    return truth, scales


def scaled_norm(vector, scales):
    return float(np.linalg.norm(np.asarray(vector, dtype=float) / scales))


def read_bag(path):
    shadows, v3_rows, identities, shadow_labels, v3_labels = [], [], [], set(), set()
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, _ in bag.read_messages(
                topics=[SHADOW_TOPIC, IDENTITY_TOPIC, V3_TOPIC]):
            if topic == SHADOW_TOPIC:
                shadow_labels.add(message.layout.dim[0].label
                                  if message.layout.dim else "")
                shadows.append(np.asarray(message.data, dtype=float))
            elif topic == V3_TOPIC:
                v3_labels.add(message.layout.dim[0].label
                              if message.layout.dim else "")
                v3_rows.append(np.asarray(message.data, dtype=float))
            else:
                identities.append(json.loads(message.data))
    if shadow_labels != {SHADOW_LABEL} or v3_labels != {V3_LABEL}:
        raise ValueError("D5H0/V3 schema label mismatch")
    shadow = np.asarray(sorted(shadows, key=lambda row: row[0]), dtype=float)
    v3 = np.asarray(sorted(v3_rows, key=lambda row: row[0]), dtype=float)
    if shadow.ndim != 2 or shadow.shape[1] != 65 or not len(shadow):
        raise ValueError("missing or malformed D5H0 shadow rows")
    if v3.ndim != 2 or v3.shape[1] != 118 or not len(v3):
        raise ValueError("missing or malformed D5H0 input rows")
    if not np.all(np.isfinite(shadow)) or not np.all(np.isfinite(v3)):
        raise ValueError("D5H0 bag contains nonfinite numeric data")
    if np.any(np.diff(shadow[:, 0]) <= 0.0) or np.any(np.diff(v3[:, 0]) <= 0.0):
        raise ValueError("D5H0 timestamps are not strictly increasing")
    unique_identities = {json.dumps(item, sort_keys=True) for item in identities}
    if len(unique_identities) != 1:
        raise ValueError("D5H0 identity is missing or inconsistent")
    return shadow, v3, json.loads(next(iter(unique_identities)))


def match_v3(shadow, v3, tolerance=1.0e-8):
    matched = []
    errors = []
    index = 0
    for row in shadow:
        while index + 1 < len(v3) and abs(v3[index + 1, 0] - row[0]) < abs(v3[index, 0] - row[0]):
            index += 1
        matched.append(v3[index])
        errors.append(v3[index, 0] - row[0])
    errors = np.asarray(errors)
    keep = np.abs(errors) <= tolerance
    coverage = float(np.mean(keep))
    if coverage < 0.99:
        raise ValueError("D5H0 shadow/v3 timestamp coverage is below 99%")
    # rosbag may subscribe to the two already-live topics one callback apart.
    # Only unmatched recording-edge rows are discarded from cross-topic
    # closure.  Full-shadow cumulative closure is still checked separately.
    return shadow[keep], np.asarray(matched)[keep], errors[keep], int(np.sum(~keep))


def cumulative_reconstruction(shadow, gamma):
    raw, corrected = np.zeros(10), np.zeros(10)
    raw_error, corrected_error = 0.0, 0.0
    for row in shadow:
        dt = row[12]
        if row[6] == 1.0:
            raw += dt * gamma.dot(row[15:25])
            corrected += dt * gamma.dot(row[35:45])
        raw_error = max(raw_error, float(np.max(np.abs(raw - row[45:55]))))
        corrected_error = max(
            corrected_error, float(np.max(np.abs(corrected - row[55:65]))))
    return raw, corrected, raw_error, corrected_error


def write_timeseries(path, rows):
    fields = ["observation_time", "mode", "phase_valid", "steady_active",
              "normalized_phase", "phase_bin", "cycle_boundary", "dt"]
    fields += [prefix + "_" + name for prefix in
               ("raw_drive", "baseline_drive", "corrected_drive",
                "cumulative_raw", "cumulative_corrected")
               for name in PARAMETERS]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            output = {
                "observation_time": row[0], "mode": int(row[4]),
                "phase_valid": int(row[5]), "steady_active": int(row[6]),
                "normalized_phase": row[7], "phase_bin": int(row[8]),
                "cycle_boundary": int(row[9]), "dt": row[12],
            }
            for offset, prefix in ((15, "raw_drive"), (25, "baseline_drive"),
                                   (35, "corrected_drive"),
                                   (45, "cumulative_raw"),
                                   (55, "cumulative_corrected")):
                for index, name in enumerate(PARAMETERS):
                    output[prefix + "_" + name] = row[offset + index]
            writer.writerow(output)


def plot_run(path, shadow):
    time = shadow[:, 0]
    figure, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)
    for parameter in (3, 4, 7):
        axes[0].plot(time, shadow[:, 15 + parameter], label="raw " + PARAMETERS[parameter])
        axes[1].plot(time, shadow[:, 25 + parameter], label="baseline " + PARAMETERS[parameter])
        axes[2].plot(time, shadow[:, 35 + parameter], label="corrected " + PARAMETERS[parameter])
        axes[3].plot(time, shadow[:, 45 + parameter], label="raw cumulative " + PARAMETERS[parameter])
        axes[3].plot(time, shadow[:, 55 + parameter], linestyle="--",
                     label="corrected cumulative " + PARAMETERS[parameter])
    axes[4].step(time, shadow[:, 8], where="post", label="phase bin")
    axes[4].step(time, 24.0 * shadow[:, 6], where="post", label="steady active")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend(ncol=3, fontsize=8)
    axes[-1].set_xlabel("MPC / observation time [s]")
    figure.suptitle("D5H0 online shadow diagnostics (no estimator write path)")
    figure.tight_layout()
    figure.savefig(path, dpi=170)
    plt.close(figure)


def analyze(run_directory):
    run = Path(run_directory)
    bag_path = run / "task_d_d5h0.bag"
    config = json.loads((run / "effective_config.json").read_text())
    shadow, v3, identity = read_bag(bag_path)
    matched_shadow, matched, time_error, unmatched_edge_rows = match_v3(shadow, v3)
    gamma = np.asarray(config["gamma"], dtype=float)
    if gamma.shape != (10, 10):
        raise ValueError("effective-config Gamma is not 10x10")
    if not np.allclose(gamma, np.diag(identity["gamma_diagonal"]),
                       rtol=0.0, atol=1.0e-12):
        raise ValueError("D5H0 diagnostic Gamma differs from effective config")

    regressor = matched[:, 48:108].reshape(-1, 6, 10)
    sigma = matched[:, 42:48]
    reconstructed_raw = np.einsum("nij,ni->nj", regressor, sigma)
    raw_closure = float(np.max(np.abs(
        reconstructed_raw - matched_shadow[:, 15:25])))
    derivative_closure = float(np.max(np.abs(
        matched[:, 32:42] - np.einsum("ij,nj->ni", gamma, reconstructed_raw))))
    corrected_closure = float(np.max(np.abs(
        shadow[:, 35:45] - shadow[:, 15:25] + shadow[:, 25:35])))
    cumulative_raw, cumulative_corrected, cumulative_raw_error, cumulative_corrected_error = \
        cumulative_reconstruction(shadow, gamma)

    active = shadow[:, 6] == 1.0
    if not np.any(active):
        raise ValueError("D5H0 run contains no steady active shadow samples")
    active_duration = float(np.sum(shadow[:, 12]))
    complete_active_cycles = int(np.sum((shadow[:, 9] == 1.0) & active))
    truth, scales = physical_truth_and_scales(config)
    raw_scaled = scaled_norm(cumulative_raw, scales)
    corrected_scaled = scaled_norm(cumulative_corrected, scales)
    suppression = 1.0 - corrected_scaled / raw_scaled if raw_scaled > 0.0 else 0.0
    checks = {
        "all_finite": bool(np.all(np.isfinite(shadow))),
        "identity_shadow_only": bool(identity.get("shadow_only") is True and
                                     identity.get("controller_write_path") is False),
        "row_shadow_only_flag": bool(np.all(shadow[:, 14] == 1.0)),
        "baseline_loaded_flag": bool(np.all(shadow[:, 13] == 1.0)),
        "snapshot_time_sync": bool(np.max(np.abs(shadow[:, 2])) <= 1.0e-6),
        "raw_drive_closure": raw_closure <= 1.0e-10,
        "actual_estimator_derivative_unchanged": derivative_closure <= 1.0e-10,
        "corrected_drive_closure": corrected_closure <= 1.0e-12,
        "cumulative_raw_closure": cumulative_raw_error <= 1.0e-10,
        "cumulative_corrected_closure": cumulative_corrected_error <= 1.0e-10,
        "phase_bins_valid_when_active": bool(np.all(
            (shadow[active, 8] >= 0.0) & (shadow[active, 8] < 24.0))),
        "has_complete_active_cycle": complete_active_cycles >= 1,
    }
    passed = all(checks.values())
    summary = {
        "schema_version": 1,
        "stage": "D5H0_shadow_diagnostics",
        "group": config["group"],
        "run_index": config["run_index"],
        "run_scope": config["run_scope"],
        "passed": passed,
        "formal_eligibility": bool(passed and config["run_scope"] == "formal"),
        "shadow_only": True,
        "actual_estimator_modified": False,
        "bag": str(bag_path),
        "bag_sha256": sha256(bag_path),
        "baseline_sha256": identity["baseline_sha256"],
        "sample_count": int(len(shadow)),
        "cross_topic_matched_sample_count": int(len(matched_shadow)),
        "unmatched_recording_edge_rows": unmatched_edge_rows,
        "cross_topic_match_fraction": float(len(matched_shadow) / len(shadow)),
        "steady_active_sample_count": int(np.sum(active)),
        "steady_active_duration_s": active_duration,
        "complete_active_cycles": complete_active_cycles,
        "closures": {
            "shadow_v3_time_max_abs_error_s": float(np.max(np.abs(time_error))),
            "raw_drive_max_abs_error": raw_closure,
            "actual_pi_dot_vs_gamma_raw_max_abs_error": derivative_closure,
            "corrected_drive_max_abs_error": corrected_closure,
            "cumulative_raw_max_abs_error": cumulative_raw_error,
            "cumulative_corrected_max_abs_error": cumulative_corrected_error,
        },
        "terminal": {
            "raw_parameter_change": dict(zip(PARAMETERS, cumulative_raw.tolist())),
            "corrected_parameter_change": dict(zip(
                PARAMETERS, cumulative_corrected.tolist())),
            "raw_scaled_norm": raw_scaled,
            "corrected_scaled_norm": corrected_scaled,
            "suppression_fraction": suppression,
        },
        "payload_truth": dict(zip(PARAMETERS, truth.tolist())),
        "physical_coordinate_scales": dict(zip(PARAMETERS, scales.tolist())),
        "checks": checks,
        "identity": identity,
        "interpretation_limit": (
            "Shadow replay only. Corrected values do not enter Eq.(8), snapshot, "
            "Eq.(10), MPC policy, or plant."),
    }
    (run / "task_d_d5h0_run_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    write_timeseries(run / "task_d_d5h0_timeseries.csv", shadow)
    plot_run(run / "task_d_d5h0_shadow.png", shadow)
    report = """# D5H0 shadow run validation

- Group: `{group}` / run `{run_index}` / `{scope}`
- Result: **{result}**
- Shadow-only: `true`; actual estimator modified: `false`
- Samples / active duration / complete active cycles: `{samples}` / `{duration:.3f} s` / `{cycles}`
- Raw terminal scaled norm: `{raw:.8g}`
- Corrected terminal scaled norm: `{corrected:.8g}`
- Candidate suppression: `{suppression:.2%}`
- Raw `Y^T sigma` reconstruction max error: `{raw_error:.3e}`
- Actual estimator `pi_dot = Gamma Y^T sigma` max error: `{derivative_error:.3e}`
- Corrected-drive closure max error: `{corrected_error:.3e}`

This run validates a diagnostic counterfactual only.  The corrected drive was
not applied to Eq.(8), the frozen snapshot, Eq.(10), the MPC policy, or the
plant.  Formal robustness and payload-contrast claims require the paired D5H0
campaign analysis.
""".format(
        group=config["group"], run_index=config["run_index"],
        scope=config["run_scope"], result="PASS" if passed else "FAIL",
        samples=len(shadow), duration=active_duration, cycles=complete_active_cycles,
        raw=raw_scaled, corrected=corrected_scaled, suppression=suppression,
        raw_error=raw_closure, derivative_error=derivative_closure,
        corrected_error=corrected_closure)
    (run / "stage_d5h0_run_report.md").write_text(report)
    if not passed:
        raise RuntimeError("D5H0 run validation failed")
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory")
    args = parser.parse_args()
    summary = analyze(args.run_directory)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
