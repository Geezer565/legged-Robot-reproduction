#!/usr/bin/env python3
"""Audit one D5I4 four-mode online smoke from exact synchronized topics."""

import csv
import hashlib
import json
import math
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


UPDATE = "/task_d/d5i4_composite_update_sample"
V3 = "/task_d/payload_inertial_10d_v3_sample"
FLOW = "/task_d/equation10_full_flow_sample"
CLF = "/task_d/d5h1_eq11_sample"
SCHEDULER = "/task_d/d5i2_scheduler_sample"
MODEL_IDENTITY = "/task_d/model_identity"
SCHEDULER_IDENTITY = "/task_d/d5i2_scheduler_identity"
OBSERVATION = "/legged_robot_mpc_observation"
TARGET = "/legged_robot_mpc_target"
LABELS = {
    UPDATE: "payload_inertial_d5i4_composite_update_v1_76_fields",
    V3: "payload_inertial_10d_v3_118_fields",
    FLOW: "payload_inertial_eq10_full_flow_v2_80_fields",
    CLF: "payload_inertial_d5h1_eq11_v1_119_fields",
    SCHEDULER: "d5i2_scheduler_v1_28_fields",
}
SIZES = {UPDATE: 76, V3: 118, FLOW: 80, CLF: 119, SCHEDULER: 28}
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy",
              "Jyz", "Jzz")


def json_default(value):
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError("not JSON serializable: " + type(value).__name__)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def max_abs(left, right):
    left, right = np.asarray(left), np.asarray(right)
    return float(np.max(np.abs(left - right))) if left.size else math.inf


def deduplicate(rows):
    return np.asarray([row for _, row in sorted(
        {float(row[0]): row for row in rows}.items())], dtype=float)


def rows_by_version(rows):
    return {int(round(row[3])): row for row in rows}


def interpolate_matrix(grid, times, values):
    return np.column_stack([
        np.interp(grid, times, values[:, index])
        for index in range(values.shape[1])])


def load_baseline(path):
    result, seen = {}, {}
    parameter_index = {name: index for index, name in enumerate(PARAMETERS)}
    with Path(path).open(newline="") as stream:
        for row in csv.DictReader(stream):
            key = (int(row["profile_phase_bin"]),
                   int(row["gait_phase_bin"]), int(row["executed_mode"]))
            if key not in result:
                result[key] = np.full(10, np.nan)
                seen[key] = set()
            index = parameter_index[row["parameter"]]
            if index in seen[key]:
                raise ValueError("duplicate frozen baseline key/parameter")
            result[key][index] = float(row["nominal_baseline_drive_rate"])
            seen[key].add(index)
    if not result or any(not np.all(np.isfinite(value))
                         for value in result.values()):
        raise ValueError("incomplete D5I4 frozen baseline")
    return result


def eq11_closures(clf, kd):
    sigma, pi_hat = clf[:, 13:19], clf[:, 19:29]
    contact, nominal = clf[:, 29:35], clf[:, 35:41]
    estimated, residual = clf[:, 41:47], clf[:, 47:53]
    contribution = clf[:, 53:59]
    regressor = clf[:, 59:119].reshape(-1, 6, 10)
    expected_estimated = np.einsum("nij,nj->ni", regressor, pi_hat)
    expected_residual = contact - nominal - estimated
    expected_contribution = sigma * expected_residual
    expected_linear = np.sum(expected_contribution, axis=1)
    expected_damping = 0.5 * np.einsum(
        "ni,ij,nj->n", sigma, kd, sigma)
    expected_h = expected_linear - expected_damping
    return {
        "estimated_wrench_Y_pi": max_abs(estimated, expected_estimated),
        "residual_wrench": max_abs(residual, expected_residual),
        "component_contribution": max_abs(contribution,
                                           expected_contribution),
        "linear_term": max_abs(clf[:, 11], expected_linear),
        "damping_term": max_abs(clf[:, 12], expected_damping),
        "reconstructed_h": max_abs(clf[:, 9], expected_h),
        "h_clf": max_abs(clf[:, 8], expected_h),
        "violation": max_abs(clf[:, 10], np.maximum(0.0, -clf[:, 8])),
    }


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_task_d_d5i4_smoke.py RUN_DIR")
    run = Path(sys.argv[1]).resolve()
    config = json.loads((run / "effective_config.json").read_text())
    protocol = json.loads((run / "frozen_d5i4_protocol.json").read_text())
    bag_path = run / "task_d_d5i4.bag"
    numeric = {topic: [] for topic in LABELS}
    observed_labels = {topic: set() for topic in LABELS}
    identities, scheduler_identities, observations, targets = [], [], [], []
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _ in bag.read_messages(topics=(
                list(LABELS) + [MODEL_IDENTITY, SCHEDULER_IDENTITY,
                                OBSERVATION, TARGET])):
            if topic in LABELS:
                observed_labels[topic].add(
                    message.layout.dim[0].label if message.layout.dim else "")
                numeric[topic].append([float(value) for value in message.data])
            elif topic == MODEL_IDENTITY:
                identities.append(json.loads(message.data))
            elif topic == SCHEDULER_IDENTITY:
                scheduler_identities.append(json.loads(message.data))
            elif topic == OBSERVATION:
                observations.append([
                    float(message.time), float(message.mode),
                    *map(float, message.state.value),
                    *map(float, message.input.value)])
            elif topic == TARGET:
                targets.append((
                    np.asarray(message.timeTrajectory, dtype=float),
                    np.asarray([value.value for value in
                                message.stateTrajectory], dtype=float)))

    numeric = {topic: deduplicate(rows) for topic, rows in numeric.items()}
    observations = deduplicate(observations)
    schema_ok = bool(all(
        len(numeric[topic]) >= 100 and numeric[topic].ndim == 2 and
        numeric[topic].shape[1] == SIZES[topic] and
        observed_labels[topic] == {LABELS[topic]}
        for topic in LABELS))
    all_finite = bool(schema_ok and len(observations) >= 100 and
                      all(np.all(np.isfinite(value))
                          for value in list(numeric.values()) + [observations]))

    keyed_topics = (UPDATE, V3, FLOW, CLF)
    keyed = {topic: rows_by_version(numeric[topic]) for topic in keyed_topics}
    common = sorted(set.intersection(*(set(value) for value in keyed.values())))
    match_fraction = len(common) / max(
        max(len(keyed[topic]) for topic in keyed_topics), 1)
    arrays = {topic: np.asarray([keyed[topic][version] for version in common])
              for topic in keyed_topics}
    update, v3, flow, clf = (arrays[UPDATE], arrays[V3], arrays[FLOW],
                             arrays[CLF])
    metadata_error = max(
        max_abs(update[:, :2], v3[:, :2]),
        max_abs(update[:, :2], flow[:, :2]),
        max_abs(update[:, :2], clf[:, :2]))
    snapshot_time_error = max(
        np.max(np.abs(update[:, 2])), np.max(np.abs(v3[:, 2])),
        np.max(np.abs(flow[:, 2])), np.max(np.abs(clf[:, 2])))
    versions_ok = bool(len(common) >= 100 and np.all(np.diff(update[:, 3]) > 0))

    raw, baseline, applied = (update[:, 20:30], update[:, 30:40],
                              update[:, 40:50])
    pi_dot, pi_hat, sigma = (update[:, 50:60], update[:, 60:70],
                             update[:, 70:76])
    reconstructed_raw = np.einsum(
        "nij,ni->nj", v3[:, 48:108].reshape(-1, 6, 10), v3[:, 42:48])
    update_closures = {
        "raw_Y_transpose_sigma": max_abs(raw, reconstructed_raw),
        "applied_raw_minus_baseline": max_abs(applied, raw - baseline),
        "pi_dot_Gamma_applied_excluding_first":
            max_abs(pi_dot[1:], applied[1:]),
        "pi_dot_cross_topic": max_abs(pi_dot, v3[:, 32:42]),
        "pi_hat_cross_topic": max_abs(pi_hat, v3[:, 22:32]),
        "sigma_cross_topic": max_abs(sigma, v3[:, 42:48]),
    }
    dt = np.diff(update[:, 0])
    update_closures["explicit_euler"] = max_abs(
        pi_hat[1:] - pi_hat[:-1], dt[:, None] * pi_dot[1:])

    expected_corrected = bool(
        config["enableCompositePhaseBaselineCorrection"])
    configured = update[:, 5] == 1.0
    steady = update[:, 6] == 1.0
    gait_valid = update[:, 7] == 1.0
    key_supported = update[:, 8] == 1.0
    correction = update[:, 9] == 1.0
    expected_steady = ((update[:, 0] >= 4.8 - 1.0e-10) &
                       (update[:, 0] < 28.8 - 1.0e-10))
    stage_flag_ok = bool(
        np.array_equal(steady, expected_steady) if expected_corrected
        else not np.any(steady))
    frozen = load_baseline(config["compositePhaseBaselineFile"])
    active_indices = np.flatnonzero(correction)
    if expected_corrected:
        expected_values = np.asarray([
            frozen[(int(round(update[index, 11])),
                    int(round(update[index, 12])),
                    int(round(update[index, 13])))]
            for index in active_indices])
        lookup_error = max_abs(baseline[active_indices], expected_values)
        mode_contract = bool(
            np.all(configured) and len(active_indices) > 0 and
            np.all(correction == (steady & gait_valid & key_supported)) and
            np.max(np.abs(baseline[~correction])) == 0.0)
    else:
        lookup_error = float(np.max(np.abs(baseline)))
        mode_contract = bool(not np.any(configured) and
                             not np.any(correction) and lookup_error == 0.0)
    active_duration = float(np.sum(np.where(correction[1:], dt, 0.0)))
    lookup_coverage = (active_duration /
                       config["schedule"]["steady_duration_s"]
                       if expected_corrected else 1.0)

    candidate, applied_flow = flow[:, 32:56], flow[:, 56:80]
    candidate_norm = np.linalg.norm(candidate, axis=1)
    applied_flow_norm = np.linalg.norm(applied_flow, axis=1)
    eq10_closures = {
        "candidate_equals_applied": max_abs(candidate, applied_flow),
        "candidate_norm_matches_v3": max_abs(candidate_norm, v3[:, 116]),
        "applied_norm_matches_v3": max_abs(applied_flow_norm, v3[:, 117]),
    }
    eq10_flags = bool(
        np.all(flow[:, 4:8] == np.asarray([1.0, 1.0, 1.0, 1.0])) and
        np.all(v3[:, 4:7] == np.asarray([1.0, 1.0, 1.0])) and
        np.all(v3[:, 8:10] == np.asarray([1.0, 1.0])))
    clf_closures = eq11_closures(clf, np.asarray(config["kd"], dtype=float))
    eq11_flags = bool(
        np.all(clf[:, 4:8] == np.asarray([1.0, 1.0, 1.0, 10.0])) and
        max_abs(pi_hat, clf[:, 19:29]) <= 1.0e-12 and
        max_abs(sigma, clf[:, 13:19]) <= 1.0e-12 and
        max_abs(v3[:, 48:108], clf[:, 59:119]) <= 1.0e-12)

    identity = identities[-1] if identities else {}
    expected_payload = bool(config["truePlantPayloadInertialEnabled"])
    identity_ok = bool(
        identity.get("schema_version") == 2 and
        identity.get("controller_model_id") == "nominal_urdf_srbd_v1" and
        identity.get("payload_inertial_override_active") is expected_payload and
        ((expected_payload and identity.get("payload_mass") == 5.213485 and
          identity.get("plant_model_id") ==
          "srbd_payload_inertial_10d_nominal_coordinates_v2") or
         (not expected_payload and identity.get("payload_mass") == 0 and
          identity.get("plant_model_id") == "nominal_ocs2_rollout_v1")))
    scheduler_identity = scheduler_identities[-1] if scheduler_identities else {}
    scheduler_ok = bool(
        scheduler_identity.get("candidate") == "E3_combined" and
        scheduler_identity.get("profile_id") == protocol["profile_id"] and
        abs(scheduler_identity.get("complete_time", math.inf) - 32.4) <= 1e-12)

    target_ok = False
    if targets:
        target_time, target_state = targets[-1]
        target_ok = bool(
            len(target_time) == 3241 and target_state.shape == (3241, 24) and
            abs(target_time[0]) <= 1e-12 and
            abs(target_time[-1] - 32.4) <= 1e-12 and
            np.all(np.diff(target_time) > 0) and
            np.all(np.isfinite(target_state)))
    coverage_ok = bool(
        len(observations) >= 100 and observations[0, 0] < 2.4 and
        observations[-1, 0] >= 32.4 and numeric[SCHEDULER][-1, 0] >= 32.4)
    tracking = {}
    if target_ok and len(observations):
        desired = interpolate_matrix(observations[:, 0], target_time,
                                     target_state)
        actual = observations[:, 2:26]
        mask = ((observations[:, 0] >= 2.4) &
                (observations[:, 0] <= 31.2))
        position_error = desired[:, 6:9] - actual[:, 6:9]
        orientation_error = ((desired[:, 9:12] - actual[:, 9:12] + np.pi) %
                             (2 * np.pi) - np.pi)
        tracking = {
            "position_rmse_m": float(np.sqrt(np.mean(np.sum(
                position_error[mask] ** 2, axis=1)))),
            "position_peak_m": float(np.max(np.linalg.norm(
                position_error[mask], axis=1))),
            "orientation_rmse_rad": float(np.sqrt(np.mean(np.sum(
                orientation_error[mask] ** 2, axis=1)))),
            "orientation_peak_rad": float(np.max(np.linalg.norm(
                orientation_error[mask], axis=1))),
            "contact_force_peak": float(np.max(np.abs(observations[:, 26:38]))),
            "sigma_peak": float(np.max(np.linalg.norm(sigma, axis=1))),
            "terminal_pi_hat_physical_scaled_norm": float(np.linalg.norm(
                pi_hat[-1] / np.asarray(protocol.get(
                    "physical_coordinate_scales",
                    [5.213485, 0.9697642673581994, 0.9697642673581994,
                     0.9697642673581994, 0.12, 0.12, 0.12, 0.12, 0.12,
                     0.12])))),
        }
    safety_ok = bool(
        tracking and tracking["position_peak_m"] < 0.20 and
        tracking["orientation_peak_rad"] < 0.50 and
        tracking["contact_force_peak"] < 1e5)
    launch_text = (run / "roslaunch.log").read_text(errors="replace")
    runtime_log_ok = re.search(
        r"\[FATAL\]|Segmentation fault|terminate called|ROSException|nan detected",
        launch_text, re.IGNORECASE) is None

    gates = protocol["engineering_gates"]
    checks = {
        "schemas_and_finite": schema_ok and all_finite,
        "frozen_artifact_hashes":
            sha256(run / "frozen_composite_baseline.csv") ==
            config["compositePhaseBaselineSha256"] ==
            protocol["frozen_baseline"]["csv_sha256"] and
            sha256(run / "frozen_composite_baseline_provenance.json") ==
            config["compositePhaseBaselineProvenanceSha256"] ==
            protocol["frozen_baseline"]["provenance_sha256"],
        "controller_plant_identity": identity_ok,
        "scheduler_and_target": scheduler_ok and target_ok and coverage_ok,
        "snapshot_and_cross_topic_sync":
            match_fraction >= 0.99 and metadata_error <= 1e-12 and
            snapshot_time_error <= gates["maximum_snapshot_time_error_s"] and
            versions_ok,
        "mode_and_steady_stage_contract": mode_contract and stage_flag_ok,
        "corrected_lookup_coverage":
            lookup_coverage >= gates[
                "minimum_corrected_steady_lookup_coverage"],
        "frozen_baseline_lookup_exact": lookup_error <= 1e-12,
        "eq8_applied_update_closure":
            max(update_closures.values()) <=
            gates["maximum_pi_dot_closure_error"],
        "eq10_full_flow_applied":
            eq10_flags and max(eq10_closures.values()) <=
            gates["maximum_eq10_closure_error"] and
            np.max(applied_flow_norm) > 1e-12,
        "eq11_algebra_and_shared_snapshot":
            eq11_flags and max(clf_closures.values()) <=
            gates["maximum_eq11_closure_error"],
        "safety_and_runtime": safety_ok and runtime_log_ok,
    }
    passed = all(checks.values())

    csv_path = run / "task_d_d5i4_timeseries.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "time_s", "mode", "steady_active", "gait_valid",
            "key_supported", "correction_applied", "profile_bin",
            "gait_bin", "raw_norm", "baseline_norm", "applied_norm",
            "pi_hat_scaled_norm", "sigma_norm"])
        scales = np.asarray([5.213485, 0.9697642673581994,
                             0.9697642673581994, 0.9697642673581994] +
                            [0.12] * 6)
        for index, row in enumerate(update):
            writer.writerow([
                row[0], int(round(row[13])), row[6], row[7], row[8], row[9],
                int(round(row[11])), int(round(row[12])), row[17], row[18],
                row[19], np.linalg.norm(pi_hat[index] / scales),
                np.linalg.norm(sigma[index])])

    figure, axes = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    axes[0].plot(update[:, 0], np.linalg.norm(raw, axis=1), label="raw Y^T sigma")
    axes[0].plot(update[:, 0], np.linalg.norm(applied, axis=1), label="applied drive")
    axes[0].plot(update[:, 0], np.linalg.norm(baseline, axis=1), label="baseline")
    axes[1].plot(update[:, 0], np.linalg.norm(pi_hat, axis=1), label="||pi_hat||")
    axes[1].plot(update[:, 0], np.linalg.norm(sigma, axis=1), label="||sigma||")
    axes[2].step(update[:, 0], steady.astype(float), where="post", label="steady")
    axes[2].step(update[:, 0], correction.astype(float), where="post", label="applied")
    if tracking:
        desired = interpolate_matrix(observations[:, 0], target_time, target_state)
        axes[3].plot(observations[:, 0], desired[:, 6], "k--", label="desired x")
        axes[3].plot(observations[:, 0], observations[:, 8], label="actual x")
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8)
        axis.axvspan(4.8, 28.8, color="tab:green", alpha=0.06)
    axes[-1].set_xlabel("MPC time [s]")
    figure.suptitle("D5I4 online smoke: " + config["group"])
    figure.tight_layout()
    figure.savefig(run / "figure_d5i4_online_smoke.png", dpi=180)
    plt.close(figure)

    summary = {
        "schema_version": 1,
        "stage": "D5I4_E3_combined_four_mode_online_smoke_run",
        "group": config["group"],
        "passed": passed,
        "formal_eligibility": False,
        "checks": checks,
        "sample_counts": {topic: len(rows)
                          for topic, rows in numeric.items()},
        "cross_topic_match_fraction": match_fraction,
        "lookup": {
            "expected_corrected": expected_corrected,
            "active_sample_count": int(np.sum(correction)),
            "active_duration_s": active_duration,
            "steady_duration_coverage": lookup_coverage,
            "maximum_frozen_value_error": lookup_error,
            "occupied_frozen_keys": len(frozen),
        },
        "closures": {
            "metadata_max_abs_error": metadata_error,
            "snapshot_time_max_abs_error_s": snapshot_time_error,
            "eq8": update_closures,
            "eq10": eq10_closures,
            "eq11": clf_closures,
        },
        "tracking": tracking,
        "model_identity": identity,
        "scheduler_identity": scheduler_identity,
        "terminal_pi_hat_10d": pi_hat[-1].tolist(),
        "scientific_boundary": protocol["scientific_boundary"],
    }
    (run / "task_d_d5i4_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True,
                   default=json_default) + "\n")
    report = """# D5I4 online smoke — {group}

- Gate: **{state}**
- Frozen composite lookup coverage: `{coverage:.3%}`
- Frozen lookup max error: `{lookup:.3g}`
- Eq.(8) max closure: `{eq8:.3g}`
- Eq.(10) max closure: `{eq10:.3g}`
- Eq.(11) max closure: `{eq11:.3g}`
- Position RMSE / peak: `{rmse:.6g}` / `{peak:.6g}` m

This is one-run mechanism/safety smoke. It is not a repeated performance or
parameter-convergence result.
""".format(
        group=config["group"], state="PASS" if passed else "FAIL",
        coverage=lookup_coverage, lookup=lookup_error,
        eq8=max(update_closures.values()), eq10=max(eq10_closures.values()),
        eq11=max(clf_closures.values()),
        rmse=tracking.get("position_rmse_m", math.nan),
        peak=tracking.get("position_peak_m", math.nan))
    (run / "stage_d5i4_smoke_report.md").write_text(report)
    print(report)
    if not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
