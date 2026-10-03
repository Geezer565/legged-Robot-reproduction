#!/usr/bin/env python3
"""Strict per-run D5H1 provenance, phase, Eq.(10), and Eq.(11) gate."""

import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import rosbag


PACKAGE = Path(__file__).resolve().parents[1]
CORE = PACKAGE.parent / "ocs2_legged_robot_adaptive"
D5H1 = "/task_d/d5h1_applied_update_sample"
V3 = "/task_d/payload_inertial_10d_v3_sample"
FLOW = "/task_d/equation10_full_flow_sample"
CLF = "/task_d/d5h1_eq11_sample"
LABELS = {
    D5H1: "payload_inertial_d5h1_applied_v1_71_fields",
    V3: "payload_inertial_10d_v3_118_fields",
    FLOW: "payload_inertial_eq10_full_flow_v2_80_fields",
    CLF: "payload_inertial_d5h1_eq11_v1_119_fields",
}
SIZES = {D5H1: 71, V3: 118, FLOW: 80, CLF: 119}
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz",
              "Jyy", "Jyz", "Jzz")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def max_abs(left, right):
    left = np.asarray(left, dtype=float)
    right = np.asarray(right, dtype=float)
    return float(np.max(np.abs(left - right))) if left.size else math.inf


def eq11_closures(clf, kd):
    clf = np.asarray(clf, dtype=float)
    kd = np.asarray(kd, dtype=float)
    sigma = clf[:, 13:19]
    pi_hat = clf[:, 19:29]
    contact = clf[:, 29:35]
    nominal = clf[:, 35:41]
    estimated = clf[:, 41:47]
    residual = clf[:, 47:53]
    contribution = clf[:, 53:59]
    regressor = clf[:, 59:119].reshape(-1, 6, 10)
    expected_estimated = np.einsum("nij,nj->ni", regressor, pi_hat)
    expected_residual = contact - nominal - estimated
    expected_contribution = sigma * residual
    expected_linear = np.sum(contribution, axis=1)
    expected_damping = 0.5 * np.einsum(
        "ni,ij,nj->n", sigma, kd, sigma)
    expected_h = expected_linear - expected_damping
    return {
        "estimated_wrench_Y_pi": max_abs(estimated, expected_estimated),
        "residual_wrench": max_abs(residual, expected_residual),
        "component_contribution": max_abs(contribution, expected_contribution),
        "linear_term": max_abs(clf[:, 11], expected_linear),
        "damping_term": max_abs(clf[:, 12], expected_damping),
        "reconstructed_h": max_abs(clf[:, 9], expected_h),
        "h_clf": max_abs(clf[:, 8], expected_h),
        "violation": max_abs(clf[:, 10], np.maximum(0.0, -clf[:, 8])),
    }


def load_phase_baseline(path):
    values = np.full((24, 10), np.nan)
    parameter_index = {name: index for index, name in enumerate(PARAMETERS)}
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        phase = int(row["phase_bin"])
        parameter = parameter_index[row["parameter"]]
        values[phase, parameter] = float(row["nominal_baseline_drive_rate"])
    if not np.all(np.isfinite(values)):
        raise ValueError("D5H1 phase baseline is incomplete")
    return values


def rows_by_version(rows):
    return {int(round(row[3])): row for row in rows}


def provenance_audit(run, config):
    baseline = Path(config["gaitPhaseBaselineFile"])
    application = Path(config["gaitPhaseBaselineApplicationProvenanceFile"])
    copied_baseline = run / "frozen_phase_baseline.csv"
    copied_application = run / "phase_baseline_application_provenance.json"
    provenance = json.loads(application.read_text())
    training = provenance["training"]
    validation = provenance["held_out_shadow_validation"]
    source = Path(training["source_directory"])
    source_baseline = source / training["source_baseline_file"]
    source_summary = source / training["source_summary_file"]
    source_manifest = source / training["source_manifest_file"]
    validation_summary = Path(validation["source_summary"])
    d5g = json.loads(source_summary.read_text())
    d5h0 = json.loads(validation_summary.read_text())
    canonical_source = source_baseline.read_bytes().replace(b"\r\n", b"\n")
    checks = {
        "packaged_baseline_hash": sha256(baseline)
            == config["gaitPhaseBaselineSha256"]
            == provenance["baseline"]["packaged_canonical_lf_sha256"],
        "run_baseline_copy_hash": sha256(copied_baseline) == sha256(baseline),
        "application_provenance_hash": sha256(application)
            == config["gaitPhaseBaselineApplicationProvenanceSha256"],
        "run_application_copy_hash": sha256(copied_application)
            == sha256(application),
        "source_baseline_hash": sha256(source_baseline)
            == training["source_baseline_sha256"],
        "source_baseline_canonical_content":
            hashlib.sha256(canonical_source).hexdigest() == sha256(baseline),
        "source_summary_hash_and_pass": sha256(source_summary)
            == training["source_summary_sha256"] and d5g.get("passed") is True,
        "source_manifest_hash_and_pass": sha256(source_manifest)
            == training["source_manifest_sha256"]
            and json.loads(source_manifest.read_text()).get("passed") is True,
        "held_out_validation_hash_and_pass": sha256(validation_summary)
            == validation["source_summary_sha256"]
            and d5h0.get("passed") is True
            and all(d5h0.get("gates", {}).values()),
        "nominal_training_only":
            training["uncertain_or_payload_data_used"] is False
            and config["gaitPhaseBaselineTrainingUsesPayloadData"] is False,
        "no_rejected_gate_branch":
            provenance["startup_gate_used"] is False
            and provenance["confidence_gate_used"] is False
            and provenance["gamma_modified"] is False,
    }
    return checks, provenance


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: audit_task_d_d5h1_gate.py RUN_DIR")
    run = Path(sys.argv[1]).resolve()
    config = json.loads((run / "effective_config.json").read_text())
    smoke = json.loads((run / "task_d_d5h1_smoke_summary.json").read_text())
    collected = {topic: [] for topic in LABELS}
    observed_labels = {topic: set() for topic in LABELS}
    with rosbag.Bag(str(run / "task_d_d5h1.bag")) as bag:
        for topic, message, _ in bag.read_messages(topics=list(LABELS)):
            observed_labels[topic].add(
                message.layout.dim[0].label if message.layout.dim else "")
            collected[topic].append(
                np.asarray(message.data, dtype=float))
    for topic in collected:
        collected[topic].sort(key=lambda row: row[0])

    schema = bool(all(
        len(collected[topic]) >= 100
        and all(len(row) == SIZES[topic] for row in collected[topic])
        and observed_labels[topic] == {LABELS[topic]}
        for topic in LABELS))
    finite = bool(schema and all(
        np.all(np.isfinite(np.asarray(collected[topic]))) for topic in LABELS))
    keyed = {topic: rows_by_version(collected[topic]) for topic in LABELS}
    common = sorted(set.intersection(*(set(values) for values in keyed.values())))
    match_fraction = len(common) / max(max(len(values) for values in keyed.values()), 1)
    arrays = {topic: np.asarray([keyed[topic][version] for version in common])
              for topic in LABELS}
    d5h1, v3, flow, clf = (arrays[D5H1], arrays[V3], arrays[FLOW], arrays[CLF])

    metadata_error = max(
        max_abs(d5h1[:, :2], v3[:, :2]),
        max_abs(d5h1[:, :2], flow[:, :2]),
        max_abs(d5h1[:, :2], clf[:, :2]),
        max_abs(d5h1[:, 3], v3[:, 3]),
        max_abs(d5h1[:, 3], flow[:, 3]),
        max_abs(d5h1[:, 3], clf[:, 3]))
    time_error = float(max(
        np.max(np.abs(d5h1[:, 2])), np.max(np.abs(v3[:, 2])),
        np.max(np.abs(flow[:, 2])), np.max(np.abs(clf[:, 2]))))
    snapshot = bool(
        match_fraction >= 0.99 and metadata_error <= 1.0e-12
        and time_error <= 1.0e-6 and np.all(np.diff(d5h1[:, 3]) > 0.0))

    pi_error = max(max_abs(d5h1[:, 55:65], v3[:, 22:32]),
                   max_abs(d5h1[:, 55:65], clf[:, 19:29]))
    sigma_error = max(max_abs(d5h1[:, 65:71], v3[:, 42:48]),
                      max_abs(d5h1[:, 65:71], clf[:, 13:19]))
    y_error = max_abs(v3[:, 48:108], clf[:, 59:119])

    candidate = flow[:, 32:56]
    applied = flow[:, 56:80]
    candidate_norm = np.linalg.norm(candidate, axis=1)
    applied_norm = np.linalg.norm(applied, axis=1)
    eq10_closures = {
        "candidate_equals_applied": max_abs(candidate, applied),
        "candidate_norm_matches_v3": max_abs(candidate_norm, v3[:, 116]),
        "applied_norm_matches_v3": max_abs(applied_norm, v3[:, 117]),
    }
    eq10_flags = bool(
        np.all(flow[:, 4:8] == np.asarray([1.0, 1.0, 1.0, 1.0]))
        and np.all(v3[:, 4:7] == np.asarray([1.0, 1.0, 1.0]))
        and np.all(v3[:, 8:10] == np.asarray([1.0, 1.0])))
    eq10 = bool(eq10_flags and max(eq10_closures.values()) <= 1.0e-12
                and np.max(applied_norm) > 1.0e-12)

    kd = np.asarray(config["kd"], dtype=float)
    clf_closures = eq11_closures(clf, kd)
    eq11_flags = bool(np.all(clf[:, 4:8] == np.asarray([1.0, 1.0, 1.0, 10.0])))
    eq11 = bool(eq11_flags and max(clf_closures.values()) <= 1.0e-10
                and pi_error <= 1.0e-12 and sigma_error <= 1.0e-12
                and y_error <= 1.0e-12)

    provenance_checks, provenance = provenance_audit(run, config)
    provenance_ok = all(provenance_checks.values())
    baseline = load_phase_baseline(config["gaitPhaseBaselineFile"])
    expected_corrected = bool(config["enableGaitPhaseBaselineCorrection"])
    configured = d5h1[:, 5] == 1.0
    active = d5h1[:, 7] == 1.0
    baseline_rows = d5h1[:, 25:35]
    if expected_corrected:
        active_indices = np.flatnonzero(active)
        lookup_error = max_abs(
            baseline_rows[active_indices],
            baseline[d5h1[active_indices, 9].astype(int)])
        phase_contract = bool(
            np.all(configured) and len(active_indices)
            and np.all(d5h1[active_indices, 12] >= 2.2)
            and np.all((d5h1[active_indices, 9] >= 0)
                       & (d5h1[active_indices, 9] < 24))
            and lookup_error <= 1.0e-12)
    else:
        lookup_error = float(np.max(np.abs(baseline_rows)))
        phase_contract = bool(not np.any(configured) and not np.any(active)
                              and lookup_error == 0.0)
    corrector_source = (CORE / "src" / "adaptive" /
                        "GaitPhaseBaselineCorrector.cpp").read_text()
    causality_static = bool(
        "GaitPhaseBaselineCorrector::sample(" in corrector_source
        and "scalar_t time, std::size_t mode" in corrector_source
        and "previousMode_" in corrector_source
        and "minimumCycleStartTime_" in corrector_source
        and "pi_star" not in corrector_source)

    constraint_source = (CORE / "src" / "constraint" /
                         "AdaptiveClfConstraint.cpp").read_text()
    evaluator_source = (CORE / "src" / "adaptive" /
                        "AdaptiveClfEvaluator.cpp").read_text()
    shared_eq11_implementation = bool(
        "evaluator_.evaluate(time, state, input, modelData_->snapshot())"
        in constraint_source
        and "diagnostics.reconstructedH = diagnostics.clfLinearTerm - diagnostics.dampingTerm"
        in evaluator_source)

    checks = {
        "existing_smoke_passed": bool(smoke["passed"]),
        "schemas_and_finite": schema and finite,
        "snapshot_and_cross_topic_sync": snapshot,
        "baseline_provenance": provenance_ok,
        "gait_phase_lookup": phase_contract,
        "phase_lookup_causality_static": causality_static,
        "eq10_full_flow_applied_path": eq10,
        "eq11_algebra_and_snapshot": eq11,
        "constraint_and_monitor_share_eq11_implementation":
            shared_eq11_implementation,
    }
    passed = all(checks.values())
    summary = {
        "schema_version": 1,
        "stage": "D5H1_strict_gate_audit",
        "group": config["group"],
        "passed": passed,
        "checks": checks,
        "sample_counts": {topic: len(rows) for topic, rows in collected.items()},
        "cross_topic_match_fraction": match_fraction,
        "closures": {
            "metadata_max_abs_error": metadata_error,
            "snapshot_time_max_abs_error_s": time_error,
            "pi_hat_cross_topic_max_abs_error": pi_error,
            "sigma_cross_topic_max_abs_error": sigma_error,
            "regressor_cross_topic_max_abs_error": y_error,
            "phase_baseline_lookup_max_abs_error": lookup_error,
            "eq10": eq10_closures,
            "eq11": clf_closures,
        },
        "provenance_checks": provenance_checks,
        "provenance": provenance,
        "formal_eligibility": bool(passed and config["run_scope"] == "formal"),
        "boundary": (
            "This gate proves provenance, causal lookup, snapshot alignment, "
            "and Eq.(10)/(11) applied-path algebra. It does not prove tracking "
            "improvement or 10D parameter convergence."),
    }
    (run / "d5h1_gate_audit_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    report = """# D5H1 strict gate audit: {group}

- Result: **{result}**
- Common synchronized samples: `{samples}` ({fraction:.2%})
- Baseline provenance: `{provenance}`
- Causal gait/phase lookup: `{phase}`
- Snapshot synchronization: `{snapshot}`
- Eq.(10) full-flow applied path: `{eq10}`
- Eq.(11) algebra/shared snapshot: `{eq11}`
- Constraint and monitor share the same evaluator implementation: `{shared}`

Maximum Eq.(10) closure error: `{eq10_error:.3g}`.
Maximum Eq.(11) closure error: `{eq11_error:.3g}`.

This is an implementation/mechanism gate, not a performance or parameter-convergence conclusion.
""".format(
        group=config["group"], result="PASS" if passed else "FAIL",
        samples=len(common), fraction=match_fraction, provenance=provenance_ok,
        phase=phase_contract and causality_static, snapshot=snapshot, eq10=eq10,
        eq11=eq11, shared=shared_eq11_implementation,
        eq10_error=max(eq10_closures.values()),
        eq11_error=max(clf_closures.values()))
    (run / "d5h1_gate_audit_report.md").write_text(report)
    print(report)
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
