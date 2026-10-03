#!/usr/bin/env python3
"""Compare the four D5H1 estimator-baseline smoke runs.

This is a mechanism/safety comparison only.  It intentionally does not turn
one short run per mode into a robustness or payload-identification claim.
"""

import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz",
              "Jyy", "Jyz", "Jzz")
GROUPS = ("nominal_raw", "nominal_corrected",
          "uncertain_raw", "uncertain_corrected")


def load_run(root, group):
    run = root / group / "run_01"
    summary = json.loads(
        (run / "task_d_d5h1_smoke_summary.json").read_text())
    config = json.loads((run / "effective_config.json").read_text())
    with (run / "task_d_d5h1_timeseries.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("empty D5H1 timeseries: {}".format(run))
    terminal = np.asarray(
        [float(rows[-1]["pi_hat_" + name]) for name in PARAMETERS])
    return {"run": run, "summary": summary, "config": config,
            "terminal": terminal}


def truth_and_scales(config):
    payload = config["payload"]
    mass = float(payload["mass_kg"])
    com = np.asarray(payload["com_nominal_c0_base_m"], dtype=float)
    inertia = np.asarray(payload["inertia_com_base_kg_m2"], dtype=float)
    truth = np.asarray([mass, *(mass * com), *inertia], dtype=float)
    length = float(np.linalg.norm(com))
    inertia_scale = max(abs(float(value)) for value in inertia)
    scales = np.asarray([mass] + [mass * length] * 3 +
                        [inertia_scale] * 6, dtype=float)
    if np.any(scales <= 0.0):
        raise ValueError("invalid D5H1 physical coordinate scale")
    return truth, scales


def scaled_norm(vector, scales):
    return float(np.linalg.norm(np.asarray(vector) / scales))


def scaled_rmse(vector, truth, scales):
    return float(np.sqrt(np.mean(((np.asarray(vector) - truth) / scales) ** 2)))


def scaled_cosine(vector, truth, scales):
    left = np.asarray(vector) / scales
    right = np.asarray(truth) / scales
    denominator = np.linalg.norm(left) * np.linalg.norm(right)
    return float(np.dot(left, right) / denominator) if denominator > 0.0 else None


def scientific_invariants(config):
    keys = (
        "parameterization", "enableParameterUpdate", "enableAdaptiveDynamics",
        "enableClfConstraint", "enableInputBiasCost", "useSoftConstraint",
        "softConstraintMu", "softConstraintDelta", "gamma", "kd",
        "lambdaLinear", "lambdaAngular", "initialParameterEstimate",
        "minimumEstimatedTotalMass", "timestampTolerance", "payload",
        "target", "gait", "fixed_files", "gaitPhaseBins",
        "gaitCycleDuration", "gaitBaselineMinimumCycleStartTime",
        "gaitPhaseBaselineSha256",
        "scheduler_condition", "acquisition_end_mpc_time_s",
    )
    return {key: config[key] for key in keys}


def main():
    if len(sys.argv) not in (2, 3):
        raise SystemExit(
            "usage: analyze_task_d_d5h1_smoke_set.py SMOKE_ROOT [OUTPUT_DIR]")
    root = Path(sys.argv[1]).resolve()
    output = (Path(sys.argv[2]).resolve() if len(sys.argv) == 3
              else root / "analysis")
    output.mkdir(parents=True, exist_ok=True)

    runs = {group: load_run(root, group) for group in GROUPS}
    truth, scales = truth_and_scales(runs["uncertain_corrected"]["config"])
    invariants = [scientific_invariants(runs[group]["config"])
                  for group in GROUPS]
    fairness = all(item == invariants[0] for item in invariants[1:])
    mode_identity = all(
        runs[group]["config"]["group"] == group for group in GROUPS)
    correction_switches = bool(
        not runs["nominal_raw"]["config"]["enableGaitPhaseBaselineCorrection"]
        and runs["nominal_corrected"]["config"]["enableGaitPhaseBaselineCorrection"]
        and not runs["uncertain_raw"]["config"]["enableGaitPhaseBaselineCorrection"]
        and runs["uncertain_corrected"]["config"]["enableGaitPhaseBaselineCorrection"])
    plant_switches = bool(
        not runs["nominal_raw"]["config"]["truePlantPayloadInertialEnabled"]
        and not runs["nominal_corrected"]["config"]["truePlantPayloadInertialEnabled"]
        and runs["uncertain_raw"]["config"]["truePlantPayloadInertialEnabled"]
        and runs["uncertain_corrected"]["config"]["truePlantPayloadInertialEnabled"])
    all_single_run_checks_pass = all(
        bool(runs[group]["summary"]["passed"]) for group in GROUPS)
    active_counts_valid = bool(
        runs["nominal_raw"]["summary"]["correction_active_sample_count"] == 0
        and runs["uncertain_raw"]["summary"]["correction_active_sample_count"] == 0
        and runs["nominal_corrected"]["summary"]["correction_active_sample_count"] > 0
        and runs["uncertain_corrected"]["summary"]["correction_active_sample_count"] > 0)

    n_raw = runs["nominal_raw"]["terminal"]
    n_corrected = runs["nominal_corrected"]["terminal"]
    u_raw = runs["uncertain_raw"]["terminal"]
    u_corrected = runs["uncertain_corrected"]["terminal"]
    raw_contrast = u_raw - n_raw
    corrected_contrast = u_corrected - n_corrected

    n_raw_norm = scaled_norm(n_raw, scales)
    n_corrected_norm = scaled_norm(n_corrected, scales)
    u_raw_norm = scaled_norm(u_raw, scales)
    u_corrected_norm = scaled_norm(u_corrected, scales)
    raw_contrast_norm = scaled_norm(raw_contrast, scales)
    corrected_contrast_norm = scaled_norm(corrected_contrast, scales)
    metrics = {
        "nominal_terminal_scaled_norm_raw": n_raw_norm,
        "nominal_terminal_scaled_norm_corrected": n_corrected_norm,
        "nominal_terminal_norm_suppression_fraction":
            1.0 - n_corrected_norm / n_raw_norm,
        "uncertain_terminal_scaled_norm_raw": u_raw_norm,
        "uncertain_terminal_scaled_norm_corrected": u_corrected_norm,
        "uncertain_terminal_norm_reduction_fraction":
            1.0 - u_corrected_norm / u_raw_norm,
        "raw_payload_contrast_scaled_norm": raw_contrast_norm,
        "corrected_payload_contrast_scaled_norm": corrected_contrast_norm,
        "payload_contrast_norm_retention_fraction":
            corrected_contrast_norm / raw_contrast_norm,
        "raw_payload_contrast_scaled_rmse_vs_pi_star":
            scaled_rmse(raw_contrast, truth, scales),
        "corrected_payload_contrast_scaled_rmse_vs_pi_star":
            scaled_rmse(corrected_contrast, truth, scales),
        "raw_payload_contrast_scaled_cosine_vs_pi_star":
            scaled_cosine(raw_contrast, truth, scales),
        "corrected_payload_contrast_scaled_cosine_vs_pi_star":
            scaled_cosine(corrected_contrast, truth, scales),
        "nominal_x_rmse_m_descriptive_only":
            runs["nominal_corrected"]["summary"]["metrics"]
                ["x_rmse_m_descriptive_only"],
        "uncertain_x_rmse_m_descriptive_only":
            runs["uncertain_corrected"]["summary"]["metrics"]
                ["x_rmse_m_descriptive_only"],
    }
    mechanism_pass = bool(fairness and mode_identity and correction_switches
                          and plant_switches and all_single_run_checks_pass
                          and active_counts_valid)

    rows = []
    for index, name in enumerate(PARAMETERS):
        rows.append({
            "parameter": name,
            "scale": float(scales[index]),
            "pi_star": float(truth[index]),
            "nominal_raw": float(n_raw[index]),
            "uncertain_raw": float(u_raw[index]),
            "raw_payload_contrast": float(raw_contrast[index]),
            "nominal_corrected": float(n_corrected[index]),
            "uncertain_corrected": float(u_corrected[index]),
            "corrected_payload_contrast": float(corrected_contrast[index]),
        })
    with (output / "stage_d5h1_terminal_parameter_comparison.csv").open(
            "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "stage": "D5H1_estimator_phase_baseline_four_mode_smoke",
        "scope": "one short run per mode; mechanism and safety only",
        "checks": {
            "all_single_run_checks_pass": all_single_run_checks_pass,
            "scientific_invariants_identical": fairness,
            "mode_identity": mode_identity,
            "only_expected_correction_switches": correction_switches,
            "only_expected_plant_switches": plant_switches,
            "correction_active_counts_valid": active_counts_valid,
        },
        "mechanism_smoke_passed": mechanism_pass,
        "formal_eligibility": False,
        "payload_pi_star": truth.tolist(),
        "physical_coordinate_scales": dict(zip(PARAMETERS, scales.tolist())),
        "terminal_vectors": {
            group: runs[group]["terminal"].tolist() for group in GROUPS
        },
        "payload_contrasts": {
            "raw_uncertain_minus_nominal": raw_contrast.tolist(),
            "corrected_uncertain_minus_nominal": corrected_contrast.tolist(),
        },
        "metrics": metrics,
        "interpretation_boundary": (
            "D5H1 subtracts a nominal steady-gait phase baseline from the "
            "actual Eq.(8) update drive. It starts only after a complete cycle "
            "whose start time is >=2.2 s, so startup/target-transient bias "
            "already integrated before that time is not removed."),
        "next_gate": (
            "Do not claim estimator robustness or 10D convergence from this "
            "smoke. Complete the provenance and Eq.(10)/(11) strict Gate, then "
            "freeze and execute the repeated four-mode protocol."),
    }
    (output / "stage_d5h1_smoke_set_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")

    x = np.arange(len(PARAMETERS))
    figure, axes = plt.subplots(2, 1, figsize=(12, 8), constrained_layout=True)
    axes[0].plot(x, n_raw / scales, "o-", label="Nominal raw")
    axes[0].plot(x, n_corrected / scales, "o-", label="Nominal corrected")
    axes[0].axhline(0.0, color="black", linewidth=0.7)
    axes[0].set_ylabel("terminal pi_hat / physical scale")
    axes[0].set_title("Nominal false estimate: actual estimator output")
    axes[0].legend()
    axes[0].grid(alpha=0.25)
    axes[1].plot(x, truth / scales, "ko--", label="payload pi_star")
    axes[1].plot(x, raw_contrast / scales, "o-", label="raw U-N")
    axes[1].plot(x, corrected_contrast / scales, "o-", label="corrected U-N")
    axes[1].axhline(0.0, color="black", linewidth=0.7)
    axes[1].set_ylabel("parameter / physical scale")
    axes[1].set_title("Paired payload contrast (vector subtraction)")
    axes[1].set_xticks(x, PARAMETERS)
    axes[1].legend()
    axes[1].grid(alpha=0.25)
    figure.savefig(output / "figure_D5H1_four_mode_terminal_comparison.png",
                   dpi=180)
    plt.close(figure)

    table = "\n".join(
        "| {parameter} | {pi_star:.6g} | {nominal_raw:.6g} | "
        "{uncertain_raw:.6g} | {raw_payload_contrast:.6g} | "
        "{nominal_corrected:.6g} | {uncertain_corrected:.6g} | "
        "{corrected_payload_contrast:.6g} |".format(**row)
        for row in rows)
    report = """# Stage D5H1 four-mode smoke report

## Gate result

- Mechanism/safety smoke: **{gate}**
- Formal scientific eligibility: **NO** (one short run per mode)
- Scientific invariants identical: `{fairness}`
- All four per-run schema/synchronization/Euler/finite checks: `{single}`

## Terminal 10D comparison

| parameter | pi_star | N raw | U raw | raw U-N | N corrected | U corrected | corrected U-N |
|---|---:|---:|---:|---:|---:|---:|---:|
{table}

Physical-scale vector metrics:

- Nominal terminal norm: raw `{nraw:.8g}` -> corrected `{ncorr:.8g}`; reduction **{nsupp:.2%}**.
- Uncertain terminal norm: raw `{uraw:.8g}` -> corrected `{ucorr:.8g}`; reduction **{usupp:.2%}**.
- Payload contrast norm: raw `{craw:.8g}`, corrected `{ccorr:.8g}`; retention **{cret:.2%}**.
- Payload contrast RMSE vs pi_star: raw `{rraw:.8g}`, corrected `{rcorr:.8g}`.
- Payload contrast cosine vs pi_star: raw `{cosraw:.8g}`, corrected `{coscorr:.8g}`.

## Interpretation boundary

The D5H1 correction is in the actual Eq.(8) estimator update path, and all
logged algebraic/integration closures pass.  It suppresses steady walking
phase-correlated drift.  It does **not** erase estimate accumulated during
startup and the target transient, because the baseline deliberately activates
only after a complete gait cycle starting at or after 2.2 s.  Therefore a
nonzero corrected Nominal terminal estimate remains, and this smoke cannot be
called a complete estimator-robustness solution or 10D parameter convergence.
""".format(
        gate="PASS" if mechanism_pass else "FAIL", fairness=fairness,
        single=all_single_run_checks_pass, table=table,
        nraw=n_raw_norm, ncorr=n_corrected_norm,
        nsupp=metrics["nominal_terminal_norm_suppression_fraction"],
        uraw=u_raw_norm, ucorr=u_corrected_norm,
        usupp=metrics["uncertain_terminal_norm_reduction_fraction"],
        craw=raw_contrast_norm, ccorr=corrected_contrast_norm,
        cret=metrics["payload_contrast_norm_retention_fraction"],
        rraw=metrics["raw_payload_contrast_scaled_rmse_vs_pi_star"],
        rcorr=metrics["corrected_payload_contrast_scaled_rmse_vs_pi_star"],
        cosraw=metrics["raw_payload_contrast_scaled_cosine_vs_pi_star"],
        coscorr=metrics["corrected_payload_contrast_scaled_cosine_vs_pi_star"])
    (output / "stage_d5h1_smoke_set_report.md").write_text(report)
    print(report)
    if not mechanism_pass:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
