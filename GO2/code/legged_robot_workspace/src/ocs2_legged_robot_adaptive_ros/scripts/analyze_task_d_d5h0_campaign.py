#!/usr/bin/env python3
"""Paired D5H0 shadow-only robustness and payload-contrast analysis."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz",
              "Jyy", "Jyz", "Jzz")
GROUPS = ("nominal", "uncertain")


def sample_std(values, axis=0):
    values = np.asarray(values, dtype=float)
    if values.shape[axis] <= 1:
        return np.zeros(values.shape[1:] if axis == 0 else ())
    return np.std(values, axis=axis, ddof=1)


def cosine(left, right):
    denominator = np.linalg.norm(left) * np.linalg.norm(right)
    return float(np.dot(left, right) / denominator) if denominator > 0.0 else 0.0


def load_runs(root, expected_runs):
    runs = {group: [] for group in GROUPS}
    baseline_hashes = set()
    for group in GROUPS:
        for run_index in range(1, expected_runs + 1):
            path = root / group / "run_{:02d}".format(run_index)
            summary = json.loads((path / "task_d_d5h0_run_summary.json").read_text())
            config = json.loads((path / "effective_config.json").read_text())
            if (summary["group"] != group or summary["run_index"] != run_index or
                    not summary["formal_eligibility"] or not summary["passed"]):
                raise ValueError("ineligible D5H0 run: {}".format(path))
            if config["stage"] != "D5H0_shadow_diagnostics":
                raise ValueError("D5H0 config stage mismatch")
            baseline_hashes.add(summary["baseline_sha256"])
            runs[group].append({"path": path, "summary": summary, "config": config})
    if len(baseline_hashes) != 1:
        raise ValueError("D5H0 runs do not use one frozen baseline")
    return runs, next(iter(baseline_hashes))


def vector(run, key):
    values = run["summary"]["terminal"][key]
    return np.asarray([values[name] for name in PARAMETERS], dtype=float)


def analyze(root_directory, output_directory, expected_runs=3):
    root, output = Path(root_directory), Path(output_directory)
    output.mkdir(parents=True, exist_ok=False)
    runs, baseline_hash = load_runs(root, expected_runs)
    nominal_raw = np.vstack([vector(run, "raw_parameter_change")
                             for run in runs["nominal"]])
    nominal_corrected = np.vstack([vector(run, "corrected_parameter_change")
                                   for run in runs["nominal"]])
    uncertain_raw = np.vstack([vector(run, "raw_parameter_change")
                               for run in runs["uncertain"]])
    uncertain_corrected = np.vstack([vector(run, "corrected_parameter_change")
                                     for run in runs["uncertain"]])
    truth = np.asarray([runs["uncertain"][0]["summary"]["payload_truth"][name]
                        for name in PARAMETERS], dtype=float)
    scales = np.asarray([runs["uncertain"][0]["summary"][
        "physical_coordinate_scales"][name] for name in PARAMETERS], dtype=float)
    raw_contrast = uncertain_raw - nominal_raw
    corrected_contrast = uncertain_corrected - nominal_corrected

    nominal_raw_norm = np.linalg.norm(nominal_raw / scales[None, :], axis=1)
    nominal_corrected_norm = np.linalg.norm(
        nominal_corrected / scales[None, :], axis=1)
    raw_suppression = 1.0 - np.mean(nominal_corrected_norm) / np.mean(nominal_raw_norm)
    raw_error = (raw_contrast - truth[None, :]) / scales[None, :]
    corrected_error = (corrected_contrast - truth[None, :]) / scales[None, :]
    raw_rmse = float(np.sqrt(np.mean(raw_error ** 2)))
    corrected_rmse = float(np.sqrt(np.mean(corrected_error ** 2)))
    raw_cosine = cosine(np.mean(raw_contrast, axis=0) / scales, truth / scales)
    corrected_cosine = cosine(
        np.mean(corrected_contrast, axis=0) / scales, truth / scales)
    contrast_retention = float(
        np.linalg.norm(np.mean(corrected_contrast, axis=0) / scales) /
        np.linalg.norm(np.mean(raw_contrast, axis=0) / scales))

    closure_pass = all(all(run["summary"]["checks"].values())
                       for group in GROUPS for run in runs[group])
    gates = {
        "all_formal_runs_valid": bool(closure_pass),
        "nominal_mean_suppression_at_least_95_percent": bool(
            raw_suppression >= 0.95),
        "every_nominal_run_improves": bool(np.all(
            nominal_corrected_norm < nominal_raw_norm)),
        "payload_contrast_norm_retention_at_least_95_percent":
            bool(contrast_retention >= 0.95),
        "payload_scaled_rmse_not_worse_by_more_than_5_percent":
            bool(corrected_rmse <= 1.05 * raw_rmse),
        "payload_scaled_cosine_not_lower_by_more_than_0_05":
            bool(corrected_cosine >= raw_cosine - 0.05),
    }
    passed = all(gates.values())

    rows = []
    for run_index in range(expected_runs):
        for group, raw, corrected in (
                ("nominal", nominal_raw, nominal_corrected),
                ("uncertain", uncertain_raw, uncertain_corrected)):
            row = {"group": group, "run_index": run_index + 1}
            for index, name in enumerate(PARAMETERS):
                row["raw_" + name] = raw[run_index, index]
                row["corrected_" + name] = corrected[run_index, index]
            row["raw_scaled_norm"] = float(np.linalg.norm(raw[run_index] / scales))
            row["corrected_scaled_norm"] = float(
                np.linalg.norm(corrected[run_index] / scales))
            rows.append(row)
    with (output / "d5h0_terminal_metrics_per_run.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)

    component_rows = []
    for index, name in enumerate(PARAMETERS):
        component_rows.append({
            "parameter": name,
            "nominal_raw_mean": float(np.mean(nominal_raw[:, index])),
            "nominal_raw_sample_std": float(sample_std(nominal_raw[:, index])),
            "nominal_corrected_mean": float(np.mean(nominal_corrected[:, index])),
            "nominal_corrected_sample_std": float(sample_std(nominal_corrected[:, index])),
            "raw_payload_contrast_mean": float(np.mean(raw_contrast[:, index])),
            "raw_payload_contrast_sample_std": float(sample_std(raw_contrast[:, index])),
            "corrected_payload_contrast_mean": float(np.mean(corrected_contrast[:, index])),
            "corrected_payload_contrast_sample_std": float(sample_std(corrected_contrast[:, index])),
            "pi_star": float(truth[index]),
        })
    with (output / "d5h0_component_summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(component_rows[0]))
        writer.writeheader(); writer.writerows(component_rows)

    summary = {
        "schema_version": 1,
        "stage": "D5H0_formal_shadow_campaign",
        "passed": passed,
        "gate": "D5H0",
        "shadow_only": True,
        "actual_estimator_modified": False,
        "runs_per_group": expected_runs,
        "groups": list(GROUPS),
        "baseline_sha256": baseline_hash,
        "nominal": {
            "raw_scaled_terminal_norm_mean": float(np.mean(nominal_raw_norm)),
            "raw_scaled_terminal_norm_sample_std": float(sample_std(nominal_raw_norm)),
            "corrected_scaled_terminal_norm_mean": float(np.mean(nominal_corrected_norm)),
            "corrected_scaled_terminal_norm_sample_std": float(sample_std(nominal_corrected_norm)),
            "suppression_fraction": float(raw_suppression),
        },
        "payload_contrast": {
            "raw_scaled_rmse_vs_pi_star": raw_rmse,
            "corrected_scaled_rmse_vs_pi_star": corrected_rmse,
            "raw_scaled_cosine_vs_pi_star": raw_cosine,
            "corrected_scaled_cosine_vs_pi_star": corrected_cosine,
            "corrected_to_raw_scaled_norm_ratio": contrast_retention,
        },
        "gates": gates,
        "interpretation": (
            "D5H0 validates an online shadow counterfactual on new held-out runs. "
            "No corrected update entered Eq.(8), snapshot, Eq.(10), policy, or plant."),
    }
    (output / "stage_d5h0_campaign_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")

    figure, axes = plt.subplots(1, 2, figsize=(13, 5))
    positions = np.arange(expected_runs)
    axes[0].bar(positions - 0.18, nominal_raw_norm, 0.36, label="raw")
    axes[0].bar(positions + 0.18, nominal_corrected_norm, 0.36,
                label="phase-baseline shadow")
    axes[0].set_xticks(positions)
    axes[0].set_xticklabels(["run_{:02d}".format(index + 1)
                             for index in positions])
    axes[0].set_ylabel("Nominal scaled terminal norm")
    axes[0].legend(); axes[0].grid(True, axis="y", alpha=0.3)
    x = np.arange(10)
    axes[1].plot(x, truth / scales, "ko-", label="pi_star")
    axes[1].plot(x, np.mean(raw_contrast, axis=0) / scales,
                 "o--", label="raw U-N contrast")
    axes[1].plot(x, np.mean(corrected_contrast, axis=0) / scales,
                 "s--", label="corrected U-N contrast")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(PARAMETERS, rotation=45)
    axes[1].set_ylabel("dimensionless parameter")
    axes[1].legend(); axes[1].grid(True, alpha=0.3)
    figure.suptitle("D5H0 robustness versus payload-contrast shadow gate")
    figure.tight_layout(); figure.savefig(
        output / "figure_D5H0_01_gate_summary.png", dpi=180)
    plt.close(figure)

    report = """# Stage D5H0 — Online shadow diagnostics formal gate

## Scope

New held-out Nominal and Uncertain runs, `{runs}` runs per group.  The sidecar
computed `Y^T sigma - nominal_phase_baseline` online but never wrote it to the
estimator, frozen snapshot, Eq.(10), MPC policy, or plant.

## Result

- Gate D5H0: **{result}**
- Nominal raw scaled terminal norm: `{nraw:.8g} +/- {nrawstd:.3g}`
- Nominal corrected scaled terminal norm: `{ncorr:.8g} +/- {ncorrstd:.3g}`
- Nominal suppression: `{suppression:.2%}`
- Raw payload-contrast scaled RMSE / cosine: `{rrmse:.8g}` / `{rcos:.6g}`
- Corrected payload-contrast scaled RMSE / cosine: `{crmse:.8g}` / `{ccos:.6g}`
- Corrected/raw payload-contrast norm ratio: `{retention:.2%}`

## Gate checks

{checks}

## Boundary

Passing D5H0 means the phase baseline remains promising when evaluated online
on new held-out traces.  It does not prove that applying the corrected update
in closed loop is stable or improves tracking; that is D5H1.
""".format(
        runs=expected_runs, result="PASS" if passed else "FAIL",
        nraw=np.mean(nominal_raw_norm), nrawstd=sample_std(nominal_raw_norm),
        ncorr=np.mean(nominal_corrected_norm), ncorrstd=sample_std(nominal_corrected_norm),
        suppression=raw_suppression, rrmse=raw_rmse, rcos=raw_cosine,
        crmse=corrected_rmse, ccos=corrected_cosine, retention=contrast_retention,
        checks="\n".join("- `{}`: `{}`".format(key, value)
                         for key, value in gates.items()))
    (output / "stage_d5h0_report.md").write_text(report)
    if not passed:
        raise RuntimeError("D5H0 formal shadow gate failed")
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root_directory")
    parser.add_argument("output_directory")
    parser.add_argument("--runs", type=int, default=3)
    args = parser.parse_args()
    result = analyze(args.root_directory, args.output_directory, args.runs)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
