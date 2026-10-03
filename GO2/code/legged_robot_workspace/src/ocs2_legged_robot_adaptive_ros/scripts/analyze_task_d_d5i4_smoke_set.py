#!/usr/bin/env python3
"""Aggregate the four one-run D5I4 mechanism/safety smokes."""

import hashlib
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


GROUPS = ("nominal_original", "nominal_phase_corrected",
          "payload_original", "payload_phase_corrected")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_task_d_d5i4_smoke_set.py ROOT")
    root = Path(sys.argv[1]).resolve()
    protocol = json.loads((root / "mode_audit" /
                           "frozen_d5i4_protocol.json").read_text())
    missing = [group for group in GROUPS
               if not (root / group / "task_d_d5i4_summary.json").exists()]
    if missing:
        raise SystemExit("missing D5I4 summaries: " + ", ".join(missing))
    runs = {group: json.loads((root / group /
                              "task_d_d5i4_summary.json").read_text())
            for group in GROUPS}
    configs = {group: json.loads((root / group /
                                 "effective_config.json").read_text())
               for group in GROUPS}
    source_hashes = {sha256(root / group / "source_files.sha256")
                     for group in GROUPS}
    baseline_hashes = {config["compositePhaseBaselineSha256"]
                       for config in configs.values()}
    all_run_gates = all(run["passed"] for run in runs.values())
    mechanism_gate = bool(
        all_run_gates and len(source_hashes) == 1 and
        len(baseline_hashes) == 1 and
        json.loads((root / "mode_audit" /
                    "fairness_audit.json").read_text())["passed"])

    scales = np.asarray([5.213485, 0.9697642673581994,
                         0.9697642673581994, 0.9697642673581994] +
                        [0.12] * 6)
    terminal = {group: np.asarray(run["terminal_pi_hat_10d"]) / scales
                for group, run in runs.items()}
    nominal_suppression = 1.0 - (
        np.linalg.norm(terminal["nominal_phase_corrected"]) /
        max(np.linalg.norm(terminal["nominal_original"]), 1e-15))
    payload_corrected_contrast = float(np.linalg.norm(
        terminal["payload_phase_corrected"] -
        terminal["nominal_phase_corrected"]))
    tracking = {group: run["tracking"] for group, run in runs.items()}
    indicative = {
        "nominal_terminal_scaled_norm_original": float(np.linalg.norm(
            terminal["nominal_original"])),
        "nominal_terminal_scaled_norm_phase_corrected": float(np.linalg.norm(
            terminal["nominal_phase_corrected"])),
        "nominal_terminal_suppression_fraction": float(nominal_suppression),
        "payload_corrected_minus_nominal_corrected_terminal_scaled_norm":
            payload_corrected_contrast,
        "tracking": tracking,
    }
    summary = {
        "schema_version": 1,
        "stage": "D5I4_E3_combined_four_mode_online_smoke_set",
        "passed": mechanism_gate,
        "checks": {
            "all_four_run_gates_passed": all_run_gates,
            "source_manifests_identical": len(source_hashes) == 1,
            "frozen_baseline_identical": len(baseline_hashes) == 1,
            "mode_fairness_audit_passed": json.loads(
                (root / "mode_audit" / "fairness_audit.json").read_text())[
                    "passed"],
        },
        "runs": {group: {
            "passed": runs[group]["passed"],
            "lookup": runs[group]["lookup"],
            "tracking": runs[group]["tracking"],
            "terminal_pi_hat_10d": runs[group]["terminal_pi_hat_10d"],
        } for group in GROUPS},
        "single_run_indicative_comparison": indicative,
        "next_stage_decision": (
            "D5I4 mechanism/safety Gate passed. A separate pre-registered "
            ">=3-repeat round-robin campaign may now be designed; these smoke "
            "runs remain scientifically ineligible for final claims."
            if mechanism_gate else
            "D5I4 Gate failed. Preserve all raw runs, diagnose the failed "
            "mechanism/safety checks, and do not begin a formal campaign."),
        "scientific_boundary": protocol["scientific_boundary"],
    }
    analysis = root / "analysis"
    analysis.mkdir(exist_ok=False)
    (analysis / "stage_d5i4_smoke_set_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")

    figure, axes = plt.subplots(2, 1, figsize=(10, 8))
    x = np.arange(10)
    for group in GROUPS:
        axes[0].plot(x, terminal[group], marker="o", label=group)
    axes[0].set_xticks(x, ("m", "hx", "hy", "hz", "Jxx", "Jxy",
                            "Jxz", "Jyy", "Jyz", "Jzz"))
    axes[0].set_ylabel("terminal pi_hat / physical scale")
    position = [tracking[group].get("position_rmse_m", math.nan)
                for group in GROUPS]
    axes[1].bar(np.arange(4), position)
    axes[1].set_xticks(np.arange(4), GROUPS, rotation=15, ha="right")
    axes[1].set_ylabel("position RMSE [m]")
    for axis in axes:
        axis.grid(True, alpha=0.25)
    axes[0].legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(analysis / "figure_D5I4_four_mode_smoke.png", dpi=180)
    plt.close(figure)

    report = """# Stage D5I4 — E3_combined four-mode online smoke

## Gate result

- D5I4 mechanism/safety Gate: **{state}**
- Four per-run strict gates passed: `{runs}`
- Fairness/source/frozen-template identity: `{identity}`

## Single-run diagnostics (not formal evidence)

- Nominal terminal scaled norm, Original: `{raw:.8g}`
- Nominal terminal scaled norm, PhaseCorrected: `{corrected:.8g}`
- Indicative nominal terminal suppression: `{suppression:.3%}`
- PayloadCorrected − NominalCorrected terminal contrast: `{contrast:.8g}`

These are one-run smoke diagnostics only. They verify runtime wiring and expose
obvious failures; they do not establish estimator robustness, retained payload
signal, tracking non-inferiority, tracking benefit, or 10D convergence.

## Decision

{decision}
""".format(
        state="PASS" if mechanism_gate else "FAIL", runs=all_run_gates,
        identity=(len(source_hashes) == 1 and len(baseline_hashes) == 1),
        raw=indicative["nominal_terminal_scaled_norm_original"],
        corrected=indicative[
            "nominal_terminal_scaled_norm_phase_corrected"],
        suppression=nominal_suppression, contrast=payload_corrected_contrast,
        decision=summary["next_stage_decision"])
    (analysis / "stage_d5i4_smoke_set_report.md").write_text(report)
    manifest = {
        "schema_version": 1, "stage": "D5I4_smoke_artifact_manifest",
        "passed": mechanism_gate,
        "artifacts": {
            str(path.relative_to(root)): sha256(path)
            for path in [
                analysis / "stage_d5i4_smoke_set_summary.json",
                analysis / "stage_d5i4_smoke_set_report.md",
                analysis / "figure_D5I4_four_mode_smoke.png"]},
        "raw_bags": {group: sha256(root / group / "task_d_d5i4.bag")
                     for group in GROUPS},
    }
    (root / "stage_d5i4_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(report)
    if not mechanism_gate:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
