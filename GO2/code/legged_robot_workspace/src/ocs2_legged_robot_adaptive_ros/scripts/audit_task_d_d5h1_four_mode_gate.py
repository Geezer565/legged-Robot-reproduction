#!/usr/bin/env python3
"""Aggregate the four online D5H1 strict Gate smoke results."""

import json
import sys
from pathlib import Path


GROUPS = ("nominal_raw", "nominal_corrected",
          "uncertain_raw", "uncertain_corrected")


def invariants(config):
    keys = (
        "parameterization", "enableParameterUpdate", "enableAdaptiveDynamics",
        "enableClfConstraint", "enableInputBiasCost", "useSoftConstraint",
        "softConstraintMu", "softConstraintDelta", "gamma", "kd",
        "lambdaLinear", "lambdaAngular", "initialParameterEstimate",
        "minimumEstimatedTotalMass", "timestampTolerance", "payload",
        "target", "gait", "fixed_files", "gaitPhaseBins",
        "gaitCycleDuration", "gaitBaselineMinimumCycleStartTime",
        "gaitPhaseBaselineSha256",
        "gaitPhaseBaselineApplicationProvenanceSha256",
        "gaitPhaseBaselineTrainingUsesPayloadData", "scheduler_condition",
        "acquisition_end_mpc_time_s", "d5h1_applied_update_schema",
        "d5h1_eq11_schema",
    )
    return {key: config[key] for key in keys}


def main():
    if len(sys.argv) not in (2, 3):
        raise SystemExit(
            "usage: audit_task_d_d5h1_four_mode_gate.py ROOT [OUTPUT_DIR]")
    root = Path(sys.argv[1]).resolve()
    output = (Path(sys.argv[2]).resolve() if len(sys.argv) == 3
              else root / "gate_analysis")
    output.mkdir(parents=True, exist_ok=True)
    runs = {}
    for group in GROUPS:
        directory = root / group / "run_01"
        runs[group] = {
            "directory": directory,
            "config": json.loads((directory / "effective_config.json").read_text()),
            "smoke": json.loads((directory / "task_d_d5h1_smoke_summary.json").read_text()),
            "gate": json.loads((directory / "d5h1_gate_audit_summary.json").read_text()),
            "source_manifest": (directory / "source_files.sha256").read_text(),
        }
    all_smoke = all(run["smoke"]["passed"] for run in runs.values())
    all_gate = all(run["gate"]["passed"] for run in runs.values())
    common_invariants = len({json.dumps(invariants(run["config"]), sort_keys=True)
                             for run in runs.values()}) == 1
    source_identical = len({run["source_manifest"] for run in runs.values()}) == 1
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
    checks = {
        "all_four_smoke_checks_pass": all_smoke,
        "all_four_strict_gate_checks_pass": all_gate,
        "scientific_invariants_identical": common_invariants,
        "source_manifests_identical": source_identical,
        "only_expected_correction_switches": correction_switches,
        "only_expected_plant_switches": plant_switches,
    }
    passed = all(checks.values())
    rows = []
    for group, run in runs.items():
        eq10 = run["gate"]["closures"]["eq10"]
        eq11 = run["gate"]["closures"]["eq11"]
        rows.append({
            "group": group,
            "samples": min(run["gate"]["sample_counts"].values()),
            "match": run["gate"]["cross_topic_match_fraction"],
            "eq10": max(eq10.values()),
            "eq11": max(eq11.values()),
        })
    summary = {
        "schema_version": 1,
        "stage": "D5H1_four_mode_online_strict_gate",
        "passed": passed,
        "checks": checks,
        "groups": rows,
        "formal_campaign_authorized_by_gate": passed,
        "boundary": (
            "This authorizes repeated acquisition only. It does not pre-judge "
            "nominal drift, payload-signal retention, tracking, or identifiability."),
    }
    (output / "d5h1_four_mode_gate_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    table = "\n".join(
        "| {group} | {samples} | {match:.2%} | {eq10:.3g} | {eq11:.3g} |".format(**row)
        for row in rows)
    report = """# D5H1 four-mode Online Smoke Gate

- Result: **{result}**
- Formal round-robin acquisition authorized: **{authorized}**

| group | synchronized samples | topic match | max Eq.(10) error | max Eq.(11) error |
|---|---:|---:|---:|---:|
{table}

Checks: `{checks}`

Passing this Gate establishes implementation/provenance integrity only. Formal repeated data must still determine nominal-drift reduction, payload-signal retention, and tracking effects.
""".format(result="PASS" if passed else "FAIL",
           authorized="YES" if passed else "NO", table=table,
           checks=json.dumps(checks, sort_keys=True))
    (output / "d5h1_four_mode_gate_report.md").write_text(report)
    print(report)
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
