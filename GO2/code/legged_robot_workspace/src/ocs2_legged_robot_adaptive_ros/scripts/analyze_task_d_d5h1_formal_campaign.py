#!/usr/bin/env python3
"""Unified repeated four-mode analysis for the frozen D5H1 campaign."""

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


GROUPS = ("nominal_raw", "nominal_corrected",
          "uncertain_raw", "uncertain_corrected")
RUNS = ("run_01", "run_02", "run_03")
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz",
              "Jyy", "Jyz", "Jzz")
LABELS = {
    "nominal_raw": "Nominal Original",
    "nominal_corrected": "Nominal PhaseCorrected",
    "uncertain_raw": "Payload Original",
    "uncertain_corrected": "Payload PhaseCorrected",
}
COLORS = {
    "nominal_raw": "#4c4c4c", "nominal_corrected": "#2ca02c",
    "uncertain_raw": "#d62728", "uncertain_corrected": "#1f77b4",
}


def load_csv(path):
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("empty formal CSV: " + str(path))
    data = {name: np.asarray([float(row[name]) for row in rows])
            for name in rows[0]}
    if any(not np.all(np.isfinite(value)) for value in data.values()):
        raise RuntimeError("non-finite formal CSV: " + str(path))
    return data


def rms(values, time):
    values = np.asarray(values, dtype=float)
    squared = values ** 2 if values.ndim == 1 else np.sum(values ** 2, axis=1)
    return float(np.sqrt(np.trapz(squared, time) / (time[-1] - time[0])))


def stats(values):
    values = np.asarray(values, dtype=float)
    return {
        "n": len(values), "mean": float(np.mean(values)),
        "sample_std": float(np.std(values, ddof=1)),
        "min": float(np.min(values)), "max": float(np.max(values)),
    }


def scaled_norm(vector, scales):
    return float(np.linalg.norm(np.asarray(vector) / scales))


def scaled_rmse(vector, truth, scales):
    return float(np.sqrt(np.mean(((np.asarray(vector) - truth) / scales) ** 2)))


def scaled_cosine(vector, truth, scales):
    left, right = np.asarray(vector) / scales, np.asarray(truth) / scales
    denominator = np.linalg.norm(left) * np.linalg.norm(right)
    return float(np.dot(left, right) / denominator) if denominator > 0.0 else None


def matrix(data, prefix, count):
    return np.column_stack([data[prefix + str(index)] for index in range(count)])


def invariants(config):
    keys = (
        "parameterization", "enableParameterUpdate", "enableAdaptiveDynamics",
        "enableClfConstraint", "enableInputBiasCost", "useSoftConstraint",
        "softConstraintMu", "softConstraintDelta", "gamma", "kd",
        "lambdaLinear", "lambdaAngular", "initialParameterEstimate",
        "minimumEstimatedTotalMass", "timestampTolerance", "payload", "target",
        "gait", "fixed_files", "gaitPhaseBins", "gaitCycleDuration",
        "gaitBaselineMinimumCycleStartTime", "gaitPhaseBaselineSha256",
        "gaitPhaseBaselineApplicationProvenanceSha256",
        "gaitPhaseBaselineTrainingUsesPayloadData", "scheduler_condition",
        "analysis_window_start_mpc_time_s", "analysis_window_end_mpc_time_s",
        "common_grid_dt_s", "d5h1_applied_update_schema", "d5h1_eq11_schema",
        "formalProtocolSha256", "diagnosticTopicStartToleranceS",
        "topicEndToleranceS",
    )
    return {key: config[key] for key in keys}


def write_rows(path, rows):
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_group_signal(path, grid, runs, key, ylabel):
    figure, axis = plt.subplots(figsize=(11, 4.8))
    for group in GROUPS:
        values = np.vstack([run["data"][key] for run in runs[group]])
        mean = np.mean(values, axis=0)
        std = np.std(values, axis=0, ddof=1)
        axis.plot(grid, mean, color=COLORS[group], lw=1.8, label=LABELS[group])
        axis.fill_between(grid, mean - std, mean + std,
                          color=COLORS[group], alpha=0.13)
    axis.set_xlim(grid[0], grid[-1])
    axis.set_xlabel("MPC time [s]")
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--output-directory", type=Path, required=True)
    args = parser.parse_args()
    root, output = args.root.resolve(), args.output_directory.resolve()
    output.mkdir(parents=True, exist_ok=True)
    protocol = json.loads((root / "protocol" / "stage_d5h1_formal_protocol.json").read_text())
    if not protocol.get("passed"):
        raise RuntimeError("frozen D5H1 formal protocol did not pass")
    window = protocol["analysis_window"]
    grid = np.linspace(window["start_mpc_time_s"], window["end_mpc_time_s"],
                       int(round((window["end_mpc_time_s"] -
                                  window["start_mpc_time_s"]) /
                                 window["common_grid_dt_s"])) + 1)

    grouped = {group: [] for group in GROUPS}
    all_runs, manifests, invariant_values = [], [], []
    for group in GROUPS:
        for run_name in RUNS:
            directory = root / group / run_name
            summary = json.loads((directory /
                                  "task_d_d5h1_formal_run_summary.json").read_text())
            gate = json.loads((directory / "d5h1_gate_audit_summary.json").read_text())
            config = json.loads((directory / "effective_config.json").read_text())
            data = load_csv(directory / "task_d_d5h1_formal_run.csv")
            if (len(data["time_s"]) != len(grid)
                    or not np.allclose(data["time_s"], grid, rtol=0.0, atol=1.0e-12)):
                raise RuntimeError("common grid mismatch: {}/{}".format(group, run_name))
            run = {"group": group, "run": run_name, "directory": directory,
                   "summary": summary, "gate": gate, "config": config, "data": data}
            grouped[group].append(run)
            all_runs.append(run)
            manifests.append((directory / "source_files.sha256").read_text())
            invariant_values.append(json.dumps(invariants(config), sort_keys=True))

    run_validity = all(run["summary"].get("pass")
                       and run["summary"].get("formal_eligibility")
                       and run["gate"].get("passed") for run in all_runs)
    source_manifests_identical = len(set(manifests)) == 1
    scientific_invariants_identical = len(set(invariant_values)) == 1
    protocol_sha256 = hashlib.sha256(
        (root / "protocol" / "stage_d5h1_formal_protocol.json").read_bytes()).hexdigest()
    protocol_hashes_match = all(
        run["config"]["formalProtocolSha256"] == protocol_sha256
        for run in all_runs)
    mode_flags_ok = all(
        run["config"]["enableGaitPhaseBaselineCorrection"] ==
            ("corrected" in run["group"])
        and run["config"]["truePlantPayloadInertialEnabled"] ==
            run["group"].startswith("uncertain")
        and run["config"]["run_index"] == int(run["run"][-2:])
        for run in all_runs)
    expected_sequences = {}
    sequence = 0
    for round_index, order in enumerate(protocol["round_robin_order"], start=1):
        for group in order:
            sequence += 1
            expected_sequences[(group, round_index)] = sequence
    acquisition_order_ok = all(
        run["config"]["acquisition_sequence"] ==
            expected_sequences[(run["group"], int(run["run"][-2:]))]
        for run in all_runs)

    metric_names = sorted(set.intersection(*[
        set(run["summary"]["metrics"]) for run in all_runs]))
    group_stats = {
        group: {metric: stats([run["summary"]["metrics"][metric]
                               for run in grouped[group]])
                for metric in metric_names}
        for group in GROUPS}
    per_run_rows = []
    for run in all_runs:
        row = {"group": run["group"], "run": run["run"]}
        row.update(run["summary"]["metrics"])
        per_run_rows.append(row)
    write_rows(output / "d5h1_formal_per_run_metrics.csv", per_run_rows)
    aggregate_rows = [
        {"group": group, "metric": metric, **group_stats[group][metric]}
        for group in GROUPS for metric in metric_names]
    write_rows(output / "d5h1_formal_group_mean_std.csv", aggregate_rows)

    truth = np.asarray(grouped["uncertain_corrected"][0]["summary"]
                       ["plant_pi_star_10d"], dtype=float)
    scales = np.asarray(grouped["uncertain_corrected"][0]["summary"]
                        ["physical_coordinate_scales"], dtype=float)
    terminal_vectors = {
        group: [np.asarray(run["summary"]["last2s_mean_pi_hat_10d"])
                for run in grouped[group]] for group in GROUPS}
    raw_contrasts, corrected_contrasts, contrast_rows = [], [], []
    for index, run_name in enumerate(RUNS):
        raw = terminal_vectors["uncertain_raw"][index] - \
            terminal_vectors["nominal_raw"][index]
        corrected = terminal_vectors["uncertain_corrected"][index] - \
            terminal_vectors["nominal_corrected"][index]
        raw_contrasts.append(raw)
        corrected_contrasts.append(corrected)
        for method, vector in (("Original", raw), ("PhaseCorrected", corrected)):
            contrast_rows.append({
                "run": run_name, "method": method,
                "scaled_norm": scaled_norm(vector, scales),
                "scaled_rmse_vs_pi_star": scaled_rmse(vector, truth, scales),
                "scaled_cosine_vs_pi_star": scaled_cosine(vector, truth, scales),
            })
    write_rows(output / "d5h1_payload_contrast_per_run.csv", contrast_rows)
    raw_mean = np.mean(raw_contrasts, axis=0)
    corrected_mean = np.mean(corrected_contrasts, axis=0)
    raw_std = np.std(raw_contrasts, axis=0, ddof=1)
    corrected_std = np.std(corrected_contrasts, axis=0, ddof=1)
    component_rows = [{
        "parameter": name, "scale": scales[index], "pi_star": truth[index],
        "original_U_minus_N_mean": raw_mean[index],
        "original_U_minus_N_sample_std": raw_std[index],
        "phase_corrected_U_minus_N_mean": corrected_mean[index],
        "phase_corrected_U_minus_N_sample_std": corrected_std[index],
    } for index, name in enumerate(PARAMETERS)]
    write_rows(output / "d5h1_payload_contrast_components.csv", component_rows)

    decision = protocol["frozen_engineering_decision_rules"]
    nominal_raw = [scaled_norm(value, scales)
                   for value in terminal_vectors["nominal_raw"]]
    nominal_corrected = [scaled_norm(value, scales)
                         for value in terminal_vectors["nominal_corrected"]]
    nominal_suppression = 1.0 - np.mean(nominal_corrected) / np.mean(nominal_raw)
    nominal_every_pair_lower = all(c < r for r, c in
                                   zip(nominal_raw, nominal_corrected))
    raw_contrast_norm = scaled_norm(raw_mean, scales)
    corrected_contrast_norm = scaled_norm(corrected_mean, scales)
    retention = corrected_contrast_norm / raw_contrast_norm
    raw_rmse = scaled_rmse(raw_mean, truth, scales)
    corrected_rmse = scaled_rmse(corrected_mean, truth, scales)
    rmse_relative_increase = corrected_rmse / raw_rmse - 1.0
    raw_cosine = scaled_cosine(raw_mean, truth, scales)
    corrected_cosine = scaled_cosine(corrected_mean, truth, scales)
    cosine_drop = raw_cosine - corrected_cosine

    rel = decision["tracking_noninferiority_relative_margin"]
    absolute = decision["tracking_noninferiority_absolute_margin_m"]
    tracking_results = {}
    for plant, raw_group, corrected_group in (
            ("nominal", "nominal_raw", "nominal_corrected"),
            ("payload", "uncertain_raw", "uncertain_corrected")):
        tracking_results[plant] = {}
        for metric in decision["tracking_primary_metrics"]:
            raw_value = group_stats[raw_group][metric]["mean"]
            corrected_value = group_stats[corrected_group][metric]["mean"]
            limit = raw_value * (1.0 + rel) + absolute
            tracking_results[plant][metric] = {
                "original_mean": raw_value,
                "phase_corrected_mean": corrected_value,
                "noninferiority_limit": limit,
                "passed": corrected_value <= limit,
            }

    policy_rows = []
    for plant, raw_group, corrected_group in (
            ("nominal", "nominal_raw", "nominal_corrected"),
            ("payload", "uncertain_raw", "uncertain_corrected")):
        for index, run_name in enumerate(RUNS):
            raw_policy = matrix(grouped[raw_group][index]["data"], "policy_", 24)
            corrected_policy = matrix(grouped[corrected_group][index]["data"],
                                      "policy_", 24)
            delta = corrected_policy - raw_policy
            policy_rows.append({
                "plant": plant, "run": run_name,
                "full_policy_rms_vector_difference": rms(delta, grid),
                "contact_force_rms_vector_difference_N": rms(delta[:, :12], grid),
            })
    write_rows(output / "d5h1_policy_differences.csv", policy_rows)

    checks = {
        "all_12_runs_formally_eligible_and_strict_gate_pass": run_validity,
        "source_manifests_identical": source_manifests_identical,
        "scientific_invariants_identical": scientific_invariants_identical,
        "all_run_protocol_hashes_match_frozen_protocol": protocol_hashes_match,
        "only_expected_mode_flags": mode_flags_ok,
        "pre_registered_round_robin_order_observed": acquisition_order_ok,
        "nominal_drift_mean_suppression_at_least_50_percent":
            bool(nominal_suppression >= decision[
                "nominal_drift_minimum_mean_suppression_fraction"]),
        "nominal_corrected_lower_in_every_paired_run": nominal_every_pair_lower,
        "payload_contrast_norm_retention_at_least_80_percent":
            bool(retention >= decision[
                "payload_contrast_minimum_norm_retention_fraction"]),
        "payload_contrast_cosine_drop_within_limit":
            bool(cosine_drop <= decision["payload_cosine_maximum_allowed_drop"]),
        "payload_contrast_rmse_relative_increase_within_limit":
            bool(rmse_relative_increase <= decision[
                "payload_scaled_rmse_maximum_relative_increase"]),
        "tracking_noninferiority_all_primary_metrics": all(
            result["passed"] for plant in tracking_results.values()
            for result in plant.values()),
    }
    passed = all(checks.values())
    summary = {
        "schema_version": 1, "stage": "D5H1_formal_four_mode_campaign",
        "passed": passed, "checks": checks,
        "run_count": len(all_runs), "group_statistics": group_stats,
        "nominal_false_estimate": {
            "original_per_run_scaled_norm": nominal_raw,
            "phase_corrected_per_run_scaled_norm": nominal_corrected,
            "mean_suppression_fraction": float(nominal_suppression),
            "every_paired_run_lower": nominal_every_pair_lower,
        },
        "payload_signal": {
            "original_mean_U_minus_N_10d": raw_mean.tolist(),
            "phase_corrected_mean_U_minus_N_10d": corrected_mean.tolist(),
            "pi_star_10d": truth.tolist(),
            "norm_retention_fraction": retention,
            "original_scaled_rmse": raw_rmse,
            "phase_corrected_scaled_rmse": corrected_rmse,
            "rmse_relative_increase": rmse_relative_increase,
            "original_scaled_cosine": raw_cosine,
            "phase_corrected_scaled_cosine": corrected_cosine,
            "cosine_drop": cosine_drop,
        },
        "tracking_noninferiority": tracking_results,
        "policy_differences": policy_rows,
        "analysis_provenance": {
            "acquisition_source_manifest_sha256": hashlib.sha256(
                manifests[0].encode()).hexdigest(),
            "offline_analysis_script": str(Path(__file__).resolve()),
            "offline_analysis_script_sha256": hashlib.sha256(
                Path(__file__).read_bytes()).hexdigest(),
            "frozen_protocol_sha256": protocol_sha256,
            "note": (
                "Acquisition source manifests are identical across all 12 runs. "
                "The offline analyzer hash is recorded separately because a "
                "NumPy-bool JSON serialization defect was fixed after acquisition; "
                "no metric, threshold, controller, estimator, or raw bag changed."),
        },
        "next_stage_decision": (
            "Proceed to a separately frozen richer-excitation/10D "
            "identifiability design only after interpreting these results."
            if passed else
            "Do not enter richer excitation yet; diagnose the failed frozen "
            "D5H1 decision rule without post-hoc threshold or Gamma changes."),
        "scientific_boundary": protocol["scientific_boundary"],
    }
    (output / "stage_d5h1_formal_campaign_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")

    for filename, key, ylabel in (
            ("figure_D5H1_01_x_error.png", "x_error", "x error [m]"),
            ("figure_D5H1_02_position_error.png", "position_error_norm",
             "position error norm [m]"),
            ("figure_D5H1_03_pi_scaled_norm.png", "pi_hat_scaled_norm",
             "physical-scaled ||pi_hat||"),
            ("figure_D5H1_04_sigma_norm.png", "sigma_norm", "||sigma||"),
            ("figure_D5H1_05_eq10_applied.png", "eq10_applied_norm",
             "Eq.(10) applied correction norm"),
            ("figure_D5H1_06_eq11_violation.png", "eq11_violation",
             "Eq.(11) violation")):
        plot_group_signal(output / filename, grid, grouped, key, ylabel)

    x = np.arange(len(PARAMETERS))
    figure, axes = plt.subplots(2, 1, figsize=(12, 8), constrained_layout=True)
    nraw_mean = np.mean(terminal_vectors["nominal_raw"], axis=0) / scales
    ncorr_mean = np.mean(terminal_vectors["nominal_corrected"], axis=0) / scales
    axes[0].plot(x, nraw_mean, "o-", label="Nominal Original")
    axes[0].plot(x, ncorr_mean, "o-", label="Nominal PhaseCorrected")
    axes[0].axhline(0.0, color="black", lw=0.8)
    axes[0].set_ylabel("last-2s mean pi_hat / scale")
    axes[0].grid(alpha=0.3); axes[0].legend()
    axes[1].errorbar(x, raw_mean / scales, yerr=raw_std / scales,
                     fmt="o-", capsize=3, label="Original U-N")
    axes[1].errorbar(x, corrected_mean / scales, yerr=corrected_std / scales,
                     fmt="o-", capsize=3, label="PhaseCorrected U-N")
    axes[1].plot(x, truth / scales, "k--", label="pi_star")
    axes[1].set_xticks(x, PARAMETERS)
    axes[1].set_ylabel("payload contrast / scale")
    axes[1].grid(alpha=0.3); axes[1].legend()
    figure.savefig(output / "figure_D5H1_07_nominal_and_payload_contrast.png", dpi=180)
    plt.close(figure)

    report = """# Stage D5H1 formal four-mode campaign

- Frozen engineering Gate: **{result}**
- Valid formal runs: **{valid}/12**
- Source manifests identical: `{source}`
- Scientific invariants identical: `{fairness}`

## Nominal false-estimate suppression

- Original last-2s scaled norm, mean +/- sample std: `{nr:.8g} +/- {nrs:.8g}`
- PhaseCorrected: `{nc:.8g} +/- {ncs:.8g}`
- Mean suppression: **{supp:.2%}**; corrected lower in every paired run: `{every}`

## Payload U-N signal

- Norm retention: **{ret:.2%}**
- Scaled RMSE vs pi_star: `{rr:.8g}` -> `{cr:.8g}`
- Scaled cosine vs pi_star: `{rc:.8g}` -> `{cc:.8g}`

## Tracking noninferiority

`{tracking}`

## Decision

{decision_text}

This is evidence for the fixed animal_dummy/payload/0.1 m walking condition.
It is not a proof that the 10 inertial parameters are individually identifiable
or that pi_hat has converged to the physical payload parameters.
""".format(
        result="PASS" if passed else "FAIL",
        valid=sum(run["summary"].get("pass", False) for run in all_runs),
        source=source_manifests_identical, fairness=scientific_invariants_identical,
        nr=np.mean(nominal_raw), nrs=np.std(nominal_raw, ddof=1),
        nc=np.mean(nominal_corrected), ncs=np.std(nominal_corrected, ddof=1),
        supp=nominal_suppression, every=nominal_every_pair_lower,
        ret=retention, rr=raw_rmse, cr=corrected_rmse,
        rc=raw_cosine, cc=corrected_cosine,
        tracking=json.dumps(tracking_results, sort_keys=True),
        decision_text=summary["next_stage_decision"])
    (output / "stage_d5h1_formal_campaign_report.md").write_text(report)
    artifacts = sorted(path for path in output.iterdir() if path.is_file())
    checksums = "".join("{}  {}\n".format(
        hashlib.sha256(path.read_bytes()).hexdigest(), path.name)
        for path in artifacts)
    (output / "analysis_artifact_checksums.sha256").write_text(checksums)
    print(report)
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
