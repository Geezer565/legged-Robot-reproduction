#!/usr/bin/env python3
"""Train and held-out validate the frozen nominal-only E3 D5I3 baseline."""

import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ESTIMATE = "/task_d/payload_inertial_10d_v3_sample"
SCHEDULER = "/task_d/d5i2_scheduler_sample"
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy",
              "Jyz", "Jzz")
EXPECTED_MODES = (13, 7, 14, 11)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def deduplicate(rows):
    return np.asarray([row for _, row in sorted(
        {float(row[0]): row for row in rows}.items())], dtype=float)


def mode_transitions(scheduler):
    transitions, previous = [], None
    for row in scheduler:
        mode = int(round(row[27]))
        if mode != previous:
            transitions.append((float(row[0]), mode))
            previous = mode
    return transitions


def complete_cycles(scheduler, steady_start, steady_end):
    """Find valid executed cycles that overlap the steady reference window."""
    transitions = mode_transitions(scheduler)
    starts = [index for index, (_, mode) in enumerate(transitions)
              if mode == EXPECTED_MODES[0]]
    cycles = []
    for left, right in zip(starts[:-1], starts[1:]):
        sequence = tuple(mode for _, mode in transitions[left:right])
        start, end = transitions[left][0], transitions[right][0]
        if (sequence == EXPECTED_MODES and
                end > steady_start + 1.0e-8 and
                start < steady_end - 1.0e-8):
            cycles.append({"start": start, "end": end,
                           "duration": end - start,
                           "sequence": list(sequence),
                           "fully_contained": bool(
                               start >= steady_start - 1.0e-8 and
                               end <= steady_end + 1.0e-8)})
    return cycles


def nearest_rows(source, target_time, tolerance=1.0e-8):
    index = np.searchsorted(source[:, 0], target_time, side="left")
    index = np.clip(index, 0, len(source) - 1)
    left = np.clip(index - 1, 0, len(source) - 1)
    choose_left = np.abs(target_time - source[left, 0]) <= np.abs(
        source[index, 0] - target_time)
    index = np.where(choose_left, left, index)
    error = np.abs(source[index, 0] - target_time)
    if np.max(error) > tolerance:
        raise ValueError("D5I3 scheduler/estimate time match failed")
    return source[index]


def extract_steady_samples(estimates, scheduler, schedule, profile_bins,
                           gait_bins):
    """Return exact explicit-Euler samples from complete steady gait cycles."""
    steady_start = (schedule["profile_start_time_s"] +
                    schedule["ramp_in_duration_s"])
    steady_end = steady_start + schedule["steady_duration_s"]
    period = schedule["supercycle_period_s"]
    cycles = complete_cycles(scheduler, steady_start, steady_end)
    values = []
    for cycle_index, cycle in enumerate(cycles):
        interval_start = max(cycle["start"], steady_start)
        interval_end = min(cycle["end"], steady_end)
        mask = ((estimates[:, 0] > interval_start + 1.0e-10) &
                (estimates[:, 0] <= interval_end + 1.0e-10))
        rows = estimates[mask]
        if len(rows) < 2:
            raise ValueError("D5I3 gait interval has too few samples")
        scheduler_rows = nearest_rows(scheduler, rows[:, 0])
        # The stage transition sample at steady_end is RampOut and therefore
        # intentionally has zero baseline. Keep only samples whose current
        # causal scheduler stage is Steady.
        steady = np.rint(scheduler_rows[:, 6]).astype(int) == 2
        rows, scheduler_rows = rows[steady], scheduler_rows[steady]
        if len(rows) < 2:
            raise ValueError("D5I3 steady gait interval has too few samples")
        previous = np.r_[interval_start, rows[:-1, 0]]
        dt = rows[:, 0] - previous
        if np.any(dt <= 0.0) or rows[-1, 0] > interval_end + 1.0e-8:
            raise ValueError("D5I3 explicit-Euler intervals are invalid")
        gait_phase = np.minimum(
            (rows[:, 0] - cycle["start"]) / cycle["duration"],
            np.nextafter(1.0, 0.0))
        profile_phase = np.mod(scheduler_rows[:, 2], 1.0)
        profile_bin = np.minimum(
            (profile_phase * profile_bins).astype(int), profile_bins - 1)
        gait_bin = np.minimum(
            (gait_phase * gait_bins).astype(int), gait_bins - 1)
        mode = np.rint(scheduler_rows[:, 27]).astype(int)
        stage = np.rint(scheduler_rows[:, 6]).astype(int)
        if not np.all(stage == 2):
            raise ValueError("D5I3 selected a non-steady baseline sample")
        regressor = rows[:, 48:108].reshape(-1, 6, 10)
        sigma = rows[:, 42:48]
        drive = np.einsum("nij,ni->nj", regressor, sigma)
        supercycle = np.floor(
            (rows[:, 0] - steady_start) / period + 1.0e-10).astype(int)
        supercycle = np.clip(supercycle, 0,
                             int(round(schedule["steady_duration_s"] / period)) - 1)
        for index in range(len(rows)):
            values.append({
                "time": float(rows[index, 0]),
                "dt": float(dt[index]),
                "drive": drive[index],
                "sigma": sigma[index],
                "profile_phase": float(profile_phase[index]),
                "profile_bin": int(profile_bin[index]),
                "gait_phase": float(gait_phase[index]),
                "gait_bin": int(gait_bin[index]),
                "mode": int(mode[index]),
                "cycle_index": cycle_index,
                "supercycle_index": int(supercycle[index]),
            })
    if not values:
        raise ValueError("D5I3 found no complete steady samples")
    return values, cycles


def build_template(runs):
    """Duration-weighted mean drive using calibration samples only."""
    numerator, duration, count = {}, {}, {}
    for run in runs:
        if run.get("role") != "calibration":
            raise ValueError("heldout data attempted to enter D5I3 template")
        for sample in run["samples"]:
            key = (sample["profile_bin"], sample["gait_bin"], sample["mode"])
            numerator[key] = numerator.get(key, np.zeros(10)) + \
                sample["dt"] * sample["drive"]
            duration[key] = duration.get(key, 0.0) + sample["dt"]
            count[key] = count.get(key, 0) + 1
    if not numerator or any(value <= 0.0 for value in duration.values()):
        raise ValueError("empty or invalid D5I3 template")
    values = {key: numerator[key] / duration[key] for key in numerator}
    return {"values": values, "duration": duration, "count": count}


def evaluate_template(run, template, scales):
    raw_terminal, corrected_terminal = np.zeros(10), np.zeros(10)
    raw_energy = corrected_energy = 0.0
    covered_duration = total_duration = 0.0
    rows = []
    per_supercycle = {}
    for sample in run["samples"]:
        key = (sample["profile_bin"], sample["gait_bin"], sample["mode"])
        supported = key in template["values"]
        baseline = template["values"].get(key, np.zeros(10))
        corrected = sample["drive"] - baseline
        dt = sample["dt"]
        raw_terminal += dt * sample["drive"]
        corrected_terminal += dt * corrected
        raw_energy += dt * np.sum(np.square(sample["drive"] / scales))
        corrected_energy += dt * np.sum(np.square(corrected / scales))
        total_duration += dt
        if supported:
            covered_duration += dt
        slot = per_supercycle.setdefault(
            sample["supercycle_index"],
            {"raw": np.zeros(10), "corrected": np.zeros(10), "duration": 0.0})
        slot["raw"] += dt * sample["drive"]
        slot["corrected"] += dt * corrected
        slot["duration"] += dt
        rows.append({**sample, "baseline": baseline, "corrected": corrected,
                     "supported": supported})
    scaled_raw = float(np.linalg.norm(raw_terminal / scales))
    scaled_corrected = float(np.linalg.norm(corrected_terminal / scales))
    return {
        "rows": rows,
        "raw_terminal": raw_terminal,
        "corrected_terminal": corrected_terminal,
        "raw_terminal_scaled_norm": scaled_raw,
        "corrected_terminal_scaled_norm": scaled_corrected,
        "terminal_suppression_fraction": (
            1.0 - scaled_corrected / scaled_raw if scaled_raw > 0.0 else None),
        "lookup_duration_coverage": covered_duration / total_duration,
        "evaluated_duration_s": total_duration,
        "raw_drive_scaled_rms": math.sqrt(raw_energy / total_duration),
        "corrected_drive_scaled_rms": math.sqrt(
            corrected_energy / total_duration),
        "per_supercycle": {
            str(index): {
                "duration_s": values["duration"],
                "raw_terminal_scaled_norm": float(np.linalg.norm(
                    values["raw"] / scales)),
                "corrected_terminal_scaled_norm": float(np.linalg.norm(
                    values["corrected"] / scales)),
            } for index, values in sorted(per_supercycle.items())
        },
    }


def load_run(path, role, run_index, protocol):
    import rosbag
    config = json.loads((path / "effective_config.json").read_text())
    summary = json.loads((path / "task_d_d5i3_summary.json").read_text())
    if (config.get("calibration_role") != role or
            config.get("run_index") != run_index or
            config.get("candidate") != "E3_combined"):
        raise ValueError("D5I3 run identity mismatch: " + str(path))
    if config.get("frozen_protocol_sha256") != sha256(
            path / "frozen_d5i3_protocol.json"):
        raise ValueError("D5I3 per-run protocol hash mismatch: " + str(path))
    if not summary.get("acquisition_valid") or not summary.get(
            "candidate_admitted_for_next_stage"):
        raise ValueError("D5I3 invalid per-run acquisition: " + str(path))
    estimates, schedulers, labels = [], [], {ESTIMATE: set(), SCHEDULER: set()}
    bag_path = path / "task_d_d5i3.bag"
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _ in bag.read_messages(topics=[ESTIMATE, SCHEDULER]):
            labels[topic].add(message.layout.dim[0].label
                              if message.layout.dim else "")
            ({ESTIMATE: estimates, SCHEDULER: schedulers}[topic]).append(
                [float(value) for value in message.data])
    estimates, schedulers = deduplicate(estimates), deduplicate(schedulers)
    if (estimates.shape[1:] != (118,) or schedulers.shape[1:] != (28,) or
            labels[ESTIMATE] != {"payload_inertial_10d_v3_118_fields"} or
            labels[SCHEDULER] != {"d5i2_scheduler_v1_28_fields"} or
            not np.all(np.isfinite(estimates)) or
            not np.all(np.isfinite(schedulers))):
        raise ValueError("D5I3 bag schema/finite check failed: " + str(path))
    baseline = protocol["baseline"]
    samples, cycles = extract_steady_samples(
        estimates, schedulers, protocol["schedule"],
        baseline["profile_phase_bins"], baseline["gait_phase_bins"])
    return {
        "path": path,
        "role": role,
        "run_index": run_index,
        "config": config,
        "summary": summary,
        "bag": bag_path,
        "bag_sha256": sha256(bag_path),
        "source_manifest_sha256": sha256(path / "source_files.sha256"),
        "samples": samples,
        "cycles": cycles,
    }


def write_template_csv(path, template, profile_bins, gait_bins):
    columns = [
        "profile_phase_bin", "profile_phase_start", "profile_phase_end",
        "gait_phase_bin", "gait_phase_start", "gait_phase_end",
        "executed_mode", "training_duration_s", "training_sample_count",
        "parameter", "nominal_baseline_drive_rate"]
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for key in sorted(template["values"]):
            profile, gait, mode = key
            for parameter, value in zip(PARAMETERS, template["values"][key]):
                writer.writerow({
                    "profile_phase_bin": profile,
                    "profile_phase_start": profile / profile_bins,
                    "profile_phase_end": (profile + 1) / profile_bins,
                    "gait_phase_bin": gait,
                    "gait_phase_start": gait / gait_bins,
                    "gait_phase_end": (gait + 1) / gait_bins,
                    "executed_mode": mode,
                    "training_duration_s": template["duration"][key],
                    "training_sample_count": template["count"][key],
                    "parameter": parameter,
                    "nominal_baseline_drive_rate": value,
                })


def write_rows(path, rows):
    if not rows:
        return
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def serializable_evaluation(value):
    return {
        "raw_terminal_10d": value["raw_terminal"].tolist(),
        "corrected_terminal_10d": value["corrected_terminal"].tolist(),
        "raw_terminal_scaled_norm": value["raw_terminal_scaled_norm"],
        "corrected_terminal_scaled_norm":
            value["corrected_terminal_scaled_norm"],
        "terminal_suppression_fraction":
            value["terminal_suppression_fraction"],
        "lookup_duration_coverage": value["lookup_duration_coverage"],
        "evaluated_duration_s": value["evaluated_duration_s"],
        "raw_drive_scaled_rms": value["raw_drive_scaled_rms"],
        "corrected_drive_scaled_rms": value["corrected_drive_scaled_rms"],
        "per_supercycle": value["per_supercycle"],
    }


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_task_d_d5i3_campaign.py CAMPAIGN_ROOT")
    root = Path(sys.argv[1]).resolve()
    output = root / "analysis"
    output.mkdir(parents=True, exist_ok=False)
    protocol_path = root / "stage_d5i3_protocol.json"
    protocol = json.loads(protocol_path.read_text())
    scales = np.asarray(protocol["physical_coordinate_scales"], dtype=float)
    runs = {role: [load_run(root / role / "run_{:02d}".format(index),
                            role, index, protocol)
                   for index in (1, 2, 3)]
            for role in ("calibration", "heldout")}

    template = build_template(runs["calibration"])
    baseline_csv = output / "nominal_e3_composite_phase_baseline_v1.csv"
    write_template_csv(
        baseline_csv, template, protocol["baseline"]["profile_phase_bins"],
        protocol["baseline"]["gait_phase_bins"])
    heldout = [evaluate_template(run, template, scales)
               for run in runs["heldout"]]
    calibration_loo = []
    for index, run in enumerate(runs["calibration"]):
        loo_template = build_template([
            value for other, value in enumerate(runs["calibration"])
            if other != index])
        calibration_loo.append(evaluate_template(run, loo_template, scales))

    evaluation_rows = []
    for role, evaluations in (("calibration_loo", calibration_loo),
                              ("heldout", heldout)):
        for index, value in enumerate(evaluations, 1):
            evaluation_rows.append({
                "role": role, "run": "run_{:02d}".format(index),
                "evaluated_duration_s": value["evaluated_duration_s"],
                "lookup_duration_coverage": value["lookup_duration_coverage"],
                "raw_terminal_scaled_norm":
                    value["raw_terminal_scaled_norm"],
                "corrected_terminal_scaled_norm":
                    value["corrected_terminal_scaled_norm"],
                "terminal_suppression_fraction":
                    value["terminal_suppression_fraction"],
                "raw_drive_scaled_rms": value["raw_drive_scaled_rms"],
                "corrected_drive_scaled_rms":
                    value["corrected_drive_scaled_rms"],
            })
    write_rows(output / "d5i3_baseline_validation_per_run.csv", evaluation_rows)

    raw_norms = np.asarray([value["raw_terminal_scaled_norm"]
                            for value in heldout])
    corrected_norms = np.asarray([value["corrected_terminal_scaled_norm"]
                                  for value in heldout])
    suppression = 1.0 - float(np.mean(corrected_norms) / np.mean(raw_norms))
    coverage = [value["lookup_duration_coverage"] for value in heldout]
    gates = protocol["frozen_engineering_gates"]
    source_hashes = {run["source_manifest_sha256"]
                     for group in runs.values() for run in group}
    invariant_keys = ("adaptive_config_sha256", "fixed_files", "candidate",
                      "schedule", "target", "baseline_key",
                      "frozen_protocol_sha256")
    invariants = all(
        len({json.dumps(run["config"][key], sort_keys=True)
             for group in runs.values() for run in group}) == 1
        for key in invariant_keys)
    expected_order = protocol["data_split"]["pre_registered_order"]
    observed_order = (root / "acquisition_order.log").read_text().splitlines()
    all_runs = [run for group in runs.values() for run in group]
    cycle_counts = [sum(cycle["fully_contained"] for cycle in run["cycles"])
                    for run in all_runs]
    overlap_cycle_counts = [len(run["cycles"]) for run in all_runs]
    steady_durations = [sum(sample["dt"] for sample in run["samples"])
                        for run in all_runs]
    supercycle_durations = []
    for run in all_runs:
        values = {}
        for sample in run["samples"]:
            values[sample["supercycle_index"]] = values.get(
                sample["supercycle_index"], 0.0) + sample["dt"]
        supercycle_durations.append(values)
    required_supercycles = gates["minimum_complete_supercycles_per_run"]
    period = protocol["schedule"]["supercycle_period_s"]
    # One 50 Hz endpoint at steady_end is causally RampOut and must receive
    # zero baseline, hence the explicit one-sample tolerance below.
    supercycle_coverage_ok = all(
        len(values) >= required_supercycles and
        all(values.get(index, 0.0) >= period - 0.05
            for index in range(required_supercycles))
        for values in supercycle_durations)
    checks = {
        "all_six_runs_valid": True,
        "source_manifests_identical": len(source_hashes) == 1,
        "scientific_invariants_identical": invariants,
        "pre_registered_order_observed": observed_order == expected_order,
        "all_training_sources_are_nominal_calibration_only": all(
            run["role"] == "calibration" and
            run["config"]["baseline_training_eligible"] and
            not run["config"]["truePlantPayloadInertialEnabled"]
            for run in runs["calibration"]),
        "heldout_sources_excluded_from_training": all(
            run["role"] == "heldout" and
            run["config"]["heldout_excluded_from_training"]
            for run in runs["heldout"]),
        "minimum_two_complete_supercycles_per_run": supercycle_coverage_ok,
        "heldout_lookup_coverage_gate": min(coverage) >=
            gates["minimum_heldout_lookup_duration_coverage"],
        "heldout_mean_terminal_drift_suppression_at_least_50_percent":
            suppression >= gates[
                "minimum_mean_heldout_terminal_drift_suppression_fraction"],
        "corrected_terminal_drift_lower_in_every_heldout_run": bool(
            np.all(corrected_norms < raw_norms)),
    }
    passed = all(checks.values())

    provenance = {
        "schema_version": 1,
        "stage": "D5I3_frozen_nominal_E3_composite_phase_baseline",
        "passed": passed,
        "baseline_csv": str(baseline_csv),
        "baseline_csv_sha256": sha256(baseline_csv),
        "protocol_file": str(protocol_path),
        "protocol_sha256": sha256(protocol_path),
        "training_formula": protocol["baseline"]["training_formula"],
        "application_formula": protocol["baseline"]["application_formula"],
        "composite_key": protocol["baseline"]["composite_key"],
        "causality": protocol["baseline"]["causality"],
        "unsupported_key_behavior":
            protocol["baseline"]["unsupported_key_behavior"],
        "training_runs": [{
            "path": str(run["path"]), "bag_sha256": run["bag_sha256"],
            "source_manifest_sha256": run["source_manifest_sha256"]}
            for run in runs["calibration"]],
        "heldout_runs_not_used_for_training": [{
            "path": str(run["path"]), "bag_sha256": run["bag_sha256"]}
            for run in runs["heldout"]],
        "occupied_composite_keys": len(template["values"]),
        "training_duration_s": float(sum(template["duration"].values())),
        "offline_analysis_script": str(Path(__file__).resolve()),
        "offline_analysis_script_sha256": sha256(Path(__file__)),
    }
    provenance_path = output / "nominal_e3_composite_phase_baseline_v1.json"
    provenance_path.write_text(json.dumps(provenance, indent=2,
                                          sort_keys=True) + "\n")

    heldout_detail = []
    for index, value in enumerate(heldout, 1):
        heldout_detail.append({
            "run": "run_{:02d}".format(index),
            "raw_terminal_10d": value["raw_terminal"].tolist(),
            "corrected_terminal_10d": value["corrected_terminal"].tolist(),
            "raw_terminal_scaled_norm": value["raw_terminal_scaled_norm"],
            "corrected_terminal_scaled_norm":
                value["corrected_terminal_scaled_norm"],
            "terminal_suppression_fraction":
                value["terminal_suppression_fraction"],
            "lookup_duration_coverage": value["lookup_duration_coverage"],
            "raw_drive_scaled_rms": value["raw_drive_scaled_rms"],
            "corrected_drive_scaled_rms":
                value["corrected_drive_scaled_rms"],
            "per_supercycle": value["per_supercycle"],
        })
    tracking = {
        role: {
            metric: {
                "mean": float(np.mean([run["summary"]["tracking"][metric]
                                       for run in group])),
                "sample_std": float(np.std(
                    [run["summary"]["tracking"][metric] for run in group],
                    ddof=1)),
            } for metric in ("position_rmse_m", "position_peak_m",
                             "orientation_rmse_rad", "orientation_peak_rad")
        } for role, group in runs.items()
    }
    summary = {
        "schema_version": 1,
        "stage": "D5I3_nominal_only_baseline_calibration",
        "passed": passed,
        "checks": checks,
        "run_count": 6,
        "fully_contained_gait_cycle_counts": cycle_counts,
        "steady_overlapping_gait_cycle_counts": overlap_cycle_counts,
        "steady_evaluated_durations_s": steady_durations,
        "per_run_supercycle_evaluated_durations_s": [
            {str(key): value for key, value in values.items()}
            for values in supercycle_durations],
        "template": provenance,
        "heldout_validation": {
            "per_run": heldout_detail,
            "raw_terminal_scaled_norm_mean": float(np.mean(raw_norms)),
            "raw_terminal_scaled_norm_sample_std": float(np.std(raw_norms,
                                                                  ddof=1)),
            "corrected_terminal_scaled_norm_mean":
                float(np.mean(corrected_norms)),
            "corrected_terminal_scaled_norm_sample_std":
                float(np.std(corrected_norms, ddof=1)),
            "mean_terminal_suppression_fraction": suppression,
            "minimum_lookup_duration_coverage": min(coverage),
        },
        "calibration_leave_one_out_diagnostic": [
            serializable_evaluation(value) for value in calibration_loo],
        "tracking_repeatability_estimator_off": tracking,
        "next_stage_decision": (
            "D5I3 Gate passed. The frozen template may enter D5I4 four-mode "
            "online smoke; it must not be retrained with payload data."
            if passed else
            "D5I3 Gate failed. Do not enter D5I4 or payload acquisition."),
        "scientific_boundary": (
            "D5I3 is nominal-only estimator-OFF calibration plus held-out "
            "counterfactual validation. It does not show an applied estimator, "
            "Eq.(10) policy effect, payload retention, tracking benefit, or "
            "10D parameter convergence."),
    }
    summary_path = output / "stage_d5i3_campaign_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")

    representative = heldout[0]
    time = np.asarray([row["time"] for row in representative["rows"]])
    raw = np.asarray([row["drive"] for row in representative["rows"]]) / scales
    base = np.asarray([row["baseline"] for row in representative["rows"]]) / scales
    corrected = np.asarray([row["corrected"] for row in representative["rows"]]) / scales
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for axis, index in zip(axes, (3, 4, 7)):
        axis.plot(time, raw[:, index], label="raw Y^T sigma", alpha=0.8)
        axis.plot(time, base[:, index], label="frozen nominal baseline", alpha=0.8)
        axis.plot(time, corrected[:, index], label="corrected", alpha=0.8)
        axis.set_ylabel(PARAMETERS[index] + " / scale / s")
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8)
    axes[-1].set_xlabel("MPC time [s]")
    fig.suptitle("D5I3 held-out run_01: composite-phase baseline")
    fig.tight_layout()
    fig.savefig(output / "figure_D5I3_01_heldout_drive.png", dpi=180)
    plt.close(fig)

    raw_vectors = np.asarray([value["raw_terminal"] for value in heldout]) / scales
    corrected_vectors = np.asarray(
        [value["corrected_terminal"] for value in heldout]) / scales
    x = np.arange(10)
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.bar(x - 0.2, np.mean(raw_vectors, axis=0), 0.4,
           yerr=np.std(raw_vectors, axis=0, ddof=1), label="raw")
    ax.bar(x + 0.2, np.mean(corrected_vectors, axis=0), 0.4,
           yerr=np.std(corrected_vectors, axis=0, ddof=1), label="corrected")
    ax.set_xticks(x, PARAMETERS)
    ax.set_ylabel("terminal counterfactual estimate / physical scale")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / "figure_D5I3_02_heldout_terminal.png", dpi=180)
    plt.close(fig)

    report = """# Stage D5I3 — E3_combined Nominal Baseline Calibration

## Gate result

- D5I3 Gate: **{state}**
- Acquisition: 3 calibration + 3 held-out nominal runs; estimator OFF; Eq.(10) adaptive dynamics OFF.
- Each run uses two complete 12 s steady supercycles.
- Frozen composite keys: {keys}; calibration duration: {duration:.6g} s.
- Minimum held-out lookup duration coverage: {coverage:.3%}.

## Held-out nominal robustness

- Raw terminal physical-scaled norm: {raw:.8g} ± {raw_std:.3g}.
- Corrected terminal physical-scaled norm: {corrected:.8g} ± {corrected_std:.3g}.
- Mean suppression: {suppression:.3%}; corrected lower in every held-out run: {every_lower}.

The template was trained only from the three pre-assigned nominal calibration
runs.  The three held-out runs were not used to create or update any cell.
Lookup is causal and steady-only: current supercycle phase bin, causally
reconstructed executed gait phase bin, and current executed mode. Unsupported
keys and all startup/ramp intervals produce zero baseline.

## Boundary and next action

D5I3 is an estimator-OFF calibration/held-out counterfactual Gate.  It does not
show that the correction has entered Eq.(8), Eq.(10), Eq.(11), MPC policy, or
payload tracking.  Only if this Gate passes may the frozen artifact enter D5I4
four-mode online smoke.  Payload data must never retrain this baseline.
""".format(
        state="PASS" if passed else "FAIL", keys=len(template["values"]),
        duration=sum(template["duration"].values()), coverage=min(coverage),
        raw=np.mean(raw_norms), raw_std=np.std(raw_norms, ddof=1),
        corrected=np.mean(corrected_norms),
        corrected_std=np.std(corrected_norms, ddof=1),
        suppression=suppression,
        every_lower=bool(np.all(corrected_norms < raw_norms)))
    (output / "stage_d5i3_campaign_report.md").write_text(report)

    manifest = {
        "schema_version": 1, "stage": "D5I3_artifact_manifest",
        "passed": passed,
        "artifacts": {path.name: sha256(path) for path in (
            protocol_path, baseline_csv, provenance_path, summary_path,
            output / "d5i3_baseline_validation_per_run.csv",
            output / "figure_D5I3_01_heldout_drive.png",
            output / "figure_D5I3_02_heldout_terminal.png",
            output / "stage_d5i3_campaign_report.md")},
        "raw_bags": {str(run["bag"]): run["bag_sha256"]
                     for group in runs.values() for run in group},
    }
    (root / "stage_d5i3_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(report)
    if not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
