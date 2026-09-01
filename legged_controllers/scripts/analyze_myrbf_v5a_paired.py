#!/usr/bin/env python3

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

import numpy as np

import myrbf_v5a_campaign_schema as campaign_schema


MIN_DISTURBANCE_SAMPLES = 10
MIN_STREAM_OVERLAP = 0.95
PAIRED_METRICS = (
    "estimate_rmse_n",
    "estimate_correlation",
    "estimate_bias_n",
    "response_lag_s",
    "requested_compensation_rms_n",
    "achieved_compensation_rms_n",
    "requested_achieved_rms_gap_n",
    "compensation_sign_agreement",
    "lateral_rms_m",
    "lateral_max_m",
    "roll_rms_deg",
    "roll_max_deg",
    "allocator_scale_mean",
    "allocator_scale_min",
    "saturation_duration_s",
    "freeze_rate",
    "projection_rate",
    "minimum_base_height_m",
    "recovery_2s_lateral_m",
)


def _mean(values):
    return sum(values) / len(values)


def _rms(values):
    return math.sqrt(_mean([value * value for value in values]))


def _correlation(left, right):
    left_mean = _mean(left)
    right_mean = _mean(right)
    numerator = sum(
        (left_value - left_mean) * (right_value - right_mean)
        for left_value, right_value in zip(left, right)
    )
    left_energy = sum((value - left_mean) ** 2 for value in left)
    right_energy = sum((value - right_mean) ** 2 for value in right)
    denominator = math.sqrt(left_energy * right_energy)
    return numerator / denominator if denominator > 0.0 else 0.0


def _response_lag(time, target, estimate):
    if len(time) < 3:
        return 0.0
    if (_correlation(target, target) == 0.0 or
            _correlation(estimate, estimate) == 0.0):
        return 0.0
    sample_period = float(np.median(np.diff(np.asarray(time, dtype=float))))
    maximum_shift = min(len(time) // 2, int(round(1.0 / sample_period)))
    best_shift = 0
    best_correlation = -float("inf")
    for shift in range(-maximum_shift, maximum_shift + 1):
        if shift > 0:
            left = target[:-shift]
            right = estimate[shift:]
        elif shift < 0:
            left = target[-shift:]
            right = estimate[:shift]
        else:
            left = target
            right = estimate
        if len(left) < 3:
            continue
        value = _correlation(left, right)
        if value > best_correlation:
            best_correlation = value
            best_shift = shift
    return best_shift * sample_period


def force_response_metrics(time, external, estimate, requested, achieved):
    lengths = {len(time), len(external), len(estimate), len(requested), len(achieved)}
    if lengths != {len(time)} or not time:
        raise ValueError("force metric series must have one nonempty common length")
    target = [-float(value) for value in external]
    estimate = [float(value) for value in estimate]
    requested = [float(value) for value in requested]
    achieved = [float(value) for value in achieved]
    errors = [value - wanted for value, wanted in zip(estimate, target)]
    gaps = [wanted - value for wanted, value in zip(requested, achieved)]
    sign_samples = [
        (wanted, value)
        for outside, wanted, value in zip(external, target, achieved)
        if abs(float(outside)) >= 0.05
    ]
    agreement = (
        _mean([1.0 if wanted * value > 0.0 else 0.0
               for wanted, value in sign_samples])
        if sign_samples else 0.0
    )
    return {
        "estimate_rmse_n": _rms(errors),
        "estimate_correlation": _correlation(target, estimate),
        "estimate_bias_n": _mean(errors),
        "response_lag_s": _response_lag(time, target, estimate),
        "requested_compensation_rms_n": _rms(requested),
        "achieved_compensation_rms_n": _rms(achieved),
        "requested_achieved_rms_gap_n": _rms(gaps),
        "compensation_sign_agreement": agreement,
    }


def onset_relative_rms(time, values, onset, stop, unwrap=False):
    selected = [
        float(value)
        for sample_time, value in zip(time, values)
        if onset <= float(sample_time) <= stop
    ]
    if not selected:
        raise ValueError("motion window has no usable samples")
    if unwrap:
        selected = np.unwrap(np.asarray(selected, dtype=float)).tolist()
    baseline = selected[0]
    relative = [value - baseline for value in selected]
    return math.sqrt(sum(value * value for value in relative) / len(relative))


def load_manifest(root):
    path = Path(root) / "manifest.json"
    if not path.is_file():
        raise ValueError("campaign manifest.json is missing")
    try:
        with path.open("r", encoding="utf-8") as handle:
            manifest = json.load(handle)
    except (OSError, ValueError) as error:
        raise ValueError("campaign manifest.json is invalid: %s" % error)
    if not isinstance(manifest, dict):
        raise ValueError("campaign manifest must be a JSON object")
    return manifest


def _missing_fields(mapping, required):
    return [field for field in required if field not in mapping]


def _run_directory(root, run):
    root = Path(root).resolve()
    relative = Path(str(run.get("output_dir", run["run_id"])))
    if relative.is_absolute():
        raise ValueError("run output directory must be relative to campaign root")
    directory = (root / relative).resolve()
    try:
        directory.relative_to(root)
    except ValueError:
        raise ValueError("run output directory escapes campaign root")
    return directory


def validate_campaign(manifest, root):
    missing = _missing_fields(manifest, campaign_schema.REQUIRED_MANIFEST_FIELDS)
    if missing:
        raise ValueError("manifest missing required fields: %s" % ", ".join(missing))
    if manifest["schema_version"] != campaign_schema.SCHEMA_VERSION:
        raise ValueError("manifest schema version mismatch")
    expected_snapshot = campaign_schema.protocol_snapshot()
    if manifest["protocol_snapshot"] != expected_snapshot:
        raise ValueError("protocol snapshot does not match frozen campaign schema")
    protocol_path = Path(root) / campaign_schema.PROTOCOL_FILENAME
    if not protocol_path.is_file():
        raise ValueError("frozen PROTOCOL.md is missing from campaign root")
    protocol_hash = hashlib.sha256(protocol_path.read_bytes()).hexdigest()
    if protocol_hash != manifest["protocol_snapshot"]["protocol_sha256"]:
        raise ValueError("PROTOCOL.md SHA-256 does not match protocol snapshot")

    runs = manifest["runs"]
    if not isinstance(runs, list):
        raise ValueError("manifest runs must be a list")
    identities = []
    run_ids = set()
    for index, run in enumerate(runs):
        if not isinstance(run, dict):
            raise ValueError("manifest run %d must be an object" % index)
        missing = _missing_fields(run, campaign_schema.REQUIRED_RUN_FIELDS)
        if missing:
            raise ValueError("run %d missing required fields: %s" %
                             (index, ", ".join(missing)))
        identity = (
            run["method"], run["scenario"], run["repeat"], run["seed"],
            run["order"],
        )
        pair_identity = identity[:4]
        if pair_identity in [item[:4] for item in identities]:
            raise ValueError("duplicate run identity: %r" % (pair_identity,))
        identities.append(identity)
        if run["run_id"] in run_ids:
            raise ValueError("duplicate run_id: %s" % run["run_id"])
        run_ids.add(run["run_id"])
        method_snapshot = manifest["protocol_snapshot"]["methods"].get(
            run["method"])
        if method_snapshot is None:
            raise ValueError("run method does not match protocol snapshot")
        if run["config_path"] != method_snapshot["config_path"]:
            raise ValueError("run configuration path does not match protocol snapshot")
        if run["config_sha256"] != method_snapshot["config_sha256"]:
            raise ValueError("run configuration hash does not match protocol snapshot")
        if (run["exit_code"] != 0 or not run["finite"] or
                not run["complete"] or run["controller_rows"] <= 0 or
                run["estimator_rows"] <= 0):
            raise ValueError("run %s is not complete and usable" % run["run_id"])
        directory = _run_directory(root, run)
        if not directory.is_dir():
            raise ValueError("run directory is missing: %s" % run["run_id"])

    expected = set(campaign_schema.expected_run_identities())
    actual = set(identities)
    if actual != expected:
        raise ValueError(
            "incomplete campaign: expected 30 exact runs, found %d" % len(runs))


def _read_csv(path, required_fields, string_fields=(), skip_leading_nonfinite=False):
    path = Path(path)
    if not path.is_file():
        raise ValueError("required CSV is missing: %s" % path.name)
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = tuple(reader.fieldnames or ())
        missing = [field for field in required_fields if field not in fields]
        if missing:
            raise ValueError("%s missing required fields: %s" %
                             (path.name, ", ".join(missing)))
        rows = []
        usable_started = False
        for row_number, source in enumerate(reader, start=2):
            row = dict(source)
            finite = True
            for field in required_fields:
                if field in string_fields:
                    continue
                try:
                    value = float(row[field])
                except (TypeError, ValueError):
                    finite = False
                    break
                if not math.isfinite(value):
                    finite = False
                    break
                row[field] = value
            if not finite:
                if skip_leading_nonfinite and not usable_started:
                    continue
                raise ValueError("%s row %d has a nonfinite usable value" %
                                 (path.name, row_number))
            usable_started = True
            rows.append(row)
    if not rows:
        raise ValueError("%s has no usable samples" % path.name)
    return rows


def _nearest_values(sample_times, source_rows, time_field, value_field):
    source_times = np.asarray([row[time_field] for row in source_rows], dtype=float)
    source_values = np.asarray([row[value_field] for row in source_rows], dtype=float)
    order = np.argsort(source_times)
    source_times = source_times[order]
    source_values = source_values[order]
    unique_times, unique_indices = np.unique(source_times, return_index=True)
    unique_values = source_values[unique_indices]
    return np.interp(np.asarray(sample_times, dtype=float),
                     unique_times, unique_values).tolist()


def _relative_values(time, values, onset, stop, unwrap=False):
    selected = [
        (float(sample_time), float(value))
        for sample_time, value in zip(time, values)
        if onset <= float(sample_time) <= stop
    ]
    if not selected:
        raise ValueError("motion window has no usable samples")
    selected_times = [item[0] for item in selected]
    selected_values = np.asarray([item[1] for item in selected], dtype=float)
    if unwrap:
        selected_values = np.unwrap(selected_values)
    selected_values = selected_values - selected_values[0]
    return selected_times, selected_values.tolist()


def _longest_saturation_duration(times, allocator):
    if not times:
        return 0.0
    period = (float(np.median(np.diff(np.asarray(times, dtype=float))))
              if len(times) > 1 else 0.0)
    longest = 0.0
    start = None
    previous = None
    for sample_time, scale in zip(times, allocator):
        if scale < 1.0 - 1e-9:
            if start is None:
                start = sample_time
            previous = sample_time
        elif start is not None:
            longest = max(longest, previous - start + period)
            start = None
            previous = None
    if start is not None:
        longest = max(longest, previous - start + period)
    return longest


def analyze_run(run, root):
    directory = _run_directory(root, run)
    shadow = _read_csv(
        directory / "shadow.csv",
        campaign_schema.REQUIRED_CSV_FIELDS["shadow.csv"],
    )
    state = _read_csv(
        directory / "state.csv",
        campaign_schema.REQUIRED_CSV_FIELDS["state.csv"],
        skip_leading_nonfinite=True,
    )
    wrench = _read_csv(
        directory / "wrench.csv",
        campaign_schema.REQUIRED_CSV_FIELDS["wrench.csv"],
    )
    motion = _read_csv(
        directory / "motion.csv",
        campaign_schema.REQUIRED_CSV_FIELDS["motion.csv"],
        string_fields=("gait",),
    )

    scenario = run["scenario"]
    if scenario == "no_disturbance":
        if any(abs(row["fy"]) > 1e-9 for row in wrench):
            raise ValueError("no_disturbance contains nonzero force truth")
        active_motion = [row for row in motion if row["active"] > 0.5]
        if len(active_motion) < 2:
            raise ValueError("no_disturbance has too few active motion samples")
        onset = active_motion[0]["ros_time"]
        stop = active_motion[-1]["ros_time"]
        disturbance_count = 0
    else:
        active_truth = [row for row in wrench if abs(row["fy"]) >= 0.05]
        if len(active_truth) < MIN_DISTURBANCE_SAMPLES:
            raise ValueError("too few disturbance samples in wrench truth")
        onset = active_truth[0]["ros_time"]
        stop = active_truth[-1]["ros_time"]
        disturbance_count = len(active_truth)
    if stop <= onset:
        raise ValueError("analysis window has nonpositive duration")

    active_shadow = [row for row in shadow
                     if onset <= row["time_s"] <= stop]
    if len(active_shadow) < 2:
        raise ValueError("controller CSV has too few analysis-window samples")
    shadow_times = [row["time_s"] for row in active_shadow]
    overlap_start = max(onset, shadow_times[0], state[0]["ros_time"])
    overlap_stop = min(stop, shadow_times[-1], state[-1]["ros_time"])
    overlap = max(0.0, overlap_stop - overlap_start) / (stop - onset)
    if overlap + 1e-12 < MIN_STREAM_OVERLAP:
        raise ValueError("controller/estimator timestamp overlap is below 95%")

    external = (_nearest_values(shadow_times, wrench, "ros_time", "fy")
                if scenario != "no_disturbance"
                else [0.0] * len(active_shadow))
    estimate = [row["estimate_after_fy_n"] for row in active_shadow]
    requested = [row["requested_control_fy_n"] for row in active_shadow]
    achieved = [row["achieved_control_fy_n"] for row in active_shadow]
    metrics = force_response_metrics(
        shadow_times, external, estimate, requested, achieved)

    _, lateral = _relative_values(
        shadow_times,
        [row["actual_base_y_m"] for row in active_shadow],
        onset,
        stop,
    )
    _, roll_rad = _relative_values(
        shadow_times,
        [row["actual_base_roll_rad"] for row in active_shadow],
        onset,
        stop,
        unwrap=True,
    )
    roll_deg = [math.degrees(value) for value in roll_rad]
    allocator = [row["allocator_scale"] for row in active_shadow]
    projection = [
        1.0 if any(row["projection_active_" + axis] > 0.5
                   for axis in ("fx", "fy", "fz", "tx", "ty", "tz"))
        else 0.0
        for row in active_shadow
    ]
    recovery_time = stop + 2.0
    all_shadow_times = [row["time_s"] for row in shadow]
    if recovery_time > all_shadow_times[-1] + 1e-12:
        raise ValueError("controller CSV does not cover two-second recovery")
    onset_y = _nearest_values(
        [onset], shadow, "time_s", "actual_base_y_m")[0]
    recovery_y = _nearest_values(
        [recovery_time], shadow, "time_s", "actual_base_y_m")[0]

    metrics.update({
        "run_id": run["run_id"],
        "method": run["method"],
        "scenario": scenario,
        "repeat": run["repeat"],
        "seed": run["seed"],
        "order": run["order"],
        "analysis_onset_s": onset,
        "analysis_stop_s": stop,
        "analysis_sample_count": len(active_shadow),
        "disturbance_sample_count": disturbance_count,
        "stream_overlap_fraction": overlap,
        "lateral_rms_m": _rms(lateral),
        "lateral_max_m": max(abs(value) for value in lateral),
        "roll_rms_deg": _rms(roll_deg),
        "roll_max_deg": max(abs(value) for value in roll_deg),
        "allocator_scale_mean": _mean(allocator),
        "allocator_scale_min": min(allocator),
        "saturation_duration_s": _longest_saturation_duration(
            shadow_times, allocator),
        "freeze_rate": _mean([row["update_frozen"]
                              for row in active_shadow]),
        "projection_rate": _mean(projection),
        "minimum_base_height_m": min(row["base_z"] for row in state),
        "recovery_2s_lateral_m": abs(recovery_y - onset_y),
    })
    return metrics


def paired_differences(run_metrics):
    grouped = {}
    for run in run_metrics:
        key = (run["scenario"], run["repeat"], run["seed"])
        methods = grouped.setdefault(key, {})
        if run["method"] in methods:
            raise ValueError("duplicate analyzed run for pair %r" % (key,))
        methods[run["method"]] = run
    pairs = []
    scenario_order = {name: index for index, name in
                      enumerate(campaign_schema.SCENARIOS)}
    for key in sorted(grouped, key=lambda item: (
            scenario_order.get(item[0], len(scenario_order)), item[1], item[2])):
        methods = grouped[key]
        if set(methods) != set(campaign_schema.METHODS):
            raise ValueError("pair %r does not contain V4 and V5-A" % (key,))
        v4_run = methods["V4"]
        v5a_run = methods["V5-A"]
        for metric in PAIRED_METRICS:
            if metric not in v4_run or metric not in v5a_run:
                raise ValueError("pair %r is missing metric %s" % (key, metric))
            v4 = float(v4_run[metric])
            v5a = float(v5a_run[metric])
            if not math.isfinite(v4) or not math.isfinite(v5a):
                raise ValueError("pair %r has nonfinite metric %s" % (key, metric))
            difference = v5a - v4
            if v4 == 0.0:
                percent = 0.0 if v5a == 0.0 else None
            else:
                percent = 100.0 * difference / v4
            pairs.append({
                "scenario": key[0],
                "repeat": key[1],
                "seed": key[2],
                "metric": metric,
                "v4": v4,
                "v5a": v5a,
                "difference": difference,
                "percent_difference": percent,
            })
    return pairs


def summarize_pairs(pairs):
    grouped = {}
    for pair in pairs:
        grouped.setdefault((pair["scenario"], pair["metric"]), []).append(pair)
    scenario_order = {name: index for index, name in
                      enumerate(campaign_schema.SCENARIOS)}
    metric_order = {name: index for index, name in enumerate(PAIRED_METRICS)}
    summary = []
    for key in sorted(grouped, key=lambda item: (
            scenario_order.get(item[0], len(scenario_order)),
            metric_order.get(item[1], len(metric_order)))):
        points = sorted(grouped[key], key=lambda item: item["repeat"])
        if len(points) != 5 or [item["repeat"] for item in points] != [1, 2, 3, 4, 5]:
            raise ValueError("summary %r must contain five paired repeats" % (key,))
        differences = [item["difference"] for item in points]
        median_difference = statistics.median(differences)
        percents = [item["percent_difference"] for item in points
                    if item["percent_difference"] is not None]
        summary.append({
            "scenario": key[0],
            "metric": key[1],
            "v4_median": statistics.median([item["v4"] for item in points]),
            "v5a_median": statistics.median([item["v5a"] for item in points]),
            "median_difference": median_difference,
            "median_percent_difference": (
                statistics.median(percents) if percents else None),
            "mad_difference": statistics.median([
                abs(value - median_difference) for value in differences
            ]),
            "pairs": [
                {
                    "repeat": item["repeat"],
                    "seed": item["seed"],
                    "v4": item["v4"],
                    "v5a": item["v5a"],
                    "difference": item["difference"],
                    "percent_difference": item["percent_difference"],
                }
                for item in points
            ],
        })
    return summary


def _summary_row(summary, scenario, metric):
    matches = [row for row in summary
               if row["scenario"] == scenario and row["metric"] == metric]
    if len(matches) != 1:
        raise ValueError("missing unique summary for %s/%s" % (scenario, metric))
    return matches[0]


def _all_run_values(row):
    values = []
    for point in row["pairs"]:
        values.extend((float(point["v4"]), float(point["v5a"])))
    return values


def _ratio_check(summary, scenario, metric, limit):
    row = _summary_row(summary, scenario, metric)
    v4 = float(row["v4_median"])
    v5a = float(row["v5a_median"])
    passed = v5a <= limit * v4 + 1e-12
    return {
        "metric": metric,
        "rule": "v5a_median <= %.12g * v4_median" % limit,
        "v4_median": v4,
        "v5a_median": v5a,
        "limit_ratio": limit,
        "observed_ratio": (v5a / v4 if v4 != 0.0 else None),
        "passed": passed,
    }


def evaluate_gates(summary, protocol):
    gates = protocol.get("gates")
    if not isinstance(gates, dict):
        raise ValueError("protocol snapshot has no gate thresholds")

    safety_rules = gates["safety"]
    heights = []
    freezes = []
    saturations = []
    for scenario in campaign_schema.SCENARIOS:
        heights.extend(_all_run_values(
            _summary_row(summary, scenario, "minimum_base_height_m")))
        freezes.extend(_all_run_values(
            _summary_row(summary, scenario, "freeze_rate")))
        saturations.extend(_all_run_values(
            _summary_row(summary, scenario, "saturation_duration_s")))
    safety_checks = [
        {
            "metric": "minimum_base_height_m",
            "rule": "every_run >= threshold",
            "threshold": safety_rules["minimum_base_height_m"],
            "observed_worst": min(heights),
            "passed": all(value >= safety_rules["minimum_base_height_m"]
                          for value in heights),
        },
        {
            "metric": "freeze_rate",
            "rule": "every_run < threshold",
            "threshold": safety_rules["maximum_freeze_rate"],
            "observed_worst": max(freezes),
            "passed": all(value < safety_rules["maximum_freeze_rate"]
                          for value in freezes),
        },
        {
            "metric": "saturation_duration_s",
            "rule": "every_run <= threshold",
            "threshold": safety_rules["maximum_saturation_duration_s"],
            "observed_worst": max(saturations),
            "passed": all(
                value <= safety_rules["maximum_saturation_duration_s"] + 1e-12
                for value in saturations),
        },
    ]
    safety = {
        "passed": all(check["passed"] for check in safety_checks),
        "checks": safety_checks,
    }

    scenarios = {}
    no_disturbance_rules = gates["no_disturbance"]
    no_disturbance_checks = [
        _ratio_check(summary, "no_disturbance", "lateral_rms_m",
                     no_disturbance_rules["lateral_rms_ratio"]),
        _ratio_check(summary, "no_disturbance", "roll_rms_deg",
                     no_disturbance_rules["roll_rms_ratio"]),
    ]
    compensation = _summary_row(
        summary, "no_disturbance", "achieved_compensation_rms_n")
    no_disturbance_checks.append({
        "metric": "achieved_compensation_rms_n",
        "rule": "v5a_median <= threshold",
        "threshold": no_disturbance_rules[
            "maximum_achieved_compensation_rms_n"],
        "observed": compensation["v5a_median"],
        "passed": compensation["v5a_median"] <= no_disturbance_rules[
            "maximum_achieved_compensation_rms_n"] + 1e-12,
    })
    scenarios["no_disturbance"] = {
        "passed": all(check["passed"] for check in no_disturbance_checks),
        "checks": no_disturbance_checks,
    }

    for scenario in ("constant_fy", "sine_fy"):
        rules = gates[scenario]
        checks = [
            _ratio_check(summary, scenario, "estimate_rmse_n",
                         rules["estimate_rmse_ratio"]),
            _ratio_check(summary, scenario, "lateral_rms_m",
                         rules["lateral_rms_ratio"]),
            _ratio_check(summary, scenario, "roll_rms_deg",
                         rules["roll_rms_ratio"]),
        ]
        scenarios[scenario] = {
            "passed": all(check["passed"] for check in checks),
            "checks": checks,
        }

    passed = safety["passed"] and all(
        item["passed"] for item in scenarios.values())
    return {
        "passed": passed,
        "decision": "PASS_ALL" if passed else "GENERAL_FAIL",
        "safety": safety,
        "scenarios": scenarios,
    }


def _write_summary_csv(path, summary):
    fields = [
        "scenario",
        "metric",
        "v4_median",
        "v5a_median",
        "median_difference",
        "median_percent_difference",
        "mad_difference",
    ]
    for repeat in range(1, 6):
        fields.extend((
            "pair_%d_repeat" % repeat,
            "pair_%d_seed" % repeat,
            "pair_%d_v4" % repeat,
            "pair_%d_v5a" % repeat,
            "pair_%d_difference" % repeat,
            "pair_%d_percent_difference" % repeat,
        ))
    with Path(path).open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for item in summary:
            row = {field: item.get(field, "") for field in fields[:7]}
            for index, point in enumerate(item["pairs"], start=1):
                row["pair_%d_repeat" % index] = point["repeat"]
                row["pair_%d_seed" % index] = point["seed"]
                row["pair_%d_v4" % index] = point["v4"]
                row["pair_%d_v5a" % index] = point["v5a"]
                row["pair_%d_difference" % index] = point["difference"]
                row["pair_%d_percent_difference" % index] = (
                    "" if point["percent_difference"] is None
                    else point["percent_difference"])
            writer.writerow(row)


def _plot_scenario_pairs(path, scenario, summary):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    panels = (
        ("estimate_rmse_n", "Estimate RMSE (N)"),
        ("lateral_rms_m", "Onset-relative lateral RMS (m)"),
        ("roll_rms_deg", "Onset-relative roll RMS (deg)"),
        ("achieved_compensation_rms_n", "Achieved compensation RMS (N)"),
    )
    colors = ("#0072B2", "#E69F00", "#009E73", "#CC79A7", "#D55E00")
    fig, axes = plt.subplots(2, 2, figsize=(9.2, 7.0), constrained_layout=True)
    for axis, (metric, label) in zip(axes.flat, panels):
        row = _summary_row(summary, scenario, metric)
        for color, point in zip(colors, row["pairs"]):
            axis.plot(
                (0, 1), (point["v4"], point["v5a"]),
                marker="o", linewidth=1.5, color=color,
                label="Repeat %d" % point["repeat"],
            )
        axis.set_xticks((0, 1))
        axis.set_xticklabels(("V4", "V5-A"))
        axis.set_ylabel(label)
        axis.grid(True, axis="y", alpha=0.25)
    axes.flat[0].legend(frameon=False, fontsize=8, ncol=2)
    fig.suptitle("MyRBF paired repeats: %s" % scenario.replace("_", " "))
    fig.savefig(Path(path), dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def analyze_campaign(root):
    root = Path(root)
    manifest = load_manifest(root)
    validate_campaign(manifest, root)
    run_metrics = [analyze_run(run, root) for run in manifest["runs"]]
    pairs = paired_differences(run_metrics)
    summary = summarize_pairs(pairs)
    gate_results = evaluate_gates(summary, manifest["protocol_snapshot"])
    result = {
        "schema_version": manifest["schema_version"],
        "protocol_snapshot": manifest["protocol_snapshot"],
        "run_metrics": run_metrics,
        "paired_differences": pairs,
        "paired_summary": summary,
        "gate_results": gate_results,
    }
    (root / "metrics.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    _write_summary_csv(root / "paired_summary.csv", summary)
    for scenario in campaign_schema.SCENARIOS:
        _plot_scenario_pairs(
            root / ("paired_%s.png" % scenario), scenario, summary)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Analyze the frozen MyRBF V4/V5-A paired campaign.")
    parser.add_argument("root", type=Path, help="campaign root containing manifest.json")
    args = parser.parse_args(argv)
    result = analyze_campaign(args.root)
    print(json.dumps(result["gate_results"], indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
