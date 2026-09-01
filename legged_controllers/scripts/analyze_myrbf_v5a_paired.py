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
MAX_SAMPLE_GAP_FACTOR = 3.0
FORCE_TOLERANCE_N = 1e-6
COMMAND_TOLERANCE = 1e-9
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


def _validate_launch_command(run):
    command = run["command"]
    if (not isinstance(command, list) or not command or
            any(not isinstance(argument, str) or not argument
                for argument in command)):
        raise ValueError("run command argument list is invalid")
    prefix_length = len(campaign_schema.LAUNCH_PREFIX)
    if tuple(command[:prefix_length]) != campaign_schema.LAUNCH_PREFIX:
        raise ValueError("run launch command has the wrong executable or launch file")
    arguments = {}
    for argument in command[prefix_length:]:
        if ":=" not in argument:
            raise ValueError("run launch command contains a malformed argument")
        name, value = argument.split(":=", 1)
        if not name or not value or name in arguments:
            raise ValueError("run launch command contains an invalid argument")
        arguments[name] = value
    expected = campaign_schema.expected_launch_arguments(
        run["method"], run["scenario"], run["seed"])
    expected_names = set(expected) | set(campaign_schema.LAUNCH_OUTPUT_ARGUMENTS)
    if set(arguments) != expected_names:
        raise ValueError("run launch command arguments do not match frozen schema")
    if any(arguments[name] != value for name, value in expected.items()):
        raise ValueError("run launch command settings do not match frozen schema")


def _initial_state_difference(field, left, right):
    difference = abs(float(left[field]) - float(right[field]))
    if field in campaign_schema.INITIAL_ORIENTATION_FIELDS:
        difference = min(difference, abs(2.0 * math.pi - difference))
    return difference


def _validate_paired_initial_state(reference, candidate):
    for field, tolerance in campaign_schema.INITIAL_STATE_TOLERANCES.items():
        if _initial_state_difference(field, reference, candidate) > tolerance + 1e-12:
            raise ValueError("recorded initial state differs within pair")


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
    source_commit = None
    pair_conditions = {}
    pair_initial_states = {}
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
        commit = run["git_commit"]
        if (not isinstance(commit, str) or len(commit) != 40 or
                any(character not in "0123456789abcdef"
                    for character in commit)):
            raise ValueError("run source commit must be one exact full hash")
        if source_commit is None:
            source_commit = commit
        elif commit != source_commit:
            raise ValueError("campaign source commit differs between runs")
        expected_conditions = campaign_schema.paired_conditions(
            run["scenario"], run["seed"])
        if (not isinstance(run["paired_conditions"], dict) or
                run["paired_conditions"] != expected_conditions):
            raise ValueError(
                "run paired conditions do not match frozen scenario and seed")
        pair_key = (run["scenario"], run["repeat"], run["seed"])
        if pair_key in pair_conditions:
            if run["paired_conditions"] != pair_conditions[pair_key]:
                raise ValueError("paired conditions differ within pair")
        else:
            pair_conditions[pair_key] = run["paired_conditions"]
        _validate_launch_command(run)
        if not isinstance(run["safety_stop_detected"], bool):
            raise ValueError("run safety_stop_detected must be boolean")
        if run["safety_stop_detected"]:
            raise ValueError("run %s recorded a safety stop" % run["run_id"])
        if (run["exit_code"] != 0 or not run["finite"] or
                not run["complete"] or run["controller_rows"] <= 0 or
                run["estimator_rows"] <= 0):
            raise ValueError("run %s is not complete and usable" % run["run_id"])
        directory = _run_directory(root, run)
        if not directory.is_dir():
            raise ValueError("run directory is missing: %s" % run["run_id"])
        state = _read_csv(
            directory / "state.csv",
            campaign_schema.REQUIRED_CSV_FIELDS["state.csv"],
            skip_leading_nonfinite=True,
        )
        initial_state = state[0]
        if pair_key in pair_initial_states:
            _validate_paired_initial_state(
                pair_initial_states[pair_key], initial_state)
        else:
            pair_initial_states[pair_key] = initial_state

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
    if not sample_times:
        return []
    source_times = np.asarray([row[time_field] for row in source_rows], dtype=float)
    source_values = np.asarray([row[value_field] for row in source_rows], dtype=float)
    order = np.argsort(source_times)
    source_times = source_times[order]
    source_values = source_values[order]
    unique_times, unique_indices = np.unique(source_times, return_index=True)
    unique_values = source_values[unique_indices]
    requested = np.asarray(sample_times, dtype=float)
    if (requested.min() < unique_times[0] - 1e-12 or
            requested.max() > unique_times[-1] + 1e-12):
        raise ValueError("source coverage does not include requested times")
    return np.interp(requested,
                     unique_times, unique_values).tolist()


def _validate_strictly_increasing(rows, field, label):
    values = [row[field] for row in rows]
    if any(right <= left for left, right in zip(values, values[1:])):
        raise ValueError("%s must be strictly increasing" % label)


def _validate_stream_continuity(rows, field, label):
    if len(rows) < 3:
        raise ValueError("%s has too few usable samples" % label)
    differences = [right[field] - left[field]
                   for left, right in zip(rows, rows[1:])]
    period = float(np.median(np.asarray(differences, dtype=float)))
    if max(differences) > MAX_SAMPLE_GAP_FACTOR * period + 1e-12:
        raise ValueError("%s has an excessive gap" % label)


def _validate_schedule_coverage(rows, start, stop, label):
    wall_times = [row["wall_elapsed"] for row in rows]
    if len(wall_times) < 3:
        raise ValueError("%s has too few samples" % label)
    differences = [right - left
                   for left, right in zip(wall_times, wall_times[1:])]
    if any(value <= 0.0 for value in differences):
        raise ValueError("%s has nonmonotonic sample times" % label)
    period = float(np.median(np.asarray(differences, dtype=float)))
    if wall_times[0] > start or wall_times[-1] < stop:
        raise ValueError("%s does not cover the frozen window" % label)
    if max(differences) > MAX_SAMPLE_GAP_FACTOR * period + 1e-12:
        raise ValueError("%s is disjoint inside the frozen window" % label)
    ros_times = [row["ros_time"] for row in rows]
    ros_differences = [right - left
                       for left, right in zip(ros_times, ros_times[1:])]
    if any(value <= 0.0 for value in ros_differences):
        raise ValueError("%s must have strictly increasing ROS times" % label)
    ros_period = float(np.median(np.asarray(ros_differences, dtype=float)))
    if max(ros_differences) > MAX_SAMPLE_GAP_FACTOR * ros_period + 1e-12:
        raise ValueError("%s has an excessive ROS-time gap" % label)
    return tuple(_nearest_values(
        [start, stop], rows, "wall_elapsed", "ros_time"))


def _validate_motion(motion, scenario_spec):
    start = scenario_spec["motion_delay_s"]
    stop = start + scenario_spec["motion_duration_s"]
    onset_ros, stop_ros = _validate_schedule_coverage(
        motion, start, stop, "motion coverage")
    for row in motion:
        expected_active = start <= row["wall_elapsed"] < stop
        recorded_active = row["active"] > 0.5
        if recorded_active != expected_active:
            raise ValueError("motion timing does not match frozen window")
        expected_gait = scenario_spec["gait"] if expected_active else "stance"
        if row["gait"] != expected_gait:
            raise ValueError("motion gait does not match frozen command")
        expected = (
            scenario_spec["vx_mps"] if expected_active else 0.0,
            scenario_spec["vy_mps"] if expected_active else 0.0,
            scenario_spec["yaw_rate_rps"] if expected_active else 0.0,
        )
        recorded = (row["vx"], row["vy"], row["yaw_rate"])
        if any(abs(value - wanted) > COMMAND_TOLERANCE
               for value, wanted in zip(recorded, expected)):
            raise ValueError("motion command does not match frozen velocity")
    return onset_ros, stop_ros


def _expected_force_value(scenario_spec, elapsed):
    start = scenario_spec["force_onset_s"]
    stop = start + scenario_spec["force_duration_s"]
    if not start <= elapsed < stop:
        return 0.0
    if scenario_spec["force_profile"] == "constant":
        return scenario_spec["force_value_n"]
    if scenario_spec["force_profile"] == "sine":
        return (
            scenario_spec["force_value_n"] +
            scenario_spec["force_amplitude_n"] * math.sin(
                2.0 * math.pi * scenario_spec["force_frequency_hz"] *
                (elapsed - start))
        )
    raise ValueError("unsupported frozen disturbance profile")


def _validate_disturbance_wrench(wrench, scenario_spec):
    start = scenario_spec["force_onset_s"]
    stop = start + scenario_spec["force_duration_s"]
    onset_ros, stop_ros = _validate_schedule_coverage(
        wrench, start, stop, "wrench truth coverage")
    called_rows = [row for row in wrench if row["service_called"] > 0.5]
    if len(called_rows) < MIN_DISTURBANCE_SAMPLES:
        raise ValueError("too few disturbance samples in wrench truth")
    for row in wrench:
        expected_active = start <= row["wall_elapsed"] < stop
        recorded_active = row["service_called"] > 0.5
        if recorded_active != expected_active:
            raise ValueError("force schedule does not match frozen duration")
        if recorded_active and row["service_success"] <= 0.5:
            raise ValueError("wrench service call failed in force window")
        expected_fy = _expected_force_value(
            scenario_spec, row["wall_elapsed"])
        if abs(row["fy"] - expected_fy) > FORCE_TOLERANCE_N:
            raise ValueError("force waveform does not match frozen scenario")
        if any(abs(row[axis]) > FORCE_TOLERANCE_N
               for axis in ("fx", "fz", "tx", "ty", "tz")):
            raise ValueError("force waveform uses a non-frozen wrench axis")
    return onset_ros, stop_ros, len(called_rows)


def _validate_zero_wrench(wrench):
    if any(abs(row[axis]) > FORCE_TOLERANCE_N
           for row in wrench
           for axis in ("fx", "fy", "fz", "tx", "ty", "tz")):
        raise ValueError("no_disturbance contains nonzero force truth")


def _relative_values(time, values, onset, stop, unwrap=False):
    all_times = np.asarray(time, dtype=float)
    all_values = np.asarray(values, dtype=float)
    if (not len(all_times) or onset < all_times[0] - 1e-12 or
            onset > all_times[-1] + 1e-12):
        raise ValueError("motion baseline lacks source coverage")
    if unwrap:
        all_values = np.unwrap(all_values)
    baseline = float(np.interp(onset, all_times, all_values))
    selected_indices = [
        index for index, sample_time in enumerate(all_times)
        if onset <= sample_time <= stop
    ]
    if not selected_indices:
        raise ValueError("motion window has no usable samples")
    selected_times = [float(all_times[index]) for index in selected_indices]
    selected_values = all_values[selected_indices] - baseline
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
    scenario = run["scenario"]
    scenario_spec = campaign_schema.SCENARIOS[scenario]
    shadow = _read_csv(
        directory / "shadow.csv",
        campaign_schema.REQUIRED_CSV_FIELDS["shadow.csv"],
    )
    state = _read_csv(
        directory / "state.csv",
        campaign_schema.REQUIRED_CSV_FIELDS["state.csv"],
        skip_leading_nonfinite=True,
    )
    _validate_strictly_increasing(shadow, "time_s", "controller times")
    _validate_strictly_increasing(state, "ros_time", "state times")
    _validate_stream_continuity(shadow, "time_s", "controller times")
    _validate_stream_continuity(state, "ros_time", "state times")
    motion = _read_csv(
        directory / "motion.csv",
        campaign_schema.REQUIRED_CSV_FIELDS["motion.csv"],
        string_fields=("gait",),
    )
    motion_onset, motion_stop = _validate_motion(motion, scenario_spec)
    wrench_path = directory / "wrench.csv"
    wrench = None
    if scenario_spec["wrench_required"] or wrench_path.is_file():
        wrench = _read_csv(
            wrench_path,
            campaign_schema.REQUIRED_CSV_FIELDS["wrench.csv"],
        )

    if not scenario_spec["wrench_required"]:
        if wrench is not None:
            _validate_zero_wrench(wrench)
        onset = motion_onset
        stop = motion_stop
        disturbance_count = 0
    else:
        onset, stop, disturbance_count = _validate_disturbance_wrench(
            wrench, scenario_spec)
    if stop <= onset:
        raise ValueError("analysis window has nonpositive duration")

    all_shadow_times = [row["time_s"] for row in shadow]
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
                if scenario_spec["wrench_required"]
                else [0.0] * len(active_shadow))
    estimate = [row["estimate_after_fy_n"] for row in active_shadow]
    requested = [row["requested_control_fy_n"] for row in active_shadow]
    achieved = [row["achieved_control_fy_n"] for row in active_shadow]
    metrics = force_response_metrics(
        shadow_times, external, estimate, requested, achieved)

    _, lateral = _relative_values(
        all_shadow_times,
        [row["actual_base_y_m"] for row in shadow],
        onset,
        stop,
    )
    _, roll_rad = _relative_values(
        all_shadow_times,
        [row["actual_base_roll_rad"] for row in shadow],
        onset,
        stop,
        unwrap=True,
    )
    roll_deg = [math.degrees(value) for value in roll_rad]
    allocator = [row["allocator_scale"] for row in active_shadow]
    full_allocator = [row["allocator_scale"] for row in shadow]
    projection = [
        1.0 if any(row["projection_active_" + axis] > 0.5
                   for axis in ("fx", "fy", "fz", "tx", "ty", "tz"))
        else 0.0
        for row in active_shadow
    ]
    recovery_time = stop + 2.0
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
            all_shadow_times, full_allocator),
        "freeze_rate": _mean([row["update_frozen"] for row in shadow]),
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
    if passed:
        decision = "PASS_ALL"
    elif (safety["passed"] and scenarios["no_disturbance"]["passed"] and
          scenarios["sine_fy"]["passed"] and
          not scenarios["constant_fy"]["passed"]):
        decision = "CONSTANT_ONLY_FAIL"
    else:
        decision = "GENERAL_FAIL"
    return {
        "passed": passed,
        "decision": decision,
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
    parser.add_argument(
        "root", nargs="?", type=Path,
        help="campaign root containing manifest.json")
    parser.add_argument(
        "--root", dest="root_option", type=Path,
        help="campaign root containing manifest.json")
    args = parser.parse_args(argv)
    if (args.root is None) == (args.root_option is None):
        parser.error("provide exactly one campaign root: PATH or --root PATH")
    root = args.root if args.root is not None else args.root_option
    result = analyze_campaign(root)
    print(json.dumps(result["gate_results"], indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
