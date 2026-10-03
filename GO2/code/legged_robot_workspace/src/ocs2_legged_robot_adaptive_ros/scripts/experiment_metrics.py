#!/usr/bin/env python3
"""Pure, time-based metrics and fairness checks for Tasks B and C."""

import math


TASK_B_MODES = ("nominal", "estimate_only", "full_adaptive")
TASK_B_SWITCHES = {
    "nominal": {
        "enableParameterUpdate": False,
        "enableAdaptiveDynamics": False,
        "enableClfConstraint": False,
    },
    "estimate_only": {
        "enableParameterUpdate": True,
        "enableAdaptiveDynamics": False,
        "enableClfConstraint": False,
    },
    "full_adaptive": {
        "enableParameterUpdate": True,
        "enableAdaptiveDynamics": True,
        "enableClfConstraint": False,
    },
}


def _finite_sequence(values):
    return all(math.isfinite(float(value)) for value in values)


def _norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def trapezoid(times, values):
    if len(times) != len(values) or len(times) < 2:
        raise ValueError("trapezoid requires at least two paired samples")
    return sum(
        0.5 * (values[index] + values[index + 1]) * (times[index + 1] - times[index])
        for index in range(len(times) - 1)
    )


def time_weighted_rms(times, values):
    span = times[-1] - times[0]
    if span <= 0.0:
        raise ValueError("metric time span must be positive")
    return math.sqrt(trapezoid(times, [value * value for value in values]) / span)


def validate_rows(rows, synchronization_tolerance=1.0e-6):
    if len(rows) < 2:
        raise ValueError("at least two synchronized rows are required")
    previous_time = None
    for row in rows:
        required_vectors = ("position_error", "orientation_error", "sigma", "pi_hat")
        if any(key not in row for key in ("time", "snapshot_time") + required_vectors):
            raise ValueError("row is missing required fields")
        if len(row["position_error"]) != 3 or len(row["orientation_error"]) != 3:
            raise ValueError("pose error vectors must contain three values")
        if len(row["sigma"]) != 6 or len(row["pi_hat"]) != 6:
            raise ValueError("adaptive vectors must contain six values")
        flat = [row["time"], row["snapshot_time"]]
        for key in required_vectors:
            flat.extend(row[key])
        if not _finite_sequence(flat):
            raise ValueError("all experiment rows must be finite")
        if previous_time is not None and not row["time"] > previous_time:
            raise ValueError("row times must be strictly increasing")
        previous_time = row["time"]
        if abs(row["snapshot_time"] - row["time"]) > synchronization_tolerance:
            raise ValueError("snapshot synchronization tolerance exceeded")


def _recovery_time(times, errors, start_time, band, hold):
    if band <= 0.0 or hold <= 0.0:
        raise ValueError("recovery band and hold must be positive")
    for index, time in enumerate(times):
        if time < start_time or errors[index] > band:
            continue
        until = time + hold
        if times[-1] < until:
            continue
        held = [error for sample_time, error in zip(times[index:], errors[index:]) if sample_time <= until]
        if held and max(held) <= band:
            return time - start_time
    return None


def compute_task_b_metrics(rows, disturbance_end, recovery_band=0.005,
                           recovery_hold=1.0, synchronization_tolerance=1.0e-6):
    validate_rows(rows, synchronization_tolerance)
    times = [float(row["time"]) for row in rows]
    position = [_norm(row["position_error"]) for row in rows]
    orientation = [_norm(row["orientation_error"]) for row in rows]
    sigma = [_norm(row["sigma"]) for row in rows]
    time_errors = [abs(float(row["snapshot_time"]) - float(row["time"])) for row in rows]
    return {
        "sample_count": len(rows),
        "time_start_s": times[0],
        "time_end_s": times[-1],
        "peak_position_error_m": max(position),
        "rms_position_error_m": time_weighted_rms(times, position),
        "position_error_iae_m_s": trapezoid(times, position),
        "peak_orientation_error_rad": max(orientation),
        "rms_orientation_error_rad": time_weighted_rms(times, orientation),
        "orientation_error_iae_rad_s": trapezoid(times, orientation),
        "recovery_band_m": recovery_band,
        "recovery_hold_s": recovery_hold,
        "recovery_time_after_disturbance_s": _recovery_time(
            times, position, disturbance_end, recovery_band, recovery_hold),
        "peak_sigma_norm": max(sigma),
        "rms_sigma_norm": time_weighted_rms(times, sigma),
        "sigma_norm_integral": trapezoid(times, sigma),
        "pi_fx_initial": float(rows[0]["pi_hat"][0]),
        "pi_fx_final": float(rows[-1]["pi_hat"][0]),
        "pi_fx_change": float(rows[-1]["pi_hat"][0]) - float(rows[0]["pi_hat"][0]),
        "max_snapshot_time_error_s": max(time_errors),
    }


def validate_task_b_manifests(manifests):
    if len(manifests) != len(TASK_B_MODES):
        raise ValueError("Task B requires exactly three manifests")
    by_mode = {manifest.get("mode"): manifest for manifest in manifests}
    if set(by_mode) != set(TASK_B_MODES):
        raise ValueError("Task B modes are incomplete or duplicated")
    for mode in TASK_B_MODES:
        if by_mode[mode].get("switches") != TASK_B_SWITCHES[mode]:
            raise ValueError("{} switches do not match the declared mode".format(mode))

    reference = by_mode[TASK_B_MODES[0]]
    common = (
        "disturbance_start_s",
        "disturbance_end_s",
        "disturbance_wrench",
        "analysis_end_s",
        "initial_state",
    )
    for mode in TASK_B_MODES[1:]:
        for key in common:
            if by_mode[mode].get(key) != reference.get(key):
                label = "disturbance" if key.startswith("disturbance") else key
                raise ValueError("Task B {} mismatch for {}".format(label, mode))
    if reference.get("disturbance_wrench") != [20.0, 0.0, 0.0, 0.0, 0.0, 0.0]:
        raise ValueError("Task B disturbance must be +20 N Fx")
    if not reference.get("disturbance_start_s") < reference.get("disturbance_end_s") < reference.get("analysis_end_s"):
        raise ValueError("Task B disturbance timing is invalid")
    return True


TASK_C_MODES = ("linear_off", "linear_on", "quintic_off", "quintic_on")


def compute_task_c_metrics(rows, target_start, target_end, node_window=0.5,
                           recovery_band=0.005, recovery_hold=1.0,
                           synchronization_tolerance=1.0e-6):
    if not target_start < target_end or node_window <= 0.0:
        raise ValueError("Task C target timing is invalid")
    validate_rows(rows, synchronization_tolerance)
    for row in rows:
        if "h_clf" not in row or "violation" not in row:
            raise ValueError("Task C rows require h_clf and violation")
        if not _finite_sequence((row["h_clf"], row["violation"])):
            raise ValueError("Task C diagnostics must be finite")
        if row["violation"] < -1.0e-12:
            raise ValueError("Task C violation must be nonnegative")

    result = compute_task_b_metrics(
        rows,
        disturbance_end=target_end,
        recovery_band=recovery_band,
        recovery_hold=recovery_hold,
        synchronization_tolerance=synchronization_tolerance,
    )
    result["recovery_time_after_target_s"] = result.pop("recovery_time_after_disturbance_s")
    times = [float(row["time"]) for row in rows]
    sigma = [_norm(row["sigma"]) for row in rows]
    h_values = [float(row["h_clf"]) for row in rows]
    violation = [float(row["violation"]) for row in rows]
    start_values = [value for time, value in zip(times, sigma)
                    if target_start <= time <= target_start + node_window]
    end_values = [value for time, value in zip(times, sigma)
                  if target_end <= time <= target_end + node_window]
    if not start_values or not end_values:
        raise ValueError("Task C node windows contain no samples")
    result.update({
        "target_start_s": target_start,
        "target_end_s": target_end,
        "node_window_s": node_window,
        "start_node_peak_sigma": max(start_values),
        "end_node_peak_sigma": max(end_values),
        "min_h_clf": min(h_values),
        "mean_h_clf": trapezoid(times, h_values) / (times[-1] - times[0]),
        "max_violation": max(violation),
        "violation_integral": trapezoid(times, violation),
    })
    return result


def validate_task_c_manifests(manifests):
    if len(manifests) != len(TASK_C_MODES):
        raise ValueError("Task C requires exactly four manifests")
    by_mode = {manifest.get("mode"): manifest for manifest in manifests}
    if set(by_mode) != set(TASK_C_MODES):
        raise ValueError("Task C modes are incomplete or duplicated")
    for mode in TASK_C_MODES:
        profile, clf_mode = mode.split("_")
        manifest = by_mode[mode]
        expected_switches = {
            "enableParameterUpdate": True,
            "enableAdaptiveDynamics": True,
            "enableClfConstraint": clf_mode == "on",
        }
        if manifest.get("target_profile") != profile:
            raise ValueError("{} target profile does not match mode".format(mode))
        if manifest.get("switches") != expected_switches:
            raise ValueError("{} switches do not match mode".format(mode))

    reference = by_mode[TASK_C_MODES[0]]
    common = (
        "step_m", "target_duration_s", "sample_period_s",
        "settle_before_target_s", "analysis_end_s", "disturbance_wrench", "initial_state",
    )
    for mode in TASK_C_MODES[1:]:
        for key in common:
            if by_mode[mode].get(key) != reference.get(key):
                raise ValueError("Task C {} mismatch for {}".format(key, mode))
    if reference.get("step_m") != 0.10 or reference.get("target_duration_s", 0.0) <= 0.0:
        raise ValueError("Task C requires a 10 cm target and positive target_duration")
    if reference.get("disturbance_wrench") != [0.0] * 6:
        raise ValueError("Task C must not include an external disturbance")
    if not reference.get("settle_before_target_s") < reference.get("analysis_end_s"):
        raise ValueError("Task C analysis window is invalid")
    return True
