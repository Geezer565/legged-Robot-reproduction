#!/usr/bin/env python3
"""Validate Go2 comparisons and draw only six-axis velocity and uncertainty.

No representative-repeat selection, extrapolated planner targets, invented
references, estimate sign inference, bars, or synthetic experimental evidence.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

STATE_FIELDS = ("observation_base_x", "observation_base_y", "observation_base_z",
                "observation_base_roll", "observation_base_pitch", "observation_base_yaw")
STATE_TARGET_FIELDS = ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")
LINEAR_VELOCITY_FIELDS = ("base_vx", "base_vy", "base_vz")
LINEAR_TARGET_RATE_FIELDS = ("vx_mps", "vy_mps", "vz_mps")
REFERENCE_FIELDS = STATE_TARGET_FIELDS + LINEAR_TARGET_RATE_FIELDS + tuple(
    "normalized_angular_momentum_" + a for a in ("x","y","z"))
TRACKING_PANEL_TITLES = (
    "Forward displacement Δx", "Lateral displacement Δy", "Base height z",
    "Roll stabilization", "Pitch stabilization", "Heading change Δyaw")
TRACKING_PANEL_YLABELS = (
    "Displacement (m)", "Displacement (m)", "Height (m)", "Angle (rad)", "Angle (rad)", "Angle (rad)")
RATE_PANEL_TITLES = ("Forward velocity vx", "Lateral velocity vy", "Vertical velocity vz",
                     "d(roll)/dt", "d(pitch)/dt", "d(yaw)/dt")
RATE_PANEL_YLABELS = ("Velocity (m/s)", "Velocity (m/s)", "Velocity (m/s)",
                      "Euler derivative (rad/s)", "Euler derivative (rad/s)", "Euler derivative (rad/s)")
WRENCH_FIELDS = ("fx", "fy", "fz", "tx", "ty", "tz")
SHADOW_ESTIMATE_FIELDS = ("estimate_after_fx_n", "estimate_after_fy_n",
                          "estimate_after_fz_n", "estimate_after_tx_nm",
                          "estimate_after_ty_nm", "estimate_after_tz_nm")
ADAPTIVE_EXTERNAL_FIELDS = tuple("external_wrench_estimate_" + axis
                                 for axis in WRENCH_FIELDS)
ADAPTIVE_METHODS = ("v3_teacher", "v4_convergence", "v5_residual_dynamics", "v6_variable_gain", "v11_equivalent_wrench")
WRENCH_DISPLAY_RATE_HZ = 20.0
COLORS = {"v3_teacher": "#0072B2", "v4_convergence": "#000000",
          "v11_equivalent_wrench": "#0072B2",
          "v5_residual_dynamics": "#009E73", "v6_variable_gain": "#0072B2",
          "pure_mpc": "#666666", "direct_wrench": "#E69F00", "eth": "#E69F00", "liu_van_mpc_port": "#D55E00",
          "zhou_ssi": "#CC79A7"}
LINESTYLES = {"v3_teacher": "-", "v4_convergence": "-",
             "v11_equivalent_wrench": "-",
               "v5_residual_dynamics": "-", "v6_variable_gain": "-", "pure_mpc": "-", "direct_wrench": "--", "eth": "--",
              "liu_van_mpc_port": "-.", "zhou_ssi": ":"}
METHOD_LABELS = {"v3_teacher": "MyRBF V3 (archived)", "v4_convergence": "MyRBF V4",
                 "v11_equivalent_wrench": "MyRBF V1.1",
                  "v5_residual_dynamics": "MyRBF V5", "v6_variable_gain": "MyRBF V6",
                 "pure_mpc": "Pure MPC", "direct_wrench": "Direct Wrench (6D)", "eth": "Direct Wrench (legacy ETH alias)", "liu_van_mpc_port": "Liu-VAN",
                 "zhou_ssi": "Zhou-RFF"}
DIAGNOSTIC_REQUIRED = ("clf_lhs", "clf_rhs", "clf_violation", "clf_feasible",
                       "mpc_snapshot_id", "wbc_snapshot_id", "hold_mismatch_power", "robust_mode",
                       "diagnostic_sequence", "diagnostic_publish_drops",
                       "desired_acceleration_initialized", "reference_wrench_is_final")
V3_DIAGNOSTIC_REQUIRED = (
    "v3_teacher_enabled", "v3_previous_policy_valid", "v3_composite_ratio",
    "v3_robot_mass", "v3_actual_update_dt",
) + tuple(prefix + "_" + axis for prefix in (
    "v3_prediction_error", "v3_tracking_error", "v3_composite_error"
) for axis in WRENCH_FIELDS)

V4_DIAGNOSTIC_REQUIRED = ("v4_convergence_enabled",)
V5_DIAGNOSTIC_REQUIRED = (
    "diagnostic_sequence", "diagnostic_publish_drops",
    "actual_update_dt",
) + tuple(prefix + "_" + axis for prefix in (
    "velocity", "nominal_rate", "observed_rate",
    "known_applied_acceleration", "target_residual", "prediction_error",
    "correction_acceleration", "total_rate",
) for axis in WRENCH_FIELDS)

def adaptive_record_name(method):
    return {"v11_equivalent_wrench": "v11_equivalent_wrench.csv", "v3_teacher": "v3_teacher.csv",
            "v4_convergence": "v4_convergence.csv",
            "v5_residual_dynamics": "v5_residual_dynamics.csv",
            "v6_variable_gain": "v6_variable_gain.csv"}[method]


def read_rows(path):
    with Path(path).open(encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        if len(set(fields)) != len(fields):
            raise ValueError(str(path) + ": duplicate column names")
        rows = list(reader)
    if any(None in row or any(v is None for v in row.values()) for row in rows):
        raise ValueError(str(path) + ": malformed row length (raw file retained)")
    return fields, rows


def numbers(rows, field):
    # No sorting, filtering, downsampling or smoothing. NaNs remain visible gaps.
    return [float(row[field]) if row[field].strip() else float("nan") for row in rows]


def expected_force(d, elapsed):
    delay = float(d.get("delay_s", 0.0))
    duration = float(d["duration_s"])

    if d["profile"] == "none":
        return 0.0

    # 扰动尚未开始，或者已经超过指定持续时间。
    if elapsed < delay or elapsed >= delay + duration:
        return 0.0

    active_time = elapsed - delay

    if d["profile"] == "constant":
        return d["value"]

    primary = d["amplitude"] * math.sin(
        2 * math.pi * d["frequency"] * active_time
        + math.radians(d["phase_deg"])
    )

    secondary = 0.0
    if d["profile"] == "multisine":
        secondary = d["secondary_amplitude"] * math.sin(
            2 * math.pi * d["secondary_frequency"] * active_time
            + math.radians(d["secondary_phase_deg"])
        )

    return d["value"] + primary + secondary


def validate_velocity_reference(path, manifest, campaign_start, state_rows):
    """Check recorded commands and actually received rolling MPC references."""
    import numpy as np
    _, motion = read_rows(Path(path) / "motion.csv")
    _, targets = read_rows(Path(path) / "mpc_target.csv")
    delay, duration = manifest["timing_s"]["goal_delay"], manifest["timing_s"]["total"]
    speed = manifest["planner"]["command_vx_mps"]
    ramp = manifest["planner"].get("ramp_duration_s", 0.0)
    active = [r for r in motion if delay <= float(r["wall_elapsed"]) < duration]
    if len(active) < 2:
        raise ValueError("missing constant-velocity command samples")
    times = numbers(active, "wall_elapsed")
    if (times[0] > delay + .2 or times[-1] < duration - .2 or
            any(b < a or b-a > .2 for a,b in zip(times,times[1:]))):
        raise ValueError("incomplete constant-velocity command coverage")
    for r in active:
        s = min(1.0, max(0.0, (float(r["wall_elapsed"])-delay)/ramp)) if ramp else 1.0
        expected = speed*s*s*(3-2*s)
        values = [float(r[k]) for k in ("campaign_start_ros","vx","vy","yaw_rate","active")]
        if (not all(math.isfinite(v) for v in values) or
                abs(values[0]-campaign_start) > 1e-8 or
                abs(values[1]-expected) > 1e-8 or values[2:] != [0.0,0.0,1.0] or
                r["gait"] != manifest["gait"]):
            raise ValueError("recorded command differs from constant forward velocity")
    if not targets:
        raise ValueError("no received MPC velocity target")
    if manifest["planner"].get("heading_reference") == "hold_initial_heading":
        headings = [float(r["yaw_rad"]) for r in targets if int(r["knot_index"]) == 1]
        if not headings or not np.all(np.isfinite(headings)) or not np.allclose(headings, headings[0], atol=1e-6, rtol=0):
            raise ValueError("zero-turn command did not preserve the recorded heading target")
    case = {"data":{"state":{"rows":state_rows},"mpc_target":{"rows":targets}}}
    ref = planner_on_state(case)
    window = np.asarray([delay+ramp+.2 <= float(r["wall_elapsed"]) < duration-.2 for r in state_rows])
    velocity = np.column_stack([ref[f] for f in LINEAR_TARGET_RATE_FIELDS])
    angular = np.column_stack([ref["normalized_angular_momentum_"+a] for a in ("x","y","z")])
    finite = np.all(np.isfinite(velocity),axis=1) & np.all(np.isfinite(angular),axis=1)
    if not np.any(window) or np.mean(finite[window]) < .95:
        raise ValueError("received velocity target has insufficient causal time coverage")
    # cmd_vel is BODY-frame; rotation can change world components, not its norm.
    if (not np.allclose(np.linalg.norm(velocity[window & finite],axis=1),speed,atol=1e-6) or
            not np.allclose(angular[window & finite],0,atol=1e-8)):
        raise ValueError("received MPC reference differs from forward command / zero angular target")


def validate_case(path, require_artifacts=True, allow_legacy=False):
    path = Path(path)
    errors, spans, streams = [], {}, {}
    manifest = json.loads((path / "case_manifest.json").read_text(encoding="utf-8"))
    duration = float(manifest["timing_s"]["total"])
    disturbance = manifest["disturbance"]
    accepted_experiments = {"GO2-METHOD-COMPARISON"}
    if allow_legacy:
        accepted_experiments.add("V2-LYAPUNOV")
    if manifest.get("experiment") not in accepted_experiments:
        errors.append("wrong experiment manifest")
    wrench_delay = float(disturbance.get("delay_s", 0.0))
    wrench_duration = float(disturbance.get("duration_s", 0.0))

    if (wrench_delay < 0 or wrench_duration <= 0 or
            abs((wrench_delay + wrench_duration) - duration) > 1e-9 or
            disturbance.get("withdrawn_during_recording") is not False):
        errors.append(
            "wrench schedule must start at its declared delay and remain active to recording end"
        )

    if (disturbance.get("axis") not in WRENCH_FIELDS or
            disturbance.get("reference_frame") != "world"):
        errors.append("unsupported wrench axis/frame")
    required = {"state.csv": STATE_FIELDS + LINEAR_VELOCITY_FIELDS,
                "wrench.csv": WRENCH_FIELDS + (
                    "reference_point_x", "reference_point_y",
                    "reference_point_z",
                    "service_called", "clear_success", "service_success")}
    diagnostic_name = None
    if manifest["method"] in ("eth_adaptive_clf_mpc", "eth_reference_faithful"):
        required["eth_adaptive.csv"] = (
            "snapshot_time",
            "snapshot_version",
            "pi_hat_m",
            "pi_hat_hx", "pi_hat_hy", "pi_hat_hz",
            "pi_hat_jxx", "pi_hat_jxy", "pi_hat_jxz",
            "pi_hat_jyy", "pi_hat_jyz", "pi_hat_jzz",
            "pi_hat_fx_base", "pi_hat_fy_base", "pi_hat_fz_base",
            "pi_hat_tx_base", "pi_hat_ty_base", "pi_hat_tz_base",
            "estimated_yupi_world_com_fx",
            "estimated_yupi_world_com_fy",
            "estimated_yupi_world_com_fz",
            "estimated_yupi_world_com_tx",
            "estimated_yupi_world_com_ty",
            "estimated_yupi_world_com_tz",
        )
    elif manifest["method"] in ADAPTIVE_METHODS:
        diagnostic_name = adaptive_record_name(manifest["method"])
        method_required = (V5_DIAGNOSTIC_REQUIRED +
                            (("v6_enabled", "v6_rbf_enabled") if manifest["method"] == "v6_variable_gain"
                             else ("v5_enabled", "v5_rbf_enabled")) +
                            (tuple(prefix + "_" + axis for prefix in
                                   ("gain_evidence", "gain_smoothed_evidence", "gain_multiplier", "effective_gamma")
                                   for axis in WRENCH_FIELDS)
                             if manifest["method"] == "v6_variable_gain" else ())
                            if manifest["method"] in ("v5_residual_dynamics", "v6_variable_gain") else
                           V3_DIAGNOSTIC_REQUIRED
                           if manifest["method"] == "v3_teacher" else
                           V4_DIAGNOSTIC_REQUIRED
                           if manifest["method"] == "v4_convergence" else ())
        shared_required = (() if manifest["method"] in ("v5_residual_dynamics", "v6_variable_gain")
                           else DIAGNOSTIC_REQUIRED)
        required[diagnostic_name] = shared_required + ADAPTIVE_EXTERNAL_FIELDS + method_required
        if manifest["method"] == "v11_equivalent_wrench":
            required[diagnostic_name] = ("diagnostic_sequence","diagnostic_publish_drops","v11_enabled",
                "actual_update_dt","prediction_valid","learning_enabled","mpc_snapshot_id","wbc_snapshot_id",
                "clf_ec_value","clf_ec_violation","clf_ec_slack","wbc_status") + tuple(
                    group+"_"+axis for group in ("ee","er","ec","equivalent_wrench_body","reconstructed_reference_body")
                    for axis in ("x","y","z","rx","ry","rz"))
    elif manifest["method"] != "pure_mpc":
        required["shadow.csv"] = SHADOW_ESTIMATE_FIELDS
    shared_start = None
    diagnostic_summary = None
    for name, values in required.items():
        try:
            fields, rows = read_rows(path / name)
            needed = ("campaign_start_ros", "wall_elapsed") + values
            missing = set(needed).difference(fields)
            if missing or len(rows) < 2:
                raise ValueError("missing fields %s or fewer than two samples" % sorted(missing))
            columns = {field: numbers(rows, field) for field in needed}
            if any(not math.isfinite(v) for data in columns.values() for v in data):
                raise ValueError("nonfinite values in required fields")
            starts = columns["campaign_start_ros"]
            if shared_start is None:
                shared_start = starts[0]
            if any(abs(s - shared_start) > 1e-8 for s in starts):
                errors.append(name + ": campaign start differs across samples/streams")
            times = columns["wall_elapsed"]
            # Gazebo /clock has millisecond resolution while the controller can
            # publish more than once at one displayed clock tick. Diagnostic
            # sequence numbers preserve that within-tick order.
            time_reversed = any(b < a for a, b in zip(times, times[1:]))
            if time_reversed:
                errors.append(name + ": timestamps are not monotonically ordered")
            if "ros_time" in fields:
                if any(abs(float(r["ros_time"]) - float(r["campaign_start_ros"]) - float(r["wall_elapsed"])) > 1e-6 for r in rows):
                    errors.append(name + ": ROS timestamps disagree with elapsed time")
            # The force recorder intentionally logs its one-second cleanup tail.
            in_window = [r for r in rows if 0 <= float(r["wall_elapsed"]) < duration]
            elapsed = [float(r["wall_elapsed"]) for r in in_window]
            if not elapsed:
                raise ValueError("empty experiment window")
            gap = max((b - a for a, b in zip(elapsed, elapsed[1:])), default=duration)
            spans[name] = {"first_s": elapsed[0], "last_s": elapsed[-1], "max_gap_s": gap, "rows": len(rows)}
            if name == "wrench.csv":
                rate_hz = float(disturbance.get("rate_hz", 50.0))
                hold_s = float(disturbance.get("command_hold_s", 2.0 / rate_hz))
                tolerance = hold_s + 0.01
            else:
                # CPU-heavy Gazebo solves can occasionally delay a sampled
                # state/shadow callback by up to about 0.16 s.  A 0.20 s gate
                # still rejects real gaps while retaining those intact runs.
                tolerance = 0.20
            # ROS subscribers can join just after the shared start event. Allow
            # at most 0.20 s at startup while retaining the stricter per-stream
            # tolerance for end coverage and any gap during the experiment.
            start_tolerance = max(tolerance, 0.20)
            if elapsed[0] > start_tolerance or elapsed[-1] < duration - tolerance or gap > tolerance:
                errors.append(name + ": incomplete time coverage or a recording gap")
            streams[name] = rows
            if name == "wrench.csv":
                if (disturbance.get("reference_point_source") != "gazebo_world_net_wrench_zero_offset" or
                        any(abs(float(r["reference_point_"+a])) > 1e-12
                            for r in in_window for a in ("x", "y", "z"))):
                    errors.append("wrench.csv: nonzero/legacy world reference point adds an unrequested moment")
                for row in in_window:
                    elapsed_row = float(row["wall_elapsed"])
                    active = (
                        disturbance["profile"] != "none"
                        and wrench_delay <= elapsed_row
                        < wrench_delay + wrench_duration
                    )
                    expected_call = 1 if active else 0

                    if (float(row["service_called"]) != expected_call or
                            any(float(row[k]) != 1
                                for k in ("clear_success", "service_success"))):
                        errors.append(
                            "wrench.csv: apply/clear status differs from frozen conditions"
                        )
                        break
                if any(any(abs(float(r[axis]) - (expected_force(disturbance, float(r["wall_elapsed"]))
                                                     if axis == disturbance["axis"] else 0.0)) > 0.05
                           for axis in WRENCH_FIELDS) for r in in_window):
                    errors.append("wrench.csv: recorded six-axis wrench differs from frozen conditions")
            if name == diagnostic_name and manifest["method"] in ("v5_residual_dynamics", "v6_variable_gain"):
                is_v6 = manifest["method"] == "v6_variable_gain"
                prefix = "v6" if is_v6 else "v5"
                sequences = numbers(rows, "diagnostic_sequence")
                drops = numbers(rows, "diagnostic_publish_drops")
                if any(v < 0 or v != int(v) or v >= 2**53
                       for v in sequences + drops):
                    errors.append("v5 diagnostic counters are invalid")
                if any(b < a for a, b in zip(drops, drops[1:])):
                    errors.append("v5 diagnostic publish-drop counter reset")
                if any(b - a != 1 + (db - da) or b <= a
                       for a, b, da, db in
                       zip(sequences, sequences[1:], drops, drops[1:])):
                    errors.append("v5 diagnostic sequence gap is not accounted")
                if any(float(r[prefix + "_enabled"]) != 1 for r in rows):
                    errors.append(prefix + " was not enabled for every sample")
                if "variant" in manifest:
                    expected_rbf = 0 if manifest["variant"] == "without_rbf" else 1
                    if prefix + "_rbf_enabled" not in fields or any(
                            float(r[prefix + "_rbf_enabled"]) != expected_rbf for r in rows):
                        errors.append("v5 RBF switch differs from the frozen variant")
                    if expected_rbf == 0 and any(
                            abs(float(r["correction_acceleration_" + axis])) > 1e-10
                            for r in rows for axis in WRENCH_FIELDS):
                        errors.append("V5 no-RBF ablation produced nonzero RBF correction")
                if any(float(r["actual_update_dt"]) <= 0 for r in rows):
                    errors.append("v5 update period must stay positive")
                if is_v6:
                    import yaml
                    config = yaml.safe_load((path / "controller_config.yaml").read_text())
                    for index, axis in enumerate(WRENCH_FIELDS):
                        limit = float(config["v6_gain_multiplier_max"][index])
                        base = float(config["v5_gamma"][index])
                        for row in rows:
                            multiplier = float(row["gain_multiplier_" + axis])
                            effective = float(row["effective_gamma_" + axis])
                            if not 1.0 <= multiplier <= limit + 1e-9:
                                errors.append("V6 learning multiplier left its configured bounds")
                                break
                            if abs(effective - base * multiplier) > 1e-8:
                                errors.append("V6 effective gain disagrees with multiplier")
                                break
                diagnostic_summary = {
                    "samples": len(rows),
                    "diagnostic_sequence_first": sequences[0],
                    "diagnostic_sequence_last": sequences[-1],
                    "publish_drops_in_recorded_window": drops[-1] - drops[0],
                    "interpretation": prefix.upper() + " learns measured physical velocity-rate minus nominal model velocity-rate.",
                }
            if name == diagnostic_name and manifest["method"] == "v11_equivalent_wrench":
                if any(float(r["mpc_snapshot_id"])!=float(r["wbc_snapshot_id"]) for r in rows):
                    errors.append("V1.1 MPC/WBC snapshot mismatch")
                if any(float(r["wbc_status"])!=2 or float(r["v11_enabled"])!=1 for r in rows):
                    errors.append("V1.1 disabled or WBC failed")
                if not any(float(r["learning_enabled"])==1 for r in rows):
                    errors.append("V1.1 learning never activated")
                soft_clf = manifest.get("method_definition", {}).get("clf_mode") == "soft_squared_hinge"
                if soft_clf and any(abs(float(r["clf_ec_slack"])-max(0.,-float(r["clf_ec_value"])))>1e-8 for r in rows):
                    errors.append("V1.1 soft CLF slack disagrees with recorded violation")
                if not soft_clf and any(float(r["clf_ec_slack"])!=0 for r in rows):
                    errors.append("V1.1 hard CLF unexpectedly has slack")
                diagnostic_summary={"samples":len(rows),"maximum_clf_ec_violation":max(numbers(rows,"clf_ec_violation")),
                    "interpretation":"BODY/CoM left-hand equivalent uncertainty; reconstructed reference is not applied-force truth."}
                schema=json.loads((path / Path(name).with_suffix(".schema.json")).read_text())
                status=json.loads((path / Path(name).with_suffix(".status.json")).read_text())
                if fields!=list(schema["recorder_prefix"])+list(schema["columns"]) or status.get("errors") or status.get("rows")!=len(rows):
                    errors.append("V1.1 record/schema mismatch")
            if name == diagnostic_name and manifest["method"] not in ("v5_residual_dynamics", "v6_variable_gain", "v11_equivalent_wrench"):
                sequences = numbers(rows, "diagnostic_sequence")
                drops = numbers(rows, "diagnostic_publish_drops")
                if any(v < 0 or v != int(v) or v >= 2**53 for v in sequences + drops):
                    errors.append("v2 diagnostic counters must be exact nonnegative integers below 2^53")
                if any(b < a for a, b in zip(drops, drops[1:])):
                    errors.append("v2 diagnostic publish-drop counter reset")
                # The real-time publisher explicitly counts every unsuccessful
                # trylock. Accept only sequence gaps explained exactly by that
                # counter; an unaccounted subscriber/recorder loss still fails.
                if any(b - a != 1 + (db - da) or b <= a
                       for a, b, da, db in zip(sequences, sequences[1:], drops, drops[1:])):
                    errors.append("v2 diagnostic sequence gap is not exactly accounted by the publish-drop counter")
                if any(float(r["reference_wrench_is_final"]) != 1 for r in rows):
                    errors.append("v2 diagnostic flag is not valid for every sample: reference_wrench_is_final")
                acceleration_flags = numbers(rows, "desired_acceleration_initialized")
                if any(v not in (0, 1) for v in acceleration_flags):
                    errors.append("desired acceleration initialization flag is not binary")
                for i in range(1, len(rows)):
                    # With no omitted publication, the first frame of a newly
                    # frozen MPC snapshot must reset the finite difference and
                    # every later frame in that same snapshot must initialize it.
                    if sequences[i] - sequences[i - 1] == 1:
                        expected = 1.0 if float(rows[i]["mpc_snapshot_id"]) == float(rows[i - 1]["mpc_snapshot_id"]) else 0.0
                        if acceleration_flags[i] != expected:
                            errors.append("desired acceleration initialization disagrees with MPC snapshot transition")
                            break
                expected_mode = 1 if manifest["robust_mode"] == "sign" else 0
                if any(float(r["robust_mode"]) != expected_mode for r in rows):
                    errors.append("v2 diagnostic robust mode differs from requested sign/smooth mode")
                if manifest["method"] == "v3_teacher":
                    if any(float(r["v3_teacher_enabled"]) != 1 for r in rows):
                        errors.append("v3 teacher mode was not enabled for every sample")
                    if any(float(r["v3_previous_policy_valid"]) not in (0, 1) for r in rows):
                        errors.append("v3 previous-policy validity flag is not binary")
                    if any(not 0 < float(r["v3_composite_ratio"]) < 1 for r in rows):
                        errors.append("v3 composite ratio must remain strictly between zero and one")
                    if any(float(r["v3_robot_mass"]) <= 0 for r in rows):
                        errors.append("v3 robot mass must remain positive")
                    valid_updates = [r for r in rows
                                     if float(r["v3_previous_policy_valid"]) == 1]
                    if not valid_updates:
                        errors.append("v3 never obtained a valid previous-policy teacher sample")
                    elif any(float(r["v3_actual_update_dt"]) <= 0 for r in valid_updates):
                        errors.append("v3 valid updates must record a positive actual update period")
                elif manifest["method"] == "v4_convergence":
                    if any(float(r["v4_convergence_enabled"]) != 1 for r in rows):
                        errors.append(
                            "v4 convergence mode was not enabled for every sample")
                diagnostic_summary = {
                    "samples": len(rows),
                    "infeasible_samples": sum(float(r["clf_feasible"]) != 1 for r in rows),
                    "snapshot_mismatch_samples": sum(float(r["mpc_snapshot_id"]) != float(r["wbc_snapshot_id"]) for r in rows),
                    "maximum_clf_violation": max(numbers(rows, "clf_violation")),
                    "maximum_absolute_hold_mismatch_power": max(abs(v) for v in numbers(rows, "hold_mismatch_power")),
                    "diagnostic_sequence_first": sequences[0], "diagnostic_sequence_last": sequences[-1],
                    "publish_drops_before_first_sample": drops[0], "publish_drops_last": drops[-1],
                    "publish_drops_in_recorded_window": drops[-1] - drops[0],
                    "desired_acceleration_reset_samples": sum(v == 0 for v in acceleration_flags),
                    "interpretation": "ID equality only checks snapshot binding, not proof validity. Finite violations/infeasibility and hold mismatch remain measured outcomes.",
                }
                for row in rows:
                    if any(not math.isfinite(float(v)) for v in row.values()):
                        errors.append(name + ": nonfinite diagnostic value")
                        break
                schema = json.loads((path / Path(name).with_suffix(".schema.json")).read_text())
                if fields != list(schema["recorder_prefix"]) + list(schema["columns"]):
                    errors.append("v2 diagnostic CSV disagrees with controller-published schema")
                status = json.loads((path / Path(name).with_suffix(".status.json")).read_text())
                if status.get("errors") or status.get("rows") != len(rows):
                    errors.append("v2 diagnostic recorder reported errors or row count mismatch")
        except (OSError, ValueError, KeyError, TypeError) as error:
            errors.append(name + ": " + str(error))
    if "state.csv" in streams:
        state_window = [r for r in streams["state.csv"]
                        if 0 <= float(r["wall_elapsed"]) < duration]
        if state_window:
            minimum_height = min(float(r["observation_base_z"])
                                 for r in state_window)
            maximum_tilt = max(max(abs(float(r["observation_base_pitch"])),
                                   abs(float(r["observation_base_roll"])))
                               for r in state_window)
            if minimum_height < 0.25:
                errors.append("state.csv: base height below 0.25 m (robot fell)")
            if maximum_tilt > 0.80:
                errors.append("state.csv: body tilt above 0.80 rad (robot fell)")
    if manifest["terrain_conditions"]["motion"] and manifest["planner"].get("source") == "cmd_vel":
        try:
            validate_velocity_reference(path, manifest, shared_start, streams.get("state.csv", []))
        except (OSError, ValueError, KeyError, TypeError) as error:
            errors.append("velocity reference: " + str(error))
    elif manifest["terrain_conditions"]["motion"]:
        try:
            goal = json.loads((path / "goal.json").read_text())
            if goal.get("source") not in ("tp01_quintic_planner", "go2_reference_trajectory"):
                raise ValueError("goal provenance is not a recorded Go2 reference trajectory")
            _, planner = read_rows(path / "planner_reference.csv")
            _, received = read_rows(path / "planner_received.csv")
            _, mpc = read_rows(path / "mpc_target.csv")
            if not planner or not received or not mpc:
                raise ValueError("missing published/received planner or MPC target samples")
            # Check a full actually received message, not a made-up reference.
            for rows, name in ((received, "planner_received"), (mpc, "mpc_target")):
                message = [r for r in rows if r["message_index"] == rows[-1]["message_index"]]
                if len(message) != len(planner):
                    raise ValueError(name + ": received knot count differs from planner")
                keys = ("trajectory_time_s", "x_m", "y_m", "z_m", "yaw_rad", "pitch_rad", "roll_rad")
                if manifest.get("planner", {}).get("reference_profile") == "smooth_accel_cruise_brake":
                    keys += LINEAR_TARGET_RATE_FIELDS
                for sent, got in zip(planner, message):
                    for key in keys:
                        if abs(float(sent[key]) - float(got[key])) > 1e-6:
                            raise ValueError(name + ": received target differs from planner")
            if abs(float(goal["campaign_start_ros"]) - shared_start) > 1e-8:
                raise ValueError("goal campaign clock differs from state clock")
        except (OSError, ValueError, KeyError, TypeError) as error:
            errors.append("planner: " + str(error))
    if require_artifacts:
        for name in ("controller_config.yaml", "effective_rosparams.yaml", "run.log", "raw.bag",
                     "recording_finished.json",
                     "frozen/source_sha256.json"):
            file = path / name
            if not file.is_file() or file.stat().st_size == 0:
                errors.append("missing or empty artifact: " + name)
        try:
            digest = hashlib.sha256((path / "controller_config.yaml").read_bytes()).hexdigest()
            if digest != manifest["controller_config_sha256"]:
                errors.append("frozen controller config hash mismatch")
            scene_files = [path / name for name in ("gazebo_scene.png", "gazebo_scene.bag", "gazebo_scene.json")]
            if all(file.is_file() and file.stat().st_size > 0 for file in scene_files):
                scene = json.loads((path / "gazebo_scene.json").read_text())
                if not scene["go2_spawn_verified"] or scene["stamp_ros"] <= shared_start:
                    errors.append("camera image was not captured after campaign start and spawn")
                if (path / "gazebo_scene.png").read_bytes()[:8] != b"\x89PNG\r\n\x1a\n":
                    errors.append("scene image is not a PNG")
            else:
                unavailable = json.loads((path / "gazebo_scene_unavailable.json").read_text())
                if not unavailable.get("camera_optional") or not unavailable.get("go2_spawn_verified"):
                    errors.append("missing both valid optional camera evidence and explicit absence record")
        except (OSError, ValueError, KeyError, TypeError) as error:
            errors.append("artifact validation: " + str(error))
    if manifest.get("source_drift"):
        errors.append("source changed during recording")
    if manifest.get("status") == "failed":
        errors.append("runner reported failure: " + manifest.get("error", "unknown"))
    return {"valid": not errors, "errors": errors, "spans": spans, "diagnostic_summary": diagnostic_summary,
            "campaign_start_ros": shared_start,
            "note": "Validation includes recording integrity and basic fall detection; it is not a theorem-verification claim."}


def load_case(path, allow_legacy=False):
    path = Path(path)
    manifest = json.loads((path / "case_manifest.json").read_text(encoding="utf-8"))
    validation = validate_case(path, allow_legacy=allow_legacy)
    data = {}
    for name in ("state", "wrench", "shadow", "v3_teacher", "v4_convergence",
                 "v5_residual_dynamics", "v6_variable_gain", "v11_equivalent_wrench", "planner_reference", "mpc_target"):
        source = path / (name + ".csv")
        if source.is_file():
            fields, rows = read_rows(source)
            if name == "v11_equivalent_wrench":
                # validate_case already checks the complete raw recording.
                # Retain every sample, but not hundreds of unused weight/feature
                # columns for all 16 cases simultaneously during plotting.
                keep = {"wall_elapsed", "prediction_valid"} | {
                    prefix + "_" + axis
                    for prefix in ("equivalent_wrench_body", "reconstructed_reference_body")
                    for axis in ("x", "y", "z", "rx", "ry", "rz")}
                fields = [f for f in fields if f in keep]
                rows = [{f: row[f] for f in fields} for row in rows]
            data[name] = {"fields": fields, "rows": rows}
    return dict(path=path, manifest=manifest, validation=validation, data=data)


def label(case):
    m = case["manifest"]
    name = METHOD_LABELS[m["method"]]
    if m.get("variant") == "without_rbf":
        name += " (RBF off)"
    if m["method"] in ADAPTIVE_METHODS and m["robust_mode"] not in ("smooth", "none"):
        name += " (" + m["robust_mode"] + ")"
    if not case["validation"]["valid"]:
        name += " [INVALID RECORDING]"
    return name


def line_style(case):
    return "--" if case["manifest"].get("variant") == "without_rbf" else LINESTYLES[case["manifest"]["method"]]


def comparison_key(case):
    m = case["manifest"]
    # All substantive conditions, including repetition/seed, must match.
    key = {k: m[k] for k in ("terrain", "terrain_conditions", "disturbance", "timing_s", "planner", "gait", "seed", "repetition", "physics_step_s", "data_kind")}
    key["study"] = m.get("study", "main")
    return json.dumps(key, sort_keys=True)


FAILURE_BANNER = "FAILED/INCOMPLETE RUN — not a valid performance comparison"


def failed_group(cases):
    return any(not c["validation"]["valid"] for c in cases)


def condition_text(cases):
    m = cases[0]["manifest"]
    d = m["disturbance"]
    if d["profile"] == "none":
        experiment = "E00 no external force"
    elif d["axis"] == "tx":
        experiment = "M01 sine Tx"
    else:
        experiment = {"constant": "E01 constant Fy",
                      "sine": "E02 sine Fy",
                      "multisine": "E03 combined sine Fy"}[d["profile"]]
    trajectory_id = m.get("planner", {}).get("trajectory_id", "t01").upper()
    distance = float(m.get("planner", {}).get("relative_dx_m") or 0.0)
    move_time = float(m["timing_s"]["trajectory"])
    condition = "%s %.2g m / %.0f s | %s" % (trajectory_id, distance, move_time, experiment)
    if m.get("planner", {}).get("source") == "cmd_vel":
        condition = "Forward %.3g m/s (body) | %.0f s | %s" % (
            m["planner"]["command_vx_mps"], move_time, experiment)
    if d["profile"] == "none":
        return condition
    peak = abs(float(d["value"])) if d["profile"] == "constant" else (
        abs(float(d["value"])) + abs(float(d["amplitude"])) +
        (abs(float(d["secondary_amplitude"])) if d["profile"] == "multisine" else 0.0))
    return "%s (amplitude bound %g %s)" % (
        condition, peak, "Nm" if d["axis"] == "tx" else "N")


def short_method(case):
    m = case["manifest"]
    if m["method"] == "v11_equivalent_wrench":
        return "MyRBF V1.1"
    if m["method"] in ADAPTIVE_METHODS and m["robust_mode"] != "smooth":
        prefix = {"v3_teacher": "MyRBF V3 (archived)-",
                  "v4_convergence": "MyRBF V4-",
                   "v5_residual_dynamics": "MyRBF V5-",
                   "v6_variable_gain": "MyRBF V6-"}[m["method"]]
        return prefix + m["robust_mode"]
    return METHOD_LABELS[m["method"]]


def displayed_time_window(cases):
    """Use the complete commanded-motion interval for time-series comparison."""
    timings = [c["manifest"]["timing_s"] for c in cases]
    starts = {float(t["goal_delay"]) for t in timings}
    ends = {float(t["goal_delay"]) + float(t["trajectory"]) +
            float(t.get("final_hold", 0.0)) for t in timings}
    moving = all(c["manifest"]["terrain_conditions"]["motion"] for c in cases)
    if moving and len(starts) == 1 and len(ends) == 1:
        return starts.pop(), ends.pop()
    return 0.0, max(float(t["total"]) for t in timings)


def save_figure(fig, output, stem, cases, dpi):
    import matplotlib.pyplot as plt
    if failed_group(cases):
        # Outside the subplot/title area; bbox_inches='tight' retains this banner.
        fig.text(.5, 1.02, FAILURE_BANNER, ha="center", va="bottom", fontsize=14,
                 weight="bold", color="#A61B1B",
                 bbox=dict(facecolor="#FFF0F0", edgecolor="#A61B1B", pad=8))
    synthetic = any(c["manifest"].get("data_kind") != "simulation" for c in cases)
    if synthetic:
        stem = "synthetic_" + stem
        from matplotlib.font_manager import FontProperties
        candidates = (Path("/mnt/c/Windows/Fonts/msyh.ttc"), Path("C:/Windows/Fonts/msyh.ttc"))
        font = next((FontProperties(fname=str(p)) for p in candidates if p.exists()), None)
        text = "SYNTHETIC TEST DATA / planning only [待真实实验替换]" if font else "SYNTHETIC TEST DATA / planning only [REPLACE WITH REAL EXPERIMENTS]"
        font_kwargs = {"fontproperties": font} if font is not None else {}
        fig.text(0.5, -0.035, text, ha="center", va="top", color="#CC3311", **font_kwargs)
    for suffix in ("png", "svg"):
        fig.savefig(output / (stem + "." + suffix), dpi=dpi, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    conditions = [
        {"source_directory": str(c["path"]), "manifest": c["manifest"], "recording_validation": c["validation"],
         "controller_config": (c["path"] / "controller_config.yaml").read_text() if (c["path"] / "controller_config.yaml").exists() else None}
        for c in cases]
    serialized = json.dumps(conditions, indent=2)
    (output / (stem + ".conditions.json")).write_text(serialized + "\n", encoding="utf-8")
    (output / (stem + ".conditions.md")).write_text(
        "# Full figure conditions\n\n" + (FAILURE_BANNER + "\n\n" if failed_group(cases) else "") +
        "Exactly two figure types: [CoM velocity WORLD, angular velocity BODY], and BODY/CoM left-hand equivalent uncertainty [force, moment]. Physical velocities use recorded normalized momentum and the frozen nominal model, not pose derivatives. Each dashed velocity target is the target received in that method's run (matching color). V1.1 plots the learned product d=Y*pi, not its weights. Comparator compensation outputs are translated/rotated from their existing interface; this does not assert identical learning objectives. The grey V1.1 reference is reconstructed from nominal dynamics and measured velocity changes, not independent ground truth or the commanded external force. No position/path plots or smoothing.\n\n" +
        "Complete manifests, hashes, validation and frozen controller configurations:\n\n```json\n" +
        serialized + "\n```\n", encoding="utf-8")
    return stem


def planner_on_state(case):
    """Use actual received target knots in observation time, never endpoint-fill.

    Latest message available at each state's ROS time is selected causally. No
    line or error is invented before receipt, or outside the supplied knot range.
    """
    import numpy as np
    state = case["data"]["state"]["rows"]
    messages = {}
    for row in case["data"].get("mpc_target", {}).get("rows", []):
        messages.setdefault(row["message_index"], []).append(row)
    ref = {axis: np.full(len(state), np.nan) for axis in REFERENCE_FIELDS}
    for i, sample in enumerate(state):
        available = [knots for knots in messages.values() if float(knots[0]["received_ros_time"]) <= float(sample["ros_time"])]
        if not available:
            continue
        knots = max(available, key=lambda k: float(k[0]["received_ros_time"]))
        t = np.array(numbers(knots, "trajectory_time_s"))
        query = float(sample["observation_time"])
        if np.any(np.diff(t) <= 0) or query < t[0] or query > t[-1]:
            continue
        for axis in ref:
            if axis in knots[0]:
                ref[axis][i] = np.interp(query, t, numbers(knots, axis))
    return ref

def tracking_display_series(actual, reference, origin):
    """Show planar pose relative to the commanded start; keep height/tilt absolute."""
    import numpy as np
    if len(actual) != 6 or len(reference) != 6 or len(origin) != 6:
        raise ValueError("tracking display requires six state and reference axes")
    shown_actual = [np.asarray(values, dtype=float).copy() for values in actual]
    shown_reference = [np.asarray(values, dtype=float).copy() for values in reference]
    for index in (0, 1, 5):
        shown_actual[index] -= origin[index]
        shown_reference[index] -= origin[index]
    for values in shown_actual[3:] + shown_reference[3:]:
        finite = np.isfinite(values)
        values[finite] = (values[finite] + np.pi) % (2 * np.pi) - np.pi
    return shown_actual, shown_reference


def derivative_on_support(time_values, values, angular=False):
    """Differentiate recorded values without filling missing target support."""
    import numpy as np
    time_values = np.asarray(time_values, dtype=float)
    values = np.asarray(values, dtype=float)
    result = np.full(values.shape, np.nan, dtype=float)
    valid_indices = np.flatnonzero(np.isfinite(time_values) & np.isfinite(values))
    if valid_indices.size < 2:
        return result
    splits = np.split(valid_indices, np.where(np.diff(valid_indices) != 1)[0] + 1)
    for segment in splits:
        if segment.size < 2:
            continue
        t = time_values[segment]
        if np.any(np.diff(t) <= 0):
            continue
        v = values[segment]
        if angular:
            v = np.unwrap(v)
        result[segment] = np.gradient(v, t)
    return result


def path_tangent_heading(vx, vy, target_yaw):
    """World-frame target tangent; hold its direction at stationary endpoints."""
    import numpy as np
    vx, vy, target_yaw = (np.asarray(v, dtype=float) for v in (vx, vy, target_yaw))
    valid = np.isfinite(vx) & np.isfinite(vy) & (np.hypot(vx, vy) > 1e-4)
    heading = np.full(vx.shape, np.nan, dtype=float)
    indices = np.flatnonzero(valid)
    if indices.size:
        heading = np.interp(np.arange(vx.size), indices,
                            np.unwrap(np.arctan2(vy[indices], vx[indices])))
    else:
        heading[:] = target_yaw
    heading[~np.isfinite(target_yaw)] = np.nan
    return heading


def attitude_error_angle(actual_rpy, target_rpy):
    """Shortest rotation angle between recorded and target Rz(yaw)Ry(pitch)Rx(roll)."""
    import numpy as np

    def quaternion(rpy):
        roll, pitch, yaw = [np.asarray(v, dtype=float) / 2 for v in rpy]
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        return np.stack((cr * cp * cy + sr * sp * sy,
                         sr * cp * cy - cr * sp * sy,
                         cr * sp * cy + sr * cp * sy,
                         cr * cp * sy - sr * sp * cy), axis=0)

    dot = np.sum(quaternion(actual_rpy) * quaternion(target_rpy), axis=0)
    return 2 * np.arccos(np.clip(np.abs(dot), 0.0, 1.0))


def planned_origin_in_display_order(case):
    origin = [0.0] * 6
    goal_path = case["path"] / "goal.json"
    if goal_path.exists():
        stored = json.loads(goal_path.read_text())["planned_start_pose"]
        origin = [stored[0], stored[1], stored[2], stored[5], stored[4], stored[3]]
    return origin


def wrench_display_grid(duration):
    """Return a shared 20 Hz time grid over the recorded interval."""
    import numpy as np
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("wrench display duration must be positive and finite")
    sample_count = int(math.ceil(duration * WRENCH_DISPLAY_RATE_HZ))
    return np.arange(sample_count, dtype=float) / WRENCH_DISPLAY_RATE_HZ


def wrench_on_display_grid(time_values, values, grid):
    """Interpolate recorded samples without filtering or extrapolation."""
    import numpy as np
    time_values = np.asarray(time_values, dtype=float)
    values = np.asarray(values, dtype=float)
    if time_values.size == 0 or values.size == 0:
        return np.full(np.asarray(grid).shape, np.nan, dtype=float)
    return np.interp(grid, time_values, values, left=np.nan, right=np.nan)


def rotation_zyx(angles):
    """R_WB for rows [yaw,pitch,roll]."""
    import numpy as np
    yaw,pitch,roll=np.asarray(angles).T
    cz,sz,cy,sy,cx,sx=np.cos(yaw),np.sin(yaw),np.cos(pitch),np.sin(pitch),np.cos(roll),np.sin(roll)
    return np.stack((cy*cz,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx,
                     cy*sz,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx,
                     -sy,cy*sx,cy*cx),axis=1).reshape(-1,3,3)


def nominal_model(case):
    import numpy as np
    import yaml
    model=yaml.safe_load((case["path"]/"effective_rosparams.yaml").read_text())["go2_nominal_model"]
    if int(model["type"]) != 1:
        raise ValueError("Physical velocity conversion requires the recorded SRBD model")
    mass=float(model["mass"])
    inertia=np.asarray(model["inertia_body_row_major"],dtype=float).reshape(3,3)
    com_to_base=np.asarray(model["com_to_base_body"],dtype=float)
    if mass<=0 or not np.all(np.isfinite(inertia)) or np.min(np.linalg.eigvalsh(inertia))<=0:
        raise ValueError("Invalid frozen nominal mass/inertia")
    return mass,inertia,com_to_base


def physical_velocity(momentum, rotations, mass, inertia):
    """[v_CoM,W; omega_B] from recorded normalized centroidal momentum."""
    import numpy as np
    momentum=np.asarray(momentum,dtype=float)
    angular_body=np.einsum("nji,nj->ni",rotations,momentum[:,3:])*mass
    return np.column_stack((momentum[:,:3],np.linalg.solve(inertia,angular_body.T).T))


def plot_group(cases, output, stem, dpi=300):
    """Exactly two figures: physical six-axis velocity and equivalent uncertainty."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.lines import Line2D
    current=("pure_mpc","v11_equivalent_wrench","direct_wrench","eth","liu_van_mpc_port","zhou_ssi")
    if any(c["manifest"]["method"] not in current for c in cases):
        raise ValueError("Archived methods require their archived plotting script")
    plt.rcParams.update({"font.size":10,"axes.titlesize":11,
                         "axes.spines.top":False,"axes.spines.right":False,
                         "axes.grid":True,"grid.alpha":.2,"legend.frameon":False})
    prepared,metrics=[],[]
    angular_target_fields=tuple("normalized_angular_momentum_"+a for a in ("x","y","z"))
    for case in cases:
        m=case["manifest"]; rows=case["data"]["state"]["rows"]
        t=np.asarray(numbers(rows,"wall_elapsed"))
        mass,inertia,com_to_base=nominal_model(case)
        angles=np.column_stack([numbers(rows,"observation_base_"+a) for a in ("yaw","pitch","roll")])
        momentum=np.column_stack([numbers(rows,"observation_normalized_momentum_"+str(i)) for i in range(6)])
        actual=physical_velocity(momentum,rotation_zyx(angles),mass,inertia)
        target=planner_on_state(case)
        target_angles=np.column_stack([target[a+"_rad"] for a in ("yaw","pitch","roll")])
        target_momentum=np.column_stack([target[a] for a in LINEAR_TARGET_RATE_FIELDS+angular_target_fields])
        reference=physical_velocity(target_momentum,rotation_zyx(target_angles),mass,inertia)
        metric={"case":case["path"].name,"method":m["method"],"study":m.get("study","main"),
                "trajectory_id":m["planner"]["trajectory_id"],"profile":m["disturbance"]["profile"],
                "recording_valid":case["validation"]["valid"],"samples":len(rows)}
        for i,a in enumerate(("vx","vy","vz","omega_x","omega_y","omega_z")):
            error=actual[:,i]-reference[:,i]; error=error[np.isfinite(error)]
            if not error.size:
                raise ValueError("No recorded target support for "+a)
            metric[a+"_max_abs_error"]=float(np.max(np.abs(error)))
            metric[a+"_final_abs_error"]=float(abs(error[-1]))
            metric[a+"_rmse"]=float(np.sqrt(np.mean(error**2)))
        if m["method"]=="v11_equivalent_wrench":
            drows=case["data"]["v11_equivalent_wrench"]["rows"]
            dt=np.asarray(numbers(drows,"wall_elapsed"))
            axes=("x","y","z","rx","ry","rz")
            estimate=np.column_stack([numbers(drows,"equivalent_wrench_body_"+a) for a in axes])
            reconstructed=np.column_stack([numbers(drows,"reconstructed_reference_body_"+a) for a in axes])
            reconstructed[np.asarray(numbers(drows,"prediction_valid"))!=1]=np.nan
        elif m["method"] != "pure_mpc":
            drows=case["data"]["shadow"]["rows"]
            dt=np.asarray(numbers(drows,"wall_elapsed"))
            # Existing comparator interface defines raw estimator outputs as
            # world-frame COMPENSATION wrenches about base, not external forces.
            # Translate to CoM and rotate; no sign inference from curve shape.
            dR=rotation_zyx(np.column_stack([numbers(drows,"actual_base_"+a+"_rad")
                                           for a in ("yaw","pitch","roll")]))
            raw=np.column_stack([numbers(drows,a) for a in SHADOW_ESTIMATE_FIELDS])
            force=np.einsum("nji,nj->ni",dR,raw[:,:3])
            moment=np.einsum("nji,nj->ni",dR,raw[:,3:])+np.cross(com_to_base,force)
            estimate=np.column_stack((force,moment));reconstructed=None
        else:
            # Pure MPC deliberately has no estimator or compensation wrench.
            # Leave this panel empty rather than inventing an all-zero estimate.
            dt=np.empty(0); estimate=np.empty((0,6)); reconstructed=None
        prepared.append(dict(case=case,t=t,actual=actual,target=reference,
                             dt=dt,estimate=estimate,reconstructed=reconstructed))
        metrics.append(metric)
    outputs=[]
    for kind in ("velocity_tracking_6d","equivalent_uncertainty_6d"):
        fig,axs=plt.subplots(2,3,figsize=(11.5,6.8),sharex=True)
        for item in prepared:
            method=item["case"]["manifest"]["method"];color=COLORS[method]
            for i,ax in enumerate(axs.flat):
                if kind=="velocity_tracking_6d":
                    ax.plot(item["t"],item["actual"][:,i],color=color,
                            lw=1.05,label=short_method(item["case"]))
                    ax.plot(item["t"],item["target"][:,i],color=color,ls="--",lw=1.7,zorder=4)
                else:
                    ax.plot(item["dt"],item["estimate"][:,i],color=color,
                            lw=1.15,label=short_method(item["case"]))
                    if item["reconstructed"] is not None:
                        ax.plot(item["dt"],item["reconstructed"][:,i],color=".45",ls=":",
                                lw=.65,alpha=.65,label="V1.1 reconstructed reference (not truth)")
        for i,ax in enumerate(axs.flat):
            if kind=="velocity_tracking_6d":
                ax.set_title(("vx (world)","vy (world)","vz (world)",
                              "omega x (body)","omega y (body)","omega z (body)")[i])
                ax.set_ylabel("m/s" if i<3 else "rad/s")
            else:
                ax.set_title(("Equivalent Fx","Equivalent Fy","Equivalent Fz",
                              "Equivalent Tx","Equivalent Ty","Equivalent Tz")[i])
                ax.set_ylabel("N" if i<3 else "Nm")
            ax.set_xlabel("Experiment time (s)")
        handles,labels=axs.flat[0].get_legend_handles_labels()
        if kind=="velocity_tracking_6d":
            handles.append(Line2D([],[],color="black",ls="--",lw=1.7))
            labels.append("Received target (matching color)")
        fig.legend(handles,labels,loc="upper center",bbox_to_anchor=(.5,.945),
                   ncol=min(3,len(labels)),fontsize=9)
        fig.suptitle(condition_text(cases),fontsize=11)
        fig.tight_layout(rect=(0,0,1,.85))
        outputs.append(save_figure(fig,output,stem+"_"+kind,cases,dpi))
    return outputs,metrics




def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument("--input",type=Path)
    source.add_argument("--case-index",type=Path)
    p.add_argument("--output",type=Path)
    p.add_argument("--validate-only",action="store_true")
    p.add_argument("--dpi",type=int,default=300)
    args=p.parse_args(argv)
    if args.case_index:
        paths=[Path(item["path"]) if Path(item["path"]).is_absolute()
               else args.case_index.parent / item["path"]
               for item in json.loads(args.case_index.read_text())["cases"]]
        input_root=args.case_index.parent
    else:
        paths=([args.input] if (args.input/"case_manifest.json").exists() else
               sorted(m.parent for m in args.input.glob("*/case_manifest.json")))
        input_root=args.input
    if not paths:
        p.error("No experiment recordings found")
    output=args.output or input_root/("figures_"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ"))
    output.mkdir(parents=True,exist_ok=False)
    cases,report=[],{}
    current={"pure_mpc","v11_equivalent_wrench","direct_wrench","eth","liu_van_mpc_port","zhou_ssi"}
    for path in paths:
        try:
            case=load_case(path)
            if case["manifest"]["method"] not in current:
                raise ValueError("Use the archived plotter for historical methods")
            nominal_model(case)
            required={"observation_normalized_momentum_"+str(i) for i in range(6)}
            if not required.issubset(case["data"]["state"]["fields"]):
                raise ValueError("Missing recorded momentum: cannot invent angular velocity from pose")
            target_fields={"normalized_angular_momentum_"+a for a in ("x","y","z")}
            if not target_fields.issubset(case["data"].get("mpc_target",{}).get("fields",[])):
                raise ValueError("Missing recorded angular-velocity reference")
            report[str(path)]=case["validation"]
            cases.append(case)
        except (OSError,ValueError,KeyError,TypeError) as error:
            report[str(path)]={"valid":False,"errors":[str(error)]}
    plan=input_root/"campaign_plan.json"
    if plan.exists():
        def identity(m):
            return (m.get("study","main"),m["method"],m.get("variant","standard"),
                    m["planner"]["trajectory_id"],m["terrain"],m["disturbance"]["profile"],
                    m["disturbance"]["axis"],m["seed"],m["repetition"])
        present={identity(c["manifest"]) for c in cases}
        missing=[identity(m) for m in json.loads(plan.read_text())["cases"] if identity(m) not in present]
        if missing:
            report["unrecorded_planned_cases"]={"valid":False,"errors":["Planned recordings are missing"],"missing":missing}
    groups={}
    for case in cases:
        if case["validation"]["valid"]:
            groups.setdefault(comparison_key(case),[]).append(case)
    lines=["# Figure data manifest","","Only physical six-axis velocity and equivalent uncertainty; raw recorded samples.","",
           "| Figure | Recorded source directories |","|---|---|"]
    metrics=[]
    if not args.validate_only:
        for key,group in groups.items():
            m=group[0]["manifest"]
            stem="%s_%s_%s_%s"%(m["terrain"],m["planner"]["trajectory_id"],
                                 m["disturbance"]["profile"],hashlib.sha256(key.encode()).hexdigest()[:8])
            try:
                stems,rows=plot_group(group,output,stem,args.dpi)
                metrics.extend(rows)
                for name in stems:
                    lines.append("| %s | %s |"%(name,"; ".join(str(c["path"]) for c in group)))
            except (ValueError,KeyError,TypeError) as error:
                report[stem]={"valid":False,"errors":[str(error)]}
        (output/"data-manifest.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
        (output/"metrics.json").write_text(json.dumps(metrics,indent=2,allow_nan=False)+"\n")
        if metrics:
            with (output/"metrics.csv").open("w",newline="",encoding="utf-8") as handle:
                writer=csv.DictWriter(handle,fieldnames=list(metrics[0]))
                writer.writeheader();writer.writerows(metrics)
    (output/"validation.json").write_text(json.dumps(report,indent=2)+"\n")
    print(output)
    return 0 if report and all(row["valid"] for row in report.values()) else 1


if __name__=="__main__":
    raise SystemExit(main())
