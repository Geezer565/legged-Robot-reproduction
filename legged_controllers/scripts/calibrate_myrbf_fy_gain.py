#!/usr/bin/env python3

"""Replay Go2 Fy shadow data to choose one balanced MyRBF learning gain."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


SCALE = 18.757520233566048
EXTENT = 2.0
WIDTH_RATIO = 0.85
LEAKAGE = 0.02
CURRENT_GAIN = 0.00096634
CANDIDATES = (
    0.00050, 0.00075, CURRENT_GAIN, 0.00125, 0.00150, 0.00200,
    0.00250, 0.00300, 0.00400, 0.00500, 0.00750, 0.01000,
    0.01500, 0.02000, 0.03000, 0.05000, 0.07500, 0.10000,
)


def read_csv(path):
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    return {key: np.asarray([float(row[key]) for row in rows]) for key in rows[0]}


def basis(feature):
    spacing = EXTENT / 2.0
    width = WIDTH_RATIO * spacing
    values = [1.0]
    for row in range(5):
        for column in range(5):
            cx = -EXTENT + spacing * column
            cy = -EXTENT + spacing * row
            dx, dy = feature[0] - cx, feature[1] - cy
            values.append(np.exp(-(dx * dx + dy * dy) / (2.0 * width * width)))
    return np.asarray(values)


def replay(filtered_residual, dt, gain):
    weights = np.zeros(26)
    previous_estimate = 0.0
    estimates = []
    for residual, step_dt in zip(filtered_residual, dt):
        feature = (residual / SCALE, (residual - previous_estimate) / SCALE)
        phi = basis(feature)
        estimate_before = SCALE * np.dot(weights, phi)
        gradient = SCALE * phi
        derivative = gain * (residual - estimate_before) * gradient - LEAKAGE * weights
        weights += step_dt * derivative
        previous_estimate = SCALE * np.dot(weights, phi)
        estimates.append(previous_estimate)
    return np.asarray(estimates)


def rms(values):
    return float(np.sqrt(np.mean(np.asarray(values) ** 2)))


def active_window(wrench):
    mask = (wrench["service_called"] > 0.5) & (wrench["service_success"] > 0.5)
    return float(np.min(wrench["ros_time"][mask])), float(np.max(wrench["ros_time"][mask]))


def evaluate_case(case_dir, gain):
    shadow = read_csv(case_dir / "shadow.csv")
    wrench = read_csv(case_dir / "wrench.csv")
    estimate = replay(shadow["filtered_fy_n"], shadow["dt_s"], gain)
    target = -np.interp(shadow["time_s"], wrench["ros_time"], wrench["fy"])
    start, stop = active_window(wrench)
    active = (shadow["time_s"] >= start) & (shadow["time_s"] <= stop)
    quiet = ((shadow["time_s"] >= start - 2.0) & (shadow["time_s"] < start)) | ((shadow["time_s"] > stop) & (shadow["time_s"] <= stop + 2.0))
    return {
        "active_rmse_n": rms(estimate[active] - target[active]),
        "quiet_rmse_n": rms(estimate[quiet] - target[quiet]),
        "estimate": estimate,
        "recorded": shadow["estimate_after_fy_n"],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    current_replay_error = {}
    for gain in CANDIDATES:
        constant = evaluate_case(args.root / "myrbf_constant", gain)
        sine = evaluate_case(args.root / "myrbf_sine", gain)
        objective = constant["active_rmse_n"] / 20.0 + sine["active_rmse_n"] / 10.62 + 0.15 * (constant["quiet_rmse_n"] + sine["quiet_rmse_n"]) / 2.0
        rows.append({
            "gain": gain,
            "constant_active_rmse_n": constant["active_rmse_n"],
            "sine_active_rmse_n": sine["active_rmse_n"],
            "constant_quiet_rmse_n": constant["quiet_rmse_n"],
            "sine_quiet_rmse_n": sine["quiet_rmse_n"],
            "balanced_objective": objective,
        })
        if abs(gain - CURRENT_GAIN) < 1e-12:
            current_replay_error = {
                "constant_max_abs_n": float(np.max(np.abs(constant["estimate"] - constant["recorded"]))),
                "sine_max_abs_n": float(np.max(np.abs(sine["estimate"] - sine["recorded"]))),
            }
    raw_best = min(rows, key=lambda item: item["balanced_objective"])
    near_best = [
        item for item in rows
        if item["balanced_objective"] <= 1.01 * raw_best["balanced_objective"]
    ]
    selected = min(near_best, key=lambda item: item["gain"])
    payload = {
        "current_replay_error": current_replay_error,
        "selection_rule": "smallest gain within 1% of the minimum balanced objective",
        "raw_best": raw_best,
        "candidates": rows,
        "selected": selected,
    }
    (args.output / "fy_gain_calibration.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    with (args.output / "fy_gain_calibration.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
