#!/usr/bin/env python3
"""Trace six-axis information through residual -> RBF basis -> estimate."""

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np
import rosbag


TOPIC = "/rbf_closed_loop/sample"
AXES = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")


def disturbance_trace(time, start, end, primary, secondary, profile, frequency):
    time = np.asarray(time, dtype=float)
    primary = np.asarray(primary, dtype=float)
    secondary = np.asarray(secondary, dtype=float)
    result = np.zeros((time.size, 6), dtype=float)
    active = (time > start) & (time <= end)
    phase = 2.0 * math.pi * frequency * (time[active] - start)
    if profile == "constant":
        result[active] = primary
    elif profile == "sine":
        result[active] = np.sin(phase)[:, None] * primary[None, :]
    elif profile == "sine_cosine":
        result[active] = (np.sin(phase)[:, None] * primary[None, :] +
                          np.cos(phase)[:, None] * secondary[None, :])
    else:
        raise ValueError("profile must be constant, sine, or sine_cosine")
    return result, active


def _block(text, name):
    match = re.search(r"\b" + re.escape(name) + r"\s*\{([^{}]*)\}",
                      text, re.DOTALL)
    if not match:
        raise RuntimeError("missing config block: " + name)
    return match.group(1)


def load_basis(path):
    text = Path(path).read_text()

    def vector(name):
        values = np.zeros(6)
        block = _block(text, name)
        matches = re.findall(r"\((\d+),0\)\s+([-+0-9.eE]+)", block)
        if len(matches) != 6:
            raise RuntimeError(name + " must contain six entries")
        for axis, value in matches:
            values[int(axis)] = float(value)
        return values

    centers = np.zeros((6, 25, 2))
    widths = np.zeros((6, 25))
    layout = re.search(r"\bcenterLayout\s+(\S+)", text)
    if layout and layout.group(1) == "symmetric_grid_5x5":
        linear_extent = float(re.search(r"\blinearExtent\s+([-+0-9.eE]+)", text).group(1))
        angular_extent = float(re.search(r"\bangularExtent\s+([-+0-9.eE]+)", text).group(1))
        width_ratio = float(re.search(r"\bgridWidthRatio\s+([-+0-9.eE]+)", text).group(1))
        for axis in range(6):
            extent = linear_extent if axis < 3 else angular_extent
            spacing = extent / 2.0
            for row in range(5):
                for column in range(5):
                    node = 5 * row + column
                    centers[axis, node] = [-extent + spacing * column,
                                           -extent + spacing * row]
                    widths[axis, node] = width_ratio * spacing
    else:
        center_matches = re.findall(
            r"\((\d+),(\d+),(\d+)\)\s+([-+0-9.eE]+)",
            _block(text, "centers"))
        if len(center_matches) != 300:
            raise RuntimeError("centers must contain 300 entries")
        for axis, node, feature, value in center_matches:
            centers[int(axis), int(node), int(feature)] = float(value)
        width_matches = re.findall(r"\((\d+),(\d+)\)\s+([-+0-9.eE]+)",
                                   _block(text, "widths"))
        if len(width_matches) != 150:
            raise RuntimeError("widths must contain 150 entries")
        for axis, node, value in width_matches:
            widths[int(axis), int(node)] = float(value)
    return vector("residualScale"), vector("discrepancyScale"), centers, widths


def read_diagnostics(path):
    rows = []
    with rosbag.Bag(str(path), "r") as bag:
        for _, message, _ in bag.read_messages(topics=[TOPIC]):
            row = np.asarray(message.data, dtype=float)
            if row.shape == (50,):
                rows.append(row)
    if len(rows) < 100:
        raise RuntimeError("not enough 50-field RBF diagnostics")
    rows = np.vstack(rows)
    if not np.isfinite(rows).all():
        raise RuntimeError("diagnostics contain NaN or Inf")
    return rows


def safe_corr(a, b):
    if np.std(a) <= 1e-12 or np.std(b) <= 1e-12:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("basis", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--end", type=float, required=True)
    parser.add_argument("--profile", choices=("constant", "sine", "sine_cosine"),
                        required=True)
    parser.add_argument("--frequency", type=float, default=0.35)
    parser.add_argument("--primary", type=float, nargs=6, required=True)
    parser.add_argument("--secondary", type=float, nargs=6, default=[0.0] * 6)
    args = parser.parse_args()

    rows = read_diagnostics(args.bag)
    residual_scale, discrepancy_scale, centers, widths = load_basis(args.basis)
    time = rows[:, 0]
    raw = rows[:, 12:18]
    filtered = rows[:, 18:24]
    sigma = rows[:, 24:30]
    estimate_before = rows[:, 30:36]
    estimate = rows[:, 36:42]
    truth, active = disturbance_trace(
        time, args.start, args.end, args.primary, args.secondary,
        args.profile, args.frequency)
    feature = np.stack((filtered / residual_scale,
                        (filtered - estimate_before) / discrepancy_scale), axis=1)

    max_basis = np.zeros((time.size, 6))
    active_nodes = np.zeros((time.size, 6))
    for axis in range(6):
        delta = feature[:, :, axis][:, None, :] - centers[axis][None, :, :]
        phi = np.exp(-np.sum(delta * delta, axis=2) /
                     (2.0 * widths[axis][None, :] ** 2))
        max_basis[:, axis] = np.max(phi, axis=1)
        active_nodes[:, axis] = np.sum(phi > 0.1, axis=1)

    report = {
        "bag": str(args.bag),
        "basis": str(args.basis),
        "active_sample_count": int(np.sum(active)),
        "axes": {},
    }
    for axis, name in enumerate(AXES):
        target = truth[active, axis]
        signal_present = float(np.max(np.abs(target))) > 1e-9
        report["axes"][name] = {
            "truth_peak_abs": float(np.max(np.abs(target))),
            "raw_residual_correlation": safe_corr(raw[active, axis], target)
                if signal_present else None,
            "filtered_residual_correlation": safe_corr(filtered[active, axis], target)
                if signal_present else None,
            "estimate_correlation": safe_corr(estimate[active, axis], target)
                if signal_present else None,
            "raw_residual_rmse": float(np.sqrt(np.mean(
                (raw[active, axis] - target) ** 2))),
            "filtered_residual_rmse": float(np.sqrt(np.mean(
                (filtered[active, axis] - target) ** 2))),
            "estimate_rmse": float(np.sqrt(np.mean(
                (estimate[active, axis] - target) ** 2))),
            "filtered_peak_abs": float(np.max(np.abs(filtered[active, axis]))),
            "estimate_peak_abs": float(np.max(np.abs(estimate[active, axis]))),
            "sigma_rms": float(np.sqrt(np.mean(sigma[active, axis] ** 2))),
            "feature_level_min_max": [float(np.min(feature[active, 0, axis])),
                                      float(np.max(feature[active, 0, axis]))],
            "feature_discrepancy_min_max": [
                float(np.min(feature[active, 1, axis])),
                float(np.max(feature[active, 1, axis]))],
            "basis_max_median": float(np.median(max_basis[active, axis])),
            "basis_max_p05": float(np.percentile(max_basis[active, axis], 5)),
            "basis_uncovered_ratio_lt_0_05": float(np.mean(
                max_basis[active, axis] < 0.05)),
            "active_gaussian_nodes_median": float(np.median(
                active_nodes[active, axis])),
        }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
