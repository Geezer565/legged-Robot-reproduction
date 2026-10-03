#!/usr/bin/env python3
"""Aggregate repeated four-method adaptive-control comparison reports."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


METHOD_ORDER = ("baseline", "current_rbf", "composite_rbf", "rff_ogd")
METHOD_LABELS = {
    "baseline": "BASELINE",
    "current_rbf": "CURRENT RBF",
    "composite_rbf": "COMPOSITE RBF",
    "rff_ogd": "RFF-OGD",
}
METHOD_COLORS = {
    "baseline": "#0072B2",
    "current_rbf": "#D55E00",
    "composite_rbf": "#009E73",
    "rff_ogd": "#CC79A7",
}
METRIC_NAMES = (
    "active_sigma_rms",
    "active_sigma_peak",
    "post_sigma_rms",
    "active_tail_dominant_estimate_mean",
    "active_tail_estimate_fraction_of_applied",
    "active_tail_dominant_estimate_slope_per_s",
)


def load_reports(paths):
    reports = []
    expected_protocol = None
    for path in paths:
        report = json.loads(path.read_text(encoding="utf-8"))
        if not report.get("integration_passed", False):
            raise RuntimeError(f"integration checks did not pass: {path}")
        if tuple(report.get("methods", ())) != METHOD_ORDER:
            raise RuntimeError(f"unexpected method order: {path}")
        protocol = report.get("experiment_protocol")
        if expected_protocol is None:
            expected_protocol = protocol
        elif protocol != expected_protocol:
            raise RuntimeError(f"experiment protocol mismatch: {path}")
        reports.append(report)
    if len(reports) < 2:
        raise RuntimeError("at least two repeated reports are required")
    return reports, expected_protocol


def summarize(reports, protocol):
    methods = {}
    for method in METHOD_ORDER:
        metric_summary = {}
        for metric in METRIC_NAMES:
            values = np.asarray(
                [report["metrics"][method][metric] for report in reports],
                dtype=float)
            metric_summary[metric] = {
                "values": values.tolist(),
                "mean": float(np.mean(values)),
                "std": float(np.std(values, ddof=1)),
                "min": float(np.min(values)),
                "max": float(np.max(values)),
            }
        methods[method] = metric_summary

    improvements = {}
    baseline = np.asarray(
        [report["metrics"]["baseline"]["active_sigma_rms"]
         for report in reports], dtype=float)
    baseline_post = np.asarray(
        [report["metrics"]["baseline"]["post_sigma_rms"]
         for report in reports], dtype=float)
    for method in METHOD_ORDER[1:]:
        active = np.asarray(
            [report["metrics"][method]["active_sigma_rms"]
             for report in reports], dtype=float)
        post = np.asarray(
            [report["metrics"][method]["post_sigma_rms"]
             for report in reports], dtype=float)
        active_improvement = 1.0 - active / baseline
        post_improvement = 1.0 - post / baseline_post
        improvements[method] = {
            "active_sigma_rms_fraction": {
                "values": active_improvement.tolist(),
                "mean": float(np.mean(active_improvement)),
                "std": float(np.std(active_improvement, ddof=1)),
                "min": float(np.min(active_improvement)),
                "max": float(np.max(active_improvement)),
            },
            "post_sigma_rms_fraction": {
                "values": post_improvement.tolist(),
                "mean": float(np.mean(post_improvement)),
                "std": float(np.std(post_improvement, ddof=1)),
                "min": float(np.min(post_improvement)),
                "max": float(np.max(post_improvement)),
            },
        }

    winners = [report["ranking_by_active_sigma_rms"][0]
               for report in reports]
    return {
        "schema_version": 1,
        "trial_count": len(reports),
        "experiment_protocol": protocol,
        "methods": methods,
        "improvement_vs_baseline": improvements,
        "active_rms_winner_counts": {
            method: winners.count(method) for method in METHOD_ORDER
        },
    }


def write_csv(output, summary):
    with (output / "campaign_summary.csv").open(
            "w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow(["method", "metric", "mean", "std", "min", "max"])
        for method in METHOD_ORDER:
            for metric in METRIC_NAMES:
                row = summary["methods"][method][metric]
                writer.writerow([method, metric, row["mean"], row["std"],
                                 row["min"], row["max"]])


def configure_axis(axis):
    axis.grid(True, axis="y", alpha=0.25)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)


def save_figures(output, summary):
    x = np.arange(len(METHOD_ORDER))
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.8),
                             constrained_layout=True)
    for axis, metric, title in (
            (axes[0], "active_sigma_rms", "During disturbance"),
            (axes[1], "post_sigma_rms", "After disturbance release")):
        means = [summary["methods"][method][metric]["mean"]
                 for method in METHOD_ORDER]
        stds = [summary["methods"][method][metric]["std"]
                for method in METHOD_ORDER]
        axis.bar(x, means, yerr=stds,
                 color=[METHOD_COLORS[method] for method in METHOD_ORDER],
                 capsize=4)
        axis.set_xticks(x)
        axis.set_xticklabels(
            [METHOD_LABELS[method] for method in METHOD_ORDER],
            rotation=15, ha="right")
        axis.set_ylabel("Tracking-error measure (lower is better)")
        axis.set_title(title)
        configure_axis(axis)
    fig.suptitle(f"Repeated long-force comparison (n={summary['trial_count']})")
    fig.savefig(output / "figure_1_repeated_tracking_comparison.png", dpi=260,
                bbox_inches="tight", facecolor="white")
    fig.savefig(output / "figure_1_repeated_tracking_comparison.pdf",
                bbox_inches="tight", facecolor="white")
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(9.2, 4.8), constrained_layout=True)
    positions = np.arange(len(METHOD_ORDER) - 1)
    for index, method in enumerate(METHOD_ORDER[1:]):
        values = np.asarray(summary["improvement_vs_baseline"][method]
                            ["active_sigma_rms_fraction"]["values"]) * 100.0
        axis.scatter(np.full(values.shape, positions[index]), values,
                     color=METHOD_COLORS[method], s=38, zorder=3)
        axis.plot([positions[index] - 0.16, positions[index] + 0.16],
                  [np.mean(values), np.mean(values)], color="#222222",
                  linewidth=2.0)
    axis.axhline(0.0, color="#555555", linewidth=0.8)
    axis.set_xticks(positions)
    axis.set_xticklabels(
        [METHOD_LABELS[method] for method in METHOD_ORDER[1:]])
    axis.set_ylabel("Reduction versus BASELINE (%)")
    axis.set_title("Every point is one complete repeated trial")
    configure_axis(axis)
    fig.savefig(output / "figure_2_trialwise_improvement.png", dpi=260,
                bbox_inches="tight", facecolor="white")
    fig.savefig(output / "figure_2_trialwise_improvement.pdf",
                bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metrics", type=Path, nargs="+")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    reports, protocol = load_reports(args.metrics)
    summary = summarize(reports, protocol)
    (args.output_dir / "campaign_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    write_csv(args.output_dir, summary)
    save_figures(args.output_dir, summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
