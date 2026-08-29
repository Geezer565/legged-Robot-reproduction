#!/usr/bin/env python3

"""Compare Go2 shadow-only and limited-authority MyRBF closed-loop runs."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


COLORS = {
    "Shadow-only": "#7F7F7F",
    "Closed loop 3 N": "#E69F00",
    "Closed loop 6 N": "#0072B2",
}


def read_csv(path):
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"No rows in {path}")
    result = {}
    for field in rows[0]:
        values = []
        for row in rows:
            try:
                values.append(float(row[field]))
            except (TypeError, ValueError):
                values.append(float("nan"))
        result[field] = np.asarray(values, dtype=float)
    return result


def rms(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(values * values)))


def load_case(directory):
    case = {
        "state": read_csv(directory / "state.csv"),
        "shadow": read_csv(directory / "shadow.csv"),
    }
    wrench_path = directory / "wrench.csv"
    if wrench_path.exists():
        case["wrench"] = read_csv(wrench_path)
    return case


def active_window(case):
    wrench = case.get("wrench")
    if wrench is None:
        return 5.0, 11.0
    success = (wrench["service_called"] > 0.5) & (wrench["service_success"] > 0.5)
    if not np.any(success):
        raise RuntimeError("No successful wrench samples")
    return float(np.min(wrench["ros_time"][success])), float(np.max(wrench["ros_time"][success]))


def case_metrics(case):
    shadow = case["shadow"]
    start, stop = active_window(case)
    active = (shadow["time_s"] >= start) & (shadow["time_s"] <= stop)
    pre = (shadow["time_s"] >= start - 1.0) & (shadow["time_s"] < start)
    y_offset = float(np.mean(shadow["actual_base_y_m"][pre]))
    y_error = shadow["actual_base_y_m"][active] - y_offset
    roll_error = (
        shadow["actual_base_roll_rad"][active]
        - shadow["desired_base_roll_rad"][active]
    )
    applied_series = shadow.get("applied_control_fy_n", np.zeros_like(shadow["time_s"]))
    applied = applied_series[active]
    return {
        "active_start_s": start,
        "active_end_s": stop,
        "samples": int(np.sum(active)),
        "max_abs_y_shift_cm": 100.0 * float(np.max(np.abs(y_error))),
        "rms_y_shift_cm": 100.0 * rms(y_error),
        "max_abs_roll_error_deg": math.degrees(float(np.max(np.abs(roll_error)))),
        "rms_roll_error_deg": math.degrees(rms(roll_error)),
        "max_abs_roll_deg": math.degrees(float(np.max(np.abs(shadow["actual_base_roll_rad"][active])))),
        "min_base_z_m": float(np.min(shadow["actual_base_z_m"])),
        "final_base_z_m": float(shadow["actual_base_z_m"][-1]),
        "mean_applied_fy_n": float(np.mean(applied)),
        "peak_abs_applied_fy_n": float(np.max(np.abs(applied))),
        "valid_samples_final": int(shadow["valid_sample_count"][-1]),
        "rejected_samples_final": int(shadow["rejected_sample_count"][-1]),
    }


def style_axis(ax):
    ax.grid(True, color="#D9D9D9", linewidth=0.7, alpha=0.8)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def plot_constant(cases, output):
    fig, axes = plt.subplots(3, 1, figsize=(10.5, 9.0), sharex=True, constrained_layout=True)
    active_duration = max(active_window(case)[1] - active_window(case)[0] for case in cases.values())
    for label, case in cases.items():
        shadow = case["shadow"]
        start, _ = active_window(case)
        t = shadow["time_s"] - start
        pre = (t >= -1.0) & (t < 0.0)
        y0 = float(np.mean(shadow["actual_base_y_m"][pre]))
        axes[0].plot(t, 100.0 * (shadow["actual_base_y_m"] - y0), color=COLORS[label], label=label)
        axes[1].plot(t, np.degrees(shadow["actual_base_roll_rad"] - shadow["desired_base_roll_rad"]),
                     color=COLORS[label], label=label)
        applied = shadow.get("applied_control_fy_n", np.zeros_like(shadow["time_s"]))
        axes[2].plot(t, applied, color=COLORS[label], label=label)
    for ax in axes:
        ax.axvspan(0.0, active_duration, color="#CC3311", alpha=0.08)
        ax.axvline(0.0, color="#CC3311", linestyle="--", linewidth=1.0)
        style_axis(ax)
    axes[0].set_ylabel("Lateral shift (cm)")
    axes[1].set_ylabel("Roll error (deg)")
    axes[2].set_ylabel("Applied Fy (N)")
    axes[2].set_xlabel("Time relative to +20 N disturbance onset (s)")
    axes[0].legend(frameon=False, ncol=3)
    fig.suptitle("Go2 limited-authority MyRBF closed-loop check", fontweight="bold")
    fig.savefig(output, dpi=300, facecolor="white", bbox_inches="tight")
    fig.savefig(output.with_suffix(".pdf"), facecolor="white", bbox_inches="tight")
    plt.close(fig)


def plot_metrics(metrics, output):
    labels = list(metrics)
    x = np.arange(len(labels))
    fig, axes = plt.subplots(1, 3, figsize=(12.5, 4.2), constrained_layout=True)
    fields = (
        ("max_abs_y_shift_cm", "Maximum lateral shift (cm)"),
        ("rms_y_shift_cm", "RMS lateral shift (cm)"),
        ("max_abs_roll_deg", "Maximum body roll (deg)"),
    )
    for ax, (field, title) in zip(axes, fields):
        values = [metrics[label][field] for label in labels]
        ax.bar(x, values, color=[COLORS[label] for label in labels])
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=15, ha="right")
        ax.set_title(title, fontweight="bold")
        style_axis(ax)
    fig.savefig(output, dpi=300, facecolor="white", bbox_inches="tight")
    fig.savefig(output.with_suffix(".pdf"), facecolor="white", bbox_inches="tight")
    plt.close(fig)


def plot_sine(cases, output):
    fig, axes = plt.subplots(4, 1, figsize=(10.5, 10.8), sharex=True, constrained_layout=True)
    active_duration = max(active_window(case)[1] - active_window(case)[0] for case in cases.values())
    for label, case in cases.items():
        shadow = case["shadow"]
        start, _ = active_window(case)
        t = shadow["time_s"] - start
        pre = (t >= -1.0) & (t < 0.0)
        y0 = float(np.mean(shadow["actual_base_y_m"][pre]))
        axes[0].plot(t, 100.0 * (shadow["actual_base_y_m"] - y0), color=COLORS[label], label=label)
        axes[1].plot(t, np.degrees(shadow["actual_base_roll_rad"] - shadow["desired_base_roll_rad"]),
                     color=COLORS[label], label=label)
        applied = shadow.get("applied_control_fy_n", np.zeros_like(shadow["time_s"]))
        axes[2].plot(t, applied, color=COLORS[label], label=label)
        axes[3].plot(t, shadow["estimate_after_fy_n"], color=COLORS[label], label=f"{label} estimate")
    reference = next(iter(cases.values()))
    wrench = reference["wrench"]
    start, _ = active_window(reference)
    axes[3].plot(wrench["ros_time"] - start, -wrench["fy"], color="#CC3311",
                 linestyle="--", linewidth=1.5, label="Ideal compensation")
    for ax in axes:
        ax.axvspan(0.0, active_duration, color="#CC3311", alpha=0.08)
        ax.axvline(0.0, color="#CC3311", linestyle="--", linewidth=1.0)
        style_axis(ax)
    axes[0].set_ylabel("Lateral shift (cm)")
    axes[1].set_ylabel("Roll error (deg)")
    axes[2].set_ylabel("Applied Fy (N)")
    axes[3].set_ylabel("Estimated Fy (N)")
    axes[3].set_xlabel("Time relative to sinusoidal disturbance onset (s)")
    axes[0].legend(frameon=False, ncol=2)
    axes[3].legend(frameon=False, ncol=2)
    fig.suptitle("Go2 MyRBF response to sinusoidal lateral disturbance", fontweight="bold")
    fig.savefig(output, dpi=300, facecolor="white", bbox_inches="tight")
    fig.savefig(output.with_suffix(".pdf"), facecolor="white", bbox_inches="tight")
    plt.close(fig)


def write_summary(constant_metrics, sine_metrics, output):
    baseline = constant_metrics["Shadow-only"]
    lines = [
        "# Go2 MyRBF 有限闭环检查（2026-08-29）",
        "",
        "本轮只做单次安全性与方向性检查，不作为重复统计结果。恒定扰动为横向 +20 N；周期扰动为横向 ±15 N。",
        "",
        "| 条件 | 最大横向偏移 (cm) | 横向均方根偏移 (cm) | 最大机身横滚 (deg) | 最大目标跟踪误差 (deg) | 平均补偿 Fy (N) | 补偿峰值 (N) | 最低高度 (m) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for label, item in constant_metrics.items():
        lines.append(
            f"| {label} | {item['max_abs_y_shift_cm']:.3f} | {item['rms_y_shift_cm']:.3f} | "
            f"{item['max_abs_roll_deg']:.3f} | {item['max_abs_roll_error_deg']:.3f} | {item['mean_applied_fy_n']:.3f} | "
            f"{item['peak_abs_applied_fy_n']:.3f} | {item['min_base_z_m']:.3f} |"
        )
    lines.extend(["", "## 相对只观察模式", ""])
    for label in ("Closed loop 3 N", "Closed loop 6 N"):
        item = constant_metrics[label]
        max_gain = 100.0 * (baseline["max_abs_y_shift_cm"] - item["max_abs_y_shift_cm"]) / baseline["max_abs_y_shift_cm"]
        rms_gain = 100.0 * (baseline["rms_y_shift_cm"] - item["rms_y_shift_cm"]) / baseline["rms_y_shift_cm"]
        roll_change = 100.0 * (item["max_abs_roll_deg"] - baseline["max_abs_roll_deg"]) / baseline["max_abs_roll_deg"]
        lines.append(
            f"- {label}：最大横向偏移改善 {max_gain:.1f}%，横向均方根偏移改善 {rms_gain:.1f}%，"
            f"但最大机身横滚增加 {roll_change:.1f}%。"
        )
    lines.extend([
        "",
        "",
        "## 正弦 ±15 N 扰动",
        "",
        "| 条件 | 最大横向偏移 (cm) | 横向均方根偏移 (cm) | 最大机身横滚 (deg) | 最大目标跟踪误差 (deg) | 平均补偿 Fy (N) | 补偿峰值 (N) | 最低高度 (m) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ])
    for label, item in sine_metrics.items():
        lines.append(
            f"| {label} | {item['max_abs_y_shift_cm']:.3f} | {item['rms_y_shift_cm']:.3f} | "
            f"{item['max_abs_roll_deg']:.3f} | {item['max_abs_roll_error_deg']:.3f} | {item['mean_applied_fy_n']:.3f} | "
            f"{item['peak_abs_applied_fy_n']:.3f} | {item['min_base_z_m']:.3f} |"
        )
    sine_base = sine_metrics["Shadow-only"]
    sine_closed = sine_metrics["Closed loop 6 N"]
    sine_max_gain = 100.0 * (sine_base["max_abs_y_shift_cm"] - sine_closed["max_abs_y_shift_cm"]) / sine_base["max_abs_y_shift_cm"]
    sine_rms_gain = 100.0 * (sine_base["rms_y_shift_cm"] - sine_closed["rms_y_shift_cm"]) / sine_base["rms_y_shift_cm"]
    sine_roll_change = 100.0 * (sine_closed["max_abs_roll_deg"] - sine_base["max_abs_roll_deg"]) / sine_base["max_abs_roll_deg"]
    lines.extend([
        "",
        f"- 正弦扰动下：最大横向偏移改善 {sine_max_gain:.1f}%，横向均方根偏移改善 {sine_rms_gain:.1f}%，但最大机身横滚增加 {sine_roll_change:.1f}%。",
        "- 因为横向偏移的小幅改善伴随横滚增大，本轮只能证明闭环接线和补偿方向有效，不能证明整体稳定性更好；不再继续放大补偿。",
        "",
    ])
    output.write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--shadow-root", type=Path, required=True)
    parser.add_argument("--closedloop-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    constant_cases = {
        "Shadow-only": load_case(args.shadow_root / "myrbf_go2_constant"),
        "Closed loop 3 N": load_case(args.closedloop_root / "C1_closedloop_constant"),
        "Closed loop 6 N": load_case(args.closedloop_root / "C2_closedloop_constant_6N"),
    }
    sine_cases = {
        "Shadow-only": load_case(args.shadow_root / "myrbf_go2_sine"),
        "Closed loop 6 N": load_case(args.closedloop_root / "C3_closedloop_sine_6N"),
    }
    constant_metrics = {label: case_metrics(case) for label, case in constant_cases.items()}
    sine_metrics = {label: case_metrics(case) for label, case in sine_cases.items()}
    all_metrics = {"constant": constant_metrics, "sine": sine_metrics}
    (args.output / "metrics.json").write_text(json.dumps(all_metrics, indent=2), encoding="utf-8")
    write_summary(constant_metrics, sine_metrics, args.output / "RESULT_SUMMARY.md")
    plot_constant(constant_cases, args.output / "01_closedloop_constant_comparison.png")
    plot_metrics(constant_metrics, args.output / "02_closedloop_metric_summary.png")
    plot_sine(sine_cases, args.output / "03_closedloop_sine_comparison.png")
    print(json.dumps(all_metrics, indent=2))


if __name__ == "__main__":
    main()
