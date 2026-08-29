#!/usr/bin/env python3

"""Analyze the single-run Go2 Baseline/ETHRBF/MyRBF shadow comparison."""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


METHODS = {
    "Baseline": "baseline",
    "ETHRBF": "eth",
    "MyRBF": "myrbf_go2",
}
COLORS = {"Baseline": "#7F7F7F", "ETHRBF": "#E69F00", "MyRBF": "#0072B2"}
SCENARIOS = ("constant", "sine")


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
    return float(np.sqrt(np.mean(values * values))) if values.size else float("nan")


def correlation(a, b):
    mask = np.isfinite(a) & np.isfinite(b)
    a, b = a[mask], b[mask]
    if a.size < 3 or np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def load_cases(root):
    cases = {}
    for label, stem in METHODS.items():
        for scenario in SCENARIOS:
            directory = root / f"{stem}_{scenario}"
            cases[(label, scenario)] = {
                "shadow": read_csv(directory / "shadow.csv"),
                "state": read_csv(directory / "state.csv"),
                "wrench": read_csv(directory / "wrench.csv"),
            }
    return cases


def active_window(wrench):
    mask = (wrench["service_called"] > 0.5) & (wrench["service_success"] > 0.5)
    if not np.any(mask):
        raise RuntimeError("No successful wrench samples")
    return float(np.min(wrench["ros_time"][mask])), float(np.max(wrench["ros_time"][mask]))


def case_metrics(case):
    shadow, state, wrench = case["shadow"], case["state"], case["wrench"]
    start, stop = active_window(wrench)
    mask = (shadow["time_s"] >= start) & (shadow["time_s"] <= stop)
    time_s = shadow["time_s"][mask]
    estimate = shadow["estimate_after_fy_n"][mask]
    target = -np.interp(time_s, wrench["ros_time"], wrench["fy"])
    actual_y = shadow["actual_base_y_m"][mask]
    desired_y = shadow["desired_base_y_m"][mask]
    actual_z = shadow["actual_base_z_m"][mask]
    desired_z = shadow["desired_base_z_m"][mask]
    actual_roll = shadow["actual_base_roll_rad"][mask]
    desired_roll = shadow["desired_base_roll_rad"][mask]
    state_mask = (state["ros_time"] >= start) & (state["ros_time"] <= stop)
    called = wrench["service_called"] > 0.5
    return {
        "active_start_s": start,
        "active_end_s": stop,
        "samples": int(np.sum(mask)),
        "estimate_mean_n": float(np.mean(estimate)),
        "estimate_peak_abs_n": float(np.max(np.abs(estimate))),
        "estimation_rmse_n": rms(estimate - target),
        "estimation_correlation": correlation(estimate, target),
        "tracking_y_rmse_cm": 100.0 * rms(actual_y - desired_y),
        "tracking_z_rmse_cm": 100.0 * rms(actual_z - desired_z),
        "tracking_roll_rmse_deg": math.degrees(rms(actual_roll - desired_roll)),
        "max_abs_base_y_cm": 100.0 * float(np.max(np.abs(state["base_y"][state_mask]))),
        "min_base_z_m": float(np.min(state["base_z"][state_mask])),
        "final_base_z_m": float(state["base_z"][np.isfinite(state["base_z"])][-1]),
        "wrench_success_rate": float(np.mean(wrench["service_success"][called])),
        "valid_samples_final": int(shadow["valid_sample_count"][-1]),
        "rejected_samples_final": int(shadow["rejected_sample_count"][-1]),
    }


def style_axis(ax):
    ax.grid(True, color="#D9D9D9", linewidth=0.7, alpha=0.8)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def plot_estimates(cases, scenario, output):
    fig, ax = plt.subplots(figsize=(10.5, 5.4), constrained_layout=True)
    reference_case = cases[("MyRBF", scenario)]
    reference = reference_case["wrench"]
    reference_start, _ = active_window(reference)
    ax.plot(reference["ros_time"] - reference_start, -reference["fy"], color="#CC3311",
            linestyle="--", linewidth=2.0, label="Ideal compensation")
    for method in METHODS:
        case = cases[(method, scenario)]
        shadow = case["shadow"]
        start, _ = active_window(case["wrench"])
        ax.plot(shadow["time_s"] - start, shadow["estimate_after_fy_n"],
                color=COLORS[method], linewidth=1.6, label=method)
    title = "Constant +20 N lateral disturbance" if scenario == "constant" else "Sinusoidal ±15 N lateral disturbance"
    ax.set_title(f"Go2 shadow-mode disturbance estimate — {title}", fontweight="bold")
    ax.set_xlabel("Time relative to disturbance onset (s)")
    ax.set_ylabel("Fy compensation estimate (N)")
    ax.legend(ncol=2, frameon=False)
    style_axis(ax)
    fig.savefig(output, dpi=300, facecolor="white", bbox_inches="tight")
    fig.savefig(output.with_suffix(".pdf"), facecolor="white", bbox_inches="tight")
    plt.close(fig)


def plot_tracking(cases, scenario, output):
    fig, axes = plt.subplots(2, 3, figsize=(13, 7), sharex="col", constrained_layout=True)
    for column, method in enumerate(METHODS):
        case = cases[(method, scenario)]
        shadow = case["shadow"]
        start, _ = active_window(case["wrench"])
        t = shadow["time_s"] - start
        axes[0, column].plot(t, 100.0 * shadow["desired_base_y_m"],
                             color="#CC3311", linestyle="--", linewidth=1.8, label="Target")
        axes[0, column].plot(t, 100.0 * shadow["actual_base_y_m"],
                             color=COLORS[method], linewidth=1.3, label="Actual")
        axes[1, column].plot(t, np.degrees(shadow["desired_base_roll_rad"]),
                             color="#CC3311", linestyle="--", linewidth=1.8)
        axes[1, column].plot(t, np.degrees(shadow["actual_base_roll_rad"]),
                             color=COLORS[method], linewidth=1.3)
        axes[0, column].set_title(method, fontweight="bold")
        axes[1, column].set_xlabel("Time from disturbance onset (s)")
        style_axis(axes[0, column])
        style_axis(axes[1, column])
    axes[0, 0].set_ylabel("Base lateral y (cm)")
    axes[1, 0].set_ylabel("Base roll (deg)")
    axes[0, 0].legend(frameon=False, loc="best")
    title = "constant disturbance" if scenario == "constant" else "sinusoidal disturbance"
    fig.suptitle(f"Target versus actual Go2 body state — {title}\n(all estimators remain shadow-only)", fontweight="bold")
    fig.savefig(output, dpi=300, facecolor="white", bbox_inches="tight")
    fig.savefig(output.with_suffix(".pdf"), facecolor="white", bbox_inches="tight")
    plt.close(fig)


def plot_metric_summary(metrics, output):
    labels = list(METHODS)
    x = np.arange(len(labels))
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.5), constrained_layout=True)
    width = 0.34
    for offset, scenario in ((-width / 2, "constant"), (width / 2, "sine")):
        values = [metrics[f"{method}_{scenario}"]["estimation_rmse_n"] for method in labels]
        axes[0].bar(x + offset, values, width, label=scenario.capitalize(),
                    color="#56B4E9" if scenario == "constant" else "#009E73")
        tracking = [metrics[f"{method}_{scenario}"]["tracking_y_rmse_cm"] for method in labels]
        axes[1].bar(x + offset, tracking, width, label=scenario.capitalize(),
                    color="#56B4E9" if scenario == "constant" else "#009E73")
    axes[0].set_ylabel("Fy estimation RMSE (N)")
    axes[1].set_ylabel("Lateral tracking RMSE (cm)")
    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.legend(frameon=False)
        style_axis(ax)
    axes[0].set_title("Disturbance estimation", fontweight="bold")
    axes[1].set_title("Controller tracking (shadow-only)", fontweight="bold")
    fig.savefig(output, dpi=300, facecolor="white", bbox_inches="tight")
    fig.savefig(output.with_suffix(".pdf"), facecolor="white", bbox_inches="tight")
    plt.close(fig)


def write_summary(metrics, output):
    lines = [
        "# Go2 三方法单次对比结果（2026-08-29）",
        "",
        "> 本轮 Baseline、ETHRBF、MyRBF 都处于只观察模式，没有修改控制输出。因此外力估计可以公平比较；机身跟踪曲线用于证明数据链与站立稳定，不能据此声称某种方法改善了控制。",
        "",
        "| 方法 | 场景 | 估计均值 (N) | 估计峰值 (N) | 估计 RMSE (N) | 波形相关性 | 横向跟踪 RMSE (cm) | 最低高度 (m) |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    scenario_cn = {"constant": "恒定 +20 N", "sine": "正弦 ±15 N"}
    for method in METHODS:
        for scenario in SCENARIOS:
            item = metrics[f"{method}_{scenario}"]
            corr = item["estimation_correlation"]
            corr_text = "—" if not math.isfinite(corr) else f"{corr:.3f}"
            lines.append(
                f"| {method} | {scenario_cn[scenario]} | {item['estimate_mean_n']:.2f} | "
                f"{item['estimate_peak_abs_n']:.2f} | {item['estimation_rmse_n']:.2f} | "
                f"{corr_text} | {item['tracking_y_rmse_cm']:.2f} | {item['min_base_z_m']:.3f} |"
            )
    lines.extend([
        "",
        "## 结论",
        "",
        "- Baseline 的网络更新被关闭，输出应保持为零，用于确认分析与记录链没有人为偏置。",
        "- ETHRBF 只根据目标与实际运动的偏差更新；MyRBF 只根据动力学残差与当前估计的差更新。",
        "- 只有在 MyRBF 的估计误差明显较小、所有场景均保持站立且没有拒绝样本异常时，才进入小比例闭环试验。",
        "- 本表是每个条件一次正式运行，不包含重复统计或显著性结论。",
        "",
    ])
    output.write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    cases = load_cases(args.root)
    metrics = {
        f"{method}_{scenario}": case_metrics(cases[(method, scenario)])
        for method in METHODS for scenario in SCENARIOS
    }
    (args.output / "metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    write_summary(metrics, args.output / "RESULT_SUMMARY.md")
    plot_estimates(cases, "constant", args.output / "01_constant_estimation_comparison.png")
    plot_estimates(cases, "sine", args.output / "02_sine_estimation_comparison.png")
    plot_tracking(cases, "constant", args.output / "03_constant_target_vs_actual.png")
    plot_tracking(cases, "sine", args.output / "04_sine_target_vs_actual.png")
    plot_metric_summary(metrics, args.output / "05_metric_summary.png")
    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
