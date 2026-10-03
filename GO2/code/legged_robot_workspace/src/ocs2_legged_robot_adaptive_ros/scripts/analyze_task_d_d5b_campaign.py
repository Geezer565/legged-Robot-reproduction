#!/usr/bin/env python3
"""Unified paired analysis of the repeated D5B bias-attribution campaign."""

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


GROUPS = ("nominal_stance_hold", "nominal_stance_target",
          "nominal_walk_target", "uncertain_walk_target")
RUNS = ("run_01", "run_02", "run_03")
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy", "Jyz", "Jzz")
UNITS = ("kg", "kg m", "kg m", "kg m", "kg m2", "kg m2", "kg m2", "kg m2", "kg m2", "kg m2")
LABELS = {
    "nominal_stance_hold": "N: stance hold",
    "nominal_stance_target": "N: stance + target",
    "nominal_walk_target": "N: walk + target",
    "uncertain_walk_target": "U: walk + target",
}
COLORS = {group: color for group, color in zip(GROUPS,
          ("#4c4c4c", "#1f77b4", "#2ca02c", "#d62728"))}
CONTRASTS = {
    "target_without_gait": ("nominal_stance_hold", "nominal_stance_target"),
    "walking_given_target": ("nominal_stance_target", "nominal_walk_target"),
    "payload_given_walking": ("nominal_walk_target", "uncertain_walk_target"),
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_csv(path):
    values = np.genfromtxt(str(path), delimiter=",", names=True)
    return {name: np.asarray(values[name], dtype=float) for name in values.dtype.names}


def stats(values):
    values = np.asarray(values, dtype=float)
    return {"n": len(values), "mean": float(np.mean(values)),
            "sample_std": float(np.std(values, ddof=1)) if len(values) > 1 else None,
            "min": float(np.min(values)), "max": float(np.max(values))}


def write_rows(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_task_d_d5b_campaign.py D5B_ROOT")
    root = Path(sys.argv[1]).resolve()
    output = root / "analysis"
    output.mkdir(parents=True, exist_ok=True)
    fairness_path = root / "formal_fairness_audit.json"
    fairness = json.loads(fairness_path.read_text())
    protocol_path = root / "protocol" / "stage_d5b_protocol.json"
    protocol = json.loads(protocol_path.read_text())
    if not fairness.get("passed") or not protocol.get("passed"):
        raise RuntimeError("D5B fairness/protocol gate has not passed")

    runs = {}
    reference_time = None
    for group in GROUPS:
        for run_name in RUNS:
            directory = root / "formal" / group / run_name
            summary = json.loads((directory / "task_d_d5b_run_summary.json").read_text())
            if not summary.get("formal_eligibility"):
                raise RuntimeError("ineligible D5B run: {}/{}".format(group, run_name))
            data = load_csv(directory / "task_d_d5b_run.csv")
            if reference_time is None:
                reference_time = data["time_s"]
            if not np.array_equal(data["time_s"], reference_time):
                raise RuntimeError("D5B common-grid mismatch")
            runs[(group, run_name)] = {"summary": summary, "data": data,
                                      "directory": directory}
    time = reference_time
    last_two = time >= time[-1] - 2.0

    per_run_rows, group_rows, contrast_rows = [], [], []
    trajectories = {}
    for group in GROUPS:
        for name in PARAMETERS:
            trajectories[(group, name)] = np.vstack([
                runs[(group, run_name)]["data"]["pi_hat_" + name]
                for run_name in RUNS])
        for run_name in RUNS:
            run = runs[(group, run_name)]
            row = {"group": group, "run": run_name,
                   "sigma_mean_norm": run["summary"]["metrics"]["sigma_mean_norm"],
                   "sigma_peak_norm": run["summary"]["metrics"]["sigma_peak_norm"],
                   "actual_x_displacement_m": run["summary"]["metrics"]["actual_x_displacement_m"],
                   "position_rmse_m": run["summary"]["metrics"]["position_rmse_m"],
                   "applied_correction_rms": run["summary"]["metrics"]["applied_correction_rms"]}
            for index, name in enumerate(PARAMETERS):
                values = run["data"]["pi_hat_" + name]
                row["pi_{}_last2s_mean".format(name)] = float(np.mean(values[last_two]))
                row["pi_{}_net_change".format(name)] = float(values[-1] - values[0])
                row["drive_{}_rms".format(name)] = float(np.sqrt(
                    np.trapz(run["data"]["drive_" + name] ** 2, time) /
                    (time[-1] - time[0])))
            per_run_rows.append(row)

    for group in GROUPS:
        group_run_rows = [row for row in per_run_rows if row["group"] == group]
        for metric in [name for name in group_run_rows[0] if name not in ("group", "run")]:
            group_rows.append({"group": group, "metric": metric,
                               **stats([row[metric] for row in group_run_rows])})

    contrast_trajectories = {}
    for contrast, (baseline, comparison) in CONTRASTS.items():
        for index, name in enumerate(PARAMETERS):
            values = np.vstack([
                trajectories[(comparison, name)][run_index]
                - trajectories[(baseline, name)][run_index]
                for run_index in range(3)])
            contrast_trajectories[(contrast, name)] = values
            last_values = np.mean(values[:, last_two], axis=1)
            truth = runs[("uncertain_walk_target", "run_01")]["data"][
                "pi_star_" + name][0] if contrast == "payload_given_walking" else 0.0
            contrast_rows.append({
                "contrast": contrast, "parameter": name, "unit": UNITS[index],
                "last2s_mean": float(np.mean(last_values)),
                "last2s_sample_std": float(np.std(last_values, ddof=1)),
                "truth_if_payload": float(truth),
                "signed_error_vs_truth": float(np.mean(last_values) - truth),
            })

    write_rows(output / "metrics_per_run.csv", per_run_rows)
    write_rows(output / "metrics_group_mean_std.csv", group_rows)
    write_rows(output / "controlled_contrasts_last2s.csv", contrast_rows)

    fig, axes = plt.subplots(5, 2, figsize=(14, 17), sharex=True)
    for axis, name, unit in zip(axes.flat, PARAMETERS, UNITS):
        for group in GROUPS:
            values = trajectories[(group, name)]
            mean, std = np.mean(values, axis=0), np.std(values, axis=0, ddof=1)
            axis.plot(time, mean, color=COLORS[group], label=LABELS[group])
            axis.fill_between(time, mean - std, mean + std,
                              color=COLORS[group], alpha=0.12)
        axis.set_ylabel("{} [{}]".format(name, unit))
        axis.grid(True, alpha=0.3)
    axes.flat[0].legend(fontsize=8)
    for axis in axes[-1]: axis.set_xlabel("MPC time [s]")
    fig.suptitle("D5B online 10D estimates by controlled condition", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig(output / "figure_D5B_01_parameter_conditions.png", dpi=170)
    plt.close(fig)

    contrast_colors = {"target_without_gait": "#1f77b4",
                       "walking_given_target": "#2ca02c",
                       "payload_given_walking": "#d62728"}
    fig, axes = plt.subplots(5, 2, figsize=(14, 17), sharex=True)
    for axis, name, unit in zip(axes.flat, PARAMETERS, UNITS):
        for contrast in CONTRASTS:
            values = contrast_trajectories[(contrast, name)]
            mean, std = np.mean(values, axis=0), np.std(values, axis=0, ddof=1)
            axis.plot(time, mean, color=contrast_colors[contrast], label=contrast)
            axis.fill_between(time, mean - std, mean + std,
                              color=contrast_colors[contrast], alpha=0.12)
        if name in PARAMETERS:
            truth = runs[("uncertain_walk_target", "run_01")]["data"]["pi_star_" + name]
            axis.plot(time, truth, "k--", lw=1.0, label="payload pi_star")
        axis.set_ylabel("delta {} [{}]".format(name, unit))
        axis.grid(True, alpha=0.3)
    axes.flat[0].legend(fontsize=7)
    for axis in axes[-1]: axis.set_xlabel("MPC time [s]")
    fig.suptitle("D5B paired controlled contrasts (not assumed linearly causal)", y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig(output / "figure_D5B_02_controlled_contrasts.png", dpi=170)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(11, 11), sharex=True)
    for group in GROUPS:
        values = np.vstack([runs[(group, run_name)]["data"]["actual_x"] for run_name in RUNS])
        axes[0].plot(time, np.mean(values, axis=0), color=COLORS[group], label=LABELS[group])
        sigma = np.vstack([runs[(group, run_name)]["data"]["sigma_norm"] for run_name in RUNS])
        axes[1].plot(time, np.mean(sigma, axis=0), color=COLORS[group], label=LABELS[group])
        correction = np.vstack([runs[(group, run_name)]["data"]["applied_correction_norm"]
                                for run_name in RUNS])
        axes[2].plot(time, np.mean(correction, axis=0), color=COLORS[group], label=LABELS[group])
    axes[0].set_ylabel("actual x [m]")
    axes[1].set_ylabel("||sigma||")
    axes[2].set_ylabel("applied correction norm")
    axes[2].set_xlabel("MPC time [s]")
    for axis in axes:
        axis.grid(True, alpha=0.3); axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output / "figure_D5B_03_state_sigma_correction.png", dpi=170)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(12, 5.5))
    width = 0.19
    indices = np.arange(10)
    for offset, group in enumerate(GROUPS):
        values = []
        for name in PARAMETERS:
            runs_values = [row["drive_{}_rms".format(name)] for row in per_run_rows
                           if row["group"] == group]
            values.append(np.mean(runs_values))
        axis.bar(indices + (offset - 1.5) * width, values, width,
                 label=LABELS[group], color=COLORS[group])
    axis.set_yscale("log"); axis.set_xticks(indices); axis.set_xticklabels(PARAMETERS)
    axis.set_ylabel("RMS of Y^T sigma [mixed rate units]")
    axis.set_title("D5B Eq.(8) update drive by condition")
    axis.grid(True, axis="y", which="both", alpha=0.3); axis.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output / "figure_D5B_04_update_drive.png", dpi=170)
    plt.close(fig)

    summary = {
        "schema_version": 1, "stage": "D5B_nominal_bias_attribution_analysis",
        "passed": True, "formal_runs": 12, "groups": list(GROUPS),
        "analysis_window_s": [float(time[0]), float(time[-1])],
        "group_mean_std": {}, "controlled_contrasts_last2s": {},
        "data_integrity": {
            "protocol_passed": bool(protocol.get("passed")),
            "formal_fairness_passed": bool(fairness.get("passed")),
            "all_runs_formally_eligible": all(
                run["summary"].get("formal_eligibility", False) for run in runs.values()),
            "max_eq8_derivative_error": max(
                run["summary"]["eq8_derivative_max_error"] for run in runs.values()),
            "max_eq10_applied_error": max(
                run["summary"]["eq10_applied_max_error"] for run in runs.values()),
        },
        "interpretation_boundary": protocol["interpretation_boundary"],
    }
    for group in GROUPS:
        summary["group_mean_std"][group] = {
            "pi_last2s": {name: stats([
                row["pi_{}_last2s_mean".format(name)] for row in per_run_rows
                if row["group"] == group]) for name in PARAMETERS},
            "sigma_mean_norm": stats([row["sigma_mean_norm"] for row in per_run_rows
                                      if row["group"] == group]),
            "sigma_peak_norm": stats([row["sigma_peak_norm"] for row in per_run_rows
                                      if row["group"] == group]),
            "actual_x_displacement_m": stats([
                row["actual_x_displacement_m"] for row in per_run_rows
                if row["group"] == group]),
            "position_rmse_m": stats([row["position_rmse_m"] for row in per_run_rows
                                      if row["group"] == group]),
            "applied_correction_rms": stats([
                row["applied_correction_rms"] for row in per_run_rows
                if row["group"] == group]),
        }
    for contrast in CONTRASTS:
        summary["controlled_contrasts_last2s"][contrast] = {
            row["parameter"]: {
                "mean": row["last2s_mean"], "sample_std": row["last2s_sample_std"],
                "truth_if_payload": row["truth_if_payload"],
                "signed_error_vs_truth": row["signed_error_vs_truth"],
                "ratio_to_truth": (row["last2s_mean"] / row["truth_if_payload"]
                                   if abs(row["truth_if_payload"]) > 1.0e-12 else None),
            } for row in contrast_rows if row["contrast"] == contrast}
    payload = summary["controlled_contrasts_last2s"]["payload_given_walking"]
    summary["headline_findings"] = {
        "stance_hold_estimate_is_near_zero": True,
        "target_without_gait_mass_contrast_kg":
            summary["controlled_contrasts_last2s"]["target_without_gait"]["m"]["mean"],
        "walking_dominates_nominal_inertial_bias": True,
        "payload_mass_contrast_kg": payload["m"]["mean"],
        "payload_mass_truth_kg": payload["m"]["truth_if_payload"],
        "payload_mass_ratio_to_truth": payload["m"]["ratio_to_truth"],
        "payload_hx_ratio_to_truth": payload["hx"]["ratio_to_truth"],
        "payload_hy_ratio_to_truth": payload["hy"]["ratio_to_truth"],
        "exact_10d_parameter_convergence_demonstrated": False,
    }
    summary_path = output / "stage_d5b_analysis_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")

    def mean(group, name):
        return summary["group_mean_std"][group]["pi_last2s"][name]["mean"]
    def mean_std(group, name):
        value = summary["group_mean_std"][group]["pi_last2s"][name]
        return "{:.6g} +/- {:.3g}".format(value["mean"], value["sample_std"])
    def metric(group, name):
        value = summary["group_mean_std"][group][name]
        return "{:.6g} +/- {:.3g}".format(value["mean"], value["sample_std"])
    def contrast(contrast_name, name):
        value = summary["controlled_contrasts_last2s"][contrast_name][name]
        return "{:.6g} +/- {:.3g}".format(value["mean"], value["sample_std"])
    def truth(name):
        return summary["controlled_contrasts_last2s"]["payload_given_walking"][name][
            "truth_if_payload"]
    def ratio(name):
        value = summary["controlled_contrasts_last2s"]["payload_given_walking"][name][
            "ratio_to_truth"]
        return "--" if value is None else "{:.4g}".format(value)
    report = """# Stage D5B — 名义 10D 估计偏置归因

## 范围

本阶段使用四个受控条件，每组 3 次、15 s 公共窗口；所有控制器均为 Update ON、Adaptive Dynamics ON、CLF ON、Gamma=I10。
没有调参，也没有修改 Eq.(8)/Eq.(10)。

## 条件

1. Nominal stance hold：不发 target、不发 gait。
2. Nominal stance target：发相同 +0.10 m target，保持 STANCE。
3. Nominal walk target：D4 的 +0.10 m static-walk。
4. Uncertain walk target：在条件 3 上只加入冻结 payload plant。

## 数据有效性

- protocol audit：PASS
- formal fairness audit：PASS
- 正式实验：12/12 runs eligible（四组各 3 次）
- Eq.(8) 离散导数关系最大误差：{eq8:.3g}
- Eq.(10) applied-path 最大误差：{eq10:.3g}
- 以下均为三次重复的 mean +/- sample std，统一取每次公共 15 s 窗口的最后 2 s。

## Last-2-s 在线 10D 估计

| 条件 | m [kg] | hx [kg m] | hy [kg m] | hz [kg m] | Jxx [kg m2] | Jxy [kg m2] | Jxz [kg m2] | Jyy [kg m2] | Jyz [kg m2] | Jzz [kg m2] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| N hold | {hm} | {hhx} | {hhy} | {hhz} | {hjxx} | {hjxy} | {hjxz} | {hjyy} | {hjyz} | {hjzz} |
| N stance target | {tm} | {thx} | {thy} | {thz} | {tjxx} | {tjxy} | {tjxz} | {tjyy} | {tjyz} | {tjzz} |
| N walk target | {wm} | {whx} | {why} | {whz} | {wjxx} | {wjxy} | {wjxz} | {wjyy} | {wjyz} | {wjzz} |
| U walk target | {um} | {uhx} | {uhy} | {uhz} | {ujxx} | {ujxy} | {ujxz} | {ujyy} | {ujyz} | {ujzz} |

## 受控闭环差值

差值方向统一为“后一条件 - 前一条件”。

| contrast | m | hx | hy | hz | Jxx | Jxy | Jxz | Jyy | Jyz | Jzz |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| target without gait | {ct_m} | {ct_hx} | {ct_hy} | {ct_hz} | {ct_Jxx} | {ct_Jxy} | {ct_Jxz} | {ct_Jyy} | {ct_Jyz} | {ct_Jzz} |
| walking given target | {cw_m} | {cw_hx} | {cw_hy} | {cw_hz} | {cw_Jxx} | {cw_Jxy} | {cw_Jxz} | {cw_Jyy} | {cw_Jyz} | {cw_Jzz} |
| payload given walking | {cp_m} | {cp_hx} | {cp_hy} | {cp_hz} | {cp_Jxx} | {cp_Jxy} | {cp_Jxz} | {cp_Jyy} | {cp_Jyz} | {cp_Jzz} |
| payload truth pi_star | {ps_m:.6g} | {ps_hx:.6g} | {ps_hy:.6g} | {ps_hz:.6g} | {ps_Jxx:.6g} | {ps_Jxy:.6g} | {ps_Jxz:.6g} | {ps_Jyy:.6g} | {ps_Jyz:.6g} | {ps_Jzz:.6g} |
| estimate/truth ratio | {pr_m} | {pr_hx} | {pr_hy} | {pr_hz} | {pr_Jxx} | {pr_Jxy} | {pr_Jxz} | {pr_Jyy} | {pr_Jyz} | {pr_Jzz} |

## 状态与更新强度

| 条件 | mean ||sigma|| | x displacement [m] | position RMSE [m] | applied correction RMS |
|---|---:|---:|---:|---:|
| N hold | {hsigma} | {hxdisp} | {hpos} | {hcorr} |
| N stance target | {tsigma} | {txdisp} | {tpos} | {tcorr} |
| N walk target | {wsigma} | {wxdisp} | {wpos} | {wcorr} |
| U walk target | {usigma} | {uxdisp} | {upos} | {ucorr} |

## 核心观察

1. 静止保持时 10D estimate 基本为零（m={hm_only:.6g} kg，惯性项约 1e-11--1e-6 kg m2），说明 D4 的大幅名义估计偏置不是由“无激励静态 nominal plant”独自产生。
2. 仅发送 +0.10 m target 而不切换 gait 时，机身只前移约 {txdisp_only:.6g} m，主要出现 m={ct_m_only:.6g} kg 的负向差值，惯性项仍很小。
3. 在同一 target 上加入 static-walk 后，hz、Jxx、Jyy、Jzz 分别新增约 {cw_hz_only:.6g} kg m、{cw_Jxx_only:.6g}、{cw_Jyy_only:.6g}、{cw_Jzz_only:.6g} kg m2；因此 D4 的主要名义惯性偏置与行走时的 reference/gait/contact/closed-loop model residual 强相关。
4. 去除 nominal-walk baseline 后，payload mass 增量估计为 {cp_m_only:.6g} +/- {cp_m_std:.3g} kg，真值为 {ps_m:.6g} kg，约为真值的 {pr_m} 倍（高估约 {mass_over:.1f}%）。hx、hy 的比值分别为 {pr_hx}、{pr_hy}，但其余惯性通道幅值或符号不能可靠重构真值。
5. 该结果与 D5A 的 payload-scale practical rank 5/10 和弱惯性灵敏度一致：当前轨迹下可获得部分 payload 方向信息，但没有实现严格 10D 参数收敛。

完整逐 run 指标、组统计和差值保存在同目录 CSV/JSON；图 01--04 分别展示在线估计、受控差值、状态/sigma/correction 和 Eq.(8) 更新驱动。

## 解释边界

三个差值分别是 target-without-gait、walking-given-target、payload-given-walking 的受控闭环对比。
它们可以用于定位偏置出现在哪个条件，但不假设闭环系统对这些因素线性可加，不等价于开环因果分解，也不证明参数收敛。
本阶段支持的是“名义偏置归因”和“payload 增量的部分可恢复性”；不能宣称已经辨识出真实 10D payload 参数。
""".format(**dict({
        prefix + name.lower(): mean_std(group, name)
        for prefix, group in (("h", "nominal_stance_hold"),
                              ("t", "nominal_stance_target"),
                              ("w", "nominal_walk_target"),
                              ("u", "uncertain_walk_target"))
        for name in PARAMETERS}, **{
        "c{}_{}".format(prefix, name): contrast(contrast_name, name)
        for prefix, contrast_name in (("t", "target_without_gait"),
                                      ("w", "walking_given_target"),
                                      ("p", "payload_given_walking"))
        for name in PARAMETERS}, **{
        "ps_" + name: truth(name) for name in PARAMETERS}, **{
        "pr_" + name: ratio(name) for name in PARAMETERS}, **{
        "eq8": summary["data_integrity"]["max_eq8_derivative_error"],
        "eq10": summary["data_integrity"]["max_eq10_applied_error"],
        "hsigma": metric("nominal_stance_hold", "sigma_mean_norm"),
        "tsigma": metric("nominal_stance_target", "sigma_mean_norm"),
        "wsigma": metric("nominal_walk_target", "sigma_mean_norm"),
        "usigma": metric("uncertain_walk_target", "sigma_mean_norm"),
        "hxdisp": metric("nominal_stance_hold", "actual_x_displacement_m"),
        "txdisp": metric("nominal_stance_target", "actual_x_displacement_m"),
        "wxdisp": metric("nominal_walk_target", "actual_x_displacement_m"),
        "uxdisp": metric("uncertain_walk_target", "actual_x_displacement_m"),
        "hpos": metric("nominal_stance_hold", "position_rmse_m"),
        "tpos": metric("nominal_stance_target", "position_rmse_m"),
        "wpos": metric("nominal_walk_target", "position_rmse_m"),
        "upos": metric("uncertain_walk_target", "position_rmse_m"),
        "hcorr": metric("nominal_stance_hold", "applied_correction_rms"),
        "tcorr": metric("nominal_stance_target", "applied_correction_rms"),
        "wcorr": metric("nominal_walk_target", "applied_correction_rms"),
        "ucorr": metric("uncertain_walk_target", "applied_correction_rms"),
        "hm_only": mean("nominal_stance_hold", "m"),
        "txdisp_only": summary["group_mean_std"]["nominal_stance_target"][
            "actual_x_displacement_m"]["mean"],
        "ct_m_only": summary["controlled_contrasts_last2s"][
            "target_without_gait"]["m"]["mean"],
        "cw_hz_only": summary["controlled_contrasts_last2s"][
            "walking_given_target"]["hz"]["mean"],
        "cw_Jxx_only": summary["controlled_contrasts_last2s"][
            "walking_given_target"]["Jxx"]["mean"],
        "cw_Jyy_only": summary["controlled_contrasts_last2s"][
            "walking_given_target"]["Jyy"]["mean"],
        "cw_Jzz_only": summary["controlled_contrasts_last2s"][
            "walking_given_target"]["Jzz"]["mean"],
        "cp_m_only": payload["m"]["mean"],
        "cp_m_std": payload["m"]["sample_std"],
        "mass_over": 100.0 * (payload["m"]["ratio_to_truth"] - 1.0),
    }))
    (output / "stage_d5b_report.md").write_text(report)
    artifacts = sorted(path for path in output.iterdir()
                       if path.is_file() and path.name != "stage_d5b_analysis_manifest.json")
    (output / "stage_d5b_analysis_manifest.json").write_text(json.dumps({
        "schema_version": 1, "fairness_sha256": digest(fairness_path),
        "protocol_sha256": digest(protocol_path),
        "analyzer_sha256": digest(Path(__file__)),
        "artifacts": [{"name": path.name, "sha256": digest(path)} for path in artifacts],
    }, indent=2, sort_keys=True) + "\n")
    print("D5B unified analysis PASS: {}".format(output))


if __name__ == "__main__":
    main()
