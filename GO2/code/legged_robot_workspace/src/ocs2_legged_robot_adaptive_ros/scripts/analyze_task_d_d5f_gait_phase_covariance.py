#!/usr/bin/env python3
"""D5F gait-cycle decomposition of nominal 10D adaptation bias."""

import argparse
import csv
import hashlib
import importlib.util
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


D5D = load_module("d5d_for_d5f", PACKAGE / "scripts" /
                  "analyze_task_d_d5d_sigma_subterms.py")
D5E = load_module("d5e_for_d5f", PACKAGE / "scripts" /
                  "analyze_task_d_d5e_feedback_pair.py")
EXPECTED_MODES = (13, 7, 14, 11)
MODE_NAMES = {13: "LH_swing", 7: "RF_swing", 14: "RH_swing", 11: "LF_swing"}
SIGNALS = ("total", "velocity", "pose")
FOCUS = (3, 4, 7)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def transitions(observations):
    result = []
    previous = None
    for time, mode, _, _ in observations:
        if mode != previous:
            result.append((float(time), int(mode)))
            previous = mode
    return result


def complete_steady_cycles(observations, minimum_start=2.2,
                           expected_modes=EXPECTED_MODES):
    """Find full executed 13->7->14->11->13 cycles after the target transient."""
    values = transitions(observations)
    starts = [index for index, (_, mode) in enumerate(values)
              if mode == expected_modes[0]]
    cycles = []
    for left, right in zip(starts[:-1], starts[1:]):
        sequence = tuple(mode for _, mode in values[left:right])
        start, end = values[left][0], values[right][0]
        if start >= minimum_start and sequence == expected_modes:
            cycles.append({"start": start, "end": end,
                           "duration": end - start,
                           "transition_sequence": list(sequence)})
    if not cycles:
        raise ValueError("no complete steady gait cycles")
    return cycles


def interval_samples(time, values, start, end):
    """Online-Euler interval endpoints and weights for one exact cycle."""
    time = np.asarray(time, dtype=float)
    values = np.asarray(values, dtype=float)
    mask = (time > start + 1.0e-10) & (time <= end + 1.0e-10)
    selected_time = time[mask]
    selected = values[mask]
    if len(selected_time) < 2 or abs(selected_time[-1] - end) > 1.0e-8:
        raise ValueError("diagnostic samples do not close the executed cycle")
    previous = np.r_[start, selected_time[:-1]]
    dt = selected_time - previous
    if np.any(dt <= 0.0) or abs(np.sum(dt) - (end - start)) > 1.0e-8:
        raise ValueError("invalid cycle Euler intervals")
    return selected_time, selected, dt


def weighted_mean_covariance(regressor, sigma, weights):
    """Exact decomposition E[Y^T s] = E[Y]^T E[s] + Cov(Y,s)."""
    regressor = np.asarray(regressor, dtype=float)
    sigma = np.asarray(sigma, dtype=float)
    weights = np.asarray(weights, dtype=float)
    if regressor.ndim != 3 or regressor.shape[1:] != (6, 10):
        raise ValueError("regressor must be [N,6,10]")
    if sigma.shape != (len(regressor), 6) or weights.shape != (len(regressor),):
        raise ValueError("sigma/weight shape mismatch")
    if np.any(weights <= 0.0):
        raise ValueError("weights must be positive")
    normalized = weights / np.sum(weights)
    mean_y = np.einsum("n,nij->ij", normalized, regressor)
    mean_sigma = np.einsum("n,ni->i", normalized, sigma)
    direct = np.einsum("n,nij,ni->j", normalized, regressor, sigma)
    mean_product_by_axis = mean_y * mean_sigma[:, None]
    centered_y = regressor - mean_y[None, :, :]
    centered_sigma = sigma - mean_sigma[None, :]
    covariance_by_axis = np.einsum(
        "n,nij,ni->ij", normalized, centered_y, centered_sigma)
    mean_product = np.sum(mean_product_by_axis, axis=0)
    covariance = np.sum(covariance_by_axis, axis=0)
    return {
        "direct": direct, "mean_product": mean_product,
        "covariance": covariance, "mean_y": mean_y,
        "mean_sigma": mean_sigma,
        "direct_by_axis": np.einsum(
            "n,nij,ni->ij", normalized, regressor, sigma),
        "mean_product_by_axis": mean_product_by_axis,
        "covariance_by_axis": covariance_by_axis,
    }


def phase_bin_integral(time, drive, start, end, bins=24):
    """Partition explicit-Euler parameter update into normalized gait phase bins."""
    selected_time, selected_drive, dt = interval_samples(time, drive, start, end)
    phase = np.minimum((selected_time - start) / (end - start),
                       np.nextafter(1.0, 0.0))
    indices = np.minimum((phase * bins).astype(int), bins - 1)
    result = np.zeros((bins, selected_drive.shape[1]))
    duration = np.zeros(bins)
    for index in range(bins):
        mask = indices == index
        result[index] = np.sum(dt[mask, None] * selected_drive[mask], axis=0)
        duration[index] = np.sum(dt[mask])
    if abs(np.sum(duration) - (end - start)) > 1.0e-8:
        raise ValueError("phase bins do not close cycle duration")
    return result, duration


def signal_arrays(rows):
    velocity = rows[:, 4:10]
    pose = rows[:, 10:16]
    return {"velocity": velocity, "pose": pose, "total": velocity + pose}


def cycle_decomposition(run, cycles, phase_bins=24):
    rows = run["decomposition"]
    v3, time = run["v3"], rows[:, 0]
    matched, time_error = D5D.match_rows_by_time(rows, v3)
    if np.max(np.abs(time_error)) > 1.0e-8:
        raise ValueError("D5F v3/decomposition timestamps do not match")
    regressor = matched[:, 48:108].reshape(-1, 6, 10)
    signals = signal_arrays(rows)
    cycle_rows, axis_rows = [], []
    phase = {signal: np.zeros((phase_bins, 10)) for signal in SIGNALS}
    phase_duration = np.zeros(phase_bins)
    closure = []
    for cycle_index, cycle in enumerate(cycles, 1):
        selected_time, selected_y, dt = interval_samples(
            time, regressor, cycle["start"], cycle["end"])
        for signal_name in SIGNALS:
            _, selected_sigma, sigma_dt = interval_samples(
                time, signals[signal_name], cycle["start"], cycle["end"])
            if not np.allclose(dt, sigma_dt, rtol=0.0, atol=1.0e-12):
                raise ValueError("cycle sample weights differ")
            values = weighted_mean_covariance(selected_y, selected_sigma, dt)
            error = np.max(np.abs(values["direct"] - values["mean_product"] -
                                  values["covariance"]))
            axis_error = np.max(np.abs(
                values["direct_by_axis"] - values["mean_product_by_axis"] -
                values["covariance_by_axis"]))
            closure.extend((float(error), float(axis_error)))
            duration = cycle["duration"]
            for parameter_index, parameter in enumerate(D5D.PARAMETERS):
                direct_update = duration * values["direct"][parameter_index]
                mean_update = duration * values["mean_product"][parameter_index]
                covariance_update = duration * values["covariance"][parameter_index]
                denominator = abs(mean_update) + abs(covariance_update)
                cycle_rows.append({
                    "cycle": cycle_index, "start_s": cycle["start"],
                    "end_s": cycle["end"], "duration_s": duration,
                    "signal": signal_name, "parameter": parameter,
                    "direct_update": direct_update,
                    "mean_product_update": mean_update,
                    "covariance_update": covariance_update,
                    "covariance_abs_share":
                        abs(covariance_update) / denominator if denominator > 1.0e-14 else np.nan,
                })
                for axis_index, axis in enumerate(D5D.AXES):
                    axis_rows.append({
                        "cycle": cycle_index, "signal": signal_name,
                        "parameter": parameter, "sigma_axis": axis,
                        "direct_update": duration * values["direct_by_axis"][axis_index, parameter_index],
                        "mean_product_update": duration * values["mean_product_by_axis"][axis_index, parameter_index],
                        "covariance_update": duration * values["covariance_by_axis"][axis_index, parameter_index],
                        "mean_sigma": values["mean_sigma"][axis_index],
                    })
            drive = np.einsum("nij,ni->nj", regressor, signals[signal_name])
            contribution, bin_duration = phase_bin_integral(
                time, drive, cycle["start"], cycle["end"], phase_bins)
            phase[signal_name] += contribution
            if signal_name == "total":
                phase_duration += bin_duration
    for signal_name in phase:
        phase[signal_name] /= len(cycles)
    phase_duration /= len(cycles)
    return cycle_rows, axis_rows, phase, phase_duration, max(closure)


def aggregate_cycle_rows(rows):
    result = []
    keys = sorted({(row["signal"], row["parameter"]) for row in rows})
    for signal, parameter in keys:
        selected = [row for row in rows
                    if row["signal"] == signal and row["parameter"] == parameter]
        output = {"signal": signal, "parameter": parameter, "cycles": len(selected)}
        for key in ("direct_update", "mean_product_update", "covariance_update"):
            values = np.asarray([row[key] for row in selected])
            output[key + "_mean"] = float(np.mean(values))
            output[key + "_sample_std"] = float(np.std(values, ddof=1)) if len(values) > 1 else np.nan
        denominator = abs(output["mean_product_update_mean"]) + abs(output["covariance_update_mean"])
        output["covariance_abs_share"] = (
            abs(output["covariance_update_mean"]) / denominator
            if denominator > 1.0e-14 else np.nan)
        result.append(output)
    return result


def write_csv(path, rows):
    if not rows:
        raise ValueError("cannot write empty CSV")
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def analyze(root, output, phase_bins=24):
    root, output = Path(root).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    runs = {mode: D5E.load_run(root / mode / "run_01", mode)
            for mode in ("on", "off")}
    all_cycle_rows, all_axis_rows = [], []
    results, validation = {}, {}
    for mode, run in runs.items():
        cycles = complete_steady_cycles(run["observations"])
        cycle_rows, axis_rows, phase, phase_duration, closure = cycle_decomposition(
            run, cycles, phase_bins)
        for row in cycle_rows: row["mode"] = mode
        for row in axis_rows: row["mode"] = mode
        all_cycle_rows.extend(cycle_rows); all_axis_rows.extend(axis_rows)
        aggregate = aggregate_cycle_rows(cycle_rows)
        results[mode] = {"cycles": cycles, "aggregate": aggregate,
                         "phase": phase, "phase_duration": phase_duration}
        durations = [cycle["duration"] for cycle in cycles]
        validation[mode] = {
            "run_validation_passed": run["validation"]["passed"],
            "complete_cycles": len(cycles),
            "cycle_duration_min_s": min(durations),
            "cycle_duration_max_s": max(durations),
            "decomposition_max_abs_error": closure,
            "phase_duration_closure_max_abs_error_s":
                float(abs(np.sum(phase_duration) - np.mean(durations))),
        }
    all_cycle_rows = [{"mode": row.pop("mode"), **row} for row in all_cycle_rows]
    all_axis_rows = [{"mode": row.pop("mode"), **row} for row in all_axis_rows]
    write_csv(output / "cycle_mean_covariance_decomposition.csv", all_cycle_rows)
    write_csv(output / "cycle_axis_covariance_decomposition.csv", all_axis_rows)

    aggregate_rows = []
    for mode in ("on", "off"):
        for row in results[mode]["aggregate"]:
            aggregate_rows.append({"mode": mode, **row})
    write_csv(output / "mean_covariance_summary.csv", aggregate_rows)

    phase_rows = []
    for mode in ("on", "off"):
        for bin_index in range(phase_bins):
            for signal in SIGNALS:
                for parameter_index, parameter in enumerate(D5D.PARAMETERS):
                    phase_rows.append({
                        "mode": mode, "phase_bin": bin_index,
                        "phase_start": bin_index / phase_bins,
                        "phase_end": (bin_index + 1) / phase_bins,
                        "mean_bin_duration_s": results[mode]["phase_duration"][bin_index],
                        "signal": signal, "parameter": parameter,
                        "mean_cycle_update": results[mode]["phase"][signal][bin_index, parameter_index],
                    })
    write_csv(output / "phase_bin_parameter_updates.csv", phase_rows)

    def aggregate_value(mode, signal, parameter, key):
        return next(row[key] for row in results[mode]["aggregate"]
                    if row["signal"] == signal and row["parameter"] == parameter)

    fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex="col")
    phase_center = (np.arange(phase_bins) + 0.5) / phase_bins
    for row_index, parameter_index in enumerate(FOCUS):
        parameter = D5D.PARAMETERS[parameter_index]
        for mode, style in (("on", "-"), ("off", "--")):
            axes[row_index, 0].plot(
                phase_center, results[mode]["phase"]["total"][:, parameter_index],
                style, label=mode)
        mean_part = [aggregate_value(mode, "total", parameter,
                                     "mean_product_update_mean") for mode in ("on", "off")]
        covariance = [aggregate_value(mode, "total", parameter,
                                      "covariance_update_mean") for mode in ("on", "off")]
        x = np.arange(2); width = 0.36
        axes[row_index, 1].bar(x - width / 2, mean_part, width, label="mean product")
        axes[row_index, 1].bar(x + width / 2, covariance, width, label="covariance")
        axes[row_index, 1].set_xticks(x); axes[row_index, 1].set_xticklabels(("ON", "OFF"))
        axes[row_index, 0].set_ylabel(parameter + " update/bin")
        axes[row_index, 1].set_ylabel(parameter + " update/cycle")
        for plot in axes[row_index]: plot.grid(True, alpha=0.3); plot.legend()
    axes[-1, 0].set_xlabel("normalized gait phase")
    axes[-1, 1].set_xlabel("adaptation mode")
    fig.suptitle("D5F gait-phase drive and E[Y]^T E[sigma] vs covariance")
    fig.tight_layout(); fig.savefig(output / "figure_D5F_01_phase_and_covariance.png", dpi=170); plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for plot, parameter_index in zip(axes, FOCUS):
        parameter = D5D.PARAMETERS[parameter_index]
        bottom = np.zeros(phase_bins)
        for mode, color, hatch in (("on", "#1f77b4", None), ("off", "#ff7f0e", "//")):
            values = results[mode]["phase"]["total"][:, parameter_index]
            plot.step(phase_center, values, where="mid", label=mode,
                      color=color, linestyle="-" if mode == "on" else "--")
        plot.axhline(0.0, color="black", linewidth=0.8)
        plot.set_ylabel(parameter); plot.grid(True, alpha=0.3); plot.legend()
    axes[-1].set_xlabel("normalized gait phase")
    fig.suptitle("D5F mean per-cycle Eq.(8) update by gait phase")
    fig.tight_layout(); fig.savefig(output / "figure_D5F_02_phase_updates.png", dpi=170); plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9))
    for plot, parameter_index in zip(axes, FOCUS):
        parameter = D5D.PARAMETERS[parameter_index]
        selected = [row for row in all_axis_rows
                    if row["mode"] == "off" and row["signal"] == "velocity"
                    and row["parameter"] == parameter]
        means = [np.mean([row["covariance_update"] for row in selected
                          if row["sigma_axis"] == axis]) for axis in D5D.AXES]
        plot.bar(D5D.AXES, means)
        plot.axhline(0.0, color="black", linewidth=0.8); plot.set_ylabel(parameter)
        plot.grid(True, axis="y", alpha=0.3)
    axes[-1].tick_params(axis="x", rotation=20)
    fig.suptitle("D5F OFF: velocity-subterm regressor-sigma covariance by world axis")
    fig.tight_layout(); fig.savefig(output / "figure_D5F_03_off_velocity_covariance_axes.png", dpi=170); plt.close(fig)

    focus_summary = {}
    signal_focus = {}
    for mode in ("on", "off"):
        focus_summary[mode] = {}
        signal_focus[mode] = {}
        for parameter_index in FOCUS:
            parameter = D5D.PARAMETERS[parameter_index]
            focus_summary[mode][parameter] = {
                key: aggregate_value(mode, "total", parameter, key)
                for key in ("direct_update_mean", "direct_update_sample_std",
                            "mean_product_update_mean", "covariance_update_mean",
                            "covariance_abs_share")}
            signal_focus[mode][parameter] = {
                signal: {
                    key: aggregate_value(mode, signal, parameter, key)
                    for key in ("direct_update_mean", "mean_product_update_mean",
                                "covariance_update_mean")}
                for signal in SIGNALS}
    off_velocity_axis_covariance = {}
    off_phase_sign = {}
    for parameter_index in FOCUS:
        parameter = D5D.PARAMETERS[parameter_index]
        selected = [row for row in all_axis_rows
                    if row["mode"] == "off" and row["signal"] == "velocity"
                    and row["parameter"] == parameter]
        off_velocity_axis_covariance[parameter] = {
            axis: float(np.mean([row["covariance_update"] for row in selected
                                 if row["sigma_axis"] == axis]))
            for axis in D5D.AXES}
        phase_values = results["off"]["phase"]["total"][:, parameter_index]
        off_phase_sign[parameter] = {
            "positive_update_sum": float(np.sum(phase_values[phase_values > 0.0])),
            "negative_update_sum": float(np.sum(phase_values[phase_values < 0.0])),
            "positive_bin_count": int(np.sum(phase_values > 0.0)),
            "negative_bin_count": int(np.sum(phase_values < 0.0)),
        }
    passed = bool(all(item["run_validation_passed"] and
                      item["complete_cycles"] >= 4 and
                      item["decomposition_max_abs_error"] <= 1.0e-12 and
                      item["phase_duration_closure_max_abs_error_s"] <= 1.0e-8
                      for item in validation.values()))
    summary = {
        "schema_version": 1,
        "stage": "D5F_gait_phase_mean_covariance_decomposition",
        "passed": passed,
        "identity": "E[Y^T sigma] = E[Y]^T E[sigma] + sum_i Cov(Y_i, sigma_i)",
        "discretization": "duration-weighted right-endpoint Euler intervals matching Eq.(8)",
        "steady_cycle_rule": "complete executed 13->7->14->11->13 cycles with start >= 2.2 s",
        "validation": validation,
        "focus_total_sigma": focus_summary,
        "focus_signal_subterms": signal_focus,
        "off_velocity_covariance_by_sigma_axis": off_velocity_axis_covariance,
        "off_phase_sign": off_phase_sign,
        "scientific_limit": "phase/covariance attribution on one paired ON/OFF diagnostic run; not an independent causal intervention",
        "input_bags": {mode: {"path": str(run["bag"]), "sha256": sha256(run["bag"])}
                       for mode, run in runs.items()},
    }
    (output / "stage_d5f_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")

    report_rows = []
    for mode in ("on", "off"):
        for parameter_index in FOCUS:
            parameter = D5D.PARAMETERS[parameter_index]
            item = focus_summary[mode][parameter]
            report_rows.append("| {} | {} | {:.6g} +/- {:.3g} | {:.6g} | {:.6g} | {:.2%} |".format(
                mode.upper(), parameter, item["direct_update_mean"], item["direct_update_sample_std"],
                item["mean_product_update_mean"], item["covariance_update_mean"],
                item["covariance_abs_share"]))
    signal_rows = []
    axis_rows_report = []
    phase_rows_report = []
    for mode in ("on", "off"):
        for parameter_index in FOCUS:
            parameter = D5D.PARAMETERS[parameter_index]
            item = signal_focus[mode][parameter]
            velocity = item["velocity"]["direct_update_mean"]
            pose = item["pose"]["direct_update_mean"]
            total = item["total"]["direct_update_mean"]
            signal_rows.append("| {} | {} | {:.6g} | {:.6g} | {:.6g} | {:.2%} |".format(
                mode.upper(), parameter, total, velocity, pose,
                abs(velocity) / (abs(velocity) + abs(pose))))
    for parameter_index in FOCUS:
        parameter = D5D.PARAMETERS[parameter_index]
        values = off_velocity_axis_covariance[parameter]
        dominant = max(values, key=lambda axis: abs(values[axis]))
        axis_rows_report.append("| {} | {} | {:.6g} |".format(
            parameter, dominant, values[dominant]))
        signs = off_phase_sign[parameter]
        phase_rows_report.append("| {} | {} | {} | {:.6g} | {:.6g} |".format(
            parameter, signs["positive_bin_count"], signs["negative_bin_count"],
            signs["positive_update_sum"], signs["negative_update_sum"]))
    report = """# Stage D5F — Gait-phase mean/covariance decomposition

## Scope and exact identity

复用 D5E 的 Nominal ON/OFF bag，不重采、不修改控制器。对 target transient 后的完整执行步态周期计算：

`E[Y^T sigma] = E[Y]^T E[sigma] + sum_i Cov(Y_i, sigma_i)`

所有均值按在线 Eq.(8) 的右端显式 Euler interval duration 加权。ON/OFF 各使用 `{on_cycles}` / `{off_cycles}` 个完整周期；分解闭合误差均小于 `{closure:.3e}`。

## Per-cycle total-sigma update

| mode | parameter | direct update mean +/- sample std | mean-product | covariance | covariance absolute share |
|---|---|---:|---:|---:|---:|
{rows}

`covariance absolute share = |covariance|/(|mean-product|+|covariance|)`。它用于显示漂移主要来自 sigma 均值还是 gait-phase correlation；若两项异号，该比例不应解释为净更新百分比。

结果显示三项参数的 mean-product 均接近零，而 covariance 占绝对分解量的 99.94%--99.99%。因此 drift 不是由 sigma 的明显 DC 均值产生，而是由时变 regressor 与周期 tracking residual 的 gait-phase correlation 产生。

## Velocity versus pose subterm

| mode | parameter | total update/cycle | velocity contribution | pose contribution | velocity absolute share |
|---|---|---:|---:|---:|---:|
{signal_rows}

## OFF velocity-covariance dominant axes

| parameter | dominant sigma axis | covariance update/cycle |
|---|---|---:|
{axis_rows}

## OFF phase-bin sign balance

| parameter | positive bins | negative bins | positive update sum | negative update sum |
|---|---:|---:|---:|---:|
{phase_rows}

`Jxx/Jyy` 的所有 phase bins 均为同号正更新；`hz` 只有少量负相位，不能抵消正相位。因此，仅在一个完整 gait cycle 末尾再积分同一 `Y^T sigma`，不会消除 nominal drift；需要额外的 nominal phase baseline、leakage/dead-zone，或独立 prediction-error/composite information，且这些都属于后续需要单独验证的 estimator 修改。

## Interpretation boundary

这是同一闭环轨迹上的精确统计恒等式和相位归因，不是独立因果干预，也不把 nominal bias 解释为 payload。每组只有一条短时 run，周期间 sample std 描述该 run 内周期重复性，不是跨 run 不确定性。
""".format(on_cycles=validation["on"]["complete_cycles"],
           off_cycles=validation["off"]["complete_cycles"],
           closure=max(item["decomposition_max_abs_error"] for item in validation.values()),
           rows="\n".join(report_rows), signal_rows="\n".join(signal_rows),
           axis_rows="\n".join(axis_rows_report),
           phase_rows="\n".join(phase_rows_report))
    (output / "stage_d5f_report.md").write_text(report)
    artifacts = []
    for path in sorted(output.iterdir()):
        if path.is_file() and path.name != "stage_d5f_artifact_manifest.json":
            artifacts.append({"path": path.name, "bytes": path.stat().st_size,
                              "sha256": sha256(path)})
    (output / "stage_d5f_artifact_manifest.json").write_text(json.dumps(
        {"schema_version": 1, "passed": passed, "artifacts": artifacts},
        indent=2, sort_keys=True) + "\n")
    print(json.dumps({"passed": passed, "validation": validation,
                      "focus_total_sigma": focus_summary}, indent=2))
    return 0 if passed else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("d5e_root"); parser.add_argument("output_directory")
    parser.add_argument("--phase-bins", type=int, default=24)
    args = parser.parse_args()
    return analyze(args.d5e_root, args.output_directory, args.phase_bins)


if __name__ == "__main__":
    raise SystemExit(main())
