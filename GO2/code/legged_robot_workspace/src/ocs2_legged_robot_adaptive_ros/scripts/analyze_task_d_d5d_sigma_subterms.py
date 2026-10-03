#!/usr/bin/env python3
"""D5D: exact offline decomposition of nominal-walking sigma subterms."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


DECOMP_TOPIC = "/task_d/payload_inertial_sigma_decomposition_sample"
DECOMP_LABEL = "payload_inertial_sigma_decomposition_v1_52_fields"
V3_TOPIC = "/task_d/payload_inertial_10d_v3_sample"
V3_LABEL = "payload_inertial_10d_v3_118_fields"
PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz", "Jyy", "Jyz", "Jzz")
AXES = ("linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z")
FOCUS = (3, 4, 7)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def schema_label(message):
    return message.layout.dim[0].label if message.layout.dim else ""


def read_bag(path):
    decomposition = []
    v3 = []
    labels = {DECOMP_TOPIC: set(), V3_TOPIC: set()}
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, _ in bag.read_messages(topics=[DECOMP_TOPIC, V3_TOPIC]):
            labels[topic].add(schema_label(message))
            values = np.asarray(message.data, dtype=float)
            expected = 52 if topic == DECOMP_TOPIC else 118
            if values.shape != (expected,) or not np.all(np.isfinite(values)):
                raise ValueError("{} has invalid/non-finite {}-field sample".format(topic, expected))
            (decomposition if topic == DECOMP_TOPIC else v3).append(values)
    if labels[DECOMP_TOPIC] != {DECOMP_LABEL}:
        raise ValueError("unexpected D5D schema labels: {}".format(labels[DECOMP_TOPIC]))
    if labels[V3_TOPIC] != {V3_LABEL}:
        raise ValueError("unexpected v3 schema labels: {}".format(labels[V3_TOPIC]))
    if len(decomposition) < 10 or len(v3) < 10:
        raise ValueError("insufficient D5D/v3 samples")
    decomposition = np.asarray(sorted(decomposition, key=lambda row: row[0]))
    v3 = np.asarray(sorted(v3, key=lambda row: row[0]))
    if np.any(np.diff(decomposition[:, 0]) <= 0.0):
        raise ValueError("D5D observation time is not strictly increasing")
    return decomposition, v3


def match_rows_by_time(primary, secondary, tolerance=1.0e-8):
    """Return secondary rows nearest to each primary observation timestamp."""
    indices = np.searchsorted(secondary[:, 0], primary[:, 0])
    indices = np.clip(indices, 0, len(secondary) - 1)
    previous = np.clip(indices - 1, 0, len(secondary) - 1)
    use_previous = (np.abs(secondary[previous, 0] - primary[:, 0]) <
                    np.abs(secondary[indices, 0] - primary[:, 0]))
    indices[use_previous] = previous[use_previous]
    error = secondary[indices, 0] - primary[:, 0]
    if np.max(np.abs(error)) > tolerance:
        raise ValueError("D5D and v3 timestamps do not match: max {}".format(np.max(np.abs(error))))
    return secondary[indices], error


def euler_contributions(time, rate):
    """Right-endpoint Euler contributions matching AdaptiveParameterEstimator."""
    time = np.asarray(time, dtype=float)
    rate = np.asarray(rate, dtype=float)
    if rate.shape[0] != time.size:
        raise ValueError("time/rate length mismatch")
    delta = np.zeros_like(rate)
    delta[1:] = np.diff(time)[:, None] * rate[1:]
    return delta, np.cumsum(delta, axis=0)


def time_segments(time):
    end = float(time[-1])
    return (
        ("pre_target_walk", 0.0, 1.2),
        ("target_ramp", 1.2, 1.4),
        ("first_post_ramp_cycle", 1.4, 2.6),
        ("steady_walk", 2.6, end + 1.0e-12),
    )


def segment_sum(time, delta, start, stop):
    mask = (time > start) & (time <= stop)
    return np.sum(delta[mask], axis=0) if np.any(mask) else np.zeros(delta.shape[1])


def write_csv(path, fieldnames, rows):
    with Path(path).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def analyze(run_dir, output):
    run_dir = Path(run_dir).resolve()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    bag_path = run_dir / "task_d_d5b.bag"
    if not bag_path.is_file():
        raise FileNotFoundError(bag_path)

    rows, v3 = read_bag(bag_path)
    v3_matched, topic_time_error = match_rows_by_time(rows, v3)
    time = rows[:, 0]
    velocity_sigma = rows[:, 4:10]
    pose_sigma = rows[:, 10:16]
    sigma = rows[:, 16:22]
    pi_hat = rows[:, 22:32]
    velocity_rate = rows[:, 32:42]
    pose_rate = rows[:, 42:52]
    total_rate = velocity_rate + pose_rate
    v3_rate = v3_matched[:, 32:42]
    v3_sigma = v3_matched[:, 42:48]

    velocity_delta, velocity_cumulative = euler_contributions(time, velocity_rate)
    pose_delta, pose_cumulative = euler_contributions(time, pose_rate)
    total_delta = velocity_delta + pose_delta
    total_cumulative = velocity_cumulative + pose_cumulative
    logged_delta = pi_hat - pi_hat[0]

    sigma_closure = float(np.max(np.abs(sigma - velocity_sigma - pose_sigma)))
    v3_sigma_error = float(np.max(np.abs(sigma - v3_sigma)))
    rate_closure = float(np.max(np.abs(total_rate - v3_rate)))
    euler_closure = float(np.max(np.abs(total_cumulative - logged_delta)))
    snapshot_time_error = float(np.max(np.abs(rows[:, 2])))
    timestamp_topic_error = float(np.max(np.abs(topic_time_error)))
    all_finite = bool(all(np.all(np.isfinite(value)) for value in
                          (rows, v3_matched, total_cumulative, logged_delta)))
    passed = bool(all_finite and sigma_closure <= 1.0e-12 and
                  v3_sigma_error <= 1.0e-12 and rate_closure <= 1.0e-10 and
                  euler_closure <= 1.0e-9 and snapshot_time_error <= 1.0e-6 and
                  timestamp_topic_error <= 1.0e-8)

    parameter_rows = []
    for index, name in enumerate(PARAMETERS):
        net_velocity = float(velocity_cumulative[-1, index])
        net_pose = float(pose_cumulative[-1, index])
        net_total = float(total_cumulative[-1, index])
        denominator = abs(net_velocity) + abs(net_pose)
        parameter_rows.append({
            "parameter": name,
            "logged_net_update": float(logged_delta[-1, index]),
            "velocity_error_net_update": net_velocity,
            "lambda_pose_error_net_update": net_pose,
            "reconstructed_net_update": net_total,
            "velocity_abs_share": abs(net_velocity) / denominator if denominator > 1.0e-14 else math.nan,
            "pose_abs_share": abs(net_pose) / denominator if denominator > 1.0e-14 else math.nan,
            "subterm_cancellation": 1.0 - abs(net_total) / denominator if denominator > 1.0e-14 else math.nan,
        })
    write_csv(output / "parameter_subterm_contributions.csv", list(parameter_rows[0]), parameter_rows)

    sigma_rows = []
    for index, name in enumerate(AXES):
        sigma_rows.append({
            "axis": name,
            "velocity_error_mean": float(np.mean(velocity_sigma[:, index])),
            "velocity_error_rms": float(np.sqrt(np.mean(velocity_sigma[:, index] ** 2))),
            "lambda_pose_error_mean": float(np.mean(pose_sigma[:, index])),
            "lambda_pose_error_rms": float(np.sqrt(np.mean(pose_sigma[:, index] ** 2))),
            "sigma_mean": float(np.mean(sigma[:, index])),
            "sigma_rms": float(np.sqrt(np.mean(sigma[:, index] ** 2))),
        })
    write_csv(output / "sigma_subterm_metrics.csv", list(sigma_rows[0]), sigma_rows)

    segment_rows = []
    for segment, start, stop in time_segments(time):
        velocity_sum = segment_sum(time, velocity_delta, start, stop)
        pose_sum = segment_sum(time, pose_delta, start, stop)
        for index, name in enumerate(PARAMETERS):
            segment_rows.append({
                "segment": segment, "start_s": start, "stop_s": min(stop, float(time[-1])),
                "parameter": name,
                "velocity_error_update": float(velocity_sum[index]),
                "lambda_pose_error_update": float(pose_sum[index]),
                "total_update": float(velocity_sum[index] + pose_sum[index]),
            })
    write_csv(output / "time_segment_subterm_contributions.csv", list(segment_rows[0]), segment_rows)

    series_fields = ["time"]
    series = {"time": time}
    for axis_index, axis in enumerate(AXES):
        for prefix, values in (("velocity_error", velocity_sigma),
                               ("lambda_pose_error", pose_sigma), ("sigma", sigma)):
            key = "{}_{}".format(prefix, axis)
            series_fields.append(key); series[key] = values[:, axis_index]
    for parameter_index, parameter in enumerate(PARAMETERS):
        for prefix, values in (("pi_hat", pi_hat),
                               ("cumulative_velocity", velocity_cumulative),
                               ("cumulative_pose", pose_cumulative),
                               ("cumulative_total", total_cumulative)):
            key = "{}_{}".format(prefix, parameter)
            series_fields.append(key); series[key] = values[:, parameter_index]
    with (output / "sigma_subterm_timeseries.csv").open("w", newline="") as stream:
        writer = csv.writer(stream); writer.writerow(series_fields)
        writer.writerows(zip(*(series[key] for key in series_fields)))

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for plot, index in zip(axes, (3, 4)):
        plot.plot(time, sigma[:, index], label="total sigma", color="black", linewidth=1.5)
        plot.plot(time, velocity_sigma[:, index], label="velocity error", alpha=0.85)
        plot.plot(time, pose_sigma[:, index], label="Lambda pose error", alpha=0.85)
        plot.axvspan(1.2, 1.4, color="grey", alpha=0.15)
        plot.set_ylabel(AXES[index]); plot.grid(True, alpha=0.3); plot.legend(ncol=3)
    axes[-1].set_xlabel("MPC time [s]")
    fig.suptitle("D5D nominal walking: angular sigma subterms")
    fig.tight_layout(); fig.savefig(output / "figure_D5D_01_angular_sigma_subterms.png", dpi=170); plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for plot, index in zip(axes, FOCUS):
        plot.plot(time, logged_delta[:, index], label="logged pi_hat change", color="black", linewidth=1.8)
        plot.plot(time, velocity_cumulative[:, index], label="from velocity error")
        plot.plot(time, pose_cumulative[:, index], label="from Lambda pose error")
        plot.plot(time, total_cumulative[:, index], "--", label="reconstructed total")
        plot.set_ylabel(PARAMETERS[index]); plot.grid(True, alpha=0.3); plot.legend(ncol=2)
    axes[-1].set_xlabel("MPC time [s]")
    fig.suptitle("D5D exact Eq.(8) cumulative subterm attribution")
    fig.tight_layout(); fig.savefig(output / "figure_D5D_02_cumulative_parameter_subterms.png", dpi=170); plt.close(fig)

    x = np.arange(len(PARAMETERS)); width = 0.38
    fig, axis = plt.subplots(figsize=(12, 5.5))
    axis.bar(x - width / 2, velocity_cumulative[-1], width, label="velocity error")
    axis.bar(x + width / 2, pose_cumulative[-1], width, label="Lambda pose error")
    axis.scatter(x, logged_delta[-1], color="black", marker="x", label="logged net update", zorder=3)
    axis.axhline(0.0, color="black", linewidth=0.8)
    axis.set_xticks(x); axis.set_xticklabels(PARAMETERS); axis.set_ylabel("net parameter update")
    axis.set_title("D5D endpoint parameter update by sigma subterm")
    axis.grid(True, axis="y", alpha=0.3); axis.legend(); fig.tight_layout()
    fig.savefig(output / "figure_D5D_03_endpoint_parameter_subterms.png", dpi=170); plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for plot, index in zip(axes, FOCUS):
        plot.plot(time, velocity_rate[:, index], label="velocity-error drive")
        plot.plot(time, pose_rate[:, index], label="pose-error drive")
        plot.plot(time, total_rate[:, index], color="black", alpha=0.65, label="total pi_dot")
        plot.set_ylabel("pi_dot " + PARAMETERS[index]); plot.grid(True, alpha=0.3); plot.legend(ncol=3)
    axes[-1].set_xlabel("MPC time [s]")
    fig.suptitle("D5D instantaneous Eq.(8) drive")
    fig.tight_layout(); fig.savefig(output / "figure_D5D_04_parameter_rate_subterms.png", dpi=170); plt.close(fig)

    summary = {
        "schema_version": 1,
        "stage": "D5D_nominal_walking_sigma_subterm_decomposition",
        "passed": passed,
        "scientific_scope": "diagnostic-only nominal walking; pi_star remains zero",
        "input": {"run_directory": str(run_dir), "bag": str(bag_path),
                  "bag_sha256": sha256(bag_path), "sample_count": int(len(time)),
                  "time_start_s": float(time[0]), "time_end_s": float(time[-1])},
        "closure": {"sigma_subterm_max_abs_error": sigma_closure,
                    "v3_sigma_max_abs_error": v3_sigma_error,
                    "pi_dot_subterm_max_abs_error": rate_closure,
                    "euler_accumulation_max_abs_error": euler_closure,
                    "snapshot_time_max_abs_error_s": snapshot_time_error,
                    "cross_topic_time_max_abs_error_s": timestamp_topic_error,
                    "all_finite": all_finite},
        "parameter_contributions": {row["parameter"]: row for row in parameter_rows},
        "sigma_metrics": {row["axis"]: row for row in sigma_rows},
        "interpretation_limits": [
            "This attributes Eq.(8) update drive; it does not turn nominal bias into nonzero pi_star.",
            "Velocity and pose subterms are closed-loop correlated and are not independent causal interventions.",
            "No estimator, CLF, dynamics, reference, gain, or cost was changed.",
        ],
    }
    (output / "stage_d5d_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")

    def focus_line(index):
        row = parameter_rows[index]
        return "| {} | {:.6g} | {:.6g} | {:.6g} | {:.2%} | {:.2%} |".format(
            row["parameter"], row["logged_net_update"], row["velocity_error_net_update"],
            row["lambda_pose_error_net_update"], row["velocity_abs_share"], row["pose_abs_share"])

    report = """# Stage D5D — Nominal walking sigma subterm decomposition

## Scope

本阶段只增加 diagnostics 并重采一条短时 Nominal walking。plant 与 controller model 相同，故真实 payload uncertainty 仍为 `pi_star=0`。分析目标是把

`sigma = [v_d-v; omega_d-omega] + [Lambda_l(p_d-p); Lambda_o e_o]`

两项对 `pi_dot = Gamma Y^T sigma` 的贡献精确拆开，不修改 Eq.(8)、Eq.(10)、CLF、reference 或任何 gain。

## Machine-precision closure

- sigma 子项闭合最大误差：`{sigma_closure:.3e}`
- 与既有 v3 sigma 最大误差：`{v3_sigma_error:.3e}`
- 两类 pi_dot 与既有在线 pi_dot 最大误差：`{rate_closure:.3e}`
- 右端显式 Euler 累计与实际 pi_hat 变化最大误差：`{euler_closure:.3e}`
- 结果：**{status}**

## Focus coordinates

| parameter | logged net update | velocity-error contribution | Lambda-pose-error contribution | velocity absolute share | pose absolute share |
|---|---:|---:|---:|---:|---:|
{focus_rows}

绝对占比使用 `|a|/(|a|+|b|)`，用于存在符号抵消时避免误导；它不是独立因果比例。

## Interpretation boundary

这份结果回答“Eq.(8) 的 nominal drift 更主要由速度跟踪误差还是位姿反馈误差驱动”。它不把 nominal bias 解释为 payload，也不证明修改 estimator 后一定改善 tracking。两个子项来自同一闭环轨迹，存在相关性，因此这里是精确代数归因，不是独立干预的因果识别。

## Artifacts

- `parameter_subterm_contributions.csv`
- `sigma_subterm_metrics.csv`
- `time_segment_subterm_contributions.csv`
- `sigma_subterm_timeseries.csv`
- `figure_D5D_01_angular_sigma_subterms.png`
- `figure_D5D_02_cumulative_parameter_subterms.png`
- `figure_D5D_03_endpoint_parameter_subterms.png`
- `figure_D5D_04_parameter_rate_subterms.png`
- `stage_d5d_summary.json`
""".format(sigma_closure=sigma_closure, v3_sigma_error=v3_sigma_error,
           rate_closure=rate_closure, euler_closure=euler_closure,
           status="PASS" if passed else "FAIL",
           focus_rows="\n".join(focus_line(index) for index in FOCUS))
    (output / "stage_d5d_report.md").write_text(report)

    artifacts = []
    for path in sorted(output.iterdir()):
        if path.is_file() and path.name != "stage_d5d_artifact_manifest.json":
            artifacts.append({"path": path.name, "bytes": path.stat().st_size,
                              "sha256": sha256(path)})
    manifest = {"schema_version": 1, "passed": passed, "artifacts": artifacts}
    (output / "stage_d5d_artifact_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"passed": passed, "output": str(output),
                      "closure": summary["closure"]}, indent=2))
    return 0 if passed else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory")
    parser.add_argument("output_directory")
    args = parser.parse_args()
    return analyze(args.run_directory, args.output_directory)


if __name__ == "__main__":
    raise SystemExit(main())
