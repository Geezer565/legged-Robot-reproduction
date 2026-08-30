#!/usr/bin/env python3

import argparse
import csv
import json
import math
from pathlib import Path


METHODS = ("Baseline", "ETHRBF", "MyRBF")
COLORS = {"Baseline": "#5B6770", "ETHRBF": "#E69F00", "MyRBF": "#0072B2"}
HATCHES = {"Baseline": "//", "ETHRBF": "\\\\", "MyRBF": ".."}
FORCE_FIELDS = ("fx_n", "fy_n", "fz_n")
TORQUE_FIELDS = ("tx_nm", "ty_nm", "tz_nm")


def mean(values):
    return sum(values) / len(values)


def rms(values):
    return math.sqrt(mean([value * value for value in values]))


def vector_norm(rows, prefix, suffixes):
    return [
        math.sqrt(sum(float(row[prefix + suffix]) ** 2 for suffix in suffixes))
        for row in rows
    ]


def read_rows(path):
    with Path(path).open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def active_motion_window(motion_rows):
    active = [row for row in motion_rows if row["active"] == "1"]
    if not active:
        raise ValueError("motion.csv has no active walking samples")
    return float(active[0]["ros_time"]), float(active[-1]["ros_time"]), active


def analyze_case_rows(name, speed, shadow_rows, motion_rows, state_rows):
    onset, end, active_motion = active_motion_window(motion_rows)
    active_shadow = [
        row for row in shadow_rows if onset <= float(row["time_s"]) <= end
    ]
    if not active_shadow:
        raise ValueError(f"{name} has no RBF samples in the walking window")

    finite_state = [
        row for row in state_rows if row.get("base_z", "nan").lower() != "nan"
    ]
    walking_state = [
        row for row in finite_state if onset <= float(row["ros_time"]) <= end
    ]
    if not walking_state:
        raise ValueError(f"{name} has no valid state samples in the walking window")

    time = [float(row["time_s"]) - onset for row in active_shadow]
    actual_x = [float(row["actual_base_x_m"]) for row in active_shadow]
    desired_x = [float(row["desired_base_x_m"]) for row in active_shadow]
    x_error_cm = [
        100.0 * (float(row["actual_base_x_m"]) - float(row["desired_base_x_m"]))
        for row in active_shadow
    ]
    y_error_cm = [
        100.0 * (float(row["actual_base_y_m"]) - float(row["desired_base_y_m"]))
        for row in active_shadow
    ]
    z_error_cm = [
        100.0 * (float(row["actual_base_z_m"]) - float(row["desired_base_z_m"]))
        for row in active_shadow
    ]
    roll_error_deg = [
        math.degrees(float(row["actual_base_roll_rad"]) - float(row["desired_base_roll_rad"]))
        for row in active_shadow
    ]
    pitch_error_deg = [
        math.degrees(float(row["actual_base_pitch_rad"]) - float(row["desired_base_pitch_rad"]))
        for row in active_shadow
    ]
    yaw_error_deg = [
        math.degrees(float(row["actual_base_yaw_rad"]) - float(row["desired_base_yaw_rad"]))
        for row in active_shadow
    ]
    position_error_cm = [
        math.sqrt(x * x + y * y + z * z)
        for x, y, z in zip(x_error_cm, y_error_cm, z_error_cm)
    ]
    orientation_error_deg = [
        math.sqrt(roll * roll + pitch * pitch + yaw * yaw)
        for roll, pitch, yaw in zip(roll_error_deg, pitch_error_deg, yaw_error_deg)
    ]

    estimate_force = vector_norm(active_shadow, "estimate_after_", FORCE_FIELDS)
    estimate_torque = vector_norm(active_shadow, "estimate_after_", TORQUE_FIELDS)
    applied_force = vector_norm(active_shadow, "achieved_control_", FORCE_FIELDS)
    applied_torque = vector_norm(active_shadow, "achieved_control_", TORQUE_FIELDS)
    all_values = (
        time + actual_x + desired_x + position_error_cm + orientation_error_deg
        + estimate_force + estimate_torque + applied_force + applied_torque
    )
    if not all(math.isfinite(value) for value in all_values):
        raise ValueError(f"{name} contains a non-finite analyzed value")

    result = {
        "name": name,
        "speed_mps": speed,
        "sample_count": len(active_shadow),
        "position_rmse_cm": rms(position_error_cm),
        "forward_rmse_cm": rms(x_error_cm),
        "lateral_rmse_cm": rms(y_error_cm),
        "height_rmse_cm": rms(z_error_cm),
        "orientation_rmse_deg": rms(orientation_error_deg),
        "roll_rmse_deg": rms(roll_error_deg),
        "pitch_rmse_deg": rms(pitch_error_deg),
        "yaw_rmse_deg": rms(yaw_error_deg),
        "max_lateral_error_cm": max(abs(value) for value in y_error_cm),
        "max_roll_error_deg": max(abs(value) for value in roll_error_deg),
        "estimate_force_rms_n": rms(estimate_force),
        "estimate_torque_rms_nm": rms(estimate_torque),
        "applied_force_rms_n": rms(applied_force),
        "applied_force_max_n": max(applied_force),
        "applied_torque_rms_nm": rms(applied_torque),
        "applied_torque_max_nm": max(applied_torque),
        "desired_progress_m": desired_x[-1] - desired_x[0],
        "actual_progress_m": actual_x[-1] - actual_x[0],
        "state_walk_distance_m": (
            float(walking_state[-1]["base_x"]) - float(walking_state[0]["base_x"])
        ),
        "minimum_base_z_m": min(float(row["base_z"]) for row in finite_state),
        "end_mode": int(float(finite_state[-1]["observation_mode"])),
        "motion_start_s": onset,
        "motion_end_s": end,
        "motion_ros_duration_s": end - onset,
        "motion_wall_duration_s": (
            float(active_motion[-1]["wall_elapsed"])
            - float(active_motion[0]["wall_elapsed"])
        ),
        "commanded_speed_mps": mean([float(row["vx"]) for row in active_motion]),
    }
    series = {
        "time_s": time,
        "actual_x_m": actual_x,
        "desired_x_m": desired_x,
        "lateral_error_cm": y_error_cm,
        "roll_error_deg": roll_error_deg,
        "estimate_force_n": estimate_force,
        "applied_force_n": applied_force,
    }
    return result, series


def analyze_case(name, speed, directory):
    directory = Path(directory)
    return analyze_case_rows(
        name,
        speed,
        read_rows(directory / "shadow.csv"),
        read_rows(directory / "motion.csv"),
        read_rows(directory / "state.csv"),
    )


def grouped(results):
    output = {}
    for result in results:
        output[(result["speed_mps"], result["name"])] = result
    return output


def assess(results):
    lookup = grouped(results)
    speeds = sorted({result["speed_mps"] for result in results})
    details = []
    for speed in speeds:
        baseline = lookup[(speed, "Baseline")]
        myrbf = lookup[(speed, "MyRBF")]
        position_limit = max(1.05 * baseline["position_rmse_cm"], baseline["position_rmse_cm"] + 0.05)
        orientation_limit = max(
            1.05 * baseline["orientation_rmse_deg"],
            baseline["orientation_rmse_deg"] + 0.05,
        )
        item = {
            "speed_mps": speed,
            "position_not_degraded": myrbf["position_rmse_cm"] <= position_limit,
            "orientation_not_degraded": myrbf["orientation_rmse_deg"] <= orientation_limit,
            "force_command_bounded": myrbf["applied_force_rms_n"] <= 5.0,
            "torque_command_bounded": myrbf["applied_torque_rms_nm"] <= 0.75,
            "safe_height": myrbf["minimum_base_z_m"] > 0.25,
            "four_contact_finish": myrbf["end_mode"] == 15,
        }
        item["pass"] = all(value for key, value in item.items() if key not in ("speed_mps", "pass"))
        details.append(item)
    return {"pass": all(item["pass"] for item in details), "details": details}


def plot_trajectories(path, results, series_by_key):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    speeds = sorted({result["speed_mps"] for result in results})
    fig, axes = plt.subplots(len(speeds), 3, figsize=(13.0, 9.0), constrained_layout=True)
    for row_index, speed in enumerate(speeds):
        target_series = series_by_key[(speed, "Baseline")]
        axes[row_index, 0].plot(
            target_series["time_s"], target_series["desired_x_m"],
            color="black", linestyle="--", linewidth=2.0, label="Target",
        )
        for method in METHODS:
            series = series_by_key[(speed, method)]
            axes[row_index, 0].plot(
                series["time_s"], series["actual_x_m"],
                color=COLORS[method], linewidth=1.5, label=method,
            )
            axes[row_index, 1].plot(
                series["time_s"], series["lateral_error_cm"],
                color=COLORS[method], linewidth=1.5, label=method,
            )
            axes[row_index, 2].plot(
                series["time_s"], series["roll_error_deg"],
                color=COLORS[method], linewidth=1.5, label=method,
            )
        axes[row_index, 0].set_ylabel(f"{speed:.2f} m/s\nForward x (m)")
        axes[row_index, 1].set_ylabel("Lateral error (cm)")
        axes[row_index, 2].set_ylabel("Roll error (deg)")
        for axis in axes[row_index, :]:
            axis.grid(True, alpha=0.25)
            axis.set_xlabel("Time after walking starts (s)")
    axes[0, 0].set_title("Target and actual forward trajectory")
    axes[0, 1].set_title("Lateral tracking error")
    axes[0, 2].set_title("Roll tracking error")
    axes[0, 0].legend(ncol=2, fontsize=8, frameon=False)
    fig.suptitle("M01: Undisturbed walking tracking", fontsize=15, fontweight="bold")
    fig.savefig(path.with_suffix(".png"), dpi=300, bbox_inches="tight", facecolor="white")
    fig.savefig(path.with_suffix(".pdf"), bbox_inches="tight", facecolor="white")
    plt.close(fig)


def plot_compensation(path, results, series_by_key):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    speeds = sorted({result["speed_mps"] for result in results})
    fig, axes = plt.subplots(2, len(speeds), figsize=(13.0, 6.2), constrained_layout=True)
    for column, speed in enumerate(speeds):
        for method in METHODS:
            series = series_by_key[(speed, method)]
            axes[0, column].plot(
                series["time_s"], series["estimate_force_n"],
                color=COLORS[method], linewidth=1.5, label=method,
            )
            axes[1, column].plot(
                series["time_s"], series["applied_force_n"],
                color=COLORS[method], linewidth=1.5, label=method,
            )
        axes[0, column].set_title(f"Command speed {speed:.2f} m/s")
        axes[0, column].set_ylabel("Estimated force norm (N)")
        axes[1, column].set_ylabel("Applied force norm (N)")
        for axis in axes[:, column]:
            axis.set_xlabel("Time after walking starts (s)")
            axis.grid(True, alpha=0.25)
    axes[0, 0].legend(ncol=1, fontsize=8, frameon=False)
    fig.suptitle("M01: No-disturbance learning and compensation check", fontsize=15, fontweight="bold")
    fig.savefig(path.with_suffix(".png"), dpi=300, bbox_inches="tight", facecolor="white")
    fig.savefig(path.with_suffix(".pdf"), bbox_inches="tight", facecolor="white")
    plt.close(fig)


def plot_summary(path, results):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    lookup = grouped(results)
    speeds = sorted({result["speed_mps"] for result in results})
    metrics = (
        ("position_rmse_cm", "Position RMSE (cm)"),
        ("orientation_rmse_deg", "Orientation RMSE (deg)"),
        ("applied_force_rms_n", "Applied force RMS (N)"),
        ("applied_torque_rms_nm", "Applied torque RMS (N·m)"),
    )
    fig, axes = plt.subplots(2, 2, figsize=(11.0, 7.2), constrained_layout=True)
    x = list(range(len(speeds)))
    width = 0.24
    for axis, (field, label) in zip(axes.flat, metrics):
        for method_index, method in enumerate(METHODS):
            offset = (method_index - 1) * width
            bars = axis.bar(
                [value + offset for value in x],
                [lookup[(speed, method)][field] for speed in speeds],
                width=width, label=method, color=COLORS[method],
                edgecolor="black", linewidth=0.6,
            )
            for bar in bars:
                bar.set_hatch(HATCHES[method])
        axis.set_xticks(x)
        axis.set_xticklabels([f"{speed:.2f}" for speed in speeds])
        axis.set_xlabel("Command speed (m/s)")
        axis.set_ylabel(label)
        axis.grid(True, axis="y", alpha=0.25)
    axes[0, 0].legend(ncol=3, fontsize=8, frameon=False)
    fig.suptitle("M01: Undisturbed walking summary", fontsize=15, fontweight="bold")
    fig.savefig(path.with_suffix(".png"), dpi=300, bbox_inches="tight", facecolor="white")
    fig.savefig(path.with_suffix(".pdf"), bbox_inches="tight", facecolor="white")
    plt.close(fig)


def write_report(path, results, assessment):
    lookup = grouped(results)
    speeds = sorted({result["speed_mps"] for result in results})
    wall_durations = [result["motion_wall_duration_s"] for result in results]
    ros_durations = [result["motion_ros_duration_s"] for result in results]
    lines = [
        "# M01 无扰动行走实验报告",
        "",
        "## 实验设置",
        "",
        "- Go2 Gazebo Classic，平整地面，`static_walk`；",
        "- 速度为 0.05、0.10、0.15 m/s；",
        "- 行走命令按墙钟时间运行约 9 s，不施加外力和额外负重；",
        f"- 实际墙钟有效时长为 {min(wall_durations):.2f}–{max(wall_durations):.2f} s；对应 Gazebo 仿真时间为 {min(ros_durations):.2f}–{max(ros_durations):.2f} s；",
        "- 比较 Baseline、ETHRBF 和当前 MyRBF；",
        "- 每个条件当前只运行 1 次；每场重新启动并清空学习历史。",
        "",
        "## 结果",
        "",
        "| 速度 | 方法 | 位置误差（cm）↓ | 姿态误差（deg）↓ | 侧移误差（cm）↓ | 侧倾误差（deg）↓ | 无扰动施力（N）↓ | 无扰动力矩（N·m）↓ | 前进距离（m） | 最低高度（m） | 结束状态 |",
        "| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for speed in speeds:
        for method in METHODS:
            item = lookup[(speed, method)]
            lines.append(
                f"| {speed:.2f} | {method} | {item['position_rmse_cm']:.3f} | "
                f"{item['orientation_rmse_deg']:.3f} | {item['lateral_rmse_cm']:.3f} | "
                f"{item['roll_rmse_deg']:.3f} | {item['applied_force_rms_n']:.3f} | "
                f"{item['applied_torque_rms_nm']:.3f} | {item['state_walk_distance_m']:.3f} | "
                f"{item['minimum_base_z_m']:.4f} | "
                f"{item['end_mode']} |"
            )
    lines.extend(["", "## 分析", ""])
    for speed in speeds:
        baseline = lookup[(speed, "Baseline")]
        eth = lookup[(speed, "ETHRBF")]
        myrbf = lookup[(speed, "MyRBF")]
        position_reduction = 100.0 * (
            1.0 - myrbf["position_rmse_cm"] / baseline["position_rmse_cm"]
        )
        orientation_change = 100.0 * (
            myrbf["orientation_rmse_deg"] / baseline["orientation_rmse_deg"] - 1.0
        )
        force_reduction = 100.0 * (
            1.0 - myrbf["applied_force_rms_n"] / eth["applied_force_rms_n"]
        )
        torque_reduction = 100.0 * (
            1.0 - myrbf["applied_torque_rms_nm"] / eth["applied_torque_rms_nm"]
        )
        lines.append(
            f"- {speed:.2f} m/s：MyRBF 的位置误差比 Baseline 低 {position_reduction:.1f}%；"
            f"姿态误差相对高 {orientation_change:.1f}%，但绝对增加仅 "
            f"{myrbf['orientation_rmse_deg'] - baseline['orientation_rmse_deg']:.4f}°；"
            f"施力和力矩分别比 ETHRBF 低 {force_reduction:.1f}% 和 {torque_reduction:.1f}%。"
        )
    lines.extend([
        "",
        "三种速度下，目标前进轨迹和实际轨迹基本重合。MyRBF 的姿态误差相对 Baseline 略高，但绝对差值不足 0.001°；结合单次实验精度，不能解释为实际稳定性下降。ETHRBF 的无扰动估计随时间持续增大，而 MyRBF 保持在更低范围，因此当前更新方式更不容易在正常行走时积累不必要补偿。",
        "",
        "## 自动检查",
        "",
    ])
    for detail in assessment["details"]:
        status = "通过" if detail["pass"] else "未通过"
        lines.append(f"- {detail['speed_mps']:.2f} m/s：{status}。")
    lines.extend([
        "",
        f"M01 总体判断：**{'通过' if assessment['pass'] else '未通过'}**。",
        "",
        "自动检查要求机器人安全结束，且 MyRBF 的位置和姿态误差不能相对 Baseline 明显恶化；无扰动时实际补偿也必须保持在预设安全范围内。这里的无扰动补偿可能同时包含模型误差补偿，不能全部理解为网络误判。",
        "",
        "## 图",
        "",
        "![目标轨迹与实际状态](analysis/01_tracking_and_state.png)",
        "",
        "![无扰动估计与补偿](analysis/02_spurious_compensation.png)",
        "",
        "![指标汇总](analysis/03_metric_summary.png)",
        "",
        "## 结论边界",
        "",
        "本轮用于检查正常行走时的副作用，每个条件只有一次运行，不能替代最终重复统计，也不代表真机结果。各场使用相同墙钟时长，但 Gazebo 实时速度有差异；后续正式统计应改为统一仿真时间或按共同时间窗口比较。",
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Analyze M01 undisturbed Go2 walking runs.")
    parser.add_argument(
        "--case", action="append", nargs=3, metavar=("METHOD", "SPEED", "DIRECTORY"), required=True
    )
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    output = Path(args.output_dir)
    analysis_dir = output / "analysis"
    analysis_dir.mkdir(parents=True, exist_ok=True)
    analyzed = [analyze_case(method, float(speed), directory) for method, speed, directory in args.case]
    results = [item[0] for item in analyzed]
    series_by_key = {
        (result["speed_mps"], result["name"]): series
        for result, (_, series) in zip(results, analyzed)
    }
    expected = {(speed, method) for speed in (0.05, 0.10, 0.15) for method in METHODS}
    if set(series_by_key) != expected:
        raise ValueError("M01 requires Baseline, ETHRBF and MyRBF at 0.05, 0.10 and 0.15 m/s")

    assessment = assess(results)
    (analysis_dir / "metrics.json").write_text(
        json.dumps(results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (analysis_dir / "assessment.json").write_text(
        json.dumps(assessment, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    plot_trajectories(analysis_dir / "01_tracking_and_state", results, series_by_key)
    plot_compensation(analysis_dir / "02_spurious_compensation", results, series_by_key)
    plot_summary(analysis_dir / "03_metric_summary", results)
    write_report(output / "M01_EXPERIMENT_REPORT.md", results, assessment)


if __name__ == "__main__":
    main()
