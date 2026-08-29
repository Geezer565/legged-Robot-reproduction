#!/usr/bin/env python3

import argparse
import bisect
import csv
import json
import math
from pathlib import Path


def read_rows(path):
    with Path(path).open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def mean(values):
    return sum(values) / len(values)


def correlation(left, right):
    left_mean = mean(left)
    right_mean = mean(right)
    covariance = sum(
        (x - left_mean) * (y - right_mean) for x, y in zip(left, right)
    )
    left_energy = sum((x - left_mean) ** 2 for x in left)
    right_energy = sum((y - right_mean) ** 2 for y in right)
    denominator = math.sqrt(left_energy * right_energy)
    return covariance / denominator if denominator > 0.0 else 0.0


def analyze_case(name, shadow_path, wrench_path):
    shadow = read_rows(shadow_path)
    wrench = read_rows(wrench_path)
    active_truth = [row for row in wrench if abs(float(row["fy"])) > 0.05]
    if not shadow or not active_truth:
        raise ValueError(f"{name} has no usable shadow or wrench samples")

    onset = float(active_truth[0]["ros_time"])
    end = float(active_truth[-1]["ros_time"])
    active_shadow = [
        row for row in shadow if onset <= float(row["time_s"]) <= end
    ]
    truth_times = [float(row["ros_time"]) for row in wrench]
    truth_fy = [float(row["fy"]) for row in wrench]
    target_compensation = []
    estimate = []
    for row in active_shadow:
        time_value = float(row["time_s"])
        insertion = bisect.bisect_left(truth_times, time_value)
        candidates = [max(0, insertion - 1), min(len(truth_times) - 1, insertion)]
        nearest = min(candidates, key=lambda index: abs(truth_times[index] - time_value))
        target_compensation.append(-truth_fy[nearest])
        estimate.append(float(row["estimate_after_fy_n"]))

    y_values = [float(row["actual_base_y_m"]) for row in active_shadow]
    roll_values = [float(row["actual_base_roll_rad"]) for row in active_shadow]
    applied = [float(row["achieved_control_fy_n"]) for row in active_shadow]
    allocator_scale = [float(row["allocator_scale"]) for row in active_shadow]
    estimation_error = [
        actual - target for actual, target in zip(estimate, target_compensation)
    ]
    return {
        "name": name,
        "onset_s": onset,
        "end_s": end,
        "sample_count": len(active_shadow),
        "max_abs_y_cm": 100.0 * max(abs(value) for value in y_values),
        "rms_y_cm": 100.0 * math.sqrt(mean([value * value for value in y_values])),
        "max_abs_roll_deg": math.degrees(max(abs(value) for value in roll_values)),
        "rms_roll_deg": math.degrees(
            math.sqrt(mean([value * value for value in roll_values]))
        ),
        "estimate_rmse_n": math.sqrt(
            mean([value * value for value in estimation_error])
        ),
        "estimate_correlation": correlation(target_compensation, estimate),
        "mean_abs_applied_fy_n": mean([abs(value) for value in applied]),
        "max_abs_applied_fy_n": max(abs(value) for value in applied),
        "minimum_allocator_scale": min(allocator_scale),
        "valid_samples": int(float(shadow[-1]["valid_sample_count"])),
        "rejected_samples": int(float(shadow[-1]["rejected_sample_count"])),
    }


def write_markdown(path, results):
    lines = [
        "# MyRBF v4 Go2 接口验证结果",
        "",
        "| 方法 | 最大侧移 cm | 侧移均方根 cm | 最大侧倾 deg | 侧倾均方根 deg | 估计误差 N | 波形相关性 | 平均补偿 N |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for result in results:
        lines.append(
            "| {name} | {max_abs_y_cm:.3f} | {rms_y_cm:.3f} | "
            "{max_abs_roll_deg:.3f} | {rms_roll_deg:.3f} | "
            "{estimate_rmse_n:.3f} | {estimate_correlation:.3f} | "
            "{mean_abs_applied_fy_n:.3f} |".format(**result)
        )
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--case", action="append", nargs=3, metavar=("NAME", "SHADOW", "WRENCH"),
        required=True,
    )
    parser.add_argument("--json-output", required=True)
    parser.add_argument("--markdown-output", required=True)
    args = parser.parse_args()

    results = [analyze_case(*case) for case in args.case]
    Path(args.json_output).write_text(
        json.dumps(results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    write_markdown(args.markdown_output, results)


if __name__ == "__main__":
    main()
