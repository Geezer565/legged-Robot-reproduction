#!/usr/bin/env python3
"""离线、公平比较 CLF OFF 与 CLF ON 的同一 base-x step rosbag。"""

import argparse
import bisect
import math
import sys

import rosbag


OBSERVATION = "/legged_robot_mpc_observation"
TARGET = "/legged_robot_mpc_target"
# 仅使用真实 observation state/input 旁路重算的 executed Eq.(11) 指标。
# /aclf/* 是 DDP rollout/latest diagnostics，保留调试用途但绝不能混入 ON/OFF
# executed comparison，否则不同 solver internal node 会污染公平实验结论。
EXECUTED_SIGMA = "/aclf_experiment/executed_sigma_norm"
EXECUTED_H_CLF = "/aclf_experiment/executed_h_clf"
EXECUTED_VIOLATION = "/aclf_experiment/executed_violation"
EXECUTED_CONTRIBUTION = "/aclf_experiment/executed_contribution"
EXECUTED_RESIDUAL_WRENCH = "/aclf_experiment/executed_residual_wrench"
EXECUTED_DAMPING_TERM = "/aclf_experiment/executed_damping_term"
EXECUTED_LINEAR_TERM = "/aclf_experiment/executed_linear_term"
EXECUTED_OBSERVATION_TIME = "/aclf_experiment/executed_observation_time"
EXECUTED_SNAPSHOT_TIME = "/aclf_experiment/executed_snapshot_time"
EXECUTED_TIME_ERROR = "/aclf_experiment/executed_time_error"
DEFAULT_ANALYSIS_WINDOW_S = 15.0
PAIRING_TOLERANCE_S = 0.01


def interpolate(time, times, states):
    if time <= times[0]:
        return states[0]
    if time >= times[-1]:
        return states[-1]
    for index in range(len(times) - 1):
        if times[index] < time <= times[index + 1]:
            alpha = (times[index + 1] - time) / (times[index + 1] - times[index])
            return [alpha * a + (1.0 - alpha) * b for a, b in zip(states[index], states[index + 1])]
    return states[-1]


def trapezoid(times, values):
    return sum(0.5 * (values[i] + values[i + 1]) * (times[i + 1] - times[i]) for i in range(len(values) - 1))


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def mean(values):
    return sum(values) / len(values)


def unavailable():
    return float("nan")


def latest_target_before(targets, bag_time):
    latest = None
    for published_time, target in targets:
        if published_time <= bag_time:
            latest = target
        else:
            break
    return latest


def nearest_index(sorted_times, value):
    """返回 sorted_times 中与 value bag timestamp 最近的元素下标。"""
    right = bisect.bisect_left(sorted_times, value)
    if right == 0:
        return 0
    if right == len(sorted_times):
        return len(sorted_times) - 1
    return right if abs(sorted_times[right] - value) < abs(sorted_times[right - 1] - value) else right - 1


def pair_executed_diagnostics(raw, pairing_tolerance_s=PAIRING_TOLERANCE_S):
    """
    将无 Header 的 executed diagnostics 归并回同一次 timer publication。

    ROS node 在同一 timer callback 中依次发布所有 executed_* topic，且
    executed_observation_time 是该次真实 MPC observation 的时间。bag timestamp
    仅在此处用于把相邻标准消息归组，绝不用于积分或 window 裁剪；归组完成后唯一
    时间坐标为 executed_observation_time。
    """
    anchors = raw["observation_time"]
    if not anchors:
        return [], {"unmatched_messages": 0, "max_pairing_bag_time_error_s": unavailable()}
    anchors.sort(key=lambda item: item[0])
    anchor_bag_times = [time for time, _ in anchors]
    groups = [{"observation_time": observation_time, "_pair_error": 0.0}
              for _, observation_time in anchors]
    unmatched = []

    for field in ("sigma", "h_clf", "violation", "contribution", "residual_wrench", "damping_term",
                  "linear_term", "snapshot_time", "time_error"):
        for bag_time, value in raw[field]:
            index = nearest_index(anchor_bag_times, bag_time)
            pair_error = abs(anchor_bag_times[index] - bag_time)
            if pair_error > pairing_tolerance_s:
                unmatched.append((field, bag_time, "outside_pairing_tolerance", groups[index]["observation_time"]))
                continue
            # 同类消息可能在 bag 起止边界多一条；保留更接近 anchor 的一条。
            previous_error = groups[index].get("_" + field + "_pair_error")
            if previous_error is None or pair_error < previous_error:
                if previous_error is not None:
                    unmatched.append((field, anchor_bag_times[index], "replaced_by_closer_message",
                                      groups[index]["observation_time"]))
                groups[index][field] = value
                groups[index]["_" + field + "_pair_error"] = pair_error
                groups[index]["_pair_error"] = max(groups[index]["_pair_error"], pair_error)
            else:
                unmatched.append((field, bag_time, "duplicate_for_same_observation", groups[index]["observation_time"]))

    required = ("sigma", "h_clf", "violation", "contribution", "residual_wrench", "damping_term",
                "linear_term", "snapshot_time", "time_error")
    samples = [group for group in groups if all(field in group for field in required)]
    samples.sort(key=lambda sample: sample["observation_time"])
    max_pair_error = max((sample["_pair_error"] for sample in samples), default=unavailable())
    return samples, {"unmatched_messages": unmatched, "max_pairing_bag_time_error_s": max_pair_error}


def time_weighted_mean(times, values):
    span = times[-1] - times[0]
    return trapezoid(times, values) / span if len(times) >= 2 and span > 0.0 else unavailable()


def read_bag(path, analysis_window_s=DEFAULT_ANALYSIS_WINDOW_S):
    if analysis_window_s <= 0.0:
        raise ValueError("analysis_window_s 必须为正")
    targets, observations = [], []
    raw_executed = {"sigma": [], "h_clf": [], "violation": [], "contribution": [], "residual_wrench": [],
                    "damping_term": [], "linear_term": [],
                    "observation_time": [], "snapshot_time": [], "time_error": []}
    topics = [OBSERVATION, TARGET, EXECUTED_SIGMA, EXECUTED_H_CLF, EXECUTED_VIOLATION, EXECUTED_CONTRIBUTION,
              EXECUTED_RESIDUAL_WRENCH, EXECUTED_DAMPING_TERM, EXECUTED_LINEAR_TERM,
              EXECUTED_OBSERVATION_TIME, EXECUTED_SNAPSHOT_TIME, EXECUTED_TIME_ERROR]
    with rosbag.Bag(path) as bag:
        for topic, message, timestamp in bag.read_messages(topics=topics):
            bag_time = timestamp.to_sec()
            if topic == TARGET:
                targets.append((bag_time, (list(message.timeTrajectory), [list(s.value) for s in message.stateTrajectory])))
            elif topic == OBSERVATION:
                observations.append((bag_time, message))
            elif topic == EXECUTED_SIGMA:
                raw_executed["sigma"].append((bag_time, message.data))
            elif topic == EXECUTED_H_CLF:
                raw_executed["h_clf"].append((bag_time, message.data))
            elif topic == EXECUTED_VIOLATION:
                raw_executed["violation"].append((bag_time, message.data))
            elif topic == EXECUTED_CONTRIBUTION and len(message.data) == 6:
                raw_executed["contribution"].append((bag_time, list(message.data)))
            elif topic == EXECUTED_RESIDUAL_WRENCH and len(message.data) == 6:
                raw_executed["residual_wrench"].append((bag_time, list(message.data)))
            elif topic == EXECUTED_DAMPING_TERM:
                raw_executed["damping_term"].append((bag_time, message.data))
            elif topic == EXECUTED_LINEAR_TERM:
                raw_executed["linear_term"].append((bag_time, message.data))
            elif topic == EXECUTED_OBSERVATION_TIME:
                raw_executed["observation_time"].append((bag_time, message.data))
            elif topic == EXECUTED_SNAPSHOT_TIME:
                raw_executed["snapshot_time"].append((bag_time, message.data))
            elif topic == EXECUTED_TIME_ERROR:
                raw_executed["time_error"].append((bag_time, message.data))

    if not observations or not targets:
        raise RuntimeError("bag 必须包含真实 observation 与 mpc_target")
    targets.sort(key=lambda item: item[0])
    observations.sort(key=lambda item: item[0])

    # 识别原生 relative target 发送的 step：目标末端与起点 base-x 相差至少 1 cm。
    step = None
    for target_bag_time, candidate in targets:
        times, states = candidate
        if len(times) >= 2 and len(states) >= 2 and len(states[0]) > 6 and abs(states[-1][6] - states[0][6]) >= 0.01:
            step = (target_bag_time, candidate)
            break
    if step is None:
        raise RuntimeError("未在 target 中找到 >= 0.01 m 的 base-x step")
    step_bag_time, step_target = step
    step_start, step_end = step_target[0][0], step_target[0][-1]
    window_end = step_start + analysis_window_s
    amplitude = step_target[1][-1][6] - step_target[1][0][6]
    threshold = max(0.005, 0.1 * abs(amplitude))  # 0.05 m step 的 10% = 5 mm。

    series = []
    for bag_time, observation in observations:
        target = latest_target_before(targets, bag_time)
        if target is None:
            continue
        times, states = target
        desired = interpolate(observation.time, times, states)
        actual = list(observation.state.value)
        # 关键公平性约束：绝不把 bag 后续长时间 standing 样本纳入 ON/OFF 指标。
        if len(actual) < 12 or len(desired) < 12 or not (step_start <= observation.time <= window_end):
            continue
        x_error = desired[6] - actual[6]
        pitch_error = math.atan2(math.sin(desired[10] - actual[10]), math.cos(desired[10] - actual[10]))
        position_norm = math.sqrt(sum((desired[index] - actual[index]) ** 2 for index in (6, 7, 8)))
        series.append((observation.time, x_error, pitch_error, position_norm))
    if len(series) < 2:
        raise RuntimeError("固定 15 s step window 内有效 observation 数量不足")

    times = [item[0] for item in series]
    x_errors = [item[1] for item in series]
    pitch_errors = [item[2] for item in series]
    position_norms = [item[3] for item in series]
    settling = unavailable()
    hold_seconds = 2.0
    for index, time in enumerate(times):
        if time < step_end or abs(x_errors[index]) > threshold:
            continue
        until = time + hold_seconds
        later = [abs(error) for sample_time, error in zip(times[index:], x_errors[index:]) if sample_time <= until]
        if later and times[-1] >= until and max(later) <= threshold:
            settling = time - step_start
            break

    executed_samples, pairing = pair_executed_diagnostics(raw_executed)
    # 关键修复：window 与积分都只使用 executed_observation_time，即真实 MPC
    # 时间。bag wall-clock 只做 publication grouping，绝不参与 trapezoid。
    executed_window = [sample for sample in executed_samples
                       if step_start <= sample["observation_time"] <= window_end]
    executed_times = [sample["observation_time"] for sample in executed_window]
    sigma_values = [sample["sigma"] for sample in executed_window]
    h_values = [sample["h_clf"] for sample in executed_window]
    violation_values = [sample["violation"] for sample in executed_window]
    pitch_values = [sample["contribution"][4] for sample in executed_window]
    time_errors = [abs(sample["time_error"]) for sample in executed_window]
    executed_span = executed_times[-1] - executed_times[0] if len(executed_times) >= 2 else unavailable()

    result = {
        "analysis_window_s": analysis_window_s,
        "requested_window_start_s": step_start,
        "requested_window_end_s": window_end,
        "observation_data_start_s": times[0],
        "observation_data_end_s": times[-1],
        "step_amplitude_m": amplitude,
        "settling_band_m": threshold,
        "peak_abs_x_error_m": max(abs(value) for value in x_errors),
        "rms_x_error_m": rms(x_errors),
        "x_error_iae_m_s": trapezoid(times, [abs(value) for value in x_errors]),
        "settling_time_s": settling,
        "peak_abs_pitch_error_rad": max(abs(value) for value in pitch_errors),
        "rms_position_error_m": rms(position_norms),
        "executed_sample_count": len(executed_window),
        "executed_observation_time_start": executed_times[0] if executed_times else unavailable(),
        "executed_observation_time_end": executed_times[-1] if executed_times else unavailable(),
        "actual_executed_time_span_s": executed_span,
        "max_abs_executed_time_error": max(time_errors) if time_errors else unavailable(),
        "mean_abs_executed_time_error": mean(time_errors) if time_errors else unavailable(),
        "unmatched_executed_messages": len(pairing["unmatched_messages"]),
        "unmatched_executed_details": pairing["unmatched_messages"],
        "max_pairing_bag_time_error_s": pairing["max_pairing_bag_time_error_s"],
        "peak_sigma_norm": max(sigma_values) if sigma_values else unavailable(),
        "mean_sigma_norm": mean(sigma_values) if sigma_values else unavailable(),
        "sigma_norm_integral": trapezoid(executed_times, sigma_values)
                               if len(sigma_values) >= 2 else unavailable(),
        "time_weighted_mean_sigma_norm": time_weighted_mean(executed_times, sigma_values),
    }
    if h_values:
            result["mean_executed_h_clf"] = mean(h_values)
            result["min_executed_h_clf"] = min(h_values)
            # 用相邻 diagnostics 样本的时间长度加权，衡量 executed h<0 在固定
            # window 中的占比，而非因 publisher frequency 改变样本权重。
            durations = [executed_times[i + 1] - executed_times[i] for i in range(len(executed_times) - 1)]
            negative_duration = sum(duration for duration, value in zip(durations, h_values[:-1]) if value < 0.0)
            total_duration = sum(durations)
            result["executed_h_negative_time_ratio"] = negative_duration / total_duration if total_duration > 0.0 else unavailable()
    if violation_values:
            result["mean_executed_violation"] = mean(violation_values)
            result["max_executed_violation"] = max(violation_values)
            result["executed_violation_integral"] = trapezoid(
                executed_times, violation_values) if len(violation_values) >= 2 else unavailable()
            result["time_weighted_mean_executed_violation"] = time_weighted_mean(executed_times, violation_values)
    if pitch_values:
            result["mean_executed_pitch_contribution"] = mean(pitch_values)
            result["min_executed_pitch_contribution"] = min(pitch_values)

    # 数学 sanity checks：time integral 的上界由非负 signal 的峰值与真实
    # executed MPC time span 给出。此前错误地使用 wall-clock 正是在此被捕获。
    sanity_tolerance = 1.0e-9 + 1.0e-6 * (abs(executed_span) if isinstance(executed_span, float) else 0.0)
    result["sigma_nonnegative"] = all(value >= -1.0e-12 for value in sigma_values)
    result["violation_nonnegative"] = all(value >= -1.0e-12 for value in violation_values)
    result["sigma_integral_upper_bound_ok"] = (not sigma_values or len(executed_times) < 2 or
        result["sigma_norm_integral"] <= result["peak_sigma_norm"] * executed_span + sanity_tolerance)
    result["violation_integral_upper_bound_ok"] = (not violation_values or len(executed_times) < 2 or
        result["executed_violation_integral"] <= result["max_executed_violation"] * executed_span + sanity_tolerance)
    # 仅供本脚本的后续诊断输出使用；不影响已有 A/B 统计的时间轴或指标定义。
    result["_step_start"] = step_start
    result["_executed_window"] = executed_window
    result["_observation_series"] = series
    return result


def format_value(value):
    return "unavailable" if isinstance(value, float) and math.isnan(value) else f"{value:.6g}"


def percent_change(off, on):
    if not isinstance(off, (float, int)) or not isinstance(on, (float, int)) or math.isnan(off) or abs(off) < 1e-12:
        return "unavailable"
    return f"{100.0 * (on - off) / abs(off):+.2f}%"


def executed_segment_metrics(samples, step_start, begin_tau, end_tau):
    """仅以 executed_observation_time 裁剪的一段 Eq.(11) 统计。"""
    selected = [sample for sample in samples
                if begin_tau <= sample["observation_time"] - step_start <= end_tau]
    if not selected:
        return {"sample_count": 0}
    times = [sample["observation_time"] for sample in selected]
    sigma = [sample["sigma"] for sample in selected]
    h = [sample["h_clf"] for sample in selected]
    violation = [sample["violation"] for sample in selected]
    pitch = [sample["contribution"][4] for sample in selected]
    span = times[-1] - times[0] if len(times) > 1 else 0.0
    durations = [times[i + 1] - times[i] for i in range(len(times) - 1)]
    negative_duration = sum(dt for dt, value in zip(durations, h[:-1]) if value < 0.0)
    return {
        "sample_count": len(selected), "start_tau": times[0] - step_start, "end_tau": times[-1] - step_start,
        "time_span": span, "peak_sigma_norm": max(sigma), "mean_sigma_norm": mean(sigma),
        "mean_h": mean(h), "min_h": min(h),
        "h_negative_time_ratio": negative_duration / sum(durations) if sum(durations) > 0.0 else unavailable(),
        "mean_violation": mean(violation), "max_violation": max(violation),
        "violation_integral": trapezoid(times, violation) if len(times) > 1 else 0.0,
        "mean_pitch_contribution": mean(pitch), "min_pitch_contribution": min(pitch),
    }


def print_segment_metrics(label, result):
    print(f"\n{label}")
    for key in ("sample_count", "start_tau", "end_tau", "time_span", "peak_sigma_norm", "mean_sigma_norm",
                "mean_h", "min_h", "h_negative_time_ratio", "mean_violation", "max_violation",
                "violation_integral", "mean_pitch_contribution", "min_pitch_contribution"):
        print(f"  {key:29s} = {format_value(result.get(key, unavailable()))}")


def sample_tau(sample, step_start):
    return sample["observation_time"] - step_start


def print_extrema(label, result):
    samples = result["_executed_window"]
    step_start = result["_step_start"]
    extrema = {
        "time_of_peak_sigma_norm": max(samples, key=lambda sample: sample["sigma"]),
        "time_of_min_h_clf": min(samples, key=lambda sample: sample["h_clf"]),
        "time_of_max_violation": max(samples, key=lambda sample: sample["violation"]),
        "time_of_min_pitch_contribution": min(samples, key=lambda sample: sample["contribution"][4]),
    }
    pitch_time, pitch_value = max(result["_observation_series"], key=lambda item: abs(item[2]))[0], max(
        result["_observation_series"], key=lambda item: abs(item[2]))[2]
    print(f"\n{label} extrema (t_since_step, MPC/observation time)")
    for name, sample in extrema.items():
        print(f"  {name:32s} = {sample_tau(sample, step_start):.9f} s")
    print(f"  time_of_peak_abs_pitch_error    = {pitch_time - step_start:.9f} s (pitch={pitch_value:.9g} rad)")
    return extrema


def print_min_h_decomposition(label, result, min_h_sample):
    names = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
    contribution = min_h_sample["contribution"]
    residual = min_h_sample["residual_wrench"]
    reconstructed = sum(contribution) - min_h_sample["damping_term"]
    print(f"\n{label} min executed h Eq.(11) decomposition")
    print(f"  observation_time = {min_h_sample['observation_time']:.9f}")
    print(f"  t_since_step     = {sample_tau(min_h_sample, result['_step_start']):.9f} s")
    print(f"  h                = {min_h_sample['h_clf']:.12g}")
    print(f"  linear_term      = {min_h_sample['linear_term']:.12g}")
    print(f"  damping_term     = {min_h_sample['damping_term']:.12g}")
    print(f"  sigma_norm       = {min_h_sample['sigma']:.12g}")
    print("  contribution / residual_wrench (world, wrench torque about COM):")
    for name, c_value, r_value in zip(names, contribution, residual):
        print(f"    {name}: contribution={c_value:.12g}, residual={r_value:.12g}")
    print(f"  sum(contribution)-damping = {reconstructed:.12g}; residual_to_h = {reconstructed - min_h_sample['h_clf']:.3g}")


def print_neighborhood(label, result, title, center_sample, radius=5):
    samples = result["_executed_window"]
    center = samples.index(center_sample)
    start, end = max(0, center - radius), min(len(samples), center + radius + 1)
    print(f"\n{label} {title}: executed samples {start}..{end - 1}")
    print("  tau[s]        sigma_norm        h        violation  linear_term  damping     pitch_contribution")
    for sample in samples[start:end]:
        print(f"  {sample_tau(sample, result['_step_start']): .9f}  {sample['sigma']: .9g}  "
              f"{sample['h_clf']: .9g}  {sample['violation']: .9g}  {sample['linear_term']: .9g}  "
              f"{sample['damping_term']: .9g}  {sample['contribution'][4]: .9g}")


def print_unmatched(label, result):
    details = result["unmatched_executed_details"]
    print(f"\n{label} unmatched executed diagnostics = {len(details)}")
    for field, bag_time, reason, observation_time in details:
        tau = observation_time - result["_step_start"]
        location = "inside 0..15 s window" if 0.0 <= tau <= result["analysis_window_s"] else "outside analysis window"
        print(f"  topic_field={field}, reason={reason}, paired_observation_tau={tau:.9f} s ({location}), "
              f"bag_time={bag_time:.9f}")


def main():
    parser = argparse.ArgumentParser(description="比较同一 base-x step 的 Adaptive MPC CLF OFF/ON rosbag")
    parser.add_argument("clf_off_bag")
    parser.add_argument("clf_on_bag")
    parser.add_argument("--analysis-window-s", type=float, default=DEFAULT_ANALYSIS_WINDOW_S)
    args = parser.parse_args()
    off, on = read_bag(args.clf_off_bag, args.analysis_window_s), read_bag(args.clf_on_bag, args.analysis_window_s)

    print(f"analysis_window_s = {args.analysis_window_s:.1f}")
    print(f"CLF OFF requested MPC window = [{off['requested_window_start_s']:.6f}, {off['requested_window_end_s']:.6f}] s")
    print(f"CLF ON  requested MPC window = [{on['requested_window_start_s']:.6f}, {on['requested_window_end_s']:.6f}] s")
    print(f"CLF OFF observation range used = [{off['observation_data_start_s']:.6f}, {off['observation_data_end_s']:.6f}] s")
    print(f"CLF ON  observation range used = [{on['observation_data_start_s']:.6f}, {on['observation_data_end_s']:.6f}] s")
    print("\nExecuted diagnostics synchronization (MPC time; bag timestamp is grouping-only)")
    for label, result in (("CLF OFF", off), ("CLF ON", on)):
        print(f"{label}: samples={result['executed_sample_count']}, "
              f"executed_observation_time=[{format_value(result['executed_observation_time_start'])}, "
              f"{format_value(result['executed_observation_time_end'])}] s, "
              f"span={format_value(result['actual_executed_time_span_s'])} s, "
              f"max|time_error|={format_value(result['max_abs_executed_time_error'])}, "
              f"mean|time_error|={format_value(result['mean_abs_executed_time_error'])}, "
              f"unmatched={result['unmatched_executed_messages']}, "
              f"max_grouping_wall_dt={format_value(result['max_pairing_bag_time_error_s'])} s")
        if result['max_abs_executed_time_error'] > 1.0e-6:
            print(f"WARNING: {label} max_abs_executed_time_error > 1e-6")
        if result['unmatched_executed_messages'] != 0:
            print(f"WARNING: {label} 存在未能配对的 executed diagnostics message")
    print("Settling time 定义：从原生 target 起点起，x error 进入 ±max(5 mm, step 的 10%)，并连续保持 2 s。")
    print("\nmetric                              CLF OFF          CLF ON       ON vs OFF")
    for key in ("step_amplitude_m", "peak_abs_x_error_m", "rms_x_error_m", "x_error_iae_m_s", "settling_time_s",
                "peak_abs_pitch_error_rad", "rms_position_error_m", "peak_sigma_norm", "mean_sigma_norm",
                "sigma_norm_integral", "time_weighted_mean_sigma_norm"):
        print(f"{key:34s} {format_value(off.get(key, unavailable())):>12s} {format_value(on.get(key, unavailable())):>15s} {percent_change(off.get(key, unavailable()), on.get(key, unavailable())):>14s}")
    print("\nExecuted Eq.(11): CLF OFF vs CLF ON")
    for key in ("mean_executed_h_clf", "min_executed_h_clf", "executed_h_negative_time_ratio",
                "mean_executed_violation", "max_executed_violation", "executed_violation_integral",
                "time_weighted_mean_executed_violation",
                "mean_executed_pitch_contribution", "min_executed_pitch_contribution"):
        print(f"{key:34s} {format_value(off.get(key, unavailable())):>12s} {format_value(on.get(key, unavailable())):>15s} {percent_change(off.get(key, unavailable()), on.get(key, unavailable())):>14s}")
    print("\nExecuted integral sanity checks")
    for key in ("sigma_nonnegative", "sigma_integral_upper_bound_ok", "violation_nonnegative", "violation_integral_upper_bound_ok"):
        print(f"{key:34s} {str(off.get(key)):>12s} {str(on.get(key)):>15s}")

    # 本轮 reference-knot 诊断：这些统计仍完全使用 executed_observation_time，
    # 所以可直接判断极值与 tau=0/0.20 s 速度跳变的时间关系。
    off_extrema = print_extrema("CLF OFF", off)
    on_extrema = print_extrema("CLF ON", on)
    for label, result in (("CLF OFF", off), ("CLF ON", on)):
        samples = result["_executed_window"]
        step_start = result["_step_start"]
        print_segment_metrics(f"{label}: entire 0..15 s", executed_segment_metrics(samples, step_start, 0.0, 15.0))
        print_segment_metrics(f"{label}: initial reference-transition 0..0.30 s",
                              executed_segment_metrics(samples, step_start, 0.0, 0.30))
        print_segment_metrics(f"{label}: post-transition 0.30..15 s",
                              executed_segment_metrics(samples, step_start, 0.30, 15.0))
    print_min_h_decomposition("CLF OFF", off, off_extrema["time_of_min_h_clf"])
    print_min_h_decomposition("CLF ON", on, on_extrema["time_of_min_h_clf"])
    for label, result, extrema in (("CLF OFF", off, off_extrema), ("CLF ON", on, on_extrema)):
        print_neighborhood(label, result, "neighborhood of min h", extrema["time_of_min_h_clf"])
        print_neighborhood(label, result, "neighborhood of peak sigma", extrema["time_of_peak_sigma_norm"])
        print_unmatched(label, result)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("analyze_aclf_ab.py:", error, file=sys.stderr)
        sys.exit(1)
