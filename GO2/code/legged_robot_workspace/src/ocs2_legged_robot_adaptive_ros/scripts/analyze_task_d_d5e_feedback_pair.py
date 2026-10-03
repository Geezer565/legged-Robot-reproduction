#!/usr/bin/env python3
"""D5E paired analysis: nominal walking with adaptation ON versus OFF."""

import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag


PACKAGE = Path(__file__).resolve().parents[1]
D5D_PATH = PACKAGE / "scripts" / "analyze_task_d_d5d_sigma_subterms.py"
SPEC = importlib.util.spec_from_file_location("d5d", str(D5D_PATH))
D5D = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D5D)
OBSERVATION = "/legged_robot_mpc_observation"
TARGET = "/legged_robot_mpc_target"
FLOW = "/task_d/equation10_full_flow_sample"
IDENTITY = "/task_d/model_identity"
SCHEDULE = "/legged_robot_mpc_mode_schedule"
STATUS = "/task_c/locomotion_scheduler_status"
FLOW_LABEL = "payload_inertial_eq10_full_flow_v2_80_fields"
FOCUS = (3, 4, 7)


def sha256(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def interp(grid, time, values):
    time = np.asarray(time, dtype=float)
    values = np.asarray(values, dtype=float)
    return np.column_stack([np.interp(grid, time, values[:, column])
                            for column in range(values.shape[1])])


def cumulative_trapezoid(time, values):
    time = np.asarray(time, dtype=float)
    values = np.asarray(values, dtype=float)
    result = np.zeros_like(values)
    result[1:] = np.cumsum(
        0.5 * np.diff(time)[:, None] * (values[:-1] + values[1:]), axis=0)
    return result


def rms(time, values):
    values = np.asarray(values, dtype=float)
    duration = float(time[-1] - time[0])
    return np.sqrt(np.trapz(values ** 2, time, axis=0) / duration)


def load_run(path, expected_mode):
    path = Path(path).resolve()
    config = json.loads((path / "effective_config.json").read_text())
    bag_path = path / "task_d_d5e.bag"
    decomposition, v3 = D5D.read_bag(bag_path)
    observations, targets, flows, statuses = [], [], [], []
    identity = None
    schedule_count = 0
    flow_labels = set()
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, message, _ in bag.read_messages(
                topics=[OBSERVATION, TARGET, FLOW, IDENTITY, SCHEDULE, STATUS]):
            if topic == OBSERVATION:
                observations.append((float(message.time), int(message.mode),
                                     np.asarray(message.state.value, dtype=float),
                                     np.asarray(message.input.value, dtype=float)))
            elif topic == TARGET:
                targets.append((np.asarray(message.timeTrajectory, dtype=float),
                                np.asarray([item.value for item in message.stateTrajectory], dtype=float)))
            elif topic == FLOW:
                flow_labels.add(message.layout.dim[0].label if message.layout.dim else "")
                flows.append(np.asarray(message.data, dtype=float))
            elif topic == IDENTITY:
                identity = json.loads(message.data)
            elif topic == SCHEDULE:
                schedule_count += 1
            elif topic == STATUS:
                statuses.append(json.loads(message.data))
    observations = [item for _, item in sorted(
        {item[0]: item for item in observations}.items())]
    flows = np.asarray(sorted(flows, key=lambda row: row[0]), dtype=float)
    observation_time = np.asarray([item[0] for item in observations])
    expected_enabled = expected_mode == "on"
    flags_ok = bool(
        config["enableParameterUpdate"] == expected_enabled and
        config["enableAdaptiveDynamics"] == expected_enabled and
        np.all(v3[:, 4] == float(expected_enabled)) and
        np.all(v3[:, 5] == float(expected_enabled)))
    target_ok = bool(len(targets) == 1 and targets[0][0].shape == (2,) and
                     targets[0][1].shape == (2, 24) and
                     abs(targets[0][0][0] - 1.2) <= 1.0e-9 and
                     abs(targets[0][0][1] - 1.4) <= 1.0e-9 and
                     abs(targets[0][1][1, 6] - targets[0][1][0, 6] - 0.1) <= 1.0e-6)
    identity_ok = bool(identity and identity.get("schema_version") == 2 and
                       identity.get("payload_inertial_override_active") is False)
    schema_ok = bool(len(observations) >= 50 and flows.shape[1:] == (80,) and
                     flow_labels == {FLOW_LABEL} and
                     all(len(item[2]) == 24 and len(item[3]) == 24 for item in observations))
    finite_ok = bool(np.all(np.isfinite(decomposition)) and np.all(np.isfinite(v3)) and
                     np.all(np.isfinite(flows)) and all(
                         np.all(np.isfinite(np.r_[item[2], item[3]])) for item in observations))
    schedule_ok = bool(schedule_count == 1 and any(item[1] != 15 for item in observations) and
                       any(item.get("stage") == "target_published" for item in statuses))
    pi_hat = decomposition[:, 22:32]
    total_rate = decomposition[:, 32:42] + decomposition[:, 42:52]
    _, cumulative = D5D.euler_contributions(decomposition[:, 0], total_rate)
    on_euler_error = float(np.max(np.abs(cumulative - (pi_hat - pi_hat[0]))))
    off_pi_max = float(np.max(np.abs(pi_hat)))
    update_path_ok = (on_euler_error <= 1.0e-9 if expected_enabled else off_pi_max <= 1.0e-12)
    applied_norm = np.linalg.norm(flows[:, 56:80], axis=1)
    dynamics_path_ok = bool(np.max(applied_norm) > 1.0e-8 if expected_enabled
                            else np.max(applied_norm) <= 1.0e-12)
    passed = bool(flags_ok and target_ok and identity_ok and schema_ok and finite_ok and
                  schedule_ok and update_path_ok and dynamics_path_ok)
    return {
        "path": path, "config": config, "bag": bag_path,
        "decomposition": decomposition, "v3": v3, "flows": flows,
        "observations": observations, "observation_time": observation_time,
        "target": targets[0] if targets else None,
        "validation": {"passed": passed, "flags": flags_ok, "target": target_ok,
                       "identity": identity_ok, "schema": schema_ok, "finite": finite_ok,
                       "schedule": schedule_ok, "update_path": update_path_ok,
                       "dynamics_path": dynamics_path_ok,
                       "online_euler_error": on_euler_error,
                       "off_pi_hat_max_abs": off_pi_max},
    }


def fair_configs(on, off):
    keys = ("enableParameterUpdate", "enableAdaptiveDynamics",
            "truePlantPayloadInertialEnabled")
    changed = [key for key in keys if on["config"][key] != off["config"][key]]
    return {
        "changed_keys": changed,
        "only_adaptive_switches_change": changed == ["enableParameterUpdate", "enableAdaptiveDynamics"],
        "mode_neutral_config_identical":
            on["config"]["mode_neutral_adaptive_config_sha256"] ==
            off["config"]["mode_neutral_adaptive_config_sha256"],
        "fixed_files_identical": on["config"]["fixed_files"] == off["config"]["fixed_files"],
        "source_hash_manifest_identical":
            (on["path"] / "source_files.sha256").read_text() ==
            (off["path"] / "source_files.sha256").read_text(),
    }


def grid_run(run, grid):
    rows = run["decomposition"]
    observations = run["observations"]
    flow = run["flows"]
    velocity_sigma = interp(grid, rows[:, 0], rows[:, 4:10])
    pose_sigma = interp(grid, rows[:, 0], rows[:, 10:16])
    sigma = interp(grid, rows[:, 0], rows[:, 16:22])
    pi_hat = interp(grid, rows[:, 0], rows[:, 22:32])
    rate = interp(grid, rows[:, 0], rows[:, 32:42] + rows[:, 42:52])
    state = interp(grid, run["observation_time"], [item[2] for item in observations])
    policy = interp(grid, run["observation_time"], [item[3] for item in observations])
    applied = interp(grid, flow[:, 0], flow[:, 56:80])
    target_time, target_state = run["target"]
    desired = interp(grid, target_time, target_state)
    return {"velocity_sigma": velocity_sigma, "pose_sigma": pose_sigma,
            "sigma": sigma, "pi_hat": pi_hat, "rate": rate,
            "hypothetical_cumulative": cumulative_trapezoid(grid, rate),
            "state": state, "policy": policy, "applied": applied,
            "desired": desired}


def endpoint_euler(run, end):
    rows = run["decomposition"]
    rows = rows[rows[:, 0] <= end + 1.0e-9]
    rate = rows[:, 32:42] + rows[:, 42:52]
    _, cumulative = D5D.euler_contributions(rows[:, 0], rate)
    return cumulative[-1]


def analyze(root, output):
    root = Path(root).resolve(); output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    runs = {name: load_run(root / name / "run_01", name) for name in ("on", "off")}
    fairness = fair_configs(runs["on"], runs["off"])
    fairness_passed = all(value for key, value in fairness.items() if key != "changed_keys")
    end = min(8.0, *(run["decomposition"][-1, 0] for run in runs.values()),
              *(run["observation_time"][-1] for run in runs.values()))
    grid = np.linspace(0.0, end, int(round(end / 0.02)) + 1)
    data = {name: grid_run(run, grid) for name, run in runs.items()}

    metrics = {}
    for name in ("on", "off"):
        item = data[name]
        x_error = item["desired"][:, 6] - item["state"][:, 6]
        metrics[name] = {
            "velocity_sigma_rms": rms(grid, item["velocity_sigma"]).tolist(),
            "pose_sigma_rms": rms(grid, item["pose_sigma"]).tolist(),
            "sigma_rms": rms(grid, item["sigma"]).tolist(),
            "hypothetical_euler_endpoint": endpoint_euler(runs[name], end).tolist(),
            "x_rmse_m": float(rms(grid, x_error[:, None])[0]),
            "actual_x_displacement_m": float(item["state"][-1, 6] - item["state"][0, 6]),
            "applied_correction_rms": float(rms(grid, np.linalg.norm(item["applied"], axis=1)[:, None])[0]),
        }
    comparison = {
        "angular_x_velocity_error_rms_on_over_off":
            metrics["on"]["velocity_sigma_rms"][3] / metrics["off"]["velocity_sigma_rms"][3],
        "angular_y_velocity_error_rms_on_over_off":
            metrics["on"]["velocity_sigma_rms"][4] / metrics["off"]["velocity_sigma_rms"][4],
        "six_axis_velocity_error_rms_difference": float(rms(
            grid, data["on"]["velocity_sigma"] - data["off"]["velocity_sigma"]).sum()),
        "policy_rms_difference": float(rms(
            grid, np.linalg.norm(data["on"]["policy"] - data["off"]["policy"], axis=1)[:, None])[0]),
    }
    for index in FOCUS:
        on_value = metrics["on"]["hypothetical_euler_endpoint"][index]
        off_value = metrics["off"]["hypothetical_euler_endpoint"][index]
        comparison[D5D.PARAMETERS[index] + "_hypothetical_endpoint_on_over_off"] = (
            on_value / off_value if abs(off_value) > 1.0e-12 else math.nan)

    rows = []
    for name in ("on", "off"):
        for index, parameter in enumerate(D5D.PARAMETERS):
            rows.append({"mode": name, "parameter": parameter,
                         "hypothetical_euler_endpoint":
                             metrics[name]["hypothetical_euler_endpoint"][index],
                         "logged_pi_hat_endpoint": float(data[name]["pi_hat"][-1, index])})
    with (output / "parameter_pair_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for plot, axis_index in zip(axes, (3, 4)):
        for name, style in (("on", "-"), ("off", "--")):
            plot.plot(grid, data[name]["velocity_sigma"][:, axis_index], style, label=name)
        plot.set_ylabel("velocity error " + D5D.AXES[axis_index]); plot.grid(True, alpha=0.3); plot.legend()
    axes[-1].set_xlabel("MPC time [s]"); fig.suptitle("D5E nominal walking angular-velocity residual: adaptation ON vs OFF")
    fig.tight_layout(); fig.savefig(output / "figure_D5E_01_angular_velocity_error_pair.png", dpi=170); plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for plot, index in zip(axes, FOCUS):
        for name, style in (("on", "-"), ("off", "--")):
            plot.plot(grid, data[name]["hypothetical_cumulative"][:, index], style,
                      label=name + " hypothetical integral")
        plot.plot(grid, data["on"]["pi_hat"][:, index] - data["on"]["pi_hat"][0, index],
                  color="black", alpha=0.55, label="ON logged pi_hat change")
        plot.set_ylabel(D5D.PARAMETERS[index]); plot.grid(True, alpha=0.3); plot.legend()
    axes[-1].set_xlabel("MPC time [s]"); fig.suptitle("D5E hypothetical Eq.(8) accumulation")
    fig.tight_layout(); fig.savefig(output / "figure_D5E_02_hypothetical_drive_pair.png", dpi=170); plt.close(fig)

    x = np.arange(6); width = 0.36
    fig, axis = plt.subplots(figsize=(11, 5.5))
    axis.bar(x - width / 2, metrics["on"]["velocity_sigma_rms"], width, label="ON")
    axis.bar(x + width / 2, metrics["off"]["velocity_sigma_rms"], width, label="OFF")
    axis.set_xticks(x); axis.set_xticklabels(D5D.AXES, rotation=20, ha="right")
    axis.set_ylabel("velocity-error RMS"); axis.set_title("D5E velocity tracking residual magnitude")
    axis.grid(True, axis="y", alpha=0.3); axis.legend(); fig.tight_layout()
    fig.savefig(output / "figure_D5E_03_velocity_error_rms.png", dpi=170); plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for name, style in (("on", "-"), ("off", "--")):
        axes[0].plot(grid, data[name]["desired"][:, 6] - data[name]["state"][:, 6], style, label=name)
        axes[1].plot(grid, np.linalg.norm(data[name]["applied"], axis=1), style, label=name)
    axes[0].set_ylabel("x error [m]"); axes[1].set_ylabel("applied correction norm")
    axes[1].set_xlabel("MPC time [s]")
    for plot in axes: plot.grid(True, alpha=0.3); plot.legend()
    fig.suptitle("D5E closed-loop tracking and Eq.(10) activity")
    fig.tight_layout(); fig.savefig(output / "figure_D5E_04_tracking_and_correction.png", dpi=170); plt.close(fig)

    validation_passed = all(run["validation"]["passed"] for run in runs.values())
    passed = bool(validation_passed and fairness_passed)
    ratios = [comparison[key] for key in (
        "angular_x_velocity_error_rms_on_over_off",
        "angular_y_velocity_error_rms_on_over_off")]
    if all(0.9 <= value <= 1.1 for value in ratios):
        mechanism = "angular-velocity residual magnitude is already present with adaptation OFF; ON does not materially amplify its RMS"
    elif any(value > 1.1 for value in ratios):
        mechanism = "adaptation ON increases at least one dominant angular-velocity residual RMS by more than 10%"
    else:
        mechanism = "adaptation ON reduces at least one dominant angular-velocity residual RMS; inspect component ratios and phase"
    summary = {
        "schema_version": 1, "stage": "D5E_nominal_walking_adaptation_feedback",
        "passed": passed, "fairness": fairness,
        "run_validation": {name: run["validation"] for name, run in runs.items()},
        "analysis_grid": {"start_s": 0.0, "end_s": end, "dt_s": 0.02},
        "metrics": metrics, "comparison": comparison,
        "mechanism_observation": mechanism,
        "scientific_limit": "single paired diagnostic run; ratios are observations, not population statistics",
        "input_bags": {name: {"path": str(run["bag"]), "sha256": sha256(run["bag"])}
                       for name, run in runs.items()},
    }
    (output / "stage_d5e_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    focus_lines = []
    for index in FOCUS:
        parameter = D5D.PARAMETERS[index]
        on_value = metrics["on"]["hypothetical_euler_endpoint"][index]
        off_value = metrics["off"]["hypothetical_euler_endpoint"][index]
        focus_lines.append("| {} | {:.6g} | {:.6g} | {:.3f} |".format(
            parameter, on_value, off_value,
            comparison[parameter + "_hypothetical_endpoint_on_over_off"]))
    report = """# Stage D5E — Nominal walking adaptation-feedback pair

## Fair comparison

- 两组均为 nominal plant、相同 10 cm target、相同 static-walk schedule、相同初值和 solver/config。
- 唯一变化：`enableParameterUpdate` 与 `enableAdaptiveDynamics`。
- ON/OFF run validation：`{on_status}` / `{off_status}`；fairness：`{fairness_status}`。

## Dominant velocity residual RMS

| channel | ON | OFF | ON/OFF |
|---|---:|---:|---:|
| angular_x | {on_ax:.6g} | {off_ax:.6g} | {ratio_ax:.3f} |
| angular_y | {on_ay:.6g} | {off_ay:.6g} | {ratio_ay:.3f} |

## Hypothetical Eq.(8) endpoint accumulation

OFF 组的 pi_hat 被固定为零，但这里仍以完全相同的 `Gamma Y^T sigma` 计算 hypothetical accumulation。

| parameter | ON | OFF hypothetical | ON/OFF |
|---|---:|---:|---:|
{focus_rows}

## Observation

`{mechanism}`

本阶段只有每组一次短时配对，因此是机制诊断，不是正式统计结论。它不修改 estimator，也不把 nominal residual 解释为 payload。
""".format(
        on_status="PASS" if runs["on"]["validation"]["passed"] else "FAIL",
        off_status="PASS" if runs["off"]["validation"]["passed"] else "FAIL",
        fairness_status="PASS" if fairness_passed else "FAIL",
        on_ax=metrics["on"]["velocity_sigma_rms"][3], off_ax=metrics["off"]["velocity_sigma_rms"][3],
        ratio_ax=comparison["angular_x_velocity_error_rms_on_over_off"],
        on_ay=metrics["on"]["velocity_sigma_rms"][4], off_ay=metrics["off"]["velocity_sigma_rms"][4],
        ratio_ay=comparison["angular_y_velocity_error_rms_on_over_off"],
        focus_rows="\n".join(focus_lines), mechanism=mechanism)
    (output / "stage_d5e_report.md").write_text(report)
    artifacts = []
    for path in sorted(output.iterdir()):
        if path.is_file() and path.name != "stage_d5e_artifact_manifest.json":
            artifacts.append({"path": path.name, "bytes": path.stat().st_size, "sha256": sha256(path)})
    (output / "stage_d5e_artifact_manifest.json").write_text(json.dumps(
        {"schema_version": 1, "passed": passed, "artifacts": artifacts}, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"passed": passed, "mechanism": mechanism,
                      "comparison": comparison}, indent=2))
    return 0 if passed else 1


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("campaign_root"); parser.add_argument("output_directory")
    args = parser.parse_args(); return analyze(args.campaign_root, args.output_directory)


if __name__ == "__main__":
    raise SystemExit(main())
