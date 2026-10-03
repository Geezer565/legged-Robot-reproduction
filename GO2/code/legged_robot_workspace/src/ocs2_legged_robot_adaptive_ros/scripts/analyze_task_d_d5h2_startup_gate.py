#!/usr/bin/env python3
"""Offline causal screening for the D5H2 estimator startup gate.

The counterfactual update is deliberately simple:

  pi_dot_gate(t) = pi_dot_D5H1(t), if gait baseline correction is active
                   0,                otherwise.

It replays the recorded D5H1 closed-loop trajectory, so it screens the update
law but does not predict the closed-loop trajectory of a future runtime gate.
"""

import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz",
              "Jyy", "Jyz", "Jzz")
GROUPS = ("nominal_corrected", "uncertain_corrected")


def truth_and_scales(config):
    payload = config["payload"]
    mass = float(payload["mass_kg"])
    com = np.asarray(payload["com_nominal_c0_base_m"], dtype=float)
    inertia = np.asarray(payload["inertia_com_base_kg_m2"], dtype=float)
    truth = np.asarray([mass, *(mass * com), *inertia], dtype=float)
    length = float(np.linalg.norm(com))
    inertia_scale = max(abs(float(value)) for value in inertia)
    scales = np.asarray([mass] + [mass * length] * 3 +
                        [inertia_scale] * 6, dtype=float)
    if np.any(scales <= 0.0):
        raise ValueError("invalid physical scales")
    return truth, scales


def replay(time, derivative, correction_active, initial=None):
    """Explicit-Euler replay using derivative[k] over (time[k-1], time[k])."""
    time = np.asarray(time, dtype=float)
    derivative = np.asarray(derivative, dtype=float)
    correction_active = np.asarray(correction_active, dtype=bool)
    if time.ndim != 1 or derivative.shape != (len(time), 10):
        raise ValueError("D5H2 replay input shape mismatch")
    if correction_active.shape != (len(time),):
        raise ValueError("D5H2 correction flag shape mismatch")
    if len(time) < 2 or np.any(np.diff(time) <= 0.0):
        raise ValueError("D5H2 time must be strictly increasing")
    if not np.all(np.isfinite(time)) or not np.all(np.isfinite(derivative)):
        raise ValueError("D5H2 replay input is nonfinite")
    estimate = np.zeros_like(derivative)
    if initial is not None:
        estimate[0] = np.asarray(initial, dtype=float)
    gated_derivative = np.where(correction_active[:, None], derivative, 0.0)
    for index in range(1, len(time)):
        estimate[index] = (estimate[index - 1]
                           + (time[index] - time[index - 1])
                           * gated_derivative[index])
    return estimate, gated_derivative


def read_run(root, group):
    directory = root / group / "run_01"
    summary = json.loads(
        (directory / "task_d_d5h1_smoke_summary.json").read_text())
    config = json.loads((directory / "effective_config.json").read_text())
    with (directory / "task_d_d5h1_timeseries.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("empty D5H1 timeseries: {}".format(directory))
    time = np.asarray([float(row["time"]) for row in rows])
    active = np.asarray(
        [float(row["correction_applied"]) == 1.0 for row in rows])
    phase_bin = np.asarray([int(row["phase_bin"]) for row in rows])
    derivative = np.asarray([
        [float(row["pi_dot_" + name]) for name in PARAMETERS]
        for row in rows])
    estimate = np.asarray([
        [float(row["pi_hat_" + name]) for name in PARAMETERS]
        for row in rows])
    reconstructed = np.zeros_like(estimate)
    reconstructed[0] = estimate[0]
    for index in range(1, len(time)):
        reconstructed[index] = (
            reconstructed[index - 1]
            + (time[index] - time[index - 1]) * derivative[index])
    closure = float(np.max(np.abs(reconstructed - estimate)))
    gated, gated_derivative = replay(time, derivative, active, estimate[0])
    first_active = int(np.flatnonzero(active)[0]) if np.any(active) else -1
    return {
        "directory": directory,
        "summary": summary,
        "config": config,
        "time": time,
        "active": active,
        "phase_bin": phase_bin,
        "derivative": derivative,
        "estimate": estimate,
        "gated": gated,
        "gated_derivative": gated_derivative,
        "reconstruction_closure": closure,
        "first_active_index": first_active,
        "first_active_time": (float(time[first_active])
                              if first_active >= 0 else None),
    }


def cycle_increments(run, scales):
    """Split applied estimator increments using the logged 23->0 phase wrap."""
    cycles = []
    current = None
    previous_bin = None
    for index in range(1, len(run["time"])):
        if not run["active"][index]:
            continue
        phase_bin = int(run["phase_bin"][index])
        if current is None or (previous_bin == 23 and phase_bin == 0):
            if current is not None:
                current["complete"] = bool(current["last_phase_bin"] == 23)
                cycles.append(current)
            current = {
                "cycle_index": len(cycles),
                "start_sample_time": float(run["time"][index]),
                "end_sample_time": float(run["time"][index]),
                "first_phase_bin": phase_bin,
                "last_phase_bin": phase_bin,
                "sample_count": 0,
                "increment": np.zeros(10),
                "complete": False,
            }
        dt = run["time"][index] - run["time"][index - 1]
        current["increment"] += dt * run["gated_derivative"][index]
        current["end_sample_time"] = float(run["time"][index])
        current["last_phase_bin"] = phase_bin
        current["sample_count"] += 1
        previous_bin = phase_bin
    if current is not None:
        current["complete"] = bool(current["last_phase_bin"] == 23)
        cycles.append(current)
    for cycle in cycles:
        cycle["scaled_increment_norm"] = scaled_norm(
            cycle["increment"], scales)
    return cycles


def scaled_norm(vector, scales):
    return float(np.linalg.norm(np.asarray(vector) / scales))


def scaled_rmse(vector, truth, scales):
    return float(np.sqrt(np.mean(((np.asarray(vector) - truth) / scales) ** 2)))


def scaled_cosine(vector, truth, scales):
    left = np.asarray(vector) / scales
    right = np.asarray(truth) / scales
    denominator = np.linalg.norm(left) * np.linalg.norm(right)
    return float(np.dot(left, right) / denominator) if denominator > 0.0 else None


def write_replay_csv(path, run):
    names = ["time", "correction_active"]
    names += ["logged_pi_hat_" + name for name in PARAMETERS]
    names += ["gated_pi_hat_" + name for name in PARAMETERS]
    names += ["gated_pi_dot_" + name for name in PARAMETERS]
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        for index, time in enumerate(run["time"]):
            writer.writerow(
                [time, int(run["active"][index])]
                + run["estimate"][index].tolist()
                + run["gated"][index].tolist()
                + run["gated_derivative"][index].tolist())


def main():
    if len(sys.argv) not in (2, 3):
        raise SystemExit(
            "usage: analyze_task_d_d5h2_startup_gate.py D5H1_ROOT [OUTPUT_DIR]")
    root = Path(sys.argv[1]).resolve()
    output = (Path(sys.argv[2]).resolve() if len(sys.argv) == 3
              else root / "analysis_d5h2_startup_gate")
    output.mkdir(parents=True, exist_ok=True)
    runs = {group: read_run(root, group) for group in GROUPS}
    nominal = runs["nominal_corrected"]
    uncertain = runs["uncertain_corrected"]
    truth, scales = truth_and_scales(uncertain["config"])

    configs_match = bool(
        nominal["config"]["gaitPhaseBaselineSha256"]
        == uncertain["config"]["gaitPhaseBaselineSha256"]
        and nominal["config"]["gamma"] == uncertain["config"]["gamma"]
        and nominal["config"]["target"] == uncertain["config"]["target"]
        and nominal["config"]["gait"] == uncertain["config"]["gait"]
        and nominal["config"]["fixed_files"]
        == uncertain["config"]["fixed_files"])
    inputs_valid = bool(
        all(run["summary"]["passed"] for run in runs.values())
        and all(run["reconstruction_closure"] <= 1.0e-10
                for run in runs.values())
        and all(run["first_active_index"] > 0 for run in runs.values())
        and abs(nominal["first_active_time"]
                - uncertain["first_active_time"]) <= 1.0e-12)

    n_actual = nominal["estimate"][-1]
    n_gated = nominal["gated"][-1]
    u_actual = uncertain["estimate"][-1]
    u_gated = uncertain["gated"][-1]
    actual_contrast = u_actual - n_actual
    gated_contrast = u_gated - n_gated
    n_actual_norm = scaled_norm(n_actual, scales)
    n_gated_norm = scaled_norm(n_gated, scales)
    actual_contrast_norm = scaled_norm(actual_contrast, scales)
    gated_contrast_norm = scaled_norm(gated_contrast, scales)
    nominal_suppression = 1.0 - n_gated_norm / n_actual_norm
    contrast_retention = gated_contrast_norm / actual_contrast_norm
    actual_rmse = scaled_rmse(actual_contrast, truth, scales)
    gated_rmse = scaled_rmse(gated_contrast, truth, scales)
    actual_cosine = scaled_cosine(actual_contrast, truth, scales)
    gated_cosine = scaled_cosine(gated_contrast, truth, scales)
    cycle_data = {
        group: cycle_increments(run, scales) for group, run in runs.items()
    }

    # Engineering screening criteria, fixed independently of a convergence
    # claim. They only decide whether a default-off runtime smoke is worth it.
    criteria = {
        "nominal_residual_reduction_at_least_80_percent":
            nominal_suppression >= 0.80,
        "payload_contrast_norm_retention_at_least_50_percent":
            contrast_retention >= 0.50,
        "payload_contrast_not_anti_aligned":
            gated_cosine is not None and gated_cosine >= 0.0,
        "scaled_rmse_degradation_no_more_than_0_1":
            gated_rmse <= actual_rmse + 0.10,
    }
    screening_passed = bool(configs_match and inputs_valid
                            and all(criteria.values()))

    metrics = {
        "first_gate_active_time_s": nominal["first_active_time"],
        "nominal_actual_corrected_terminal_scaled_norm": n_actual_norm,
        "nominal_gated_terminal_scaled_norm": n_gated_norm,
        "nominal_additional_suppression_fraction": nominal_suppression,
        "actual_corrected_payload_contrast_scaled_norm": actual_contrast_norm,
        "gated_payload_contrast_scaled_norm": gated_contrast_norm,
        "gated_payload_contrast_retention_fraction": contrast_retention,
        "actual_corrected_payload_contrast_scaled_rmse_vs_pi_star": actual_rmse,
        "gated_payload_contrast_scaled_rmse_vs_pi_star": gated_rmse,
        "actual_corrected_payload_contrast_scaled_cosine_vs_pi_star": actual_cosine,
        "gated_payload_contrast_scaled_cosine_vs_pi_star": gated_cosine,
    }

    for group, run in runs.items():
        write_replay_csv(output / (group + "_d5h2_offline_replay.csv"), run)
    with (output / "d5h2_cycle_increment_decomposition.csv").open(
            "w", newline="") as stream:
        fieldnames = (["group", "cycle_index", "complete",
                       "start_sample_time", "end_sample_time",
                       "first_phase_bin", "last_phase_bin", "sample_count",
                       "scaled_increment_norm"]
                      + ["increment_" + name for name in PARAMETERS])
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for group, cycles in cycle_data.items():
            for cycle in cycles:
                row = {key: value for key, value in cycle.items()
                       if key != "increment"}
                row["group"] = group
                row.update({"increment_" + name: float(cycle["increment"][i])
                            for i, name in enumerate(PARAMETERS)})
                writer.writerow(row)

    component_rows = []
    for index, name in enumerate(PARAMETERS):
        component_rows.append({
            "parameter": name,
            "scale": float(scales[index]),
            "pi_star": float(truth[index]),
            "nominal_actual_corrected": float(n_actual[index]),
            "nominal_gated": float(n_gated[index]),
            "uncertain_actual_corrected": float(u_actual[index]),
            "uncertain_gated": float(u_gated[index]),
            "actual_corrected_U_minus_N": float(actual_contrast[index]),
            "gated_U_minus_N": float(gated_contrast[index]),
        })
    with (output / "d5h2_terminal_parameter_comparison.csv").open(
            "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(component_rows[0]))
        writer.writeheader()
        writer.writerows(component_rows)

    summary = {
        "stage": "D5H2_offline_startup_gate_screening",
        "candidate": (
            "pi_dot=logged D5H1 corrected derivative only while "
            "correction_applied; otherwise pi_dot=0"),
        "screening_passed": screening_passed,
        "runtime_implementation_recommended": screening_passed,
        "formal_eligibility": False,
        "checks": {
            "D5H1_inputs_valid": inputs_valid,
            "paired_configs_match": configs_match,
            **criteria,
        },
        "reconstruction_closure_max_abs": {
            group: run["reconstruction_closure"]
            for group, run in runs.items()
        },
        "payload_pi_star": truth.tolist(),
        "physical_coordinate_scales": dict(zip(PARAMETERS, scales.tolist())),
        "terminal_vectors": {
            "nominal_actual_corrected": n_actual.tolist(),
            "nominal_gated": n_gated.tolist(),
            "uncertain_actual_corrected": u_actual.tolist(),
            "uncertain_gated": u_gated.tolist(),
            "actual_corrected_U_minus_N": actual_contrast.tolist(),
            "gated_U_minus_N": gated_contrast.tolist(),
        },
        "metrics": metrics,
        "cycle_increment_decomposition": {
            group: [
                {**{key: value for key, value in cycle.items()
                    if key != "increment"},
                 "increment": cycle["increment"].tolist()}
                for cycle in cycles]
            for group, cycles in cycle_data.items()
        },
        "interpretation_boundary": (
            "This is a fixed-trajectory counterfactual replay. A runtime gate "
            "would change pi_hat, Eq.(10), policy, state, sigma, and subsequent "
            "updates, so new closed-loop smoke data remain mandatory."),
    }
    (output / "stage_d5h2_startup_gate_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")

    figure, axes = plt.subplots(2, 1, figsize=(12, 8), constrained_layout=True)
    for group, color in (("nominal_corrected", "tab:blue"),
                         ("uncertain_corrected", "tab:orange")):
        run = runs[group]
        axes[0].plot(run["time"],
                     np.linalg.norm(run["estimate"] / scales, axis=1),
                     color=color, linestyle="--", label=group + " logged")
        axes[0].plot(run["time"],
                     np.linalg.norm(run["gated"] / scales, axis=1),
                     color=color, label=group + " gated replay")
    axes[0].axvline(nominal["first_active_time"], color="black",
                    linestyle=":", label="gate first active")
    axes[0].set_xlabel("MPC time [s]")
    axes[0].set_ylabel("scaled ||pi_hat||")
    axes[0].set_title("D5H2 offline causal startup gate")
    axes[0].grid(alpha=0.3)
    axes[0].legend()

    x = np.arange(len(PARAMETERS))
    axes[1].plot(x, truth / scales, "ko--", label="payload pi_star")
    axes[1].plot(x, actual_contrast / scales, "o-",
                 label="D5H1 corrected U-N")
    axes[1].plot(x, gated_contrast / scales, "o-",
                 label="D5H2 gated replay U-N")
    axes[1].axhline(0.0, color="black", linewidth=0.7)
    axes[1].set_xticks(x, PARAMETERS)
    axes[1].set_ylabel("parameter / physical scale")
    axes[1].set_title("Payload contrast retained after startup gating")
    axes[1].grid(alpha=0.3)
    axes[1].legend()
    figure.savefig(output / "figure_D5H2_offline_startup_gate.png", dpi=180)
    plt.close(figure)

    table = "\n".join(
        "| {parameter} | {pi_star:.6g} | {nominal_actual_corrected:.6g} | "
        "{nominal_gated:.6g} | {actual_corrected_U_minus_N:.6g} | "
        "{gated_U_minus_N:.6g} |".format(**row)
        for row in component_rows)
    cycle_table = "\n".join(
        "| {} | {} | {} | {:.6g} | {:.6g} | {:.6g} |".format(
            group, cycle["cycle_index"], cycle["complete"],
            cycle["start_sample_time"], cycle["end_sample_time"],
            cycle["scaled_increment_norm"])
        for group, cycles in cycle_data.items() for cycle in cycles)
    report = """# Stage D5H2 offline startup-gate screening

## Result

- Engineering screening: **{result}**
- Runtime implementation recommended: **{recommended}**
- Formal scientific eligibility: **NO**
- Gate first active: `{gate_time:.6g} s` MPC time

| parameter | pi_star | N D5H1 | N gated | D5H1 U-N | gated U-N |
|---|---:|---:|---:|---:|---:|
{table}

## Physical-scale metrics

- Nominal terminal norm: D5H1 `{n_actual:.8g}` -> gated `{n_gated:.8g}`;
  additional suppression **{n_suppression:.2%}**.
- Payload contrast norm: D5H1 `{c_actual:.8g}` -> gated `{c_gated:.8g}`;
  retention **{retention:.2%}**.
- Payload contrast RMSE vs pi_star: D5H1 `{rmse_actual:.8g}` -> gated `{rmse_gated:.8g}`.
- Payload contrast cosine vs pi_star: D5H1 `{cos_actual:.8g}` -> gated `{cos_gated:.8g}`.

## Residual by gait cycle

| group | cycle | complete | first sample [s] | last sample [s] | scaled increment norm |
|---|---:|---|---:|---:|---:|
{cycle_table}

## Boundary

This is a fixed-trajectory counterfactual replay, not a new closed-loop run.
It shows what the logged Eq.(8) increments would have produced if the estimator
had been frozen before the gait baseline became causally valid. A runtime gate
will change Eq.(10), policy, state and future sigma, so a default-off online
implementation plus a new four-mode smoke is still required before any formal
robustness claim.
""".format(
        result="PASS" if screening_passed else "FAIL",
        recommended="YES" if screening_passed else "NO",
        gate_time=nominal["first_active_time"], table=table,
        cycle_table=cycle_table,
        n_actual=n_actual_norm, n_gated=n_gated_norm,
        n_suppression=nominal_suppression,
        c_actual=actual_contrast_norm, c_gated=gated_contrast_norm,
        retention=contrast_retention, rmse_actual=actual_rmse,
        rmse_gated=gated_rmse, cos_actual=actual_cosine,
        cos_gated=gated_cosine)
    (output / "stage_d5h2_startup_gate_report.md").write_text(report)
    print(report)
    if not screening_passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
