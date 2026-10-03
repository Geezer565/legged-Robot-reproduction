#!/usr/bin/env python3
"""Generate additional Task C estimate-vs-ground-truth diagnostic figures.

This is offline post-processing only.  It never changes the controller, the
adaptive update, the frozen snapshots, or any recorded run.
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


GROUPS = ("nominal", "uncertain_adaptive", "uncertain_off")
UNCERTAIN_GROUPS = ("uncertain_adaptive", "uncertain_off")
RUNS = ("run_01", "run_02", "run_03")
FORCE_COMPONENTS = ("Fx", "Fy", "Fz")
GROUP_LABELS = {
    "nominal": "Nominal plant + Full Adaptive",
    "uncertain_adaptive": "+10% mass plant + Full Adaptive",
    "uncertain_off": "+10% mass plant + Adaptive OFF",
}
GROUP_COLORS = {
    "nominal": "#555555",
    "uncertain_adaptive": "#d62728",
    "uncertain_off": "#1f77b4",
}
WINDOW_START = 0.50
WINDOW_END = 15.50
GRID_DT = 0.02
TARGET_RAMP_END = 0.70
GRAVITY = 9.81


def load_aligned_run(path, grid):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("empty Task C CSV: {}".format(path))

    required = ["observation_time", "nominal_mass", "true_mass", "payload_delta"]
    required += ["pi_hat_" + name for name in FORCE_COMPONENTS]
    required += ["pi_star_" + name for name in FORCE_COMPONENTS]
    missing = [name for name in required if name not in rows[0]]
    if missing:
        raise RuntimeError("missing fields in {}: {}".format(path, missing))

    time = np.asarray([float(row["observation_time"]) for row in rows])
    if np.any(~np.isfinite(time)) or np.any(np.diff(time) <= 0.0):
        raise RuntimeError("non-finite or non-monotonic time in {}".format(path))
    if time[0] > grid[0] or time[-1] < grid[-1]:
        raise RuntimeError("run does not cover common analysis grid: {}".format(path))

    def vector(prefix):
        raw = np.asarray([
            [float(row[prefix + name]) for name in FORCE_COMPONENTS]
            for row in rows
        ])
        if np.any(~np.isfinite(raw)):
            raise RuntimeError("non-finite {} in {}".format(prefix, path))
        return np.column_stack([
            np.interp(grid, time, raw[:, index]) for index in range(3)
        ])

    nominal_mass = np.asarray([float(row["nominal_mass"]) for row in rows])
    true_mass = np.asarray([float(row["true_mass"]) for row in rows])
    payload_delta = np.asarray([float(row["payload_delta"]) for row in rows])
    for name, values in (("nominal_mass", nominal_mass), ("true_mass", true_mass),
                         ("payload_delta", payload_delta)):
        if np.any(~np.isfinite(values)) or np.max(values) - np.min(values) > 1.0e-9:
            raise RuntimeError("{} is invalid or changes within {}".format(name, path))

    return {
        "pi_hat_force": vector("pi_hat_"),
        "pi_star_force": vector("pi_star_"),
        "nominal_mass": float(nominal_mass[0]),
        "true_mass": float(true_mass[0]),
        "payload_delta": float(payload_delta[0]),
    }


def load_campaign(root, grid):
    campaign = {}
    for group in GROUPS:
        campaign[group] = []
        for run in RUNS:
            path = root / group / run / "task_c_run.csv"
            if not path.is_file():
                raise RuntimeError("missing formal run CSV: {}".format(path))
            campaign[group].append(load_aligned_run(path, grid))
    return campaign


def mean_std(arrays):
    values = np.stack(arrays)
    return np.mean(values, axis=0), np.std(values, axis=0, ddof=1)


def decorate(axis, ylabel):
    axis.axvspan(WINDOW_START, TARGET_RAMP_END, color="#2ca02c", alpha=0.08,
                 label="0.10 m target ramp" if not axis.get_legend_handles_labels()[0] else None)
    axis.axvline(TARGET_RAMP_END, color="#777777", linestyle=":", linewidth=1.0)
    axis.axhline(0.0, color="#999999", linewidth=0.8)
    axis.grid(True, alpha=0.28)
    axis.set_xlim(WINDOW_START, WINDOW_END)
    axis.set_ylabel(ylabel)


def plot_equivalent_payload_mass(path, grid, campaign):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(len(GROUPS), 1, figsize=(11, 10.8), sharex=True)
    for axis, group in zip(axes, GROUPS):
        runs = campaign[group]
        hat_mean, hat_std = mean_std([run["pi_hat_force"][:, 2] / GRAVITY for run in runs])
        star_mean, star_std = mean_std([run["pi_star_force"][:, 2] / GRAVITY for run in runs])
        payload_values = [run["payload_delta"] for run in runs]
        if max(payload_values) - min(payload_values) > 1.0e-9:
            raise RuntimeError("payload delta differs across {} runs".format(group))
        payload_delta = payload_values[0]
        color = GROUP_COLORS[group]

        axis.plot(grid, hat_mean, color=color, linewidth=2.0,
                  label=r"$\hat{\pi}_{Fz}/g$ (adaptive equivalent)")
        axis.fill_between(grid, hat_mean - hat_std, hat_mean + hat_std,
                          color=color, alpha=0.15)
        axis.plot(grid, star_mean, color="black", linewidth=1.8, linestyle="--",
                  label=r"$\pi^\star_{Fz}/g$ (model-residual truth)")
        axis.fill_between(grid, star_mean - star_std, star_mean + star_std,
                          color="black", alpha=0.07)
        axis.axhline(payload_delta, color="#2ca02c", linewidth=1.6, linestyle="-.",
                     label=r"actual $\Delta m$ = {:.4f} kg".format(payload_delta))
        decorate(axis, "Fz-equivalent payload (kg)")
        axis.set_title(GROUP_LABELS[group])
        axis.legend(loc="best")

    axes[-1].set_xlabel("MPC / simulation time (s)")
    figure.suptitle("Task C: Fz-equivalent payload diagnostic\n"
                    r"Conversion only: $m_{eq}=\pi_{Fz}/g$; $\hat{\pi}$ is not a mass estimator",
                    y=0.995)
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.965))
    figure.savefig(path, dpi=180)
    plt.close(figure)


def plot_force_component_errors(path, grid, campaign):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(len(GROUPS), len(FORCE_COMPONENTS),
                               figsize=(17, 10.5), sharex=True, sharey="col")
    for row_index, group in enumerate(GROUPS):
        runs = campaign[group]
        for column_index, component in enumerate(FORCE_COMPONENTS):
            axis = axes[row_index, column_index]
            errors = [
                run["pi_hat_force"][:, column_index] - run["pi_star_force"][:, column_index]
                for run in runs
            ]
            mean, std = mean_std(errors)
            color = GROUP_COLORS[group]
            axis.plot(grid, mean, color=color, linewidth=1.9,
                      label=r"$\hat{{\pi}}_{{{}}}-\pi^\star_{{{}}}$".format(component, component))
            axis.fill_between(grid, mean - std, mean + std, color=color, alpha=0.15)
            decorate(axis, "{} error (N)".format(component))
            if row_index == 0:
                axis.set_title(component + " estimation error")
            if column_index == 0:
                axis.text(0.015, 0.92, GROUP_LABELS[group], transform=axis.transAxes,
                          ha="left", va="top", fontsize=10,
                          bbox=dict(facecolor="white", alpha=0.78, edgecolor="none"))
            if row_index == len(GROUPS) - 1:
                axis.set_xlabel("MPC / simulation time (s)")
            axis.legend(loc="best", fontsize=8)

    figure.suptitle(r"Task C force-component estimation error: $e_F=\hat{F}-F^\star$",
                    y=0.995)
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.975))
    figure.savefig(path, dpi=180)
    plt.close(figure)


def force_direction_signals(run):
    estimate = run["pi_hat_force"]
    truth = run["pi_star_force"]
    truth_norm = np.linalg.norm(truth, axis=1)
    if np.any(truth_norm <= 1.0):
        raise RuntimeError("uncertain run has ||F_star|| <= 1 N; projection is ill-defined")
    unit_truth = truth / truth_norm[:, None]
    parallel = np.sum(estimate * unit_truth, axis=1)
    perpendicular = estimate - parallel[:, None] * unit_truth
    return truth_norm, parallel, np.linalg.norm(perpendicular, axis=1)


def plot_force_direction_decomposition(path, grid, campaign):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(len(UNCERTAIN_GROUPS), 1, figsize=(11, 8.2), sharex=True, sharey=True)
    for axis, group in zip(axes, UNCERTAIN_GROUPS):
        signals = [force_direction_signals(run) for run in campaign[group]]
        truth_mean, truth_std = mean_std([values[0] for values in signals])
        parallel_mean, parallel_std = mean_std([values[1] for values in signals])
        perpendicular_mean, perpendicular_std = mean_std([values[2] for values in signals])

        axis.plot(grid, truth_mean, color="black", linewidth=1.8, linestyle="--",
                  label=r"$\|F^\star\|$")
        axis.fill_between(grid, truth_mean - truth_std, truth_mean + truth_std,
                          color="black", alpha=0.07)
        axis.plot(grid, parallel_mean, color=GROUP_COLORS[group], linewidth=2.0,
                  label=r"$\hat{F}_{\parallel}=\hat{F}^{T}F^\star/\|F^\star\|$")
        axis.fill_between(grid, parallel_mean - parallel_std, parallel_mean + parallel_std,
                          color=GROUP_COLORS[group], alpha=0.15)
        axis.plot(grid, perpendicular_mean, color="#ff7f0e", linewidth=1.8,
                  label=r"$\|\hat{F}_{\perp}\|$")
        axis.fill_between(grid, perpendicular_mean - perpendicular_std,
                          perpendicular_mean + perpendicular_std, color="#ff7f0e", alpha=0.12)
        decorate(axis, "Force magnitude (N)")
        axis.set_title(GROUP_LABELS[group])
        axis.legend(loc="best")

    axes[-1].set_xlabel("MPC / simulation time (s)")
    figure.suptitle("Task C force-only direction and cross-axis leakage diagnostic\n"
                    "Force [Fx,Fy,Fz] only; torque is excluded to avoid mixing N and N m",
                    y=0.995)
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.955))
    figure.savefig(path, dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--records-root", required=True, type=Path,
                        help="Task C campaign root containing nominal/uncertain_* runs")
    parser.add_argument("--output", required=True, type=Path,
                        help="Output directory for additional figures")
    args = parser.parse_args()

    grid = np.arange(WINDOW_START, WINDOW_END + 0.5 * GRID_DT, GRID_DT)
    campaign = load_campaign(args.records_root, grid)
    args.output.mkdir(parents=True, exist_ok=True)

    paths = {
        "C11": args.output / "figure_C11_Fz_equivalent_payload_mass.png",
        "C12": args.output / "figure_C12_force_component_estimation_error.png",
        "C13": args.output / "figure_C13_force_direction_parallel_perpendicular.png",
    }
    plot_equivalent_payload_mass(paths["C11"], grid, campaign)
    plot_force_component_errors(paths["C12"], grid, campaign)
    plot_force_direction_decomposition(paths["C13"], grid, campaign)

    metadata = {
        "passed": True,
        "input": str(args.records_root.resolve()),
        "window_s": [WINDOW_START, WINDOW_END],
        "grid_dt_s": GRID_DT,
        "runs_per_group": len(RUNS),
        "gravity_m_s2": GRAVITY,
        "definitions": {
            "equivalent_payload_mass": "pi_Fz / g; diagnostic conversion only, not mass parameter identification",
            "component_error": "pi_hat_force - pi_star_force",
            "parallel_estimate": "dot(F_hat,F_star)/norm(F_star)",
            "perpendicular_leakage": "norm(F_hat - parallel_estimate*F_star/norm(F_star))",
            "projection_scope": "force-only [Fx,Fy,Fz]; torques excluded",
        },
        "figures": {key: str(value.resolve()) for key, value in paths.items()},
    }
    metadata_path = args.output / "additional_estimate_diagnostics.json"
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
