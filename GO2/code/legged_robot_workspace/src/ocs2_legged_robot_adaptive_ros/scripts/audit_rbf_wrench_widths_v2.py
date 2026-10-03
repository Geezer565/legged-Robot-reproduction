#!/usr/bin/env python3
"""Offline Phase2B.1 width-locality audit; centers and scales are read-only."""
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path("/home/han/ocs2_ws/records/rbf_basis_design")
CONFIG = Path("/home/han/ocs2_ws/src/ocs2_legged_robot_adaptive/config/rbf_wrench_basis_v2.info")
RHO_VALUES = (0.60, 0.70, 0.80, 0.90, 1.00, 1.20, 1.50)
CASE_ORDER = ("Case0", "CaseFxP", "CaseFxN", "CaseFyP", "CaseFzP", "CaseTxP", "CaseTyP", "CaseTzP")


def stats(x):
    x = np.asarray(x, float)
    return {"mean": float(x.mean()), "median": float(np.median(x)), "p05": float(np.percentile(x, 5)),
            "p01": float(np.percentile(x, 1)), "min": float(x.min()), "max": float(x.max()),
            "ratio_lt_0_01": float(np.mean(x < .01)), "ratio_lt_0_05": float(np.mean(x < .05)),
            "ratio_lt_0_10": float(np.mean(x < .10)), "ratio_lt_0_30": float(np.mean(x < .30)),
            "ratio_gt_0_95": float(np.mean(x > .95))}


def count_stats(x):
    x = np.asarray(x, float)
    return {"mean": float(x.mean()), "median": float(np.median(x)), "p05": float(np.percentile(x, 5)),
            "p95": float(np.percentile(x, 95)), "min": int(x.min()), "max": int(x.max())}


def activation(x, centers, widths):
    d2 = ((x[:, None, :] - centers[None, :, :]) ** 2).sum(axis=2)
    return np.exp(-d2 / (2.0 * widths[None, :] ** 2))


def correlation(phi):
    col_std = phi.std(axis=0)
    corr = np.corrcoef(phi, rowvar=False)
    vals = np.abs(corr[np.triu_indices_from(corr, k=1)])
    gram = phi.T @ phi / len(phi)
    eig = np.linalg.eigvalsh(gram)
    positive = eig[eig > np.finfo(float).eps * max(1.0, eig.max())]
    condition = float(eig.max() / eig.min()) if len(positive) == len(eig) else None
    return {"activation_column_std": col_std.tolist(),
            "absolute_off_diagonal_correlation": {"median": float(np.median(vals)), "p95": float(np.percentile(vals, 95)), "max": float(vals.max())},
            "gram_eigenvalues": eig.tolist(), "gram_condition_number": condition,
            "gram_condition_note": "computed from symmetric eigenspectrum" if condition is not None else "numerically rank-deficient; condition number not reported"}


def passed(hold, cases, locality):
    failures = []
    if hold["p05"] < .60: failures.append("A overall holdout P05 < 0.60")
    bad = [case for case, result in cases.items() if result["p05"] < .50]
    if bad: failures.append("B per-case P05 < 0.50: " + ",".join(bad))
    if hold["ratio_lt_0_10"] > .01: failures.append("C overall ratio(phiMax<0.10) > 1%")
    if not 4 <= locality["active01"]["median"] <= 12: failures.append("D median active(phi>0.1) outside [4,12]")
    if not 1 <= locality["active05"]["median"] <= 6: failures.append("E median active(phi>0.5) outside [1,6]")
    return not failures, failures


def write_config(residual_scale, sigma_scale, centers, widths, rho):
    lines = ["; Phase2B.1 width-locality calibration.", "; Centers and scales unchanged from v1; centers were fitted only on the v1 design split.",
             "; Holdout was evaluation-only. Ground-truth wrench is not a network feature or label.", "rbf_wrench_basis {", "  gaussianNodeCount 25",
             "  residualScale { " + " ".join("(%d,0) %.17g" % (i, v) for i, v in enumerate(residual_scale)) + " }",
             "  sigmaScale { " + " ".join("(%d,0) %.17g" % (i, v) for i, v in enumerate(sigma_scale)) + " }", "  beta 0.7",
             "  widthRho %.17g" % rho, "  centers {"]
    lines += ["    " + " ".join("(%d,%d) %.17g" % (j, i, value) for i, value in enumerate(row)) for j, row in enumerate(centers)]
    lines += ["  }", "  widths { " + " ".join("(%d,0) %.17g" % (i, value) for i, value in enumerate(widths)) + " }", "}", ""]
    CONFIG.write_text("\n".join(lines))


def main():
    prior = json.loads((ROOT / "basis_design_report_v1.json").read_text())
    data = np.load(ROOT / "rbf_basis_dataset_v1.npz", allow_pickle=False)
    design, holdout = data["design_chi"], data["holdout_chi"]
    hold_case = data["holdout_case_id"].astype(str)
    centers = np.load(ROOT / "rbf_centers_v1.npy")
    pair = np.sqrt(((centers[:, None, :] - centers[None, :, :]) ** 2).sum(axis=2)); np.fill_diagonal(pair, np.inf)
    local = np.sort(pair, axis=1)[:, :3].mean(axis=1)
    results = {}
    for rho in RHO_VALUES:
        widths = rho * local
        dp, dm = activation(design, centers, widths), None
        hp = activation(holdout, centers, widths)
        dmax, hmax = dp.max(axis=1), hp.max(axis=1)
        locality = {"active01": count_stats((hp > .1).sum(axis=1)), "active05": count_stats((hp > .5).sum(axis=1))}
        per_case = {case: stats(hmax[hold_case == case]) for case in CASE_ORDER}
        ok, failures = passed(stats(hmax), per_case, locality)
        results[str(rho)] = {"rho": rho, "widths": widths.tolist(), "width_statistics": {"min": float(widths.min()), "median": float(np.median(widths)), "max": float(widths.max())},
                             "design_coverage": stats(dmax), "holdout_coverage": stats(hmax), "holdout_locality": locality,
                             "per_case_holdout_coverage": per_case, "design_gaussian_correlation": correlation(dp),
                             "selection_pass": ok, "selection_failures": failures}
    selected = next((rho for rho in RHO_VALUES if results[str(rho)]["selection_pass"]), None)
    audit = {"phase": "Phase2B_1_width_locality_calibration", "frozen_inputs": {"k": 25, "centers": str(ROOT / "rbf_centers_v1.npy"), "dataset": str(ROOT / "rbf_basis_dataset_v1.npz"), "residual_scale": prior["residual_scale"]["scale"], "sigma_scale": prior["sigma_scale"]["scale"]},
             "local_distance": local.tolist(), "candidate_rho": list(RHO_VALUES), "selection_rule": {"A_holdout_p05_min": .60, "B_each_case_p05_min": .50, "C_holdout_ratio_lt_0_10_max": .01, "D_active01_median_range": [4, 12], "E_active05_median_range": [1, 6], "tie_break": "smallest rho"},
             "results": results, "selected_rho": selected, "final_width_statistics": results[str(selected)]["width_statistics"] if selected is not None else None}
    (ROOT / "basis_width_audit_v2.json").write_text(json.dumps(audit, indent=2) + "\n")
    rhos = list(RHO_VALUES)
    fig, ax = plt.subplots(figsize=(7, 4)); ax.plot(rhos, [results[str(r)]["holdout_coverage"]["p05"] for r in rhos], "o-", label="P05"); ax.plot(rhos, [results[str(r)]["holdout_coverage"]["median"] for r in rhos], "o-", label="median"); ax.axhline(.60, color="0.4", ls="--"); ax.set_xlabel("rho"); ax.set_ylabel("holdout phiMax"); ax.legend(); fig.tight_layout(); fig.savefig(ROOT / "B7_rho_vs_coverage.png", dpi=180); plt.close(fig)
    fig, ax = plt.subplots(figsize=(7, 4)); ax.plot(rhos, [results[str(r)]["holdout_locality"]["active01"]["median"] for r in rhos], "o-", label="phi > 0.1"); ax.plot(rhos, [results[str(r)]["holdout_locality"]["active05"]["median"] for r in rhos], "o-", label="phi > 0.5"); ax.axhspan(4,12,color="green",alpha=.1); ax.axhspan(1,6,color="orange",alpha=.1); ax.set_xlabel("rho"); ax.set_ylabel("median active nodes"); ax.legend(); fig.tight_layout(); fig.savefig(ROOT / "B8_rho_vs_active_nodes.png", dpi=180); plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 5));
    for case in CASE_ORDER: ax.plot(rhos, [results[str(r)]["per_case_holdout_coverage"][case]["p05"] for r in rhos], "o-", label=case)
    ax.axhline(.50,color="0.4",ls="--"); ax.set_xlabel("rho"); ax.set_ylabel("holdout P05 phiMax"); ax.legend(ncol=2); fig.tight_layout(); fig.savefig(ROOT / "B9_rho_per_case_p05.png", dpi=180); plt.close(fig)
    fig, ax = plt.subplots(figsize=(7, 4)); ax.plot(rhos, [results[str(r)]["design_gaussian_correlation"]["absolute_off_diagonal_correlation"]["p95"] for r in rhos], "o-"); ax.set_xlabel("rho"); ax.set_ylabel("P95 |off-diagonal correlation|"); fig.tight_layout(); fig.savefig(ROOT / "B10_rho_basis_correlation.png", dpi=180); plt.close(fig)
    if selected is not None:
        widths = selected * local
        np.savetxt(ROOT / "rbf_widths_v2.csv", np.column_stack((np.arange(25), widths)), delimiter=",", header="center_index,width", comments="")
        write_config(prior["residual_scale"]["scale"], prior["sigma_scale"]["scale"], centers, widths, selected)


if __name__ == "__main__":
    main()
