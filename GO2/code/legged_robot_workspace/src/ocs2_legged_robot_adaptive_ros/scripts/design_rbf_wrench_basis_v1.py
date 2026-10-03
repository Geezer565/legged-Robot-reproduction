#!/usr/bin/env python3
"""Offline-only Phase 2B RBF wrench basis design from recorded pilot bags."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag

SEED = 20260819
K = 25
N_INIT = 20
RHO = 1.5
TOL = 0.011
NAMES = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
WINDOWS = {"pre": (2.0, 3.5), "active": (5.0, 17.0), "post": (18.7, 20.0)}
CASES = {
    "CaseFxP": (Path("/home/han/ocs2_ws/records/rbf_residual_observer_20N/pilot.bag"), [20, 0, 0, 0, 0, 0]),
    "Case0": (Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/Case0/pilot.bag"), [0, 0, 0, 0, 0, 0]),
    "CaseFxN": (Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/CaseFxN/pilot.bag"), [-20, 0, 0, 0, 0, 0]),
    "CaseFyP": (Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/CaseFyP/pilot.bag"), [0, 20, 0, 0, 0, 0]),
    "CaseFzP": (Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/CaseFzP/pilot.bag"), [0, 0, 20, 0, 0, 0]),
    "CaseTxP": (Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/CaseTxP/pilot.bag"), [0, 0, 0, 2, 0, 0]),
    "CaseTyP": (Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/CaseTyP/pilot.bag"), [0, 0, 0, 0, 2, 0]),
    "CaseTzP": (Path("/home/han/ocs2_ws/records/rbf_basis_design/pilots/CaseTzP/pilot.bag"), [0, 0, 0, 0, 0, 2]),
}
OUT = Path("/home/han/ocs2_ws/records/rbf_basis_design")
CONFIG = Path("/home/han/ocs2_ws/src/ocs2_legged_robot_adaptive/config/rbf_wrench_basis_v1.info")
RESIDUAL_TOPIC = "/rbf_diagnostics/wrench_residual_filtered"
SIGMA_TOPIC = "/adaptive_update/sample"


def window_of(time):
    for name, (left, right) in WINDOWS.items():
        if left <= time <= right:
            return name
    return None


def wrench(message):
    q = message.wrench
    return [q.force.x, q.force.y, q.force.z, q.torque.x, q.torque.y, q.torque.z]


def nearest_sigma(times, values, query):
    i = np.searchsorted(times, query)
    candidates = [x for x in (i - 1, i) if 0 <= x < len(times)]
    best = min(candidates, key=lambda x: abs(times[x] - query))
    return values[best], abs(times[best] - query)


def read_case(case, path):
    sigma_t, sigma_v, residuals = [], [], []
    with rosbag.Bag(str(path)) as bag:
        for topic, message, _ in bag.read_messages(topics=[RESIDUAL_TOPIC, SIGMA_TOPIC]):
            if topic == SIGMA_TOPIC and len(message.data) >= 16:
                sigma_t.append(float(message.data[0]))
                sigma_v.append(list(message.data[10:16]))
            elif topic == RESIDUAL_TOPIC:
                time = message.header.stamp.to_sec()
                if window_of(time) is not None:
                    residuals.append((time, wrench(message)))
    sigma_t = np.asarray(sigma_t, dtype=float)
    sigma_v = np.asarray(sigma_v, dtype=float)
    order = np.argsort(sigma_t)
    sigma_t, sigma_v = sigma_t[order], sigma_v[order]
    aligned, errors = [], []
    for time, residual in residuals:
        sigma, error = nearest_sigma(sigma_t, sigma_v, time)
        if error <= TOL:
            aligned.append({"time": time, "residual": residual, "sigma": sigma.tolist(), "window": window_of(time)})
            errors.append(error)
    return aligned, {"raw_valid": len(residuals), "aligned": len(aligned),
                     "alignment_dropped": len(residuals) - len(aligned),
                     "alignment_error_median_s": float(np.median(errors)) if errors else None,
                     "alignment_error_max_s": float(np.max(errors)) if errors else None,
                     "sigma_topic": SIGMA_TOPIC, "residual_topic": RESIDUAL_TOPIC}


def positive_floor(q, eps):
    positive = q[q > 0]
    return max(0.05 * float(np.median(positive)) if len(positive) else 0.0, eps)


def summary(values):
    values = np.asarray(values, dtype=float)
    return {"mean": float(np.mean(values)), "std": float(np.std(values)), "min": float(np.min(values)),
            "p01": float(np.percentile(values, 1)), "p05": float(np.percentile(values, 5)),
            "p50": float(np.percentile(values, 50)), "p95": float(np.percentile(values, 95)),
            "p99": float(np.percentile(values, 99)), "max": float(np.max(values))}


def kmeans_plus_plus(x, rng):
    centers = [x[rng.integers(len(x))]]
    closest = np.sum((x - centers[0]) ** 2, axis=1)
    for _ in range(1, K):
        total = closest.sum()
        index = rng.integers(len(x)) if total <= 0 else rng.choice(len(x), p=closest / total)
        centers.append(x[index])
        closest = np.minimum(closest, np.sum((x - centers[-1]) ** 2, axis=1))
    return np.asarray(centers)


def fit_kmeans(x):
    best_centers, best_inertia = None, np.inf
    for init in range(N_INIT):
        rng = np.random.default_rng(SEED + init)
        centers = kmeans_plus_plus(x, rng)
        for _ in range(300):
            labels = np.argmin(((x[:, None, :] - centers[None, :, :]) ** 2).sum(axis=2), axis=1)
            new = np.array([x[labels == j].mean(axis=0) if np.any(labels == j) else centers[j] for j in range(K)])
            if np.max(np.abs(new - centers)) <= 1e-10:
                centers = new
                break
            centers = new
        inertia = float(np.sum(np.min(((x[:, None, :] - centers[None, :, :]) ** 2).sum(axis=2), axis=1)))
        if inertia < best_inertia:
            best_centers, best_inertia = centers.copy(), inertia
    return best_centers, best_inertia


def coverage(x, centers, widths):
    distances2 = ((x[:, None, :] - centers[None, :, :]) ** 2).sum(axis=2)
    phi = np.exp(-distances2 / (2.0 * widths[None, :] ** 2))
    return phi, np.max(phi, axis=1), np.argmax(phi, axis=1)


def coverage_stats(phi_max):
    return {"mean": float(np.mean(phi_max)), "median": float(np.median(phi_max)),
            "p05": float(np.percentile(phi_max, 5)), "p01": float(np.percentile(phi_max, 1)), "min": float(np.min(phi_max)),
            "ratio_lt_0_01": float(np.mean(phi_max < .01)), "ratio_lt_0_05": float(np.mean(phi_max < .05)),
            "ratio_lt_0_10": float(np.mean(phi_max < .10)), "ratio_gt_0_95": float(np.mean(phi_max > .95))}


def usage(labels):
    count = np.bincount(labels, minlength=K)
    ratio = count / count.sum()
    return [{"center": int(i), "winner_count": int(count[i]), "winner_ratio": float(ratio[i]),
             "status": "unused" if count[i] == 0 else "poorly_used" if ratio[i] < .005 else "dominant" if ratio[i] > .15 else "normal"}
            for i in range(K)]


def write_config(residual_scale, sigma_scale, centers, widths):
    lines = ["; Phase2B v1: derived from balanced residual pilot dataset.",
             "; Centers fitted only on the design split; holdout not used for fitting.",
             "; Ground-truth wrench is not a network feature or label.", "rbf_wrench_basis {", "  gaussianNodeCount 25",
             "  residualScale { " + " ".join("(%d,0) %.17g" % (i, v) for i, v in enumerate(residual_scale)) + " }",
             "  sigmaScale { " + " ".join("(%d,0) %.17g" % (i, v) for i, v in enumerate(sigma_scale)) + " }", "  beta 0.7",
             "  centers {"]
    lines += ["    " + " ".join("(%d,%d) %.17g" % (j, i, value) for i, value in enumerate(row)) for j, row in enumerate(centers)]
    lines += ["  }", "  widths { " + " ".join("(%d,0) %.17g" % (i, value) for i, value in enumerate(widths)) + " }", "}", ""]
    CONFIG.write_text("\n".join(lines))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(SEED)
    per_case, metadata = {}, {}
    for case, (path, truth) in CASES.items():
        samples, meta = read_case(case, path)
        per_case[case], metadata[case] = samples, {**meta, "bag_path": str(path), "true_wrench_metadata": truth}
    # A common case contribution must also be feasible in every prescribed
    # window without replacement.  Case0's equal thirds are the limiting
    # stratum for these recorded bags (66 post-window samples => 198 total).
    feasible = []
    for case, rows in per_case.items():
        counts = {name: sum(row["window"] == name for row in rows) for name in WINDOWS}
        if case == "Case0":
            feasible.append(3 * min(counts.values()))
        else:
            feasible.append(min(int(counts["pre"] / .20), int(counts["active"] / .60), int(counts["post"] / .20)))
    target = min(min(len(rows) for rows in per_case.values()), min(feasible))
    all_rows = []
    for case, rows in per_case.items():
        buckets = {name: [row for row in rows if row["window"] == name] for name in WINDOWS}
        if case == "Case0":
            allocation = {"pre": target // 3, "active": target // 3, "post": target - 2 * (target // 3)}
        else:
            allocation = {"active": int(round(.60 * target)), "pre": int(round(.20 * target))}
            allocation["post"] = target - allocation["active"] - allocation["pre"]
        selected = []
        for window in WINDOWS:
            if len(buckets[window]) < allocation[window]:
                raise RuntimeError("insufficient %s samples for %s" % (window, case))
            selected.extend([buckets[window][i] for i in rng.choice(len(buckets[window]), allocation[window], replace=False)])
        metadata[case]["balanced"] = len(selected)
        metadata[case]["window_raw_aligned"] = {w: len(buckets[w]) for w in WINDOWS}
        metadata[case]["window_balanced"] = allocation
        for row in selected:
            row["case"] = case
        all_rows.extend(selected)
    design, holdout = [], []
    for case in CASES:
        for window in WINDOWS:
            rows = [r for r in all_rows if r["case"] == case and r["window"] == window]
            permutation = rng.permutation(len(rows)); cut = int(np.floor(.70 * len(rows)))
            design.extend(rows[i] for i in permutation[:cut]); holdout.extend(rows[i] for i in permutation[cut:])
    def arrays(rows):
        return (np.asarray([r["residual"] for r in rows]), np.asarray([r["sigma"] for r in rows]),
                np.asarray([r["case"] for r in rows]), np.asarray([r["time"] for r in rows]), np.asarray([r["window"] for r in rows]))
    dr, ds, dc, dt, dw = arrays(design); hr, hs, hc, ht, hw = arrays(holdout)
    q95r = np.percentile(np.abs(dr), 95, axis=0); force_floor = positive_floor(q95r[:3], 1e-6); torque_floor = positive_floor(q95r[3:], 1e-6)
    residual_scale = np.maximum(q95r, [force_floor] * 3 + [torque_floor] * 3)
    q95s = np.percentile(np.abs(ds), 95, axis=0); linear_floor = positive_floor(q95s[:3], 1e-8); angular_floor = positive_floor(q95s[3:], 1e-8)
    sigma_scale = np.maximum(q95s, [linear_floor] * 3 + [angular_floor] * 3)
    dchi, hchi = dr / residual_scale, hr / residual_scale
    centers, inertia = fit_kmeans(dchi)
    pairwise = np.sqrt(((centers[:, None, :] - centers[None, :, :]) ** 2).sum(axis=2)); np.fill_diagonal(pairwise, np.inf)
    local = np.sort(pairwise, axis=1)[:, :3].mean(axis=1)
    if np.any(local <= np.finfo(float).eps):
        raise RuntimeError("degenerate center encountered")
    widths = RHO * local
    dphi, dmax, dwinner = coverage(dchi, centers, widths); hphi, hmax, hwinner = coverage(hchi, centers, widths)
    np.savetxt(OUT / "rbf_centers_v1.csv", centers, delimiter=",", header=",".join(NAMES), comments="")
    np.save(OUT / "rbf_centers_v1.npy", centers)
    np.savetxt(OUT / "rbf_widths_v1.csv", np.column_stack((np.arange(K), widths)), delimiter=",", header="center_index,width", comments="")
    np.savez_compressed(OUT / "rbf_basis_dataset_v1.npz", design_chi=dchi, holdout_chi=hchi, design_residual=dr, holdout_residual=hr,
                        design_sigma=ds, holdout_sigma=hs, design_case_id=dc, holdout_case_id=hc, design_time=dt, holdout_time=ht,
                        design_window_id=dw, holdout_window_id=hw)
    write_config(residual_scale, sigma_scale, centers, widths)
    chi_stats = {NAMES[i]: {**summary(dchi[:, i]), "ratio_abs_gt_1": float(np.mean(abs(dchi[:, i]) > 1)), "ratio_abs_gt_2": float(np.mean(abs(dchi[:, i]) > 2)), "ratio_abs_gt_3": float(np.mean(abs(dchi[:, i]) > 3))} for i in range(6)}
    per_case_coverage = {}
    for case in CASES:
        values = hmax[hc == case]
        per_case_coverage[case] = {"holdout_sample_count": int(len(values)), "mean_phiMax": float(values.mean()), "median_phiMax": float(np.median(values)), "p05_phiMax": float(np.percentile(values, 5)), "p01_phiMax": float(np.percentile(values, 1)), "ratio_phiMax_lt_0_05": float(np.mean(values < .05)), "ratio_phiMax_lt_0_10": float(np.mean(values < .10))}
    active = {"holdout_gt_0_1": {k: float(np.percentile((hphi > .1).sum(1), p)) for k, p in (("median", 50), ("p05", 5), ("p95", 95))},
              "holdout_gt_0_5": {k: float(np.percentile((hphi > .5).sum(1), p)) for k, p in (("median", 50), ("p05", 5), ("p95", 95))}}
    report = {"phase": "Phase2B_v1_offline_basis_design", "seed": SEED, "k": K, "n_init": N_INIT, "rho": RHO, "alignment_tolerance_s": TOL,
              "topic_schema": {"residual": {"topic": RESIDUAL_TOPIC, "type": "geometry_msgs/WrenchStamped", "time": "header.stamp = OCS2 observation/simulation time", "order": list(NAMES)}, "sigma": {"topic": SIGMA_TOPIC, "type": "std_msgs/Float64MultiArray", "time_field": 0, "sigma_fields": "data[10:16]", "order": list(NAMES)}},
              "case_metadata": metadata, "balanced_target_per_case": target, "design_count": len(design), "holdout_count": len(holdout),
              "residual_scale": {"q95": q95r.tolist(), "force_floor": force_floor, "torque_floor": torque_floor, "scale": residual_scale.tolist()},
              "sigma_scale": {"q95": q95s.tolist(), "linear_floor": linear_floor, "angular_floor": angular_floor, "scale": sigma_scale.tolist()},
              "normalized_chi_design_statistics": chi_stats, "centers": centers.tolist(), "kmeans_inertia": inertia, "widths": widths.tolist(), "width_statistics": summary(widths),
              "design_coverage": coverage_stats(dmax), "holdout_coverage": coverage_stats(hmax), "per_case_holdout_coverage": per_case_coverage,
              "center_usage": {"design": usage(dwinner), "holdout": usage(hwinner)}, "active_node_count": active,
              "ground_truth_note": "Recorded only as case metadata; not used as an RBF feature, label, scale input, center input, width input, or coverage input."}
    (OUT / "basis_design_report_v1.json").write_text(json.dumps(report, indent=2) + "\n")
    fig, axes = plt.subplots(2, 3, figsize=(13, 7));
    for i, ax in enumerate(axes.ravel()): ax.hist(dr[:, i], bins=60); ax.set_title(NAMES[i]); ax.set_xlabel("filtered residual")
    fig.tight_layout(); fig.savefig(OUT / "B1_residual_axis_histograms.png", dpi=180); plt.close(fig)
    fig, axes = plt.subplots(2, 3, figsize=(13, 7));
    for i, ax in enumerate(axes.ravel()): ax.hist(dchi[:, i], bins=60); ax.set_title(NAMES[i]); ax.set_xlabel("normalized residual chi")
    fig.tight_layout(); fig.savefig(OUT / "B2_normalized_residual_histograms.png", dpi=180); plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 4)); ax.hist(dmax, bins=50, alpha=.65, label="design"); ax.hist(hmax, bins=50, alpha=.65, label="holdout"); ax.set_xlabel("phiMax"); ax.legend(); fig.tight_layout(); fig.savefig(OUT / "B3_basis_max_activation_histogram.png", dpi=180); plt.close(fig)
    fig, ax = plt.subplots(figsize=(11, 4)); x = np.arange(K); ax.bar(x-.2, [v["winner_ratio"] for v in usage(dwinner)], .4, label="design"); ax.bar(x+.2, [v["winner_ratio"] for v in usage(hwinner)], .4, label="holdout"); ax.axhline(.005, color="orange", ls="--"); ax.axhline(.15, color="red", ls="--"); ax.set_xlabel("center index"); ax.set_ylabel("winner ratio"); ax.legend(); fig.tight_layout(); fig.savefig(OUT / "B4_center_usage.png", dpi=180); plt.close(fig)
    mean = dchi.mean(0); _, _, vt = np.linalg.svd(dchi - mean, full_matrices=False); transform = vt[:2].T
    fig, ax = plt.subplots(figsize=(7, 6)); ax.scatter(*((dchi-mean)@transform).T, s=4, alpha=.2, label="design"); ax.scatter(*((hchi-mean)@transform).T, s=5, alpha=.25, label="holdout"); ax.scatter(*((centers-mean)@transform).T, s=50, marker="x", label="centers"); ax.set_xlabel("PC1"); ax.set_ylabel("PC2"); ax.legend(); fig.tight_layout(); fig.savefig(OUT / "B5_rbf_centers_pca.png", dpi=180); plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 4)); names=list(CASES); med=[per_case_coverage[c]["median_phiMax"] for c in names]; p05=[per_case_coverage[c]["p05_phiMax"] for c in names]; x=np.arange(len(names)); ax.bar(x-.2, med,.4,label="median"); ax.bar(x+.2,p05,.4,label="P05"); ax.set_xticks(x,names); ax.set_ylim(0,1.05); ax.set_ylabel("holdout phiMax"); ax.legend(); fig.tight_layout(); fig.savefig(OUT / "B6_case_coverage.png", dpi=180); plt.close(fig)


if __name__ == "__main__":
    main()
