#!/usr/bin/env python3
"""Build a Stage B3 completeness/fairness manifest without performance analysis."""

import argparse
import hashlib
import json
import os


MODES = ("nominal", "estimate_only", "full_adaptive")
EXPECTED_FLAGS = {
    "nominal": (False, False),
    "estimate_only": (True, False),
    "full_adaptive": (True, True),
}
REQUIRED_FILES = (
    "task_b.bag",
    "task_b_smoke.csv",
    "task_b_smoke_summary.json",
    "task_b_smoke.png",
    "effective_config.json",
    "experiment_schedule.json",
    "source_files.sha256",
    "terminal.log",
    "roscore.log",
    "roslaunch.log",
    "rosbag.log",
)


def load_json(path):
    with open(path) as stream:
        return json.load(stream)


def canonical_hash(value):
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def load_source_hashes(path):
    result = {}
    with open(path) as stream:
        for line in stream:
            digest, source = line.rstrip("\n").split(maxsplit=1)
            result[source] = digest
    return result


def runtime_manifest_hash(source_hashes):
    runtime_files = {
        "task_b_experiment.launch",
        "aclf_observation_monitor.py",
        "aclf_small_step_target.py",
    }
    rows = sorted(
        (path, digest)
        for path, digest in source_hashes.items()
        if os.path.splitext(path)[1] in (".cpp", ".h", ".hpp", ".tpp")
        or os.path.basename(path) in runtime_files
    )
    return canonical_hash(rows)


def main():
    parser = argparse.ArgumentParser(
        description="Summarize nine formal Task B acquisitions without computing tracking metrics"
    )
    parser.add_argument("records_directory")
    parser.add_argument("--runs", nargs="+", default=("run_01", "run_02", "run_03"))
    parser.add_argument("--output", default=None)
    arguments = parser.parse_args()

    records = os.path.abspath(arguments.records_directory)
    output = arguments.output or os.path.join(records, "stage_b3_acquisition_manifest.json")
    runs = []
    neutral_config_hashes = set()
    schedule_hashes = set()
    runtime_hashes = set()
    full_source_manifest_hashes = set()
    fixed_file_hashes = {}
    csv_hashes_by_mode = {mode: set() for mode in MODES}

    for mode in MODES:
        expected_update, expected_dynamics = EXPECTED_FLAGS[mode]
        for run_id in arguments.runs:
            directory = os.path.join(records, mode, run_id)
            summary = load_json(os.path.join(directory, "task_b_smoke_summary.json"))
            config = load_json(os.path.join(directory, "effective_config.json"))
            schedule = load_json(os.path.join(directory, "experiment_schedule.json"))
            sources = load_source_hashes(os.path.join(directory, "source_files.sha256"))
            with open(os.path.join(directory, "task_b_smoke.csv"), "rb") as stream:
                csv_hashes_by_mode[mode].add(hashlib.sha256(stream.read()).hexdigest())

            neutral_config_hashes.add(config["mode_neutral_config_sha256"])
            neutral_schedule = dict(schedule)
            neutral_schedule.pop("mode", None)
            schedule_hashes.add(canonical_hash(neutral_schedule))
            runtime_hash = runtime_manifest_hash(sources)
            runtime_hashes.add(runtime_hash)
            with open(os.path.join(directory, "source_files.sha256"), "rb") as stream:
                full_source_manifest_hashes.add(hashlib.sha256(stream.read()).hexdigest())
            for label, item in config["fixed_files"].items():
                fixed_file_hashes.setdefault(label, set()).add(item["sha256"])

            missing = []
            empty = []
            for filename in REQUIRED_FILES:
                path = os.path.join(directory, filename)
                if not os.path.isfile(path):
                    missing.append(filename)
                elif os.path.getsize(path) == 0:
                    empty.append(filename)
            flags = (
                config["enableParameterUpdate"],
                config["enableAdaptiveDynamics"],
            )
            runs.append({
                "mode": mode,
                "run": run_id,
                "directory": directory,
                "validator_passed": summary["passed"],
                "effective_flags": list(flags),
                "expected_flags": [expected_update, expected_dynamics],
                "flags_ok": flags == (expected_update, expected_dynamics),
                "missing_files": missing,
                "empty_files": empty,
                "required_files_ok": not missing and not empty,
                "no_missing_topics": not summary["missing_required_topics"],
                "all_numeric_values_finite": summary["all_recorded_numeric_values_finite"],
                "target_ok": summary["target"]["ok"],
                "disturbance_ok": summary["disturbance"]["ok"],
                "snapshot_ok": summary["snapshot"]["ok"],
                "equation10_ok": summary["equation10"]["ok"],
                "disturbance_pair_violation_count": summary["pairing"].get(
                    "disturbance_pair_violation_count", 0
                ),
                "initial_t0_pair_exception_count": summary["pairing"].get(
                    "initial_t0_exception_count", 0
                ),
                "sample_count": summary["snapshot"]["sample_count"],
                "observation_start_s": summary["tracking"]["observation_start_s"],
                "observation_end_s": summary["tracking"]["observation_end_s"],
                "peak_pi_hat_norm": summary["equation10"]["peak_pi_hat_norm"],
                "peak_candidate_correction_norm": summary["equation10"][
                    "peak_candidate_correction_norm"
                ],
                "peak_applied_correction_norm": summary["equation10"][
                    "peak_applied_correction_norm"
                ],
                "mode_neutral_config_sha256": config["mode_neutral_config_sha256"],
                "runtime_source_manifest_sha256": runtime_hash,
            })

    checks = {
        "run_count_is_nine": len(runs) == 9,
        "all_single_run_validators_passed": all(row["validator_passed"] for row in runs),
        "all_required_files_present_and_nonempty": all(row["required_files_ok"] for row in runs),
        "all_effective_flags_match_mode": all(row["flags_ok"] for row in runs),
        "all_required_topics_present": all(row["no_missing_topics"] for row in runs),
        "all_numeric_values_finite": all(row["all_numeric_values_finite"] for row in runs),
        "all_target_checks_passed": all(row["target_ok"] for row in runs),
        "all_disturbance_checks_passed": all(row["disturbance_ok"] for row in runs),
        "all_snapshot_checks_passed": all(row["snapshot_ok"] for row in runs),
        "all_equation10_path_checks_passed": all(row["equation10_ok"] for row in runs),
        "no_strict_disturbance_pairing_violations": all(
            row["disturbance_pair_violation_count"] == 0 for row in runs
        ),
        "one_mode_neutral_config_hash": len(neutral_config_hashes) == 1,
        "one_mode_neutral_schedule_hash": len(schedule_hashes) == 1,
        "one_runtime_source_manifest_hash": len(runtime_hashes) == 1,
        "fixed_external_file_hashes_identical": all(
            len(hashes) == 1 for hashes in fixed_file_hashes.values()
        ),
    }
    manifest = {
        "stage": "B3 acquisition completeness and fairness",
        "performance_analysis_performed": False,
        "passed": all(checks.values()),
        "checks": checks,
        "mode_neutral_config_hashes": sorted(neutral_config_hashes),
        "mode_neutral_schedule_hashes": sorted(schedule_hashes),
        "runtime_source_manifest_hashes": sorted(runtime_hashes),
        "full_source_manifest_hash_count": len(full_source_manifest_hashes),
        "full_source_manifest_hash_note": (
            "May differ because acquisition/analyzer shell and Python scripts were hardened during B3; "
            "the C++ control/disturbance sources, runtime launch/target/monitor scripts, neutralized "
            "effective configuration, fixed external files, and neutralized schedule are checked separately."
        ),
        "fixed_external_file_hashes": {
            label: sorted(hashes) for label, hashes in sorted(fixed_file_hashes.items())
        },
        "repeatability": {
            mode: {
                "exact_csv_identical": len(csv_hashes_by_mode[mode]) == 1,
                "distinct_csv_hash_count": len(csv_hashes_by_mode[mode]),
                "note": "Exact identity is reported, not required for the B3 acquisition gate.",
            }
            for mode in MODES
        },
        "runs": runs,
    }
    with open(output, "w") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(manifest, indent=2, sort_keys=True))
    if not manifest["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
