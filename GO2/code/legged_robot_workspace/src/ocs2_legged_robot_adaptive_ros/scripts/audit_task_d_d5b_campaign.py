#!/usr/bin/env python3
"""Audit D5B run integrity, source identity, roles, and acquisition order."""

import argparse
import hashlib
import json
from pathlib import Path


GROUPS = ("nominal_stance_hold", "nominal_stance_target",
          "nominal_walk_target", "uncertain_walk_target")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def normalized(config):
    ignored = {
        "group", "source_task_d_mode", "scheduler_condition",
        "truePlantPayloadInertialEnabled", "expected_target_published",
        "expected_walking", "run_index", "acquisition_sequence", "run_scope",
        "acquisition_end_mpc_time_s", "analysis_window_end_mpc_time_s",
        "d5b_protocol_sha256", "stage", "schema_version",
    }
    return {key: value for key, value in config.items() if key not in ignored}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--runs", type=int, required=True)
    parser.add_argument("--scope", choices=("smoke", "formal"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    protocol_path = root / "protocol" / "stage_d5b_protocol.json"
    protocol = json.loads(protocol_path.read_text())
    base = root / args.scope
    configs, summaries, source_hashes, sequences = {}, {}, {}, []
    artifact_hashes = {}
    for index in range(1, args.runs + 1):
        run_name = "run_{:02d}".format(index)
        for group in GROUPS:
            run = base / group / run_name
            key = group + "/" + run_name
            configs[key] = json.loads((run / "effective_config.json").read_text())
            summaries[key] = json.loads((run / "task_d_d5b_run_summary.json").read_text())
            source_hashes[key] = digest(run / "source_files.sha256")
            sequences.append(configs[key]["acquisition_sequence"])
            artifact_hashes[key] = {
                name: digest(run / name) for name in (
                    "task_d_d5b.bag", "task_d_d5b_run.csv",
                    "task_d_d5b_run_summary.json", "task_d_d5b_overview.png",
                    "effective_config.json", "effective_adaptive_settings.info")}
    roles_ok = True
    for key, config in configs.items():
        group = key.split("/")[0]
        role = protocol["groups"][group]
        roles_ok = roles_ok and bool(
            config["source_task_d_mode"] == role["source_mode"]
            and config["scheduler_condition"] == role["scheduler_condition"]
            and config["truePlantPayloadInertialEnabled"] == role["payload"]
            and config["expected_target_published"] == role["target"]
            and config["expected_walking"] == role["walking"]
            and config["enableParameterUpdate"] is True
            and config["enableAdaptiveDynamics"] is True
            and config["enableClfConstraint"] is True
            and config["run_scope"] == args.scope)
    all_pass = all(value.get("pass") for value in summaries.values())
    formal_ok = (args.scope != "formal" or all(
        value.get("formal_eligibility") for value in summaries.values()))
    source_equal = len(set(source_hashes.values())) == 1
    normalized_equal = len({json.dumps(normalized(value), sort_keys=True)
                            for value in configs.values()}) == 1
    sequence_ok = sequences == list(range(1, len(sequences) + 1))
    protocol_ok = bool(protocol.get("passed")) and all(
        config.get("d5b_protocol_sha256") == digest(protocol_path)
        for config in configs.values())
    passed = bool(roles_ok and all_pass and formal_ok and source_equal
                  and normalized_equal and sequence_ok and protocol_ok)
    result = {
        "schema_version": 1, "stage": "D5B_{}_campaign_audit".format(args.scope),
        "passed": passed, "scope": args.scope, "groups": list(GROUPS),
        "runs_per_group": args.runs, "total_runs": len(summaries),
        "checks": {
            "roles": roles_ok, "all_run_validations": all_pass,
            "formal_eligibility": formal_ok, "source_snapshots_identical": source_equal,
            "normalized_configs_identical": normalized_equal,
            "acquisition_sequence": sequence_ok, "protocol": protocol_ok,
        },
        "source_snapshot_sha256": source_hashes,
        "summaries": summaries, "artifact_sha256": artifact_hashes,
        "interpretation_boundary": protocol["interpretation_boundary"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
