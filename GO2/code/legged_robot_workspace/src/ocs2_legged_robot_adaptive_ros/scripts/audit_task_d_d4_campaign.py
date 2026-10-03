#!/usr/bin/env python3
"""Cross-audit the D4 full-horizon gate or completed three-round campaign."""

import argparse
import hashlib
import json
from pathlib import Path


GROUPS = ("nominal", "uncertain_off", "uncertain_adaptive")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def normalized_config(config):
    value = json.loads(json.dumps(config))
    for key in (
            "group", "adaptive_config_sha256", "round_index",
            "acquisition_sequence"):
        value.pop(key, None)
    value["enableParameterUpdate"] = "<D4_MODE>"
    value["enableAdaptiveDynamics"] = "<D4_MODE>"
    value["truePlantPayloadInertialEnabled"] = "<D4_PLANT>"
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--runs", type=int, choices=(1, 3), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    protocol = json.loads((root / "protocol" / "stage_d4_protocol.json").read_text())
    run_names = tuple("run_{:02d}".format(index) for index in range(1, args.runs + 1))
    summaries, configs, source_hashes, artifact_hashes = {}, {}, {}, {}
    sequence_values = []
    for round_index, run_name in enumerate(run_names, start=1):
        for group in GROUPS:
            key = group + "/" + run_name
            run = root / group / run_name
            summaries[key] = json.loads((run / "task_d_d4_run_summary.json").read_text())
            configs[key] = json.loads((run / "effective_config.json").read_text())
            source_hashes[key] = digest(run / "source_files.sha256")
            sequence_values.append(configs[key]["acquisition_sequence"])
            artifact_hashes[key] = {
                name: digest(run / name) for name in (
                    "task_d_d4.bag", "task_d_d4_run.csv",
                    "task_d_d4_run_summary.json", "task_d_d4_overview.png",
                    "effective_config.json", "effective_adaptive_settings.info")}
            if configs[key]["round_index"] != round_index:
                summaries[key]["pass"] = False

    roles_ok = True
    for run_name in run_names:
        nominal = configs["nominal/" + run_name]
        off = configs["uncertain_off/" + run_name]
        on = configs["uncertain_adaptive/" + run_name]
        roles_ok = roles_ok and bool(
            nominal["enableParameterUpdate"] is True
            and nominal["enableAdaptiveDynamics"] is True
            and nominal["truePlantPayloadInertialEnabled"] is False
            and off["enableParameterUpdate"] is False
            and off["enableAdaptiveDynamics"] is False
            and off["truePlantPayloadInertialEnabled"] is True
            and on["enableParameterUpdate"] is True
            and on["enableAdaptiveDynamics"] is True
            and on["truePlantPayloadInertialEnabled"] is True)
    normalized = {json.dumps(normalized_config(value), sort_keys=True)
                  for value in configs.values()}
    all_pass = all(value.get("pass") and value.get("formal_eligibility")
                   for value in summaries.values())
    source_equal = len(set(source_hashes.values())) == 1
    expected_sequences = list(range(1, len(run_names) * len(GROUPS) + 1))
    sequence_ok = sequence_values == expected_sequences
    protocol_ok = bool(protocol.get("passed")) and all(
        config.get("d4_protocol_sha256") == digest(
            root / "protocol" / "stage_d4_protocol.json")
        for config in configs.values())
    passed = bool(
        protocol_ok and roles_ok and len(normalized) == 1 and all_pass
        and source_equal and sequence_ok)
    result = {
        "schema_version": 4,
        "stage": ("D4_round_01_full_horizon_gate" if args.runs == 1
                  else "D4_formal_three_group_campaign_fairness"),
        "passed": passed,
        "formal_campaign_started": args.runs == 3,
        "round_01_retained_as_formal": args.runs == 3,
        "repetitions_authorized": passed,
        "groups": list(GROUPS),
        "runs_per_group_audited": args.runs,
        "total_runs_audited": len(summaries),
        "group_roles_ok": roles_ok,
        "normalized_effective_configs_identical": len(normalized) == 1,
        "source_snapshots_identical": source_equal,
        "source_snapshot_sha256": source_hashes,
        "acquisition_sequence_ok": sequence_ok,
        "acquisition_sequence": sequence_values,
        "all_run_validations_pass": all_pass,
        "protocol_ok": protocol_ok,
        "summaries": summaries,
        "artifacts_sha256": artifact_hashes,
        "scientific_scope": (
            "full-horizon structural gate" if args.runs == 1 else
            "fairness and data-integrity audit; performance is analyzed separately"),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
