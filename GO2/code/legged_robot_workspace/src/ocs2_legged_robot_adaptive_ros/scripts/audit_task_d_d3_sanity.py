#!/usr/bin/env python3
"""Cross-audit three short D3 sanity runs without making formal inference."""

import hashlib
import json
import sys
from pathlib import Path


GROUPS = ("nominal", "uncertain_off", "uncertain_adaptive")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def normalized_config(config):
    value = json.loads(json.dumps(config))
    for key in ("group", "adaptive_config_sha256", "end_mpc_time_s"):
        value.pop(key, None)
    value["enableParameterUpdate"] = "<D3_MODE>"
    value["enableAdaptiveDynamics"] = "<D3_MODE>"
    value["truePlantPayloadInertialEnabled"] = "<D3_PLANT>"
    return value


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: audit_task_d_d3_sanity.py SANITY_ROOT OUTPUT_JSON")
    root = Path(sys.argv[1]).resolve()
    output = Path(sys.argv[2]).resolve()
    summaries, configs, source_hashes, artifact_hashes = {}, {}, {}, {}
    for group in GROUPS:
        run = root / group / "run_01"
        summaries[group] = json.loads((run / "d3_sanity_summary.json").read_text())
        configs[group] = json.loads((run / "effective_config.json").read_text())
        source_hashes[group] = digest(run / "source_files.sha256")
        artifact_hashes[group] = {
            name: digest(run / name) for name in
            ("d3_sanity.bag", "d3_sanity.csv", "d3_sanity_summary.json",
             "d3_sanity.png", "effective_adaptive_settings.info")
        }
    roles_ok = (
        configs["nominal"]["enableParameterUpdate"] is True
        and configs["nominal"]["enableAdaptiveDynamics"] is True
        and configs["nominal"]["truePlantPayloadInertialEnabled"] is False
        and configs["uncertain_off"]["enableParameterUpdate"] is False
        and configs["uncertain_off"]["enableAdaptiveDynamics"] is False
        and configs["uncertain_off"]["truePlantPayloadInertialEnabled"] is True
        and configs["uncertain_adaptive"]["enableParameterUpdate"] is True
        and configs["uncertain_adaptive"]["enableAdaptiveDynamics"] is True
        and configs["uncertain_adaptive"]["truePlantPayloadInertialEnabled"] is True
    )
    normalized = {json.dumps(normalized_config(value), sort_keys=True)
                  for value in configs.values()}
    all_pass = all(value["pass"] for value in summaries.values())
    source_equal = len(set(source_hashes.values())) == 1
    passed = roles_ok and len(normalized) == 1 and all_pass and source_equal
    result = {
        "schema_version": 3,
        "passed": passed,
        "scientific_scope": "D3 short sanity gate only; no formal group comparison",
        "formal_campaign_started": False,
        "group_roles_ok": roles_ok,
        "normalized_effective_configs_identical": len(normalized) == 1,
        "source_snapshots_identical": source_equal,
        "source_snapshot_sha256": source_hashes,
        "all_group_sanity_pass": all_pass,
        "summaries": summaries,
        "artifacts_sha256": artifact_hashes,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
