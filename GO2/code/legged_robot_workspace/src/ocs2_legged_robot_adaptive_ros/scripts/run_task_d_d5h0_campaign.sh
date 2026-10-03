#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
output_root="${1:-$workspace/records/task_d_payload_inertial_10d/stage_d5h0_shadow_v1}"
runs="${2:-3}"
end_time="${3:-16.40}"
if [[ ! "$runs" =~ ^[1-9][0-9]*$ ]]; then
  echo "runs must be a positive integer" >&2
  exit 2
fi
if [[ -e "$output_root" ]] && find "$output_root" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty D5H0 campaign: $output_root" >&2
  exit 2
fi
mkdir -p "$output_root/protocol"

python3 "$package_dir/scripts/set_task_d_mode.py" audit \
  --output-dir "$output_root/protocol/task_d_mode_audit" \
  >"$output_root/protocol/task_d_mode_audit.log"
python3 - "$output_root/protocol/task_d_mode_audit/fairness_audit.json" \
  "$output_root/protocol/d5h0_protocol.json" "$runs" "$end_time" <<'PY'
import hashlib, json, sys
from pathlib import Path
audit_path, output_path, runs, end = Path(sys.argv[1]), Path(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4])
audit = json.loads(audit_path.read_text())
pair = audit["pairwise"]["nominal__uncertain_adaptive"]
if not audit["passed"] or pair["changed_keys"] != ["truePlantPayloadInertialEnabled"]:
    raise RuntimeError("Task D fairness audit does not isolate true plant payload")
protocol = {
    "schema_version": 1,
    "stage": "D5H0_shadow_diagnostics",
    "passed": True,
    "shadow_only": True,
    "actual_estimator_modified": False,
    "groups": ["nominal", "uncertain"],
    "runs_per_group": runs,
    "round_robin": "N1,U1,N2,U2,...",
    "end_mpc_time_s": end,
    "allowed_independent_variable": "truePlantPayloadInertialEnabled",
    "task_d_fairness_audit_sha256": hashlib.sha256(audit_path.read_bytes()).hexdigest(),
    "gate_thresholds": {
        "nominal_suppression_fraction": 0.95,
        "payload_contrast_norm_retention_fraction": 0.95,
        "payload_scaled_rmse_max_ratio": 1.05,
        "payload_scaled_cosine_allowed_drop": 0.05
    },
    "interpretation": "online shadow counterfactual; no corrected update reaches control"
}
output_path.write_text(json.dumps(protocol, indent=2, sort_keys=True) + "\n")
PY

runner="$package_dir/scripts/run_task_d_d5h0_experiment.sh"
for run in $(seq 1 "$runs"); do
  index="$(printf '%02d' "$run")"
  "$runner" nominal "$output_root/nominal/run_$index" "$run" formal "$end_time"
  "$runner" uncertain "$output_root/uncertain/run_$index" "$run" formal "$end_time"
done

python3 "$package_dir/scripts/analyze_task_d_d5h0_campaign.py" \
  "$output_root" "$output_root/analysis" --runs "$runs" \
  >"$output_root/campaign_analysis.log" 2>&1
cat "$output_root/campaign_analysis.log"
find "$output_root" -type f ! -path '*/analysis/artifact_manifest.json' -print0 \
  | sort -z | xargs -0 sha256sum >"$output_root/campaign_artifacts.sha256"
echo "D5H0 formal campaign complete: $output_root"
