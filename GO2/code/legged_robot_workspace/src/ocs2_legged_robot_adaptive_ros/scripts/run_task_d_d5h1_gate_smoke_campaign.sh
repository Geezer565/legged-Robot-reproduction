#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
output_root="${1:-$workspace/records/task_d_payload_inertial_10d/stage_d5h1_gate_smoke_v2}"
end_time="${2:-6.40}"
if [[ -e "$output_root" ]] && find "$output_root" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty D5H1 Gate smoke root: $output_root" >&2
  exit 2
fi
mkdir -p "$output_root/protocol"

python3 "$package_dir/scripts/set_task_d_d5h1_mode.py" audit \
  --output-dir "$output_root/protocol/mode_audit" \
  >"$output_root/protocol/mode_audit.log"

runner="$package_dir/scripts/run_task_d_d5h1_experiment.sh"
for group in nominal_raw nominal_corrected uncertain_raw uncertain_corrected; do
  "$runner" "$group" "$output_root/$group/run_01" 1 smoke "$end_time"
done

python3 "$package_dir/scripts/analyze_task_d_d5h1_smoke_set.py" "$output_root" \
  "$output_root/smoke_analysis" >"$output_root/smoke_analysis.log"
python3 "$package_dir/scripts/audit_task_d_d5h1_four_mode_gate.py" "$output_root" \
  "$output_root/gate_analysis" >"$output_root/gate_analysis.log"
sha256sum "$output_root/protocol/mode_audit/fairness_audit.json" \
  "$output_root/smoke_analysis/stage_d5h1_smoke_set_summary.json" \
  "$output_root/gate_analysis/d5h1_four_mode_gate_summary.json" \
  >"$output_root/campaign_gate_artifacts.sha256"
cat "$output_root/gate_analysis.log"
