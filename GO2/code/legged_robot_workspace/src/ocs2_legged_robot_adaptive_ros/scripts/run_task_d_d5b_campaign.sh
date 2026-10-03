#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
root="${1:-$workspace/records/task_d_payload_inertial_10d/stage_d5b_bias_attribution_v1}"
groups=(nominal_stance_hold nominal_stance_target nominal_walk_target uncertain_walk_target)
if [[ -e "$root" ]] && find "$root" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty D5B campaign: $root" >&2
  exit 2
fi
mkdir -p "$root/protocol"
exec >"$root/campaign.log" 2>&1

d5a="$workspace/records/task_d_payload_inertial_10d/stage_d5_identifiability_v1/stage_d5_identifiability_summary.json"
python3 "$package_dir/scripts/audit_task_d_d5b_protocol.py" \
  --d5a-summary "$d5a" --output "$root/protocol/stage_d5b_protocol.json" \
  >"$root/protocol/protocol.log"
protocol="$root/protocol/stage_d5b_protocol.json"
runner="$package_dir/scripts/run_task_d_d5b_experiment.sh"

sequence=0
for group in "${groups[@]}"; do
  sequence=$((sequence + 1))
  echo "D5B smoke sequence $sequence: $group"
  "$runner" "$group" "$root/smoke/$group/run_01" 1 "$sequence" \
    "$protocol" smoke 3.40
done
python3 "$package_dir/scripts/audit_task_d_d5b_campaign.py" "$root" \
  --runs 1 --scope smoke --output "$root/smoke_gate.json" >"$root/smoke_gate.log"

sequence=0
for index in 01 02 03; do
  for group in "${groups[@]}"; do
    sequence=$((sequence + 1))
    echo "D5B formal sequence $sequence: $group/run_$index"
    "$runner" "$group" "$root/formal/$group/run_$index" "$((10#$index))" \
      "$sequence" "$protocol" formal 16.40
  done
done
python3 "$package_dir/scripts/audit_task_d_d5b_campaign.py" "$root" \
  --runs 3 --scope formal --output "$root/formal_fairness_audit.json" \
  >"$root/formal_fairness_audit.log"
python3 "$package_dir/scripts/analyze_task_d_d5b_campaign.py" "$root" \
  >"$root/analysis.log"
echo "D5B campaign PASS: $root"
