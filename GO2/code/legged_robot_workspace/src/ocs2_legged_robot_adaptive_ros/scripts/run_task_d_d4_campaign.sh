#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
root="${1:-$workspace/records/task_d_payload_inertial_10d/stage_d4_formal_v1}"
runner="$package_dir/scripts/run_task_d_d4_experiment.sh"
d3_root="$workspace/records/task_d_payload_inertial_10d/stage_d3_gate"

for group in nominal uncertain_off uncertain_adaptive; do
  for index in 01 02 03; do
    target="$root/$group/run_$index"
    if [[ -e "$target" ]] && find "$target" -mindepth 1 -print -quit | grep -q .; then
      echo "refusing to overwrite existing D4 run: $target" >&2
      exit 2
    fi
  done
done
mkdir -p "$root/protocol"
exec >"$root/campaign.log" 2>&1

python3 "$package_dir/scripts/set_task_d_mode.py" audit \
  --output-dir "$root/protocol/config_audit" \
  >"$root/protocol/config_audit.log"
python3 "$package_dir/scripts/audit_task_d_d4_protocol.py" \
  --d3-fairness "$d3_root/config_audit/fairness_audit.json" \
  --d3-gate "$d3_root/sanity/d3_sanity_gate.json" \
  --config-audit "$root/protocol/config_audit/fairness_audit.json" \
  --output "$root/protocol/stage_d4_protocol.json" \
  >"$root/protocol/protocol_audit.log"

sequence=0
run_one() {
  local group="$1"
  local round_index="$2"
  sequence=$((sequence + 1))
  echo "D4 acquisition sequence ${sequence}: ${group}/run_${round_index}"
  "$runner" "$group" "$root/$group/run_$round_index" \
    "$((10#$round_index))" "$sequence" "$root/protocol/stage_d4_protocol.json"
}

# Round 01 is the full-horizon gate. It is retained as formal run_01 only if
# all three structural validations and the cross-run audit pass.
run_one nominal 01
run_one uncertain_off 01
run_one uncertain_adaptive 01
python3 "$package_dir/scripts/audit_task_d_d4_campaign.py" "$root" \
  --runs 1 --output "$root/round_01_full_horizon_gate.json" \
  >"$root/round_01_gate.log"

for index in 02 03; do
  run_one nominal "$index"
  run_one uncertain_off "$index"
  run_one uncertain_adaptive "$index"
done

python3 "$package_dir/scripts/audit_task_d_d4_campaign.py" "$root" \
  --runs 3 --output "$root/stage_d4_fairness_audit.json" \
  >"$root/fairness_audit.log"
python3 "$package_dir/scripts/analyze_task_d_d4_campaign.py" "$root" \
  --output-directory "$root/analysis" >"$root/analysis.log"

echo "D4 formal three-group campaign and unified analysis PASS: $root"
