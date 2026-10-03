#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
root="${1:-$workspace/records/task_d_payload_inertial_10d/stage_d5h1_formal_v3}"
gate_root="${2:-$workspace/records/task_d_payload_inertial_10d/stage_d5h1_gate_smoke_v2}"
runner="$package_dir/scripts/run_task_d_d5h1_experiment.sh"

for group in nominal_raw nominal_corrected uncertain_raw uncertain_corrected; do
  for index in 01 02 03; do
    target="$root/$group/run_$index"
    if [[ -e "$target" ]] && find "$target" -mindepth 1 -print -quit | grep -q .; then
      echo "refusing to overwrite existing D5H1 formal run: $target" >&2
      exit 2
    fi
  done
done
if [[ ! -f "$gate_root/gate_analysis/d5h1_four_mode_gate_summary.json" ]]; then
  echo "missing D5H1 four-mode Online Gate: $gate_root" >&2
  exit 2
fi

mkdir -p "$root/protocol"
exec >"$root/campaign.log" 2>&1
python3 "$package_dir/scripts/set_task_d_d5h1_mode.py" audit \
  --output-dir "$root/protocol/config_audit" \
  >"$root/protocol/config_audit.log"
python3 "$package_dir/scripts/prepare_task_d_d5h1_formal_protocol.py" \
  --online-gate "$gate_root/gate_analysis/d5h1_four_mode_gate_summary.json" \
  --config-audit "$root/protocol/config_audit/fairness_audit.json" \
  --output "$root/protocol/stage_d5h1_formal_protocol.json" \
  >"$root/protocol/protocol.log"

sequence=0
run_one() {
  local group="$1"
  local index="$2"
  sequence=$((sequence + 1))
  echo "D5H1 formal sequence ${sequence}: ${group}/run_${index}"
  "$runner" "$group" "$root/$group/run_$index" "$((10#$index))" formal 15.0 \
    "$root/protocol/stage_d5h1_formal_protocol.json" "$sequence"
}

# Pre-registered cyclic round-robin order; every round contains all four modes.
run_one nominal_raw 01
run_one nominal_corrected 01
run_one uncertain_raw 01
run_one uncertain_corrected 01

run_one nominal_corrected 02
run_one uncertain_raw 02
run_one uncertain_corrected 02
run_one nominal_raw 02

run_one uncertain_raw 03
run_one uncertain_corrected 03
run_one nominal_raw 03
run_one nominal_corrected 03

python3 "$package_dir/scripts/analyze_task_d_d5h1_formal_campaign.py" "$root" \
  --output-directory "$root/analysis" >"$root/analysis.log"
sha256sum "$root/protocol/stage_d5h1_formal_protocol.json" \
  "$root/analysis/stage_d5h1_formal_campaign_summary.json" \
  "$root/analysis/stage_d5h1_formal_campaign_report.md" \
  >"$root/campaign_artifact_checksums.sha256"
echo "D5H1 formal four-mode x3 campaign completed: $root"
