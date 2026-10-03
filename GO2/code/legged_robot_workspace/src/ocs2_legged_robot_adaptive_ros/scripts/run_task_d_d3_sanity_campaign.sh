#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
root="${1:-$workspace/records/task_d_payload_inertial_10d/stage_d3_gate/sanity}"
end_time="${2:-3.0}"

for group in nominal uncertain_off uncertain_adaptive; do
  "$package_dir/scripts/run_task_d_d3_sanity.sh" "$group" \
    "$root/$group/run_01" "$end_time"
done
python3 "$package_dir/scripts/audit_task_d_d3_sanity.py" "$root" \
  "$root/d3_sanity_gate.json"
echo "D3 three-group short sanity gate PASS: $root"
