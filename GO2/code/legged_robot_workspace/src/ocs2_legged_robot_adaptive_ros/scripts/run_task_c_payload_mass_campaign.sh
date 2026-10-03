#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
records_root="${1:-$workspace/records/task_c_payload_mass_1d/formal}"
runner="$package_dir/scripts/run_task_c_payload_mass_formal.sh"

for group in nominal uncertain; do
  for index in 01 02 03; do
    target="$records_root/$group/run_$index"
    if [[ -e "$target" ]] && find "$target" -mindepth 1 -print -quit | grep -q .; then
      echo "refusing to overwrite existing PayloadMass1D formal run: $target" >&2
      exit 2
    fi
  done
done
mkdir -p "$records_root"
exec >"$records_root/campaign.log" 2>&1

run_one() {
  local group="$1"
  local index="$2"
  echo "PayloadMass1D formal acquisition: ${group}/run_${index}"
  "$runner" "$group" "$records_root/$group/run_$index"
}

# Paired round-robin order reduces time-correlated environment bias.
run_one nominal 01
run_one uncertain 01
run_one nominal 02
run_one uncertain 02
run_one nominal 03
run_one uncertain 03

python3 "$package_dir/scripts/audit_task_c_payload_mass_campaign.py" "$records_root" \
  >"$records_root/fairness_audit.log" 2>&1
python3 "$package_dir/scripts/analyze_task_c_payload_mass_campaign.py" "$records_root" \
  >"$records_root/analysis.log" 2>&1

echo "PayloadMass1D formal paired campaign PASS"
