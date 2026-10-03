#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
records_root="${1:-$workspace/records/task_c_payload_mass_1d/formal_adaptive_off}"
runner="$package_dir/scripts/run_task_c_payload_mass_formal.sh"

for group in nominal uncertain_off uncertain; do
  for index in 01 02 03; do
    target="$records_root/$group/run_$index"
    if [[ -e "$target" ]] && find "$target" -mindepth 1 -print -quit | grep -q .; then
      echo "refusing to overwrite existing PayloadMass1D run: $target" >&2
      exit 2
    fi
  done
done
mkdir -p "$records_root"
exec >"$records_root/campaign.log" 2>&1

run_one() {
  local group="$1"
  local index="$2"
  echo "PayloadMass1D three-group acquisition: ${group}/run_${index}"
  "$runner" "$group" "$records_root/$group/run_$index"
}

# Three-group round-robin order reduces time-correlated environment bias.
# OFF changes only enableParameterUpdate and enableAdaptiveDynamics; CLF stays ON.
for index in 01 02 03; do
  run_one nominal "$index"
  run_one uncertain_off "$index"
  run_one uncertain "$index"
done

python3 "$package_dir/scripts/audit_task_c_payload_mass_adaptive_off.py" "$records_root" \
  >"$records_root/fairness_audit_adaptive_off.log" 2>&1
python3 "$package_dir/scripts/analyze_task_c_payload_mass_campaign.py" "$records_root" \
  --include-adaptive-off --analysis-directory analysis_adaptive_off \
  >"$records_root/analysis_adaptive_off.log" 2>&1

echo "PayloadMass1D formal three-group campaign PASS"
