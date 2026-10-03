#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
records_root="${1:-$workspace/records/task_c_mass}"
runner="$package_dir/scripts/run_task_c_mass_experiment.sh"
smoke_dir="$records_root/smoke/uncertain_off/run_01"

for target in "$records_root/uncertain_off/run_01" \
  "$records_root/uncertain_off/run_02" \
  "$records_root/uncertain_off/run_03"; do
  if [[ -e "$target" ]] && find "$target" -mindepth 1 -print -quit | grep -q .; then
    echo "refusing to overwrite existing Adaptive-OFF run: $target" >&2
    exit 2
  fi
done

mkdir -p "$records_root"
exec >"$records_root/stage_c5_adaptive_off_campaign.log" 2>&1

restore_full_adaptive() {
  python3 "$package_dir/scripts/set_task_b_mode.py" set full_adaptive \
    >"$records_root/stage_c5_restore_full_adaptive.log" 2>&1 || true
}
trap restore_full_adaptive EXIT

echo "Task C Stage C5 Adaptive-OFF smoke start"
if [[ ! -f "$smoke_dir/task_c_run_summary.json" ]]; then
  "$runner" uncertain_off "$smoke_dir" smoke
fi
python3 - "$smoke_dir/task_c_run_summary.json" <<'PY'
import json
import sys
with open(sys.argv[1]) as stream:
    result = json.load(stream)
if result.get("passed") is not True:
    raise RuntimeError("Adaptive-OFF smoke validation failed")
PY
echo "Task C Stage C5 Adaptive-OFF smoke PASS"

for index in 01 02 03; do
  echo "Task C Stage C5 formal acquisition: uncertain_off/run_$index"
  "$runner" uncertain_off "$records_root/uncertain_off/run_$index" formal
done

python3 "$package_dir/scripts/audit_task_c_adaptive_off.py" "$records_root" \
  >"$records_root/stage_c5_fairness.log" 2>&1

restore_full_adaptive
trap - EXIT
echo "Task C Stage C5 Adaptive-OFF acquisition and fairness audit complete"
