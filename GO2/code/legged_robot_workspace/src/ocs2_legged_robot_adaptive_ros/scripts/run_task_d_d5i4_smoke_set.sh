#!/usr/bin/env bash
set -uo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
root="${1:-}"
if [[ -z "$root" ]]; then
  echo "usage: $0 NEW_CAMPAIGN_ROOT" >&2
  exit 2
fi
if [[ -e "$root" ]] && find "$root" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty D5I4 root: $root" >&2
  exit 2
fi
mkdir -p "$root"
python3 "$package_dir/scripts/set_task_d_d5i4_mode.py" audit \
  --output-dir "$root/mode_audit" >"$root/mode_audit.log" 2>&1 || exit $?

groups=(nominal_original nominal_phase_corrected payload_original payload_phase_corrected)
status=0
: >"$root/acquisition_order.log"
: >"$root/acquisition_status.log"
for group in "${groups[@]}"; do
  echo "$group" >>"$root/acquisition_order.log"
  "$package_dir/scripts/run_task_d_d5i4_smoke.sh" \
    "$group" "$root/$group"
  code=$?
  echo "$group $code" >>"$root/acquisition_status.log"
  if [[ "$code" -ne 0 ]]; then status="$code"; fi
done
analysis_status=0
python3 "$package_dir/scripts/analyze_task_d_d5i4_smoke_set.py" "$root" \
  >"$root/smoke_set_analysis.log" 2>&1 || analysis_status=$?
cat "$root/smoke_set_analysis.log"
if [[ "$status" -ne 0 ]]; then
  echo "At least one D5I4 run failed; all raw runs and the aggregate failure report were preserved." >&2
  exit "$status"
fi
exit "$analysis_status"
