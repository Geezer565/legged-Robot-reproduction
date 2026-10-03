#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
root="${1:-}"
if [[ -z "$root" ]]; then
  echo "usage: $0 NEW_CAMPAIGN_ROOT" >&2
  exit 2
fi
for protected in calibration heldout analysis stage_d5i3_protocol.json \
                 acquisition_order.log stage_d5i3_manifest.json; do
  if [[ -e "$root/$protected" ]]; then
    echo "refusing to overwrite existing D5I3 campaign artifact: $root/$protected" >&2
    exit 2
  fi
done
mkdir -p "$root"

if [[ -d "$root/mode_audit" ]]; then
  python3 - "$root/mode_audit/fairness_and_split_audit.json" <<'PY'
import json, sys
if not json.load(open(sys.argv[1])).get("passed"):
    raise SystemExit("existing D5I3 mode audit did not pass")
PY
else
  python3 "$package_dir/scripts/set_task_d_d5i3_mode.py" audit \
    --output-dir "$root/mode_audit" >"$root/mode_audit.log" 2>&1
fi
cp "$package_dir/config/d5i3_nominal_baseline_protocol_v1.json" \
   "$root/stage_d5i3_protocol.json"
: >"$root/acquisition_order.log"

for run_index in 1 2 3; do
  for role in calibration heldout; do
    printf '%s/run_%02d\n' "$role" "$run_index" \
      >>"$root/acquisition_order.log"
    "$package_dir/scripts/run_task_d_d5i3_calibration.sh" \
      "$role" "$run_index" \
      "$root/$role/$(printf 'run_%02d' "$run_index")"
  done
done

python3 "$package_dir/scripts/analyze_task_d_d5i3_campaign.py" "$root" \
  >"$root/campaign_analysis.log" 2>&1
cat "$root/campaign_analysis.log"
