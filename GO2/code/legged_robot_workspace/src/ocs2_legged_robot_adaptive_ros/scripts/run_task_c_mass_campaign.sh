#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
records_root="${1:-$workspace/records/task_c_mass}"
runner="$package_dir/scripts/run_task_c_mass_experiment.sh"

for group in nominal uncertain_adaptive; do
  for index in 01 02 03; do
    target="$records_root/$group/run_$index"
    if [[ -e "$target" ]] && find "$target" -mindepth 1 -print -quit | grep -q .; then
      echo "refusing to overwrite existing Stage C3 run: $target" >&2
      exit 2
    fi
  done
done

exec >"$records_root/stage_c3_campaign.log" 2>&1
echo "Task C Stage C3 round-robin campaign start"

run_one() {
  local group="$1"
  local index="$2"
  "$runner" "$group" "$records_root/$group/run_$index"
}

# Required single-run sanity gate.
run_one nominal 01
run_one uncertain_adaptive 01

python3 - "$records_root" <<'PY'
import json
import os
import sys

root = sys.argv[1]
results = {}
for group in ("nominal", "uncertain_adaptive"):
    path = os.path.join(root, group, "run_01", "task_c_run_summary.json")
    with open(path) as stream:
        results[group] = json.load(stream).get("passed") is True
if not all(results.values()):
    raise RuntimeError("C3 single-run sanity gate failed: {}".format(results))
with open(os.path.join(root, "stage_c3_sanity_gate.json"), "w") as stream:
    json.dump({"stage": "C3", "passed": True, "runs": results,
               "next_action": "continue round-robin repetitions"},
              stream, indent=2, sort_keys=True)
    stream.write("\n")
PY
echo "Task C Stage C3 single-run sanity gate PASS"

# Continue only after N1/U1 have passed their complete per-run validation.
run_one nominal 02
run_one uncertain_adaptive 02
run_one nominal 03
run_one uncertain_adaptive 03

python3 "$package_dir/scripts/audit_task_c_campaign.py" "$records_root" \
  >"$records_root/stage_c3_fairness.log" 2>&1

echo "Task C Stage C3 acquisition and fairness audit complete"
echo "Stage C4 unified metrics have not been run"
