#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
records_root="${1:-$workspace/records/task_c_locomotion_payload_mass_1d/formal_v1}"
runner="$package_dir/scripts/run_task_c_locomotion_payload_experiment.sh"

for group in nominal uncertain_off uncertain; do
  for index in 01 02 03; do
    target="$records_root/$group/run_$index"
    if [[ -e "$target" ]] && find "$target" -mindepth 1 -print -quit | grep -q .; then
      echo "refusing to overwrite existing locomotion payload run: $target" >&2
      exit 2
    fi
  done
done
mkdir -p "$records_root"
exec >"$records_root/campaign.log" 2>&1

run_one() {
  local group="$1"
  local index="$2"
  echo "Static-walk PayloadMass1D acquisition: ${group}/run_${index}"
  "$runner" "$group" "$records_root/$group/run_$index"
}

# First round is the required three-group sanity round.  Each runner must pass
# payload algebra, snapshot, Eq.(10), fixed-target, executed-mode and foot-swing
# validation before repetitions two and three are allowed to start.
run_one nominal 01
run_one uncertain_off 01
run_one uncertain 01
python3 - "$records_root" <<'PY'
import json
import sys
from pathlib import Path
root = Path(sys.argv[1])
result = {"passed": True, "scope": "first-round three-group locomotion sanity", "runs": {}}
for group in ("nominal", "uncertain_off", "uncertain"):
    run = root / group / "run_01"
    payload = json.loads((run / "payload_mass_smoke_summary.json").read_text())
    locomotion = json.loads((run / "locomotion_smoke_summary.json").read_text())
    passed = bool(payload.get("passed") and locomotion.get("locomotion_confirmed") and
                  locomotion.get("passed_structural_validation"))
    result["runs"][group] = {
        "passed": passed,
        "target": locomotion.get("target"),
        "non_stance_modes": locomotion.get("non_stance_modes"),
        "swinging_foot_count": locomotion.get("swinging_foot_count"),
    }
    result["passed"] = result["passed"] and passed
(root / "round_01_sanity.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
if not result["passed"]:
    raise SystemExit("first-round sanity failed; repetitions stopped")
PY

for index in 02 03; do
  run_one nominal "$index"
  run_one uncertain_off "$index"
  run_one uncertain "$index"
done

python3 "$package_dir/scripts/audit_task_c_locomotion_payload_campaign.py" "$records_root" \
  >"$records_root/fairness_audit.log" 2>&1
python3 "$package_dir/scripts/analyze_task_c_payload_mass_campaign.py" "$records_root" \
  --include-adaptive-off --analysis-directory analysis \
  --grid-start 1.20 --grid-end 30.00 --expected-target-start 1.20 \
  >"$records_root/analysis.log" 2>&1

echo "Static-walk PayloadMass1D formal three-group campaign PASS"
