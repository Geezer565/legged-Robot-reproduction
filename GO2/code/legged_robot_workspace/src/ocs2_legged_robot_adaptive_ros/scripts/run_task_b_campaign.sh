#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
first_run="${1:-1}"
last_run="${2:-3}"
resume_mode="${3:-}"

if ! [[ "$first_run" =~ ^[1-9][0-9]*$ && "$last_run" =~ ^[1-9][0-9]*$ ]] \
    || (( first_run > last_run )) \
    || [[ -n "$resume_mode" && "$resume_mode" != "--resume" ]]; then
  echo "usage: $0 [first_run] [last_run] [--resume]" >&2
  exit 2
fi

run_has_passed_validation() {
  local output_dir="$1"
  python3 -c 'import json,sys; sys.exit(0 if json.load(open(sys.argv[1])).get("passed") is True else 1)' \
    "$output_dir/task_b_smoke_summary.json" 2>/dev/null
}

modes=(nominal estimate_only full_adaptive)
for run_number in $(seq "$first_run" "$last_run"); do
  printf -v run_id "run_%02d" "$run_number"
  for mode in "${modes[@]}"; do
    output_dir="$workspace/records/task_b/$mode/$run_id"
    if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
      if [[ "$resume_mode" == "--resume" ]] && run_has_passed_validation "$output_dir"; then
        continue
      fi
      echo "refusing to overwrite formal Task B run: $output_dir" >&2
      exit 2
    fi
  done
done

log_suffix=""
if [[ "$resume_mode" == "--resume" ]]; then
  log_suffix="_resume"
fi
campaign_log="$workspace/records/task_b/campaign_runs_${first_run}_to_${last_run}${log_suffix}.log"
if [[ -e "$campaign_log" ]]; then
  echo "refusing to overwrite campaign log: $campaign_log" >&2
  exit 2
fi
exec > >(tee "$campaign_log") 2>&1

restore_full_adaptive() {
  python3 "$package_dir/scripts/set_task_b_mode.py" set full_adaptive \
    >"$workspace/records/task_b/campaign_restore_full_adaptive.log" 2>&1 || true
}
trap restore_full_adaptive EXIT

# Round-robin order ensures every repetition contains all three modes before
# advancing to the next repetition number.  Each child runner performs its own
# isolated ROS-master check, acquisition, cleanup, and single-run validation.
for run_number in $(seq "$first_run" "$last_run"); do
  printf -v run_id "run_%02d" "$run_number"
  for mode in "${modes[@]}"; do
    output_dir="$workspace/records/task_b/$mode/$run_id"
    if [[ "$resume_mode" == "--resume" ]] && run_has_passed_validation "$output_dir"; then
      echo "Task B validated run already present; skipping: mode=$mode run=$run_id"
      continue
    fi
    echo "Task B formal acquisition: mode=$mode run=$run_id"
    "$package_dir/scripts/run_task_b_experiment.sh" "$mode" "$output_dir"
  done
done

restore_full_adaptive
trap - EXIT
echo "Task B campaign acquisition complete: runs ${first_run}..${last_run}; active config restored to full_adaptive"
