#!/usr/bin/env bash
# Shared bootstrap only; experiment definitions remain in the existing runner.
set -eo pipefail
GO2_ENTRY_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GO2_ROOT="$(cd -- "$GO2_ENTRY_DIR/../.." && pwd)"
GO2_WS="$GO2_ROOT/code/legged_robot_workspace"
GO2_RUNNER="$GO2_WS/src/leggedcontrol_go2/legged_controllers/scripts/run_go2_method_comparison.py"

run_selected() {
  local mode=--execute token
  local forwarded=()
  for token in "$@"; do
    case "$token" in
      --plan|--dry-run) mode=--plan ;;
      --execute) ;;
      *) forwarded+=("$token") ;;
    esac
  done
  if [[ "$mode" == --execute ]]; then
    source /opt/ros/noetic/setup.bash
    source "$GO2_WS/devel/setup.bash"
    export ROBOT_TYPE=go2 GO2_EXPERIMENT_ROOT="$GO2_ROOT/experi"
    if [[ -z "${DISPLAY:-}" ]]; then export DISPLAY=:99; fi
    if [[ "$DISPLAY" == :99 ]] && ! pgrep -f '^Xvfb :99([[:space:]]|$)' >/dev/null; then
      Xvfb :99 -screen 0 1280x720x24 -nolisten tcp >/tmp/go2_xvfb_99.log 2>&1 &
    fi
  fi
  exec python3 "$GO2_RUNNER" "$mode" --method all --repetitions 1 \
    --seed 4102 --wall-timeout 900 --fresh --continue-on-error "${forwarded[@]}"
}
