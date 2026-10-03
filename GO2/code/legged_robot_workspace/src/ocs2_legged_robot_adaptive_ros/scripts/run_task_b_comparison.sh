#!/usr/bin/env bash
set -eo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
output_dir="${1:-$workspace/records/task_b_formal}"
config="$workspace/src/ocs2_legged_robot_adaptive/config/adaptive_settings.info"
horizon="6.0"

mkdir -p "$output_dir"
source "$workspace/devel/setup.bash"
set -u

original_config="$output_dir/adaptive_settings.original.info"
cp "$config" "$original_config"
core_pid=""
launch_pid=""
bag_pid=""
waiter_pid=""

stop_pid() {
  local pid="$1"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

stop_group() {
  stop_pid "$waiter_pid"
  stop_pid "$launch_pid"
  stop_pid "$bag_pid"
  stop_pid "$core_pid"
  core_pid=""; launch_pid=""; bag_pid=""; waiter_pid=""
}

cleanup() {
  stop_group
  if [[ -f "$original_config" ]]; then
    cp "$original_config" "$config"
  fi
}
trap cleanup EXIT INT TERM

wait_for_core() {
  for _ in $(seq 1 60); do
    if rosparam list >/dev/null 2>&1; then return 0; fi
    if ! kill -0 "$core_pid" 2>/dev/null; then return 1; fi
    sleep 1
  done
  return 1
}

run_group() {
  local mode="$1"
  local group="$output_dir/$mode"
  mkdir -p "$group"
  python3 "$package_dir/scripts/set_adaptive_experiment_mode.py" "$mode" --config "$config" > "$group/mode.json"
  cp "$config" "$group/adaptive_settings.info"

  roscore > "$group/roscore.log" 2>&1 & core_pid=$!
  if ! wait_for_core; then
    echo "roscore startup failed" > "$group/runner.error"
    return 1
  fi

  rosbag record --buffsize=256 -O "$group/task_b.bag" \
    /legged_robot_mpc_observation \
    /adaptive_update/sample /adaptive_update/disturbance_sample \
    /aclf_experiment/executed_sigma_norm /aclf_experiment/executed_h_clf \
    /aclf_experiment/executed_violation /aclf_experiment/executed_observation_time \
    /aclf_experiment/executed_snapshot_time /aclf_experiment/executed_time_error \
    > "$group/rosbag.log" 2>&1 & bag_pid=$!

  timeout 300s python3 - "$horizon" > "$group/waiter.log" 2>&1 <<'PY' &
import sys
import rospy
from std_msgs.msg import Float64MultiArray

horizon = float(sys.argv[1])
rospy.init_node("task_b_waiter", anonymous=True)

def callback(message):
    if len(message.data) == 8 and message.data[0] >= horizon:
        rospy.signal_shutdown("formal Task B horizon reached")

rospy.Subscriber("/adaptive_update/disturbance_sample", Float64MultiArray, callback, queue_size=10)
rospy.spin()
PY
  waiter_pid=$!

  roslaunch ocs2_legged_robot_adaptive_ros adaptive_update_validation.launch \
    disturbance_start_time:=1.0 disturbance_end_time:=4.0 disturbance_fx:=20.0 \
    > "$group/roslaunch.log" 2>&1 & launch_pid=$!

  set +e
  wait "$waiter_pid"
  local waiter_status=$?
  set -e
  waiter_pid=""
  if [[ "$waiter_status" -ne 0 ]]; then
    echo "$waiter_status" > "$group/runner.status"
    return "$waiter_status"
  fi

  stop_pid "$launch_pid"; launch_pid=""
  sleep 1
  stop_pid "$bag_pid"; bag_pid=""
  stop_pid "$core_pid"; core_pid=""
  echo 0 > "$group/runner.status"
}

for mode in nominal estimate_only full_adaptive; do
  run_group "$mode"
  stop_group
done

python3 "$package_dir/scripts/analyze_task_b.py" \
  --nominal-bag "$output_dir/nominal/task_b.bag" \
  --nominal-config "$output_dir/nominal/adaptive_settings.info" \
  --estimate-only-bag "$output_dir/estimate_only/task_b.bag" \
  --estimate-only-config "$output_dir/estimate_only/adaptive_settings.info" \
  --full-adaptive-bag "$output_dir/full_adaptive/task_b.bag" \
  --full-adaptive-config "$output_dir/full_adaptive/adaptive_settings.info" \
  --output-dir "$output_dir"
