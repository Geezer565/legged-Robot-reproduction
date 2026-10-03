#!/usr/bin/env bash
set -eo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
output_dir="${1:-$workspace/records/task_c_formal}"
config="$workspace/src/ocs2_legged_robot_adaptive/config/adaptive_settings.info"
horizon="8.0"

mkdir -p "$output_dir"
source "$workspace/devel/setup.bash"
set -u

original_config="$output_dir/adaptive_settings.original.info"
cp "$config" "$original_config"
core_pid=""; launch_pid=""; bag_pid=""; waiter_pid=""; target_pid=""

stop_pid() {
  local pid="$1"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

stop_group() {
  stop_pid "$target_pid"
  stop_pid "$waiter_pid"
  stop_pid "$launch_pid"
  stop_pid "$bag_pid"
  stop_pid "$core_pid"
  core_pid=""; launch_pid=""; bag_pid=""; waiter_pid=""; target_pid=""
}

cleanup() {
  stop_group
  if [[ -f "$original_config" ]]; then cp "$original_config" "$config"; fi
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
  local profile="${mode%_*}"
  local clf_mode="${mode#*_}"
  local switch_mode="clf_$clf_mode"
  local group="$output_dir/$mode"
  mkdir -p "$group"
  python3 "$package_dir/scripts/set_adaptive_experiment_mode.py" "$switch_mode" --config "$config" > "$group/mode.json"
  cp "$config" "$group/adaptive_settings.info"

  roscore > "$group/roscore.log" 2>&1 & core_pid=$!
  if ! wait_for_core; then
    echo "roscore startup failed" > "$group/runner.error"
    return 1
  fi

  rosbag record --buffsize=256 -O "$group/task_c.bag" \
    /legged_robot_mpc_observation /legged_robot_mpc_target \
    /adaptive_update/sample /adaptive_update/disturbance_sample \
    /aclf_experiment/executed_sigma_norm /aclf_experiment/executed_h_clf \
    /aclf_experiment/executed_violation /aclf_experiment/executed_contribution \
    /aclf_experiment/executed_residual_wrench /aclf_experiment/executed_damping_term \
    /aclf_experiment/executed_linear_term /aclf_experiment/executed_observation_time \
    /aclf_experiment/executed_snapshot_time /aclf_experiment/executed_time_error \
    > "$group/rosbag.log" 2>&1 & bag_pid=$!

  timeout 360s python3 - "$horizon" > "$group/waiter.log" 2>&1 <<'PY' &
import sys
import rospy
from std_msgs.msg import Float64MultiArray

horizon = float(sys.argv[1])
rospy.init_node("task_c_waiter", anonymous=True)

def callback(message):
    if len(message.data) == 8 and message.data[0] >= horizon:
        rospy.signal_shutdown("formal Task C horizon reached")

rospy.Subscriber("/adaptive_update/disturbance_sample", Float64MultiArray, callback, queue_size=10)
rospy.spin()
PY
  waiter_pid=$!

  roslaunch ocs2_legged_robot_adaptive_ros task_c_validation.launch \
    > "$group/roslaunch.log" 2>&1 & launch_pid=$!

  timeout 180s python3 "$package_dir/scripts/aclf_small_step_target.py" \
    --step 0.10 --settle-before-step 2.0 --profile "$profile" \
    --duration 1.0 --sample-period 0.02 \
    --step-observation-time 2.0 \
    > "$group/target.log" 2>&1 & target_pid=$!

  set +e
  wait "$target_pid"
  local target_status=$?
  set -e
  target_pid=""
  if [[ "$target_status" -ne 0 ]]; then
    echo "$target_status" > "$group/runner.status"
    return "$target_status"
  fi

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

for mode in linear_off linear_on quintic_off quintic_on; do
  run_group "$mode"
  stop_group
done

python3 "$package_dir/scripts/analyze_task_c.py" \
  --linear-off-bag "$output_dir/linear_off/task_c.bag" \
  --linear-off-config "$output_dir/linear_off/adaptive_settings.info" \
  --linear-on-bag "$output_dir/linear_on/task_c.bag" \
  --linear-on-config "$output_dir/linear_on/adaptive_settings.info" \
  --quintic-off-bag "$output_dir/quintic_off/task_c.bag" \
  --quintic-off-config "$output_dir/quintic_off/adaptive_settings.info" \
  --quintic-on-bag "$output_dir/quintic_on/task_c.bag" \
  --quintic-on-config "$output_dir/quintic_on/adaptive_settings.info" \
  --output-dir "$output_dir"
