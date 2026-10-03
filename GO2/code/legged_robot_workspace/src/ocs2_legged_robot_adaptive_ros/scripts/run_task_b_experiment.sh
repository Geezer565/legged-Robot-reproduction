#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
core_package_dir="$workspace/src/ocs2_legged_robot_adaptive"

mode="${1:-}"
if [[ "$mode" != "nominal" && "$mode" != "estimate_only" && "$mode" != "full_adaptive" ]]; then
  echo "usage: $0 {nominal|estimate_only|full_adaptive} [new_output_directory]" >&2
  exit 2
fi
output_dir="${2:-$workspace/records/task_b/$mode/run_01}"
if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty Task B directory: $output_dir" >&2
  exit 2
fi
mkdir -p "$output_dir"
# Direct append guarantees that a failed child process has flushed its complete
# terminal record before this runner returns to the campaign driver.
exec >>"$output_dir/terminal.log" 2>&1
echo "Task B acquisition start: mode=$mode output=$output_dir"

# Formal Task B baseline: target 0.50--0.70 s, disturbance 1.0--4.0 s,
# then a fixed 15 s analysis window measured from the target start knot.
target_start_time="0.50"
target_step="0.10"
disturbance_start_time="1.0"
disturbance_end_time="4.0"
analysis_window="15.0"
experiment_end_time="15.50"

python3 "$package_dir/scripts/set_task_b_mode.py" set "$mode" \
  --snapshot-dir "$output_dir" >"$output_dir/mode_switch.log" 2>&1

python3 - "$output_dir/experiment_schedule.json" "$mode" <<'PY'
import json
import sys

path, mode = sys.argv[1:]
schedule = {
    "mode": mode,
    "target": {"axis": "x", "step_m": 0.10, "start_mpc_time_s": 0.50,
               "target_displacement_velocity_m_s": 0.5, "transition_duration_s": 0.20},
    "disturbance": {"parameterization": "ConstantWrench6D", "frame": "world",
                    "wrench": [20.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "start_simulation_time_s": 1.0, "end_simulation_time_s": 4.0},
    "analysis_window": {"time_axis": "MPC/simulation time", "start_s": 0.50,
                        "duration_s": 15.0, "end_s": 15.50},
}
with open(path, "w") as stream:
    json.dump(schedule, stream, indent=2, sort_keys=True)
    stream.write("\n")
PY

find "$core_package_dir" "$package_dir" -type f \
  ! -path '*/__pycache__/*' ! -name '*.pyc' -print0 | sort -z | xargs -0 sha256sum \
  >"$output_dir/source_files.sha256"
if git -C "$workspace" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  git -C "$workspace" rev-parse HEAD >"$output_dir/git_head.txt"
  git -C "$workspace" status --short >"$output_dir/git_status.txt"
  git -C "$workspace" diff -- "$core_package_dir" "$package_dir" >"$output_dir/git_diff.patch"
else
  printf '%s\n' "workspace is not a Git work tree; source_files.sha256 is the code-version record" \
    >"$output_dir/git_unavailable.txt"
fi

source "$workspace/devel/setup.bash"
if rosparam list >/dev/null 2>&1; then
  echo "an existing ROS master is running; refusing a non-isolated Task B run" >&2
  exit 2
fi

core_pid=""
launch_pid=""
bag_pid=""
waiter_pid=""
cleanup() {
  for pid_name in waiter_pid launch_pid bag_pid core_pid; do
    pid="${!pid_name}"
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

roscore >"$output_dir/roscore.log" 2>&1 &
core_pid=$!
for _ in $(seq 1 60); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  if ! kill -0 "$core_pid" 2>/dev/null; then
    echo "roscore stopped during startup" >&2
    exit 1
  fi
  sleep 1
done
if ! rosparam list >/dev/null 2>&1; then
  echo "roscore did not start within 60 seconds" >&2
  exit 1
fi

rosbag record --buffsize=512 -O "$output_dir/task_b.bag" \
  /legged_robot_mpc_observation /legged_robot_mpc_target \
  /adaptive_update/sample /adaptive_update/disturbance_sample /task_b/equation10_sample \
  /task_b/observation_time /task_b/desired_x /task_b/actual_x \
  /task_b/desired_position /task_b/actual_position \
  /task_b/desired_rpy /task_b/actual_rpy /task_b/orientation_error_rpy \
  /task_b/orientation_error_norm /task_b/roll_error /task_b/yaw_error \
  /task_b/policy_input_sample \
  /aclf_experiment/x_error /aclf_experiment/pitch_error /aclf_experiment/position_error_norm \
  /aclf_experiment/executed_h_clf /aclf_experiment/executed_violation \
  /aclf_experiment/executed_sigma_norm /aclf_experiment/executed_contribution \
  /aclf_experiment/executed_residual_wrench /aclf_experiment/executed_damping_term \
  /aclf_experiment/executed_linear_term /aclf_experiment/executed_observation_time \
  /aclf_experiment/executed_snapshot_time /aclf_experiment/executed_time_error \
  /aclf/sigma_linear /aclf/sigma_angular /aclf/sigma_norm /aclf/pi_hat \
  /aclf/h_clf /aclf/violation /aclf/damping_term /aclf/contact_wrench \
  /aclf/nominal_wrench /aclf/estimated_unknown_wrench /aclf/residual_wrench \
  /aclf/contribution /aclf/linear_term /aclf/reconstructed_h \
  >"$output_dir/rosbag.log" 2>&1 &
bag_pid=$!

timeout 600s python3 - "$experiment_end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys
import rospy
from std_msgs.msg import Float64MultiArray

end_time = float(sys.argv[1])
rospy.init_node("task_b_experiment_waiter", anonymous=True)

def callback(message):
    if len(message.data) == 8 and message.data[0] >= end_time:
        rospy.signal_shutdown("Task B fixed MPC horizon reached")

rospy.Subscriber("/adaptive_update/disturbance_sample", Float64MultiArray,
                 callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros task_b_experiment.launch \
  target_start_time:="$target_start_time" target_step:="$target_step" \
  disturbance_start_time:="$disturbance_start_time" disturbance_end_time:="$disturbance_end_time" \
  >"$output_dir/roslaunch.log" 2>&1 &
launch_pid=$!

wait "$waiter_pid"
waiter_pid=""
kill -INT "$launch_pid" 2>/dev/null || true
wait "$launch_pid" 2>/dev/null || true
launch_pid=""
sleep 1
kill -INT "$bag_pid" 2>/dev/null || true
wait "$bag_pid" 2>/dev/null || true
bag_pid=""
kill -INT "$core_pid" 2>/dev/null || true
wait "$core_pid" 2>/dev/null || true
core_pid=""

python3 "$package_dir/scripts/analyze_task_b_smoke.py" "$output_dir" \
  >"$output_dir/smoke_analysis.log" 2>&1

printf '%s\n' \
  "Task B raw acquisition complete." \
  "mode=$mode" \
  "bag=$output_dir/task_b.bag" \
  "analysis_window_s=$analysis_window" \
  "No final scientific conclusion is produced by this acquisition script."
