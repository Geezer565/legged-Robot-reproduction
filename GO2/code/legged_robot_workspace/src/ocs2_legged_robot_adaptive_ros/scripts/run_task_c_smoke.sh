#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
core_package_dir="$workspace/src/ocs2_legged_robot_adaptive"

output_dir="${1:-$workspace/records/task_c_mass/smoke/uncertain_adaptive/run_01}"
if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty Task C directory: $output_dir" >&2
  exit 2
fi
mkdir -p "$output_dir"
exec >>"$output_dir/terminal.log" 2>&1
echo "Task C Stage C2 smoke start: output=$output_dir"

# C2 is one short uncertain-plant + Full Adaptive smoke. It is not a formal
# paired acquisition and may not be used to claim a performance improvement.
target_start_time="0.50"
target_step="0.10"
smoke_end_time="6.00"
nominal_mass="52.13485"
true_plant_mass_scale="1.10"
true_mass="57.348335"
payload_delta="5.213485"

# Generate the same audited effective-config artifacts as Task B without
# mutating the already-active configuration file.
python3 "$package_dir/scripts/set_task_b_mode.py" set full_adaptive --dry-run \
  --snapshot-dir "$output_dir" >"$output_dir/mode_audit.log" 2>&1

python3 - "$output_dir/effective_config.json" "$output_dir/task_c_effective_config.json" \
  "$output_dir/experiment_schedule.json" <<'PY'
import json
import math
import sys

adaptive_path, task_c_path, schedule_path = sys.argv[1:]
with open(adaptive_path) as stream:
    adaptive = json.load(stream)
required = {
    "mode": "full_adaptive",
    "enableParameterUpdate": True,
    "enableAdaptiveDynamics": True,
    "enableClfConstraint": True,
    "enableInputBiasCost": False,
}
for key, expected in required.items():
    if adaptive.get(key) != expected:
        raise RuntimeError("unexpected adaptive setting {}={!r}, expected {!r}".format(
            key, adaptive.get(key), expected))

nominal_mass = 52.13485
mass_scale = 1.10
true_mass = nominal_mass * mass_scale
task_c = {
    "stage": "C2",
    "run_type": "smoke_only",
    "controller_mode": "full_adaptive",
    "controller_model_id": "nominal_urdf_srbd_v1",
    "plant_model_id": "srbd_mass_scalar_v1",
    "nominal_mass_kg": nominal_mass,
    "true_plant_mass_scale": mass_scale,
    "true_mass_kg": true_mass,
    "payload_delta_kg": true_mass - nominal_mass,
    "experimental_independent_variable": "true_plant_mass_scale",
    "task_c_diagnostics_enabled": True,
    "artificial_constant_wrench_enabled": False,
    "artificial_constant_wrench_world": [0.0] * 6,
    "smoke_end_mpc_time_s": 6.0,
    "scientific_scope": "Smoke validation only; not formal Task C evidence.",
}
if not math.isclose(true_mass, 57.348335, rel_tol=0.0, abs_tol=1.0e-12):
    raise RuntimeError("mass arithmetic changed")
with open(task_c_path, "w") as stream:
    json.dump(task_c, stream, indent=2, sort_keys=True)
    stream.write("\n")

schedule = {
    "stage": "C2",
    "target": {
        "axis": "x", "step_m": 0.10, "start_mpc_time_s": 0.50,
        "target_displacement_velocity_m_s": 0.5,
        "transition_duration_s": 0.20,
    },
    "artificial_disturbance": {
        "enabled": False, "frame": "world", "wrench": [0.0] * 6,
    },
    "plant_mass_override": {
        "nominal_mass_kg": nominal_mass, "true_mass_kg": true_mass,
        "payload_delta_kg": true_mass - nominal_mass, "mass_scale": mass_scale,
        "active_for_entire_run": True,
    },
    "smoke_window": {
        "time_axis": "MPC/simulation time", "start_s": 0.0, "end_s": 6.0,
    },
}
with open(schedule_path, "w") as stream:
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
  echo "an existing ROS master is running; refusing a non-isolated Task C smoke" >&2
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

rosbag record --buffsize=512 -O "$output_dir/task_c_smoke.bag" \
  /legged_robot_mpc_observation /legged_robot_mpc_target \
  /adaptive_update/sample /adaptive_update/disturbance_sample /task_b/equation10_sample \
  /task_c/model_identity /task_c/model_mismatch_sample \
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

timeout 600s python3 - "$smoke_end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys
import rospy
from std_msgs.msg import Float64MultiArray

end_time = float(sys.argv[1])
rospy.init_node("task_c_smoke_waiter", anonymous=True)

def callback(message):
    if len(message.data) == 41 and message.data[0] >= end_time:
        rospy.signal_shutdown("Task C C2 fixed MPC horizon reached")

rospy.Subscriber("/task_c/model_mismatch_sample", Float64MultiArray,
                 callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros task_c_mass_smoke.launch \
  target_start_time:="$target_start_time" target_step:="$target_step" \
  true_plant_mass_scale:="$true_plant_mass_scale" \
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

python3 "$package_dir/scripts/analyze_task_c_smoke.py" "$output_dir" \
  >"$output_dir/smoke_analysis.log" 2>&1

printf '%s\n' \
  "Task C Stage C2 smoke acquisition and validation complete." \
  "controller_mode=full_adaptive" \
  "nominal_mass_kg=$nominal_mass" \
  "true_mass_kg=$true_mass" \
  "payload_delta_kg=$payload_delta" \
  "bag=$output_dir/task_c_smoke.bag" \
  "This smoke run is not formal scientific evidence."
