#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
core_package_dir="$workspace/src/ocs2_legged_robot_adaptive"

group="${1:-}"
case "$group" in
  nominal)
    true_plant_mass_scale="1.00"
    true_mass="52.13485"
    payload_delta="0.0"
    plant_model_id="nominal_ocs2_rollout_v1"
    task_b_mode="full_adaptive"
    controller_mode="full_adaptive"
    ;;
  uncertain_adaptive)
    true_plant_mass_scale="1.10"
    true_mass="57.348335"
    payload_delta="5.213485"
    plant_model_id="srbd_mass_scalar_v1"
    task_b_mode="full_adaptive"
    controller_mode="full_adaptive"
    ;;
  uncertain_off)
    true_plant_mass_scale="1.10"
    true_mass="57.348335"
    payload_delta="5.213485"
    plant_model_id="srbd_mass_scalar_v1"
    task_b_mode="nominal"
    controller_mode="adaptive_off"
    ;;
  *)
    echo "usage: $0 {nominal|uncertain_adaptive|uncertain_off} [new_output_directory] [formal|smoke]" >&2
    exit 2
    ;;
esac

output_dir="${2:-$workspace/records/task_c_mass/$group/run_01}"
run_kind="${3:-formal}"
case "$run_kind" in
  formal)
    stage="C5"
    run_type="formal_adaptive_off_extension"
    experiment_end_time="15.50"
    analysis_window="15.0"
    ;;
  smoke)
    stage="C5-smoke"
    run_type="smoke_only"
    experiment_end_time="3.00"
    analysis_window="2.5"
    ;;
  *)
    echo "run kind must be formal or smoke" >&2
    exit 2
    ;;
esac
if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty Task C directory: $output_dir" >&2
  exit 2
fi
mkdir -p "$output_dir"
exec >>"$output_dir/terminal.log" 2>&1
echo "Task C $stage acquisition start: group=$group controller_mode=$controller_mode output=$output_dir"

core_pid=""
launch_pid=""
bag_pid=""
waiter_pid=""
mode_was_switched="false"
cleanup() {
  for pid_name in waiter_pid launch_pid bag_pid core_pid; do
    pid="${!pid_name}"
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  if [[ "$mode_was_switched" == "true" ]]; then
    python3 "$package_dir/scripts/set_task_b_mode.py" set full_adaptive \
      >"$output_dir/restore_full_adaptive.log" 2>&1 || true
  fi
}
trap cleanup EXIT

target_start_time="0.50"
target_step="0.10"
nominal_mass="52.13485"

python3 "$package_dir/scripts/set_task_b_mode.py" set "$task_b_mode" \
  --snapshot-dir "$output_dir" >"$output_dir/mode_audit.log" 2>&1
mode_was_switched="true"

python3 - "$output_dir/effective_config.json" "$output_dir/task_c_effective_config.json" \
  "$output_dir/experiment_schedule.json" "$group" "$true_plant_mass_scale" \
  "$plant_model_id" "$task_b_mode" "$controller_mode" "$stage" "$run_type" \
  "$experiment_end_time" "$analysis_window" <<'PY'
import json
import math
import sys

adaptive_path, task_c_path, schedule_path, group, scale_text, plant_model_id, \
    task_b_mode, controller_mode, stage, run_type, end_text, duration_text = sys.argv[1:]
with open(adaptive_path) as stream:
    adaptive = json.load(stream)
expected_update = task_b_mode == "full_adaptive"
expected_dynamics = task_b_mode == "full_adaptive"
required = {
    "mode": task_b_mode,
    "enableParameterUpdate": expected_update,
    "enableAdaptiveDynamics": expected_dynamics,
    "enableClfConstraint": True,
    "enableInputBiasCost": False,
}
for key, expected in required.items():
    if adaptive.get(key) != expected:
        raise RuntimeError("unexpected adaptive setting {}={!r}, expected {!r}".format(
            key, adaptive.get(key), expected))

nominal_mass = 52.13485
mass_scale = float(scale_text)
true_mass = nominal_mass * mass_scale
task_c = {
    "stage": stage,
    "run_type": run_type,
    "group": group,
    "controller_mode": controller_mode,
    "task_b_mode": task_b_mode,
    "controller_model_id": "nominal_urdf_srbd_v1",
    "plant_model_id": plant_model_id,
    "nominal_mass_kg": nominal_mass,
    "true_plant_mass_scale": mass_scale,
    "true_mass_kg": true_mass,
    "payload_delta_kg": true_mass - nominal_mass,
    "experimental_independent_variables": [
        "true_plant_mass_scale", "enableParameterUpdate", "enableAdaptiveDynamics"],
    "task_c_diagnostics_enabled": True,
    "artificial_constant_wrench_enabled": False,
    "artificial_constant_wrench_world": [0.0] * 6,
    "analysis_window_start_mpc_time_s": 0.50,
    "analysis_window_duration_s": float(duration_text),
    "experiment_end_mpc_time_s": float(end_text),
    "bag_basename": "task_c_run.bag",
    "analysis_output_basename": "task_c_run",
    "scientific_scope": "Adaptive-OFF comparison acquisition; no conclusion before unified three-group analysis.",
}
expected_scale = 1.0 if group == "nominal" else 1.10
if not math.isclose(mass_scale, expected_scale, rel_tol=0.0, abs_tol=1.0e-12):
    raise RuntimeError("group/mass scale mismatch")
with open(task_c_path, "w") as stream:
    json.dump(task_c, stream, indent=2, sort_keys=True)
    stream.write("\n")

schedule = {
    "stage": stage,
    "target": {
        "axis": "x", "step_m": 0.10, "start_mpc_time_s": 0.50,
        "target_displacement_velocity_m_s": 0.5,
        "transition_duration_s": 0.20,
    },
    "artificial_disturbance": {
        "enabled": False, "frame": "world", "wrench": [0.0] * 6,
    },
    "plant_mass_override": {
        "nominal_mass_kg": nominal_mass,
        "true_mass_kg": true_mass,
        "payload_delta_kg": true_mass - nominal_mass,
        "mass_scale": mass_scale,
        "active_for_entire_run": True,
    },
    "analysis_window": {
        "time_axis": "MPC/simulation time", "start_s": 0.50,
        "duration_s": float(duration_text), "end_s": float(end_text),
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
  echo "an existing ROS master is running; refusing a non-isolated Task C run" >&2
  exit 2
fi

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

rosbag record --buffsize=512 -O "$output_dir/task_c_run.bag" \
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

timeout 900s python3 - "$experiment_end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys
import rospy
from std_msgs.msg import Float64MultiArray

end_time = float(sys.argv[1])
rospy.init_node("task_c_experiment_waiter", anonymous=True)

def callback(message):
    if len(message.data) == 41 and message.data[0] >= end_time:
        rospy.signal_shutdown("Task C fixed MPC horizon reached")

rospy.Subscriber("/task_c/model_mismatch_sample", Float64MultiArray,
                 callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros task_c_mass_experiment.launch \
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
  >"$output_dir/run_validation.log" 2>&1

python3 - "$output_dir" <<'PY'
import json
import os
import sys

run_dir = sys.argv[1]
with open(os.path.join(run_dir, "effective_config.json")) as stream:
    controller = json.load(stream)
with open(os.path.join(run_dir, "task_c_effective_config.json")) as stream:
    experiment = json.load(stream)
with open(os.path.join(run_dir, "task_c_run_summary.json")) as stream:
    summary = json.load(stream)
identity = summary["model_identity"]["value"]
controller_info = {
    "controller_model_id": identity["controller_model_id"],
    "controller_urdf": identity["controller_urdf"],
    "nominal_mass_kg": identity["nominal_mass"],
    "controller_mode": experiment["controller_mode"],
    "adaptive_config_sha256": controller["config_sha256"],
    "mode_neutral_config_sha256": controller["mode_neutral_config_sha256"],
    "fixed_files": controller["fixed_files"],
}
plant_info = {
    "plant_model_id": identity["plant_model_id"],
    "true_mass_kg": identity["true_mass"],
    "payload_delta_kg": identity["payload_delta"],
    "true_plant_mass_scale": identity["true_plant_mass_scale"],
    "override_active": identity["override_active"],
    "task_c_diagnostics_enabled": identity["task_c_diagnostics_enabled"],
}
for name, value in (("controller_model_info.json", controller_info),
                    ("plant_model_info.json", plant_info)):
    with open(os.path.join(run_dir, name), "w") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
PY

printf '%s\n' \
  "Task C acquisition and per-run validation complete." \
  "stage=$stage" \
  "group=$group" \
  "controller_mode=$controller_mode" \
  "task_b_mode=$task_b_mode" \
  "nominal_mass_kg=$nominal_mass" \
  "true_mass_kg=$true_mass" \
  "payload_delta_kg=$payload_delta" \
  "analysis_window_s=$analysis_window" \
  "bag=$output_dir/task_c_run.bag" \
  "Unified three-group metrics are deferred to the adaptive-OFF comparison analysis."
