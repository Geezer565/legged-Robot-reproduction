#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
core_package_dir="$workspace/src/ocs2_legged_robot_adaptive"
group="${1:-}"
case "$group" in
  nominal)
    true_plant_mass_scale="1.00"
    settings_file="$core_package_dir/config/adaptive_settings_payload_mass_1d.info"
    adaptive_enabled="true"
    ;;
  uncertain)
    true_plant_mass_scale="1.10"
    settings_file="$core_package_dir/config/adaptive_settings_payload_mass_1d.info"
    adaptive_enabled="true"
    ;;
  uncertain_off)
    true_plant_mass_scale="1.10"
    settings_file="$core_package_dir/config/adaptive_settings_payload_mass_1d_off.info"
    adaptive_enabled="false"
    ;;
  *)
    echo "usage: $0 {nominal|uncertain|uncertain_off} NEW_OUTPUT_DIRECTORY" >&2
    exit 2
    ;;
esac

output_dir="${2:-$workspace/records/task_c_locomotion_payload_mass_1d/$group/run_01}"
end_time="30.00"
target_start_time="1.20"
target_step="0.10"
nominal_mass="52.13485"

if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty locomotion payload directory: $output_dir" >&2
  exit 2
fi
mkdir -p "$output_dir"
exec >"$output_dir/terminal.log" 2>&1

core_pid=""; launch_pid=""; bag_pid=""; waiter_pid=""
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

mpc_task="$workspace/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/mpc/task.info"
reference="$workspace/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/command/reference.info"
gait="$workspace/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/command/gait.info"
urdf="$workspace/src/ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf"
for required in "$settings_file" "$mpc_task" "$reference" "$gait" "$urdf"; do
  [[ -f "$required" ]] || { echo "missing fixed experiment file: $required" >&2; exit 2; }
done

cp "$settings_file" "$output_dir/effective_adaptive_settings_payload_mass_1d.info"
find "$core_package_dir" "$package_dir" -type f \
  ! -path '*/__pycache__/*' ! -name '*.pyc' -print0 | sort -z | xargs -0 sha256sum \
  >"$output_dir/source_files.sha256"

python3 - "$output_dir" "$group" "$true_plant_mass_scale" "$adaptive_enabled" \
  "$settings_file" "$mpc_task" "$reference" "$gait" "$urdf" <<'PY'
import hashlib
import json
import math
import re
import sys
from pathlib import Path

out, group, scale_text, adaptive_text, settings, task, reference, gait, urdf = sys.argv[1:]
out = Path(out)
scale = float(scale_text)
adaptive_enabled = adaptive_text == "true"
nominal_mass = 52.13485
true_mass = nominal_mass * scale

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

text = Path(settings).read_text()
def scalar(name):
    match = re.search(r"^\s*" + re.escape(name) + r"\s+(\S+)", text, re.MULTILINE)
    if not match:
        raise RuntimeError("missing setting " + name)
    token = match.group(1).rstrip(";")
    if token in ("true", "false"):
        return token == "true"
    try:
        return float(token)
    except ValueError:
        return token

expected = {
    "enableParameterUpdate": adaptive_enabled,
    "enableAdaptiveDynamics": adaptive_enabled,
    "enableClfConstraint": True,
    "enableInputBiasCost": False,
    "useSoftConstraint": True,
    "softConstraintMu": 0.1,
    "softConstraintDelta": 5.0,
    "parameterization": "payload_mass_1d",
    "payloadMassAdaptationGain": 5.0,
    "minimumEstimatedTotalMass": 1.0,
}
actual = {name: scalar(name) for name in expected}
for name, value in expected.items():
    if isinstance(value, float):
        valid = math.isclose(float(actual[name]), value, rel_tol=0.0, abs_tol=1.0e-12)
    else:
        valid = actual[name] == value
    if not valid:
        raise RuntimeError("unexpected setting {}={!r}, expected {!r}".format(
            name, actual[name], value))

fixed_files = {name: {"path": path, "sha256": digest(path)} for name, path in {
    "adaptive_settings": settings, "mpc_task": task, "reference": reference,
    "gait": gait, "urdf": urdf,
}.items()}
(out / "payload_mass_controller_config.json").write_text(json.dumps({
    "schema_version": 1,
    "controller_model_id": "nominal_urdf_srbd_v1",
    "controller_nominal_mass_kg": nominal_mass,
    "adaptive_settings": actual,
    "lambdaLinear": [2.0, 2.0, 2.0],
    "lambdaAngular": [1.5, 1.5, 1.5],
    "kd_diagonal": [1.0] * 6,
    "initial_payload_mass_estimate_kg": 0.0,
    "fixed_files": fixed_files,
    "controller_config_sha256": digest(settings),
}, indent=2, sort_keys=True) + "\n")

(out / "payload_mass_experiment_config.json").write_text(json.dumps({
    "schema_version": 2,
    "stage": "PayloadMass1D-static-walk-formal-paired",
    "group": group,
    "experimental_independent_variables": [
        "true_plant_mass_scale", "enableParameterUpdate", "enableAdaptiveDynamics"],
    "true_plant_mass_scale": scale,
    "nominal_mass_kg": nominal_mass,
    "true_mass_kg": true_mass,
    "true_payload_delta_kg": true_mass - nominal_mass,
    "plant_model_id": "nominal_ocs2_rollout_v1" if group == "nominal" else "srbd_mass_scalar_v1",
    "controller_parameterization": "payload_mass_1d",
    "adaptive_enabled": adaptive_enabled,
    "artificial_constant_wrench_enabled": False,
    "analysis_window_start_mpc_time_s": 1.20,
    "analysis_window_end_mpc_time_s": 30.00,
    "scientific_scope": "formal repeated 10 cm static-walk payload-mismatch comparison",
}, indent=2, sort_keys=True) + "\n")

(out / "experiment_schedule.json").write_text(json.dumps({
    "target": {"axis": "x", "absolute_start_x_m": 0.0, "absolute_final_x_m": 0.10,
               "relative_step_m": 0.10, "start_mpc_time_s": 1.20,
               "target_displacement_velocity_m_s": 0.50, "transition_duration_s": 0.20},
    "gait": {"name": "static_walk", "mode_sequence": [13, 7, 14, 11],
             "switching_times_s": [0.0, 0.3, 0.6, 0.9, 1.2]},
    "analysis_window": {"start_s": 1.20, "end_s": 30.00, "common_grid_dt_s": 0.02},
    "artificial_disturbance": {"enabled": False, "wrench_world": [0.0] * 6},
    "plant_mass": {"nominal_mass_kg": nominal_mass, "true_mass_kg": true_mass,
                   "payload_delta_kg": true_mass - nominal_mass, "scale": scale,
                   "active_for_entire_run": True},
}, indent=2, sort_keys=True) + "\n")
PY

if git -C "$workspace" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  git -C "$workspace" rev-parse HEAD >"$output_dir/git_head.txt"
  git -C "$workspace" status --short >"$output_dir/git_status.txt"
else
  printf '%s\n' "workspace is not a Git work tree; source_files.sha256 records the code version" \
    >"$output_dir/git_unavailable.txt"
fi

source "$workspace/devel/setup.bash"
if rosparam list >/dev/null 2>&1; then
  echo "an existing ROS master is running; refusing non-isolated acquisition" >&2
  exit 2
fi
roscore >"$output_dir/roscore.log" 2>&1 & core_pid=$!
for _ in $(seq 1 60); do
  rosparam list >/dev/null 2>&1 && break
  kill -0 "$core_pid" 2>/dev/null || exit 1
  sleep 1
done
rosparam list >/dev/null 2>&1 || { echo "roscore startup timeout" >&2; exit 1; }

rosbag record --buffsize=512 -O "$output_dir/payload_mass_smoke.bag" \
  /legged_robot_mpc_observation /legged_robot_mpc_target \
  /legged_robot_mpc_mode_schedule /task_c/locomotion_scheduler_status \
  /legged_robot/currentState /adaptive_update/payload_mass_sample \
  /adaptive_update/sample /adaptive_update/disturbance_sample \
  /task_b/equation10_sample /task_b/policy_input_sample \
  /task_c/model_identity /task_c/model_mismatch_sample \
  /aclf_experiment/x_error /aclf_experiment/position_error_norm \
  /aclf_experiment/executed_sigma_norm \
  >"$output_dir/rosbag.log" 2>&1 & bag_pid=$!

timeout 1800s python3 - "$end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys
import rospy
from std_msgs.msg import Float64MultiArray
end_time = float(sys.argv[1])
rospy.init_node("locomotion_payload_waiter", anonymous=True)
def callback(message):
    if len(message.data) == 41 and message.data[0] >= end_time:
        rospy.signal_shutdown("fixed campaign horizon reached")
rospy.Subscriber("/task_c/model_mismatch_sample", Float64MultiArray, callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros task_c_locomotion_smoke.launch \
  adaptiveSettingsFile:="$settings_file" \
  true_plant_mass_scale:="$true_plant_mass_scale" \
  target_start_time:="$target_start_time" target_step:="$target_step" \
  >"$output_dir/roslaunch.log" 2>&1 & launch_pid=$!

wait "$waiter_pid"; waiter_pid=""
kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true; launch_pid=""
sleep 1
kill -INT "$bag_pid" 2>/dev/null || true; wait "$bag_pid" 2>/dev/null || true; bag_pid=""
kill -INT "$core_pid" 2>/dev/null || true; wait "$core_pid" 2>/dev/null || true; core_pid=""

python3 "$package_dir/scripts/analyze_payload_mass_smoke.py" "$output_dir" \
  >"$output_dir/run_validation.log" 2>&1
python3 "$package_dir/scripts/analyze_task_c_locomotion_smoke.py" "$output_dir" \
  --expected-target-start 1.20 --require-locomotion \
  >"$output_dir/locomotion_validation.log" 2>&1

python3 - "$output_dir" <<'PY'
import json
import sys
from pathlib import Path
run = Path(sys.argv[1])
summary = json.loads((run / "payload_mass_smoke_summary.json").read_text())
locomotion = json.loads((run / "locomotion_smoke_summary.json").read_text())
if not locomotion["locomotion_confirmed"]:
    raise RuntimeError("locomotion validation failed")
identity = summary["model_identity"]
controller = json.loads((run / "payload_mass_controller_config.json").read_text())
(run / "controller_model_info.json").write_text(json.dumps({
    "controller_model_id": identity["controller_model_id"],
    "controller_urdf": identity["controller_urdf"],
    "nominal_mass_kg": identity["nominal_mass"],
    "parameterization": "payload_mass_1d",
    "adaptive_config_sha256": controller["controller_config_sha256"],
}, indent=2, sort_keys=True) + "\n")
(run / "plant_model_info.json").write_text(json.dumps({
    "plant_model_id": identity["plant_model_id"],
    "true_mass_kg": identity["true_mass"],
    "payload_delta_kg": identity["payload_delta"],
    "true_plant_mass_scale": identity["true_plant_mass_scale"],
    "override_active": identity["override_active"],
}, indent=2, sort_keys=True) + "\n")
PY

sha256sum "$output_dir/payload_mass_smoke.bag" \
  "$output_dir/payload_mass_smoke.csv" "$output_dir/payload_mass_smoke_summary.json" \
  "$output_dir/payload_mass_smoke.png" "$output_dir/locomotion_smoke_summary.json" \
  "$output_dir/locomotion_smoke.png" "$output_dir/payload_mass_controller_config.json" \
  "$output_dir/payload_mass_experiment_config.json" >"$output_dir/artifact_checksums.sha256"

cat "$output_dir/run_validation.log"
cat "$output_dir/locomotion_validation.log"
echo "PayloadMass1D static-walk formal run PASS: group=$group output=$output_dir"
