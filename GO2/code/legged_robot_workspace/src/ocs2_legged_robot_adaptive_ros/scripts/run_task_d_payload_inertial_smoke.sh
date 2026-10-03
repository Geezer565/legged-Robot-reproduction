#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
output_dir="${1:-$workspace/records/task_d_payload_inertial_10d/smoke/run_01}"
end_time="${2:-3.0}"
settings="$workspace/src/ocs2_legged_robot_adaptive/config/adaptive_settings_payload_inertial_10d_smoke.info"
task="$workspace/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/mpc/task.info"
reference="$workspace/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/command/reference.info"
gait="$workspace/src/ocs2/ocs2_robotic_examples/ocs2_legged_robot/config/command/gait.info"
urdf="$workspace/src/ocs2_robotic_assets/resources/anymal_c/urdf/anymal.urdf"

if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty D2 smoke directory: $output_dir" >&2
  exit 2
fi
for required in "$settings" "$task" "$reference" "$gait" "$urdf"; do
  [[ -f "$required" ]] || { echo "missing D2 smoke input: $required" >&2; exit 2; }
done
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

cp "$settings" "$output_dir/effective_adaptive_settings.info"
sha256sum "$settings" "$task" "$reference" "$gait" "$urdf" \
  >"$output_dir/fixed_files.sha256"
find "$workspace/src/ocs2_legged_robot_adaptive" \
     "$workspace/src/ocs2_legged_robot_adaptive_ros" \
  -type f ! -path '*/__pycache__/*' ! -name '*.pyc' -print0 \
  | sort -z | xargs -0 sha256sum >"$output_dir/source_files.sha256"
git -C "$workspace/src" status --short \
  >"$output_dir/git_status.txt" 2>&1 || true
git -C "$workspace/src" diff -- ocs2_legged_robot_adaptive \
  ocs2_legged_robot_adaptive_ros >"$output_dir/source.diff" 2>&1 || true
python3 - "$output_dir" "$end_time" <<'PY'
import json
import sys
from pathlib import Path
out = Path(sys.argv[1])
config = {
    "schema_version": 2,
    "scope": "D2 short structural smoke; no formal campaign",
    "controller_parameterization": "payload_inertial_10d",
    "enable_parameter_update": True,
    "enable_adaptive_dynamics": True,
    "enable_clf_constraint": True,
    "gamma": "I10 untuned integration value",
    "true_plant_payload_inertial_enabled": True,
    "payload_mass_kg": 5.213485,
    "payload_com_from_nominal_com_base_m": [0.16, -0.03, 0.09],
    "payload_inertia_at_payload_com_base_kg_m2":
        [0.08, 0.0, 0.0, 0.10, 0.0, 0.12],
    "controller_urdf_unchanged": True,
    "artificial_disturbance_enabled": False,
    "gait": "static_walk",
    "target_relative_x_step_m": 0.10,
    "target_start_mpc_time_s": 1.20,
    "target_displacement_velocity_m_s": 0.50,
    "end_mpc_time_s": float(sys.argv[2]),
}
(out / "effective_experiment_config.json").write_text(
    json.dumps(config, indent=2, sort_keys=True) + "\n")
PY

source "$workspace/devel/setup.bash"
if rosparam list >/dev/null 2>&1; then
  echo "an existing ROS master is running; refusing non-isolated D2 smoke" >&2
  exit 2
fi
roscore >"$output_dir/roscore.log" 2>&1 & core_pid=$!
for _ in $(seq 1 60); do
  rosparam list >/dev/null 2>&1 && break
  kill -0 "$core_pid" 2>/dev/null || exit 1
  sleep 1
done
rosparam list >/dev/null 2>&1 || { echo "roscore startup timeout" >&2; exit 1; }

rosbag record --buffsize=512 -O "$output_dir/payload_inertial_smoke.bag" \
  /legged_robot_mpc_observation /legged_robot_mpc_target \
  /legged_robot_mpc_mode_schedule /task_c/locomotion_scheduler_status \
  /task_d/payload_inertial_10d_sample \
  /task_d/equation10_full_flow_sample \
  /task_d/payload_inertial_plant_sample /task_d/model_identity \
  /adaptive_update/sample /task_b/equation10_sample \
  /aclf_experiment/x_error /aclf_experiment/position_error_norm \
  >"$output_dir/rosbag.log" 2>&1 & bag_pid=$!

timeout 1200s python3 - "$end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys
import rospy
from ocs2_msgs.msg import mpc_observation
end_time = float(sys.argv[1])
rospy.init_node("task_d_payload_inertial_waiter", anonymous=True)
def callback(message):
    if message.time >= end_time:
        rospy.signal_shutdown("fixed D2 smoke horizon reached")
rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation,
                 callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros task_d_payload_inertial_10d_smoke.launch \
  >"$output_dir/roslaunch.log" 2>&1 & launch_pid=$!

wait "$waiter_pid"; waiter_pid=""
kill -INT "$launch_pid" 2>/dev/null || true
wait "$launch_pid" 2>/dev/null || true; launch_pid=""
sleep 1
kill -INT "$bag_pid" 2>/dev/null || true
wait "$bag_pid" 2>/dev/null || true; bag_pid=""
kill -INT "$core_pid" 2>/dev/null || true
wait "$core_pid" 2>/dev/null || true; core_pid=""

set +e
python3 "$package_dir/scripts/analyze_task_d_payload_inertial_smoke.py" \
  "$output_dir" >"$output_dir/analysis.log" 2>&1
analysis_status=$?
set -e
sha256sum "$output_dir/payload_inertial_smoke.bag" \
  "$output_dir/payload_inertial_smoke.csv" \
  "$output_dir/payload_inertial_smoke_summary.json" \
  "$output_dir/payload_inertial_smoke.png" \
  >"$output_dir/artifact_checksums.sha256" 2>/dev/null || true
cat "$output_dir/analysis.log"
if [[ "$analysis_status" -ne 0 ]]; then
  echo "D2 smoke structural audit FAILED; data preserved at $output_dir" >&2
  exit "$analysis_status"
fi
echo "D2 payload-inertial structural smoke PASS: $output_dir"
