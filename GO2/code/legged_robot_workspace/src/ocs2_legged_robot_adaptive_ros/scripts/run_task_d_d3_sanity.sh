#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
group="${1:-}"
case "$group" in
  nominal) payload_enabled="false" ;;
  uncertain_off|uncertain_adaptive) payload_enabled="true" ;;
  *) echo "usage: $0 {nominal|uncertain_off|uncertain_adaptive} [NEW_OUTPUT_DIR] [END_TIME]" >&2; exit 2 ;;
esac
output_dir="${2:-$workspace/records/task_d_payload_inertial_10d/stage_d3_gate/sanity/$group/run_01}"
end_time="${3:-3.0}"
mode_tool="$package_dir/scripts/set_task_d_mode.py"
base_settings="$workspace/src/ocs2_legged_robot_adaptive/config/adaptive_settings_payload_inertial_10d_smoke.info"

if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty D3 sanity directory: $output_dir" >&2
  exit 2
fi
mkdir -p "$output_dir"
exec >"$output_dir/terminal.log" 2>&1

core_pid=""; launch_pid=""; bag_pid=""; waiter_pid=""
cleanup() {
  for name in waiter_pid launch_pid bag_pid core_pid; do
    pid="${!name}"
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

python3 "$mode_tool" --base-config "$base_settings" emit "$group" \
  --output-config "$output_dir/effective_adaptive_settings.info" \
  --output-json "$output_dir/effective_config.json" \
  >"$output_dir/config_emission.log"
python3 - "$output_dir/effective_config.json" "$end_time" <<'PY'
import json
import sys
from pathlib import Path
path = Path(sys.argv[1])
value = json.loads(path.read_text())
value["run_scope"] = "D3 short single-group sanity; no formal inference"
value["end_mpc_time_s"] = float(sys.argv[2])
path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
PY
find "$workspace/src/ocs2_legged_robot_adaptive" \
     "$workspace/src/ocs2_legged_robot_adaptive_ros" \
  -type f ! -path '*/__pycache__/*' ! -name '*.pyc' -print0 \
  | sort -z | xargs -0 sha256sum >"$output_dir/source_files.sha256"

source "$workspace/devel/setup.bash"
if rosparam list >/dev/null 2>&1; then
  echo "existing ROS master detected; refusing non-isolated D3 sanity" >&2
  exit 2
fi
roscore >"$output_dir/roscore.log" 2>&1 & core_pid=$!
for _ in $(seq 1 60); do
  rosparam list >/dev/null 2>&1 && break
  kill -0 "$core_pid" 2>/dev/null || exit 1
  sleep 1
done
rosparam list >/dev/null 2>&1 || { echo "roscore startup timeout" >&2; exit 1; }

rosbag record --buffsize=512 -O "$output_dir/d3_sanity.bag" \
  /legged_robot_mpc_observation /legged_robot_mpc_target \
  /legged_robot_mpc_mode_schedule /task_c/locomotion_scheduler_status \
  /task_b/policy_input_sample /task_d/payload_inertial_10d_v3_sample \
  /task_d/equation10_full_flow_sample \
  /task_d/payload_inertial_plant_sample /task_d/model_identity \
  /adaptive_update/sample /task_b/equation10_sample \
  >"$output_dir/rosbag.log" 2>&1 & bag_pid=$!

timeout 1200s python3 - "$end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys
import rospy
from ocs2_msgs.msg import mpc_observation
end_time = float(sys.argv[1])
rospy.init_node("task_d_d3_sanity_waiter", anonymous=True)
def callback(message):
    if message.time >= end_time:
        rospy.signal_shutdown("fixed D3 sanity horizon reached")
rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation,
                 callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros task_d_payload_inertial_10d_experiment.launch \
  adaptiveSettingsFile:="$output_dir/effective_adaptive_settings.info" \
  payload_enabled:="$payload_enabled" \
  >"$output_dir/roslaunch.log" 2>&1 & launch_pid=$!

wait "$waiter_pid"; waiter_pid=""
kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true; launch_pid=""
sleep 1
kill -INT "$bag_pid" 2>/dev/null || true; wait "$bag_pid" 2>/dev/null || true; bag_pid=""
kill -INT "$core_pid" 2>/dev/null || true; wait "$core_pid" 2>/dev/null || true; core_pid=""

set +e
python3 "$package_dir/scripts/analyze_task_d_d3_sanity.py" "$output_dir" \
  >"$output_dir/analysis.log" 2>&1
status=$?
set -e
sha256sum "$output_dir/d3_sanity.bag" "$output_dir/d3_sanity.csv" \
  "$output_dir/d3_sanity_summary.json" "$output_dir/d3_sanity.png" \
  >"$output_dir/artifact_checksums.sha256" 2>/dev/null || true
cat "$output_dir/analysis.log"
if [[ "$status" -ne 0 ]]; then
  echo "D3 $group sanity FAILED; data preserved at $output_dir" >&2
  exit "$status"
fi
echo "D3 $group sanity PASS: $output_dir"
