#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
group="${1:-}"
case "$group" in
  nominal) payload_enabled="false" ;;
  uncertain_off|uncertain_adaptive) payload_enabled="true" ;;
  *) echo "usage: $0 {nominal|uncertain_off|uncertain_adaptive} NEW_OUTPUT_DIR ROUND_INDEX ACQUISITION_SEQUENCE PROTOCOL_JSON" >&2; exit 2 ;;
esac
output_dir="${2:-}"
round_index="${3:-}"
acquisition_sequence="${4:-}"
protocol_json="${5:-}"
if [[ -z "$output_dir" || -z "$round_index" || -z "$acquisition_sequence" || -z "$protocol_json" ]]; then
  echo "usage: $0 {nominal|uncertain_off|uncertain_adaptive} NEW_OUTPUT_DIR ROUND_INDEX ACQUISITION_SEQUENCE PROTOCOL_JSON" >&2
  exit 2
fi
[[ -f "$protocol_json" ]] || { echo "missing D4 protocol: $protocol_json" >&2; exit 2; }
end_time="16.40"
window_start="1.20"
window_end="16.20"
grid_dt="0.02"
mode_tool="$package_dir/scripts/set_task_d_mode.py"
base_settings="$workspace/src/ocs2_legged_robot_adaptive/config/adaptive_settings_payload_inertial_10d_smoke.info"

if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty D4 run directory: $output_dir" >&2
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
python3 - "$output_dir/effective_config.json" "$protocol_json" \
  "$round_index" "$acquisition_sequence" "$end_time" "$window_start" \
  "$window_end" "$grid_dt" <<'PY'
import hashlib
import json
import sys
from pathlib import Path
config_path, protocol_path, round_index, sequence, end, start, stop, dt = sys.argv[1:]
config_path, protocol_path = Path(config_path), Path(protocol_path)
config = json.loads(config_path.read_text())
protocol = json.loads(protocol_path.read_text())
if not protocol.get("passed"):
    raise RuntimeError("D4 protocol has not passed")
config.update({
    "stage": "D4_formal_payload_inertial_10d",
    "run_scope": "formal only after round-01 full-horizon gate passes",
    "round_index": int(round_index),
    "acquisition_sequence": int(sequence),
    "acquisition_end_mpc_time_s": float(end),
    "analysis_window_start_mpc_time_s": float(start),
    "analysis_window_end_mpc_time_s": float(stop),
    "common_grid_dt_s": float(dt),
    "d4_protocol_sha256": hashlib.sha256(protocol_path.read_bytes()).hexdigest(),
})
config_path.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n")
PY

find "$workspace/src/ocs2_legged_robot_adaptive" \
     "$workspace/src/ocs2_legged_robot_adaptive_ros" \
  -type f ! -path '*/__pycache__/*' ! -name '*.pyc' -print0 \
  | sort -z | xargs -0 sha256sum >"$output_dir/source_files.sha256"
python3 - "$output_dir/environment.json" <<'PY'
import json
import platform
import sys
from pathlib import Path
Path(sys.argv[1]).write_text(json.dumps({
    "python": sys.version,
    "platform": platform.platform(),
    "ros_distribution": "noetic",
    "execution_environment": "distrobox ros1_noetic_gpu",
}, indent=2, sort_keys=True) + "\n")
PY

source "$workspace/devel/setup.bash"
if rosparam list >/dev/null 2>&1; then
  echo "existing ROS master detected; refusing non-isolated D4 acquisition" >&2
  exit 2
fi
roscore >"$output_dir/roscore.log" 2>&1 & core_pid=$!
for _ in $(seq 1 60); do
  rosparam list >/dev/null 2>&1 && break
  kill -0 "$core_pid" 2>/dev/null || exit 1
  sleep 1
done
rosparam list >/dev/null 2>&1 || { echo "roscore startup timeout" >&2; exit 1; }

rosbag record --buffsize=512 -O "$output_dir/task_d_d4.bag" \
  /legged_robot_mpc_observation /legged_robot_mpc_target \
  /legged_robot_mpc_mode_schedule /task_c/locomotion_scheduler_status \
  /task_b/policy_input_sample /task_d/payload_inertial_10d_v3_sample \
  /task_d/equation10_full_flow_sample \
  /task_d/payload_inertial_plant_sample /task_d/model_identity \
  /adaptive_update/sample /task_b/equation10_sample \
  >"$output_dir/rosbag.log" 2>&1 & bag_pid=$!

timeout 1800s python3 - "$end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys
import rospy
from ocs2_msgs.msg import mpc_observation
end_time = float(sys.argv[1])
rospy.init_node("task_d_d4_waiter", anonymous=True)
def callback(message):
    if message.time >= end_time:
        rospy.signal_shutdown("fixed D4 acquisition horizon reached")
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
python3 "$package_dir/scripts/analyze_task_d_d4_run.py" "$output_dir" \
  >"$output_dir/analysis.log" 2>&1
status=$?
set -e
sha256sum "$output_dir/task_d_d4.bag" "$output_dir/task_d_d4_run.csv" \
  "$output_dir/task_d_d4_run_summary.json" "$output_dir/task_d_d4_overview.png" \
  "$output_dir/effective_config.json" "$output_dir/effective_adaptive_settings.info" \
  >"$output_dir/artifact_checksums.sha256" 2>/dev/null || true
cat "$output_dir/analysis.log"
if [[ "$status" -ne 0 ]]; then
  echo "D4 ${group}/run_${round_index} FAILED; raw data preserved at $output_dir" >&2
  exit "$status"
fi
echo "D4 ${group}/run_${round_index} PASS: $output_dir"
