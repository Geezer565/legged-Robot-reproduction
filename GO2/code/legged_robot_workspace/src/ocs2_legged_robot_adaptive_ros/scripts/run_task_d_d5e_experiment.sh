#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
mode="${1:-}"
output_dir="${2:-}"
run_index="${3:-}"
sequence="${4:-}"
protocol_json="${5:-}"
end_time="${6:-8.0}"
if [[ "$mode" != "on" && "$mode" != "off" ]] || [[ -z "$output_dir" || -z "$run_index" || -z "$sequence" || -z "$protocol_json" ]]; then
  echo "usage: $0 {on|off} OUTPUT_DIR RUN_INDEX SEQUENCE PROTOCOL_JSON [END_TIME]" >&2
  exit 2
fi
[[ -f "$protocol_json" ]] || { echo "missing D5E protocol: $protocol_json" >&2; exit 2; }
if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty D5E run: $output_dir" >&2
  exit 2
fi
mkdir -p "$output_dir"
exec >"$output_dir/terminal.log" 2>&1

protocol_dir="$(dirname "$protocol_json")"
cp "$protocol_dir/$mode.info" "$output_dir/effective_adaptive_settings.info"
python3 - "$protocol_json" "$protocol_dir/$mode.json" "$output_dir/effective_config.json" "$mode" "$run_index" "$sequence" "$end_time" <<'PY'
import hashlib, json, sys
from pathlib import Path
protocol_path, config_path, output_path = map(Path, sys.argv[1:4])
mode, run_index, sequence, end = sys.argv[4:]
protocol = json.loads(protocol_path.read_text())
config = json.loads(config_path.read_text())
if not protocol.get("passed") or mode not in protocol["groups"]:
    raise RuntimeError("D5E protocol has not passed")
if config["enableParameterUpdate"] != (mode == "on") or config["enableAdaptiveDynamics"] != (mode == "on"):
    raise RuntimeError("D5E mode flags do not match role")
if config["truePlantPayloadInertialEnabled"]:
    raise RuntimeError("D5E requires nominal plant")
config.update({"run_index": int(run_index), "acquisition_sequence": int(sequence),
               "acquisition_end_mpc_time_s": float(end),
               "d5e_protocol_sha256": hashlib.sha256(protocol_path.read_bytes()).hexdigest()})
output_path.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n")
PY

find "$workspace/src/ocs2_legged_robot_adaptive" \
     "$workspace/src/ocs2_legged_robot_adaptive_ros" \
  -type f ! -path '*/__pycache__/*' ! -name '*.pyc' -print0 \
  | sort -z | xargs -0 sha256sum >"$output_dir/source_files.sha256"

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

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
source "$workspace/devel/setup.bash"
if rosparam list >/dev/null 2>&1; then
  echo "existing ROS master detected; refusing non-isolated D5E acquisition" >&2
  exit 2
fi
roscore >"$output_dir/roscore.log" 2>&1 & core_pid=$!
for _ in $(seq 1 60); do
  rosparam list >/dev/null 2>&1 && break
  kill -0 "$core_pid" 2>/dev/null || exit 1
  sleep 1
done
rosparam list >/dev/null 2>&1 || { echo "roscore startup timeout" >&2; exit 1; }

rosbag record --buffsize=512 -O "$output_dir/task_d_d5e.bag" \
  /legged_robot_mpc_observation /legged_robot_mpc_target \
  /legged_robot_mpc_mode_schedule /task_c/locomotion_scheduler_status \
  /task_b/policy_input_sample /task_d/payload_inertial_10d_v3_sample \
  /task_d/payload_inertial_sigma_decomposition_sample \
  /task_d/equation10_full_flow_sample /task_d/model_identity \
  >"$output_dir/rosbag.log" 2>&1 & bag_pid=$!

timeout 1800s python3 - "$end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys, rospy
from ocs2_msgs.msg import mpc_observation
end_time = float(sys.argv[1])
rospy.init_node("task_d_d5e_waiter", anonymous=True)
def callback(message):
    if message.time >= end_time:
        rospy.signal_shutdown("fixed D5E horizon reached")
rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation, callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros task_d_payload_inertial_10d_experiment.launch \
  adaptiveSettingsFile:="$output_dir/effective_adaptive_settings.info" \
  payload_enabled:="false" experiment_condition:="walk_target" \
  >"$output_dir/roslaunch.log" 2>&1 & launch_pid=$!

wait "$waiter_pid"; waiter_pid=""
kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true; launch_pid=""
sleep 1
kill -INT "$bag_pid" 2>/dev/null || true; wait "$bag_pid" 2>/dev/null || true; bag_pid=""
kill -INT "$core_pid" 2>/dev/null || true; wait "$core_pid" 2>/dev/null || true; core_pid=""

[[ -s "$output_dir/task_d_d5e.bag" ]] || { echo "D5E bag missing/empty" >&2; exit 1; }
sha256sum "$output_dir/task_d_d5e.bag" "$output_dir/effective_config.json" \
  "$output_dir/effective_adaptive_settings.info" >"$output_dir/acquisition_checksums.sha256"
echo "D5E $mode/run_$run_index acquisition complete: $output_dir"
