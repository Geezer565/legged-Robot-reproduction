#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
candidate="${1:-}"
case "$candidate" in
  E2_rotation|E3_combined) ;;
  *)
    echo "usage: $0 {E2_rotation|E3_combined} NEW_OUTPUT_DIR" >&2
    exit 2 ;;
esac
output_dir="${2:-}"
if [[ -z "$output_dir" ]]; then
  echo "usage: $0 {E2_rotation|E3_combined} NEW_OUTPUT_DIR" >&2
  exit 2
fi
if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty D5I2 run: $output_dir" >&2
  exit 2
fi
mkdir -p "$output_dir"
exec >"$output_dir/terminal.log" 2>&1

mode_tool="$package_dir/scripts/set_task_d_d5i2_mode.py"
base_settings="$workspace/src/ocs2_legged_robot_adaptive/config/adaptive_settings_payload_inertial_10d_smoke.info"
python3 "$mode_tool" --base-config "$base_settings" emit "$candidate" \
  --output-config "$output_dir/effective_adaptive_settings.info" \
  --output-json "$output_dir/effective_config.json" \
  >"$output_dir/config_emission.log"
python3 - "$output_dir/effective_config.json" "$output_dir" <<'PY'
import json, sys
from pathlib import Path
path, output = Path(sys.argv[1]), Path(sys.argv[2]).resolve()
config = json.loads(path.read_text())
config["run_directory"] = str(output)
config["acquisition_end_mpc_time_s"] = config["schedule"]["complete_time_s"]
path.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n")
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
  echo "existing ROS master detected; refusing non-isolated D5I2 acquisition" >&2
  exit 2
fi
roscore >"$output_dir/roscore.log" 2>&1 & core_pid=$!
for _ in $(seq 1 60); do
  rosparam list >/dev/null 2>&1 && break
  kill -0 "$core_pid" 2>/dev/null || exit 1
  sleep 1
done
rosparam list >/dev/null 2>&1 || { echo "roscore startup timeout" >&2; exit 1; }

rosbag record --buffsize=512 -O "$output_dir/task_d_d5i2.bag" \
  /legged_robot_mpc_observation /legged_robot_mpc_target \
  /legged_robot_mpc_mode_schedule /task_c/locomotion_scheduler_status \
  /task_b/policy_input_sample /task_d/payload_inertial_10d_v3_sample \
  /task_d/equation10_full_flow_sample /task_d/model_identity \
  /task_d/d5i1_reference_sample /task_d/d5i2_scheduler_identity \
  /task_d/d5i2_scheduler_sample \
  >"$output_dir/rosbag.log" 2>&1 & bag_pid=$!

end_time="20.4"
timeout 2400s python3 - "$end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys, rospy
from ocs2_msgs.msg import mpc_observation
end_time = float(sys.argv[1])
rospy.init_node("task_d_d5i2_waiter", anonymous=True)
def callback(message):
    if message.time >= end_time:
        rospy.signal_shutdown("fixed D5I2 horizon reached")
rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation,
                 callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros task_d_payload_inertial_10d_experiment.launch \
  adaptiveSettingsFile:="$output_dir/effective_adaptive_settings.info" \
  payload_enabled:="false" richer_reference_enabled:="true" \
  identification_candidate:="$candidate" identification_knot_dt:="0.01" \
  identification_profile_start_time:="2.4" \
  identification_ramp_in_duration:="2.4" \
  identification_steady_duration:="12.0" \
  identification_ramp_out_duration:="2.4" \
  identification_settle_duration:="1.2" \
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
python3 "$package_dir/scripts/analyze_task_d_d5i2_smoke.py" "$output_dir" \
  >"$output_dir/analysis.log" 2>&1
status=$?
set -e
sha256sum "$output_dir/task_d_d5i2.bag" \
  "$output_dir/task_d_d5i2_timeseries.csv" \
  "$output_dir/task_d_d5i2_summary.json" \
  "$output_dir/figure_d5i2_online_smoke.png" \
  "$output_dir/figure_d5i2_executed_spectrum.png" \
  "$output_dir/effective_config.json" \
  "$output_dir/effective_adaptive_settings.info" \
  >"$output_dir/artifact_checksums.sha256" 2>/dev/null || true
cat "$output_dir/analysis.log"
if [[ "$status" -ne 0 ]]; then
  echo "D5I2 $candidate acquisition FAILED; raw data preserved" >&2
  exit "$status"
fi
python3 - "$output_dir/task_d_d5i2_summary.json" <<'PY'
import json, sys
summary = json.load(open(sys.argv[1]))
state = "ADMITTED" if summary["candidate_admitted_for_next_stage"] else "NOT_ADMITTED"
print("D5I2 {} {}: {}".format(summary["candidate"], state, sys.argv[1]))
PY
