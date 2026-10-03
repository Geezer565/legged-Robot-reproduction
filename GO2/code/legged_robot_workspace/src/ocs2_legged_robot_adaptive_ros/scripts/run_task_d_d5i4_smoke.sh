#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
group="${1:-}"
case "$group" in
  nominal_original|nominal_phase_corrected)
    payload_enabled="false" ;;
  payload_original|payload_phase_corrected)
    payload_enabled="true" ;;
  *)
    echo "usage: $0 GROUP NEW_OUTPUT_DIR" >&2
    exit 2 ;;
esac
output_dir="${2:-}"
if [[ -z "$output_dir" ]]; then
  echo "usage: $0 GROUP NEW_OUTPUT_DIR" >&2
  exit 2
fi
if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty D5I4 run: $output_dir" >&2
  exit 2
fi
mkdir -p "$output_dir"
exec >"$output_dir/terminal.log" 2>&1

mode_tool="$package_dir/scripts/set_task_d_d5i4_mode.py"
base_settings="$workspace/src/ocs2_legged_robot_adaptive/config/adaptive_settings_payload_inertial_10d_smoke.info"
protocol="$package_dir/config/d5i4_online_smoke_protocol_v1.json"
baseline="$workspace/records/task_d_payload_inertial_10d/stage_d5i3_nominal_baseline_calibration/analysis/nominal_e3_composite_phase_baseline_v1.csv"
provenance="$workspace/records/task_d_payload_inertial_10d/stage_d5i3_nominal_baseline_calibration/analysis/nominal_e3_composite_phase_baseline_v1.json"
python3 "$mode_tool" --base-config "$base_settings" \
  --baseline-file "$baseline" --provenance-file "$provenance" \
  emit "$group" \
  --output-config "$output_dir/effective_adaptive_settings.info" \
  --output-json "$output_dir/effective_config.json" \
  >"$output_dir/config_emission.log"
cp "$protocol" "$output_dir/frozen_d5i4_protocol.json"
cp "$baseline" "$output_dir/frozen_composite_baseline.csv"
cp "$provenance" "$output_dir/frozen_composite_baseline_provenance.json"
python3 - "$output_dir/effective_config.json" "$output_dir" <<'PY'
import json, sys
from pathlib import Path
path, output = Path(sys.argv[1]), Path(sys.argv[2]).resolve()
config = json.loads(path.read_text())
config["run_directory"] = str(output)
config["acquisition_end_mpc_time_s"] = 32.4
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
  echo "existing ROS master detected; refusing non-isolated D5I4 acquisition" >&2
  exit 2
fi
roscore >"$output_dir/roscore.log" 2>&1 & core_pid=$!
for _ in $(seq 1 60); do
  rosparam list >/dev/null 2>&1 && break
  kill -0 "$core_pid" 2>/dev/null || exit 1
  sleep 1
done
rosparam list >/dev/null 2>&1 || { echo "roscore startup timeout" >&2; exit 1; }

rosbag record --buffsize=512 -O "$output_dir/task_d_d5i4.bag" \
  /legged_robot_mpc_observation /legged_robot_mpc_target \
  /legged_robot_mpc_mode_schedule /task_c/locomotion_scheduler_status \
  /task_b/policy_input_sample /task_d/payload_inertial_10d_v3_sample \
  /task_d/d5i4_composite_update_sample /task_d/d5h1_eq11_sample \
  /task_d/equation10_full_flow_sample /task_d/payload_inertial_plant_sample \
  /task_d/model_identity /task_d/d5i1_reference_sample \
  /task_d/d5i2_scheduler_identity /task_d/d5i2_scheduler_sample \
  >"$output_dir/rosbag.log" 2>&1 & bag_pid=$!

timeout 3600s python3 - 32.4 >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys, time, rospy
from ocs2_msgs.msg import mpc_observation
end_time = float(sys.argv[1])
rospy.init_node("task_d_d5i4_waiter", anonymous=True)
state = {"seen": False, "complete": False, "last_wall": time.monotonic()}
def callback(message):
    state["seen"] = True
    state["last_wall"] = time.monotonic()
    if message.time >= end_time:
        state["complete"] = True
rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation,
                 callback, queue_size=10)
while not rospy.is_shutdown():
    if state["complete"]:
        raise SystemExit(0)
    if state["seen"] and time.monotonic() - state["last_wall"] > 30.0:
        print("no MPC observation for 30 wall seconds; treating solver as failed",
              file=sys.stderr)
        raise SystemExit(3)
    time.sleep(0.2)
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros task_d_payload_inertial_10d_experiment.launch \
  adaptiveSettingsFile:="$output_dir/effective_adaptive_settings.info" \
  payload_enabled:="$payload_enabled" richer_reference_enabled:="true" \
  identification_candidate:="E3_combined" identification_knot_dt:="0.01" \
  identification_profile_start_time:="2.4" \
  identification_ramp_in_duration:="2.4" \
  identification_steady_duration:="24.0" \
  identification_ramp_out_duration:="2.4" \
  identification_settle_duration:="1.2" \
  >"$output_dir/roslaunch.log" 2>&1 & launch_pid=$!

set +e
wait "$waiter_pid"
waiter_status=$?
set -e
waiter_pid=""
early_launch_exit=0
if [[ "$waiter_status" -ne 0 ]]; then early_launch_exit=1; fi
kill -INT "$launch_pid" 2>/dev/null || true
wait "$launch_pid" 2>/dev/null || true; launch_pid=""
sleep 1
kill -INT "$bag_pid" 2>/dev/null || true
wait "$bag_pid" 2>/dev/null || true; bag_pid=""
kill -INT "$core_pid" 2>/dev/null || true
wait "$core_pid" 2>/dev/null || true; core_pid=""

set +e
python3 "$package_dir/scripts/analyze_task_d_d5i4_smoke.py" "$output_dir" \
  >"$output_dir/analysis.log" 2>&1
status=$?
if [[ "$early_launch_exit" -ne 0 ]]; then
  status=3
  echo "D5I4 launch exited before the fixed observation horizon" \
    >>"$output_dir/analysis.log"
fi
set -e
sha256sum "$output_dir/task_d_d5i4.bag" \
  "$output_dir/task_d_d5i4_timeseries.csv" \
  "$output_dir/task_d_d5i4_summary.json" \
  "$output_dir/figure_d5i4_online_smoke.png" \
  "$output_dir/stage_d5i4_smoke_report.md" \
  "$output_dir/effective_config.json" \
  "$output_dir/effective_adaptive_settings.info" \
  "$output_dir/frozen_composite_baseline.csv" \
  "$output_dir/frozen_composite_baseline_provenance.json" \
  >"$output_dir/artifact_checksums.sha256" 2>/dev/null || true
cat "$output_dir/analysis.log"
if [[ "$status" -ne 0 ]]; then
  echo "D5I4 smoke $group FAILED; raw data preserved" >&2
  exit "$status"
fi
echo "D5I4 smoke $group PASS: $output_dir"
