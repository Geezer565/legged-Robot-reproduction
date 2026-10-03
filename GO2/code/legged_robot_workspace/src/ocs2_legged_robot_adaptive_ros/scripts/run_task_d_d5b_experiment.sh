#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
group="${1:-}"
case "$group" in
  nominal_stance_hold)
    source_mode="nominal"; condition="stance_hold"; payload_enabled="false"; expected_target="false"; expected_walk="false" ;;
  nominal_stance_target)
    source_mode="nominal"; condition="stance_target"; payload_enabled="false"; expected_target="true"; expected_walk="false" ;;
  nominal_walk_target)
    source_mode="nominal"; condition="walk_target"; payload_enabled="false"; expected_target="true"; expected_walk="true" ;;
  uncertain_walk_target)
    source_mode="uncertain_adaptive"; condition="walk_target"; payload_enabled="true"; expected_target="true"; expected_walk="true" ;;
  *) echo "usage: $0 GROUP NEW_OUTPUT_DIR RUN_INDEX SEQUENCE PROTOCOL_JSON {smoke|formal} END_TIME" >&2; exit 2 ;;
esac
output_dir="${2:-}"
run_index="${3:-}"
sequence="${4:-}"
protocol_json="${5:-}"
run_scope="${6:-}"
end_time="${7:-}"
if [[ -z "$output_dir" || -z "$run_index" || -z "$sequence" ||
      -z "$protocol_json" || ( "$run_scope" != "smoke" && "$run_scope" != "formal" ) ||
      -z "$end_time" ]]; then
  echo "usage: $0 GROUP NEW_OUTPUT_DIR RUN_INDEX SEQUENCE PROTOCOL_JSON {smoke|formal} END_TIME" >&2
  exit 2
fi
[[ -f "$protocol_json" ]] || { echo "missing D5B protocol: $protocol_json" >&2; exit 2; }
if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty D5B run: $output_dir" >&2
  exit 2
fi
mkdir -p "$output_dir"
exec >"$output_dir/terminal.log" 2>&1

analysis_start="1.20"
analysis_end="$(python3 - "$end_time" <<'PY'
import sys
print("{:.2f}".format(float(sys.argv[1]) - 0.2))
PY
)"
grid_dt="0.02"
mode_tool="$package_dir/scripts/set_task_d_mode.py"
base_settings="$workspace/src/ocs2_legged_robot_adaptive/config/adaptive_settings_payload_inertial_10d_smoke.info"

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

python3 "$mode_tool" --base-config "$base_settings" emit "$source_mode" \
  --output-config "$output_dir/effective_adaptive_settings.info" \
  --output-json "$output_dir/effective_config.json" \
  >"$output_dir/config_emission.log"
python3 - "$output_dir/effective_config.json" "$protocol_json" "$group" \
  "$source_mode" "$condition" "$payload_enabled" "$expected_target" \
  "$expected_walk" "$run_index" "$sequence" "$run_scope" "$end_time" \
  "$analysis_start" "$analysis_end" "$grid_dt" <<'PY'
import hashlib, json, sys
from pathlib import Path
(config_path, protocol_path, group, source_mode, condition, payload, target,
 walk, run_index, sequence, scope, end, start, stop, dt) = sys.argv[1:]
config_path, protocol_path = Path(config_path), Path(protocol_path)
config = json.loads(config_path.read_text())
protocol = json.loads(protocol_path.read_text())
if not protocol.get("passed") or group not in protocol["groups"]:
    raise RuntimeError("D5B protocol/role has not passed")
role = protocol["groups"][group]
expected = (role["source_mode"], role["scheduler_condition"], role["payload"],
            role["target"], role["walking"])
actual = (source_mode, condition, payload == "true", target == "true", walk == "true")
if actual != expected:
    raise RuntimeError("runner role differs from D5B protocol")
config.update({
    "schema_version": 5, "stage": "D5B_nominal_bias_attribution",
    "group": group, "source_task_d_mode": source_mode,
    "scheduler_condition": condition,
    "truePlantPayloadInertialEnabled": payload == "true",
    "expected_target_published": target == "true",
    "expected_walking": walk == "true", "run_index": int(run_index),
    "acquisition_sequence": int(sequence), "run_scope": scope,
    "acquisition_end_mpc_time_s": float(end),
    "analysis_window_start_mpc_time_s": float(start),
    "analysis_window_end_mpc_time_s": float(stop),
    "common_grid_dt_s": float(dt),
    "d5b_protocol_sha256": hashlib.sha256(protocol_path.read_bytes()).hexdigest(),
})
config_path.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n")
PY

find "$workspace/src/ocs2_legged_robot_adaptive" \
     "$workspace/src/ocs2_legged_robot_adaptive_ros" \
  -type f ! -path '*/__pycache__/*' ! -name '*.pyc' -print0 \
  | sort -z | xargs -0 sha256sum >"$output_dir/source_files.sha256"

# The Noetic setup hook reads ROS_MASTER_URI while this audited runner uses
# nounset. Provide the standard isolated-local-master default explicitly so a
# clean shell cannot fail before the runner performs its own master check.
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
source "$workspace/devel/setup.bash"
if rosparam list >/dev/null 2>&1; then
  echo "existing ROS master detected; refusing non-isolated D5B acquisition" >&2
  exit 2
fi
roscore >"$output_dir/roscore.log" 2>&1 & core_pid=$!
for _ in $(seq 1 60); do
  rosparam list >/dev/null 2>&1 && break
  kill -0 "$core_pid" 2>/dev/null || exit 1
  sleep 1
done
rosparam list >/dev/null 2>&1 || { echo "roscore startup timeout" >&2; exit 1; }

rosbag record --buffsize=512 -O "$output_dir/task_d_d5b.bag" \
  /legged_robot_mpc_observation /legged_robot_mpc_target \
  /legged_robot_mpc_mode_schedule /task_c/locomotion_scheduler_status \
  /task_b/policy_input_sample /task_d/payload_inertial_10d_v3_sample \
  /task_d/payload_inertial_sigma_decomposition_sample \
  /task_d/equation10_full_flow_sample /task_d/payload_inertial_plant_sample \
  /task_d/model_identity >"$output_dir/rosbag.log" 2>&1 & bag_pid=$!

timeout 1800s python3 - "$end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys, rospy
from ocs2_msgs.msg import mpc_observation
end_time = float(sys.argv[1])
rospy.init_node("task_d_d5b_waiter", anonymous=True)
def callback(message):
    if message.time >= end_time:
        rospy.signal_shutdown("fixed D5B horizon reached")
rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation,
                 callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros task_d_payload_inertial_10d_experiment.launch \
  adaptiveSettingsFile:="$output_dir/effective_adaptive_settings.info" \
  payload_enabled:="$payload_enabled" experiment_condition:="$condition" \
  >"$output_dir/roslaunch.log" 2>&1 & launch_pid=$!

wait "$waiter_pid"; waiter_pid=""
kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true; launch_pid=""
sleep 1
kill -INT "$bag_pid" 2>/dev/null || true; wait "$bag_pid" 2>/dev/null || true; bag_pid=""
kill -INT "$core_pid" 2>/dev/null || true; wait "$core_pid" 2>/dev/null || true; core_pid=""

set +e
python3 "$package_dir/scripts/analyze_task_d_d5b_run.py" "$output_dir" \
  >"$output_dir/analysis.log" 2>&1
status=$?
set -e
sha256sum "$output_dir/task_d_d5b.bag" "$output_dir/task_d_d5b_run.csv" \
  "$output_dir/task_d_d5b_run_summary.json" "$output_dir/task_d_d5b_overview.png" \
  "$output_dir/effective_config.json" "$output_dir/effective_adaptive_settings.info" \
  >"$output_dir/artifact_checksums.sha256" 2>/dev/null || true
cat "$output_dir/analysis.log"
if [[ "$status" -ne 0 ]]; then
  echo "D5B $run_scope $group/run_$run_index FAILED; raw data preserved" >&2
  exit "$status"
fi
echo "D5B $run_scope $group/run_$run_index PASS: $output_dir"
