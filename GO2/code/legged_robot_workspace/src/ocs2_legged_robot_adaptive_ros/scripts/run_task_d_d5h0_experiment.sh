#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
group="${1:-}"
case "$group" in
  nominal)
    task_d_mode="nominal"; payload_enabled="false" ;;
  uncertain)
    task_d_mode="uncertain_adaptive"; payload_enabled="true" ;;
  *)
    echo "usage: $0 {nominal|uncertain} NEW_OUTPUT_DIR RUN_INDEX {smoke|formal} END_MPC_TIME" >&2
    exit 2 ;;
esac
output_dir="${2:-}"
run_index="${3:-}"
run_scope="${4:-}"
end_time="${5:-}"
if [[ -z "$output_dir" || -z "$run_index" ||
      ( "$run_scope" != "smoke" && "$run_scope" != "formal" ) ||
      -z "$end_time" ]]; then
  echo "usage: $0 {nominal|uncertain} NEW_OUTPUT_DIR RUN_INDEX {smoke|formal} END_MPC_TIME" >&2
  exit 2
fi
if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty D5H0 run: $output_dir" >&2
  exit 2
fi
mkdir -p "$output_dir"
exec >"$output_dir/terminal.log" 2>&1

mode_tool="$package_dir/scripts/set_task_d_mode.py"
base_settings="$workspace/src/ocs2_legged_robot_adaptive/config/adaptive_settings_payload_inertial_10d_smoke.info"
baseline="$package_dir/config/d5h0_nominal_gait_phase_baseline_v1.csv"
baseline_provenance="$package_dir/config/d5h0_nominal_gait_phase_baseline_v1.json"

python3 "$mode_tool" --base-config "$base_settings" emit "$task_d_mode" \
  --output-config "$output_dir/effective_adaptive_settings.info" \
  --output-json "$output_dir/effective_config.json" \
  >"$output_dir/config_emission.log"
python3 - "$output_dir/effective_config.json" "$group" "$task_d_mode" \
  "$payload_enabled" "$run_index" "$run_scope" "$end_time" "$baseline" \
  "$baseline_provenance" <<'PY'
import hashlib, json, sys
from pathlib import Path
(config_path, group, mode, payload, run_index, scope, end,
 baseline_path, provenance_path) = sys.argv[1:]
config_path = Path(config_path)
baseline_path, provenance_path = Path(baseline_path), Path(provenance_path)
config = json.loads(config_path.read_text())
if config["group"] != mode:
    raise RuntimeError("set_task_d_mode output identity mismatch")
if config["parameterization"] != "payload_inertial_10d":
    raise RuntimeError("D5H0 requires payload_inertial_10d")
if config["gamma"] != [[1.0 if row == column else 0.0 for column in range(10)]
                        for row in range(10)]:
    raise RuntimeError("D5H0 diagnostic Gamma contract requires I10")
baseline_sha = hashlib.sha256(baseline_path.read_bytes()).hexdigest()
provenance = json.loads(provenance_path.read_text())
if baseline_sha != provenance["packaged_canonical_lf_sha256"]:
    raise RuntimeError("packaged D5H0 baseline hash differs from audited canonical copy")
config.update({
    "schema_version": 6,
    "stage": "D5H0_shadow_diagnostics",
    "group": group,
    "source_task_d_mode": mode,
    "truePlantPayloadInertialEnabled": payload == "true",
    "run_index": int(run_index),
    "run_scope": scope,
    "acquisition_end_mpc_time_s": float(end),
    "scheduler_condition": "walk_target",
    "expected_target_published": True,
    "expected_walking": True,
    "d5h0": {
        "shadow_only": True,
        "actual_estimator_modified": False,
        "baseline_file": str(baseline_path.resolve()),
        "baseline_sha256": baseline_sha,
        "baseline_provenance_file": str(provenance_path.resolve()),
        "baseline_provenance_sha256": hashlib.sha256(
            provenance_path.read_bytes()).hexdigest(),
        "phase_bins": 24,
        "cycle_duration_s": 1.2,
        "minimum_cycle_start_time_s": 2.2,
        "actual_update": "Gamma*Y^T*sigma",
        "shadow_update": "Gamma*(Y^T*sigma-nominal_phase_baseline)",
    },
})
config_path.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n")
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
  echo "existing ROS master detected; refusing non-isolated D5H0 acquisition" >&2
  exit 2
fi
roscore >"$output_dir/roscore.log" 2>&1 & core_pid=$!
for _ in $(seq 1 60); do
  rosparam list >/dev/null 2>&1 && break
  kill -0 "$core_pid" 2>/dev/null || exit 1
  sleep 1
done
rosparam list >/dev/null 2>&1 || { echo "roscore startup timeout" >&2; exit 1; }

rosbag record --buffsize=512 -O "$output_dir/task_d_d5h0.bag" \
  /legged_robot_mpc_observation /legged_robot_mpc_target \
  /legged_robot_mpc_mode_schedule /task_c/locomotion_scheduler_status \
  /task_b/policy_input_sample /task_d/payload_inertial_10d_v3_sample \
  /task_d/equation10_full_flow_sample /task_d/payload_inertial_plant_sample \
  /task_d/model_identity /task_d/d5h0_shadow_update_sample \
  /task_d/d5h0_shadow_identity >"$output_dir/rosbag.log" 2>&1 & bag_pid=$!

timeout 1800s python3 - "$end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys, rospy
from ocs2_msgs.msg import mpc_observation
end_time = float(sys.argv[1])
rospy.init_node("task_d_d5h0_waiter", anonymous=True)
def callback(message):
    if message.time >= end_time:
        rospy.signal_shutdown("fixed D5H0 horizon reached")
rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation,
                 callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros task_d_payload_inertial_10d_experiment.launch \
  adaptiveSettingsFile:="$output_dir/effective_adaptive_settings.info" \
  payload_enabled:="$payload_enabled" experiment_condition:="walk_target" \
  d5h0_shadow_enabled:="true" d5h0_baseline_file:="$baseline" \
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
python3 "$package_dir/scripts/analyze_task_d_d5h0_shadow.py" "$output_dir" \
  >"$output_dir/analysis.log" 2>&1
status=$?
set -e
sha256sum "$output_dir/task_d_d5h0.bag" \
  "$output_dir/task_d_d5h0_timeseries.csv" \
  "$output_dir/task_d_d5h0_run_summary.json" \
  "$output_dir/task_d_d5h0_shadow.png" \
  "$output_dir/effective_config.json" \
  "$output_dir/effective_adaptive_settings.info" \
  >"$output_dir/artifact_checksums.sha256" 2>/dev/null || true
cat "$output_dir/analysis.log"
if [[ "$status" -ne 0 ]]; then
  echo "D5H0 $run_scope $group/run_$run_index FAILED; raw data preserved" >&2
  exit "$status"
fi
echo "D5H0 $run_scope $group/run_$run_index PASS: $output_dir"
