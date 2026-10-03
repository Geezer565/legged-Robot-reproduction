#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
group="${1:-}"
case "$group" in
  nominal_raw|nominal_corrected)
    payload_enabled="false" ;;
  uncertain_raw|uncertain_corrected)
    payload_enabled="true" ;;
  *)
    echo "usage: $0 GROUP NEW_OUTPUT_DIR RUN_INDEX {smoke|formal} END_MPC_TIME [FROZEN_PROTOCOL ACQUISITION_SEQUENCE]" >&2
    exit 2 ;;
esac
output_dir="${2:-}"
run_index="${3:-}"
run_scope="${4:-}"
end_time="${5:-}"
formal_protocol="${6:-}"
acquisition_sequence="${7:-0}"
if [[ -z "$output_dir" || -z "$run_index" ||
      ( "$run_scope" != "smoke" && "$run_scope" != "formal" ) ||
      -z "$end_time" ]]; then
  echo "usage: $0 GROUP NEW_OUTPUT_DIR RUN_INDEX {smoke|formal} END_MPC_TIME [FROZEN_PROTOCOL ACQUISITION_SEQUENCE]" >&2
  exit 2
fi
if [[ "$run_scope" == "formal" ]] && \
   { [[ ! -f "$formal_protocol" ]] || [[ "$acquisition_sequence" -le 0 ]]; }; then
  echo "formal D5H1 run requires FROZEN_PROTOCOL and positive ACQUISITION_SEQUENCE" >&2
  exit 2
fi
if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty D5H1 run: $output_dir" >&2
  exit 2
fi
mkdir -p "$output_dir"
exec >"$output_dir/terminal.log" 2>&1

mode_tool="$package_dir/scripts/set_task_d_d5h1_mode.py"
base_settings="$workspace/src/ocs2_legged_robot_adaptive/config/adaptive_settings_payload_inertial_10d_smoke.info"
baseline="$package_dir/config/d5h0_nominal_gait_phase_baseline_v1.csv"
baseline_provenance="$package_dir/config/d5h1_phase_baseline_application_v1.json"
python3 "$mode_tool" --base-config "$base_settings" \
  --baseline-file "$baseline" emit "$group" \
  --output-config "$output_dir/effective_adaptive_settings.info" \
  --output-json "$output_dir/effective_config.json" \
  >"$output_dir/config_emission.log"
cp "$baseline" "$output_dir/frozen_phase_baseline.csv"
cp "$baseline_provenance" "$output_dir/phase_baseline_application_provenance.json"
python3 - "$output_dir/effective_config.json" "$group" "$run_index" \
  "$run_scope" "$end_time" "$formal_protocol" "$acquisition_sequence" <<'PY'
import hashlib, json, sys
from pathlib import Path
path, group, run_index, scope, end, protocol, sequence = sys.argv[1:]
path = Path(path)
config = json.loads(path.read_text())
if config["stage"] != "D5H1_estimator_phase_baseline" or config["group"] != group:
    raise RuntimeError("D5H1 effective config identity mismatch")
if config["parameterization"] != "payload_inertial_10d":
    raise RuntimeError("D5H1 requires PayloadInertial10D")
if not config["enableParameterUpdate"] or not config["enableAdaptiveDynamics"]:
    raise RuntimeError("D5H1 keeps the full adaptive path enabled")
config.update({
    "run_index": int(run_index),
    "run_scope": scope,
    "acquisition_end_mpc_time_s": float(end),
    "formal_eligibility": False if scope == "smoke" else True,
    "scheduler_condition": "walk_target",
    "expected_target_published": True,
    "expected_walking": True,
    "analysis_window_start_mpc_time_s": 0.0,
    "analysis_window_end_mpc_time_s": float(end),
    "common_grid_dt_s": 0.02,
    "diagnosticTopicStartToleranceS": 0.10,
    "topicEndToleranceS": 0.05,
    "acquisition_sequence": int(sequence) if scope == "formal" else None,
    "formalProtocolFile": str(Path(protocol).resolve()) if protocol else None,
    "formalProtocolSha256": (hashlib.sha256(Path(protocol).read_bytes()).hexdigest()
                              if protocol else None),
})
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
  echo "existing ROS master detected; refusing non-isolated D5H1 acquisition" >&2
  exit 2
fi
roscore >"$output_dir/roscore.log" 2>&1 & core_pid=$!
for _ in $(seq 1 60); do
  rosparam list >/dev/null 2>&1 && break
  kill -0 "$core_pid" 2>/dev/null || exit 1
  sleep 1
done
rosparam list >/dev/null 2>&1 || { echo "roscore startup timeout" >&2; exit 1; }

rosbag record --buffsize=512 -O "$output_dir/task_d_d5h1.bag" \
  /legged_robot_mpc_observation /legged_robot_mpc_target \
  /legged_robot_mpc_mode_schedule /task_c/locomotion_scheduler_status \
  /task_b/policy_input_sample /task_d/payload_inertial_10d_v3_sample \
  /task_d/d5h1_applied_update_sample \
  /task_d/d5h1_eq11_sample \
  /task_d/equation10_full_flow_sample /task_d/payload_inertial_plant_sample \
  /task_d/model_identity >"$output_dir/rosbag.log" 2>&1 & bag_pid=$!

timeout 1800s python3 - "$end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys, rospy
from ocs2_msgs.msg import mpc_observation
end_time = float(sys.argv[1])
rospy.init_node("task_d_d5h1_waiter", anonymous=True)
def callback(message):
    if message.time >= end_time:
        rospy.signal_shutdown("fixed D5H1 horizon reached")
rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation,
                 callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros task_d_payload_inertial_10d_experiment.launch \
  adaptiveSettingsFile:="$output_dir/effective_adaptive_settings.info" \
  payload_enabled:="$payload_enabled" experiment_condition:="walk_target" \
  d5h0_shadow_enabled:="false" \
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
python3 "$package_dir/scripts/analyze_task_d_d5h1_smoke.py" "$output_dir" \
  >"$output_dir/analysis.log" 2>&1
status=$?
if [[ "$status" -eq 0 ]]; then
  python3 "$package_dir/scripts/audit_task_d_d5h1_gate.py" "$output_dir" \
    >"$output_dir/gate_audit.log" 2>&1
  status=$?
fi
if [[ "$status" -eq 0 && "$run_scope" == "formal" ]]; then
  python3 "$package_dir/scripts/analyze_task_d_d5h1_formal_run.py" "$output_dir" \
    >"$output_dir/formal_analysis.log" 2>&1
  status=$?
fi
set -e
sha256sum "$output_dir/task_d_d5h1.bag" \
  "$output_dir/task_d_d5h1_timeseries.csv" \
  "$output_dir/task_d_d5h1_smoke_summary.json" \
  "$output_dir/task_d_d5h1_smoke.png" \
  "$output_dir/effective_config.json" \
  "$output_dir/effective_adaptive_settings.info" \
  "$output_dir/frozen_phase_baseline.csv" \
  "$output_dir/phase_baseline_application_provenance.json" \
  "$output_dir/d5h1_gate_audit_summary.json" \
  "$output_dir/d5h1_gate_audit_report.md" \
  "$output_dir/task_d_d5h1_formal_run.csv" \
  "$output_dir/task_d_d5h1_formal_run_summary.json" \
  "$output_dir/task_d_d5h1_formal_run.png" \
  >"$output_dir/artifact_checksums.sha256" 2>/dev/null || true
cat "$output_dir/analysis.log"
if [[ -f "$output_dir/gate_audit.log" ]]; then
  cat "$output_dir/gate_audit.log"
fi
if [[ -f "$output_dir/formal_analysis.log" ]]; then
  cat "$output_dir/formal_analysis.log"
fi
if [[ "$status" -ne 0 ]]; then
  echo "D5H1 $run_scope $group/run_$run_index FAILED; raw data preserved" >&2
  exit "$status"
fi
echo "D5H1 $run_scope $group/run_$run_index PASS: $output_dir"
