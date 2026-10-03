#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
output_dir="${1:-$workspace/records/task_c_payload_mass_1d/smoke/run_01}"
end_time="${2:-3.0}"
true_plant_mass_scale="${3:-1.10}"
settings_file="$workspace/src/ocs2_legged_robot_adaptive/config/adaptive_settings_payload_mass_1d.info"

python3 - "$end_time" <<'PY'
import math, sys
value=float(sys.argv[1])
if not math.isfinite(value) or value <= 0.5 or value > 60.0:
    raise SystemExit("PayloadMass1D smoke end time must satisfy 0.5 < t <= 60 s")
PY
python3 - "$true_plant_mass_scale" <<'PY'
import math, sys
value=float(sys.argv[1])
if not math.isfinite(value) or value not in (1.0, 1.1):
    raise SystemExit("PayloadMass1D smoke mass scale must be exactly 1.0 or 1.1")
PY

if [[ -e "$output_dir" ]] && find "$output_dir" -mindepth 1 -print -quit | grep -q .; then
  echo "refusing to overwrite non-empty PayloadMass1D smoke directory: $output_dir" >&2
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

source "$workspace/devel/setup.bash"
if rosparam list >/dev/null 2>&1; then
  echo "an existing ROS master is running; refusing non-isolated smoke" >&2
  exit 2
fi

sha256sum "$settings_file" >"$output_dir/effective_settings.sha256"
cp "$settings_file" "$output_dir/adaptive_settings_payload_mass_1d.info"

roscore >"$output_dir/roscore.log" 2>&1 & core_pid=$!
for _ in $(seq 1 60); do
  rosparam list >/dev/null 2>&1 && break
  kill -0 "$core_pid" 2>/dev/null || exit 1
  sleep 1
done
rosparam list >/dev/null 2>&1 || { echo "roscore startup timeout" >&2; exit 1; }

rosbag record --buffsize=512 -O "$output_dir/payload_mass_smoke.bag" \
  /legged_robot_mpc_observation /legged_robot_mpc_target \
  /adaptive_update/sample /adaptive_update/payload_mass_sample \
  /task_b/equation10_sample /task_c/model_identity \
  /task_c/model_mismatch_sample \
  >"$output_dir/rosbag.log" 2>&1 & bag_pid=$!

timeout 900s python3 - "$end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys
import rospy
from std_msgs.msg import Float64MultiArray
end_time=float(sys.argv[1])
rospy.init_node("payload_mass_smoke_waiter", anonymous=True)
def callback(message):
    if len(message.data) == 41 and message.data[0] >= end_time:
        rospy.signal_shutdown("fixed smoke horizon reached")
rospy.Subscriber("/task_c/model_mismatch_sample", Float64MultiArray, callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros task_c_payload_mass_smoke.launch \
  adaptiveSettingsFile:="$settings_file" \
  true_plant_mass_scale:="$true_plant_mass_scale" \
  >"$output_dir/roslaunch.log" 2>&1 & launch_pid=$!

wait "$waiter_pid"; waiter_pid=""
kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true; launch_pid=""
sleep 1
kill -INT "$bag_pid" 2>/dev/null || true; wait "$bag_pid" 2>/dev/null || true; bag_pid=""
kill -INT "$core_pid" 2>/dev/null || true; wait "$core_pid" 2>/dev/null || true; core_pid=""

python3 "$package_dir/scripts/analyze_payload_mass_smoke.py" "$output_dir" \
  >"$output_dir/validation.log" 2>&1
cat "$output_dir/validation.log"
echo "PayloadMass1D isolated smoke PASS"
