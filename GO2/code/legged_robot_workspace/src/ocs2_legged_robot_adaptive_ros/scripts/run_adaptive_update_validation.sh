#!/usr/bin/env bash
set -eo pipefail

package_dir="$(cd "$(dirname "$0")/.." && pwd)"
workspace="$(cd "$package_dir/../.." && pwd)"
output_dir="${1:-$workspace/records/adaptive_update_validation}"
end_time="${2:-6.0}"
mkdir -p "$output_dir"
source "$workspace/devel/setup.bash"
set -u

core_pid=""
launch_pid=""
bag_pid=""
waiter_pid=""
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

roscore >"$output_dir/roscore.log" 2>&1 &
core_pid=$!
for _ in $(seq 1 60); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  if ! kill -0 "$core_pid" 2>/dev/null; then
    echo "roscore stopped during startup" >&2
    exit 1
  fi
  sleep 1
done
if ! rosparam list >/dev/null 2>&1; then
  echo "roscore did not start within 60 seconds" >&2
  exit 1
fi

rosbag record --buffsize=256 -O "$output_dir/adaptive_update.bag" \
  /adaptive_update/sample /adaptive_update/disturbance_sample \
  >"$output_dir/rosbag.log" 2>&1 &
bag_pid=$!

timeout 300s python3 - "$end_time" >"$output_dir/waiter.log" 2>&1 <<'PY' &
import sys
import rospy
from std_msgs.msg import Float64MultiArray

end_time = float(sys.argv[1])
rospy.init_node("adaptive_update_validation_waiter", anonymous=True)

def callback(message):
    if len(message.data) == 8 and message.data[0] >= end_time:
        rospy.signal_shutdown("requested observation horizon reached")

rospy.Subscriber("/adaptive_update/disturbance_sample", Float64MultiArray,
                 callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros adaptive_update_validation.launch \
  >"$output_dir/roslaunch.log" 2>&1 &
launch_pid=$!

wait "$waiter_pid"
waiter_pid=""
kill -INT "$launch_pid" 2>/dev/null || true
wait "$launch_pid" 2>/dev/null || true
launch_pid=""
sleep 1
kill -INT "$bag_pid" 2>/dev/null || true
wait "$bag_pid" 2>/dev/null || true
bag_pid=""
kill -INT "$core_pid" 2>/dev/null || true
wait "$core_pid" 2>/dev/null || true
core_pid=""

python3 "$package_dir/scripts/analyze_adaptive_update.py" \
  "$output_dir/adaptive_update.bag" --output-dir "$output_dir"