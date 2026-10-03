#!/usr/bin/env bash
set -eo pipefail

if [ "$#" -lt 3 ] || [ "$#" -gt 25 ]; then
  echo "usage: $0 OUTPUT_DIR ADAPTIVE_SETTINGS_FILE ROS_PORT [END_TIME [DISTURBANCE_START DISTURBANCE_END FX FY FZ TX TY TZ TRUE_PLANT_MASS_SCALE PROFILE FREQUENCY_HZ SECONDARY_FX SECONDARY_FY SECONDARY_FZ SECONDARY_TX SECONDARY_TY SECONDARY_TZ LOCOMOTION_MODE WALKING_TARGET_STEP WALKING_TARGET_START WALKING_TARGET_DURATION]]" >&2
  exit 64
fi

output_dir="$1"
settings_file="$2"
ros_port="$3"
end_time="${4:-8.0}"
disturbance_start="${5:-2.0}"
disturbance_end="${6:-6.0}"
disturbance_fx="${7:-20.0}"
disturbance_fy="${8:-0.0}"
disturbance_fz="${9:-0.0}"
disturbance_tx="${10:-0.0}"
disturbance_ty="${11:-0.0}"
disturbance_tz="${12:-0.0}"
true_plant_mass_scale="${13:-1.0}"
disturbance_profile="${14:-constant}"
disturbance_frequency_hz="${15:-0.25}"
disturbance_secondary_fx="${16:-0.0}"
disturbance_secondary_fy="${17:-0.0}"
disturbance_secondary_fz="${18:-0.0}"
disturbance_secondary_tx="${19:-0.0}"
disturbance_secondary_ty="${20:-0.0}"
disturbance_secondary_tz="${21:-0.0}"
locomotion_mode="${22:-stance}"
walking_target_step="${23:-0.60}"
walking_target_start="${24:-1.20}"
walking_target_duration="${25:-12.0}"
workspace="/workspace"

python3 - "$end_time" "$disturbance_start" "$disturbance_end" \
  "$disturbance_fx" "$disturbance_fy" "$disturbance_fz" \
  "$disturbance_tx" "$disturbance_ty" "$disturbance_tz" \
  "$true_plant_mass_scale" "$disturbance_profile" \
  "$disturbance_frequency_hz" "$disturbance_secondary_fx" \
  "$disturbance_secondary_fy" "$disturbance_secondary_fz" \
  "$disturbance_secondary_tx" "$disturbance_secondary_ty" \
  "$disturbance_secondary_tz" <<'PY'
import math
import sys

values = [float(value) for value in sys.argv[1:11]]
end_time, disturbance_start, disturbance_end = values[:3]
if not all(math.isfinite(value) for value in values):
    raise SystemExit("all experiment parameters must be finite")
if end_time <= 0.0:
    raise SystemExit("END_TIME must be positive")
if disturbance_start < 0.0 or disturbance_end <= disturbance_start:
    raise SystemExit("the disturbance interval must satisfy 0 <= start < end")
if disturbance_end >= end_time:
    raise SystemExit("END_TIME must leave a recovery window after the disturbance")
if values[-1] <= 0.0:
    raise SystemExit("TRUE_PLANT_MASS_SCALE must be positive")
profile = sys.argv[11]
if profile not in {"constant", "sign_reversal", "sine", "pulse_train",
                   "sine_cosine"}:
    raise SystemExit(
        "PROFILE must be constant, sign_reversal, sine, pulse_train, or sine_cosine")
profile_values = [float(value) for value in sys.argv[12:]]
if not all(math.isfinite(value) for value in profile_values):
    raise SystemExit("profile parameters must be finite")
if profile_values[0] <= 0.0:
    raise SystemExit("FREQUENCY_HZ must be positive")
PY

python3 - "$locomotion_mode" "$walking_target_step" \
  "$walking_target_start" "$walking_target_duration" <<'PY'
import math
import sys

mode = sys.argv[1]
if mode not in {"stance", "walk"}:
    raise SystemExit("LOCOMOTION_MODE must be stance or walk")
values = [float(value) for value in sys.argv[2:]]
if not all(math.isfinite(value) for value in values):
    raise SystemExit("walking target parameters must be finite")
if values[0] <= 0.0 or values[1] < 0.0 or values[2] <= 0.0:
    raise SystemExit("walking target requires step > 0, start >= 0, duration > 0")
PY

if [ -e "$output_dir/closed_loop.bag" ] || [ -e "$output_dir/protocol.txt" ]; then
  echo "refusing to overwrite existing run: $output_dir" >&2
  exit 2
fi
if [ ! -f "$settings_file" ]; then
  echo "settings file not found: $settings_file" >&2
  exit 2
fi

mkdir -p "$output_dir"
source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"
export ROS_MASTER_URI="http://127.0.0.1:${ros_port}"
export ROS_IP="127.0.0.1"
export ROS_HOME="$output_dir/ros_home"
set -u
mkdir -p "$ROS_HOME/log"

sha256sum "$settings_file" > "$output_dir/settings.sha256"
cp "$settings_file" "$output_dir/settings_used.info"
printf '%s\n' \
  "settings=$settings_file" \
  "disturbance_world_wrench=[$disturbance_fx,$disturbance_fy,$disturbance_fz,$disturbance_tx,$disturbance_ty,$disturbance_tz]" \
  "disturbance_profile=$disturbance_profile" \
  "disturbance_frequency_hz=$disturbance_frequency_hz" \
  "disturbance_secondary_world_wrench=[$disturbance_secondary_fx,$disturbance_secondary_fy,$disturbance_secondary_fz,$disturbance_secondary_tx,$disturbance_secondary_ty,$disturbance_secondary_tz]" \
  "disturbance_interval=[$disturbance_start,$disturbance_end]" \
  "true_plant_mass_scale=$true_plant_mass_scale" \
  "locomotion_mode=$locomotion_mode" \
  "walking_target=[$walking_target_step,$walking_target_start,$walking_target_duration]" \
  "end_time=$end_time" \
  "ros_port=$ros_port" > "$output_dir/protocol.txt"

core_pid=""
launch_pid=""
bag_pid=""
waiter_pid=""
cleanup() {
  for pid_name in waiter_pid launch_pid bag_pid core_pid; do
    pid="${!pid_name}"
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

roscore -p "$ros_port" > "$output_dir/roscore.log" 2>&1 &
core_pid=$!
for _ in $(seq 1 80); do
  rosparam list >/dev/null 2>&1 && break
  kill -0 "$core_pid" 2>/dev/null || {
    echo "roscore stopped during startup" >&2
    exit 1
  }
  sleep 0.25
done
rosparam list >/dev/null 2>&1 || {
  echo "roscore did not start" >&2
  exit 1
}

rosbag record --buffsize=512 -O "$output_dir/closed_loop.bag" \
  /rbf_closed_loop/sample \
  /adaptive_update/sample \
  /adaptive_update/disturbance_sample \
  /task_b/equation10_sample \
  /legged_robot_mpc_observation \
  /legged_robot_mpc_target \
  /legged_robot_mpc_mode_schedule \
  /task_c/locomotion_scheduler_status \
  > "$output_dir/rosbag.log" 2>&1 &
bag_pid=$!

timeout 600s python3 - "$end_time" > "$output_dir/waiter.log" 2>&1 <<'PY' &
import sys
import rospy
from std_msgs.msg import Float64MultiArray

end_time = float(sys.argv[1])
rospy.init_node("rbf_closed_loop_validation_waiter", anonymous=True)

def callback(message):
    if len(message.data) == 8 and message.data[0] >= end_time:
        rospy.signal_shutdown("requested simulation horizon reached")

rospy.Subscriber("/adaptive_update/disturbance_sample", Float64MultiArray,
                 callback, queue_size=10)
rospy.spin()
PY
waiter_pid=$!

roslaunch ocs2_legged_robot_adaptive_ros adaptive_update_validation.launch \
  adaptiveSettingsFile:="$settings_file" \
  disturbance_start_time:="$disturbance_start" disturbance_end_time:="$disturbance_end" \
  disturbance_fx:="$disturbance_fx" disturbance_fy:="$disturbance_fy" disturbance_fz:="$disturbance_fz" \
  disturbance_tx:="$disturbance_tx" disturbance_ty:="$disturbance_ty" disturbance_tz:="$disturbance_tz" \
  disturbance_profile:="$disturbance_profile" disturbance_frequency_hz:="$disturbance_frequency_hz" \
  disturbance_secondary_fx:="$disturbance_secondary_fx" \
  disturbance_secondary_fy:="$disturbance_secondary_fy" \
  disturbance_secondary_fz:="$disturbance_secondary_fz" \
  disturbance_secondary_tx:="$disturbance_secondary_tx" \
  disturbance_secondary_ty:="$disturbance_secondary_ty" \
  disturbance_secondary_tz:="$disturbance_secondary_tz" \
  true_plant_mass_scale:="$true_plant_mass_scale" \
  locomotion_mode:="$locomotion_mode" \
  walking_target_step:="$walking_target_step" \
  walking_target_start_time:="$walking_target_start" \
  walking_target_duration:="$walking_target_duration" \
  > "$output_dir/roslaunch.log" 2>&1 &
launch_pid=$!

wait "$waiter_pid"
waiter_pid=""
sleep 1
cleanup
trap - EXIT

grep -E "process has died|terminate called|what\(\):|ERROR" \
  "$output_dir/roslaunch.log" > "$output_dir/runtime_errors.log" || true
if [ -s "$output_dir/runtime_errors.log" ]; then
  echo "runtime errors were found; see $output_dir/runtime_errors.log" >&2
  exit 1
fi
echo "completed=true" >> "$output_dir/protocol.txt"
