#!/usr/bin/env bash
# Blocking Phase 2B runner. Arguments: OUT_DIR FX FY FZ TX TY TZ [RUNTIME].
set -euo pipefail

if [ "$#" -lt 7 ] || [ "$#" -gt 8 ]; then
  echo "usage: $0 OUT_DIR FX FY FZ TX TY TZ [RUNTIME]" >&2
  exit 64
fi
out_dir="$1"; shift
fx="$1"; fy="$2"; fz="$3"; tx="$4"; ty="$5"; tz="$6"
runtime="${7:-20.0}"
ws_dir="/home/han/ocs2_ws"
settings_path="${ADAPTIVE_SETTINGS_FILE:-$ws_dir/src/ocs2_legged_robot_adaptive/config/adaptive_settings_constant_wrench_6d_force_only_clf_off.info}"
ros_port="${ROS_PORT:-11416}"
disturbance_end="${DISTURBANCE_END:-18.0}"

if [ -e "$out_dir/pilot.bag" ] || [ -e "$out_dir/protocol.txt" ]; then
  echo "refusing to overwrite existing run: $out_dir" >&2
  exit 2
fi
mkdir -p "$out_dir"
source "$ws_dir/devel/setup.bash"
export ROS_MASTER_URI="http://127.0.0.1:${ros_port}"
export ROS_IP="127.0.0.1"
export ROS_HOME="$out_dir/ros_home"
mkdir -p "$ROS_HOME/log"

sha256sum "$settings_path" > "$out_dir/settings.sha256"
printf '%s\n' \
  "settings=$settings_path" \
  "parameterization=constant_wrench_6d" \
  "enable_clf_constraint=false" \
  "nominal_model=animal_dummy" \
  "true_plant_mass_scale=1.0" \
  "true_plant_payload_inertial_enabled=false" \
  "world_wrench=[$fx,$fy,$fz,$tx,$ty,$tz]" \
  "disturbance_interval=[4,$disturbance_end]" \
  "runtime=$runtime" > "$out_dir/protocol.txt"

roscore -p "$ros_port" > "$out_dir/roscore.log" 2>&1 & core_pid=$!
launch_pid=""; bag_pid=""
cleanup() {
  [ -n "$bag_pid" ] && kill -INT "$bag_pid" 2>/dev/null || true
  [ -n "$launch_pid" ] && kill -INT "$launch_pid" 2>/dev/null || true
  kill -INT "$core_pid" 2>/dev/null || true
  [ -n "$bag_pid" ] && wait "$bag_pid" 2>/dev/null || true
  [ -n "$launch_pid" ] && wait "$launch_pid" 2>/dev/null || true
  wait "$core_pid" 2>/dev/null || true
}
trap cleanup EXIT
for _ in $(seq 1 40); do rosparam list >/dev/null 2>&1 && break; sleep 0.25; done
rosbag record -a -O "$out_dir/pilot" > "$out_dir/rosbag.log" 2>&1 & bag_pid=$!
roslaunch "$ws_dir/src/ocs2_legged_robot_adaptive_ros/launch/rbf_wrench_pilot_experiment.launch" \
  adaptiveSettingsFile:="$settings_path" disturbance_enabled:=true \
  disturbance_start_time:=4.0 disturbance_end_time:="$disturbance_end" \
  disturbance_fx:="$fx" disturbance_fy:="$fy" disturbance_fz:="$fz" \
  disturbance_tx:="$tx" disturbance_ty:="$ty" disturbance_tz:="$tz" \
  > "$out_dir/launch.log" 2>&1 & launch_pid=$!
set +e
timeout 1800s python3 "$ws_dir/src/ocs2_legged_robot_adaptive_ros/scripts/wait_for_mpc_time.py" "$runtime"
wait_status=$?
set -e
printf 'wait_status=%s\n' "$wait_status" >> "$out_dir/protocol.txt"
if [ "$wait_status" -eq 0 ]; then printf 'completed_mpc_time=%s\n' "$runtime" >> "$out_dir/protocol.txt"; fi
sleep 2
