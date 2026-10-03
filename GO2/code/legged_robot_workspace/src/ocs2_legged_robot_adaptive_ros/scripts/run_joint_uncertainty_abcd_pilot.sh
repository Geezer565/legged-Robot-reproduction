#!/usr/bin/env bash
# One deliberately small, mode-driven runner for the A/B/C/D quick pilot.
set -euo pipefail
mode="${1:?usage: run_joint_uncertainty_abcd_pilot.sh A|B|C|D output_dir [runtime fx end]}"
out_dir="${2:?usage: run_joint_uncertainty_abcd_pilot.sh A|B|C|D output_dir [runtime fx end]}"
runtime="${3:-20.0}"
disturbance_fx="${4:-20.0}"
disturbance_end="${5:-16.0}"
case "$mode" in
  A) settings="adaptive_settings_abcd_pilot_no_adaptation_clf.info" ;;
  B) settings="adaptive_settings_abcd_pilot_wrench_6d.info" ;;
  C) settings="adaptive_settings_abcd_pilot_inertial_10d.info" ;;
  D) settings="adaptive_settings_abcd_pilot_joint_16d.info" ;;
  *) echo "mode must be A, B, C, or D" >&2; exit 2 ;;
esac
settings="${ADAPTIVE_SETTINGS_FILE:-$settings}"
ws_dir="/home/han/ocs2_ws"
case "$mode" in A) default_port=11411 ;; B) default_port=11412 ;; C) default_port=11413 ;; D) default_port=11414 ;; esac
ros_port="${ROS_PORT:-$default_port}"
settings_path="$ws_dir/src/ocs2_legged_robot_adaptive/config/$settings"
mkdir -p "$out_dir"
source "$ws_dir/devel/setup.bash"
export ROS_MASTER_URI="http://127.0.0.1:${ros_port}"
export ROS_IP="127.0.0.1"
export ROS_HOME="$out_dir/ros_home"
mkdir -p "$ROS_HOME/log"
sha256sum "$settings_path" > "$out_dir/settings.sha256"
printf 'mode=%s\nsettings=%s\nplant=point_mass(5.213485kg,[0.05,0,0]m),world_wrench=[%s,0,0,0,0,0],t=[4,%s],runtime=%s,target=0.10@1.20\n' "$mode" "$settings" "$disturbance_fx" "$disturbance_end" "$runtime" > "$out_dir/protocol.txt"
roscore -p "$ros_port" > "$out_dir/roscore.log" 2>&1 & core_pid=$!
launch_pid=""; bag_pid=""
cleanup() {
  [ -n "$bag_pid" ] && kill "$bag_pid" 2>/dev/null || true
  [ -n "$launch_pid" ] && kill "$launch_pid" 2>/dev/null || true
  kill "$core_pid" 2>/dev/null || true
  wait "$bag_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
  wait "$core_pid" 2>/dev/null || true
}
trap cleanup EXIT
for _ in $(seq 1 40); do rosparam list >/dev/null 2>&1 && break; sleep 0.25; done
# `-a` is intentional: explicit topics absent when rosbag starts are not
# subscribed dynamically by this rosbag version.  Recording all topics makes
# the latched, one-shot actual MPC target auditable without altering it.
rosbag record -a -O "$out_dir/pilot" > "$out_dir/rosbag.log" 2>&1 & bag_pid=$!
roslaunch ocs2_legged_robot_adaptive_ros task_d_payload_inertial_10d_experiment.launch adaptiveSettingsFile:="$settings_path" payload_enabled:=true payload_mass:=5.213485 payload_com_x:=0.05 payload_com_y:=0.0 payload_com_z:=0.0 payload_inertia_xx:=0.0 payload_inertia_xy:=0.0 payload_inertia_xz:=0.0 payload_inertia_yy:=0.0 payload_inertia_yz:=0.0 payload_inertia_zz:=0.0 disturbance_enabled:=true disturbance_start_time:=4.0 disturbance_end_time:="$disturbance_end" disturbance_fx:="$disturbance_fx" target_step:=0.10 target_start_time:=1.20 richer_reference_enabled:=false > "$out_dir/launch.log" 2>&1 & launch_pid=$!
set +e
timeout 1200s python3 "$ws_dir/src/ocs2_legged_robot_adaptive_ros/scripts/wait_for_mpc_time.py" "$runtime"
wait_status=$?
set -e
printf 'wait_status=%s\n' "$wait_status" >> "$out_dir/protocol.txt"
if [ "$wait_status" -eq 0 ]; then printf 'completed_mpc_time=%s\n' "$runtime" >> "$out_dir/protocol.txt"; fi
sleep 2
