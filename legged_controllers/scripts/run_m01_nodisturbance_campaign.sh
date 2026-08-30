#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 OUTPUT_ROOT" >&2
  exit 2
fi

output_root="$1"
if [[ -e "$output_root" ]]; then
  echo "Refusing to overwrite existing output: $output_root" >&2
  exit 3
fi

mkdir -p "$output_root"

methods=(baseline ethrbf myrbf)
speeds=(0.05 0.10 0.15)

config_for_method() {
  case "$1" in
    baseline) echo "myrbf_shadow_baseline.yaml" ;;
    ethrbf) echo "myrbf_shadow_eth_go2_closed_loop.yaml" ;;
    myrbf) echo "myrbf_shadow_v4_go2_robust.yaml" ;;
    *) return 1 ;;
  esac
}

injection_for_method() {
  case "$1" in
    baseline) echo "false" ;;
    ethrbf|myrbf) echo "true" ;;
    *) return 1 ;;
  esac
}

for speed in "${speeds[@]}"; do
  speed_tag="${speed/./}"
  for method in "${methods[@]}"; do
    case_name="M01_v${speed_tag}_${method}"
    case_dir="$output_root/$case_name"
    mkdir -p "$case_dir"
    config="$(config_for_method "$method")"
    injection="$(injection_for_method "$method")"

    echo "[M01] Starting $case_name"
    roslaunch legged_controllers myrbf_shadow_gazebo.launch \
      robot_type:=go2 gui:=false duration:=15 \
      state_output:="$case_dir/state.csv" \
      shadow_output:="$case_dir/shadow.csv" \
      motion_output:="$case_dir/motion.csv" \
      enable_shadow:=true \
      enable_control_injection:="$injection" \
      myrbf_config:="$config" \
      enable_motion:=true motion_gait:=static_walk \
      motion_vx:="$speed" motion_vy:=0.0 motion_yaw_rate:=0.0 \
      motion_delay:=4 motion_duration:=9 \
      enable_wrench:=false \
      >"$case_dir/run.log" 2>&1

    for required in state.csv shadow.csv motion.csv run.log; do
      if [[ ! -s "$case_dir/$required" ]]; then
        echo "Missing or empty $case_dir/$required" >&2
        exit 4
      fi
    done
    echo "[M01] Completed $case_name"
  done
done

echo "[M01] All nine runs completed: $output_root"
