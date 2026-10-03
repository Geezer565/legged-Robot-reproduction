#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
set -euo pipefail
cd /workspace

GO2_ROOT=/workspace/src/leggedcontrol_go2
OCS2_ROOT=/workspace/src/ocs2
ASSETS_ROOT=/workspace/src/ocs2_robotic_assets
PINOCCHIO_ROOT=/workspace/deps/pinocchio
HPPFCL_ROOT=/workspace/deps/hpp-fcl

for required in "$GO2_ROOT" "$OCS2_ROOT" "$ASSETS_ROOT" "$PINOCCHIO_ROOT" "$HPPFCL_ROOT"; do
  if [[ ! -d "$required" ]]; then
    echo "Missing required source directory: $required" >&2
    exit 1
  fi
done

echo "[1/5] Updating rosdep metadata"
rosdep update

echo "[2/5] Installing source dependencies"
rosdep install --from-paths \
  "$HPPFCL_ROOT" \
  "$PINOCCHIO_ROOT" \
  "$OCS2_ROOT/ocs2_thirdparty" \
  "$OCS2_ROOT/ocs2_core" \
  "$OCS2_ROOT/ocs2_oc" \
  "$OCS2_ROOT/ocs2_mpc" \
  "$OCS2_ROOT/ocs2_ddp" \
  "$OCS2_ROOT/ocs2_sqp/blasfeo_catkin" \
  "$OCS2_ROOT/ocs2_sqp/hpipm_catkin" \
  "$OCS2_ROOT/ocs2_sqp/ocs2_sqp" \
  "$OCS2_ROOT/ocs2_ipm" \
  "$OCS2_ROOT/ocs2_msgs" \
  "$OCS2_ROOT/ocs2_ros_interfaces" \
  "$OCS2_ROOT/ocs2_robotic_tools" \
  "$OCS2_ROOT/ocs2_pinocchio/ocs2_pinocchio_interface" \
  "$OCS2_ROOT/ocs2_pinocchio/ocs2_centroidal_model" \
  "$OCS2_ROOT/ocs2_pinocchio/ocs2_self_collision" \
  "$OCS2_ROOT/ocs2_pinocchio/ocs2_self_collision_visualization" \
  "$OCS2_ROOT/ocs2_robotic_examples/ocs2_legged_robot" \
  "$OCS2_ROOT/ocs2_robotic_examples/ocs2_legged_robot_ros" \
  "$OCS2_ROOT/ocs2_test_tools/ocs2_qp_solver" \
  "$ASSETS_ROOT" \
  "$GO2_ROOT/legged_common" \
  "$GO2_ROOT/legged_interface" \
  "$GO2_ROOT/legged_estimation" \
  "$GO2_ROOT/legged_wbc" \
  "$GO2_ROOT/legged_hw" \
  "$GO2_ROOT/legged_gazebo" \
  "$GO2_ROOT/legged_controllers" \
  "$GO2_ROOT/qpoases_catkin" \
  "$GO2_ROOT/legged_examples/legged_unitree/legged_unitree_description" \
  "$GO2_ROOT/legged_examples/legged_unitree/legged_unitree_hw" \
  --ignore-src --rosdistro noetic -r -y \
  --skip-keys="cmake_clang_tools rqt_multiplot python-numpy doxygen libassimp-dev"

echo "[3/5] Building the OCS2-compatible dynamics libraries"
bash /workspace/scripts/build_deps.sh

echo "[4/5] Installing the bundled Unitree SDK"
bash "$GO2_ROOT/legged_examples/legged_unitree/unitree_sdk2-main/install.sh"

echo "[5/5] Configuring and building the validated package set"
bash /workspace/scripts/build.sh

echo "Environment bootstrap completed."
