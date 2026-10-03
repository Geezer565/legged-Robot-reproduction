#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
set -euo pipefail
cd /workspace

export CMAKE_PREFIX_PATH="/usr/local:${CMAKE_PREFIX_PATH:-}"
export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:/usr/local/lib64/pkgconfig:${PKG_CONFIG_PATH:-}"

# Keep the default conservative on Windows/WSL. Override explicitly with
# BUILD_JOBS=2 (or higher) on a machine with enough memory.
BUILD_JOBS="${BUILD_JOBS:-1}"
if ! [[ "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]]; then
  echo "[build] BUILD_JOBS must be a positive integer" >&2
  exit 2
fi
echo "[build] Parallel jobs: $BUILD_JOBS"

# The Docker image intentionally does not duplicate the teacher workspace's
# Pinocchio/HPP-FCL build. Reuse the bundled sources and persistent build tree
# whenever a fresh container does not yet have them installed in /usr/local.
if ! pkg-config --exists hpp-fcl || ! pkg-config --exists pinocchio; then
  echo "[build] Installing bundled HPP-FCL and Pinocchio into this container"
  bash /workspace/scripts/build_deps.sh
fi

DESCRIPTION_PACKAGES="legged_unitree_description;go2_description"

catkin_make \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCATKIN_ENABLE_TESTING=ON \
  -DCATKIN_WHITELIST_PACKAGES="$DESCRIPTION_PACKAGES" \
  -j"$BUILD_JOBS"

source devel/setup.bash

export CMAKE_PREFIX_PATH="/usr/local:${CMAKE_PREFIX_PATH:-}"
export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:/usr/local/lib64/pkgconfig:${PKG_CONFIG_PATH:-}"

PACKAGES=(
  ocs2_thirdparty
  ocs2_core
  ocs2_oc
  ocs2_mpc
  ocs2_ddp
  blasfeo_catkin
  ocs2_qp_solver
  hpipm_catkin
  ocs2_sqp
  ocs2_ipm
  ocs2_msgs
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_self_collision
  ocs2_self_collision_visualization
  ocs2_legged_robot
  ocs2_legged_robot_ros
  ocs2_legged_robot_adaptive
  ocs2_legged_robot_adaptive_ros
  qpoases_catkin
  legged_common
  legged_interface
  legged_estimation
  legged_wbc
  legged_hw
  legged_gazebo
  legged_unitree_description
  go2_description
  legged_unitree_hw
  legged_controllers
)

PACKAGE_LIST=$(IFS=';'; echo "${PACKAGES[*]}")

catkin_make \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCATKIN_ENABLE_TESTING=ON \
  -DCATKIN_WHITELIST_PACKAGES="$PACKAGE_LIST" \
  -j"$BUILD_JOBS"

