#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
set -euo pipefail
cd /workspace

if [[ ! -f devel/setup.bash ]]; then
  echo "Workspace is not built. Run /workspace/scripts/bootstrap.sh first." >&2
  exit 1
fi

source devel/setup.bash

TEST_PACKAGES="ocs2_legged_robot_adaptive;ocs2_legged_robot_adaptive_ros"

catkin_make \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCATKIN_ENABLE_TESTING=ON \
  -DCATKIN_WHITELIST_PACKAGES="$TEST_PACKAGES" \
  -j2

catkin_make run_tests_ocs2_legged_robot_adaptive -j2
catkin_make run_tests_ocs2_legged_robot_adaptive_ros -j2
catkin_test_results --verbose build/test_results/ocs2_legged_robot_adaptive
catkin_test_results --verbose build/test_results/ocs2_legged_robot_adaptive_ros

echo "All adaptive package tests passed."
