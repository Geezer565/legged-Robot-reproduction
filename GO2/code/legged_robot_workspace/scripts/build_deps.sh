#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
set -euo pipefail

DEPS_ROOT=/workspace/deps
BUILD_ROOT=/workspace/.deps-build

mkdir -p "$BUILD_ROOT/hpp-fcl" "$BUILD_ROOT/pinocchio"

cmake -S "$DEPS_ROOT/hpp-fcl" -B "$BUILD_ROOT/hpp-fcl" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_PYTHON_INTERFACE=OFF \
  -DBUILD_TESTING=OFF
cmake --build "$BUILD_ROOT/hpp-fcl" --parallel 2
cmake --install "$BUILD_ROOT/hpp-fcl"
ldconfig

export CMAKE_PREFIX_PATH="/usr/local:${CMAKE_PREFIX_PATH:-}"
export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:/usr/local/lib64/pkgconfig:${PKG_CONFIG_PATH:-}"

cmake -S "$DEPS_ROOT/pinocchio" -B "$BUILD_ROOT/pinocchio" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_PYTHON_INTERFACE=OFF \
  -DBUILD_TESTING=OFF \
  -DBUILD_WITH_COLLISION_SUPPORT=ON
cmake --build "$BUILD_ROOT/pinocchio" --parallel 2
cmake --install "$BUILD_ROOT/pinocchio"
ldconfig

HPP_VERSION=$(PKG_CONFIG_PATH=/usr/local/lib/pkgconfig pkg-config --modversion hpp-fcl)
PINOCCHIO_VERSION=$(PKG_CONFIG_PATH=/usr/local/lib/pkgconfig pkg-config --modversion pinocchio)

echo "Installed HPP-FCL $HPP_VERSION"
echo "Installed Pinocchio $PINOCCHIO_VERSION"

[[ "$HPP_VERSION" == 1.* ]]
[[ "$PINOCCHIO_VERSION" == 2.5.* ]]
