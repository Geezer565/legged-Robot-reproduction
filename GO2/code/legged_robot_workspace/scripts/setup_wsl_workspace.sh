#!/usr/bin/env bash
set -euo pipefail

workspace_target=/mnt/d/GO2/code/legged_robot_workspace

if [[ -L /workspace ]]; then
  current_target=$(readlink -f /workspace)
  if [[ "$current_target" != "$workspace_target" ]]; then
    echo "Refusing to replace /workspace symlink: $current_target" >&2
    exit 2
  fi
elif [[ -e /workspace ]]; then
  echo "Refusing to replace existing /workspace" >&2
  exit 2
else
  ln -s "$workspace_target" /workspace
fi

bash /workspace/scripts/build_deps.sh

