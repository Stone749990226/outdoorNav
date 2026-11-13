#!/usr/bin/env bash
set -euo pipefail
SIM_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SIM_DEPS_ROOT="${SIM_DEPS_ROOT:-$SIM_ROOT}"
if [[ -d "$SIM_DEPS_ROOT/packages" ]]; then
  SIM_MAP_ROOT="$SIM_DEPS_ROOT/packages/service-map-outdoor-rbnx"
else
  SIM_MAP_ROOT="$SIM_DEPS_ROOT/service-map-outdoor-rbnx"
fi
if [[ -x "$SIM_ROOT/simulation/.venv/bin/python" ]]; then
  SIM_DEFAULT_PYTHON="$SIM_ROOT/simulation/.venv/bin/python"
else
  SIM_DEFAULT_PYTHON="$SIM_DEPS_ROOT/.venv/bin/python"
fi
SIM_PYTHON="${SIM_PYTHON:-$SIM_DEFAULT_PYTHON}"
if [[ ! -x "$SIM_PYTHON" ]]; then
  echo 'Set SIM_PYTHON to a ROS-compatible uv environment Python.' >&2
  exit 2
fi
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "$SIM_MAP_ROOT/rbnx-build/codegen/ros2_idl/install/setup.bash"
set -u
export SIM_DEPS_ROOT
export PATH="$(dirname "$SIM_PYTHON"):$HOME/.cargo/bin:$HOME/.local/bin:$PATH"
export PYTHONPATH="$SIM_ROOT:${PYTHONPATH:-}"
cd "$SIM_ROOT"
exec "$SIM_PYTHON" -m simulation.launch "$@"
