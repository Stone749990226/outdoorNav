#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd "$(dirname "$0")/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -x "$task_root/.venv/bin/python" ]]; then
  uv venv --python /usr/bin/python3 --system-site-packages "$task_root/.venv"
fi
uv pip install --python "$task_root/.venv/bin/python" -e "$task_root" \
  -e "$(rbnx path robonix-api)[codegen]" 'numpy<2'
export PATH="$task_root/.venv/bin:$PATH"
colcon --log-base "$task_root/.ros-log" build --base-paths "$task_root" "$task_root/ros_ws/src" \
  --packages-select outdoornav_core outdoornav_ros \
  --build-base "$task_root/.ros-build" --install-base "$task_root/.ros-install"
rbnx codegen -p "$task_root/integrations/robonix_outdoor" --mcp --python "$task_root/.venv/bin/python"
