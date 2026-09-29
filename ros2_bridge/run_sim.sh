#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
# Ubuntu's setuptools and packaging must be paired on this WSL image.
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
colcon --log-base "$repo_root/.ros-log" build \
  --base-paths "$repo_root/ros2_bridge" \
  --packages-select outdoor_nav_bridge \
  --build-base "$repo_root/.ros-build" \
  --install-base "$repo_root/.ros-install"
source "$repo_root/.ros-install/setup.bash"
exec ros2 launch outdoor_nav_bridge sim.launch.py
