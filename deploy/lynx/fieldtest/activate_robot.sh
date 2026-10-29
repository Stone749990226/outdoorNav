#!/usr/bin/env bash
# Explicit physical action: stand and enter nav mode. Never called by mock mode.
set -eo pipefail
root="$(cd "$(dirname "$0")/../../.." && pwd)"
source /opt/ros/humble/setup.bash
source /home/nvidia/drdds_build_ws/install/setup.bash
pkg=/home/nvidia/robot-deep_robotics-lynx_s10/primitives/primitive-lynx_quadruped
export PATH="/home/nvidia/.cargo/bin:$PATH"
export RBNX_PACKAGE_ROOT="$pkg"
export PYTHONPATH="$(rbnx path robonix-api):$pkg:$root:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec /usr/bin/python3 "$root/deploy/lynx/fieldtest/activate_demo.py"
