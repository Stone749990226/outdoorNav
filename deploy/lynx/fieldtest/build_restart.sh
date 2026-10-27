#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$root"
bash "$root/kill_stack.sh" >/dev/null 2>&1 || true
find ros_ws -name __pycache__ -prune -exec rm -rf {} + 2>/dev/null || true
source /opt/ros/humble/setup.bash
colcon --log-base .ros-log build --paths . ros_ws/src/outdoornav_ros \
  --packages-select outdoornav_core outdoornav_ros \
  --build-base .ros-build --install-base .ros-install 2>&1 | tail -2
installed=.ros-install/outdoornav_ros/lib/python3.10/site-packages/outdoornav_ros/lynx_localizer.py
if ! grep -q "yaw_std_deg is not None" "$installed"; then echo "PATCH_MISSING"; exit 3; fi
echo "PATCH_PRESENT"
setsid nohup bash "$root/start_production.sh" > run/prod_launcher.log 2>&1 < /dev/null &
echo "LAUNCHED"
