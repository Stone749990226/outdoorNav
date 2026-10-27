#!/usr/bin/env bash
# Field demo: production mode (real /cmd_vel) + Twist->/NAV_CMD bridge.
set -eo pipefail
root="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$root"
source /opt/ros/humble/setup.bash
source /home/nvidia/rslidar_ws/install/setup.bash
source /home/nvidia/drdds_build_ws/install/setup.bash
export PYTHONPATH="$root:$root/ros_ws/src/outdoornav_ros:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OUTDOORNAV_WEB_ROOT="$root/web/dist"
export LYNX_PLACES="$root/scenes/north_campus/places.demo.json"
export LYNX_GNSS_MODE=managed LYNX_IMU_MODE=managed
export LYNX_ROBOT_CONFIG="$root/deploy/lynx/robot.fieldtest.json"
setsid /usr/bin/python3 deploy/lynx/cmd_vel_bridge.py > run/bridge.log 2>&1 &
bash deploy/lynx/start.sh > run/outdoor_prod.log 2>&1 &
echo $! > run/outdoor_prod.pid
wait
