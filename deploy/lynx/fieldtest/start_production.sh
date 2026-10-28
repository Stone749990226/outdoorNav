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
mkdir -p run
source "$root/deploy/lynx/processes.sh"
# Same URDF mount as the existing front-lidar primitive and mock profile.
# The production wrapper used to omit this TF while the mock supplied it.
start_child ros2 run tf2_ros static_transform_publisher \
  --x 0.22341 --y 0.0 --z -0.0001 --roll 3.14159 --pitch -1.5708 --yaw 0 \
  --frame-id base_link --child-frame-id airy_201 --ros-args -r __node:=field_lidar_tf \
  > run/lidar_tf.log 2>&1
start_child /usr/bin/python3 deploy/lynx/cmd_vel_bridge.py > run/bridge.log 2>&1
start_child bash deploy/lynx/start.sh > run/outdoor_prod.log 2>&1
echo "${task_children[-1]}" > run/outdoor_prod.pid
wait -n "${task_children[@]}"
