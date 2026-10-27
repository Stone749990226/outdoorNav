#!/usr/bin/env bash
# Indoor mock: real web/Nav2, synthetic RTK+IMU, virtual base. Fully local DDS so
# vendor participants on domain 0 cannot corrupt Nav2 bringup discovery.
set -eo pipefail
root="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$root"
source /opt/ros/humble/setup.bash
source /home/nvidia/rslidar_ws/install/setup.bash
source "$root/.ros-install/setup.bash"
export PYTHONPATH="$root:$root/ros_ws/src/outdoornav_ros:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=1
export OUTDOORNAV_WEB_ROOT="$root/web/dist"
export LYNX_PLACES="$root/scenes/north_campus/places.demo.json"
mkdir -p run run/mock
cat > run/mock.launch.py <<PY
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_directory
ROOT = '${root}'
def generate_launch_description():
    return LaunchDescription([GroupAction([
        SetRemap(src='lynx_command_watchdog:/cmd_vel', dst='/mock_base/cmd_vel'),
        SetRemap(src='navigation:/cmd_vel', dst='/mock_base/cmd_vel'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            get_package_share_directory('outdoornav_ros') + '/launch/lynx.launch.py'),
            launch_arguments={
                'bundle': ROOT + '/scenes/north_campus/generated/bundle.json',
                'robot_config': ROOT + '/deploy/lynx/robot.mock.json',
                'places': ROOT + '/scenes/north_campus/places.demo.json',
                'run_dir': ROOT + '/run/mock',
                'diagnostic': 'false'}.items())])])
PY
ros2 run tf2_ros static_transform_publisher \
  --x 0.22341 --y 0.0 --z -0.0001 --roll 3.14159 --pitch -1.5708 --yaw 0 \
  --frame-id base_link --child-frame-id airy_201 --ros-args -r __node:=mock_lidar_tf \
  > run/mock_tf.log 2>&1 &
/home/nvidia/rslidar_ws/install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node \
  > run/mock_lidar.log 2>&1 &
setsid "$root/.venv/bin/python" deploy/lynx/mock_virtual_base.py \
  --bundle scenes/north_campus/generated/bundle.json --port 18765 \
  > run/mock_base.log 2>&1 &
ros2 launch run/mock.launch.py > run/mock_stack.log 2>&1 &
wait
