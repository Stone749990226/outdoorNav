#!/usr/bin/env bash
set -eo pipefail
campus_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$campus_dir/generated/campus.world" ]]; then
  echo "Generate the scene first: bash gazebo_campus/build.sh" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
# gazebo_ros Python executables use /usr/bin/env python3; avoid conda's interpreter.
export PATH="/opt/ros/humble/bin:/usr/bin:/bin:$PATH"
# Keep this simulator separate from the legacy web-demo ROS graph and Gazebo.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$campus_dir/config/cyclonedds.xml"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11456}"
export GAZEBO_IP=127.0.0.1
export IGN_IP=127.0.0.1
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
exec ros2 launch "$campus_dir/launch/campus.launch.py" "$@"
