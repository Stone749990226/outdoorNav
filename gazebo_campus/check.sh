#!/usr/bin/env bash
set -eo pipefail
campus_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$campus_dir/config/cyclonedds.xml"
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
exec /usr/bin/python3 "$campus_dir/scripts/check_navigation.py" "$@"
