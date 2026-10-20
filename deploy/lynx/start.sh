#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd "$(dirname "$0")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$task_root/.ros-install/setup.bash"
export PYTHONPATH="$task_root:$task_root/ros_ws/src/outdoornav_ros:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PATH="$task_root/.venv/bin:$PATH"
export PYTHONPATH="$task_root/.venv/lib/python3.10/site-packages:$PYTHONPATH"
task_config="${LYNX_ROBOT_CONFIG:-$task_root/deploy/lynx/robot.json}"
task_places="${LYNX_PLACES:-$task_root/scenes/north_campus/places.json}"
task_run="${LYNX_RUN_DIR:-$task_root/artifacts/lynx}"
mkdir -p "$task_run"
exec 9>"$task_run/outdoor.lock"
flock -n 9 || { echo 'Outdoor navigation is already running'; exit 1; }
"$task_root/.venv/bin/python" "$task_root/deploy/lynx/preflight.py" "$task_config"
cleanup() {
  [[ -z "${task_launch_pid:-}" ]] || kill -INT "$task_launch_pid" 2>/dev/null || true
  [[ -z "${task_gnss_pid:-}" ]] || kill -TERM "$task_gnss_pid" 2>/dev/null || true
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
export LYNX_GNSS_PORT="$("$task_root/.venv/bin/python" -c 'import json,sys; print(json.load(open(sys.argv[1]))["gnss_udp_port"])' "$task_config")"
"${LYNX_NTRIP_DIR:-/home/nvidia/ntrip}/.venv/bin/python" "$task_root/deploy/lynx/gnss_stream.py" &
task_gnss_pid=$!
ros2 launch outdoornav_ros lynx.launch.py \
  bundle:="$task_root/scenes/north_campus/generated/bundle.json" \
  robot_config:="$task_config" places:="$task_places" run_dir:="$task_run" &
task_launch_pid=$!
wait -n "$task_launch_pid" "$task_gnss_pid"
