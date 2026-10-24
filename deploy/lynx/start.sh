#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd "$(dirname "$0")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$task_root/.ros-install/setup.bash"
export PYTHONPATH="$task_root:$task_root/ros_ws/src/outdoornav_ros:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PATH="$task_root/.venv/bin:$PATH"
export PYTHONPATH="$task_root/.venv/lib/python3.10/site-packages:$PYTHONPATH"
export OUTDOORNAV_WEB_ROOT="${OUTDOORNAV_WEB_ROOT:-$task_root/web/dist}"
[[ -f "$OUTDOORNAV_WEB_ROOT/index.html" ]] || { echo "Build the web application first"; exit 1; }
task_config="${LYNX_ROBOT_CONFIG:-$task_root/deploy/lynx/robot.json}"
if "$task_root/.venv/bin/python" -c 'import json,sys; sys.exit(not json.load(open(sys.argv[1])).get("lio", {}).get("enabled", False))' "$task_config"; then
  task_lio_setup="${LYNX_LIO_SETUP:-$task_root/.lio-install/setup.bash}"
  [[ -f "$task_lio_setup" ]] && source "$task_lio_setup"
  ros2 pkg prefix fast_lio >/dev/null || { echo 'Build FAST-LIO first: bash deploy/lynx/build_lio.sh'; exit 1; }
  if [[ -n "${LYNX_LIDAR_SETUP:-}" ]]; then source "$LYNX_LIDAR_SETUP"; fi
fi
task_places="${LYNX_PLACES:-$task_root/scenes/north_campus/places.json}"
task_run="${LYNX_RUN_DIR:-$task_root/artifacts/lynx}"
mkdir -p "$task_run"
exec 9>"$task_run/outdoor.lock"
flock -n 9 || { echo 'Outdoor navigation is already running'; exit 1; }
task_diagnostic="${LYNX_DIAGNOSTIC:-false}"
if [[ "${1:-}" == --diagnostic ]]; then task_diagnostic=true; shift; fi
[[ $# == 0 && ( "$task_diagnostic" == true || "$task_diagnostic" == false ) ]] || {
  echo 'Usage: start.sh [--diagnostic]'; exit 2;
}
task_preflight=()
if [[ "$task_diagnostic" == true ]]; then task_preflight+=(--diagnostic); fi
"$task_root/.venv/bin/python" "$task_root/deploy/lynx/preflight.py" "$task_config" "${task_preflight[@]}"
source "$task_root/deploy/lynx/processes.sh"
task_gnss_mode="${LYNX_GNSS_MODE:-managed}"
if [[ "$task_diagnostic" == true && -z "${LYNX_GNSS_MODE:-}" ]]; then task_gnss_mode=observe; fi
case "$task_gnss_mode" in
  managed)
    export LYNX_GNSS_PORT="$("$task_root/.venv/bin/python" -c 'import json,sys; print(json.load(open(sys.argv[1]))["gnss_udp_port"])' "$task_config")"
    start_child "${LYNX_NTRIP_DIR:-/home/nvidia/ntrip}/.venv/bin/python" "$task_root/deploy/lynx/gnss_stream.py"
    ;;
  observe) echo 'GNSS observation only: no serial reader will be started' ;;
  *) echo 'LYNX_GNSS_MODE must be managed or observe'; exit 2 ;;
esac
if "$task_root/.venv/bin/python" -c 'import sys; from outdoornav.config import load_robot_config; sys.exit(load_robot_config(sys.argv[1])["heading_source"] != "dual_antenna")' "$task_config"; then
  task_heading_mode="${LYNX_HEADING_MODE:-managed}"
  if [[ "$task_diagnostic" == true && -z "${LYNX_HEADING_MODE:-}" ]]; then task_heading_mode=observe; fi
  case "$task_heading_mode" in
    managed) start_child "$task_root/.venv/bin/python" "$task_root/deploy/lynx/heading_stream.py" --config "$task_config" ;;
    observe) echo 'Heading observation only: no serial reader will be started' ;;
    *) echo 'LYNX_HEADING_MODE must be managed or observe'; exit 2 ;;
  esac
fi
start_child ros2 launch outdoornav_ros lynx.launch.py \
  bundle:="$task_root/scenes/north_campus/generated/bundle.json" \
  robot_config:="$task_config" places:="$task_places" run_dir:="$task_run" \
  diagnostic:="$task_diagnostic"
wait -n "${task_children[@]}"
