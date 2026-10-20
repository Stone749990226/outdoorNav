#!/usr/bin/env bash
# Only starts the outdoor application; never starts/stops the robot deployment.
set -eo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$root"
config="${LYNX_ROBOT_CONFIG:-$root/deploy/lynx/existing_robot.json}"
run_dir="${OUTDOORNAV_RUN_DIR:-$root/artifacts/lynx-existing}"
mkdir -p "$run_dir"
exec "${OUTDOORNAV_PYTHON:-$root/.venv/bin/python}" -m outdoornav.existing_runtime \
  --config "$config" --places "$root/scenes/north_campus/places.json" --run-dir "$run_dir"
