#!/usr/bin/env bash
# Run on the ROS Humble robot. Fetch a fixed backend, never start hardware.
set -eo pipefail
task_root="$(cd "$(dirname "$0")/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -n "${LYNX_LIVOX_SETUP:-}" ]]; then source "$LYNX_LIVOX_SETUP"; fi
ros2 pkg prefix livox_ros_driver2 >/dev/null || {
  echo 'FAST-LIO requires the real livox_ros_driver2 interface package even for Airy.'
  echo 'Build/source that package first, or set LYNX_LIVOX_SETUP to its setup.bash.'
  exit 1
}
task_src="$task_root/.lio-src/fast_lio"
task_revision=2fffc570a25d0df172720bac034fbdb6a13d2162
if [[ ! -d "$task_src/.git" ]]; then
  mkdir -p "$task_src"
  git -C "$task_src" init
  git -C "$task_src" remote add origin https://github.com/Ericsii/FAST_LIO_ROS2.git
  git -C "$task_src" fetch --depth 1 origin "$task_revision"
  git -C "$task_src" checkout --detach FETCH_HEAD
fi
[[ "$(git -C "$task_src" rev-parse HEAD)" == "$task_revision" ]] || {
  echo 'Unexpected FAST-LIO revision; use a separate checkout'; exit 1;
}
[[ -z "$(git -C "$task_src" status --porcelain --untracked-files=no)" ]] || {
  echo 'FAST-LIO checkout has local changes; refusing to build'; exit 1;
}
git -C "$task_src" submodule update --init --recursive --depth 1
task_python=/usr/bin/python3
if [[ -x "$task_root/.venv/bin/python" ]]; then task_python="$task_root/.venv/bin/python"; fi
"$task_python" "$(command -v colcon)" --log-base "$task_root/.lio-log" build \
  --base-paths "$task_src" --packages-select fast_lio \
  --build-base "$task_root/.lio-build" --install-base "$task_root/.lio-install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
