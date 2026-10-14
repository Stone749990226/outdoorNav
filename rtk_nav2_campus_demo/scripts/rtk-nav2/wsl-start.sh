#!/usr/bin/env bash
# Run the RTK/Nav2 campus stack on an Ubuntu 22.04 WSL host.
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
demo_dir="$(cd "${script_dir}/../.." && pwd)"
repo_dir="$(cd "${demo_dir}/.." && pwd)"
ws="${demo_dir}/rtk_nav2_ws"
pkg="${ws}/src/robonix_rtk_nav2"
run_dir="${demo_dir}/artifacts/rtk-nav2/wsl"
partition="${GZ_PARTITION:-rtknav2_wsl}"
rviz=false
rebuild=false
render_mode=auto

for arg in "$@"; do
    case "${arg}" in
        --rviz) rviz=true ;;
        --no-rviz) rviz=false ;;
        --rebuild) rebuild=true ;;
        --render-auto) render_mode=auto ;;
        --render-software) render_mode=software ;;
        --render-hardware) render_mode=hardware ;;
        -h|--help)
            echo "Usage: bash scripts/rtk-nav2/wsl-start.sh [--rviz] [--rebuild] [--render-auto|--render-software|--render-hardware]"
            exit 0 ;;
        *) echo "Unknown option: ${arg}" >&2; exit 2 ;;
    esac
done

export PATH="/opt/ros/humble/bin:/usr/bin:/bin:${PATH}"
export PYTHONPATH="/usr/lib/python3/dist-packages:${pkg}/scripts${PYTHONPATH:+:${PYTHONPATH}}"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-56}"
export ROS_LOCALHOST_ONLY=1
export GZ_PARTITION="${partition}"
export IGN_PARTITION="${partition}"
export IGN_IP="${IGN_IP:-127.0.0.1}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
export IGN_GAZEBO_SERVER_CONFIG_PATH="${pkg}/config/north_campus_server.config"
export GZ_SIM_SERVER_CONFIG_PATH="${IGN_GAZEBO_SERVER_CONFIG_PATH}"
resource_paths="/opt/ros/humble/share:${repo_dir}/gazebo_campus/generated/models"
export IGN_GAZEBO_RESOURCE_PATH="${resource_paths}${IGN_GAZEBO_RESOURCE_PATH:+:${IGN_GAZEBO_RESOURCE_PATH}}"
export GZ_SIM_RESOURCE_PATH="${resource_paths}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QT_X11_NO_MITSHM=1
# WSLg otherwise commonly selects the integrated Intel adapter on laptops.
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
source "${script_dir}/wsl-mesa-env.sh"

for name in ros_gz_sim ros_gz_bridge robot_localization nav2_planner turtlebot3_description; do
    ros2 pkg prefix "${name}" >/dev/null 2>&1 || {
        echo "Missing ROS package: ${name}" >&2
        exit 1
    }
done
command -v ign >/dev/null || { echo "Missing Gazebo Fortress (ign)" >&2; exit 1; }
command -v xdpyinfo >/dev/null && xdpyinfo >/dev/null 2>&1 || {
    echo "No accessible X display; Gazebo GUI is required for the depth camera" >&2
    exit 1
}
[[ -f "${pkg}/worlds/north_campus.sdf" && -f "${IGN_GAZEBO_SERVER_CONFIG_PATH}" ]] || {
    echo "Campus world or Gazebo server configuration is missing" >&2
    exit 1
}
[[ -d "${repo_dir}/gazebo_campus/generated/models/campus_scene" ]] || {
    echo "Campus scene is missing: ${repo_dir}/gazebo_campus/generated/models/campus_scene" >&2
    exit 1
}

mkdir -p "${run_dir}/logs" "${run_dir}/gui"
if [[ -f "${run_dir}/start.pid" ]] && kill -0 "$(<"${run_dir}/start.pid")" 2>/dev/null; then
    echo "WSL stack is already running (PID $(<"${run_dir}/start.pid"))" >&2
    exit 1
fi

# Ogre Next 2.2 uses ARB_copy_image for texture copies. WSLg's D3D12
# driver may advertise NV_copy_image only, which this Ogre version cannot
# use and aborts in GL3PlusTextureGpu::copyTo. Probe the actual GL context
# before starting the stack; nvidia-smi alone cannot establish GL support.
command -v glxinfo >/dev/null || {
    echo "Missing glxinfo; install mesa-utils for the rendering compatibility check" >&2
    exit 1
}
if [[ "${render_mode}" != software ]]; then
    if env LIBGL_ALWAYS_SOFTWARE=0 glxinfo >"${run_dir}/logs/render_hardware.log" 2>&1 &&
        grep -qw GL_ARB_copy_image "${run_dir}/logs/render_hardware.log"; then
        export LIBGL_ALWAYS_SOFTWARE=0
    elif [[ "${render_mode}" == hardware ]]; then
        echo "Hardware OpenGL lacks GL_ARB_copy_image required by Ogre2; use --render-software (details: ${run_dir}/logs/render_hardware.log)" >&2
        exit 1
    else
        echo "Hardware OpenGL is incompatible with Ogre2; selecting Mesa software rendering."
        render_mode=software
    fi
fi
if [[ "${render_mode}" == software ]]; then
    export LIBGL_ALWAYS_SOFTWARE=1
    [[ -z "${WSL_MESA_ACTIVE_PREFIX:-}" ]] || export GALLIUM_DRIVER=llvmpipe
fi
if ! glxinfo >"${run_dir}/logs/render_selected.log" 2>&1 ||
    ! grep -qw GL_ARB_copy_image "${run_dir}/logs/render_selected.log"; then
    echo "Selected OpenGL renderer cannot run Ogre2; see ${run_dir}/logs/render_selected.log" >&2
    exit 1
fi
grep -m 1 'OpenGL renderer string:' "${run_dir}/logs/render_selected.log"

if [[ "${rebuild}" == true || ! -f "${ws}/install/setup.bash" ]]; then
    echo "Building RTK/Nav2 and HTTP bridge workspaces..."
    (cd "${ws}" && colcon build --base-paths "${ws}/src" "${repo_dir}/ros2_bridge" \
        --cmake-args -DBUILD_TESTING=OFF)
fi
source "${ws}/install/setup.bash"

read -r spawn_x spawn_y < <(/usr/bin/python3 - "${script_dir}/demo-data/waypoints_export.csv" <<'PY'
import csv
import sys

with open(sys.argv[1], encoding="utf-8-sig", newline="") as source:
    first = next(csv.DictReader(source))
print(first["x"], first["y"])
PY
)
[[ -n "${spawn_x:-}" && -n "${spawn_y:-}" ]] || {
    echo "Cannot read the spawn point from demo-data/waypoints_export.csv" >&2
    exit 1
}

gui_config="${run_dir}/gui/gui_demo.config"
gui_args=(--source "${demo_dir}/docker/gz-gui/gui.config" --out "${gui_config}")
[[ -n "${DEMO_GUI_CAMERA:-}" ]] && gui_args+=(--camera "${DEMO_GUI_CAMERA}")
/usr/bin/python3 "${script_dir}/make_demo_gui_config.py" "${gui_args[@]}" \
    >"${run_dir}/logs/gui_config.log"

launch_pid=""
spherical_pid=""
readiness_pid=""
cleanup() {
    trap - EXIT INT TERM
    if [[ -n "${readiness_pid}" ]] && kill -0 "${readiness_pid}" 2>/dev/null; then
        kill "${readiness_pid}" 2>/dev/null || true
    fi
    # Fortress puts its server and GUI in separate process groups.
    for child in $(pgrep -u "$(id -u)" -f '^ign gazebo (server|gui)$' || true); do
        if tr '\0' '\n' <"/proc/${child}/environ" 2>/dev/null |
            grep -Fxq "GZ_PARTITION=${partition}"; then
            kill -TERM "${child}" 2>/dev/null || true
        fi
    done
    if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
        kill -TERM -- "-${launch_pid}" 2>/dev/null || true
        for _ in {1..20}; do
            kill -0 "${launch_pid}" 2>/dev/null || break
            sleep 0.5
        done
        kill -KILL -- "-${launch_pid}" 2>/dev/null || true
        wait "${launch_pid}" 2>/dev/null || true
    fi
    for child in $(pgrep -u "$(id -u)" -f '^ign gazebo (server|gui)$' || true); do
        if tr '\0' '\n' <"/proc/${child}/environ" 2>/dev/null |
            grep -Fxq "GZ_PARTITION=${partition}"; then
            kill -KILL "${child}" 2>/dev/null || true
        fi
    done
    if [[ -n "${spherical_pid}" ]] && kill -0 "${spherical_pid}" 2>/dev/null; then
        kill "${spherical_pid}" 2>/dev/null || true
    fi
    rm -f "${run_dir}/start.pid" "${run_dir}/launch.pid"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf '%s\n' "$$" >"${run_dir}/start.pid"

# The world has spherical coordinates; the service call also confirms the live world.
OKFILE="${run_dir}/spherical_ok" bash "${script_dir}/container/set_spherical_coords.sh" \
    north_campus "${partition}" 300 >"${run_dir}/logs/spherical.log" 2>&1 &
spherical_pid=$!

setsid ros2 launch robonix_rtk_nav2 demo.launch.py \
    "world:=${pkg}/worlds/north_campus.sdf" \
    "static_world:=${pkg}/worlds/north_campus.sdf" \
    "nav2_config:=${pkg}/config/campus_nav2.yaml" \
    "rviz_config:=${pkg}/config/campus.rviz" \
    "gui_config:=${gui_config}" \
    "x_pose:=${spawn_x}" "y_pose:=${spawn_y}" "z_pose:=0.039" \
    "rviz:=${rviz}" "route:=false" \
    "trajectory_csv:=${run_dir}/trajectory.csv" \
    "param_dump_path:=${run_dir}/params_local_costmap.yaml" \
    >"${run_dir}/logs/demo_launch.log" 2>&1 &
launch_pid=$!
printf '%s\n' "${launch_pid}" >"${run_dir}/launch.pid"
echo "Starting Gazebo and Nav2 (launch PID ${launch_pid}); log: ${run_dir}/logs/demo_launch.log"

timeout -k 5 180 /usr/bin/python3 "${pkg}/scripts/wait_ready.py" 170 \
    >"${run_dir}/logs/readiness.log" 2>&1 &
readiness_pid=$!
if ! wait "${readiness_pid}"; then
    tail -30 "${run_dir}/logs/readiness.log" >&2
    echo "Stack did not become ready; see ${run_dir}/logs/demo_launch.log" >&2
    exit 1
fi
if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "ROS launch exited after readiness; see ${run_dir}/logs/demo_launch.log" >&2
    exit 1
fi
echo "RTK/Nav2 is ready. Start the web bridge with: bash ${script_dir}/wsl-frontend.sh"
wait "${launch_pid}"
