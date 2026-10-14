#!/usr/bin/env bash
# Connect the native WSL RTK/Nav2 stack to the existing web application.
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
demo_dir="$(cd "${script_dir}/../.." && pwd)"
ws="${demo_dir}/rtk_nav2_ws"
run_dir="${demo_dir}/artifacts/rtk-nav2/wsl"
frontend="${demo_dir}/outdoor-patrol-demo"
frontend_port="${FRONTEND_PORT:-5174}"
[[ "${frontend_port}" =~ ^[0-9]+$ ]] || { echo "Invalid FRONTEND_PORT" >&2; exit 2; }

export PATH="/opt/ros/humble/bin:/usr/bin:/bin:${PATH}"
export PYTHONPATH="/usr/lib/python3/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"
source /opt/ros/humble/setup.bash
[[ -f "${ws}/install/setup.bash" ]] || { echo "Run wsl-start.sh first" >&2; exit 1; }
source "${ws}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-56}"
export ROS_LOCALHOST_ONLY=1

[[ -f "${run_dir}/start.pid" ]] && kill -0 "$(<"${run_dir}/start.pid")" 2>/dev/null || {
    echo "RTK/Nav2 stack is not running" >&2
    exit 1
}
if [[ -f "${run_dir}/frontend.pid" ]] && kill -0 "$(<"${run_dir}/frontend.pid")" 2>/dev/null; then
    echo "Web bridge is already running (PID $(<"${run_dir}/frontend.pid"))" >&2
    exit 1
fi
command -v npm >/dev/null || { echo "Node.js/npm is required for the web UI" >&2; exit 1; }

if [[ ! -x "${frontend}/node_modules/.bin/vite" ]]; then
    echo "Installing frontend dependencies from package-lock.json..."
    (cd "${frontend}" && npm ci)
fi

adapter_pid=""
bridge_pid=""
vite_pid=""
cleanup() {
    trap - EXIT INT TERM
    for pid in "${vite_pid}" "${bridge_pid}" "${adapter_pid}"; do
        [[ -n "${pid}" ]] && kill -TERM -- "-${pid}" 2>/dev/null || true
    done
    for pid in "${vite_pid}" "${bridge_pid}" "${adapter_pid}"; do
        [[ -n "${pid}" ]] && wait "${pid}" 2>/dev/null || true
    done
    rm -f "${run_dir}/frontend.pid"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf '%s\n' "$$" >"${run_dir}/frontend.pid"

setsid /usr/bin/python3 "${script_dir}/container/frontend_adapter.py" \
    >"${run_dir}/logs/frontend_adapter.log" 2>&1 &
adapter_pid=$!
setsid ros2 run outdoor_nav_bridge bridge --ros-args -p use_sim_time:=true \
    >"${run_dir}/logs/frontend_bridge.log" 2>&1 &
bridge_pid=$!
setsid bash -c 'cd "$1" && exec npm run dev -- --port "$2" --strictPort' bash \
    "${frontend}" "${frontend_port}" >"${run_dir}/logs/frontend_dev.log" 2>&1 &
vite_pid=$!

sleep 3
for pid in "${adapter_pid}" "${bridge_pid}" "${vite_pid}"; do
    kill -0 "${pid}" 2>/dev/null || {
        echo "A frontend process exited; inspect ${run_dir}/logs/frontend_*.log" >&2
        exit 1
    }
done
echo "Web UI: http://localhost:${frontend_port} (bridge: http://localhost:8765/api/status)"
wait "${vite_pid}"
