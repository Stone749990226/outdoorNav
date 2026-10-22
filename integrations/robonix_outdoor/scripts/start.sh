#!/usr/bin/env bash
set -eo pipefail
task_pkg="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
task_api="$(rbnx path robonix-api)"
task_root="$(cd "$task_pkg/../.." && pwd)"
export OUTDOORNAV_PYTHON="${OUTDOORNAV_PYTHON:-$task_root/.venv/bin/python}"
export PYTHONPATH="$task_pkg/rbnx-build/codegen/robonix_mcp_types:$task_pkg/rbnx-build/codegen/proto_gen:$task_api:${PYTHONPATH:-}"
task_stop_grace_s=30
source "$task_root/deploy/lynx/processes.sh"
start_child bash "$task_root/deploy/lynx/start.sh"
start_child "$OUTDOORNAV_PYTHON" "$task_pkg/bridge.py"
wait -n "${task_children[@]}"
