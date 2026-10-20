#!/usr/bin/env bash
set -eo pipefail
task_pkg="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
task_api="$(rbnx path robonix-api)"
task_root="$(cd "$task_pkg/../.." && pwd)"
export OUTDOORNAV_PYTHON="${OUTDOORNAV_PYTHON:-$task_root/.venv/bin/python}"
export PYTHONPATH="$task_pkg/rbnx-build/codegen/robonix_mcp_types:$task_pkg/rbnx-build/codegen/proto_gen:$task_api:${PYTHONPATH:-}"
cleanup() {
  [[ -z "${task_stack_pid:-}" ]] || kill -TERM "$task_stack_pid" 2>/dev/null || true
  [[ -z "${task_bridge_pid:-}" ]] || kill -TERM "$task_bridge_pid" 2>/dev/null || true
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
bash "$task_root/deploy/lynx/start_existing.sh" &
task_stack_pid=$!
"$OUTDOORNAV_PYTHON" "$task_pkg/bridge.py" &
task_bridge_pid=$!
wait -n "$task_stack_pid" "$task_bridge_pid"
