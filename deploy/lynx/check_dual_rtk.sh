#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd "$(dirname "$0")/../.." && pwd)"
export PYTHONPATH="$task_root:${PYTHONPATH:-}"
task_python=python3
if [[ -x "$task_root/.venv/bin/python" ]]; then task_python="$task_root/.venv/bin/python"; fi
task_report="${LYNX_RTK_REPORT:-$HOME/rtk_logs/dual_$(date +%Y%m%d_%H%M%S).json}"
exec "$task_python" "$task_root/deploy/lynx/dual_rtk_check.py" --report "$task_report" "$@"
