#!/usr/bin/env bash
# Stop only processes launched by the WSL entrypoints.
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
demo_dir="$(cd "${script_dir}/../.." && pwd)"
run_dir="${demo_dir}/artifacts/rtk-nav2/wsl"

for entry in frontend start; do
    pid_file="${run_dir}/${entry}.pid"
    [[ -f "${pid_file}" ]] || continue
    pid="$(<"${pid_file}")"
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    if [[ -r "/proc/${pid}/cmdline" ]] &&
        tr '\0' ' ' <"/proc/${pid}/cmdline" | grep -q "wsl-${entry}.sh"; then
        kill -TERM "${pid}"
        echo "Stopping wsl-${entry}.sh (PID ${pid})"
    else
        echo "Skipping stale PID file: ${pid_file}" >&2
    fi
done
