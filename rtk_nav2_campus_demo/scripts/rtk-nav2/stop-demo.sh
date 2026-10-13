#!/usr/bin/env bash
# 停止 RTK 第三阶段演示栈（只作用于容器 robonix-rtk-nav2-demo）。
# 用法：scripts/rtk-nav2/stop-demo.sh [run-id]
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="${1:-}"
[ -z "${RUN_ID}" ] && RUN_ID="$(current_run_id || true)"

if ! container_running; then
    log "容器 ${CONTAINER_NAME} 未在运行，无需停止"
    exit 0
fi

log "停止容器 ${CONTAINER_NAME} 内的第三阶段演示进程${RUN_ID:+（run-id: ${RUN_ID}）}"
set +e
cexec "/work/scripts/rtk-nav2/container/stop_stack.sh '${RUN_ID}'"
set -e

# 兜底：把容器内残留的 gz 传输文件清掉，否则下次 Gazebo 起不来
cexec "rm -rf /tmp/ign-* /tmp/gz-* /dev/shm/ign* /dev/shm/gz* 2>/dev/null; true"
log "已停止。容器保留（属主可继续复用）；下次用 scripts/rtk-nav2/demo-start.sh 重新起栈（★ 无参数 ✓）。"
