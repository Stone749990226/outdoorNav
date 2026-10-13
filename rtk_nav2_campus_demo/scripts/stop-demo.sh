#!/usr/bin/env bash
# 停止本项目启动的所有进程。
#
# 只操作专用容器 robonix-slam-demo 内部的进程，且只按本项目记录的 pid 文件与
# 明确的可执行名匹配；宿主机的其他项目、其他容器与其他用户的进程都不受影响。
#
# 用法：
#   scripts/stop-demo.sh             # 停止容器内进程，容器保留
#   scripts/stop-demo.sh --remove    # 停止后删除容器（产物在宿主机目录里，不受影响）
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

REMOVE=0
RUN_ID=""
for arg in "$@"; do
    case "${arg}" in
        --remove|-r) REMOVE=1 ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) RUN_ID="${arg}" ;;
    esac
done

if ! container_running; then
    log "容器 ${CONTAINER_NAME} 未运行，无需停止"
    exit 0
fi

RUN_ID="$(resolve_run_id "${RUN_ID}")"
log "停止容器内本项目进程（run-id=${RUN_ID}）"
cexec "/work/scripts/container/stop_stack.sh '${RUN_ID}'" || true

# 兜底：万一仍有本项目进程存活（例如 ros2 launch 的父进程已消失、子进程被 init 收养），
# 直接停掉这个专用容器。容器只服务本项目，停它不会影响其他项目；产物在宿主机目录里，不受影响。
if docker exec "${CONTAINER_NAME}" pgrep -f "gzserver|gzclient|rviz2|slam_toolbox|robonix_slam_demo" >/dev/null 2>&1; then
    log "仍有残留进程，停止整个专用容器 ${CONTAINER_NAME}"
    docker stop "${CONTAINER_NAME}" >/dev/null
fi

if [ "${REMOVE}" -eq 1 ]; then
    log "删除容器 ${CONTAINER_NAME}"
    docker rm -f "${CONTAINER_NAME}" >/dev/null
fi

log "完成。宿主机上没有启动过任何 ROS 进程，其他项目不受影响。"
