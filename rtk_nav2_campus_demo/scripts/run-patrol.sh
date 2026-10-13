#!/usr/bin/env bash
# 运行固定路线自动巡游（odom 坐标闭环 + 安全停车），并记录退出状态与截图。
#
# 用法：
#   scripts/run-patrol.sh [run-id] [-- 额外的 --ros-args 参数...]
#
# 退出状态码（由巡游节点返回，本脚本原样透传）：
#   0 正常完成  3 激光失效  4 里程计失效  5 前方障碍  6 任务超时  130 被中断
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID=""
EXTRA=()
while [ $# -gt 0 ]; do
    case "$1" in
        --) shift; EXTRA=("$@"); break ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) RUN_ID="$1"; shift ;;
    esac
done
RUN_ID="$(resolve_run_id "${RUN_ID}")"

ensure_container
demo_stack_running || die "演示栈未运行，请先执行 scripts/start-demo.sh"

RUN_DIR="${PROJECT_DIR}/artifacts/${RUN_ID}"
fix_artifacts_ownership
ensure_run_dirs "${RUN_ID}"

capture_screen "${RUN_DIR}/screenshots/02_patrol_start.png" || true

log "开始自动巡游（run-id=${RUN_ID}）"
set +e
cexec "/work/scripts/container/run_patrol.sh '${RUN_ID}' ${EXTRA[*]:-}" \
    2>&1 | tee "${RUN_DIR}/logs/patrol_console.log"
EXIT_CODE=${PIPESTATUS[0]}
set -e

echo "${EXIT_CODE}" > "${RUN_DIR}/patrol_exit_code.txt"

capture_screen "${RUN_DIR}/screenshots/03_patrol_end.png" || true

case "${EXIT_CODE}" in
    0)   log "巡游正常完成（退出码 0）" ;;
    3)   log "巡游中止：激光数据缺失或过期（退出码 3）" ;;
    4)   log "巡游中止：里程计数据缺失或过期（退出码 4）" ;;
    5)   log "巡游中止：前方障碍进入停车阈值（退出码 5）" ;;
    6)   log "巡游中止：超过任务总时限（退出码 6）" ;;
    130) log "巡游被中断（退出码 130）" ;;
    *)   log "巡游异常退出（退出码 ${EXIT_CODE}）" ;;
esac

log "退出码已写入 artifacts/${RUN_ID}/patrol_exit_code.txt"
exit "${EXIT_CODE}"
