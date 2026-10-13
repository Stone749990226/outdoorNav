#!/usr/bin/env bash
# 启动竞态回归（宿主机入口）：连续启动 N 次巡逻节点，验证不会误报传感器失效。
#
# 用法：
#   scripts/start-demo.sh <run-id>
#   scripts/test-startup-race.sh <run-id> [次数]
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
COUNT="${2:-20}"

ensure_container
demo_stack_running || die "演示栈未运行，请先执行 scripts/start-demo.sh ${RUN_ID}"
fix_artifacts_ownership
ensure_run_dirs "${RUN_ID}"

log "启动竞态回归 A（带时钟守卫）：连续启动 ${COUNT} 次巡逻节点"
set +e
cexec "/work/scripts/container/startup_race.sh '${RUN_ID}' ${COUNT} guarded" \
    > "${PROJECT_DIR}/artifacts/${RUN_ID}/startup_race_console.log" 2>&1
CODE=$?
set -e
tail -20 "${PROJECT_DIR}/artifacts/${RUN_ID}/startup_race_console.log"

if [ "${CODE}" -ne 0 ]; then
    log "启动竞态回归失败（退出码 ${CODE}）：修复后不应再出现误报失效"
    exit "${CODE}"
fi
log "启动竞态回归 A 通过：${COUNT} 次启动全部正常"

log "启动竞态回归 B（负对照，绕过时钟守卫）：用于证明本用例能检出该竞态"
set +e
cexec "/work/scripts/container/startup_race.sh '${RUN_ID}' ${COUNT} unguarded" \
    > "${PROJECT_DIR}/artifacts/${RUN_ID}/startup_race_unguarded_console.log" 2>&1
set -e
tail -20 "${PROJECT_DIR}/artifacts/${RUN_ID}/startup_race_unguarded_console.log"
log "负对照结束（出现误报失效是预期结果，不影响本脚本的退出状态）"
