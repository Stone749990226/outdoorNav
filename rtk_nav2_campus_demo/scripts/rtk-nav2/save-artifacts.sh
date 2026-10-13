#!/usr/bin/env bash
# 取第三阶段产物（两张图快照 / 全局图 pgm+yaml / 存读一致性 / 控制权归属）。
# 用法：scripts/rtk-nav2/save-artifacts.sh [run-id]
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"
ensure_run_dirs "${RUN_ID}"
log "取产物（run-id: ${RUN_ID}）"
cexec "/work/scripts/rtk-nav2/container/save_artifacts.sh '${RUN_ID}'"
log "产物目录：$(artifact_dir "${RUN_ID}")"
ls -la "$(artifact_dir "${RUN_ID}")/nav2" 2>/dev/null | tail -14
