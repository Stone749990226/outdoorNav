#!/usr/bin/env bash
# ★ 阶段十第 1 步：全局图"残留障碍点"取证 —— 起**只读**探针（★ 不动栈 ✗、不改参数 ✗）
#
# 用法： rev9-residue.sh <run-id> [--seconds 900] [--interval 2.0]
#   ★ 探针只**订阅**三个话题（全局图 ＋ 两个点云 ✓）⇒ ✗ 不加发布者、✗ 不改任何参数 ✓
#   ★ 因此**不需要 gz 分区** ✓（本项目定式：带不带分区 = 看它**碰不碰 gz** ✓，它不碰 ✓）
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"; shift || true
SECONDS_RUN=900; INTERVAL=2.0
while [ $# -gt 0 ]; do
    case "$1" in
        --seconds) SECONDS_RUN="$2"; shift 2 ;;
        --interval) INTERVAL="$2"; shift 2 ;;
        *) shift ;;
    esac
done

C_RUN="${ARTIFACTS_ROOT}/${RUN_ID}"
PROBE="${CONTAINER_WORKDIR}/scripts/rtk-nav2/container/residue_probe.py"

[ -f "${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/run_state.txt" ] \
    || die "没有 ${RUN_ID}/run_state.txt ⇒ ★ 这一轮**没起过栈** ✗（先起栈再挂探针 ✓）"

log "★ 起残留取证探针（**只读** ✓）：每 ${INTERVAL}s 采一次，共 ${SECONDS_RUN}s ⇒ ${C_RUN}/probe"
cexec "mkdir -p ${C_RUN}/probe"
# ★ 后台跑 ✓：`cexec_bg` = `docker exec -d` ＋ 容器内重定向 ⇒ ★ 探针活到 --seconds 自己退 ✓
#   （★ ✗ 不靠外部 kill ✓ —— 会自清的东西别手工收 ✓，本项目定式 ✓）
cexec_bg "${C_RUN}/probe/probe.log" \
    "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && \
     python3 ${PROBE} --out ${C_RUN}/probe --seconds ${SECONDS_RUN} --interval ${INTERVAL}"

sleep 8
if [ -s "${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/probe/probe.log" ]; then
    log "★ 探针已起 ✓（日志 ${RUN_ID}/probe/probe.log ✓；✗ 它不碰栈 ✓）"
    tail -3 "${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/probe/probe.log"
else
    die "★ 探针日志没出现／为空 ⇒ ★ 记成**未起到** ✗（✗ 不当作\"跑起来了\" ✗）"
fi
