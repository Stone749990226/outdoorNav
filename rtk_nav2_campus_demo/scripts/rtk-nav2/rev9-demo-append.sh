#!/usr/bin/env bash
# ★ 阶段九演示：**在车跑的过程中**往航线文件里**按段追加**点 ✓（栈不重启 ✓）
#
# 用法： rev9-demo-append.sh <run-id> [第一段等待秒数] [第二段等待秒数]
#         默认 = 210 / 420（即先等 210 s 追加到第 2 段，再等 420 s 追加到第 3 段）
#
# ★ 为什么要它（而不是手敲两条 --append）：
#   ★ "运行中接点"这件事的**凭据**是"**追加发生在车还在跑的时候**" ✓ ⇒
#     ★ 必须**定时**、且**与巡游并发** ✓ ⇒ 落成脚本 ✓（✗ 不靠记性 ✗）。
#   ★ 脚本**不碰栈** ✓：它只调 `rev9-stream.sh --append` ✓（那条只做**原子写文件** ✓）。
# ★ 前提：`<run-id>/input/route_seg{1,2,3}.json` 已存在 ✓（由航线分段生成 ✓）。
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"; shift || true
T1="${1:-210}"; T2="${2:-420}"

log "★ 演示追加开始：初始 = 第 1 段；${T1} s 后 → 第 2 段；再 ${T2} s 后 → 第 3 段"
sleep "${T1}"
log "★ 运行中追加 → 第 2 段"
bash "$(dirname "${BASH_SOURCE[0]}")/rev9-stream.sh" "${RUN_ID}" \
    --append input/route_seg1.json input/route_seg2.json 2>&1 | tail -1
sleep "${T2}"
log "★ 运行中追加 → 第 3 段"
bash "$(dirname "${BASH_SOURCE[0]}")/rev9-stream.sh" "${RUN_ID}" \
    --append input/route_seg1.json input/route_seg2.json input/route_seg3.json 2>&1 | tail -1
# ★ 等巡游自己结束 ✓（★ 不替它做决定 ✓）
for _i in $(seq 1 240); do
    pgrep -f "rev9.stream.sh ${RUN_ID}" >/dev/null 2>&1 || { log "★ 巡游已结束"; break; }
    sleep 10
done
log "★ 演示追加脚本结束"
