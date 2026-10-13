#!/usr/bin/env bash
# ★ 周期进程快照（reviewer 2026-10-10 §3："最该做的一步"）★
#
# **为什么**：`campus8-h` 塌栈时我**只知道"跑完时那三个节点没了"** ✗ ——
#   而"**什么时候**没的"才是唯一能**对上别人**的东西 ✓（PID/节点清单只能回答"在不在" ✓）。
#   把时刻钉死 ⇒ 那一刻**同时发生了什么**（别的进程/别的动作/别的会话 ✓）就能对上 ✓。
#   （与"每一轮先取 ⑤ 快照"同一族：**把"事后才发现"变成"当场有记录"** ✓。）
#
# **成本**：一轮 20 min ⇒ 约 120 条快照 ✓ 可忽略。
# **用法**（容器内）：ps_snapshot.sh <输出文件> [间隔秒]
set -uo pipefail
OUT="${1:?用法: ps_snapshot.sh <输出文件> [间隔秒]}"
EVERY="${2:-10}"
mkdir -p "$(dirname "${OUT}")"
while :; do
    {
        printf '### %s  (uptime %s)\n' "$(date -Is)" "$(cut -d' ' -f1 /proc/uptime)"
        # `etimes` = 已运行秒数 ✓ ⇒ 事后能一眼看出"谁是那一刻新起的" ✓
        ps -eo pid,ppid,etimes,lstart,args --no-headers 2>/dev/null
    } >> "${OUT}"
    sleep "${EVERY}"
done
