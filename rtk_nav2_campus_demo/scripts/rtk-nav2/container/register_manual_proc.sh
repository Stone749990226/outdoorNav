#!/usr/bin/env bash
# 登记**本轮手工起的进程**（reviewer 2026-10-10 裁定 ②）—— 供停栈端一并收 ✓
#
# **为什么必须有它**：起栈时冻结的 PID 集合只覆盖 **`ros2 launch` 起出来的**那批 ✓；
#   而**手工起的**（`rviz2`、单独跑的 `geo_route.py` ✓）**不在里面** ✗ ⇒ 停栈收不到 ✗ ⇒
#   下次起栈被判"容器不干净"、**直接被挡住** ✗（实测被挡了三次 ✓）。
#   ★ `rviz2` 这类 argv 里**没有 run-id** ✓ ⇒ **按内容认不出来** ✗ ⇒ **登记是唯一的办法** ✓。
#
# 用法（容器内）：register_manual_proc.sh <run_id> <pid> [标签]
#   登记内容 = `pid lstart 标签 argv` ✓ —— **带 lstart** ⇒ 事后能判"这个 pid 是不是当初那个" ✓
set -uo pipefail
RUN="${1:?用法: register_manual_proc.sh <run_id> <pid> [标签]}"
PID="${2:?缺 pid}"
LABEL="${3:-manual}"
OUT="/work/artifacts/rtk-nav2/${RUN}/logs"
mkdir -p "${OUT}"
if [ ! -d "/proc/${PID}" ]; then
    echo "!! pid ${PID} 不存在，未登记" >&2
    exit 1
fi
LSTART="$(ps -o lstart= -p "${PID}" 2>/dev/null | sed 's/^ *//;s/ *$//')"
ARGV="$(tr '\0' ' ' < "/proc/${PID}/cmdline" 2>/dev/null | cut -c1-160)"
printf '%s|%s|%s|%s\n' "${PID}" "${LSTART}" "${LABEL}" "${ARGV}" >> "${OUT}/manual_procs.txt"
echo "已登记 ${PID}（${LABEL}）→ ${OUT}/manual_procs.txt"
