#!/bin/bash
# check-dockerfile-pair.sh —— 守住「Dockerfile.arm64 与 Dockerfile.amd64 只差架构」这条不变式 ✓
#
# ★ 为什么要有它（2026-10-10 ✓）：x86 侧新增的 `docker/Dockerfile.amd64` 是 `Dockerfile.arm64` 的
#   **只换架构**镜像 ✓（★ 场景语义/包清单/插件/GUI 配置一字不改 ✓ —— ★ 这正是"x86 也走 Fortress"这条
#   裁定的落地形态 ✓）。★ 但两个文件放在一起 ⇒ ★ **会各自漂移** ✗：改了一边忘了另一边 ⇒
#   ★ 两条路线**语义不同**、★ 而**没有任何东西会报错** ✗（★ 最坏那类 ✓）。
#   ⇒ ★ 本脚本把"只差架构"从**约定**变成**可跑的判据** ✓（★ 规矩落到路径上，不落到记性上 ✓）。
#
# ★ 比法（★ 关键：比**有效行**，不比注释 ✗）：
#   ① 去掉注释行与空行 ✓（★ 头部注释本来就该各写各的架构说明 ✓ —— 实测两文件有效行 57 : 57 ✓）
#   ② 把 `arm64` 与 `amd64` 都归一成 `ARCH` ✓
#   ③ 要求两边**逐字节相同** ✓
#
# 用法: check-dockerfile-pair.sh [仓库根]     ★ 退出码: 0 = 只差架构 ✓ ｜ 1 = 有别的差异 ✗ ｜ 2 = 用法/文件缺
set -u
ROOT="${1:-$(cd "$(dirname "$0")/.." && pwd)}"
A="$ROOT/docker/Dockerfile.arm64"
B="$ROOT/docker/Dockerfile.amd64"
[ -f "$A" ] || { echo "✗ 缺 $A"; exit 2; }
[ -f "$B" ] || { echo "✗ 缺 $B"; exit 2; }

norm() { command grep -vE '^[[:space:]]*(#|$)' "$1" | sed -e 's/arm64/ARCH/g' -e 's/amd64/ARCH/g'; }

na=$(norm "$A" | wc -l); nb=$(norm "$B" | wc -l)
echo "  有效行（去注释/空行）：arm64 = $na ｜ amd64 = $nb"
D=$(command diff <(norm "$A") <(norm "$B"))
if [ -z "$D" ]; then
    echo "  ✓ 归一化后**逐字节相同** ⇒ ★ 只差架构 ✓（共 $na 行有效内容，全部一致 ✓）"
    exit 0
else
    echo "  ✗ 归一化后仍有差异 ⇒ ★ 两个文件**语义已经漂了** ✗（★ 改一边忘了一边 ✓）"
    echo "$D" | sed 's/^/    /'
    exit 1
fi
