#!/usr/bin/env bash
# wait_ready_then_rc.sh —— 跑 `wait_ready.py`，并**把它的退出码落到一个文件里** ✓
#
#   ★ 为什么要有它 ✗（2026-10-14，步骤②）：★ 宿主侧原来**前台**跑 waiter ⇒ 只能等它自己结束 ✗
#     ⇒ 想"一边等、一边盯 demo_launch.log 里的 abort 特征"就做不到 ✓
#   ★ 换成 `docker exec -d`（后台）之后 ⇒ ★ 退出码不会回到宿主 ✗
#     ⇒ 让它**自己写出来** ✓（★ 本仓那条"让运行时把实际值说出来" ✓）
#   ★ ★ 为什么落成**文件**、不内联一大段 ✗：本仓栽过 ✓ ——
#     `cexec_bg` 会把整条命令**拼成一行**再重定向 ⇒ ★ 内联的 `echo $? > f` 会被
#     末尾那个 `> logfile` **吃掉** ✓（redirection 从左到右，最后一个赢 ⇒ `f` 是空的 ✗）
#     ⇒ ★ 复杂命令一律落成文件 ✓（与 `set_spherical_coords.sh` 同一个理由 ✓）
#
# 用法（★ 三个参数都必填 ✓）：
#   wait_ready_then_rc.sh <WS_DIR> <最长等秒数> <rc 文件>
#
# ★★★ 2026-10-14 实测踩到并改掉 ✗✗（★ 第一次连起 5 次时，第 1 轮整整挂了 1124 s ✗）：
#   ★ 原稿写成 `set -u` ＋ 直接 `source /opt/ros/humble/setup.bash` ⇒ ★ **当场死** ✗：
#       `/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable`
#     ⇒ ★ 本脚本 0.0 s 就退出 ⇒ ★ **rc 文件永远不写** ✗
#     ⇒ ★ 宿主侧那份"等 waiter"的循环**看不出区别** ✗（它只看见"凭据还没来"）
#       ⇒ ★ 一路等到 330 s 超时 ⇒ 三次尝试 ⇒ **1124 s 才报失败** ✗✗
#   ★ 教训（★ 本仓那条"**发起 ≠ 做成**"的又一皮 ✓）：★ 我把"waiter 在跑"当成了前提 ✗；
#     ★ 而 waiter **一次都没跑起来**，且**没有任何一处会喊** ✗
#   ⇒ ★ 改法：① 这份脚本**不用 `set -u`** ✓（ROS 的 setup.bash 依赖一堆未定义变量 ✓）
#     ② 宿主侧那条循环**加一条"waiter 到底起没起"的判据** ✓（见 start-demo.sh ✓）
set +u

WS_DIR="${1:?用法: wait_ready_then_rc.sh <WS_DIR> <最长等秒数> <rc 文件>}"
WAIT="${2:?缺最长等秒数}"
RC_FILE="${3:?缺 rc 文件}"

rm -f "${RC_FILE}"
# ★ 先写一条"我起来了"的哨兵 ✓ —— ★ 宿主侧据此判"waiter 到底起没起" ✓
#   （★ 只在真的进到这一步之后才写 ⇒ 起不来就**没有**这条 ✓）
printf 'started %s' "$(date +%s)" > "${RC_FILE}.started"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS_DIR}/install/setup.bash"

python3 "${WS_DIR}/src/robonix_rtk_nav2/scripts/wait_ready.py" "${WAIT}"
rc=$?
printf '%s' "${rc}" > "${RC_FILE}"
exit "${rc}"
