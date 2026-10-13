#!/usr/bin/env bash
# **主动构造一次"控制器自己 abort"的真失败**，用来验 `goal_navigator` 不会把它当成
# "被我们替换掉的旧 goal"吞掉（TASK B 的证据 2；reviewer 要求"主动构造、不要等它自己出现"）。
#
# 构造法：车在正常跟随时，**运行时**把 controller_server 的三个速度下限置 0
#   （`FollowPath.desired_linear_vel` / `min_approach_linear_velocity` /
#     `regulated_linear_scaling_min_speed`）。
# 为什么三个都要置 0：只置 `desired_linear_vel` 时车仍会以 `regulated_linear_scaling_min_speed`
# (0.05 m/s) 爬行，20 s 能挪 1 m > 进度门 0.15 m，`SimpleProgressChecker` 不会判失败（实测）。
# 置 0 之后**控制器真的推不动车**——不是伪造判据，abort 是控制器自己判的。
# 车推不动 → 20 s 内位移 < 0.15 m → controller_server 打 ERROR `Failed to make progress`
#   → FollowPath 以 status=6(ABORTED) 结束（**不是被抢占**）。
#
# 判据（两条都要）：① controller_server 的 ERROR 行；② goal_navigator 收到后**不吞**、
# 打 `FollowPath 未成功（status=6 …），触发重规划`，且随后真的再规划。
#
# 用法：scripts/rtk-nav2/induce-follow-failure.sh <run-id> [目标x] [目标y]
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
GX="${2:--2.6}"; GY="${3:--2.6}"
DIR="$(artifact_dir "${RUN_ID}")"
OUT="${DIR}/nav2/taskb_failure_evidence.txt"
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"
ROS_ENV="export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1; ${NAS}"

stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"

setparam() {
    cexec_t 25 "${ROS_ENV}; timeout 20 ros2 param set /controller_server $1 $2"
}
sim_now() {
    [ -f "${DIR}/trajectory.csv" ] || { echo "-1"; return; }
    tail -1 "${DIR}/trajectory.csv" | cut -d, -f1
}

log "1) 给目标 (${GX}, ${GY})，让它先正常走起来"
"${SCRIPT_DIR}/nav-to.sh" "${RUN_ID}" "${GX}" "${GY}" >/dev/null 2>&1 || true
sleep 14
T_BEFORE="$(sim_now)"
log "   置 0 之前：sim=${T_BEFORE} 真值=$(tail -1 "${DIR}/trajectory.csv")"

log "2) 把三个速度下限置 0（控制器将真的推不动车）"
setparam FollowPath.desired_linear_vel 0.0
setparam FollowPath.min_approach_linear_velocity 0.0
setparam FollowPath.regulated_linear_scaling_min_speed 0.0
T_ZERO="$(sim_now)"; W_ZERO="$(date '+%H:%M:%S')"
log "   已置 0（sim=${T_ZERO}，墙钟 ${W_ZERO}）"

log "3) 等进度门（0.15 m / 20 s）判失败，共 60 s"
sleep 60
T_AFTER="$(sim_now)"

P_BEFORE_RESUME="$(tail -1 "${DIR}/trajectory.csv")"
log "4) 恢复三个参数（恢复前真值：${P_BEFORE_RESUME}）"
setparam FollowPath.desired_linear_vel 0.16
setparam FollowPath.min_approach_linear_velocity 0.05
setparam FollowPath.regulated_linear_scaling_min_speed 0.05

log "5) 再等 20 s，确认车能重新走（否则这个实验只证明了'车被我弄停了'）"
sleep 20
T_RESUME="$(sim_now)"
P_AFTER_RESUME="$(tail -1 "${DIR}/trajectory.csv")"

{
    echo "# TASK B 证据 2：主动构造一次真失败 → 确认 goal_navigator 不吞它、并触发重规划"
    echo "# run-id ${RUN_ID}；构造法见 scripts/rtk-nav2/induce-follow-failure.sh 的头注释"
    echo "# 时间锚点（仿真秒）：置 0 前 ${T_BEFORE} / 置 0 ${T_ZERO}（墙钟 ${W_ZERO}）/ 等 60 s 后 ${T_AFTER} / 恢复后再等 20 s ${T_RESUME}"
    echo
    echo "## 一、控制器自己的中止（controller_server 打的 ERROR）"
    grep -a "Failed to make progress" "${DIR}/logs/demo_launch.log" | tail -8
    echo
    echo "## 二、goal_navigator 收到后**没有吞**、并触发重规划"
    grep -a "goal_navigator\]" "${DIR}/logs/demo_launch.log" \
        | grep -a -E "FollowPath 未成功|重规划 #|已被我们替换掉的旧 goal" | tail -14
    echo
    echo "## 三、整段会话的计数"
    for p in "已被我们替换掉的旧 goal" "FollowPath 未成功" "重规划 #" "到达"; do
        printf '%-30s %s\n' "${p}" "$(grep -ac "${p}" "${DIR}/logs/demo_launch.log")"
    done
    echo
    echo "## 四、'恢复后车能重新走'的真值对照（第 2/3 列 = 真值 x,y）"
    echo "恢复**之前**那一刻的真值：${P_BEFORE_RESUME}"
    echo "恢复后再等 20 s 的真值  ：${P_AFTER_RESUME}"
    echo "（两行必须不同，否则这个实验只证明了\"车被我弄停了\"）"
} > "${OUT}" 2>&1

log "证据已写入 ${OUT}"
tail -30 "${OUT}"
