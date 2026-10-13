#!/usr/bin/env bash
# **TASK A ablation 的"工况覆盖"行驶**：让相机把"朝外的四个方向"都扫过。
#
# 为什么不能只说"走了一圈"：幽灵格来自相机看到**地面方板板边**时那条轮廓线上的上飘像素，
# 而板边在旧世界位于 ±5.35 m。车在 ±2.6 的方环上时，**只有朝外的方向**才可能看到它。
# 所以本脚本按"四条边各走一段 + 到点后转向下一边"的方式行驶，并在每段到达时
# **把当前仿真时刻写进 legs 文件**，事后由 `outward_coverage.py --legs` 逐段核朝向范围与
# "幽灵可写射线"的条数——覆盖到了/没覆盖到都是可复核的数字。
#
# 用法：scripts/rtk-nav2/ablation-drive.sh <run-id>
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
DIR="$(artifact_dir "${RUN_ID}")"
LEGS_FILE="${DIR}/ablation-legs.txt"
DRIVE_LOG="${DIR}/ablation-drive.log"
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"

stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"

# 四条边：东->北->西->南（与 route.yaml 的方环同向），最后一段回到出生点。
LEGS=("2.6 2.6" "-2.6 2.6" "-2.6 -2.6" "2.6 -2.6")

# 仿真时刻取 `trajectory.csv` 的**最后一行第一列**（纯文件读，不起 ROS——
# 读位姿起 ROS 容易被孤儿进程挂住，见记忆里的教训）。缺文件时写 -1 并在日志里标出来。
sim_now() {
    local f="${DIR}/trajectory.csv"
    [ -f "${f}" ] || { echo "-1"; return; }
    tail -1 "${f}" 2>/dev/null | cut -d, -f1
}

: > "${LEGS_FILE}"
{
    echo "run_id=${RUN_ID}"
    echo "start_sim_t $(sim_now)"
} >> "${LEGS_FILE}"

log "开始 ablation 覆盖行驶（${#LEGS[@]} 段，方环 ±2.6）"
N=0
for leg in "${LEGS[@]}"; do
    N=$((N + 1))
    set -- ${leg}
    X="$1"; Y="$2"
    log "--- 第 ${N} 段：目标 (${X}, ${Y}) ---"
    "${SCRIPT_DIR}/nav-to.sh" "${RUN_ID}" "${X}" "${Y}" >> "${DRIVE_LOG}" 2>&1 || true

    ARRIVED=0
    for i in $(seq 1 60); do
        STATE="$(cexec_t 20 "${NAS} && timeout 12 ros2 topic echo --once /rtk/goal_navigator_state" 2>/dev/null | tr -d '\n')"
        case "${STATE}" in
            *arrived*)
                ARRIVED=1
                # **必须把 break 打出来**：这台机器上"循环静默退化成等满超时"已经发生过
                # （变量被 shell 先展开 → 每轮抛异常 → break 永不触发，而看起来一切正常）。
                log "第 ${N} 段到达（第 ${i} 次探测）：${STATE}"
                break
                ;;
        esac
        sleep 5
    done
    if [ "${ARRIVED}" -eq 0 ]; then
        log "** 第 ${N} 段在 $(printf '%s' "${i}") 次探测内**没有**到达（末态：${STATE}）**"
    fi
    echo "leg ${N} goal ${X} ${Y} arrived=${ARRIVED} arr_sim_t $(sim_now)" >> "${LEGS_FILE}"
done
echo "end_sim_t $(sim_now)" >> "${LEGS_FILE}"

log "覆盖行驶结束，legs 文件：${LEGS_FILE}"
cat "${LEGS_FILE}"
