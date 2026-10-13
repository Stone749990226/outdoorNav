#!/usr/bin/env bash
# 定位"走一段就停住"：**先把重规划churn 彻底关掉**，只发一条路径、全程不重发，看车能不能走到目标点。
#
# 为什么要这么切：goal_navigator 每次重规划都会换掉在走的 FollowPath，而 nav2 收到新 goal 会
# abort 旧 handle —— 也就是车每隔 1.5~4 s 就被打断一次。于是有两种可能：
#   甲）churn 就是"停住"的原因  → 关掉 churn 车就能一路走到；
#   乙）控制器/代价地图本身走不动 → 关掉 churn 也到不了（那就得往控制器方向查）。
# `min_resend_interval_s` 设成一个很大的值 = 第一条路径发出后所有重发都被"最小重发间隔"压住，
# 正是"甲/乙"的判别实验，不用改一行代码。
#
# 用法：scripts/rtk-nav2/diag-single-path.sh <run-id> <x> <y>
set -u
cd ~/projects/robonix
source scripts/rtk-nav2/lib.sh
RUN="${1:?用法: diag-single-path.sh <run-id> <x> <y>}"
GX="${2:?}"; GY="${3:?}"
D="docker exec robonix-rtk-nav2-demo bash -lc"
N="source /opt/ros/humble/setup.bash; source ${WS_DIR}/install/setup.bash; export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1;"

# **外层也要加 timeout**：里面那个 `timeout 30` 只管容器内的 ros2 进程，
# 而 `docker exec` 这层在机器重载（实测 load 27 / 40 用户）时会自己卡住，
# 卡住就没有任何超时能救——我第一次跑就死在"读起始位姿"这一步，日志里只剩标题。
cex() { timeout 60 $D "$N $1" 2>/dev/null; }

pose() { cex "timeout 30 ros2 run tf2_ros tf2_echo map base_footprint" | grep -a Translation | tail -1; }
dist() { printf '%s' "$1" | awk -F'[][]' -v gx="${GX}" -v gy="${GY}" '{split($2,a,","); dx=a[1]-gx; dy=a[2]-gy; printf "%.2f", sqrt(dx*dx+dy*dy)}'; }

stack_running || { echo "** 栈没在运行 **"; exit 1; }

echo "=== 起始位姿 ==="
P0="$(pose)"; echo "${P0:-（读不到位姿——机器太忙，结果是无效的，别当证据用）}"
echo "起始到目标 (${GX},${GY}) 的距离: $(dist "${P0}") m"

echo "=== 关掉重发：min_resend_interval_s = 3600 ==="
cex "timeout 30 ros2 param set /goal_navigator min_resend_interval_s 3600.0" | tail -1
cex "timeout 30 ros2 param get /goal_navigator min_resend_interval_s" | tail -1

echo "=== 发目标 (${GX},${GY})，然后每 15 s 记一次真值位置 ==="
timeout 60 scripts/rtk-nav2/nav-to.sh "${RUN}" "${GX}" "${GY}" 2>&1 | tail -1

ARRIVED=0
LAST="${P0}"
for i in $(seq 1 32); do
    sleep 15
    P="$(pose)"
    [ -n "${P}" ] && LAST="${P}"
    D2="$(dist "${LAST}")"
    echo "t=$((i*15))s  ${LAST}  到目标=${D2} m"
    if awk -v d="${D2}" 'BEGIN{ exit !(d+0 < 0.35) }'; then
        echo "** 到达（<= 0.35 m）**"
        ARRIVED=1
        break
    fi
done
[ "${ARRIVED}" -eq 1 ] || echo "** 480 s 内没到（最终距离见上）**"

L="$(artifact_dir "${RUN}")/logs/demo_launch.log"
echo "=== goal_navigator（看重发是否真被压住）==="
grep -a "goal_navigator\]: " "${L}" 2>/dev/null | tail -8
echo "=== controller（看 abort 原因）==="
grep -aE "controller_server.*(ERROR|Aborting|Passing new path|Received a goal)" "${L}" 2>/dev/null | tail -8
echo "=== 恢复 min_resend_interval_s = 2.0 ==="
cex "timeout 30 ros2 param set /goal_navigator min_resend_interval_s 2.0" | tail -1
echo "=== 结束 ==="
