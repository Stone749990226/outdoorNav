#!/usr/bin/env bash
# 复验**演示配置**（重规划开着、阈值抬到日常抖动带之上）能否走完 A → B 两段。
#
# 为什么要单独跑这个：前面那次"走到点"是在 `min_resend_interval_s=3600`（重发几乎全关）下测的，
# 那只证明了"没有 churn 就能走到"。**演示时重规划是开着的**，必须证明开着也能走到，
# 否则演示配置等于没验。两段都跑，因为"给 A 到 A、再给 B 到 B"才是要演示的闭环。
#
# 用法：scripts/rtk-nav2/verify-two-legs.sh [run-id]
set -u
cd ~/projects/robonix
source scripts/rtk-nav2/lib.sh
RUN="${1:-${RUN_ID:-nav4-fix6}}"
N="source /opt/ros/humble/setup.bash; source ${WS_DIR}/install/setup.bash; export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1;"

cex() { timeout 60 docker exec robonix-rtk-nav2-demo bash -lc "${N} $1" 2>/dev/null; }
pose() { cex "timeout 25 ros2 run tf2_ros tf2_echo map base_footprint" | grep -a Translation | tail -1; }
dist() { printf '%s' "$1" | awk -F'[][]' -v gx="$2" -v gy="$3" '{split($2,a,","); dx=a[1]-gx; dy=a[2]-gy; printf "%.2f", sqrt(dx*dx+dy*dy)}'; }

echo "### ① 停旧栈"
if stack_running; then scripts/rtk-nav2/stop-demo.sh 2>&1 | tail -1; else echo "（没有在跑的栈）"; fi
sleep 5

echo "### ② 起新栈 ${RUN}"
scripts/rtk-nav2/start-demo.sh "${RUN}" 2>&1 | grep -aE "就绪|已在运行|未就绪|崩溃" | tail -1

echo "### ③ 确认演示配置真的生效（运行时真值，不是看文件）"
cex "timeout 25 ros2 param get /goal_navigator jitter_lateral_m"
cex "timeout 25 ros2 param get /goal_navigator jitter_length_m"
cex "timeout 25 ros2 param get /goal_navigator min_resend_interval_s"

leg() {  # $1=gx $2=gy $3=名字
    local gx="$1" gy="$2" name="$3" i p d
    echo "### 第 ${name} 段：发目标 (${gx}, ${gy})"
    timeout 60 scripts/rtk-nav2/nav-to.sh "${RUN}" "${gx}" "${gy}" 2>&1 | tail -1
    for i in $(seq 1 20); do
        sleep 12
        p="$(pose)"
        [ -n "${p}" ] || continue
        d="$(dist "${p}" "${gx}" "${gy}")"
        echo "    t=$((i*12))s ${p}  到${name}=${d} m"
        if awk -v d="${d}" 'BEGIN{exit !(d+0 < 0.35)}'; then
            echo "    ** 第 ${name} 段到达（<=0.35 m）**"
            return 0
        fi
    done
    echo "    ** 第 ${name} 段 240 s 内没到 **"
    return 1
}

leg -2.6 2.6 A
leg 2.6 2.6 B

L="$(artifact_dir "${RUN}")/logs/demo_launch.log"
echo "### ④ 到达日志"
grep -a "到达" "${L}" 2>/dev/null | tail -3
echo "### ⑤ goal_navigator 末尾（看重规划是否在发生、有没有被压住）"
grep -a "goal_navigator\]: " "${L}" 2>/dev/null | tail -8
echo "### ⑥ controller 末尾"
grep -aE "controller_server.*(ERROR|Aborting)" "${L}" 2>/dev/null | tail -4
echo "### ⑦ /cmd_vel 发布者数（裁定 2）"
cex "timeout 25 ros2 topic info /cmd_vel | sed -n 2p"
echo "### 跑完（栈保持运行）"
