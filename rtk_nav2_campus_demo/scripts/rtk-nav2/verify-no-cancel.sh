#!/usr/bin/env bash
# 复验：**去掉"换路径时自己取消"之后**，重发是否不再抖动、两道门是否真正生效、两段是否仍能到达。
#
# 为什么要专门验这一条：reviewer 从实测数据里发现，我那版"自己取消 → 取消回调里发新 goal"
# 造出了"没有活跃路径"的窗口，而空闲时那条"跳过抑制直接重发"会**绕过时间下限**——
# 实测去 A 那段 62 次重发、`min_resend_interval_s=6.0` 只压住 1 次，重发间隔约 1.5 s。
# 现在改成"不取消、直接发新 goal、让 nav2 抢占"。判据：重发数应显著下降、间隔门记账应明显上升。
#
# 位置与到达**只用纯文件读**（trajectory.csv + 日志），不走 tf2_echo：
# tf2_echo 是死循环，`timeout` 杀掉的是 `ros2 run` 包装、真节点活着并占住 docker exec 的输出管道，
# 会让读位置这一步静默挂住（我为此白等过两轮）。
#
# 用法：scripts/rtk-nav2/verify-no-cancel.sh [run-id]
set -u
cd ~/projects/robonix
source scripts/rtk-nav2/lib.sh
RUN="${1:-${RUN_ID:-nav4-nc1}}"
D="docker exec robonix-rtk-nav2-demo bash -lc"
N="source /opt/ros/humble/setup.bash; source ${WS_DIR}/install/setup.bash; export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1;"
ART="$(artifact_dir "${RUN}")"
T="${ART}/trajectory.csv"
L="${ART}/logs/demo_launch.log"

pos() { tail -1 "${T}" 2>/dev/null | awk -F, '{printf "(%.2f, %.2f)", $2, $3}'; }
# 注意不能写 `grep -c ... || echo 0`：grep 没匹配时**既打印 0、又以非零退出**，
# 于是 `||` 再 echo 一个 0 → 函数返回两行 → `[ "$a" -gt "$b" ]` 报"需要整数表达式"，
# 到达判据永远不成立（我第一版就是这么把"已到达"判成"没到"的）。
arrivals() { grep -ac "到达" "${L}" 2>/dev/null | head -1; }

echo "### ① 停旧栈 / 起新栈 ${RUN}"
if stack_running; then scripts/rtk-nav2/stop-demo.sh 2>&1 | tail -1; fi
sleep 5
scripts/rtk-nav2/start-demo.sh "${RUN}" 2>&1 | grep -aE "就绪|已在运行|未就绪|崩溃" | tail -1
echo "### ② 运行时确认真值（演示配置）"
timeout 40 $D "$N timeout 25 ros2 param get /goal_navigator min_resend_interval_s" 2>/dev/null | tail -1
timeout 40 $D "$N timeout 25 ros2 param get /goal_navigator jitter_length_m" 2>/dev/null | tail -1

leg() {  # $1=gx $2=gy $3=名字
    local gx="$1" gy="$2" name="$3" i before after
    before="$(arrivals)"
    echo "### 第 ${name} 段 → (${gx}, ${gy})；起始 $(pos)"
    timeout 60 scripts/rtk-nav2/nav-to.sh "${RUN}" "${gx}" "${gy}" >/dev/null 2>&1
    for i in $(seq 1 25); do
        sleep 10
        after="$(arrivals)"
        echo "    t=$((i*10))s 位置 $(pos)  到达记录 ${before}→${after}"
        if [ "${after}" -gt "${before}" ]; then
            echo "    ** 第 ${name} 段到达 **"
            return 0
        fi
    done
    echo "    ** 第 ${name} 段 250 s 内没到 **"
    return 1
}

leg -2.6 2.6 A
leg 2.6 2.6 B

echo "### ③ 到达汇总（看重发的两个门有没有真正生效）"
grep -a "到达" "${L}" 2>/dev/null | tail -3
echo "### ④ 重发/抑制/间隔门的记账"
grep -aE "重规划 #|抖动抑制跳过|最小重发间隔|没有活跃的 FollowPath" "${L}" 2>/dev/null | tail -6
echo "### ⑤ controller 的 abort 计数（应显著少于上一版）"
grep -ac "Aborting handle" "${L}" 2>/dev/null
echo "### ⑥ /cmd_vel 发布者数（裁定 2）"
timeout 40 $D "$N timeout 25 ros2 topic info /cmd_vel" 2>/dev/null | sed -n 2p
echo "### 跑完（栈保持运行）"
