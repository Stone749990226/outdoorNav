#!/usr/bin/env bash
# 复验脚本（第四步判据 4/6 的到达）：验证 `goal_navigator.tick()` 触发顺序修好之后，
# 车能不能真的走到 A，而不是走两米就停住。
#
# 为什么要写成"一条命令跑完"：停栈 + 编译 + 起栈 + 等到达总共十几分钟，
# 中间任何一步在交互式 ssh 里被超时打断都会留下半截状态（我上两次就是这么被坑的）。
# 这个脚本把整段串起来，结论一次性写到日志里。
#
# 用法：scripts/rtk-nav2/verify-arrival.sh
set -u
cd ~/projects/robonix
source scripts/rtk-nav2/lib.sh
RUN="${RUN_ID:-nav4-fix5}"
D="docker exec robonix-rtk-nav2-demo bash -lc"
N="source /opt/ros/humble/setup.bash; source ${WS_DIR}/install/setup.bash; export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1;"

echo "=== ① 停掉在跑的栈（不管是谁起的，本轮要独占容器）==="
if stack_running; then
    scripts/rtk-nav2/stop-demo.sh 2>&1 | tail -2
else
    echo "（当前没有在跑的栈）"
fi
sleep 5

echo "=== ② 编译 ==="
cexec "cd ${WS_DIR} && colcon build --packages-select robonix_rtk_nav2 2>&1 | tail -2"
echo -n "install 里的副本是否含新顺序: "
grep -c "只有它没有欠账时" ${WS_DIR}/install/robonix_rtk_nav2/lib/python3*/site-packages/robonix_rtk_nav2/goal_navigator.py 2>/dev/null || echo "（没找到副本，可能不是拷贝安装）"

echo "=== ③ 起干净栈 ${RUN} ==="
scripts/rtk-nav2/start-demo.sh ${RUN} 2>&1 | grep -aE "就绪|已在运行|未就绪|崩溃" | tail -2

echo "=== ④ 发目标 A (-2.6, 2.6) ==="
timeout 60 scripts/rtk-nav2/nav-to.sh ${RUN} -2.6 2.6 2>&1 | tail -1

echo "=== ⑤ 每 10 s 记真值位置 / 到 A 的距离 ==="
ARRIVED=0
for i in $(seq 1 45); do
    sleep 10
    POS=$($D "$N timeout 12 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null" | grep -a "Translation" | tail -1)
    DIST=$(printf '%s' "${POS}" | awk -F'[][]' '{split($2,a,","); dx=a[1]+2.6; dy=a[2]-2.6; printf "%.2f", sqrt(dx*dx+dy*dy)}')
    echo "t=$((i*10))s  ${POS}  到A=${DIST} m"
    if awk -v d="${DIST}" 'BEGIN{ exit !(d+0 < 0.35) }'; then
        echo "** 到达 A（<= 0.35 m）**"
        ARRIVED=1
        break
    fi
done
[ "${ARRIVED}" -eq 1 ] || echo "** 450 s 内没到 A **"

L="artifacts/rtk-nav2/${RUN}/logs/demo_launch.log"
echo "=== ⑥ goal_navigator 日志 ==="
grep -a "goal_navigator\]: " "${L}" | tail -10
echo "=== ⑦ controller_server 日志 ==="
grep -a "controller_server\]: " "${L}" | tail -5
echo "=== ⑧ /cmd_vel 发布者 ==="
$D "$N timeout 20 ros2 topic info /cmd_vel 2>/dev/null" | sed -n 2p
echo "=== ⑨ 结束（栈保持运行，供继续看）==="
