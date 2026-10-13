#!/usr/bin/env bash
# 键盘遥控 TurtleBot3（teleop_twist_keyboard），发布到 /cmd_vel。
#
# 用法：scripts/teleop.sh
# 需要在能接受键盘输入的终端里运行。速度上限已限制在 0.15 m/s / 0.8 rad/s，
# 便于低速验证模型、传感器与速度指令链路。
#
# 按键：i 前进  , 后退  j 左转  l 右转  k 停止  q/z 增减速度
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

ensure_container
demo_stack_running || die "演示栈未运行，请先执行 scripts/start-demo.sh"
[ -t 0 ] || die "需要交互式终端才能键盘遥控"

log "进入键盘遥控（Ctrl-C 退出）"
docker exec -it "${CONTAINER_NAME}" bash -lc '
source /opt/ros/humble/setup.bash
source /work/demo_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p speed:=0.15 -p turn:=0.8 -p use_sim_time:=true'
