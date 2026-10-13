#!/usr/bin/env bash
# 命令行给一个目标点（与 RViz 的 2D Goal Pose 等价）。第四步 brief §2 裁定 3。
# 用法：scripts/rtk-nav2/nav-to.sh <run-id> <x> <y> [yaw_deg]
#   ★ yaw_deg 可选（默认 0 = 朝东）：观察航点需要"正对某条边"时必须给 ✓
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"; shift || true
X="${1:?用法: nav-to.sh <run-id> <x> <y> [yaw_deg]}"; Y="${2:?用法: nav-to.sh <run-id> <x> <y> [yaw_deg]}"
YAW_DEG="${3:-0}"   # ★ 必须在这里赋值 ✗：我上一版漏了它 ⇒ `set -u` 下 "YAW_DEG: 未绑定的变量" ⇒ **所有目标都发不出去** ✗
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"
stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"

# ★ 由 yaw(deg) 算四元数 z/w（`z=sin(y/2), w=cos(y/2)`）：观察航点要"正对某条边"✓
QZ=$(awk -v d="${YAW_DEG}" 'BEGIN{printf "%.6f", sin(d*3.141592653589793/360.0)}')
QW=$(awk -v d="${YAW_DEG}" 'BEGIN{printf "%.6f", cos(d*3.141592653589793/360.0)}')
log "发目标 (${X}, ${Y}) yaw=${YAW_DEG}° (z=${QZ}, w=${QW}) 给 /goal_pose"
# **不能用 `--once`**：`ros2 topic pub` 起了一个新发布者就立刻发，而这时它跟 goal_navigator
# 的订阅往往还没匹配上（实测 `ros2 topic info -v /goal_pose` 显示 Publisher count: 0 /
# Subscription count: 1，消息就那么丢了，日志里连"收到目标"都没有——看起来像导航入口没工作）。
# 连发几次跨过发现窗口；goal_navigator 侧对重合目标做了幂等，多发几次不会取消在走的路径。
cexec_t 40 "${NAS} && timeout 25 ros2 topic pub -r 2 -t 4 /goal_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: \"map\"}, pose: {position: {x: ${X}, y: ${Y}, z: 0.0}, orientation: {z: ${QZ}, w: ${QW}}}}'"
sleep 2
cexec_t 30 "${NAS} && timeout 20 ros2 topic echo --once /rtk/goal_navigator_state" 2>&1 | tail -3
log "目标已发出。看车是否开始走：ROS 侧日志搜 goal_navigator / controller_server"
