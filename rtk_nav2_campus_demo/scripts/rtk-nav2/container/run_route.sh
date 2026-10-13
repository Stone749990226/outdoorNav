#!/usr/bin/env bash
# 在容器内运行路线跟随（前台，退出码透传）。由宿主机 run-route.sh 调用。
# 退出码：0 正常 / 3 相机或代价地图失效 / 5 前方障碍停车 / 6 超时 / 7 定位失效
set -o pipefail
set +u

RUN_ID="${1:?用法: run_route.sh <run_id> [timeout] [report_path]}"
TIMEOUT="${2:-420}"
REPORT="${3:-}"
# **report 路径必须是容器视角**（/work/...）。宿主路径（/home/xjy/...）在容器里不存在，
# 而节点遇到写不了的路径不会报错——它把整份报告打到 stdout 就当完事了，
# 现象是"跑完了但产物目录里没有 route_report.json"（第三阶段实测踩到）。

WS=/work/rtk_nav2_ws
OUT="/work/artifacts/rtk-nav2/${RUN_ID}"
PKG_SHARE="${WS}/install/robonix_rtk_nav2/share/robonix_rtk_nav2"
mkdir -p "${OUT}/logs"

. /opt/ros/humble/setup.bash
. "${WS}/install/setup.bash"

echo "[容器内] 开始第三阶段路线跟随，产物目录 ${OUT}"
# **必须带超时**：没有它，节点挂住时整个测试会静默挂死
# （第二阶段实测挂过 19 分钟，靠人工才发现）。
# **注意类型**：route.yaml 里 total_timeout 是 double，写成 "420" 会被 rclpy 判成
# INTEGER 并抛 InvalidParameterTypeException（实测：节点直接起不来，退出码 1）。
# 加个 .0 让它保持 double。
timeout -s INT -k 15 "${TIMEOUT}" ros2 run robonix_rtk_nav2 route_follower --ros-args \
    --params-file "${PKG_SHARE}/config/route.yaml" \
    -p use_sim_time:=true \
    -p report_path:="${REPORT}" \
    -p total_timeout:="${TIMEOUT}.0"
code=$?
echo "[容器内] 路线跟随节点退出码 ${code}"
# 退出码落文件：宿主编排脚本要读它做对照表（光看 stdout 在人不在场时没用）
echo "${code}" > "${OUT}/route_exit_code.txt"
exit "${code}"
