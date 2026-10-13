#!/usr/bin/env bash
# 起栈就绪闸门（2026-10-08 定，reviewer 要求把**相机与激光两个 frame** 都显式列出）。
#
# 为什么需要：本阶段两次栽在"栈看起来起来了、但 TF 树不全"——`ramp7-w`、`ramp7-aa` 都是
# `tilt_frames` 没起来（或没发帧），于是**检测器整轮转不动、mask 静默不更新**，外观与
# "场景干净"完全一样。只查"图有没有发"不够：图照发、相机 TF 没了同样致命。
# 同理只查 `map→base_footprint` 也不够：那说明定位在跑，但传感器帧可能没发。
#
# 用法：scripts/rtk-nav2/check-readiness.sh <run-id> [最多等秒数，默认 180]
set -u
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="${1:?用法: check-readiness.sh <run-id> [最多等秒数]}"
WAIT="${2:-180}"
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"

pose_frames=0
end=$(( $(date +%s) + WAIT ))
while [ "$(date +%s)" -lt "$end" ]; do
    nodes="$(cexec_t 20 "${NAS} && ros2 node list" 2>/dev/null | tr '\n' ' ')"
    have_tilt=no; case "$nodes" in *tilt_frames*) have_tilt=yes;; esac
    have_drop=no; case "$nodes" in *depth_drop*) have_drop=yes;; esac
    cam="$(cexec_t 20 "${NAS} && ros2 run tf2_ros tf2_echo base_footprint robot_rtk/base_footprint/depth_camera 2>&1 | grep -m1 Translation" 2>/dev/null)"
    las="$(cexec_t 20 "${NAS} && ros2 run tf2_ros tf2_echo base_footprint robot_rtk/base_footprint/near_lidar 2>&1 | grep -m1 Translation" 2>/dev/null)"
    nf="$(cexec_t 15 "wc -l < /work/artifacts/rtk-nav2/${RUN_ID}/regime/frames.jsonl 2>/dev/null" 2>/dev/null | tr -d ' ')"
    pose_frames=$(( pose_frames + 1 ))
    log "[$pose_frames] tilt_frames=$have_tilt depth_drop=$have_drop frames=${nf:-0}"
    log "      camera_tf=[${cam:-缺}] lidar_tf=[${las:-缺}]"
    if [ "$have_tilt" = yes ] && [ "$have_drop" = yes ] && [ -n "$cam" ] && [ -n "$las" ] \
       && [ "${nf:-0}" -gt 5 ] 2>/dev/null; then
        log "就绪闸门：通过 ✓（节点齐、相机与激光两个 frame 都可查、frames.jsonl 有 ${nf} 行）"
        exit 0
    fi
    sleep 6
done
log "就绪闸门：**不通过** ✗ —— 缺项见上面逐拍输出；处置：停后多等几秒或重起一次栈（本阶段出现过两次）"
log "  （排查顺序：tilt_frames 有没有起来 → 它的日志有无 ParameterNotDeclared/异常 → 定位是否起了 map→base_footprint）"
exit 1
