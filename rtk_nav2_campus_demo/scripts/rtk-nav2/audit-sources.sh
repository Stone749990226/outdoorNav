#!/usr/bin/env bash
# 位姿/真值来源审计：确认**真值只进离线记录**，不进定位、控制、代价地图、规划任何一个环节。
#
# 这条是第二阶段验收的判据之一，第三阶段 brief §4 也把它列为边界（"真值只用于离线评估"）。
# 做法是查订阅者/发布者名单——**不是**查代码里有没有写，因为"代码里没写"证明不了
# "运行时没人订阅"。做法与第二阶段一致，只是换成第三阶段的话题与节点名。
#
# 用法：scripts/rtk-nav2/audit-sources.sh [run-id]
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
DIR="$(artifact_dir "${RUN_ID}")/audit"
mkdir -p "${DIR}"
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"

log "来源审计（run-id: ${RUN_ID}）→ ${DIR}"

TOPICS=(
    "/ground_truth/odom"
    "/rtk/imu"
    "/rtk/sim_fix"
    "/rtk/camera/depth_image"
    "/rtk/camera/points"
    "/cmd_vel"
    "/global_costmap/costmap"
    "/local_costmap/costmap"
)
for t in "${TOPICS[@]}"; do
    safe="$(printf '%s' "${t}" | tr '/' '_')"
    cexec_t 40 "${NAS} && timeout 25 ros2 topic info -v ${t}" \
        > "${DIR}/topic${safe}.txt" 2>&1 || true
    {
        echo "### ${t}"
        echo "-- 发布者："
        grep -B4 'Publisher count' "${DIR}/topic${safe}.txt" 2>/dev/null | grep 'Node name' | sort -u
        grep -A20 'Publisher count' "${DIR}/topic${safe}.txt" 2>/dev/null | grep 'Node name' | sort -u
        echo "-- 订阅者："
        grep -A60 'Subscription count' "${DIR}/topic${safe}.txt" 2>/dev/null | grep 'Node name' | sort -u
    } > "${DIR}/summary${safe}.txt"
done

# TF 里有没有真值帧（真值不该出现在 TF）
cexec_t 40 "${NAS} && timeout 25 ros2 topic echo --once /tf_static --field transforms 2>/dev/null | grep -o 'child_frame_id: .*' | sort -u" \
    > "${DIR}/tf_static_children.txt" 2>&1 || true
cexec_t 60 "${NAS} && timeout 40 python3 ${WS_DIR}/src/robonix_rtk_nav2/scripts/tf_frames_probe.py" \
    > "${DIR}/tf_frames.txt" 2>&1 || true

{
    echo "=== 审计结论（人工判读要点）==="
    echo "1) /ground_truth/odom 的订阅者应当**只有** rtk_trajectory_logger（离线记录器）；"
    echo "   定位链（absolute_localizer / ekf_local / navsat_transform）与两张 costmap 都不该出现。"
    echo "2) TF 帧列表里不该有 world 或 ground_truth 相关帧。"
    echo "3) /cmd_vel 的发布者应当只有 route_follower。"
    echo "4) 两张 costmap 的订阅者里不该有真值话题的发布者。"
} > "${DIR}/README.txt"

for f in "${DIR}"/summary*.txt; do
    [ -e "${f}" ] || continue
    echo "----- $(basename "${f}") -----"
    cat "${f}"
done
log "审计产物在 ${DIR}"
