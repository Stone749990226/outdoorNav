#!/usr/bin/env bash
# 保存当前 SLAM 会话的产物：栅格地图（YAML + PGM）、slam_toolbox 序列化数据，
# 并收尾 rosbag。每次运行独立目录，不覆盖历史结果。
#
# 用法：scripts/save-map.sh [run-id]
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
ensure_container
demo_stack_running || die "演示栈未运行，请先执行 scripts/start-demo.sh"

RUN_DIR="${PROJECT_DIR}/artifacts/${RUN_ID}"
fix_artifacts_ownership
ensure_run_dirs "${RUN_ID}"

capture_screen "${RUN_DIR}/screenshots/04_before_save.png" || true

log "保存地图与 SLAM 序列化数据（run-id=${RUN_ID}）"
cexec "/work/scripts/container/save_map.sh '${RUN_ID}'" 2>&1 | tee "${RUN_DIR}/logs/save_map.log"

capture_screen "${RUN_DIR}/screenshots/05_after_save.png" || true

log "产物清单："
ls -la "${RUN_DIR}" "${RUN_DIR}/map" || true

# 地图元数据与尺度检查（纯离线，不需要 ROS 环境）
if [ -f "${RUN_DIR}/map/map.yaml" ]; then
    log "地图元数据与尺度检查"
    python3 "${PROJECT_DIR}/demo_ws/src/robonix_slam_demo/robonix_slam_demo/map_metrics.py" \
        "${RUN_DIR}/map/map.yaml" \
        --expected-inner-size "${EXPECTED_INNER_SIZE:-9.85}" \
        --output "${RUN_DIR}/map/map_metrics.json" \
        | tee "${RUN_DIR}/logs/map_metrics.log"
fi

cat <<EOF

保存完成：${RUN_DIR}
  map/map.yaml, map/map.pgm        栅格地图（nav2_map_server 格式）
  map/slam_posegraph.posegraph      slam_toolbox 内部序列化数据
  map/slam_posegraph.data
  bag/run/                          rosbag（clock/scan/odom/tf/map/cmd_vel 等）
  trajectory.csv                    真值 + SLAM 估计轨迹
  screenshots/                      保存前后的界面截图

加载验证：scripts/stop-demo.sh 之后执行 scripts/load-map.sh ${RUN_ID}
EOF
