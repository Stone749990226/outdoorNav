#!/usr/bin/env bash
# 在干净进程中加载已保存的栅格地图并显示，验证“保存的地图可以重新加载”。
#
# 用法：
#   scripts/stop-demo.sh                    # 先停掉 slam_toolbox，避免 /map 冲突
#   scripts/load-map.sh <run-id>
#
# 行为：
#   1. 用 nav2_map_server 的 map_server 加载 map.yaml（不依赖 slam_toolbox 状态）；
#   2. 打印 /map 的元数据（分辨率/宽高/原点）作为机器可读证据；
#   3. 启动 RViz 显示地图并截图。
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
RUN_DIR="${PROJECT_DIR}/artifacts/${RUN_ID}"
MAP_YAML="${RUN_DIR}/map/map.yaml"

[ -f "${MAP_YAML}" ] || die "找不到 ${MAP_YAML}，请先运行 scripts/save-map.sh"
if map_server_running; then
    die "检测到上一次的地图加载进程仍在运行，请先执行 scripts/stop-demo.sh"
fi

ensure_container
if demo_stack_running; then
    die "演示栈仍在运行（slam_toolbox 也在发布 /map）。请先执行 scripts/stop-demo.sh"
fi

fix_artifacts_ownership
ensure_run_dirs "${RUN_ID}"

log "在干净进程中加载地图：${MAP_YAML}"
cexec_bg "${ARTIFACTS_DIR}/${RUN_ID}/logs/load_map_launch.log" \
    "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && \
     ros2 launch robonix_slam_demo load_map.launch.py map_yaml:='${ARTIFACTS_DIR}/${RUN_ID}/map/map.yaml'"

log "等待 map_server 激活并发布 /map（最长 60 秒）"
ACTIVE=0
for _ in $(seq 1 30); do
    if cexec "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic echo --once /map --field info.resolution" 2>/dev/null | grep -qE '^[0-9]'; then
        ACTIVE=1
        break
    fi
    sleep 2
done
[ "${ACTIVE}" -eq 1 ] || die "map_server 未能在 60 秒内发布 /map，日志见 artifacts/${RUN_ID}/logs/load_map_launch.log"

log "抓取 /map 元数据"
cexec "source /opt/ros/humble/setup.bash && \
       timeout 20 ros2 topic echo --once --field info --full-length /map" \
    > "${RUN_DIR}/logs/loaded_map_info.yaml" 2>&1 || true
cat "${RUN_DIR}/logs/loaded_map_info.yaml"

sleep 5
capture_screen "${RUN_DIR}/screenshots/06_loaded_map.png" \
    && log "已抓取加载截图 06_loaded_map.png"

cat <<EOF

地图加载成功。
  元数据：${RUN_DIR}/logs/loaded_map_info.yaml
  截图：  ${RUN_DIR}/screenshots/06_loaded_map.png
  说明：  与保存前的 map/map_metrics.json 对比分辨率、宽高与原点是否一致。
  停止：  scripts/stop-demo.sh
EOF
