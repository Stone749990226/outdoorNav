#!/usr/bin/env bash
# 启动二维 SLAM 演示：本地/远程 Ubuntu 图形桌面上的 Gazebo + RViz + slam_toolbox。
#
# 用法：
#   scripts/start-demo.sh [run-id] [-- 额外的 ros2 launch 参数]
#
# 行为：
#   1. 确认显示（默认 :10，即 xjy 的 xrdp 桌面）存在；
#   2. 启动或复用容器 robonix-slam-demo（只挂载本项目目录）；
#   3. 首次运行自动 colcon build；
#   4. 启动 Gazebo + RViz + slam_toolbox + 轨迹记录 + rosbag；
#   5. 等待关键话题就绪并抓第一张截图。
#
# 产物目录：artifacts/<run-id>/
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID=""
LAUNCH_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --) shift; LAUNCH_ARGS=("$@"); break ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) RUN_ID="$1"; shift ;;
    esac
done
RUN_ID="$(resolve_run_id "${RUN_ID}")"

ensure_container
ensure_workspace
fix_artifacts_ownership
unblank_display

ensure_run_dirs "${RUN_ID}"
set_current_run "${RUN_ID}"

if demo_stack_running; then
    log "演示栈已在运行。先执行 scripts/stop-demo.sh 再重新启动。"
    exit 1
fi

# load-map.sh 会启动一个 map_server 与一个 RViz，如果没清理就启动演示，
# 会出现两个 /map 发布者与两个 RViz 同跑。这里加一条与 load-map 对称的检查。
if map_server_running; then
    log "检测到 load-map 启动的 map_server 仍在运行（会与 slam_toolbox 同时发布 /map）。"
    log "请先执行 scripts/stop-demo.sh，再重新运行本脚本。"
    exit 1
fi

log "启动演示栈，run-id = ${RUN_ID}"
cexec "/work/scripts/container/start_stack.sh '${RUN_ID}' ${LAUNCH_ARGS[*]:-}"

log "等待关键话题就绪（最长 180 秒）"
if ! cexec "/work/scripts/container/wait_topics.sh 180" > "${PROJECT_DIR}/artifacts/${RUN_ID}/logs/topics_ready.log" 2>&1; then
    log "话题未全部就绪，日志见 artifacts/${RUN_ID}/logs/topics_ready.log"
    tail -20 "${PROJECT_DIR}/artifacts/${RUN_ID}/logs/topics_ready.log" || true
    exit 1
fi
log "关键话题就绪"

# 再等 slam_toolbox 真正产出第一张地图
log "等待 /map 发布（最长 90 秒）"
for _ in $(seq 1 45); do
    if cexec "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic echo --once /map --field info.resolution" 2>/dev/null | grep -qE '^[0-9]'; then
        log "/map 已发布"
        break
    fi
    sleep 2
done

log "排布 Gazebo / RViz 窗口（左右各半屏）"
cexec "/work/scripts/container/arrange_windows.sh" || true

sleep 3
if capture_screen "${PROJECT_DIR}/artifacts/${RUN_ID}/screenshots/01_startup.png"; then
    log "已抓取启动截图 01_startup.png"
else
    log "截图失败（不影响演示运行）"
fi

# Wayland 会话下两个窗口可能互相遮挡，被压在下面的窗口不会重绘，
# 因此再分别把 Gazebo 和 RViz 抬到最前各拍一张，作为各自的图形证据。
if session_is_wayland; then
    if cexec "/work/scripts/container/raise_window.sh gazebo" >/dev/null 2>&1; then
        sleep 2
        capture_screen "${PROJECT_DIR}/artifacts/${RUN_ID}/screenshots/01b_gazebo_window.png" \
            && log "已抓取 Gazebo 窗口截图 01b_gazebo_window.png"
    fi
    if cexec "/work/scripts/container/raise_window.sh rviz" >/dev/null 2>&1; then
        sleep 2
        capture_screen "${PROJECT_DIR}/artifacts/${RUN_ID}/screenshots/01c_rviz_window.png" \
            && log "已抓取 RViz 窗口截图 01c_rviz_window.png"
    fi
fi

cat <<EOF

演示已启动（run-id: ${RUN_ID}）
  产物目录（宿主机）：${PROJECT_DIR}/artifacts/${RUN_ID}
  产物目录（容器内）：${ARTIFACTS_DIR}/${RUN_ID}
  图形显示：${DEMO_DISPLAY}（远程桌面会话）
  说明：Gazebo 与 RViz 窗口应出现在该远程桌面上。

下一步：
  scripts/run-patrol.sh ${RUN_ID}     # 自动巡游一圈
  scripts/teleop.sh                   # 键盘遥控（在终端里操作）
  scripts/save-map.sh ${RUN_ID}       # 保存地图与 rosbag
  scripts/stop-demo.sh                # 停止演示
EOF
