#!/usr/bin/env bash
# 停止本容器内的第三阶段演示进程。只作用于专用容器 robonix-rtk-nav2-demo。
set -o pipefail
set +u

RUN_ID="${1:-}"
if [ -n "${RUN_ID}" ] && [ -d "/work/artifacts/rtk-nav2/${RUN_ID}/logs" ]; then
    LOG_DIRS="/work/artifacts/rtk-nav2/${RUN_ID}/logs"
else
    LOG_DIRS="$(ls -d /work/artifacts/rtk-nav2/*/logs 2>/dev/null || true)"
fi

kill_tree() {
    # ★ 只杀"我起的那一份"：凭据 = 起栈时写下的 PID（pidfile），从它往下递归到后代。
    #   不认命令行样式——`pkill -f <模式>` 只防"匹配到自己"，**不防"匹配到别的同类进程"**，
    #   在共享容器上最坏的情况就是：停栈把**别人正在用的栈**杀掉（演示当天最怕这个）。
    local pid="$1" sig="$2" kid
    for kid in $(pgrep -P "${pid}" 2>/dev/null || true); do kill_tree "${kid}" "${sig}"; done
    kill -"${sig}" "${pid}" 2>/dev/null || true
}

kill_pidfile() {
    local pidfile="$1" pid
    [ -f "${pidfile}" ] || return 0
    pid="$(cat "${pidfile}" 2>/dev/null || true)"
    if [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null; then
        echo "  停止 $(basename "${pidfile}") pid=${pid}（含其整棵子进程树）"
        kill_tree "${pid}" INT
        for _ in $(seq 1 40); do kill -0 "${pid}" 2>/dev/null || break; sleep 0.25; done
        if kill -0 "${pid}" 2>/dev/null; then
            kill_tree "${pid}" TERM; sleep 2
            kill_tree "${pid}" KILL
        fi
    fi
    rm -f "${pidfile}"
}

echo "[容器内] 停止第三阶段演示进程"
for dir in ${LOG_DIRS}; do
    for pidfile in "${dir}"/*.pid; do [ -e "${pidfile}" ] || continue; kill_pidfile "${pidfile}"; done
done

# ★ 2026-09-21 改（§9.8 / 演示前的硬要求）：**删掉了原来的三大段 `pkill -f <模式>` 清扫**。
#   理由（reviewer 原话）：清理只认"我起的那一份"的凭据（pidfile / 起栈时记下的 PID），
#   不认命令行样式。`[x]` 括号写法只防"匹配到自己"，**不防"匹配到别的同类进程"**——
#   别名一撞就可能把别人正在跑的栈杀掉。
#   残留改为**只报告、不杀**：真需要处理时按列出的 PID 手工决定。
SWEEP="$(ps -eo pid,stat,args | awk '$2 !~ /Z/ && !/awk/ && $0 ~ /ign gazebo|rviz2|ekf_node|navsat_transform|robonix_rtk_nav2|parameter_bridge|ros2 bag|planner_server|lifecycle_manager|local_costmap_host|point_cloud_xyz|robot_state_publisher|rtk_sim|wheel_odom_bridge|absolute_localizer/ {print}')"
if [ -n "${SWEEP}" ]; then
    echo "  ★ 下列进程**不在本次凭据里**（因此没动它们）："
    echo "${SWEEP}" | sed 's/^/    /'
    echo "    → 若确认是本容器上一次留下的残留，请按上面的 PID 手工处理；"
    echo "      若不确定，**先问一句再动手**（共享容器上可能正有别人在跑）。"
fi
sleep 1
echo "[容器内] 剩余相关进程（忽略 defunct）："
# `$3 != "awk"` 不能省：这条 awk 自己的命令行里就含下面那串模式，不过滤的话
# 它会把自己匹配上，于是**每次都报"仍有存活进程"**（第三阶段实测踩到，
# 在容器已经干净的情况下也报，属假警报）。
LEFT="$(ps -eo stat,pid,args | awk '$1 !~ /Z/ && $3 != "awk" && $0 ~ /ign gazebo|rviz2|ekf_node|navsat|robonix_rtk_nav2|parameter_bridge|ros2 bag|planner_server|lifecycle_manager|local_costmap_host|point_cloud_xyz/ {print}')"
if [ -n "${LEFT}" ]; then
    echo "${LEFT}"
    echo "  ** 仍有存活进程 **"
else
    echo "  （无存活进程）"
fi
