#!/usr/bin/env bash
# 停止本容器内的第三阶段演示进程。只作用于专用容器（★ 名字由调用方的 `CONTAINER_NAME` 定 ✓，默认 `rtknav2-demo` ✓）。
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
    # ★★ 2026-10-10：再按**起栈时冻结的 PID 集合**收一遍 ★★
    #   为什么 pidfile 不够：`ros2 launch` 的子进程在中间包装体退出后会 **reparent 到 init** ✗
    #   ⇒ 停栈那一刻从 launch PID walk 树**已经找不到它们** ✗ ——
    #   实测残留正是 `robot_state_publisher` / `ros_gz_bridge_*` / 两个 sensor-frame TF ✗，
    #   而**它们会卡住"起下一个栈"**（起栈前要求容器干净 ✗，我这一轮就被卡了两次 ✗）。
    #   ⇒ 这份集合是**起栈那一刻自己冻结的凭据** ✓ ⇒ 只杀凭据里的 ✓，
    #     **仍然不动"按命令行模式扫杀"那条老路** ⇒ 不会误杀共享容器上别人的进程 ✓。
    # ★★ 2026-10-10：**手工起的进程也要收** ★★（reviewer 裁定 ③）
    #   为什么冻结集合不够：它只覆盖 `ros2 launch` 起出来的那批 ✗；
    #   **手工起的**（`rviz2` ✓、单跑的 `geo_route.py` ✓）不在里面 ✗ ⇒
    #   下次起栈被判"容器不干净"、**直接被挡住** ✗（实测被挡了三次 ✓）。
    #   ⇒ 读 `manual_procs.txt`（由 `register_manual_proc.sh` 登记 ✓，带 pid+lstart ✓），
    #     **先核 lstart 再杀** ✓ —— 挡住 pid 复用 ✗（同 `stack_running` 的道理 ✓）。
    if [ -s "${dir}/manual_procs.txt" ]; then
        echo "  收尾**手工登记**的进程（$(grep -c . "${dir}/manual_procs.txt") 条）"
        while IFS='|' read -r _p _l _lab _rest; do
            [ -n "${_p}" ] || continue
            kill -0 "${_p}" 2>/dev/null || continue
            _now="$(ps -o lstart= -p "${_p}" 2>/dev/null | sed 's/^ *//;s/ *$//')"
            if [ -n "${_l}" ] && [ "${_now}" != "${_l}" ]; then
                echo "    跳过 ${_p}：lstart 不符（pid 被复用 ✗）"
                continue
            fi
            echo "    收 ${_p}（${_lab}）"
            kill_tree "${_p}" TERM
        done < "${dir}/manual_procs.txt"
        sleep 2
        while IFS='|' read -r _p _l _lab _rest; do
            [ -n "${_p}" ] || continue
            kill -9 "${_p}" 2>/dev/null || true
        done < "${dir}/manual_procs.txt"
    fi
    if [ -s "${dir}/demo_launch.pids" ]; then
        echo "  按起栈时冻结的 PID 集合收尾（$(wc -l < "${dir}/demo_launch.pids") 个）"
        while read -r _p; do
            [ -n "${_p}" ] || continue
            kill -0 "${_p}" 2>/dev/null || continue
            kill_tree "${_p}" TERM
        done < "${dir}/demo_launch.pids"
        sleep 2
        while read -r _p; do
            [ -n "${_p}" ] || continue
            kill -0 "${_p}" 2>/dev/null || continue
            kill -9 "${_p}" 2>/dev/null || true
        done < "${dir}/demo_launch.pids"
    fi
done

# ★ 2026-09-21 改（§9.8 / 演示前的硬要求）：**删掉了原来的三大段 `pkill -f <模式>` 清扫**。
#   理由（reviewer 原话）：清理只认"我起的那一份"的凭据（pidfile / 起栈时记下的 PID），
#   不认命令行样式。`[x]` 括号写法只防"匹配到自己"，**不防"匹配到别的同类进程"**——
#   别名一撞就可能把别人正在跑的栈杀掉。
#   残留改为**只报告、不杀**：真需要处理时按列出的 PID 手工决定。
SWEEP="$(ps -eo pid,stat,args | awk '$2 !~ /Z/ && !/awk/ && $0 ~ /ign gazebo|rviz2|ekf_node|navsat_transform|outdoor_nav_bridge|frontend_adapter|robonix_rtk_nav2|parameter_bridge|ros2 bag|planner_server|lifecycle_manager|local_costmap_host|point_cloud_xyz|robot_state_publisher|rtk_sim|wheel_odom_bridge|absolute_localizer/ {print}')"
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
LEFT="$(ps -eo stat,pid,args | awk '$1 !~ /Z/ && $3 != "awk" && $0 ~ /ign gazebo|rviz2|ekf_node|navsat|outdoor_nav_bridge|frontend_adapter|robonix_rtk_nav2|parameter_bridge|ros2 bag|planner_server|lifecycle_manager|local_costmap_host|point_cloud_xyz/ {print}')"
if [ -n "${LEFT}" ]; then
    echo "${LEFT}"
    echo "  ** 仍有存活进程 **"
else
    echo "  （无存活进程）"
fi
