#!/usr/bin/env bash
# ★★★ 演示版 · 停栈（**一条命令**）★★★
#
#   用法：  scripts/rtk-nav2/demo-stop.sh            # 常规：按凭据收 ＋ 兜底清扫 ⇒ 容器内归零
#           scripts/rtk-nav2/demo-stop.sh --hard     # ★ 强形态：再**把容器本身停掉** ⇒ 里面必然全 0 ✓
#
# ★ 判据（业主定的）：**容器内相关进程数 = 0** ✓ —— ★ 按 **PID** 数 ✓，✗ 不看 docker ps 的 Up ✓
#   ★★ 特别注意 **lifecycle_manager** ✓：nav2 那个节点**不理 SIGTERM** ✗ ⇒ 必须按 PID 收 ✗
#      （业主上一次就是它没收掉 ✓）
#
# ★ 安全边界（这是本脚本敢做"兜底清扫"的前提 ✓）：
#   本脚本只作用于**演示专用容器**（名字固定 = 演示用 ✓）⇒ 在这个容器里按 PID 清扫
#   **不会误伤别人** ✓（按轮脚本之所以只认凭据、扫杀只报告，是因为它们共用容器 ✓；
#   演示容器是独占的 ⇒ 可以兜底 ✓）。
# ★ 顺序：先走"按凭据收"那条正路（stop-demo.sh ✓）⇒ 再**兜底清扫** ⇒ 最后**核实归零** ✓。
set -uo pipefail
HARD=0
for a in "$@"; do [ "$a" = "--hard" ] && HARD=1; done
[ "${DEMO_STOP_HARD:-0}" = "1" ] && HARD=1
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export CONTAINER_NAME="${CONTAINER_NAME:-${DEMO_CONTAINER:-rtknav2-demo}}"
export ROS_DOMAIN_ID="${DEMO_DOMAIN:-56}"
export GZ_PARTITION="${DEMO_PARTITION:-rtknav2demo}"
export RUN_ID="demo"
CONTAINER="${CONTAINER_NAME}"

# 容器活着的判据：docker inspect 的 State.Running（✗ 不用 ps 的名字匹配 ✗）
if [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null || echo false)" != "true" ]; then
    echo "★ 容器 ${CONTAINER} 没在运行 ⇒ 无需停栈 ✓（相关进程数 = 0 ✓）"
    exit 0
fi

echo "── ① 先走正路：按起栈凭据收（pidfile ＋ 冻结 PID 集合 ＋ 手工登记）"
bash "${HERE}/stop-demo.sh" "${RUN_ID}" || true

echo "── ② 兜底清扫（仅本演示容器内；按 PID，先看目标再动手 ✓）"
# ★ 相关进程的判据：命令行里出现下面任一即算（★ 含 lifecycle_manager ✓）
# ★★ 2026-10-12 修（reviewer 抓到的**假通过** ✗）★★
#   原来这条 PAT 里写的是 `robonix_rtk_nav2`（**下划线** ✓），而业主那轮的两支记录器是
#   `bash /work/scripts/rtk-nav2/container/ps_snapshot.sh` 与 `…/window_health.sh`
#   ⇒ 路径里是 `rtk-nav2`（**连字符** ✗）⇒ **一个词都命不中** ✗✗
#   ⇒ 后果两条：① 它们不被清 ✗；② ★ **它们也不被计入"相关进程数"** ✗ ⇒ 脚本会打印
#     "相关进程数 = 0 ✓" 而它们**还在跑** ⇒ ★ **判据假通过** ✗（"发出信号 ≠ 做成了" ✓）。
#   成因与现场一致：它们 **PPID = 1** ✓（由 setsid/nohup 起 ⇒ **不在 launch 树里** ✓）。
#   ⇒ 修法：① PAT 补上 `ps_snapshot|window_health|rtk-nav2/container` ✓；
#            ② PID 1 与保活的 `sleep infinity` **显式排除** ✓（✗ 不再靠"恰好不含" ✓）。
# ★★ 2026-10-13 第三次修（★ 业主/ reviewer 抓到的漏项 ✗）：**前端那一支不是 ROS 进程** ✗ ★★
#   实测漏的是 `rtknav2-verifyb` 里那五连：`bash -lc → npm run dev → sh -c vite → node …/vite --host 0.0.0.0 → esbuild --service`
#   ⇒ ★ 后果两条 ✓（与上一轮那次**同形** ✓）：★ ① 它们不收 ✗；★ ② ★ 它们**也不被计入**"相关进程数" ✗
#     ⇒ ★ 脚本会报"相关进程数 = 0 ✓"而它们**还在跑** ✗ —— ★ 而且 ★ **它们占着端口** ✗（本例 5173 ✓）
#       ⇒ ★ 业主起来后开 localhost:5173 拿到的是**旧的** ✗
#   ★ 定式 ✓：★ "按名字枚举"这个形态**已经连漏三轮** ✓（记录器 ✓、sfp ✓、前端 ✓）
#     ⇒ ★ 改完**必须拿"活的进程表"对一遍** ✓（✗ 不许只改完就交 ✗）；★ 并且 ★ **报清单、不报个数** ✓
PAT='ign gazebo|gz sim|ruby|rviz2|ros2|nav2|lifecycle_manager|bt_navigator|waypoint_follower|controller_server|planner_server|behavior_server|smoother_server|velocity_smoother|map_server|amcl|ekf_node|navsat_transform|robonix_rtk_nav2|rtk-nav2/container|ps_snapshot|window_health|parameter_bridge|robot_state_publisher|geo_route|spherical|depth_points|scan_points|imu_|rviz|zmq|npm|vite|esbuild|node|frontend|tail -F'
# ★ 取号放**宿主侧**做：`docker exec … | ps | awk`（✗ 容器里不拼嵌套引号 ✓ —— 本项目老坑 ✓）
#   并且 ★ **动作与核实用同一套取号函数** ✓ ⇒ 两处判据必然一致 ✓
# ★★ 2026-10-12 第二次修（reviewer 又抓到一个漏项 ✗）★★
#   漏的是 PID 10008 `static_transform_publisher … --child-frame-id robot_rtk/base_footprint/navsat_sensor
#   … -r __node:=navsat_sensor_frame_tf` ✗ —— 它的**参数里一个 PAT 词都没有** ✗。
#   （它的兄弟 10003 `imu_sensor_frame_tf` 之所以命中，**纯属节点名里恰好有 `imu_`** ✓✗）
#   ⇒ ★ 结论：**按名字枚举永远会漏** ✗ ⇒ 除了 PAT，再把 ★ **"栈根的后代"整棵并进来** ✓
#     ⇒ 节点名再古怪也跑不掉 ✓（栈根 = 命令行含 `ros2 launch` 的那个 ✓）。
#   ★ 规矩（立下来 ✓）：**PAT 这类按名字枚举的目标集，改完必须拿"活的进程表"对一遍** ✗ ——
#     这台机器上已经**两轮各抓到一个漏项** ✓（两支记录器 ✓、这一支 sfp ✓）⇒ 两头标定是必做步 ✓。
ps_left() {
    docker exec "${CONTAINER}" bash -lc 'ps -eo pid,ppid,stat,args' 2>/dev/null \
      | /bin/grep -Ev '^\s*PID' \
      | awk -v p="${PAT}" '
          { pid[NR]=$1; ppid[NR]=$2; stat[NR]=$3; line[NR]=$0; n=NR
            if ($0 ~ /ros2 launch/) root[$1]=1 }
          END {
            changed=1
            while (changed) {                    # 反复传播：把栈根的**整棵后代**标出来
              changed=0
              for (i=1;i<=n;i++) {
                if (pid[i]==1) continue
                if ((ppid[i] in root) || (ppid[i] in desc)) { if (!(pid[i] in desc)) { desc[pid[i]]=1; changed=1 } }
              }
            }
            for (i=1;i<=n;i++) {
              if (pid[i]==1) continue                       # ★ 容器 init：绝不碰
              if (line[i] ~ /sleep infinity/) continue      # ★ 保活用的 sleep：绝不碰
              if (stat[i] ~ /Z/) continue                   # 僵尸已死 ⇒ 不是活进程
              if ((line[i] ~ p) || (pid[i] in desc)) print line[i]
            }
          }'
}
sweep() {
    local sig="$1" pid
    ps_left | awk '{print $1}' | while read -r pid; do
        [ -n "${pid}" ] || continue
        echo "   收到 PID ${pid}（信号 ${sig}）：$(docker exec "${CONTAINER}" bash -lc "ps -o args= -p ${pid}" 2>/dev/null | cut -c1-90)"
        docker exec "${CONTAINER}" bash -lc "kill -${sig} ${pid}" 2>/dev/null || true
    done
}
# ★ 收前清单（★ 放在函数定义**之后** ✓ —— 我第一次插在定义之前 ⇒ ps_left: command not found ✗
#   ★ 而它当时还打印了「（空 ⇒ 无需清扫 ✓）」✗ ⇒ ★ 假的「干净」 ✓ —— 这正是本项目那条「守卫谎报」✗）
declare -F ps_left >/dev/null || { echo "✗ ps_left 未定义 ⇒ 取号不可用 ⇒ 停手（✗ 不假装收干净 ✗）"; exit 1; }
echo "   ★ **收前清单**（★ 报清单、✗ 不报个数 ✗）："
BEFORE="$(ps_left)"
if [ -n "${BEFORE}" ]; then printf '%s\n' "${BEFORE}" | sed 's/^/     /'; else echo "     （取号成功且为空 ⇒ 真的无需清扫 ✓）"; fi

sweep TERM
sleep 3
sweep KILL
sleep 1

echo "── ③ 核实：★ **收后清单**（★ 与清扫同一套取号 ✓；★ 列出来看 ✓，✗ 不只报个数 ✗）"
LEFT="$(ps_left)"
N="$(printf '%s\n' "${LEFT}" | grep -c . || true)"
# ★ 清掉陈旧 ros2 CLI daemon（栈停了它还挂着 ⇒ 之后用命令行查话题会拿到空结果 ✗）
docker exec "${CONTAINER}" bash -lc 'source /opt/ros/humble/setup.bash 2>/dev/null; ros2 daemon stop >/dev/null 2>&1; true' || true
docker exec "${CONTAINER}" bash -lc 'rm -rf /tmp/ign-* /tmp/gz-* /dev/shm/ign* /dev/shm/gz* 2>/dev/null; true' || true

if [ "${N}" = "0" ]; then
    echo "     （空 ⇒ 相关进程数 = 0 ✓；含 lifecycle_manager ✓；陈旧 ros2 daemon 已清 ✓）"
else
    echo "✗ 还有 ${N} 个（就是下面这些）："
    printf '%s\n' "${LEFT}" | sed 's/^/    /'
fi

# ★★★ 强形态（--hard）：★ 把**容器本身**停掉 ⇒ 里面的进程**必然全归零** ✓ ★★★
#   ★ 为什么要有它 ✓（业主 2026-10-13 原话："最终 stop 脚本要能完全关闭" ✓）：
#     ★ 前两条（按凭据 ＋ 按名字兜底）都是**枚举** ✗ ⇒ 枚举**永远可能漏** ✓（本脚本已连漏三轮 ✓）
#     ★ 而"停容器"**不是枚举** ✓ ⇒ ★ 它是一个**封闭**的保证 ✓（容器不跑 ⇒ 里面不可能有活进程 ✓）
#   ★ 代价（实测 ✓）：下次起栈会多花"把容器拉起来"的那一小段 ✓ —— `ensure_container` 对**已存在**的
#     容器走的是 `docker start` ✓（✗ 不是 `docker run` 重建 ✗）⇒ 实测值见说明/汇报 ✓
if [ "${HARD}" = "1" ]; then
    echo "── ④ 强形态：停容器 ${CONTAINER}（★ 停掉它 ⇒ 里面必然全 0 ✓）"
    docker stop "${CONTAINER}" >/dev/null 2>&1 || true
    _run="$(docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null || echo missing)"
    if [ "${_run}" = "false" ]; then
        echo "★ 容器已停 ✓（State.Running=false ✓）⇒ ★ 容器内进程**必然为 0** ✓"
        echo "   ★ 下次起栈：bash scripts/rtk-nav2/demo-start.sh（会 docker start 这个容器 ✓）"
        [ "${N}" = "0" ] && exit 0 || exit 0
    else
        echo "✗ 容器没停成（State.Running=${_run} ✗）⇒ 仍按上面的清单为准 ✗"
        exit 1
    fi
fi

[ "${N}" = "0" ] && exit 0 || exit 1
