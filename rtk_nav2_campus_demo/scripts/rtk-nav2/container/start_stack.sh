#!/usr/bin/env bash
# 在容器内启动第三阶段演示：Gazebo + RTK 定位链 + 相机点云 + nav2 两张图 + RViz。
# 用法（容器内）：start_stack.sh <run_id> [额外的 ros2 launch 参数...]
set -eo pipefail
set +u

RUN_ID="${1:?用法: start_stack.sh <run_id> [launch 参数...]}"
shift || true

# ★★ 2026-10-11（阶段十 · 第 0 刀）：工作区名可覆盖 ✓ ★★
#   二分要拿**旧树**在**独立副本**里跑（✗ 不许就地改工作树 ✗）⇒ 整条链路按**工作区名**切 ✓。
#   ★ 由 `start-demo.sh` 用**环境变量前缀**显式带进来 ✓（`docker exec` 不继承宿主 env ✗）。
#   ★ 默认 = `rtk_nav2_ws` ⇒ 不设该变量时**行为逐字不变** ✓。
WS="/work/${WORKSPACE_NAME:-rtk_nav2_ws}"
OUT="/work/artifacts/rtk-nav2/${RUN_ID}"
mkdir -p "${OUT}/logs" "${OUT}/bag" "${OUT}/screenshots" "${OUT}/nav2" "${OUT}/coverage"

. /opt/ros/humble/setup.bash
. "${WS}/install/setup.bash"
cd "${WS}"

echo "架构 $(uname -m)；ROS_DOMAIN_ID=${ROS_DOMAIN_ID}；GZ_PARTITION=${GZ_PARTITION}"

# 1) Gazebo + 定位链 + 点云 + nav2（两张图与规划器）+ 危险距离 + RViz
# Stage-6 runs in an existing container whose baked DISPLAY may belong to another
# desktop session. Only when explicitly set, pass the authenticated display to the
# Gazebo/RViz children. Default demo launch environment is unchanged.
if [ -n "${ROBONIX_DEPTH_DROP_DISPLAY:-}" ]; then
    export DISPLAY="${ROBONIX_DEPTH_DROP_DISPLAY}"
fi

# ★★ 2026-10-10（业主裁定："之前车一直没被正确渲染？优先解决这个问题"）★★
#   **症状**：Gazebo 里车**根本没渲染** —— 日志里反复出现
#     `[Err] Failed to get Ogre item for [model://turtlebot3_description/meshes/bases/burger_base.stl]`
#     `[Err] Failed to load geometry for visual: ..._base_link_visual`
#   ⇒ 车身只剩几个 3~6 cm 的 primitive（相机方块 / 激光圆柱 / IMU、GPS 方块）✓
#     ⇒ ★ 历次截图里"车很小"的**准确说法是"车几乎不存在"** ✗
#       （顺带解释：截图里那些小方块**不是车**，是世界里的 `car_00..car_07` 停车 ✓）
#   **根因**：URDF 里 mesh 写的是 `package://turtlebot3_description/...` ✓，
#     URDF→SDF 转换后成了 `model://turtlebot3_description/...` ✓，
#     而 Gazebo 解析 `model://X/...` 要**在 resource path 的各目录下找 `X/...`** ✗。
#     实测本容器 **`IGN_GAZEBO_RESOURCE_PATH` 与 `GZ_SIM_RESOURCE_PATH` 都是空的** ✓
#     ⇒ `model://turtlebot3_description/...` **必然解析失败** ✓（不是模型文件的问题 ✓）。
#   **修法**：把 ROS 的 share 前缀加进 resource path ✓ ——
#     `/opt/ros/humble/share/turtlebot3_description/meshes/...` **实测存在** ✓
#     ⇒ ★ **模型文件一个字节都不用改** ✓（**运行环境**层的修法 ✓，不是模型改动 ✓）。
#   ★ 两个变量都设：`IGN_GAZEBO_RESOURCE_PATH` 是 Fortress 读的 ✓，
#     `GZ_SIM_RESOURCE_PATH` 是新名 ✓ —— ★ **只设一个可能静默无效** ✗（这正是本项目
#     反复踩的"环境变量没到子进程"那一族 ✓）⇒ 两个都设、且**做去重**（重复起栈不会越加越长 ✓）。
# ★★ 2026-10-10（阶段九）：**再加我们自己的 `worlds/models`** ★★
#   阶段九的世界 `north_campus.sdf` 用 `<include><uri>model://campus_scene</uri></include>` ✓
#   ⇒ Gazebo 要在 resource path 的某个目录下找到 **`campus_scene/`** ✗。
#   ★ 只加 `/opt/ros/humble/share` **不够** ✗ —— 实测起栈报：
#       `[Err] [Server.cc:139] Error Code 13: [.../north_campus.sdf:L48]:`
#       `      Msg: Unable to find uri[model://campus_scene]`
#     ⇒ **世界加载失败** ⇒ 没有 `/world/` ⇒ `ros_gz_sim create` 无限打印
#       "Requesting list of world names" 重试 ✗
#       ★ 现象看起来是"卡在起栈"，其实是**世界根本读不进来** ✓（差一步就会误判成 nav2 的问题 ✗）。
#   ⇒ ★ 把 `worlds/models` 也加进去 ✓（**运行环境层**的修法 ✓，世界/模型文件一个字节不动 ✓）。
#   ★ 仍然**去重** ✓（重复起栈不会越加越长 ✓）。
_add_resource_path() {
    local var="$1"; shift
    local cur="${!var:-}" d
    for d in "$@"; do
        case ":${cur}:" in
            *":${d}:"*) : ;;                                  # 已在里面 ⇒ 不重复加 ✓
            *) cur="${cur:+${cur}:}${d}" ;;
        esac
    done
    export "$var"="$cur"
    echo "  [$var] = ${!var}"
}
WORLDS_MODELS="${WORLDS_MODELS:-${WS}/src/robonix_rtk_nav2/worlds/models}"
# ★★ 2026-10-12：**按轮的开关** —— ★ 只为一个对照轮 ✓（reviewer 批准 ✓）
#   ★ 业主要"完整复现从 6002 移植来的那版，**不应用后面的修改**（含渲染修复）" ✓
#   ★ 而渲染修复就是**上面这两行**里的 `/opt/ros/humble/share` ✓（★ 它让 `model://turtlebot3_description/…`
#     解析得到 ✓；★ 拿掉它 ⇒ 车**又回到不渲染** ✓ —— ★ 物理无影响 ✓、★ 模型文件一个字节不用改 ✓）。
#   ★ 开关形态选**按轮的标记文件** ✗ 不用环境变量：★ `start-demo.sh` 调本脚本时**环境变量是逐条写死的** ✓
#     ⇒ 新变量**传不进来** ✗（改那一行会动到**所有**轮次 ✗）⇒ ★ 用 `${OUT}/.no_render_fix` ✓
#     ⇒ ★ **默认（文件不存在）行为逐字不变** ✓✓
#   ★ ★ **只去掉 `/opt/ros/humble/share`** ✓ —— `${WORLDS_MODELS}` **照加** ✓
#     （★ `north_campus.sdf` 的 `model://campus_scene` 靠它 ✓；★ 顺手拆掉会让世界加载失败 ✗）
#   ★★ 且**把这个模式写进产物** ✓（reviewer 2026-10-12 ✓）：★ `render_fix=off|on` ✓
#     ★ 理由：★ 否则**下一个人读产物分不出这轮是哪种模式** ✗（★ 与今天 `config_used_*`
#       "读者拿到的是上一次的值" 同族 ✓）；★ 而"有没有渲染修复"**属于轮间可比性的七项清单里
#       「配置整套」那一格** ✓ ⇒ ★ **两头都记** ✓（✗ 别只记"关"的那一支 ✗）。
if [ -f "${OUT}/.no_render_fix" ]; then
    echo "  ★ .no_render_fix 在 ⇒ **不加 /opt/ros/humble/share**（车回到不渲染 ✓）；${WORLDS_MODELS} 照加 ✓"
    printf 'render_fix=off   # %s\n' "$(date '+%H:%M:%S')" >> "${OUT}/run_state.txt"
    _add_resource_path IGN_GAZEBO_RESOURCE_PATH "${WORLDS_MODELS}"
    _add_resource_path GZ_SIM_RESOURCE_PATH  "${WORLDS_MODELS}"
else
    printf 'render_fix=on   # %s\n' "$(date '+%H:%M:%S')" >> "${OUT}/run_state.txt"
    _add_resource_path IGN_GAZEBO_RESOURCE_PATH /opt/ros/humble/share "${WORLDS_MODELS}"
    _add_resource_path GZ_SIM_RESOURCE_PATH  /opt/ros/humble/share "${WORLDS_MODELS}"
fi

# ★★ 2026-10-12：**场景去重** —— 场景可能不在本仓里，而在"**外层仓库**"里 ✓
#   背景：并入他们的仓之后，本子树**不再自带** `worlds/models/campus_scene` ✓
#   （★ 否则分支里会有**两份**：他们的 `gazebo_campus/generated/models/campus_scene` ✓
#    与子树自带那份 ✗ ⇒ ★ 两份会**分叉** ✗ —— ★ 他们的 `build_scene.py` 已被改成产修复版 ✓
#    ⇒ 谁跑一次生成器，就**只动他们那份** ✗ ⇒ ★ 这正是"多个版本反复复现"那个病 ✓）
#   ⇒ ★ 场景**只留一份**（他们的那份 ✓），本子树**指向它** ✓
#   ★ ★ **空操作的形态**（★ 关键 ✓）：本仓里 `gazebo_campus/` **不存在** ⇒ 这段**什么都不做** ✓
#     ⇒ ★ **同一份脚本、两处都对** ✓（✗ 不是"分两套分支"✗）
#   ★ 容器内能否看见那层：由 `scripts/rtk-nav2/lib.sh` 起容器时**单独只读挂进来** ✓
#     （★ 挂点是**容器内的** `/opt/rtk_nav2_external_models` ✓ —— ★ 因为 `/work` 挂的是
#      **本子树** ✗、外层**不在挂载里** ✓ ⇒ 不单独挂就**看不见** ⇒ 世界加载失败 ✗
#      ★ 而那个表相是"**看着像卡在起栈**"✗ —— 本文件上面第 51 行起就记着这个坑 ✓）
#   ★ 路径是**容器内**的 ✓（由 lib.sh 只读挂进来 ✓）—— ✗ 别指向 `/work/...` ✗：
#     那样 Docker 会在**宿主的子树里**建中间目录（★ root 属主 ⇒ 脏宿主 ✗，实测踩到 ✓）
EXTRA_MODELS_DIR="/opt/rtk_nav2_external_models"
if [ -d "${EXTRA_MODELS_DIR}" ]; then
    echo "  ★ 外层场景目录在 ⇒ 一并加进资源路径：${EXTRA_MODELS_DIR}"
    _add_resource_path IGN_GAZEBO_RESOURCE_PATH "${EXTRA_MODELS_DIR}"
    _add_resource_path GZ_SIM_RESOURCE_PATH  "${EXTRA_MODELS_DIR}"
else
    echo "  ★ 外层场景目录不在（本仓形态 ✓）⇒ 资源路径保持原样（用本仓的 ${WORLDS_MODELS}）"
fi

# ★★ 2026-10-13：**第三种形态 —— 场景在**仓内**（★ 这正是"并入他们仓之后"那份 ✓）★★
#   ★ 背景 ✓：并入之后，场景在 ★ `<仓库根>/gazebo_campus/generated/models/campus_scene` ✓，
#     ★ 而 `/work` 挂的**就是仓库根** ✓ ⇒ ★ 它在容器里＝ `${CONTAINER_WORK}` 下的 `gazebo_campus/…` ✓
#   ★ ★ 为什么**必须**显式加 ✗：上面两条都盖不到它 ✓ ——
#     ① `${WORLDS_MODELS}` 指 ★ `<WS>/…/worlds/models` ✓ ⇒ ★ 那时**整个目录都没了** ✗
#        （★ 场景去重把它移走了 ✓ —— 见本文件上面那段 ✓）
#     ② `EXTRA_MODELS_DIR` 是"**外层**仓库"的只读挂载 ✓ ⇒ ★ 并入之后**没有外层** ✗
#        （★ `lib.sh` 那个挂载分支带 `[ ! -d "${PROJECT_DIR}/gazebo_campus/…" ]` ✓ ⇒ ★ **仓内形态下它主动跳过** ✓）
#   ⇒ ★ 不加这一步 ⇒ `model://campus_scene` **解析不了** ✗ ⇒ ★ 表相＝**场景空白／像卡在起栈** ✗
#   ★ ★ **空操作的形态** ✓（★ 与上面两条同形 ✓）：本仓里 `gazebo_campus/` **不存在** ⇒
#     这段**什么都不做** ✓ ⇒ ★ **同一份脚本、合并前后都对** ✓（✗ 不是"分两套分支" ✗）
#   ★ ★ 变量名用 **`CONTAINER_WORKDIR`** ✓（★ 与 `lib.sh:27` 同名 ✓）—— ✗ **但容器里拿不到它** ✗：
#     `lib.sh` 的 `cexec` 是 `docker exec … bash -lc` ✓、**不带 `-e`** ✗ ⇒ ★ 宿主 env **不进来** ✗
#     ⇒ ★ 实际走的是**回落值 `/work`** ✓（★ 与 `lib.sh` 那处的值**一致** ✓）。
#   ★ ★ ★ **所以这里与 `lib.sh:27` 是"两处必须同值"的一对** ✗ ⇒ ★ **改一处要改两处** ✗
#     （★ 同形于本仓 `min_obstacle_height`／`odom_topic` 那几处 ✓ —— 只改一处会变成
#      **注释说一套、值写另一套** ✗）
INREPO_MODELS_DIR="${CONTAINER_WORKDIR:-/work}/gazebo_campus/generated/models"
if [ -d "${INREPO_MODELS_DIR}" ]; then
    echo "  ★ 仓内场景目录在（★ 已并入的形态 ✓）⇒ 一并加进资源路径：${INREPO_MODELS_DIR}"
    _add_resource_path IGN_GAZEBO_RESOURCE_PATH "${INREPO_MODELS_DIR}"
    _add_resource_path GZ_SIM_RESOURCE_PATH  "${INREPO_MODELS_DIR}"
else
    echo "  ★ 仓内场景目录不在（★ 未并入的形态 ✓）⇒ 资源路径保持原样"
fi

# ★★ 2026-10-10（阶段九）：可选的**服务端配置**（Fortress 的 systems 插件表 ✓）★★
#   背景：`north_campus.sdf` 是**他们的 Classic world** 改来的 ✓，里面**一个 `<plugin>` 都没有** ✗
#   ⇒ Fortress 下**相机/激光/GPS 全都不出数据** ✗（实测：gz 侧传感器话题数 = 0 ✓）
#   ⇒ 定位链无输出 ⇒ 没有 map 帧 ⇒ nav2 起不来 ⇒ 就绪闸门一直等 ✗。
#   ★ 修法 = 用 `IGN_GAZEBO_SERVER_CONFIG_PATH` 在**运行层**补插件 ✓ ⇒ 世界文件不动 ✓。
#   ★ **默认不设** ✓ —— 只在调用方显式给了 `NORTH_CAMPUS_SERVER_CONFIG` 时才生效 ✓
#     （✗ 不能无条件设 ✗：本文件里的插件表是**替换**默认表的 ✓，
#      而 `campus_100.sdf` 之类**自己在世界里声明了**插件 ⇒ 两边同时声明会**重复加载** ✗）。
if [ -n "${NORTH_CAMPUS_SERVER_CONFIG:-}" ]; then
    if [ -f "${NORTH_CAMPUS_SERVER_CONFIG}" ]; then
        # ★ 两个变量都设 ✓ —— 与资源路径同一族：只设一个可能**静默无效** ✗
        export IGN_GAZEBO_SERVER_CONFIG_PATH="${NORTH_CAMPUS_SERVER_CONFIG}"
        export GZ_SIM_SERVER_CONFIG_PATH="${NORTH_CAMPUS_SERVER_CONFIG}"
        echo "  [server-config] IGN_GAZEBO_SERVER_CONFIG_PATH = ${IGN_GAZEBO_SERVER_CONFIG_PATH}"
    else
        echo "  !! NORTH_CAMPUS_SERVER_CONFIG 指的文件不存在：${NORTH_CAMPUS_SERVER_CONFIG}"
    fi
fi
# ★★ 2026-10-10（reviewer 的假设 + 因果修法）★★
#   **`setsid`**：把 launch 放进**新会话** ⇒ **发起它的那次 ssh / `docker exec` 会话结束时
#   的 SIGHUP 带不走它** ✓。★ 这条直接对上"塌栈"的假设：`ros2 launch` 收到 SIGHUP 会
#   **干净地关掉它管的节点**（`planner_server` / `global_costmap` / `lifecycle_manager_nav2`
#   **正好是它管的那一批** ⇒ **三个一起走** ✓），而 **bridge / `robot_state_publisher` / static TF
#   它管不到 ⇒ 活着** ✓、干净关停**不打印崩溃文本** ✓、launch 退了**日志自然不再写** ✓
#   —— **四条现象一次全中** ✓。（而我这一轮起栈**正是** `ssh "... nohup bash start-campus.sh &"` ✗
#   ⇒ **很可能就是我自己的调用方式把栈带走的** ✗ —— 所以这条要修在**脚本里**、不能靠调用方自觉 ✓。）
#   **`PYTHONUNBUFFERED=1`**：python 节点的 stdout 重定向到文件时是**块缓冲** ✗ ⇒
#   **被 kill 那一刻，缓冲区里最后几行永远不落盘** ✗ ⇒ 这就是"起步后不再写"的来源 ✓
#   （**不是它们没说话** ✓）。==> 以后"**死前最后一句**"就在日志里 ✓。
PYTHONUNBUFFERED=1 setsid nohup ros2 launch robonix_rtk_nav2 demo.launch.py \
    "trajectory_csv:=${OUT}/trajectory.csv" \
    "param_dump_path:=${OUT}/nav2/params_local_costmap_local_costmap.yaml" "$@" \
    > "${OUT}/logs/demo_launch.log" 2>&1 &
# ★ `setsid` 可能在 fork 后才 exec ⇒ `$!` 未必是 launch 的 PID ✗
#   ⇒ **解析出真正的 launch PID**，拿不到就报出来（**不静默记一个错的 PID** ✗）
sleep 2
_LPID="$!"
if ! ps -o lstart= -p "${_LPID}" >/dev/null 2>&1; then
    _LPID="$(pgrep -f 'demo.launch.py' | head -1)"
fi
[ -n "${_LPID}" ] || echo "  !! 解析不出 launch 的 PID（守卫会因此判'没在跑'）"
echo "${_LPID}" > "${OUT}/logs/demo_launch.pid"
# ★ 2026-10-10（reviewer 裁定）：**连 `lstart` 一起记** ✓ ——
#   PID 会被**复用** ✗，只比 PID 过一段时间就会误判（清残留时已踩过同族的坑 ✓）
#   ⇒ 守卫要"PID 活着 **且** 启动时刻与起栈时一致"两条都满足 ✓。
sleep 1   # 让 launch 进程真正起来，lstart 才是它的
ps -o lstart= -p "$(cat "${OUT}/logs/demo_launch.pid")" 2>/dev/null | sed 's/^ *//;s/ *$//' \
    > "${OUT}/logs/demo_launch.lstart"
# ★★ 2026-10-10：**把这一轮 launch 起出来的全部 PID 当场记下来** ★★
#   为什么必须在**起栈时**记、而不是停栈时再走一遍进程树：
#   `ros2 launch` 的子进程在中间包装体退出后会**被 reparent 到 init** ✗ ⇒
#   停栈那一刻从 launch PID 往下 walk **已经找不到它们了** ✗ ——
#   实测残留正是这类：`robot_state_publisher` / `ros_gz_bridge_*` / 两个 sensor-frame TF ✗，
#   而它们**卡住的不只是守卫，是"起下一个栈"本身**（起栈前要求容器干净 ✗）。
#   ⇒ 在树还完整时把 PID 集合冻结成凭据 ✓，停栈时**只按这份凭据杀** ✓
#     （仍然不回到"按命令行模式扫杀"那条老路 ⇒ 不会误杀别人 ✓）。
sleep 6
{ echo "$(cat "${OUT}/logs/demo_launch.pid")"
  pgrep -P "$(cat "${OUT}/logs/demo_launch.pid")" 2>/dev/null || true
  ps -eo pid,ppid --no-headers 2>/dev/null | awk -v root="$(cat "${OUT}/logs/demo_launch.pid")" \
      'BEGIN{n=0} {p[$1]=$2} END{for (q in p){x=q; for(i=0;i<12;i++){ if(x==root){print q; break} if(!(x in p)||p[x]==0||p[x]==x) break; x=p[x]}}}' \
    | sort -n | uniq
} > "${OUT}/logs/demo_launch.pids" 2>/dev/null
echo "  已冻结本轮 PID 集合：$(wc -l < "${OUT}/logs/demo_launch.pids") 个（logs/demo_launch.pids）"
# ★ 周期进程快照（reviewer §3）：把"节点**什么时候**没的"钉死 ✓
#   ⇒ 那一刻同时发生了什么就能对上（别的进程/别的动作/别的会话 ✓）。
#   它自己也在 `.pid` 里 ⇒ 停栈会被一并收走 ✓（不留孤儿 ✓）。
setsid nohup /work/scripts/rtk-nav2/container/ps_snapshot.sh \
    "${OUT}/logs/ps_snapshot.log" 10 >/dev/null 2>&1 &
echo $! > "${OUT}/logs/ps_snapshot.pid"
echo "  周期快照已起（logs/ps_snapshot.log，每 10 s 一行全进程表 ✓）"
# ★ 窗口体检（reviewer 2026-10-10）：**让窗口自己自证** ✓ —— 闸门 (a)(b) 当场可判 ✓
# ★★ 2026-10-10：**不许把 stderr 丢进 `/dev/null`** ✗ ★★
#   实测本轮：`campus8-n1` 只留下 `logs/window_health.pid`（内容 `1831030`）✗、**没有任何 `.jsonl`** ✗，
#   且该 PID 已 GONE、容器内也无实例 ⇒ **脚本开机即死** ✗ ——
#   而死因**随 stderr 一起进了 /dev/null** ✗ ⇒ **不可诊断** ✗（"看不见"又被当成"没发生" ✓）。
#   ⇒ 改落到本轮产物目录里 ✓（**下一轮出问题当场能看见死因** ✓）。
setsid nohup /work/scripts/rtk-nav2/container/window_health.sh \
    "${OUT}/window_health.jsonl" 30 >"${OUT}/logs/window_health.err" 2>&1 &
echo $! > "${OUT}/logs/window_health.pid"
echo "  窗口体检已起（window_health.jsonl：按分区实例数 + 时钟回退计数 ✓）"
# ★ 记下**本轮实际用的 gz 分区** ✓（事后可核"到底串没串台" ✓）——
#   实测教训：三套仿真同分区 ⇒ bridge 收到旧仿真的话题 ⇒ 代价图被别的车写 ✗
# ★★ 把"**前提**"写进产物（reviewer 2026-10-10 §2）★★
#   实测教训：容器里同时跑着**三套同分区**的仿真 ✗ ⇒ 那一轮的 ⑤/位姿读数**不可归属**
#   （观测可能属于**另一套仿真** ✓）⇒ 不是"算错"，是"**前提不成立**" ✓
#   （= 读数有效性第 4 维"前提" ✓，与 `world_matches_model` / `calib_key` 同一族 ✓）
#   ⇒ 记下**起栈那一刻的仿真实例数** ✓ ⇒ **任何一轮都能自证"我是在单仿真下跑的"** ✓，
#     不必依赖"我记得当时是 1" ✓。
# ★ 锚在行首 ✓ —— 否则 `/bin/sh -c ruby /usr/bin/ign gazebo -r …` 这个**包装体**会被一起数上 ✗
#   （实测：真只有 1 套仿真时，宽松写法报 **2** ✗ ⇒ 这个字段会把干净轮误标成"不可归属" ✗）
# ★★ 判据换成"**能数全**"的（reviewer 2026-10-10 抓到的）★★
#   实测：锚定行首那个写法**三次起栈都报 1** ✗ —— 可那时**已经有残留仿真在跑** ✓
#   ⇒ 它匹配的是**某种命令行样式** ✗，换个启动形式就漏 ✓ ⇒ **当守卫会失效** ✗
#   （与今天早些时候 `pgrep -f "ros2 launch …"` 漏裸名进程**同一坑、第二次** ✓）
#   ⇒ 改成**按"加载了本世界文件"数** ✓ —— 与启动形式无关 ✓，能数到残留 ✓。
# ★★ 再按可执行名过滤一层（reviewer 2026-10-10）★★
#   只按 `campus_100.sdf` 数会**把起栈那条 `ros2 launch` 自己也算进去** ✗ ——
#   它的 argv 里就带 `world:=…/campus_100.sdf` 与 `static_world:=…/campus_100.sdf` ✓
#   ⇒ 健康单仿真栈会被读成 **2** ✗ ⇒ "断言 == 1" 会**把健康栈判死** ✗。
#   ⇒ 两步：先要"加载了本世界" ✓、再要"它是 ign gazebo 进程" ✓
#     ⇒ 数的是"**加载了本世界的仿真进程**" ✓（与启动形式无关 ✓、不含 launcher ✓）。
#   ★ 匹配式的**两个方向都会错**（今天两次：`pgrep -f "ros2 launch …"` **漏**裸名 ✗、
#     锚定模式**漏**另一套仿真 ✗、这条**多**数 launcher ✗）⇒ **判据要两头都验** ✓。
_IGN="$(ps -eo args --no-headers 2>/dev/null | grep 'campus_100.sdf' | grep -c '[i]gn gazebo' || echo '?')"
_IGN_LOOSE="$(ps -eo args --no-headers 2>/dev/null | grep -c '[i]gn gazebo' || echo '?')"
{
  echo "gz_partition=${GZ_PARTITION:-${IGN_PARTITION:-（未设，用容器默认 rtknav2 ✗）}}   # $(date '+%H:%M:%S')"
  # ★ **两个数都记**（reviewer 2026-10-10）：两个方向的错法**不对称** ✓ ——
  #   锚定**少数**（漏掉一个仿真 ✗）⇒ **坏轮会被放过** ✗✗（危险 ✓）；
  #   宽松**多数** ⇒ 只会把**好轮标脏** ✗（安全 ✓）。
  #   ⇒ 两个并列 ⇒ **给"锚定漏数"留一个可核的对照** ✓（"锚定 1 / 宽松 3"一眼看出是包装体 ✓，
  #     而不是"漏了一个仿真" ✓）。
  # ★★ 2026-10-10（reviewer）：**注记不许替读者下"可否归属"的结论** ✗ ★★
  #   实测（reviewer 逐进程读 /proc）：健康**单栈**在这里读到 `_IGN=2` ✓ ——
  #   包装体 `/bin/sh -c ruby /usr/bin/ign gazebo …` ＋ 真身 `ign gazebo -r …` **两者 argv 都含 `campus_100.sdf`** ✓
  #   （`ign gazebo server` / `gui` 不含 ⇒ 不计入 ✓）⇒ 2 就是"一套" ✓。
  #   ⇒ 原来那句"锚定≠1 ⇒ 本轮读数不可归属"**把一套健康栈标成不可归属** ✗✗
  #   ⇒ 下一轮的读者会照着它**撤掉一轮合格的取证** ✗
  #     （= "注释不是凭据"的**镜像**：**注记本身成了错的凭据** ✓）。
  #   ⇒ ★ 只记**中性数** ✓、**不下结论** ✓；**归属改由 `window_health.jsonl` 的 `instances_by_partition` 给** ✓。
  #   ★ 我**没有**顺手加第三个"proc 聚类"数 —— 它没两头标定过，按规矩只能当读数、不许当判据 ✓。
  echo "ign_gazebo_instances=${_IGN} (宽松 ${_IGN_LOOSE})   # 起栈时实测；★只记数，归属见 window_health.jsonl"
} >> "${OUT}/run_state.txt"

# 2) 短 rosbag：留证用。**必须有界**：图像+深度约 5 MB/s，无限录一次能吃掉十几 GB
#    （视觉版实测过一次 48 分钟 14 GB）。时间与体积双封顶，先到先停。
# **先清掉上一次留下的录制目录**：ros2 bag record 在输出目录已存在时会直接报
# "[ERROR] Output folder ... already exists" 并退出——**不录任何数据**，
# 而产物里只留下一行 ERROR，很容易被当成"录了"（第二阶段实测踩过：rtk-24/25 的 bag 只有 52K）。
# ★★ 2026-09-21 新增开关：`SKIP_BAG=1` ⇒ **不起内建录制**（也不起体积守卫）★★
#   **默认行为一个字没变**：不给这个变量时照旧录制 ✓。
#   为什么要这个：reviewer 端到端复验时**他自己录一份就够** ⇒ 避免
#     · **双份 I/O**（点云/图/来回写盘会互相抢带宽，可能反过来影响被验的现象）；
#     · **双份 1.4 GB**；
#     · 他的 run 目录里多出一个包。
#   ★ `start_stack.sh` **不在** `lib.sh::record_config_fingerprint` 的 7 个指纹文件里
#     （那 7 个是 config/*.yaml 五个 + worlds/*.sdf + models/*.xacro）
#     ⇒ 这个开关**不改变配置指纹**。**但这句话必须"重算一遍"来证，不能只引用** ✓
#     （见交接产物里的 `handoff_fingerprint_recheck.txt`）。
#   ★ 判定用 `-eq 1` 的数值比较会炸（未设变量时是空串）⇒ 用字符串比较、并给默认值。
SKIP_BAG="${SKIP_BAG:-0}"
if [ "${SKIP_BAG}" = "1" ]; then
    echo "[容器内] ** SKIP_BAG=1 ⇒ 不启动内建 rosbag 录制（也不起体积守卫）**；"
    echo "           日志里不会出现 rosbag.pid / rosbag_guard.pid，bag/ 目录不会增长。"
    mkdir -p "${OUT}/bag"
else
    if [ -d "${OUT}/bag/run" ]; then
        echo "检测到 ${OUT}/bag/run 已存在，已删除并重新录制（否则 ros2 bag record 会直接报错退出、录出空包）"
    fi
    rm -rf "${OUT}/bag/run"
    BAG_MAX_SECONDS="${BAG_MAX_SECONDS:-600}"
    BAG_MAX_BYTES="${BAG_MAX_BYTES:-2147483648}"   # 2 GiB
    # 说明：点云 /rtk/camera/points 与彩色图 /rtk/camera/image **不录**（体积）；
    # 两张 nav2 图、危险距离、定位、TF、真值都录，够做时序对照。
    # `-k` 是 SIGINT 之后的 SIGKILL 宽限期。原来给 20 s **太短**：收尾要把缓存刷盘、
    # 写 metadata.yaml，1 GB 级的包刷不完就被 SIGKILL，于是录出一个
    # **没有 metadata、`ros2 bag info` 打不开**的包（reviewer 复验实测到，1.3 G）。
    # 给到 120 s：正常收尾几百毫秒就结束，不会真等这么久。
    # **第五阶段（2026-09-21 修）**：`/rtk/scan` 补录 —— 它是"激光源还在来"的**直接**证据
    # （350 线 × 10 Hz ≈ 20 KB/s，可忽略）。**注意注释必须写在这一条命令之外**：
    # 上一版把这四行注释塞进了 `\` 续行的命令里，bash 会把 `#` 之后整段当注释，
    # 于是**话题表被截成前三条**（实测 `nav10-lay2`/`nav10-clr` 的 bag 只有
    # /clock、/rtk/camera/depth_image、/rtk/camera/camera_info，两张图一条没录），
    # 而 `rosbag.log` 里只留一行 "/rtk/camera/points: No such file or directory"。
    # **点云 /rtk/camera/points 仍不录**：10 Hz 实测 ~11 MB/s，会先撞 2 GiB 上限，
    # 一撞上限**后面的图/tf 全丢** —— 这与上面"体积"那句理由一致。
        # ★★ 2026-10-12（reviewer 要求 ✓）：**加 `/cmd_vel`** ★★
        #   为什么：★ 要查"同一棵树同一航线，录屏那轮巡航 0.493 m/s、本机 0.217 ✗"那条 2.3× 差
        #   落在哪 ⇒ ★ 必须看**控制器实际下了多少速度** ✓ —— ★ 而 `gt` 只给结果 ✓。
        #   ★ 实测：★ 之前的 bag **只有 4 个话题**（`/clock`、`/rtk/scan`、★ 相机两张 ✓）
        #     ⇒ ★ `/cmd_vel` **没录过** ✗ ⇒ ★ 那条通路**取不到** ✓（`campus8-demo2` 连 bag 都没有 ✗）。
        #   ★ 一行 ✓、★ 只增不改 ✓（✗ 不动原有话题 ✗）。
        # ★★ 2026-10-10：**加 `/rtk/navsat`**（reviewer 要求）★★
        #   为什么：它是 **Gazebo 的理想 GPS**（无噪声、无注入偏差），
        #   **标定的第二半（世界系 ↔ 经纬度）正是配它** ✓ —— 而之前 bag 里**只有 `/rtk/sim_fix`**
        #   （加了 σ/bias 的那条）✗ ⇒ 那一半**没有原始记录可重建**。
        #   实测代价：`campus8-f` 第二半第一次收尾失败时，样本全丢、而 bag **恢复不了** ✗ ⇒ 只能重跑整条路线。
        #   ⇒ 加上它，"理想 GPS 那一侧"就有**原始记录** ⇒ 第二半**可以从 bag 重算**、不必靠"跑的那一刻" ✓
        #   （与"**原始记录必须能重建结论**"同族 ✓）。开销可忽略（~7 Hz × 小消息）。
    nohup timeout -s INT -k 120 "${BAG_MAX_SECONDS}" \
        ros2 bag record -o "${OUT}/bag/run" -b 52428800 \
        /clock /rtk/camera/depth_image /rtk/camera/camera_info \
        /rtk/scan /cmd_vel \
        /rtk/imu /rtk/sim_fix /rtk/navsat \
        /global_costmap/costmap /local_costmap/costmap \
        /rtk/costmap_danger /rtk/costmap_status \
        /odometry/global /odometry/local /tf /tf_static /cmd_vel /ground_truth/odom \
        > "${OUT}/logs/rosbag.log" 2>&1 &
    BAG_PID=$!
    echo "${BAG_PID}" > "${OUT}/logs/rosbag.pid"
    (
        while kill -0 "${BAG_PID}" 2>/dev/null; do
            sz="$(du -sb "${OUT}/bag" 2>/dev/null | cut -f1)"
            if [ -n "${sz}" ] && [ "${sz}" -gt "${BAG_MAX_BYTES}" ]; then
                echo "rosbag 已达 ${sz} 字节，超过上限，停止录制"
                kill -INT "${BAG_PID}" 2>/dev/null || true
                break
            fi
            sleep 10
        done
    ) > "${OUT}/logs/rosbag_guard.log" 2>&1 &
    echo $! > "${OUT}/logs/rosbag_guard.pid"
fi

# ★★ 2026-10-13：起栈后把 GUI 相机**一次性**设到"车所在处" ★★
#   ★ 业主口径 ✓（★ 2026-10-13 放宽后 ✓）：★ "**不一定要一直跟随，只要初始时视角在小车上即可**" ✓
#     ⇒ ★ **发一次**就完 ✓ ✗ 不挂跟随节点 ✗、✗ 不周期发 ✗
#   ★ ★ 出生点是**已知且固定**的 ✓ ⇒ ★ **直接写**这里 ✓（★ 比"运行时取车位置"简单 ✓）
#   ★ ★ **视角取"车后上方俯视"** ✓ ⇒ ★ 画面里同时看得到车与它前方的路面 ✓
#   ★ ★ ★ 两件事**分两层**做 ✓（★ 因为 service 到底叫什么，**还没坐实** ✗）：
#     ① ★ **列出** GUI 暴露的全部服务 ✓ —— ★ 这一条本身就是判据 ✓：
#         ★ 列得出 ⇒ ★ 是"**名字不对**"✓；★ 列不出 ⇒ ★ 是"**分区／域不对**"或"**GUI 的 transport 没了**" ✓
#     ② ★ **尽力**发一次位姿 ✓ ⇒ ★ 发不出去 ★ **不影响起栈** ✓（★ 只记实情 ✓ ✗ 不当失败 ✗）
#   ★ ★ ✗ **别为它加等待或重试** ✗ —— ★ 拿不到就是拿不到 ✓，★ 写清比"装作设上了"强 ✓
CAM_POSE_X=${CAM_POSE_X:--6.442}   # ★ 出生点 (-0.442, -0.151) 向 -x 退 6 m
CAM_POSE_Y=${CAM_POSE_Y:--6.151}   # ★ 向 -y 退 6 m（★ 车后上方 ✓）
CAM_POSE_Z=${CAM_POSE_Z:-8.0}      # ★ 抬高 8 m ⇒ 俯视 ✓
GUI_SVCS="$(timeout 15 ign service -l 2>/dev/null | /bin/grep -i gui || true)"   # ★ 列服务也要 timeout ✗ 否则可能一直等 ✓
echo "[容器内] ★ GUI 服务清单（★ 判『相机服务有没有』✓）："
if [ -n "${GUI_SVCS}" ]; then printf '%s\n' "${GUI_SVCS}" | sed 's/^/    /'; else echo "    （一个都没有）"; fi
# ★★ 2026-10-13 第二次实测改正（★ 第一次那个 /gui/follow 是**持续模式** ✗）★★
#   ★ 业主实测现象（★ 三个 ✓）：★ ① 画面**一直往车拉近** ✗ ② 鼠标拖动后**松开就回拉** ✗
#     ③ ★ **不能平移**、只能旋转 ✗（★ 交互被锁 ✓）
#   ★ ⇒ 那正是 `follow` 的语义 ✓：★ 它是个**持续开关** ✗ ⇒ ★ "只发一次"和"它会一直生效"**不冲突** ✗
#   ★ ⇒ 改用 ★ **CameraTracking 的 `move_to/pose`** ✓ —— ★ **一次性设位姿** ✓，★ 设完就撒手 ✓
#     （★ `CameraTracking` 插件本来就在 `docker/gz-gui/gui.config` 里加载着 ✓ —— ★ 实测：那份 config 第 84 行就有它 ✓，
#       ✗ 不需要另外加插件 ✗；★ 两个服务的类型也是 `ign service -i` 量的 ✓：`move_to` ← StringMsg（实体名 ✓）、
#       `move_to/pose` ← GUICamera（位姿 ✓））
#   ★ 位姿怎么来的 ✓：★ 出生点是**已知且固定**的 ✓ ⇒ ★ 相机放在车后上方 6/6/8 m ✓，
#     朝向 = **从相机指向车** ✓（★ 按"相机看 +x"算成四元数 ✓；★ 分量都写在这儿 ✓ 便于复核 ✓）
#   ★ ✗ 别为它加等待或重试 ✗（★ 发不出去不影响起栈 ✓，★ 只记实情 ✓）
CAM_POSE_X=${CAM_POSE_X:--6.442}   # ★ 出生点 (-0.442, -0.151) 向 -x 退 6 m
CAM_POSE_Y=${CAM_POSE_Y:--6.151}   # ★ 向 -y 退 6 m（★ 车后上方 ✓）
CAM_POSE_Z=${CAM_POSE_Z:-8.0}      # ★ 抬高 8 m ⇒ 俯视 ✓
CAM_QW=${CAM_QW:-0.87018}          # ★ 朝向（★ 单位四元数 ✓）：把 +x 转到 (6,6,-8)/11.6619 ✓
CAM_QX=${CAM_QX:-0}
CAM_QY=${CAM_QY:-0.39417}
CAM_QZ=${CAM_QZ:-0.29564}
GUI_SVCS="$(timeout 15 ign service -l 2>/dev/null | /bin/grep -i gui || true)"   # ★ 列服务也要 timeout ✗ 否则可能一直等 ✓
echo "[容器内] ★ GUI 服务清单（★ 判『相机服务有没有』✓）："
if [ -n "${GUI_SVCS}" ]; then printf '%s\n' "${GUI_SVCS}" | sed 's/^/    /'; else echo "    （一个都没有）"; fi
if printf '%s' "${GUI_SVCS}" | /bin/grep -q '/gui/move_to/pose'; then
    if timeout 20 ign service -s /gui/move_to/pose \
        --reqtype ignition.msgs.GUICamera --reptype ignition.msgs.Boolean --timeout 8000 \
        --req "pose: {position: {x: ${CAM_POSE_X}, y: ${CAM_POSE_Y}, z: ${CAM_POSE_Z}}, orientation: {w: ${CAM_QW}, x: ${CAM_QX}, y: ${CAM_QY}, z: ${CAM_QZ}}}" \
        >/dev/null 2>&1; then
        echo "[容器内] ★ 视角已**一次性**设到车后上方 ✓（move_to/pose；★ 设完不再动 ✓）"
    else
        echo "[容器内] ★ 有 /gui/move_to/pose 但**发失败** ✗（★ 不影响起栈 ✓ —— 看上面那份清单 ✓）"
    fi
else
    echo "[容器内] ★ 没看到 /gui/move_to/pose ⇒ **本轮不设相机视角** ✗（★ 不影响起栈 ✓）"
    echo "   → ★ 先看上面那份清单：★ 有别的 gui 服务 ⇒ 名字不对 ✓；★ 清单为空 ⇒ 分区／域不对，或 GUI 的 transport 没了 ✓"
fi

# ---- Gazebo 世界健康检查（reviewer 建议）----
# 不"等 N 秒"，而是**要求真的能看到机器人模型**再往下。
# 理由：nav4-2 那次 Gazebo 进程在、世界服务始终不可达（create 每 5 秒重试一次、320 秒没成功），
# 这种状态**等多久都不会好**，而现象会被下游伪装成"nav2 卡住 / 定位链没起来"——
# 当时排查方向就差点跑偏。这里主动查一次并**说清是哪一环**。
GZ_OK=0
for _ in $(seq 1 30); do
    if ign model --list 2>/dev/null | grep -q robot_rtk; then GZ_OK=1; break; fi
    sleep 2
done
if [ "${GZ_OK}" -eq 1 ]; then
    echo "[容器内] Gazebo 世界正常：已能列出 robot_rtk"
else
    echo "[容器内] ** Gazebo 世界里没有 robot_rtk（等了 60 s）**"
    echo "   → Gazebo 进程可能在、但世界服务不可达（本轮起栈不会再自己好）。"
    echo "   → 先看 ${OUT}/logs/demo_launch.log 里 ign gazebo 的行数（正常应有上百行；只有两三行就是它没起来），"
    echo "     以及 create-3 是否在反复 \"Requesting list of world names.\""
    echo "   → 处置：停栈重起（共享机上 GUI 是最不稳的一环；本步栈默认已改成无界面 -s）"
fi

echo "started run ${RUN_ID}"
