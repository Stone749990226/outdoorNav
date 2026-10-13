#!/usr/bin/env bash
# ★ 阶段九入口：**他们的场景**（`worlds/north_campus.sdf`）+ **我们的车与全栈** + `campus_nav2.yaml`。
#
# 用法：scripts/rtk-nav2/start-northcampus.sh [run-id] [--no-rviz]
#
# **为什么不直接用 `start-demo.sh`**：同 `start-campus.sh` —— 它是阶段三入口，
# 默认带庭院世界与 `nav2_params.yaml` ✗ ⇒ 这里做**薄包装**，把阶段九那一套钉死，
# 其余（容器/工作区/display 自检/就绪闸门/重试/指纹/快照）**全部复用** ✓，不复制实现 ✓。
#
# ★★ 三处与 `start-campus.sh` 不同（逐条写明 ✓）★★
#   ① **世界** = `worlds/north_campus.sdf` ✓ —— ★ 用**源码绝对路径** ✓，
#      ✗ 不用 `package_share` 那份 ✗：`install/.../worlds/` 里是**按文件的软链**（build 时建的 ✓），
#      新加的世界**不会自动出现** ✗（实测 `ls` 确认 ✓）⇒ 传绝对路径即可 ✓（`start-campus.sh` 也是这么传的 ✓）。
#   ② **出生点** = 他们航线的**第 1 个点** `(-0.442, -0.151)` ✓ ——
#      ★ 从导出文件**现读** ✓（✗ 不手抄 ✗，手抄是这类脚本最容易静默失效的地方 ✓）。
#   ③ ★★ **球坐标基准**：世界文件里**没有** `<spherical_coordinates>` ✗（他们的 world 本来就没有 ✓），
#      而整条定位链（`Gazebo navsat → rtk_sim → navsat_transform → absolute_localizer → /odometry/global`）
#      **依赖它** ✗ —— 缺了它 Gazebo 报的经纬度是绕 **(0,0)** 的 ✗，
#      而 `navsat.yaml` 的 datum 是他们那个点 ⇒ **UTM 分区都对不上** ⇒ map 系整个是错的 ✗✗。
#      ⇒ ★ 用**运行时服务**把世界的球坐标设成**他们的 `origin_wgs84`** ✓
#        （✗ **不动世界文件** ✓ —— 与第 2 步那两处渲染层补偿同属"运行期参数" ✓，单独记账 ✓）。
#      ★ 为什么必须 = `navsat.yaml` 的 datum：**map 帧原点 = datum 的 UTM** ✓，
#        而世界原点的经纬度 = 世界的球坐标 ✓ ⇒ 两者相等时 **map 帧 ≡ world 帧** ✓
#        ⇒ 他们导出的 (x,y) 才能**直接用** ✓（这正是 `campus_100` 那条"必须逐字一致"的同一件事 ✓）。
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID=""
WITH_RVIZ=1
while [ $# -gt 0 ]; do
    case "$1" in
        --no-rviz) WITH_RVIZ=0; shift ;;
        --rviz) WITH_RVIZ=1; shift ;;
        -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
        *) RUN_ID="$1"; shift ;;
    esac
done
RUN_ID="$(resolve_run_id "${RUN_ID}")"

PKG="${PKG_DIR}"
PKG_C="${CONTAINER_WORKDIR}/rtk_nav2_ws/src/robonix_rtk_nav2"
WORLD_C="${PKG_C}/worlds/north_campus.sdf"
NAV2_C="${PKG_C}/config/campus_nav2.yaml"
RVIZ_C="${PKG_C}/config/campus.rviz"
# ★★ 2026-10-10（阶段九）：**他们的 world 里没有 `<plugin>`** ✗ ⇒ Fortress 下传感器全哑 ✗
#   ⇒ 用**服务端配置**在运行层补上 systems 插件 ✓（世界文件不动 ✓）。
#   ★ 这条要 export 出去 ✓ —— `start-demo.sh` 会把同名变量**显式**带进容器 ✓
#     （`docker exec` 不继承宿主 env ✗）。
export NORTH_CAMPUS_SERVER_CONFIG="${PKG_C}/config/north_campus_server.config"
WORLD_NAME="north_campus"
WP_CSV="${PROJECT_DIR}/artifacts/rtk-nav2/outnav1/exports/waypoints_export.csv"

# ---- 0) 世界必须在（✗ 不在这里自动生成 / 不改世界 ✓）----
[ -f "${PKG}/worlds/north_campus.sdf" ] || die "缺少 ${PKG}/worlds/north_campus.sdf"
[ -f "${WP_CSV}" ] || die "缺少 ${WP_CSV}（他们的航线导出）"
log "世界 sha256 前 16 = $(sha256sum "${PKG}/worlds/north_campus.sdf" | cut -c1-16)（north_campus.sdf）"
log "航线 sha256 前 16 = $(sha256sum "${WP_CSV}" | cut -c1-16)（waypoints_export.csv）"

# ---- 1) 出生点从他们的导出**现读**（★ 第 1 个点 ✓，单一真源 ✓）----
read -r SPAWN_X SPAWN_Y < <(python3 - "${WP_CSV}" <<'PY'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1], encoding='utf-8-sig')))
print(rows[0]['x'], rows[0]['y'])
PY
)
[ -n "${SPAWN_X:-}" ] && [ -n "${SPAWN_Y:-}" ] || die "没能从 ${WP_CSV} 读出第一个点"
# ★★ 2026-10-10（第 5 步）：出生点可用 `SPAWN_X`／`SPAWN_Y` **覆盖** ✓ ★★
#   为什么要这个开关（★ 实测依据 ✓）：航线的**最后一点 wp41 与出生点 wp1 是同一个点** ✓
#   ⇒ ★ 车**自己**在那个位置留下的观测（obstacle_layer ✓）把该格标成了
#     **58（膨胀带）＋ 7×7 内 5 个致命格** ✗（实测 ✓；起跑时那一格是 **0** ✓）
#     ⇒ 回来时**规划器给空路径** ✗。
#   ⇒ ★ 把出生点**挪开一点**（运行期参数 ✓，✗ 不动场景 ✗）就能让那一格不被车自己压住 ✓。
if [ -n "${SPAWN_X_OVERRIDE:-}" ] || [ -n "${SPAWN_Y_OVERRIDE:-}" ]; then
    SPAWN_X="${SPAWN_X_OVERRIDE:-${SPAWN_X}}"
    SPAWN_Y="${SPAWN_Y_OVERRIDE:-${SPAWN_Y}}"
    log "★ 出生点被覆盖 ⇒ (${SPAWN_X}, ${SPAWN_Y})（原 ${WP_CSV} 第 1 行）"
fi
log "出生点 = (${SPAWN_X}, ${SPAWN_Y})（from waypoints_export.csv 第 1 行）"

# ---- 2) 起栈前：容器干净 ----
# ★★ 2026-10-12：**不许把建容器的失败丢掉** ✗（★ 我自己踩出来的 ✓）
#   原写法 `ensure_container >/dev/null 2>&1 || true` ✗ ⇒ ★ 镜像名拼错时
#   `docker run` 的报错被**整段吞掉** ✗ ⇒ 二十行之后才报成"演示 GUI 配置没生成" ✗
#   （★ 而 `|| true` 本身是**死代码** ✓：`ensure_container` 要么 return 0、要么 `die` ✗，
#    而 `die` 是 `exit 1` ⇒ **`||` 根本接不住** ⇒ 它只是**看着像"容错"** ✗）
#   ⇒ ★ 改成**原样调用** ✓：成功时那几行（X 鉴权/外层场景/创建容器 ✓）本来就有用 ✓，
#     失败时 `die` 会带着真因停下 ✓。
ensure_container
# ★★ 2026-10-10：先 `ros2 daemon stop` 再数节点 ✗★★
#   实测：bag 录制进程**已经按 PID 收掉了** ✓（`/proc/<pid>` 已不存在 ✓），
#   而 `ros2 node list` **照样列出 `/rosbag2_recorder`** ✗ ⇒ 守卫把干净容器判成"有节点" ✗
#   ⇒ **起栈被自己的守卫挡住** ✗（★ "守卫比被测物更容易坏" ✓，本项目已记过同族三次 ✓）。
#   ⇒ 定式：**ros2 CLI 结论不可全信 ⇒ 先 `daemon stop` 再量** ✓。
# ★★ 2026-10-12：★ 换成**两层的闸** ✓（★ 一处实现 = `lib.sh` 的 `container_clean_or_die` ✓；★ 本文件以前自带一份**单层**实现 ⇒ ✗ 对"活着但 ROS 层读空"会**误放行** ✓、✗ 对"陈旧 daemon"会**误拒** ✓）
container_clean_or_die "起栈前必须干净" || exit 1

# ---- 3) 起栈（**后台**）+ ★ 尽早把世界的球坐标设成他们的基准 ★ ----
# ★★ 2026-10-10（第 4 步）：**先生成演示用的 GUI 配置** ✓ ★★
#   第 1/2 步那套"删停靠面板让 3D 视口铺满 ＋ 定机位"当时只作用于**只有世界**的起法 ✗；
#   全栈走 `ros_gz_sim` ⇒ 那条路没带 GUI 配置 ✗ ⇒ 截图里 3D 视口没铺满 ✗。
#   ⇒ 现在 `demo.launch.py` 有了 `gui_config:=` ✓，这里造好那份文件再传进去 ✓。
DEMO_GUI_C="${ARTIFACTS_ROOT}/${RUN_ID}/gui/gui_demo.config"
DEMO_GUI_H="${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/gui/gui_demo.config"
mkdir -p "${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/gui"
# ★ 机位可用 `DEMO_GUI_CAMERA` 覆盖 ✓（默认在 `make_demo_gui_config.py` 里 ✓，值都是实测定过的 ✓）
#   ★★ 两个坑（都是实测踩的 ✗）：① 空格必须换成**逗号**再传 ✓
#      （`--camera 17.4 -122 …` ⇒ argparse 报 `unrecognized arguments: -122 78 …` ✗）；
#      ② 值以 `-` 开头时**必须用 `--camera=` 形式** ✓
#      （`--camera -42.6 …` ⇒ argparse 把 `-42.6` 当 flag ⇒ `expected one argument` ✗）。
CAM_ARG=""
[ -n "${DEMO_GUI_CAMERA:-}" ] && CAM_ARG="--camera=${DEMO_GUI_CAMERA// /,}"
cexec "python3 /work/scripts/rtk-nav2/make_demo_gui_config.py --out ${DEMO_GUI_C} ${CAM_ARG}" 2>&1 | tail -2
[ -s "${DEMO_GUI_H}" ] || die "演示 GUI 配置没生成（${DEMO_GUI_H}）⇒ 截图会没铺满 ✗"
log "演示 GUI 配置已生成 ✓（sha256 前 16 = $(sha256sum "${DEMO_GUI_H}" | cut -c1-16)）"

ARGS=(
    "world:=${WORLD_C}"
    "nav2_config:=${NAV2_C}"
    "static_world:=${WORLD_C}"
    "rviz_config:=${RVIZ_C}"
    "gui_config:=${DEMO_GUI_C}"
    "x_pose:=${SPAWN_X}" "y_pose:=${SPAWN_Y}" "z_pose:=0.039"
)
LAUNCH_ARG=()
if [ "${WITH_RVIZ}" -eq 0 ]; then LAUNCH_ARG=(--no-gui); else LAUNCH_ARG=(--gui); fi

# ★★ 2026-10-13（reviewer 定 ✓）：★ **把球坐标轮询提到起栈之前** ✓ ★★
#   ★ 为什么 ✓：★ 它是**抢时间窗**的 ✓（★ 必须在 NavSat 传感器**创建之前**设上 ✓）；
#     ★ 早一步起 ⇒ ★ 窗口更大 ✓（✗ 不改世界文件 ✓，★ 这是运行期参数 ✓）。
#   ★ 且 ★ 叶子脚本**修好了** ✓（★ 原来它自己被拼坏 ⇒ 两套参数解析 ⇒ `seq` 当场炸 ⇒ 一次都没轮 ✓）
#   ★ 凭据 ✓：★ 成功会写 `/tmp/spherical_ok_<分区>`** ✓ ⇒ ★ 下面 `wait` 之后据此**判成败** ✓（✗ 不靠日志文本 ✗）
SC_LOG="/tmp/spherical_coord_${RUN_ID}.log"
# ★★ 分区**显式传进去** ✓（✗ 不靠 `cexec_bg` 注入 ✗ —— 它走 `_round_gzp` 读"当前 run" ✓，
#   而那一刻 `start-demo.sh` 还没把本轮记上 ✗ ⇒ 注进来的是容器默认 `rtknav2` ✗
#   ⇒ 轮询会在**错分区**里数服务 ⇒ 永远数不到 ✗）。实测自报出来的分区就是 `rtknav2` ✓。
cexec_bg "${SC_LOG}" "bash /work/scripts/rtk-nav2/container/set_spherical_coords.sh ${WORLD_NAME} rtknav2_${RUN_ID}"
log "已在容器内起了球坐标轮询（★ 文件式、分区显式 ✓）：/work/scripts/rtk-nav2/container/set_spherical_coords.sh（日志 ${SC_LOG} ✓）"
log "起栈（后台）…"
# ★ 这个日志目录要**先建** ✗ —— `ensure_run_dirs` 是在 `start-demo.sh` **里面**才跑的 ✓，
#   而重定向在调用它**之前**就求值了 ⇒ 实测 "No such file or directory" 直接退出 ✗。
mkdir -p "${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/logs"
"$(dirname "${BASH_SOURCE[0]}")/start-demo.sh" "${RUN_ID}" "${LAUNCH_ARG[@]}" -- "${ARGS[@]}" \
    > "${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/logs/start_northcampus.log" 2>&1 &
SD_PID=$!

# ★ 轮询"服务出现了没有"，一出现就调 ✓（★ 不盲等固定秒数 ✓）；
#   ✗ 不动世界文件 ✓ —— 这是运行期参数，与第 2 步那两处渲染层补偿同类 ✓。
#
# ★★ 2026-10-10 实测修正（★ 这一步的**速度决定成败** ✗）★★
#   第一版：轮询在**宿主侧**、每轮起一个 `cexec_t` ✗ ⇒ 每次 `docker exec` 要 ~3 s ✗
#     ⇒ 7 轮 = ~21 s 才调到服务 ✗ ⇒ **调得太晚** ✗。
#   后果（实测，逐条可复现 ✓）：
#     · 机器人在服务被调**之前**就生成了 ✓ ⇒ NavSat 系统在"球坐标未设"时更新了 3 次 ✗
#       （日志：`Failed to update NavSat sensor enity [247]. Spherical coordinates not set.` ×3 ✓）
#       ⇒ 那 ~1.5 s 里 GPS 报的是**绕 (0,0) 的垃圾值** ✗
#     · `absolute_localizer` 的**第一条 fix** 落在那个窗口里 ✗ ⇒ `self.fix` 被设成垃圾坐标 ✗
#       ★ 而它的跳变守卫在**首读**时没有基准可校（`self.fix is None` ⇒ 直接接受 ✓）
#       ⇒ 之后每一条**好**读数都成了"跳 6653 km" ✗ ⇒ **全部被丢** ✗（实测累计 2618 次 ✓）
#       ⇒ 永远 `fix_stale` ✗ ⇒ **没有 map 帧** ✗ ⇒ TF 闸门过不去 ✗ ⇒ nav2 起不来 ✗
#       ⇒ 就绪等 300 s 超时 ⇒ `start-demo.sh` 把整个栈停掉 ✗✗。
#   ⇒ ★ 修法 = **在容器里**抢那个窗口 ✓ —— 一次 `docker exec -d`、脚本内部 `sleep 0.2` 循环 ✓
#     ⇒ 服务一出现（世界刚加载完 ✓、**通常在车生成之前** ✓）就调上 ✓ ⇒ 没有垃圾窗口 ✓✓。
#   ★ 另：`ign service -l` 自身要约 0.2~0.5 s ⇒ 实际周期约 0.5 s ✓（仍远快于 3 s ✓）。
#
# ★★ 2026-10-10 第二次修正（★ 教训：**复杂命令一律落成文件** ✓）★★
#   上一版把循环**内联**进 `cexec_bg "…" "<一大段带 \$ 转义的脚本>"` ✗
#   ⇒ ★ 实测**整条没跑起来** ✗（容器里连日志文件都没建 ✓），而外层照样打印"已发起" ✓
#     ⇒ 那一轮球坐标**从未设上** ✓（NavSat 警告 813 次 ✗）⇒ 本地化照旧锁死 ✗
#   —— 这正是本项目那条"**发出信号 ≠ 做成了**" ✗，也是"一次性命令里别放需要解释的字符" ✓。
#   ⇒ 现在循环在 `container/set_spherical_coords.sh` 里 ✓，这里只按**路径**调用 ✓。

wait "${SD_PID}"
RC=$?

# ★★ 2026-10-13：★ 「球坐标设上了没有」**要判** ✗ —— ★ 因为它决定整条定位链的成败 ✓
#   ★ 实测依据 ✓（★ 脚本头自己写的链 ✓）：★ 没设上 ⇒ GPS 是绕 (0,0) 的垃圾 ⇒ absolute_localizer
#     的**第一条 fix** 落在垃圾窗口 ⇒ 它把垃圾当基准锁住 ⇒ **永远 fix_stale** ⇒ ★ **没有 map 帧** ⇒
#     ★ 代价地图不发图 ⇒ rviz 里"没有圆形区域" ✓ ⇒ ★ 就绪闸 300 s 超时 ✓（★ 实测过他那一轮 ✓）
#   ★ ⇒ ★ **没设上就报错停** ✓（✗ 不静默接着走 ✗ —— ★ 那会让表相变成"起好了但地图是空的" ✗）
_SC_OK="$(cexec "cat /tmp/spherical_ok_rtknav2_${RUN_ID} 2>/dev/null" 2>/dev/null | tr -d '\r')"
if [ -z "${_SC_OK}" ]; then
    log "✗✗ 球坐标**没设上**（凭据 /tmp/spherical_ok_rtknav2_${RUN_ID} 不在 ✗）⇒ ★ 本轮定位链必坏 ⇒ 判**失败** ✗"
    log "   ★ 依据：没有球坐标 ⇒ GPS 垃圾 ⇒ 定位器锁死 ⇒ **没有 map 帧** ⇒ 代价地图不发图 ✓"
    exit 1
fi
log "★ 球坐标已设上 ✓（凭据时间 ${_SC_OK} ✓）"
log "起栈返回 rc=${RC}"
exit "${RC}"
