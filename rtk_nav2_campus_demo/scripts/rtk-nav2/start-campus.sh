#!/usr/bin/env bash
# ★ 阶段八入口：校园 100 m 场景起栈（`worlds/campus_100.sdf` + `config/campus_nav2.yaml`）。
#
# **为什么不直接用 `start-demo.sh`**：它是**第三阶段**入口，默认带的是庭院世界与
# `nav2_params.yaml`；用它起阶段八要么改它的默认值（会破坏阶段三/七的可复跑），
# 要么每次手打一长串 `key:=value`（容易漏，尤其 `x_pose/y_pose` 与静态层栅格）。
# ⇒ 这里做成**薄包装**：把阶段八那一套钉死，其余全部复用 `start-demo.sh` 的机制
#   （容器/工作区/就绪闸门/重试/参数快照/指纹），**不复制一份实现**。
#
# 用法：scripts/rtk-nav2/start-campus.sh [run-id] [--no-rviz]
#
# ★★ **两档各砍掉了什么（写清，别靠猜）** ★★
#   · 默认（检查档）：**Gazebo GUI 开、RViz 开**。
#   · `--no-rviz`（取数档）：**只关 RViz**（透传 `rviz:=false`）。**Gazebo GUI 仍然开** ——
#     这一项**不能关**：`gz_gui:=false` 会走 `-s`（server-only），实测**相机整条链路会消失**
#     （`ign topic -l | grep camera` 计数 = 0 ⇒ 没有深度图 ⇒ 没有点云 ⇒ 两张代价图没数据），
#     启动脚本里那段长注释就是这件事。
#   ⇒ **两档的仿真与感知接线完全相同**，只差一个 RViz 窗口 ⇒ **取数档可以出验收数据** ✓
#     （这也是原来那个 `--no-gui` 名字的毛病：听起来像"把仿真也关了"，实际没有 ⇒ 改名）。
#
# 显示：**不在这里自己挑**，交给 `lib.sh` 的 `ensure_display`（`start-demo.sh` 会调）——
#   收敛到一处，免得每加一个入口再漏一次（`campus8-a` 那次就是这么栽的：容器 baked 的
#   DISPLAY 属**别人**的会话 ⇒ Gazebo GUI 起不来 ⇒ **整个 ign gazebo 进程死掉** ⇒
#   现象却是"node list 有 29 个节点、但没有 map"，看起来像定位没起来）。
#
# ★★ 出生点**从生成器现读**（`make_campus_world.py` 的 `SPAWN`，单一真源）：
#    手抄 `x_pose/y_pose` 是这类脚本最容易静默失效的地方 —— launch 的默认值还是
#    阶段四的 `2.6 / -2.6`（那个点在校园里落在**主楼北侧空地**，与声明不符）。
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID=""
WITH_RVIZ=1
while [ $# -gt 0 ]; do
    case "$1" in
        --no-rviz) WITH_RVIZ=0; shift ;;
        --rviz) WITH_RVIZ=1; shift ;;
        --no-gui|--gui) die "该开关已改名：**Gazebo GUI 不能关**（关了会丢相机链路）⇒ 请用 --no-rviz（只关 RViz）";;
        -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
        *) RUN_ID="$1"; shift ;;
    esac
done
RUN_ID="$(resolve_run_id "${RUN_ID}")"

PKG="${PKG_DIR}"
# ★ 2026-10-12 修（reviewer 批准 ✓）：这三行原来把 `rtk_nav2_ws` **写死** ✗ ⇒
#   `export WORKSPACE_NAME=…` 对它们**不生效** ⇒ 会起成"**旧工作区的树 ＋ 这个工作区的
#   配置指纹**"的**混合栈** ✗（2026-10-11 实测骗过一次：run-id `campus100-look` ✓）。
#   ⇒ 改成照 `start-campus-cut0.sh` 那样**按 ${WORKSPACE_NAME} 拼** ✓。
#   ★ 不设 WORKSPACE_NAME 时，`lib.sh:34` 的默认值仍是 `rtk_nav2_ws` ⇒ **三个路径与改动前逐字相同** ✓。
WORLD_C="${CONTAINER_WORKDIR}/${WORKSPACE_NAME}/src/robonix_rtk_nav2/worlds/campus_100.sdf"
NAV2_C="${CONTAINER_WORKDIR}/${WORKSPACE_NAME}/src/robonix_rtk_nav2/config/campus_nav2.yaml"
# ★ 阶段八自己的 RViz 配置：`demo.rviz` 的视图 Scale=45 是给 10 m 庭院调的，
#   在 100 m 校园上等于"只看得见几米"（截图里看不出车在哪 ✗）⇒ 用 `campus.rviz`（Scale=8，
#   其余显示项一字不差）。**阶段三那份 demo.rviz 没动** ✓。
RVIZ_C="${CONTAINER_WORKDIR}/${WORKSPACE_NAME}/src/robonix_rtk_nav2/config/campus.rviz"

# ---- 0) 世界必须在（离线生成器产出；不在这里自动生成，避免"起栈顺手改了世界"）----
[ -f "${PKG}/worlds/campus_100.sdf" ] || die "缺少 ${PKG}/worlds/campus_100.sdf ⇒ 先跑
    python3 ${PKG}/scripts/make_campus_world.py"

# ---- 1) 出生点从生成器现读（单一真源）+ 生成器自检（含基准值/出生点净空）----
GEN_JSON="$(python3 "${PKG}/scripts/make_campus_world.py" --check-only 2>/dev/null)"
read -r SPAWN_X SPAWN_Y < <(printf '%s' "${GEN_JSON}" | python3 -c \
    'import json,sys; d=json.load(sys.stdin); print(d["spawn"][0], d["spawn"][1])' 2>/dev/null)
[ -n "${SPAWN_X:-}" ] && [ -n "${SPAWN_Y:-}" ] || die "没能从生成器读出 SPAWN（生成器自检没过？）"
GEN_SHA="$(sha256sum "${PKG}/worlds/campus_100.sdf" | cut -c1-16)"
log "生成器自检通过：出生点 = (${SPAWN_X}, ${SPAWN_Y})（from make_campus_world.py）；世界 sha256 前 16 = ${GEN_SHA}"

# ---- 2) 起栈前：容器干净（node list = 0 且无重复名）----
ensure_container >/dev/null 2>&1 || true
# ★★ 2026-10-12：★ 换成**两层的闸** ✓（★ 一处实现 = `lib.sh` 的 `container_clean_or_die` ✓）
#   ★ 本文件以前自带一份**单层**实现（只读 ROS 层 ✓）⇒ ✗ 对"活着但 ROS 层读空"会**误放行**、
#   ✗ 对"陈旧 daemon"会**误拒** ✓ —— ★ 今天两头的判据都用真实验跑过 ✓，见该助手的注释 ✓。
container_clean_or_die "起栈前必须干净" || exit 1

# ---- 3) 交给 start-demo.sh（它负责容器/工作区/**display 自检**/就绪/重试/指纹/快照）----
ARGS=(
    "world:=${WORLD_C}"
    "nav2_config:=${NAV2_C}"
    "static_world:=${WORLD_C}"
    # ★ 静态层栅格**必须与 `campus_nav2.yaml` 的全局图逐字一致**（120x120 @0.10 origin -60）：
    #   不一致时 StaticLayer 会按全局图自己的栅格重采样 ⇒ 楼被挪位
    "static_resolution:=0.10" "static_size:=120.0" "static_origin:=-60.0"
    "rviz_config:=${RVIZ_C}"
    "x_pose:=${SPAWN_X}" "y_pose:=${SPAWN_Y}" "z_pose:=0.01"
)
if [ "${WITH_RVIZ}" -eq 1 ]; then
    log "检查档：Gazebo GUI + RViz 都开"
    exec "$(dirname "${BASH_SOURCE[0]}")/start-demo.sh" "${RUN_ID}" --gui -- "${ARGS[@]}"
else
    log "取数档：Gazebo GUI 开（**感知接线与检查档完全相同**）、只关 RViz"
    exec "$(dirname "${BASH_SOURCE[0]}")/start-demo.sh" "${RUN_ID}" --no-gui -- "${ARGS[@]}"
fi
