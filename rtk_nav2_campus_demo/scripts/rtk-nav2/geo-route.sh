#!/usr/bin/env bash
# 按**经纬度**巡游（brief §3.2）：读 `geo_calib.json` → 换算成 map 目标 → 依次发 `/goal_pose`
# → **判"到位且停稳"** → 记档；并把 map 系定位反解成经纬度发到 `/rtk/localization_geo`（自证用）。
#
# 用法：
#   scripts/rtk-nav2/geo-route.sh <run-id> [--route <文件>] [--tol 0.25] [--v-still 0.02] [--check-only]
#     · 默认航线 = artifacts/rtk-nav2/<run-id>/input/route_geo.json
#     · `--check-only` 只打印"经纬度 → map"的换算与**正反解自洽**，**不开车**（秒回，不占容器）
#
# ★ 与 `nav-to.sh` 的分工：`nav-to.sh` 收的是 **map 系 (x,y)**；本脚本收的是 **经纬度**，
#   中间那一段（经纬度→map）就是被验收的标定本身 ✓ —— 所以**本脚本不许自己算那个换算**，
#   一律读 `geo_calib.json`，且**只信它顶层的 `calib_key_is_valid` 布尔** ✓（见 geo_route.py 文件头）。
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"; shift || true
ROUTE=""; TOL="0.25"; VSTILL="0.02"; CHECK=""; CALIB_OVERRIDE=""; WAITALL=""
while [ $# -gt 0 ]; do
    case "$1" in
        --route) ROUTE="$2"; shift 2 ;;
        # ★ 标定来自**哪一轮**与"在哪一轮开车"可以不同：`calib_key` 里含
        #   `data_world_sha256` ⇒ 只要世界没换，换一轮起栈**键不变**、标定依然有效 ✓
        #   （检查档为了截图要带 GUI 起栈 ⇒ 是新的一轮，但世界没动 ⇒ 复用 `-f` 的标定 ✓）
        --calib) CALIB_OVERRIDE="$2"; shift 2 ;;
        --tol) TOL="$2"; shift 2 ;;
        --v-still) VSTILL="$2"; shift 2 ;;
        # ★ 标定轮专用：把"中间点也等停稳"的**旧行为**调回来（成对比较用 ✓）
        --wait-all) WAITALL="--wait-all"; shift ;;
        --check-only) CHECK="--check-only"; shift ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) die "未知参数：$1" ;;
    esac
done

stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"
CALIB="${CALIB_OVERRIDE:-${ARTIFACTS_ROOT}/${RUN_ID}/geo_calib.json}"
[ -n "${ROUTE}" ] || ROUTE="${ARTIFACTS_ROOT}/${RUN_ID}/input/route_geo.json"
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"
# ★ 2026-10-11（阶段十 · 第 0 刀）：按**工作区名**取包路径 ✓（原来写死 `rtk_nav2_ws` ✗）
#   ★ 不设 `WORKSPACE_NAME` 时与原来**逐字符相同** ✓；设了则整条链路切到另一个工作区 ✓。
PKG_IN_CTR="${CONTAINER_WORKDIR}/${WORKSPACE_NAME}/src/robonix_rtk_nav2"
OUT="${ARTIFACTS_ROOT}/${RUN_ID}/geo_route"
LOG_F="${ARTIFACTS_ROOT}/${RUN_ID}/geo_route_runner.log"
HOST_OUT="${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/geo_route"

# ★★ 起栈断言（reviewer 2026-10-10 裁定 ②，**优先级最高**）★★
#   背景：`campus8-h` 那一轮**跑一半栈塌了** —— `/global_costmap` 与 `/planner_server`
#   **中途消失** ✗（只剩局部图；节点 30→24）⇒ 车**顶着楼停住**、13 分钟白跑 ✗。
#   ⇒ 把"栈完整"从"**跑完才知道**"变成"**开场就断言**" ✓。
NEED_NODES="/planner_server /global_costmap/global_costmap /controller_server /lifecycle_manager_nav2"
cexec "${NAS} && timeout 30 ros2 node list > /tmp/_geo_route_nodes.txt 2>/dev/null" >/dev/null 2>&1
MISSING=""
for _n in ${NEED_NODES}; do
    cexec "grep -qx '${_n}' /tmp/_geo_route_nodes.txt" || MISSING="${MISSING} ${_n}"
done
if [ -n "${MISSING}" ]; then
    log "实际节点清单："
    cexec "cat /tmp/_geo_route_nodes.txt" | sed 's/^/    /'
    die "关键节点缺失：${MISSING} ⇒ **栈不完整，拒绝开车**（这正是 campus8-h 白跑 13 分钟的原因）"
fi
log "关键节点齐 ✓：${NEED_NODES}"

# ★ 判据 ⑤ 快照：**每一轮都要取**，不是只有"起栈那一轮" ✓（栈塌了这里就会失败 ⇒ 开场拦下 ✓）
cexec "mkdir -p ${ARTIFACTS_ROOT}/${RUN_ID}/nav2"
cexec "rm -f ${ARTIFACTS_ROOT}/${RUN_ID}/nav2/static_layer_check.json"
# ★★ `|| true` 不能省 ★★ —— `lib.sh` 是 `set -euo pipefail` ✓，而该脚本**成功时也返回 2** ✗
#   ⇒ 不加 `|| true` ⇒ **`set -e` 在调用方触发、整个脚本静默中止** ✗
#   —— 实测症状：只打印到"关键节点齐 ✓"、后面**一个字都没有**、`rc=2` ✗。
#   ★ 这正是 `lib.sh:571` 记过的那个坑（"`set -e` 在调用方触发 ⇒ 起栈脚本静默中止" ✓）⇒ 我又踩了一次 ✓。
cexec_t 200 "${NAS} && cd ${PKG_IN_CTR} && timeout 180 python3 scripts/campus_static_layer_check.py \
    --world worlds/campus_100.sdf \
    --json ${ARTIFACTS_ROOT}/${RUN_ID}/nav2/static_layer_check.json \
    --dump-grids ${ARTIFACTS_ROOT}/${RUN_ID}/nav2/grids.npz" >/dev/null 2>&1 || true
_rc=$?
# ★ 判据 = **产物在不在**，不是退出码 ✗ —— 实测该脚本**成功时也返回 2** ✗
#   （json 写出来了、障碍在两个图里都是致命 100 ✓，可 rc=2 ✓）
#   ⇒ 若按 rc 判 ⇒ **每次都被误判成失败、永远开不了车** ✗。
#   这与本项目早有的那条一致：**"发完信号不等于文件存在"** ⇒ 判产物 ✓。
if ! cexec "test -s ${ARTIFACTS_ROOT}/${RUN_ID}/nav2/static_layer_check.json"; then
    die "判据 ⑤ 快照失败（rc=${_rc} 且产物不存在）⇒ **拒绝开车**"
fi
log "判据 ⑤ 快照已取 ✓（脚本 rc=${_rc}，**按产物判** ✓）"
log "判据 ⑤ 快照已取 ✓：${RUN_ID}/nav2/static_layer_check.json"

# ★★ 2026-10-10（reviewer）：⑤ 必须**双 dump**，而且这要是个**判据、不是偶然** ★★
#   实测问题：上面**只调一次** ✗ ⇒ 复算清单第 4 项（"两次都过判 ＋ **活性量成对**"）**必缺** ✗ ⇒ 那一格给不出结论 ✗。
#   ★ 而 `campus8-k` 里出现"两份"是**两个不同调用点**凑出来的 ✗（`t0.*` 02:27:54 与 `grids.*` 02:30:21 ✓）
#     ⇒ ★ **偶然凑巧不算判据** ✓ ⇒ 在这里**固化成"同一脚本、固定间隔的第二次"** ✓。
#   ★★ 误读警告（reviewer 读码时自己先排掉的 ✗）：脚本里 `while … len(node.maps) < 2` 等的是
#     **两个话题**（`/rtk/static_map` + `/global_costmap/costmap` ✓），**不是"两次快照"** ✗ ——
#     ★ 一字之差、含义完全不同 ✓。
#   ★ 间隔 `GAP_S`：必须 **≥ 一次图刷新** 才能算"两次独立的观测" ✓；
#     ★ 这个值的**实测标定我还没做** ✗ ⇒ 现取 5 s 是**保守下界** ✓，下一轮起栈后按实际刷新率复核并调大 ✓（不许当已标定 ✗）。
GAP_S="${GAP_S:-5}"
sleep "${GAP_S}"
cexec_t 200 "${NAS} && cd ${PKG_IN_CTR} && timeout 180 python3 scripts/campus_static_layer_check.py \
    --world worlds/campus_100.sdf \
    --json ${ARTIFACTS_ROOT}/${RUN_ID}/nav2/static_layer_check_2.json \
    --dump-grids ${ARTIFACTS_ROOT}/${RUN_ID}/nav2/grids_2.npz" >/dev/null 2>&1 || true
_rc2=$?
# ★ 判据同第一次：**按产物判** ✗ 不按退出码（该脚本成功时也返回 2 ✗）
if ! cexec "test -s ${ARTIFACTS_ROOT}/${RUN_ID}/nav2/static_layer_check_2.json"; then
    log "★ 第二次 ⑤ dump **未落盘** ✗（rc=${_rc2}）⇒ 清单第 4 项**给不出结论** ✗（本轮仍可跑，但那一格空着 ✓）"
else
    log "判据 ⑤ 第二次 dump 已取 ✓（间隔 ${GAP_S}s）：${RUN_ID}/nav2/static_layer_check_2.json"
fi

cexec "test -s '${CALIB}'" || die "没有 geo_calib.json（${CALIB}）—— 两半标定都跑完才有。见 docs/campus-8-delivery.md §调定"
cexec "test -s '${ROUTE}'" || die "没有航线文件（${ROUTE}）"
log "标定：${CALIB}"
log "航线：${ROUTE}"

if [ -n "${CHECK}" ]; then
    # 快，前台跑 ✓
    cexec_t 60 "${NAS} && python3 ${PKG_IN_CTR}/scripts/geo_route.py \
        --calib '${CALIB}' --route '${ROUTE}' --out '${OUT}' --tol ${TOL} --v-still ${VSTILL} --check-only"
    RC=$?
    log "check-only 退出码 ${RC}（0=换算通过；2=标定无效已拒绝；5=正反解不自洽）"
    exit "${RC}"
fi

# ★★ 窄修开关（reviewer 2026-10-10 裁定 ②）★★
#   本脚本是**多航点顺序停靠**场景 ⇒ 必须打开 `accept_superseded_same_goal` ✓，
#   否则"同一目标的 SUCCEEDED 结果"会被 token 过期丢掉 ⇒ `arrived` 永不置位 ⇒
#   全局图一更新就重发目标 ⇒ **车离开刚到的点** ✗（实测 wp0 最小 0.0234 m、最终 4.08 m）。
#   **默认为关** ✓ ⇒ 阶段四~七的行为一字不变、其结论继续可复跑 ✓；只有本脚本会打开 ✓。
#   （该节点用 `get_parameter` **每次现读** ⇒ 运行中 set 立刻生效、不必重起栈 ✓。）
log "打开 goal_navigator 的窄修开关（仅本轮多航点模式）"
cexec_t 20 "${NAS} && timeout 15 ros2 param set /goal_navigator accept_superseded_same_goal true" 2>&1 | tail -1

# ★ 开车要好几十分钟 ⇒ **后台跑**，日志落在宿主能看到的地方 ✓
"${DOCKER_BIN}" exec -d "${CONTAINER_NAME}" bash -lc "${NAS} && cd ${PKG_IN_CTR} && \
    exec python3 scripts/geo_route.py --calib '${CALIB}' --route '${ROUTE}' --out '${OUT}' \
    --tol ${TOL} --v-still ${VSTILL} ${WAITALL} > ${LOG_F} 2>&1"
sleep 5
cexec "pgrep -f '[g]eo_route.py' >/dev/null" || die "geo_route 没起来，看 ${LOG_F}"
log "geo_route 已起（后台）。进度：tail -f ${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/geo_route_runner.log"

# 跟着日志走到结束（或超时）—— 单条命令就能看到全过程 ✓
for _ in $(seq 1 360); do
    if [ -s "${HOST_OUT}_log.json" ]; then
        log "已落盘：${HOST_OUT}_records.json / _log.json ✓"
        break
    fi
    cexec "pgrep -f '[g]eo_route.py' >/dev/null" || { log "geo_route 已退出"; break; }
    sleep 10
done
cexec "tail -12 ${LOG_F}"
# ★ 收工把开关关回去（**不留副作用**：下一个用这个容器的人拿到的仍是默认行为 ✓）
cexec_t 20 "${NAS} && timeout 15 ros2 param set /goal_navigator accept_superseded_same_goal false" 2>&1 | tail -1
python3 - "${HOST_OUT}_log.json" <<'PY' 2>/dev/null || true
import json, sys
d = json.load(open(sys.argv[1]))
print('到达 %s/%s 个点 ｜ 最差落点 d_gt = %s m' % (d['arrived'], d['points'], d.get('worst_d_gt_mean_m')))
PY

# ★★ 2026-10-12：走完**立刻**抓「走完之后」的全局图 —— ★ **内置步骤** ★★
#   为什么必须内置：这张图**只有在收栈之前**才抓得到 ✗ —— 靠外部守望去抢时间窗，
#   一旦那轮被提前停掉就**永远拿不到** ✗（2026-10-12 实测踩到：业主那轮提前关掉 ⇒ 守望必然落空 ✓）。
#   ★★ 参数与交付轮**逐字相同**（-t /global_costmap/costmap --free 0.196 --occ 0.65）⇒
#     两张 pgm 的 0／205／254 三档语义同源 ✓ ⇒ 才能并排比（✗ 参数不同就是两把尺子 ✗）。
#   ★ 抓完记四样：尺寸 ✓ ／ 值域直方图 ✓ ／ 参数 ✓ ／ **相对本步开始的秒数** ✓（复算者知道抓在哪个时点）。
#   ★ 整段**不致命**（✗ 不让它改变本脚本的退出码 ✗）：抓到就赚到，抓不到不影响航线结果 ✓。
_G_ROOT="${ARTIFACTS_ROOT}/${RUN_ID}"
_G_HOST="${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}"
_G_T0=$(date +%s)
if [ -x "$(command -v docker)" ] && [ -n "$(docker ps -q -f name="^${CONTAINER_NAME}$" 2>/dev/null)" ]; then
    log "★ 抓「走完之后」的全局图（收栈之前；内置）⇒ ${_G_HOST}/nav2/global_map_after.*"
    mkdir -p "${_G_HOST}/nav2"
    cexec_t 120 "${NAS} && timeout 60 ros2 run nav2_map_server map_saver_cli \
        -f ${_G_ROOT}/nav2/global_map_after -t /global_costmap/costmap --free 0.196 --occ 0.65" 2>&1 | tail -2 || true
    cexec_t 120 "${NAS} && python3 ${CONTAINER_WORKDIR}/scripts/rtk-nav2/container/globalmap_stats.py \
        --out ${_G_ROOT}/nav2/globalmap_stats_after.json" 2>&1 | tail -4 || true
    # ★ 宿主侧自检（纯读文件）：尺寸 ＋ 值域 ＋ 直方图 ⇒ 一眼看出参数同源没有 ✓
    if [ -s "${_G_HOST}/nav2/global_map_after.pgm" ]; then
        python3 "${SCRIPT_DIR}/pgm_hist.py" "${_G_HOST}/nav2/global_map_after.pgm" \
            "${_G_HOST}/nav2/global_map_after.hist.json" 2>&1 | tail -4 || true
    else
        log "✗ 没抓到 pgm（已收栈？）—— 不影响航线结果 ✓，但这一轮没有「走完后图」✗"
    fi
    {
        echo "# 走完之后图的抓取留痕（内置步骤）"
        echo "抓取相对本步开始的秒数: $(( $(date +%s) - _G_T0 ))"
        echo "命令: ros2 run nav2_map_server map_saver_cli -f <轮>/nav2/global_map_after -t /global_costmap/costmap --free 0.196 --occ 0.65"
        echo "统计: container/globalmap_stats.py --out <轮>/nav2/globalmap_stats_after.json"
        echo "自检: scripts/rtk-nav2/pgm_hist.py（尺寸 1250x1420 ＋ 值域 {0,205,254}）"
        echo "★ 与交付轮 final-off 的参数逐字相同 ⇒ 两张 pgm 可直接并排"
        echo "★ 记数时**分开**：pgm 的 0 值格 ≠ stats 的 occupied_gt0（✗ 别混 ✗）"
    } > "${_G_HOST}/nav2/after_grab_params.txt" 2>/dev/null || true
else
    log "（栈没在运行 ⇒ 跳过「走完之后图」的抓取；✗ 这不是错误 ✓）"
fi

