#!/usr/bin/env bash
# ★ 阶段九第 3 步：在**他们的场景**里依次走**他们的 41 个点**（★ 本步起**不再有仿射** ✓）。
#
# 用法：scripts/rtk-nav2/rev9-drive.sh [run-id] [--tol 0.25] [--check-only]
#
# ★★ 为什么**不复用** `geo-route.sh`（这是有意的 ✗，不是漏了 ✓）★★
#   `geo-route.sh` 里有一步**硬依赖**：
#       campus_static_layer_check.py --world worlds/campus_100.sdf
#   且**产物不在就 `die`**（"拒绝开车" ✗）。
#   ★ 而本步按业主口径 **全局图没有 `static_layer`** ✗ ⇒ 那条检查**不适用** ✓
#     （它查的是"静态层在不在图里" ⇒ 必然失败 ⇒ 会**恒拦**每一轮 ✗ = "恒不通过"的判据 ✓）。
#   ★ 另外它把世界路径**写死**成 `worlds/campus_100.sdf` ✗ —— 本步的世界是 `north_campus.sdf` ✓。
#   ⇒ 本脚本只保留**本步真正需要的**：① 关键节点断言 ✓ ② 跑 `geo_route.py` ✓
#     其余（起栈/就绪/指纹）仍由 `start-northcampus.sh` → `start-demo.sh` 提供 ✓，**没有复制实现** ✓。
#
# ★ 标定与航线都在 `artifacts/rtk-nav2/<run-id>/input/` ✓，
#   由 `rev9_calib_from_export.py` **从他们自己的导出反解**而来 ✓（残差 ~0.7 mm ✓，见 calib_report.json）。
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"; shift || true
TOL="0.25"; CHECK=""
while [ $# -gt 0 ]; do
    case "$1" in
        --tol) TOL="$2"; shift 2 ;;
        --check-only) CHECK="--check-only"; shift ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) die "未知参数：$1" ;;
    esac
done

stack_running || die "栈没在运行，请先 scripts/rtk-nav2/start-northcampus.sh ${RUN_ID}"
C_IN="${ARTIFACTS_ROOT}/${RUN_ID}/input"
C_OUT="${ARTIFACTS_ROOT}/${RUN_ID}/geo_route"
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"
PKG_IN_CTR="${CONTAINER_WORKDIR}/rtk_nav2_ws/src/robonix_rtk_nav2"
DRIVER="${PKG_IN_CTR}/scripts/geo_route.py"

for f in geo_calib.json route_geo.json; do
    cexec "test -s ${C_IN}/${f}" || die "缺 ${C_IN}/${f}（先跑 scripts/rtk-nav2/rev9_calib_from_export.py）"
done

# ---- ① 关键节点断言（★ 栈塌了就在这里拦下 ✓，✗ 不等到"顶着楼停住"才知道 ✗）----
NEED_NODES="/planner_server /global_costmap/global_costmap /controller_server /lifecycle_manager_nav2"
cexec_t 30 "${NAS} && ros2 node list" > /tmp/_rev9_nodes.txt 2>/dev/null
MISSING=""
for _n in ${NEED_NODES}; do
    grep -qx "${_n}" /tmp/_rev9_nodes.txt || MISSING="${MISSING} ${_n}"
done
if [ -n "${MISSING}" ]; then
    log "★ 缺节点：${MISSING} ⇒ **拒绝开车** ✗"
    log "  实际节点（$(wc -l < /tmp/_rev9_nodes.txt) 个）：$(tr '\n' ' ' < /tmp/_rev9_nodes.txt | head -c 400)"
    exit 1
fi
log "关键节点齐 ✓（$(wc -l < /tmp/_rev9_nodes.txt) 个）"

# ---- ② 开着车跑 ✓ ----
# ★ `--v-still 0.02`：本步的到达判据是"**进容差就发下一个**" ✓（业主口径：中间点不用停稳 ✓），
#   该阈值只用于**记录诊断量**（停稳时长），✗ 不用于放行 ✗ —— 见 geo_route.py 的文件头 ✓。
log "开始巡游（tol=${TOL} m；41 个点；★ 进容差即发下一个 ✓）"
cexec "${NAS} && python3 ${DRIVER} \
    --calib ${C_IN}/geo_calib.json \
    --route ${C_IN}/route_geo.json \
    --out ${C_OUT} \
    --tol ${TOL} ${CHECK}"
RC=$?
log "geo_route.py 退出码 = ${RC}"
exit "${RC}"
