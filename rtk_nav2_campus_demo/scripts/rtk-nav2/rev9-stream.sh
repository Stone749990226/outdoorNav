#!/usr/bin/env bash
# ★ 阶段九第 4 步：**运行中接点**巡游（★ 栈不重启 ✓）
#
# 用法：
#   rev9-stream.sh <run-id> [--tol 0.4] [--route <段文件>] [--done-after 30]
#   rev9-stream.sh <run-id> --append <段文件> [<段文件> …]
#       ★ `--append` = 把若干段的点**合并写回** `input/route_stream.json` ✓
#         —— ★ 用**临时文件 + mv 原子替换** ✓：✗ 不能原地写 ✗
#            （★ 节点每秒读一次 ✓，读到半截 JSON 会当"读不到"跳过 ✓ —— 但原子写能彻底避免 ✓）
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"; shift || true
TOL="0.4"; ROUTE=""; APPEND=""; SEGS=(); DONE_AFTER="30"
while [ $# -gt 0 ]; do
    case "$1" in
        --tol) TOL="$2"; shift 2 ;;
        --route) ROUTE="$2"; shift 2 ;;
        --done-after) DONE_AFTER="$2"; shift 2 ;;
        --append) APPEND=1; shift ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) SEGS+=("$1"); shift ;;
    esac
done

C_IN="${ARTIFACTS_ROOT}/${RUN_ID}/input"
C_OUT="${ARTIFACTS_ROOT}/${RUN_ID}/stream"
H_IN="${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/input"
STREAM="${C_IN}/route_stream.json"
# ★ 2026-10-11（阶段十 · 二分）：按**工作区名**取驱动路径 ✓（原来写死 `rtk_nav2_ws` ✗）
#   ★ 不设 `WORKSPACE_NAME` 时与原来**逐字符相同** ✓；设了则整条链路切到另一个工作区 ✓。
DRIVER="${CONTAINER_WORKDIR}/${WORKSPACE_NAME:-rtk_nav2_ws}/src/robonix_rtk_nav2/scripts/rev9_stream_route.py"

# ---------- 追加模式：把若干段合并、**原子**写回 route_stream.json ✓ ----------
if [ -n "${APPEND}" ]; then
    [ "${#SEGS[@]}" -gt 0 ] || die "--append 后面要给段文件"
    log "★ 运行中追加：${SEGS[*]}"
    python3 - "${H_IN}" "${SEGS[@]}" <<'PY'
import json, os, sys, glob
d = sys.argv[1]; segs = sys.argv[2:]
def segno(p):
    return int(''.join(c for c in os.path.basename(p) if c.isdigit()) or 0)
order = sorted(glob.glob(os.path.join(d, 'route_seg*.json')), key=segno)
# ★ 语义 = "把到第 N 段为止的全部点写进去" ✓ ——
#   ★ 既是"追加"（N 变大就多）✓、又**天然幂等** ✓（同一段喂两次不会重复 ✓）。
last = max(segno(s) for s in segs)
pts = []
for p in order:
    if segno(p) <= last:
        pts += json.load(open(p, encoding='utf-8'))['points']
out = {'frame_id': 'map', 'note': '★ 运行中追加到第 %d 段（共 %d 点）' % (last, len(pts)),
       'points': pts}
tmp = os.path.join(d, 'route_stream.json.tmp')
json.dump(out, open(tmp, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
os.replace(tmp, os.path.join(d, 'route_stream.json'))   # ★ 原子替换 ✓
print('  ✓ 已原子写回：共 %d 点（到第 %d 段）' % (len(pts), last))
PY
    exit 0
fi

# ---------- 正常运行：起流式巡游 ✓ ----------
# ★★ 2026-10-10：**不再用 `stack_running` 当前置守卫** ✗ ★★
#   实测（第 4 步）：栈健康 ✓（8 个关键进程 ✓、29 个节点 ✓），而 `lib.sh` 的 `stack_running`
#   判"没在运行" ✗ ⇒ **把车门挡在外面** ✓。根因同 §下面的节点断言：它建立在 `ros2 node list`
#   上 ✗，而**陈旧的 ros2 daemon** 会给出不完整的节点表 ✗。
#   ⇒ ★ 改成**只靠下面那条 daemon-fresh 的节点断言** ✓（它 `daemon stop` + 等 3 s + 数两遍 ✓，
#     并带**进程层交叉核对** ✓）—— ★ 一次判据就够，✗ 不叠一个更弱的 ✗。
#   ★ 本项目定式：**守卫比被测物更容易坏** ✓ ⇒ 少了那条守卫，反而少一处误拦 ✓。
for f in geo_calib.json; do
    cexec "test -s ${C_IN}/${f}" || die "缺 ${C_IN}/${f}"
done
[ -n "${ROUTE}" ] || ROUTE="${STREAM}"
[ -f "${H_IN}/$(basename "${ROUTE}")" ] || die "航线文件不存在：${ROUTE}"

NEED_NODES="/planner_server /global_costmap/global_costmap /controller_server /lifecycle_manager_nav2"
# ★★ 2026-10-10：**先 `ros2 daemon stop` 再数，且数两遍** ✗★★
#   实测（第 4 步）：就绪 38 s 后本守卫报"缺 `/planner_server` `/global_costmap`" ✗ ——
#   ★ 而**进程层两个都在** ✓、`daemon stop` 后节点表里也都有 ✓（29 个 ✓）
#   ⇒ ★ **是陈旧的 ros2 daemon 把健康的栈判成缺节点** ✗（★ "守卫比被测物更容易坏" ✓，本项目已记过同族多次 ✓）。
#   ★ 而且**新起的 daemon 要时间做发现** ✗ ⇒ 停完立刻数会**偏少** ✗
#   ⇒ 停 daemon → 等 3 s → 数 → 不够就**再数一遍**（两条独立读数一致才判 ✓）。
_assert_nodes() {
    cexec "source /opt/ros/humble/setup.bash && ros2 daemon stop" >/dev/null 2>&1 || true
    sleep 3
    for _try in 1 2; do
        cexec_t 40 "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && ros2 node list" \
            > /tmp/_rev9_stream_nodes.txt 2>/dev/null
        MISSING=""
        for _n in ${NEED_NODES}; do
            grep -qx "${_n}" /tmp/_rev9_stream_nodes.txt || MISSING="${MISSING} ${_n}"
        done
        [ -z "${MISSING}" ] && return 0
        sleep 5
    done
    return 1
}
if ! _assert_nodes; then
    log "★ 缺节点：${MISSING} ⇒ **拒绝开车** ✗（已数两遍 ✓）"
    log "  实际（$(wc -l < /tmp/_rev9_stream_nodes.txt) 个）：$(tr '\n' ' ' < /tmp/_rev9_stream_nodes.txt | head -c 400)"
    # ★ 交叉核对：**进程层**必须一起对得上 ✓（单靠节点表会被 daemon 骗 ✗）
    NP="$(cexec "for p in \$(ls /proc | grep -E '^[0-9]+\$'); do tr '\\0' ' ' < /proc/\$p/cmdline 2>/dev/null; echo; done | grep -cE 'planner_server|controller_server'")"
    log "  进程层交叉核对：planner/controller 进程数 = ${NP}（≥2 ⇒ 是**守卫**错了 ✗，不是栈 ✗）"
    exit 1
fi
log "关键节点齐 ✓（$(wc -l < /tmp/_rev9_stream_nodes.txt) 个，已 daemon stop + 数两遍 ✓）⇒ 起流式巡游（tol=${TOL} ✓）"

cexec "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && \
    python3 ${DRIVER} --calib ${C_IN}/geo_calib.json --route ${ROUTE} --out ${C_OUT} \
    --tol ${TOL} --done-after ${DONE_AFTER}"
RC=$?
log "流式巡游退出码 = ${RC}"
exit "${RC}"
