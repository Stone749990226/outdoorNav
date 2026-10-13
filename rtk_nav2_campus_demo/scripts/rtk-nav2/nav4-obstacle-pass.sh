#!/usr/bin/env bash
# 判据 9 第二半（"增删障碍后**走过**能实时更新图"）的**专项复测**。
#
# 为什么要在主流程之外单跑一遍：主流程把这一段接在"到达 B"之后，而 rev4-2 那轮车在
# 第 2 米就楔住了（规划照算、但 FollowPath 不发，见 nav4/freeze_probe_raw.txt），
# 于是主流程里"等车开到障碍前 1.2 m"永远不成立，量出来的"障碍在位 lethal=0"会是
# **假阴性**（相机压根没往那边看），不能当"图没更新"的证据。
#
# 本脚本因此分两步，**第一步不依赖车能动**：
#   ① 视场内放/移（reviewer 第三阶段用过的办法）：取车当前位置与朝向，把 0.4 m 见方的
#      临时障碍放到车正前方 1.6 m、横偏 0.5 m 处（在 60° 视场内、离墙 >0.8 m），
#      量紧盒子（±0.35 m）→ 放上去 → 量 → 移走 → 量。**车不需要动**。
#   ② 再试一次"走过"：把障碍放到南北向路线上，让车从 N 开到 S、再从 S 开回 N，
#      在障碍前 1.2 m 处各量一次（几何与主流程一致）。车若楔住到不了，如实记为
#      "本次没能走到"，不拿假阴性当结论。
#
# 用法：scripts/rtk-nav2/nav4-obstacle-pass.sh <run-id>
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
DIR="$(artifact_dir "${RUN_ID}")"
CDIR="$(artifact_dir_c "${RUN_ID}")"
N4="${DIR}/nav4";   N4C="${CDIR}/nav4"
TRAJ="${DIR}/trajectory.csv"
CONTAINER_SCRIPTS="/work/scripts/rtk-nav2/container"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SPAWN="/work/scripts/rtk-nav2/container/spawn_obstacle.sh"

PASS_TIMEOUT="${PASS_TIMEOUT:-240}"
ARRIVE_TIMEOUT="${ARRIVE_TIMEOUT:-240}"
FWD="${FWD:-1.6}"; LAT="${LAT:-0.5}"

stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"
mkdir -p "${N4}"

probe() {
    local label="$1" tmo="$2" cmd="$3" rc
    log "  探针 $(basename "${label}")"
    set +e
    cexec_t "${tmo}" "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && ${cmd}" \
        > "${label}.log" 2>&1
    rc=$?
    set -e
    [ "${rc}" -eq 0 ] || log "  ** 探针 $(basename "${label}") 退出码 ${rc}，见 ${label}.log **"
    [ -s "${label}.json" ] || log "  ** 探针 $(basename "${label}") 产物为空 **"
}

box() {   # box <label> <x> <y>
    probe "${N4}/$1" 90 \
        "python3 ${CONTAINER_SCRIPTS}/nav4_map_box.py --x $2 --y $3 --label $1 \
         --json ${N4C}/$1.json --snapshot ${N4C}/$1.npz"
    python3 - "${N4}/$1.json" <<'PY' || true
import json, sys
try:
    d = json.load(open(sys.argv[1], encoding='utf-8'))
except Exception as exc:
    print('  %s: ** 读不到产物（%s）**' % (sys.argv[1], exc)); raise SystemExit
t, w = d['tight_0p35'], d['wide_0p90']
print('  %-30s (%.2f,%.2f) tight(±0.35m): lethal=%3d inflated=%3d free=%3d unknown=%3d | wide lethal=%3d | 全图 lethal=%d'
      % (d['label'], d['x'], d['y'], t['lethal'], t['inflated_only'], t['free'],
         t['unknown'], w['lethal'], d['whole_map']['lethal']))
PY
}

obstacle_count() {
    cexec "ign model --list 2>/dev/null | grep -c test_obstacle" 2>/dev/null | tr -d '\r' | tail -1
}

# 车当前真值位姿（truth CSV 的最后一条；yaw 即 base_link 朝向）
read -r RX RY RYAW <<<"$(python3 - "${TRAJ}" <<'PY'
import csv, sys
rows = []
for r in csv.DictReader(open(sys.argv[1], encoding='utf-8')):
    try:
        rows.append((float(r['gt_x']), float(r['gt_y']), float(r['gt_yaw'])))
    except Exception:
        pass
print('%.3f %.3f %.3f' % rows[-1]) if rows else print('0 0 0')
PY
)"
# 视场内的落点：正前方 FWD、横偏 LAT（再用场地几何确认离墙 > 0.8 m，否则左右换边）
read -r OX OY SIDE <<<"$(NAV4_GEOM_DIR="${PKG_DIR}/scripts" python3 - "${RX}" "${RY}" "${RYAW}" "${FWD}" "${LAT}" <<'PY'
import math, os, sys
sys.path.insert(0, os.environ['NAV4_GEOM_DIR'])
from map_geometry import nearest_surface
rx, ry, yaw, fwd, lat = (float(v) for v in sys.argv[1:6])
best = None
for sign in (1.0, -1.0):
    x = rx + math.cos(yaw) * fwd - math.sin(yaw) * lat * sign
    y = ry + math.sin(yaw) * fwd + math.cos(yaw) * lat * sign
    d = nearest_surface(x, y)
    if best is None or d > best[3]:
        best = (x, y, sign, d)
print('%.3f %.3f %.0f' % (best[0], best[1], best[2]))
PY
)"
log "车真值位姿 (${RX}, ${RY}, yaw=${RYAW}) → 视场内落点 (${OX}, ${OY})（横偏侧 ${SIDE}）"

log "===== ① 视场内放/移障碍（**不依赖车能动**）====="
box box_fov_baseline "${OX}" "${OY}"
log "  放障碍"
cexec_t 60 "${SPAWN} spawn ${OX} ${OY} 0.4 1.0" > "${N4}/fov_spawn.log" 2>&1 || log "** 放障碍失败 **"
log "  世界里的 test_obstacle 个数：$(obstacle_count)"
sleep 15
box box_fov_present "${OX}" "${OY}"
log "  移走障碍"
cexec_t 60 "${SPAWN} remove" > "${N4}/fov_remove.log" 2>&1 || true
log "  世界里的 test_obstacle 个数：$(obstacle_count)"
sleep 15
box box_fov_after_removal "${OX}" "${OY}"

log '===== ② 再试一次"走过"：障碍放到 (3.0,0.0)，车 N→S→N 各经过一次 ====='
OBST2_X=3.0; OBST2_Y=0.0
box box_pass_baseline "${OBST2_X}" "${OBST2_Y}"
cexec_t 60 "${SPAWN} spawn ${OBST2_X} ${OBST2_Y} 0.4 1.0" > "${N4}/pass_spawn.log" 2>&1 || true
mark_injection "${RUN_ID}" "判据 9 专项复测：(${OBST2_X},${OBST2_Y}) 放 0.4 m 临时障碍 → 走过 → 移走 → 再走过"
set +e
scripts/rtk-nav2/nav-to.sh "${RUN_ID}" 2.6 -2.6 2>&1 | tail -3
set -e
set +e
python3 "${SCRIPT_DIR}/nav-truth.py" wait "${TRAJ}" --target 2.6 1.2 --tol 1.0 \
    --timeout "${PASS_TIMEOUT}" --json "${N4}/pass_southbound_waypoint.json" > "${N4}/pass_southbound_waypoint.txt" 2>&1
log "  南向途中（障碍前 1.2 m）判定退出码 $?"
python3 "${SCRIPT_DIR}/nav-truth.py" wait "${TRAJ}" --target 2.6 -2.6 --tol 0.30 \
    --timeout "${ARRIVE_TIMEOUT}" --json "${N4}/pass_S_arrival.json" > "${N4}/pass_S_arrival.txt" 2>&1
log "  到达 S 判定退出码 $?"
set -e
box box_pass_obstacle_present "${OBST2_X}" "${OBST2_Y}"
cexec_t 60 "${SPAWN} remove" > "${N4}/pass_remove.log" 2>&1 || true
set +e
scripts/rtk-nav2/nav-to.sh "${RUN_ID}" 2.6 2.6 2>&1 | tail -3
set -e
set +e
python3 "${SCRIPT_DIR}/nav-truth.py" wait "${TRAJ}" --target 2.6 -1.2 --tol 1.0 \
    --timeout "${PASS_TIMEOUT}" --json "${N4}/pass_northbound_waypoint.json" > "${N4}/pass_northbound_waypoint.txt" 2>&1
log "  北向途中判定退出码 $?"
python3 "${SCRIPT_DIR}/nav-truth.py" wait "${TRAJ}" --target 2.6 2.6 --tol 0.30 \
    --timeout "${ARRIVE_TIMEOUT}" --json "${N4}/pass_N_arrival.json" > "${N4}/pass_N_arrival.txt" 2>&1
log "  到达 N 判定退出码 $?"
set -e
box box_pass_after_removal "${OBST2_X}" "${OBST2_Y}"

log "===== ③ 图上看一眼（PNG）====="
for pair in "box_fov_present:pass_map_fov_present" "box_fov_after_removal:pass_map_fov_removed" \
            "box_pass_obstacle_present:pass_map_pass_present" "box_pass_after_removal:pass_map_pass_removed"; do
    src="${pair%%:*}"; dst="${pair##*:}"
    python3 "${SCRIPT_DIR}/nav-truth.py" render "${N4}/${src}.npz" --csv "${TRAJ}" \
        --png "${N4}/${dst}.png" >/dev/null 2>&1 || true
done

python3 - "${N4}" > "${N4}/pass_summary.txt" 2>&1 <<'PY' || true
import json, os, sys

d = sys.argv[1]
def tight(name):
    try:
        j = json.load(open(os.path.join(d, name), encoding='utf-8'))
        return j['tight_0p35'], j
    except Exception as exc:
        return {'error': str(exc)}, {}

print('判据 9（第二半）专项复测汇总')
for tag, keys in (('① 视场内放/移（不依赖车能动）',
                   ('box_fov_baseline', 'box_fov_present', 'box_fov_after_removal')),
                  ('② 放到南北路线上、走过',
                   ('box_pass_baseline', 'box_pass_obstacle_present', 'box_pass_after_removal'))):
    print(tag)
    vals = []
    for k in keys:
        t, j = tight(k + '.json')
        vals.append(t.get('lethal'))
        print('   %-28s lethal=%s inflated=%s free=%s unknown=%s'
              % (k, t.get('lethal'), t.get('inflated_only'), t.get('free'), t.get('unknown')))
    b, p, a = vals
    if None not in (b, p, a):
        print('   判定：增障碍 %s -> %s（%s）；移走 %s -> %s（%s）'
              % (b, p, '图实时更新' if p > b else '**没看到增加**',
                 p, a, '图实时更新' if a < p else '**没看到清除**'))
PY
cat "${N4}/pass_summary.txt" 2>/dev/null || true
log "产物：${N4}/box_fov_*.json、box_pass_*.json、pass_summary.txt、pass_map_*.png"
