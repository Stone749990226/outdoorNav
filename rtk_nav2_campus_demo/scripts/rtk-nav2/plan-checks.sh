#!/usr/bin/env bash
# 判据 4（规划可用）的成套检查。单独一个脚本，因为**目标的选取本身有讲究**——
# 第一版把"未知区域目标"选成了 (0, 12)：那在 40x40 m 的图里是**地面方板之外**（板边 ±11.35），
# 任何路径都必然穿墙，"不穿墙"这条判据就变成了伪失败。实测那一版果然报了 12 次穿墙。
#
# 三条检查：
#   P1 路线下一个航点：从机器人当前位姿规划到路线上要去的下一个角 —— 必须出路径、不穿墙。
#   P2 未知区域目标：从固定起点规划到院内**没走过、也没看到过**的点 —— 必须出路径
#      （这就是 `allow_unknown: true` 生效的证据），且路径上要真有 unknown 格。
#   P3 地图更新后路径随之变化：P2 的起点/终点**固定不变**，先规划一次，
#      跑一圈让图长出来，再用同样的起终点规划一次 —— 两次路径必须不同，
#      否则说明规划器根本不看图（= 在空图上瞎走）。
#
# 用法：scripts/rtk-nav2/plan-checks.sh <run-id> [--with-lap]
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
WITH_LAP=0
[ "${2:-}" = "--with-lap" ] && WITH_LAP=1
DIR="$(artifact_dir "${RUN_ID}")/nav2"
CDIR="$(artifact_dir_c "${RUN_ID}")/nav2"
mkdir -p "${DIR}"
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"
SCRIPTS_IN="${WS_DIR}/src/robonix_rtk_nav2/scripts"

stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"

# 院内、离路线（方环 ±2.6）足够远、又在地面方板（±11.35）以内 → 大概率是没观测过的点
UNKNOWN_X=-3.5
UNKNOWN_Y=-3.5
# P2/P3 用**固定起点**：两次规划必须是同一对起终点，否则"路径变了"可能只是因为起点变了
FIXED_START_X=2.6
FIXED_START_Y=-2.6

plan() {
    local out="$1" label="$2"
    shift 2
    cexec_t 200 "${NAS} && python3 ${SCRIPTS_IN}/plan_path.py $* \
        --json ${CDIR}/$(basename "${out}") --label '${label}'" \
        > "${DIR}/$(basename "${out}" .json).txt" 2>&1 || true
}

log "===== P1：规划到路线下一个航点（起点 = 机器人当前位姿）====="
plan "${DIR}/plan_next_waypoint.json" "路线下一个航点（方环四角顺序）" --goal 2.6 2.6

log "===== P2：规划到院内未观测点 (${UNKNOWN_X}, ${UNKNOWN_Y})（起点固定 (${FIXED_START_X}, ${FIXED_START_Y})）====="
plan "${DIR}/plan_before_lap.json" "未知区域目标（起点固定；跑圈前）" \
    --goal "${UNKNOWN_X}" "${UNKNOWN_Y}" --start "${FIXED_START_X}" "${FIXED_START_Y}"

if [ "${WITH_LAP}" -eq 1 ]; then
    log "===== 跑一圈让图长出来（P3 的地图更新来源）====="
    set +e
    scripts/rtk-nav2/run-route.sh "${RUN_ID}" --timeout 420 2>&1 | tail -6
    set -e
fi

log "===== P3：同样的起终点再规划一次（与 P2 对比）====="
plan "${DIR}/plan_after_lap.json" "未知区域目标（起点固定；跑圈后）" \
    --goal "${UNKNOWN_X}" "${UNKNOWN_Y}" --start "${FIXED_START_X}" "${FIXED_START_Y}"

python3 - "${DIR}/plan_before_lap.json" "${DIR}/plan_after_lap.json" \
    > "${DIR}/plan_map_update_comparison.json" 2>&1 <<'PY' || true
import json
import sys


def load(path):
    try:
        with open(path, encoding='utf-8') as fh:
            return json.load(fh)
    except Exception as exc:
        return {'_load_error': str(exc)}


a, b = load(sys.argv[1]), load(sys.argv[2])
out = {'before': {'planned': a.get('planned'), 'poses': a.get('poses'),
                  'length_m': a.get('path_length_m'),
                  'cell_class': a.get('cell_class_on_path'),
                  'crosses_wall': a.get('crosses_wall'), 'error': a.get('error')},
       'after': {'planned': b.get('planned'), 'poses': b.get('poses'),
                 'length_m': b.get('path_length_m'),
                 'cell_class': b.get('cell_class_on_path'),
                 'crosses_wall': b.get('crosses_wall'), 'error': b.get('error')},
       'note': '起终点完全相同，只有地图不同。两次路径若一致，说明规划器没在用这张图。'
               '"before" 那次跑到时图上基本还全是未知（allow_unknown 会让它走直线），'
               '"after" 那次应当绕开已经建出来的障碍'}
if a.get('poses') is not None and b.get('poses') is not None:
    out['path_changed'] = (a.get('poses') != b.get('poses')
                           or abs((a.get('path_length_m') or 0)
                                  - (b.get('path_length_m') or 0)) > 0.05)
if a.get('cell_class_on_path') and b.get('cell_class_on_path'):
    out['unknown_cells_before'] = a['cell_class_on_path'].get('unknown')
    out['unknown_cells_after'] = b['cell_class_on_path'].get('unknown')
print(json.dumps(out, ensure_ascii=False, indent=2))
PY

cat "${DIR}/plan_map_update_comparison.json" 2>/dev/null | tail -30
log "产物：${DIR}/plan_{next_waypoint,before_lap,after_lap}.json + plan_map_update_comparison.json"
