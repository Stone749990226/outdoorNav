#!/usr/bin/env bash
# 第五阶段（传感器增强）的**验收驱动**：判据 ①②③ + 性能记录，全部在**跑着的栈**上量。
#
# 用法：scripts/rtk-nav2/nav10-sensor-acceptance.sh <run-id> [--phase all|noecho|lowobs|regress|perf]
#                                            [--with-drive]
#
# 为什么把 A/B 的顺序写死成"先 without 再 with"（**这条最容易搞错**）：
#   nav2 的 Overwrite 语义下，**空闲格不会被"取消清除"变回未知**（未知 → 空闲是单向的，
#   除非有新观测把它标成障碍）。所以如果先测"补远点（after）"、再关掉开关测"before"，
#   第二次读到的 free 计数里**仍然带着第一次清出来的那片**，会得出"开关没用"的假结论。
#   因此：**起栈时就带 `emit_far_points:=false`**，先量 before，再重启那一个节点打开开关量 after。
#
# 停栈/起栈：本脚本**不**起栈也不停栈（栈由 start-demo.sh / stop-demo.sh 管），
# 只重启 `depth_points` 这一个节点（判据 ①的 A/B 需要）。
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID=""
PHASE="all"
WITH_DRIVE=0
while [ $# -gt 0 ]; do
    case "$1" in
        --phase) PHASE="$2"; shift 2 ;;
        --with-drive) WITH_DRIVE=1; shift ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) RUN_ID="$1"; shift ;;
    esac
done
RUN_ID="$(resolve_run_id "${RUN_ID}")"
DIR="$(artifact_dir "${RUN_ID}")"
CDIR="$(artifact_dir_c "${RUN_ID}")"
OUT="${DIR}/sensor5"
OUTC="${CDIR}/sensor5"
PKG_SCRIPTS_C="${WS_DIR}/src/robonix_rtk_nav2/scripts"
# **注意**：`nav4_map_box.py` 在 scripts/rtk-nav2/container/ 下（容器脚本目录），
# **不在**包的 scripts/ 里。第一轮就是写成了 `${PKG_SCRIPTS_C}/nav4_map_box.py`，
# 于是每档的地图读数都是 `python3: can't open file ...`（退出码 2）、
# 汇总里全是 `lethal=None`——脚本没报错退出，只是读数全空，很容易被当成"没有障碍进图"。
CONTAINER_SCRIPTS_C="${CONTAINER_WORKDIR}/scripts/rtk-nav2/container"
# 落点方位搜索窗口：默认 ±20°（在相机 60° 视场内，便于"0.30 m 档相机也该看到"的对照）。
# 车朝向不理想时，把 LOWOBS_SPREAD=180 放开成全向——**代价是那一档可能落在相机视场外**，
# 那份读数就只剩激光的贡献（相机侧旁证仍由 scan_probe.py 直接给点数与 z 分布，不受视场影响）。
LOWOBS_SPREAD="${LOWOBS_SPREAD:-20}"
SPAWN="/work/scripts/rtk-nav2/container/spawn_obstacle.sh"
TRAJ="${DIR}/trajectory.csv"
TRAJC="${CDIR}/trajectory.csv"

stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"
mkdir -p "${OUT}"

ce() { cexec_t "$1" "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && $2"; }

probe() {   # probe <容器内命令> <宿主产物前缀> <超时>
    local cmd="$1" label="$2" tmo="${3:-120}" rc
    set +e
    cexec_t "${tmo}" "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && ${cmd}" \
        > "${label}.log" 2>&1
    rc=$?
    set -e
    [ "${rc}" -eq 0 ] || log "  ** 探针 $(basename "${label}") 退出码 ${rc}（见 ${label}.log）**"
}

# ---------------------------------------------------------------- 判据 ①：无回波⇒空闲
phase_noecho() {
    log "===== 判据 ①：无回波 ⇒ 量程内判空闲（A/B = 关掉/打开"补远点"开关）====="
    # **默认不碰任何节点**：干净主线栈上是 `emit_far_points` 默认(true) → 只量 after；
    # 归因栈上用启动参数 `emit_far_points:=false` 起 → 只量 before。
    # 两次都是**新栈**（不是同栈切换开关）：同栈切换要杀/重启发布者，
    # 而第一轮正是那么做之后全局图冻住了，不能再用那种做法。
    # 只有显式给 `NOECHO_ALLOW_RESTART=1` 时才走"杀节点换开关"的老路。
    local n
    for n in ${NOECHO_LABELS:-before after}; do
        if [ "${n}" = "after" ] && [ "${NOECHO_ALLOW_RESTART:-0}" = "1" ]; then
            log "  切换到 after：重启 depth_points 并打开 emit_far_points"
            # 先杀旧节点再起新的（用 [x] 括号形式，避免匹配到执行命令的 shell 自己）
            cexec "pgrep -f '[r]tk_depth_points' | xargs -r kill -9 2>/dev/null; sleep 2; true" >/dev/null 2>&1 || true
            # 关键参数逐项与 launch 里一致（远裁剪 8.0 必须 > 相机源 obstacle_max_range 6.0）
            cexec_bg "${OUTC}/depth_points_after.log" \
                "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && \
                 ros2 run robonix_rtk_nav2 rtk_depth_points --ros-args -p use_sim_time:=true \
                 -p input_topic:=/rtk/camera/depth_image -p info_topic:=/rtk/camera/camera_info \
                 -p output_topic:=/rtk/camera/points -p far_clip:=8.0 -p emit_far_points:=true"
            sleep 25
        elif [ "${n}" = "after" ]; then
            log "  当前栈按 emit_far_points 默认值(true) 运行 → 只量 after（**不动节点**）"
        else
            log "  当前栈按启动参数 emit_far_points:=false 运行 → 只量 before（**不动节点**）"
        fi
        probe "python3 ${PKG_SCRIPTS_C}/noecho_probe.py --traj ${TRAJC} --label ${n} \
               --json ${OUTC}/noecho_${n}.json" "${OUT}/noecho_${n}" 120
    done
    python3 - "${OUT}" <<'PY' | tee "${OUT}/noecho_compare.txt"
import json, os, sys
d = sys.argv[1]
def load(n):
    try:
        return json.load(open(os.path.join(d, 'noecho_%s.json' % n), encoding='utf-8'))
    except Exception as exc:
        return {'error': str(exc)}
b, a = load('before'), load('after')
def show(tag, j):
    if 'error' in j:
        print('  %-6s ** 读不到产物：%s **' % (tag, j['error'])); return
    w = j['map']['whole']
    ne = j['no_echo_azimuths']
    print('  %-6s 全图 free=%d unknown=%d lethal=%d | 无回波方位 %d 条：free=%d unknown=%d'
          '（free 占比 %.3f / unknown 占比 %.3f）'
          % (tag, w['free'], w['unknown'], w['lethal'], ne['azimuth_count'],
             ne['state_counts']['free'], ne['state_counts']['unknown'],
             ne['free_ratio'], ne['unknown_ratio']))
print('判据 ①：无回波 ⇒ 量程内判空闲')
show('before', b); show('after', a)
if 'error' not in b and 'error' not in a:
    wb, wa = b['map']['whole'], a['map']['whole']
    nb, na = b['no_echo_azimuths'], a['no_echo_azimuths']
    print('  判定：全图 unknown %d → %d（%+d）；free %d → %d（%+d）'
          % (wb['unknown'], wa['unknown'], wa['unknown'] - wb['unknown'],
             wb['free'], wa['free'], wa['free'] - wb['free']))
    print('        无回波方位上 free 占比 %.3f → %.3f；unknown 占比 %.3f → %.3f'
          % (nb['free_ratio'], na['free_ratio'], nb['unknown_ratio'], na['unknown_ratio']))
    print('  格级证据（after，无回波方位的逐格代价）：')
    for s in na.get('samples', []):
        cells = ', '.join('(%.1fm,%d,%s)' % (c['d_m'], c['cost'], c['state']) for c in s['cells'][:6])
        print('    方位%+d°: %s' % (s['rel_deg'], cells))
PY
}

# ---------------------------------------------------------------- 判据 ②：低矮障碍
# 每一档：量基线 → 放障碍 → 量地图紧盒 + 传感器侧旁证 → 移走 → 再量（顺便验"能清掉"）
lowobs_case() {   # lowobs_case <tag> <height_m> <dist_m>
    local tag="$1" h="$2" d="$3"
    local tx ty
    # 落点由 nav_target_point.py 算（正前方 ±25° 内净空最好的方位；见该脚本的说明）
    read -r tx ty < <(python3 "${PKG_DIR}/scripts/nav_target_point.py" "${TRAJ}" "${d}" \
        --spread-deg "${LOWOBS_SPREAD:-20}" 2>"${OUT}/target_${tag}.log")
    [ -n "${tx:-}" ] || die "算不出 ${tag} 的落点（见 ${OUT}/target_${tag}.log）"
    log "  档位 ${tag}：高 ${h} m、距 ${d} m，落点 (${tx}, ${ty})；$(tail -1 "${OUT}/target_${tag}.log")"
    local base="${OUT}/lowobs_${tag}"
    # **两张图都读**：全局图在 planner_server、局部图在 controller_server，本轮实测
    # 全局图的**内容会冻结**（见 sensor5/freeze_*），局部图正常——只读全局图会把
    # "传感器没进图"和"那张图不动了"混为一谈。
    probe "python3 /tmp/both_maps_probe.py ${tx} ${ty} 0.35 --json ${OUTC}/lowobs_${tag}_baseline.json" \
        "${base}_baseline" 90
    log "  放障碍（0.4 m 见方、高 ${h} m）"
    cexec_t 60 "${SPAWN} spawn ${tx} ${ty} 0.4 ${h}" > "${base}_spawn.log" 2>&1 \
        || log "  ** 放障碍失败（见 ${base}_spawn.log）**"
    sleep 15
    probe "python3 /tmp/both_maps_probe.py ${tx} ${ty} 0.35 --json ${OUTC}/lowobs_${tag}_present.json" \
        "${base}_present" 90
    # `--box-z` 给障碍半高：相机侧按 3D 盒取点，避免把障碍**后面**的墙算进来
    # （第一轮就是没给，0.30 m@2 m 那档报出 2267 个 z>=0.30 的点，实际是远处的墙）
    probe "python3 ${PKG_SCRIPTS_C}/scan_probe.py --traj ${TRAJC} --x ${tx} --y ${ty} \
           --box-z $(python3 -c "print(${h}/2.0)") \
           --json ${OUTC}/lowobs_${tag}_scan.json" "${base}_scan" 90
    log "  移走障碍"
    cexec_t 60 "${SPAWN} remove" > "${base}_remove.log" 2>&1 || true
    sleep 15
    probe "python3 /tmp/both_maps_probe.py ${tx} ${ty} 0.35 --json ${OUTC}/lowobs_${tag}_removed.json" \
        "${base}_removed" 90
    python3 - "${OUT}" "${tag}" "${h}" "${d}" <<'PY' | tee -a "${OUT}/lowobs_summary.txt"
import json, os, sys
d, tag, h, dist = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
def tight(n):
    """返回 (全局图 lethal, 局部图 lethal)；读不到就给 None。"""
    try:
        j = json.load(open(os.path.join(d, 'lowobs_%s_%s.json' % (tag, n)), encoding='utf-8'))
        g = j.get('global_map', {}).get('lethal')
        l = j.get('local_map', {}).get('lethal')
        return {'lethal': g, 'local_lethal': l}
    except Exception as exc:
        return {'lethal': None, 'local_lethal': None, 'error': str(exc)}
def scan():
    try:
        j = json.load(open(os.path.join(d, 'lowobs_%s_scan.json' % tag), encoding='utf-8'))
        return j['scan'].get('target', {}), j.get('camera', {})
    except Exception as exc:
        return {'error': str(exc)}, {'error': str(exc)}
b, p, r = tight('baseline'), tight('present'), tight('removed')
t, cam = scan()
print('  高 %s m / 距 %s m：**全局图**紧盒 lethal 基线=%s → 放上=%s → 移走=%s'
      % (h, dist, b.get('lethal'), p.get('lethal'), r.get('lethal')))
print('      **局部图**（base_link，活的那张）lethal 基线=%s → 放上=%s → 移走=%s'
      % (b.get('local_lethal'), p.get('local_lethal'), r.get('local_lethal')))
print('     激光侧：%s' % (('几何距离 %s m，最近回波 %s m，命中=%s（差 %s m）'
                          % (t.get('geom_distance_m'), t.get('nearest_echo_m'),
                             t.get('hit'), t.get('delta_m'))) if 'geom_distance_m' in t else t))
print('     相机侧：%s' % (('目标 ±0.35 m 内 %s 个点，z∈[%s, %s]（中位 %s）；'
                          '其中 z>=0.30（相机源会交给 nav2 的）=%s'
                          % (cam.get('points_in_box'), cam.get('z_min'), cam.get('z_max'),
                             cam.get('z_median'), cam.get('camera_source_would_mark')))
                        if 'points_in_box' in cam else cam))
PY
}

phase_lowobs() {
    log "===== 判据 ②：低矮障碍（0.15 / 0.30 m × 1 / 2 / 4 m）====="
    mark_injection "${RUN_ID}" "判据 ②：视场内按档放/移 0.4 m 见方矮障碍（高 0.15/0.30 m × 距 1/2/4 m）"
    : > "${OUT}/lowobs_summary.txt"
    local h d
    # 0.15 / 0.30 是 brief 要的两档；**0.60 是"相机肯定看得见"的对照组**——
    # 0.30 那档原本想当对照，但相机源阈值恰为 0.30、箱顶面压在边界上，实测相机贡献 0
    # （0.30 顶面 = 0.30 恰好在边界），所以那档当不了对照；0.60 的顶面远高于 0.30，
    # 相机必然会出点，这样"是激光起作用"才有干净的分离证据。
    for h in 0.15 0.30 0.60; do
        for d in 1 2 4; do lowobs_case "h${h/./}d${d}" "${h}" "${d}"; done
    done
    log "  ---- 探测下限：扫描面在 0.11 m（base_link +0.10 再叠 base_joint 的 0.010），"
    log "       所以围着它上下各测一档（0.10 / 0.12 / 0.15 m，距 1.5 m）----"
    # 0.10 m 那档上一轮**没取到有效读数**（探针没产出 JSON），本轮只留 0.12 m 做下限点；
    # 0.10 m 按几何预期在扫描面（0.11 m）之下、应当不可见，但**没有实测读数就不写"已验证不可见"**。
    for h in 0.12; do lowobs_case "h${h/./}d15" "${h}" "1.5"; done
    cat "${OUT}/lowobs_summary.txt"
}

# ---------------------------------------------------------------- 判据 ③：不回归
phase_regress() {
    log "===== 判据 ③：不回归（墙/方块/立柱/4 段独立墙仍在、幽灵格不回来、控制权归属）====="
    probe "python3 ${PKG_SCRIPTS_C}/obstacle_map_check.py --json ${OUTC}/obstacle_map_check.json" \
        "${OUT}/obstacle_map_check" 120
    # **分项在场性**（第一轮漏掉的一项：数据在 json 里，只是没读出来）
    if [ -f "${OUT}/obstacle_map_check.json" ]; then
        python3 "${PKG_DIR}/scripts/obstacle_present_report.py" "${OUT}/obstacle_map_check.json" \
            | tee "${OUT}/obstacles_present.txt" || true
    fi
    probe "python3 ${PKG_SCRIPTS_C}/nav2_probe.py geometry --dump-snapshot ${OUTC}/global_after.npz \
           --json ${OUTC}/geometry.json" "${OUT}/geometry" 120
    # 容器里的 /work 就是宿主项目根，所以同一个文件在宿主侧就是 ${OUT}/global_after.npz
    if [ -f "${OUT}/global_after.npz" ]; then
        probe "python3 ${PKG_SCRIPTS_C}/phantom_cells.py ${OUTC}/global_after.npz \
               --json ${OUTC}/phantom_cells.json" "${OUT}/phantom_cells" 120
    else
        log "  ** 没拿到全局图快照（${OUT}/global_after.npz），跳过 phantom_cells **"
    fi
    # 控制权归属：`/cmd_vel` 的发布者必须只有 controller_server
    cexec "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && \
           timeout 30 ros2 topic info -v /cmd_vel" > "${OUT}/cmd_vel_publishers.txt" 2>&1 || true
    log "  /cmd_vel 发布者：" && grep -A2 "Publisher count" "${OUT}/cmd_vel_publishers.txt" | head -5

    # **8 m 假墙自检**（本轮实测出来的风险点：nav2 把无回波束放在 range_max-1e-4 = 7.9999 m，
    # 若 `obstacle_max_range` 取 8.0，这些束会被标成致命障碍 → 开阔方向长出一圈假墙）
    python3 - "${OUT}" <<'PY' | tee "${OUT}/ring8m_check.txt"
import json, os, sys
d = sys.argv[1]
try:
    j = json.load(open(os.path.join(d, 'obstacle_map_check.json'), encoding='utf-8'))
except Exception as exc:
    print('  ** 读不到 obstacle_map_check.json：%s **' % exc); raise SystemExit
far = j['lethal_far_from_real_surface']
print('  离任何真实障碍表面 > %.2f m 的致命格：%d 个（最远 %s m，bbox %s）'
      % (far['threshold_m'], far['count'], far['max_distance_m'], far['bbox_xy']))
print('  → 这一项要跟"已知基线"比（第三阶段 0.30 时是 0）；若出现**成环**的、'
      '半径约 8 m 的致命格，就是无回波束被标成了障碍（把 scan 源的 obstacle_max_range 取 7.5 可解）。')
PY
}

# ---------------------------------------------------------------- 判据 4（参考）：性能
phase_perf() {
    log "===== 性能（不作判据，只记一笔）====="
    local t
    for t in /rtk/scan /rtk/camera/points /global_costmap/costmap /local_costmap/costmap; do
        local safe
        safe="$(printf '%s' "${t}" | tr '/' '_')"
        cexec_t 25 "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && \
                    timeout 15 ros2 topic hz --window 30 ${t}" > "${OUT}/hz${safe}.log" 2>&1 || true
        log "  ${t}: $(grep -m1 average "${OUT}/hz${safe}.log" 2>/dev/null || echo '(没取到)')"
    done
    cexec "top -bn1 -o %CPU | head -12" > "${OUT}/top.txt" 2>&1 || true
    cexec "cat /proc/loadavg; nproc" > "${OUT}/loadavg.txt" 2>&1 || true
    log "  容器内 CPU/负载见 ${OUT}/top.txt、${OUT}/loadavg.txt"
}

case "${PHASE}" in
    all) phase_noecho; phase_lowobs; phase_regress; phase_perf ;;
    noecho) phase_noecho ;;
    lowobs) phase_lowobs ;;
    regress) phase_regress ;;
    perf) phase_perf ;;
    *) die "未知 --phase：${PHASE}（all|noecho|lowobs|regress|perf）" ;;
esac

if [ "${WITH_DRIVE}" -eq 1 ]; then
    log "===== 判据 ③ 的另一半：增删障碍**走过**能更新（复用 nav4-obstacle-pass.sh）====="
    scripts/rtk-nav2/nav4-obstacle-pass.sh "${RUN_ID}" 2>&1 | tail -30
fi

log "产物：${OUT}（容器内 ${OUTC}）"
log "  noecho_compare.txt ①前后计数 + 格级证据 | lowobs_summary.txt ②各档读数"
log "  obstacle_map_check.json / phantom_cells.json / ring8m_check.txt ③ | cmd_vel_publishers.txt 控制权"
