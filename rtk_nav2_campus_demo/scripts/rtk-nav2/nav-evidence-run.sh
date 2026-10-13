#!/usr/bin/env bash
# 第四步（目标点导航：规划—跟随—实时重规划闭环）的**证据采集主流程**：
# 一条命令跑出判据 1–9 的产物（口径见 docs/rtk-nav2-nav-brief.md §5）。
#
# 为什么要有主流程、以及**顺序为什么是这个顺序**（判据之间有硬依赖，手敲必漏必串）：
#   ① **判据 1（初始无地图）只能在干净起栈的那一瞬间采**，而且旁路记录器必须在
#      `start-demo.sh` **之前**启动：`/global_costmap/costmap` 是 transient_local 发布，
#      起栈之后再订阅拿到的是"最后一帧"、不是第一帧。同时车一停下相机就开始建图，
#      晚十几秒"全未知"就没了。→ 所以本脚本**自己先停掉已有的栈**，再起新的。
#   ② **判据 2/3 需要"给 A 时的那条开局路径"**：`/rtk/plan_path` 的订阅必须在**发目标之前**
#      建立（记录器承担）。事后补采拿不到历史。
#   ③ **判据 5 要的是给 B 的第一条路径**：它只能在"到达 A、地图已经建出来"之后看。
#      因此本流程**绝不调用 `plan_path.py`**——它也往 `/rtk/plan_path` 发路径，
#      会把"被接受的路径序列"污染成掺了人工探针的序列（第三阶段的 plan-checks 可以那样用，
#      第四步判据 3/5 不行）。
#   ④ **判据 4/6 的到达取真值**（`trajectory.csv`）：到达那一刻立刻存图——
#      车一继续走图就在变，"到达时的地图"只有那一刻采得到。
#   ⑤ **判据 9 的第二半必须真的"走过"**：放障碍 → 车向南开过去（在障碍前 1.2 m 处取数）
#      → 移走 → 车再向北开回来经过同一处（同样在障碍前 1.2 m 处取数）。
#      只在原地看是第三阶段的口径，本步的口径是"增删障碍后走过能实时更新图"。
#   ⑥ 判据 7/8/9 的第一、三半是全局性质，放最后汇总（不依赖顺序）。
#   ⑦ **全程不停栈**：停栈之后采的任何数都是无效证据（这条吃过亏）。
#
# 用法：scripts/rtk-nav2/nav-evidence-run.sh <run-id>
#   （脚本会先停掉当前的栈、再用 <run-id> 起一个全新的，因为判据 1 要求干净起栈）
#
# 两段式演示的几何：起点 (2.6,-2.6) → A(-2.6,2.6) → B(2.6,2.6)；
# 之后为判据 9 再走 S(2.6,-2.6) 与 N(2.6,2.6) 来回，途中在 (2.75,0.0) 增删临时障碍。
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
DIR="$(artifact_dir "${RUN_ID}")"
CDIR="$(artifact_dir_c "${RUN_ID}")"
N4="${DIR}/nav4";        N4C="${CDIR}/nav4"
REC="${DIR}/nav4rec";    RECC="${CDIR}/nav4rec"
TRAJ="${DIR}/trajectory.csv"
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"
SCRIPTS_IN="${WS_DIR}/src/robonix_rtk_nav2/scripts"
SH_IN="/work/scripts/rtk-nav2"
CONTAINER_SCRIPTS="/work/scripts/rtk-nav2/container"
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"

# 目标与超时（都可从环境覆盖；超时按"0.16 m/s 的理想速度 ×3"给足，因为重规划会反复
# 取消重发、实际推进远慢于标称速度）
GOAL_A_X="${GOAL_A_X:--2.6}"; GOAL_A_Y="${GOAL_A_Y:-2.6}"
GOAL_B_X="${GOAL_B_X:-2.6}";  GOAL_B_Y="${GOAL_B_Y:-2.6}"
GOAL_S_X=2.6;                 GOAL_S_Y=-2.6
# 临时障碍放在**车会经过、但不压在路线中心线**的位置：车从 B 到 S 基本沿 x≈2.55 走，
# 障碍 0.4 m 见方放在 (3.0, 0.0) → 西表面 2.8，离车中心线 0.25 m（不会撞上），
# 但已经在膨胀半径里、也稳稳落在相机 60° 视场内（横向 0.45 m、纵向 1~3 m）。
# 放正中心线上会让车必须绕行甚至顶住，那种失败会污染"到达 S/N"的判定。
OBST_X="${OBST_X:-3.0}";      OBST_Y="${OBST_Y:-0.0}"
ARRIVE_TIMEOUT="${ARRIVE_TIMEOUT:-300}"
PROBE_TIMEOUT="${PROBE_TIMEOUT:-180}"

# 跑一个容器内的一次性探针：**产物写容器路径**（N4C），stdout 落宿主的 <label>.log。
# 路径换算这条踩过：把宿主路径塞进 cexec 里会指向不存在的目录，而且**静默写不出来**。
# ROS 环境由本函数**统一加**（不要在调用处再写一遍）：rev4-1 那轮我在调用处漏了
# `source setup.bash`，探针全部以 `ModuleNotFoundError: No module named 'rclpy'` 退出、
# 只留下空产物——那种"看起来跑了、其实什么都没测"的失败最难发现。统一加在这儿，
# 调用处就不可能漏。
probe() {
    local label="$1" tmo="$2" cmd="$3" rc
    log "  探针 $(basename "${label}")（最长 ${tmo} s）"
    set +e
    cexec_t "${tmo}" "${NAS} && ${cmd}" > "${label}.log" 2>&1
    rc=$?
    set -e
    [ "${rc}" -eq 0 ] || log "  ** 探针 $(basename "${label}") 退出码 ${rc}，见 ${label}.log **"
    # 本流程里每个探针都把 JSON 写到同名文件（容器路径与宿主路径指向同一份目录），
    # 所以"退出码 0 但产物是空的"能当场发现——这种"看起来跑了、其实什么都没测"的
    # 失败最难发现（rev4-1 那轮探针全是空产物）。
    if [ ! -s "${label}.json" ]; then
        log "  ** 探针 $(basename "${label}") 的 JSON 产物不存在或为空，见 ${label}.log **"
    fi
}

log "===== 第零步：前置检查（独占容器、干净起栈）====="
ensure_container
OTHERS="$(other_start_demo_pids)"
if [ -n "${OTHERS// /}" ]; then
    die "已有另一个 start-demo 在运行（PID: ${OTHERS}）——共享容器不能并发起栈，停下来报告，不要抢。"
fi
PREV_RUN="$(current_run_id || true)"
if stack_running; then
    log "当前有栈在跑（run-id: ${PREV_RUN:-未知}）：判据 1 要求干净起栈，先停掉它"
    scripts/rtk-nav2/stop-demo.sh "${PREV_RUN:-}" 2>&1 | tail -3
    sleep 3
fi
stack_running && die "栈还活着，停不干净；不继续（宁可停下来报告）"
fix_artifacts_ownership --all   # ★ 跨轮／证据类 ⇒ 整棵 ✓（★ 挂载生效的机器上才需要 ✓）
ensure_run_dirs "${RUN_ID}"
mkdir -p "${N4}" "${REC}"
# 清掉同名 run 里上一轮的记录器产物：否则"第一帧"可能是上一轮留下的
rm -f "${REC}"/first_global_map.json "${REC}"/global_first_frame.npz \
      "${REC}"/global_map_timeline.csv "${REC}"/plan_paths.jsonl \
      "${REC}"/rosout.jsonl "${REC}"/nav_states.jsonl "${REC}"/follow_status.jsonl \
      "${REC}"/rates.csv "${REC}"/summary.json "${REC}"/recorder.log
rm -f "${TRAJ}"

log "===== 第一步：起旁路记录器（**必须早于起栈**，否则拿不到判据 1 的第一帧）====="
cexec_bg "${RECC}/recorder.log" \
    "${NAS} && python3 ${CONTAINER_SCRIPTS}/nav4_recorder.py --out-dir ${RECC} --duration 7200"
sleep 4
REC_PID="$(cexec "pgrep -f '[n]av4_recorder.py' | head -1" 2>/dev/null | tr -d '\r' || true)"
[ -n "${REC_PID}" ] || die "记录器没起来，见 ${REC}/recorder.log"
log "记录器 PID=${REC_PID}"

log "===== 第二步：起一个全新的栈（判据 1 的前提）====="
set +e
scripts/rtk-nav2/start-demo.sh "${RUN_ID}" 2>&1 | tail -4
START_RC=${PIPESTATUS[0]}
set -e
[ "${START_RC}" -eq 0 ] || die "起栈失败（退出码 ${START_RC}），见 ${DIR}/logs/"
grep -E '^stack_start_index|^run_kind|^config_fingerprint' "${DIR}/run_state.txt" || true
sleep 5

log "===== 第三步：判据 1 —— 初始无地图 ====="
# (a) 记录器钉住的**第一帧**（起栈前就订上了，所以这是真第一帧）
if [ -f "${REC}/first_global_map.json" ]; then
    cp "${REC}/first_global_map.json" "${N4}/criterion1_first_global_map.json"
    cp "${REC}/global_first_frame.npz" "${N4}/global_first_frame.npz" 2>/dev/null || true
    python3 - "${N4}/criterion1_first_global_map.json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1], encoding='utf-8'))
c = d['counts']
print('  第一帧全局图：unknown=%d/%d（%.2f%%）free=%d occupied=%d lethal=%d all_unknown=%s'
      % (c['unknown'], c['cells'], 100.0 * c['unknown'] / c['cells'],
         c['free'], c['occupied'], c['lethal'], d['all_unknown']))
PY
else
    log "** 记录器没写出第一帧（判据 1 缺产物）**"
fi
# (b) 起栈后立刻再取一次两张图的计数（第二视角）
probe "${N4}/criterion1_map_stats_t0" 90 \
    "python3 ${SCRIPTS_IN}/nav2_probe.py stats --json ${N4C}/criterion1_map_stats_t0.json"
# (c) "没有静态图/没有落盘图"的证据：两条参数链上都不该出现 static_layer/map_server
{
    echo "### 全局图插件表（只应有 obstacle_layer + inflation_layer）"
    grep -n "plugins:" -A2 "${PKG_DIR}/config/nav2_params.yaml" | head -20
    echo
    echo "### nav2_params.yaml 里 static_layer / map_server 的出现次数（期望 0/0）"
    echo -n "static_layer: "; grep -c "static_layer" "${PKG_DIR}/config/nav2_params.yaml" || true
    echo -n "map_server:   "; grep -c "map_server"   "${PKG_DIR}/config/nav2_params.yaml" || true
    echo
    echo "### demo.launch.py 里 map_server / static_layer / yaml_filename 的出现"
    grep -nE "map_server|static_layer|yaml_filename|map_saver" "${PKG_DIR}/launch/demo.launch.py" || echo "（无）"
} > "${N4}/criterion1_no_static_or_saved_map.txt" 2>&1
cat "${N4}/criterion1_no_static_or_saved_map.txt"

sim_now() { python3 "${SCRIPT_DIR}/nav-truth.py" now "${TRAJ}" 2>/dev/null | python3 -c \
    'import json,sys; print(json.load(sys.stdin).get("sim_t"))' 2>/dev/null || echo 0; }

# ---------------------------------------------------------------- 一段：A
log "===== 第四步：判据 2/3 —— 给目标 A(${GOAL_A_X}, ${GOAL_A_Y}) ====="
T_A="$(sim_now)"
log "  发目标前的真值时刻 sim_t=${T_A}"
set +e
scripts/rtk-nav2/nav-to.sh "${RUN_ID}" "${GOAL_A_X}" "${GOAL_A_Y}" 2>&1 | tail -4
set -e
log "  等真值到达 A（容差 0.30 m，最长 ${ARRIVE_TIMEOUT} s）"
set +e
python3 "${SCRIPT_DIR}/nav-truth.py" wait "${TRAJ}" --target "${GOAL_A_X}" "${GOAL_A_Y}" \
    --tol 0.30 --timeout "${ARRIVE_TIMEOUT}" --json "${N4}/leg_A_arrival.json" | tail -12
RC_A=$?
set -e
log "  到达 A 的判定退出码 ${RC_A}（0=到了 1=超时）"
# 到达瞬间的图（**必须此刻采**）
probe "${N4}/global_after_A" 90 \
    "python3 ${CONTAINER_SCRIPTS}/nav4_map_box.py --x 0 --y 0 --label after_A \
     --json ${N4C}/global_after_A.json --snapshot ${N4C}/global_after_A.npz"
probe "${N4}/stats_after_A" 90 \
    "python3 ${SCRIPTS_IN}/nav2_probe.py stats --json ${N4C}/stats_after_A.json"
python3 "${SCRIPT_DIR}/nav-truth.py" leg "${TRAJ}" --target "${GOAL_A_X}" "${GOAL_A_Y}" \
    --tol 0.30 --from-t "${T_A}" --json "${N4}/leg_A_report.json" > "${N4}/leg_A_report.txt" 2>&1 || true
python3 "${SCRIPT_DIR}/nav-truth.py" render "${N4}/global_after_A.npz" --csv "${TRAJ}" \
    --png "${N4}/map_after_A.png" --goal "${GOAL_A_X}" "${GOAL_A_Y}" >/dev/null 2>&1 || true
grep -E '"(reached_within_tol|min_dist_to_target_m|travelled_m|displacement_last_5s_m|crosses_wall_along_truth|min_clearance_to_surface_m)"' \
     "${N4}/leg_A_report.txt" || true

# ---------------------------------------------------------------- 二段：B
log "===== 第五步：判据 5/6 —— 给目标 B(${GOAL_B_X}, ${GOAL_B_Y}) ====="
T_B="$(sim_now)"
log "  发目标前的真值时刻 sim_t=${T_B}"
set +e
scripts/rtk-nav2/nav-to.sh "${RUN_ID}" "${GOAL_B_X}" "${GOAL_B_Y}" 2>&1 | tail -4
set -e
set +e
python3 "${SCRIPT_DIR}/nav-truth.py" wait "${TRAJ}" --target "${GOAL_B_X}" "${GOAL_B_Y}" \
    --tol 0.30 --timeout "${ARRIVE_TIMEOUT}" --json "${N4}/leg_B_arrival.json" | tail -12
RC_B=$?
set -e
log "  到达 B 的判定退出码 ${RC_B}（0=到了 1=超时）"
probe "${N4}/global_after_B" 90 \
    "python3 ${CONTAINER_SCRIPTS}/nav4_map_box.py --x 0 --y 0 --label after_B \
     --json ${N4C}/global_after_B.json --snapshot ${N4C}/global_after_B.npz"
probe "${N4}/stats_after_B" 90 \
    "python3 ${SCRIPTS_IN}/nav2_probe.py stats --json ${N4C}/stats_after_B.json"
python3 "${SCRIPT_DIR}/nav-truth.py" leg "${TRAJ}" --target "${GOAL_B_X}" "${GOAL_B_Y}" \
    --tol 0.30 --from-t "${T_B}" --json "${N4}/leg_B_report.json" > "${N4}/leg_B_report.txt" 2>&1 || true
python3 "${SCRIPT_DIR}/nav-truth.py" render "${N4}/global_after_B.npz" --csv "${TRAJ}" \
    --png "${N4}/map_after_B.png" --goal -2.6 2.6 --goal 2.6 2.6 >/dev/null 2>&1 || true
grep -E '"(reached_within_tol|min_dist_to_target_m|travelled_m|displacement_last_5s_m|crosses_wall_along_truth|min_clearance_to_surface_m)"' \
     "${N4}/leg_B_report.txt" || true

# ---------------------------------------------------------------- 判据 9 第二半：增删障碍后走过
log "===== 第六步：判据 9（第二半）—— 增删障碍后**走过**能否实时更新图 ====="
log "  基线：车在 B，障碍还没放；量 (${OBST_X}, ${OBST_Y}) 的紧盒子（±0.35 m）"
probe "${N4}/box_baseline" 90 \
    "python3 ${CONTAINER_SCRIPTS}/nav4_map_box.py --x ${OBST_X} --y ${OBST_Y} --label baseline \
     --json ${N4C}/box_baseline.json"
python3 - "${N4}/box_baseline.json" <<'PY' || true
import json, sys
d = json.load(open(sys.argv[1], encoding='utf-8'))
print('  baseline tight: lethal=%d inflated=%d free=%d unknown=%d'
      % (d['tight_0p35']['lethal'], d['tight_0p35']['inflated_only'],
         d['tight_0p35']['free'], d['tight_0p35']['unknown']))
PY

log "  向南开（目标 S(${GOAL_S_X}, ${GOAL_S_Y})），并把 0.4 m 见方、高 1.0 m 的临时障碍放到 (${OBST_X}, ${OBST_Y})"
set +e
scripts/rtk-nav2/nav-to.sh "${RUN_ID}" "${GOAL_S_X}" "${GOAL_S_Y}" 2>&1 | tail -3
set -e
cexec_t 60 "${SH_IN}/container/spawn_obstacle.sh spawn ${OBST_X} ${OBST_Y} 0.4 1.0" \
    > "${N4}/spawn_obstacle.log" 2>&1 || log "** 放障碍失败，见 ${N4}/spawn_obstacle.log **"
mark_injection "${RUN_ID}" "判据 9：在 (${OBST_X},${OBST_Y}) 放临时障碍后移走（车两次经过）"
# 在障碍前 1.2 m 处取数（此时障碍正落在相机视场里）
set +e
python3 "${SCRIPT_DIR}/nav-truth.py" wait "${TRAJ}" --target 2.6 1.2 --tol 1.0 \
    --timeout ${ARRIVE_TIMEOUT} --json "${N4}/pass_southbound_arrival.json" >/dev/null 2>&1
set -e
probe "${N4}/box_obstacle_present" 90 \
    "python3 ${CONTAINER_SCRIPTS}/nav4_map_box.py --x ${OBST_X} --y ${OBST_Y} --label obstacle_present \
     --json ${N4C}/box_obstacle_present.json --snapshot ${N4C}/global_obstacle_present.npz"
python3 - "${N4}/box_obstacle_present.json" <<'PY' || true
import json, sys
d = json.load(open(sys.argv[1], encoding='utf-8'))
print('  障碍在位 tight: lethal=%d inflated=%d free=%d unknown=%d'
      % (d['tight_0p35']['lethal'], d['tight_0p35']['inflated_only'],
         d['tight_0p35']['free'], d['tight_0p35']['unknown']))
PY
set +e
python3 "${SCRIPT_DIR}/nav-truth.py" wait "${TRAJ}" --target ${GOAL_S_X} ${GOAL_S_Y} \
    --tol 0.30 --timeout "${ARRIVE_TIMEOUT}" --json "${N4}/leg_S_arrival.json" | tail -8
RC_S=$?
set -e
log "  到达 S 的判定退出码 ${RC_S}"

log "  移走障碍，再向北开回 N(2.6,2.6)：**移走之后要再走过一次**"
cexec_t 60 "${SH_IN}/container/spawn_obstacle.sh remove" > "${N4}/remove_obstacle.log" 2>&1 || true
set +e
scripts/rtk-nav2/nav-to.sh "${RUN_ID}" 2.6 2.6 2>&1 | tail -3
set -e
set +e
python3 "${SCRIPT_DIR}/nav-truth.py" wait "${TRAJ}" --target 2.6 -1.2 --tol 1.0 \
    --timeout ${ARRIVE_TIMEOUT} --json "${N4}/pass_northbound_arrival.json" >/dev/null 2>&1
set -e
probe "${N4}/box_after_removal_passed" 90 \
    "python3 ${CONTAINER_SCRIPTS}/nav4_map_box.py --x ${OBST_X} --y ${OBST_Y} --label after_removal_passed \
     --json ${N4C}/box_after_removal_passed.json --snapshot ${N4C}/global_after_removal.npz"
python3 - "${N4}/box_after_removal_passed.json" <<'PY' || true
import json, sys
d = json.load(open(sys.argv[1], encoding='utf-8'))
print('  移走并走过之后 tight: lethal=%d inflated=%d free=%d unknown=%d'
      % (d['tight_0p35']['lethal'], d['tight_0p35']['inflated_only'],
         d['tight_0p35']['free'], d['tight_0p35']['unknown']))
PY
set +e
python3 "${SCRIPT_DIR}/nav-truth.py" wait "${TRAJ}" --target 2.6 2.6 --tol 0.30 \
    --timeout "${ARRIVE_TIMEOUT}" --json "${N4}/leg_N_arrival.json" | tail -8
RC_N=$?
set -e
python3 "${SCRIPT_DIR}/nav-truth.py" render "${N4}/global_obstacle_present.npz" --csv "${TRAJ}" \
    --png "${N4}/map_obstacle_present.png" --goal "${OBST_X}" "${OBST_Y}" >/dev/null 2>&1 || true
python3 "${SCRIPT_DIR}/nav-truth.py" render "${N4}/global_after_removal.npz" --csv "${TRAJ}" \
    --png "${N4}/map_after_removal.png" --goal "${OBST_X}" "${OBST_Y}" >/dev/null 2>&1 || true

# ---------------------------------------------------------------- 判据 8 / 9
log "===== 第七步：判据 8（控制权唯一）====="
cexec_t 60 "${NAS} && timeout 45 ros2 topic info -v /cmd_vel" > "${N4}/cmd_vel_publishers.txt" 2>&1 || true
PUB_COUNT="$(grep -m1 'Publisher count' "${N4}/cmd_vel_publishers.txt" | awk '{print $3}')"
PUB_NODES="$(grep -A4 'Publisher count' "${N4}/cmd_vel_publishers.txt" | grep 'Node name' \
             | awk '{print $3}' | sort -u | tr '\n' ' ')"
log "  /cmd_vel 发布者数=${PUB_COUNT:-取不到} 节点=${PUB_NODES:-（无）}"

log "===== 第八步：判据 7/9 —— 几何核对（致命格是否落在真实表面）+ 覆盖曲线 ====="
probe "${N4}/geometry_final" "${PROBE_TIMEOUT}" \
    "python3 ${SCRIPTS_IN}/nav2_probe.py geometry --json ${N4C}/geometry_final.json \
     --dump-snapshot ${N4C}/global_final.npz"
log '  全局图另存一份 pgm/yaml（给"图上看效果"留直观产物）'
cexec_t 90 "${NAS} && timeout 60 ros2 run nav2_map_server map_saver_cli \
    -f ${N4C}/global_final -t /global_costmap/costmap --free 0.196 --occ 0.65" \
    > "${N4}/map_saver.log" 2>&1 || true
python3 "${SCRIPT_DIR}/nav-truth.py" render "${N4}/global_final.npz" --csv "${TRAJ}" \
    --png "${N4}/map_final.png" --goal -2.6 2.6 --goal 2.6 2.6 >/dev/null 2>&1 || true

log "===== 第九步：收尾（记录器写 summary，然后汇总）====="
cexec "kill -TERM ${REC_PID} >/dev/null 2>&1 || true"
sleep 6
fix_artifacts_ownership --all   # ★ 跨轮／证据类 ⇒ 整棵 ✓（★ 挂载生效的机器上才需要 ✓）
python3 "${SCRIPT_DIR}/nav4_summarize.py" all "${REC}" --out-dir "${N4}" || true
python3 - "${N4}/summary_plans.json" <<'PY' || true
import json, sys
try:
    d = json.load(open(sys.argv[1], encoding='utf-8'))
except Exception as exc:
    print('  ** 汇总读不到：%s **' % exc); raise SystemExit
f = d.get('first_path_of_leg_A') or {}
b = d.get('first_path_of_leg_B') or {}
print('  判据 2 开局路径(A)：%s 点 / %s m / unknown_frac=%s / 穿墙 %s'
      % (f.get('n_poses'), f.get('length_m'), f.get('unknown_frac'), f.get('crossings_recomputed')))
print('  判据 3 路径是否变化过：%s（共 %s 条被接受的路径）'
      % (d.get('any_path_changed_vs_prev'), d.get('paths_recorded')))
print('  判据 5 给 B 的第一条：%s 点 / %s m / unknown_frac=%s / free_frac=%s / 穿墙 %s'
      % (b.get('n_poses'), b.get('length_m'), b.get('unknown_frac'),
         b.get('free_frac'), b.get('crossings_recomputed')))
print('  判据 7 有穿墙的路径条数：%s（重算总穿墙段 %s，与记录时不一致的条数 %d）'
      % (d.get('crossing_paths'), d.get('crossings_total_recomputed'),
         len(d.get('recheck_mismatch') or [])))
PY

cat <<MSG

证据采集完成（run-id: ${RUN_ID}）
  产物根目录：${DIR}
  判据 1：nav4/criterion1_first_global_map.json + global_first_frame.npz + criterion1_no_static_or_saved_map.txt
  判据 2/3：nav4rec/plan_paths.jsonl + nav4rec/rosout.jsonl → nav4/summary_plans.json、summary_rosout.json
  判据 4：nav4/leg_A_arrival.json + leg_A_report.json + map_after_A.png
  判据 5/6：nav4/leg_B_arrival.json + leg_B_report.json + map_after_B.png
  判据 7：nav4/summary_plans.json（crossing_paths）
  判据 8：nav4/cmd_vel_publishers.txt
  判据 9：nav4/box_{baseline,obstacle_present,after_removal_passed}.json + summary_coverage.json
          + nav4/geometry_final.json + map_{obstacle_present,after_removal,final}.png
  全过程时序：nav4rec/{global_map_timeline.csv,rates.csv,follow_status.jsonl,nav_states.jsonl}
  栈**仍在运行**（不停栈采数，也别在停栈后补采）：停止用 scripts/rtk-nav2/stop-demo.sh ${RUN_ID}
MSG
