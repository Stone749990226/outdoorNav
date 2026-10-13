#!/usr/bin/env bash
# 第三阶段的**证据采集主流程**：一条命令跑出判据 1/2/3/4/5/6(障碍停车)/7/8 的产物。
#
# 为什么要有这么一个"主流程"脚本：判据之间是有**顺序依赖**的——
#   - 判据 3（覆盖增长）要在**整圈行驶期间**采，不能在静止时采；
#   - 判据 1/4 要在图长出来**之后**才核对（否则图上没东西可核）；
#   - 判据 2（障碍清除）要先放障碍→确认进图→移走→**再经过**；
#   - 判据 7（性能）要放在旋转**之前**（旋转会把图搅动）；
#   - 判据 8（旋转）是最后一个动作，因为它会把机器人和地图都动掉。
# 手工按这个顺序敲很容易漏或串，所以固化成脚本。
#
# 用法：scripts/rtk-nav2/start-demo.sh <run-id> && scripts/rtk-nav2/evidence-run.sh <run-id>
source "$(dirname "${BASH_SOURCE[0]}")/test-common.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
set_paths "${RUN_ID}"
DIR="${HDIR}"
CD="${CDIR}"
LAP_TIMEOUT="${LAP_TIMEOUT:-420}"
ensure_container
stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"
fix_artifacts_ownership --all   # ★ 跨轮／证据类 ⇒ 整棵 ✓（★ 挂载生效的机器上才需要 ✓）
ensure_run_dirs "${RUN_ID}"
mkdir -p "${DIR}/nav2" "${DIR}/coverage" "${DIR}/spin" "${DIR}/behavior"
health_check 0 || die "前置健康检查失败"

OBSTACLE_X=2.6
OBSTACLE_Y=0.0
OBSTACLE_SIDE=0.4

log "===== ① 整圈行驶（期间后台采覆盖增长曲线）====="
# 后台起覆盖探针：它必须在**行驶期间**采，静置段是平的，证明不了"边跑边建"。
cexec_bg "${CD}/coverage/coverage_lap.log" \
    "${NAS} && cd ${SCRIPTS_IN} && python3 nav2_probe.py coverage \
     --csv ${CD}/coverage/coverage_lap.csv --json ${CD}/coverage/coverage_lap.json \
     --duration $((LAP_TIMEOUT + 60))"
sleep 5
set +e
scripts/rtk-nav2/run-route.sh "${RUN_ID}" --timeout "${LAP_TIMEOUT}" 2>&1 | tail -8
LAP_RC=${PIPESTATUS[0]}
set -e
log "整圈退出码 ${LAP_RC}（期望 0）"
cp "${DIR}/route_report.json" "${DIR}/lap1_report.json" 2>/dev/null || true

log "===== ② 判据 5：取产物 + 存读一致性（跑完一圈，图已经长出来了）====="
scripts/rtk-nav2/save-artifacts.sh "${RUN_ID}" >/dev/null 2>&1 || true

log "===== ③ 判据 1：逐格几何核对 + 近墙回归 ====="
run_probe "${RUN_ID}" "nav2/geometry_after_lap.json" 180 \
    "nav2_probe.py geometry --json ${CD}/nav2/geometry_after_lap.json \
     --dump-snapshot ${CD}/nav2/global_after_lap.npz" || true

log "===== ④ 判据 4（一）：规划到路线下一个航点 ====="
# 这里只做"路线下一个航点"这一条。判据 4 的另外两条（未知区域目标、地图更新后路径变化）
# 由 regression-run.sh 里的 plan-checks.sh 做——那两条要求"第一次规划时图上基本还全是未知"，
# 必须在**干净起栈后的第一个动作**里做；本脚本跑到这一步图已经长出来了，做不了。
# 第一版把"未知区域目标"错选成 (0, 12)（在外墙之外，任何路径都必然穿墙），
# 实测报出 12 次穿墙——那是**伪失败**，不是规划器的问题。目标已改到院内未观测点。
cexec_t 200 "${NAS} && python3 ${SCRIPTS_IN}/plan_path.py --goal 2.6 2.6 \
    --json ${CD}/nav2/plan_next_waypoint.json --csv ${CD}/nav2/plan_next_waypoint.csv \
    --label '路线下一个航点（跑完一圈后，图已长出来）'" \
    > "${DIR}/nav2/plan_next_waypoint.txt" 2>&1 || true
grep -E '"planned"|"crosses_wall"|"poses"|"path_length_m"' "${DIR}/nav2/plan_next_waypoint.txt" 2>/dev/null | head -6

log "===== ⑤ 判据 2 + 判据 6（障碍停车）：放障碍 → 确认进图 → 停车 → 移走 → 再经过 ====="
cexec_t 60 "/work/scripts/rtk-nav2/container/spawn_obstacle.sh spawn ${OBSTACLE_X} ${OBSTACLE_Y} ${OBSTACLE_SIDE} 1.0" \
    > "${DIR}/behavior/spawn_obstacle.log" 2>&1 || log "** 放障碍失败 **"
mark_injection "${RUN_ID}" "路线上放障碍 (${OBSTACLE_X},${OBSTACLE_Y}) 后移走"
sleep 8
run_probe "${RUN_ID}" "nav2/obstacle_present_stats.json" 120 \
    "nav2_probe.py stats --json ${CD}/nav2/obstacle_present_stats.json" || true
run_probe "${RUN_ID}" "nav2/geometry_obstacle_present.json" 180 \
    "nav2_probe.py geometry --json ${CD}/nav2/geometry_obstacle_present.json \
     --dump-snapshot ${CD}/nav2/global_obstacle_present.npz" || true

set +e
scripts/rtk-nav2/run-route.sh "${RUN_ID}" --timeout 240 2>&1 | tail -5
OBST_RC=${PIPESTATUS[0]}
set -e
mkdir -p "${DIR}/behavior/obstacle"
cp "${DIR}/route_exit_code.txt" "${DIR}/behavior/obstacle/exit_code.txt" 2>/dev/null || true
cp "${DIR}/route_report.json" "${DIR}/behavior/obstacle/route_report.json" 2>/dev/null || true
log "有障碍时退出码 ${OBST_RC}（期望 5）"
# 实际停车距离（真值轨迹到障碍表面的最小距离）
run_probe "${RUN_ID}" "behavior/obstacle/stop_distance.json" 120 \
    "stop_distance.py ${CD}/trajectory.csv --obstacle ${OBSTACLE_X} ${OBSTACLE_Y} \
     --side ${OBSTACLE_SIDE} --json ${CD}/behavior/obstacle/stop_distance.json" || true
# 障碍确实进了图（不是"停在别的原因上"）
run_probe "${RUN_ID}" "behavior/obstacle/local_with_obstacle.json" 120 \
    "nav2_probe.py localmap --json ${CD}/behavior/obstacle/local_with_obstacle.json \
     --snapshot ${CD}/behavior/obstacle/local_with_obstacle.npz" || true

log "  移走障碍"
cexec_t 60 "/work/scripts/rtk-nav2/container/spawn_obstacle.sh remove" \
    > "${DIR}/behavior/remove_obstacle.log" 2>&1 || true
sleep 10
# 移走后**原地**先看一眼：相机还看得见那个空位置，射线应当把它清成空闲
run_probe "${RUN_ID}" "behavior/obstacle/global_after_removal.json" 180 \
    "nav2_probe.py geometry --json ${CD}/behavior/obstacle/global_after_removal.json \
     --dump-snapshot ${CD}/behavior/obstacle/global_after_removal.npz" || true
# 再**经过**一次：这是判据 2 的原话（"移走障碍后再经过"）
set +e
scripts/rtk-nav2/run-route.sh "${RUN_ID}" --timeout "${LAP_TIMEOUT}" 2>&1 | tail -5
AFTER_RC=${PIPESTATUS[0]}
set -e
log "移走后再跑整圈退出码 ${AFTER_RC}（期望 0）"
cp "${DIR}/route_report.json" "${DIR}/lap2_after_removal_report.json" 2>/dev/null || true
run_probe "${RUN_ID}" "nav2/geometry_after_removal_lap.json" 180 \
    "nav2_probe.py geometry --json ${CD}/nav2/geometry_after_removal_lap.json \
     --dump-snapshot ${CD}/nav2/global_after_removal_lap.npz" || true

log "===== ⑥ 判据 6：定位 RMSE 与横向跟踪（两次整圈都用真值算）====="
cp "${DIR}/trajectory.csv" "${DIR}/trajectory_lap2.csv" 2>/dev/null || true
run_probe "${RUN_ID}" "nav2/trajectory_report_lap2.json" 120 \
    "trajectory_report.py ${CD}/trajectory_lap2.csv --report ${CD}/route_report.json \
     --output ${CD}/nav2/trajectory_report_lap2.json" || true

log "===== ⑦ 判据 7：性能（旋转之前）====="
run_probe "${RUN_ID}" "perf/perf.json" 240 \
    "nav2_probe.py perf --json ${CD}/perf/perf.json --duration 60" || true

log "===== ⑧ 判据 8：局部图旋转帧（原地转一圈，最后一个动作）====="
mark_injection "${RUN_ID}" "原地旋转（判据 8）"
kill_pattern "[r]oute_follower" >/dev/null 2>&1 || true
sleep 2
run_probe "${RUN_ID}" "spin/spin_summary.json" 300 \
    "spin_probe.py --out-dir ${CD}/spin --rate 0.6 --turns 1 --half-angle 30" || true

log "===== ⑨ 收尾：控制权归属 + 参数快照 ====="
cexec_t 40 "${NAS} && timeout 25 ros2 topic info -v /cmd_vel" \
    > "${DIR}/nav2/cmd_vel_publishers_final.txt" 2>&1 || true
PUB="$(grep -A3 'Publisher count' "${DIR}/nav2/cmd_vel_publishers_final.txt" 2>/dev/null \
       | grep 'Node name' | awk '{print $3}' | sort -u | tr '\n' ' ')"
log "  /cmd_vel 发布者：${PUB:-（无）}"

cat <<MSG

证据采集完成（${DIR}）：
  整圈：lap1_report.json（${LAP_RC}）、lap2_after_removal_report.json（${AFTER_RC}）
  障碍：behavior/obstacle/（退出码 ${OBST_RC}、停止距离、放障碍前后两张图快照）
  几何：nav2/geometry_{after_lap,obstacle_present,after_removal_lap}.json + 对应 npz
  规划：nav2/plan_{next_waypoint,unknown_goal}.{json,csv,txt}
  存读：nav2/{map_saver.log,global_map.yaml,global_map.pgm,map_roundtrip.json,save_load_check.json}
  性能：perf/perf.json
  旋转：spin/spin_summary.json + spin_samples.csv
MSG
