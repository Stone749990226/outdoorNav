#!/usr/bin/env bash
# 第三阶段的"不回归"套件（判据 6）：第一阶段已通过的五项至少各复测一次。
#
#   1. 基准跑                —— 正常一圈（定位 RMSE、横向跟踪）
#   2. 里程计劣化跑          —— 运行中缩放+加噪，且**证明劣化确实生效**
#   3. 路线上放障碍          —— 停车（退出码 5），记录**实际停车距离**
#   4. 断掉相机数据          —— 停车（退出码 3）
#
# 三条硬约束（都是被上一轮的失败教出来的，原样保留）：
#   - **每一步都带超时**：无超时的命令在这台机器上会无限等待（第二阶段实测挂过 19 分钟）。
#   - **开跑前先做健康检查**：核心节点缺一个就立刻非零退出。
#   - **障碍必须落在路线上**：脚本自己断言，路线改了而障碍坐标没跟着改就直接失败。
#
# 用法：scripts/rtk-nav2/start-demo.sh <run-id> && scripts/rtk-nav2/test-behavior.sh <run-id>
source "$(dirname "${BASH_SOURCE[0]}")/test-common.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
set_paths "${RUN_ID}"
DIR="${HDIR}"
CD="${CDIR}"
ensure_container
stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"
fix_artifacts_ownership --all   # ★ 跨轮／证据类 ⇒ 整棵 ✓（★ 挂载生效的机器上才需要 ✓）
ensure_run_dirs "${RUN_ID}"
OUT_DIR="${DIR}/behavior"
mkdir -p "${OUT_DIR}"

# 障碍放在**第一段航线**的中点：机器人从 (2.6,-2.6) 出发，第一个目标是 (2.6,2.6)，
# 沿 x=2.6 向北走，会在 (2.6, 0.0) 遇到它。**必须落在路线上**——第二阶段有一版
# 障碍坐标是旧路线上的点，脚本"跑完了"却什么都没测到。
OBSTACLE_X=2.6
OBSTACLE_Y=0.0
OBSTACLE_SIDE=0.4

# 路线是方环 ±2.6（config/route.yaml）。断言障碍点确实在路线上（到边的距离 < 5 cm）
python3 - "$OBSTACLE_X" "$OBSTACLE_Y" <<'PY' || die "障碍点不在路线上，请同步更新 OBSTACLE_X/Y 与 route.yaml"
import sys
x, y = float(sys.argv[1]), float(sys.argv[2])
s = 2.6
d = min(abs(abs(x) - s) if abs(y) <= s else 9e9,
        abs(abs(y) - s) if abs(x) <= s else 9e9)
if d > 0.05:
    print('障碍 (%g,%g) 到路线最近距离 %.3f m > 0.05 m' % (x, y, d), file=sys.stderr)
    raise SystemExit(1)
print('障碍 (%g,%g) 在路线上（距离 %.4f m）' % (x, y, d))
PY

health_check 0 || die "前置健康检查失败"

# ---------------------------------------------------------------- 1) 基准跑
log "===== 1) 基准跑（干净）====="
mark_injection "${RUN_ID}" "行为套件：基准/劣化/障碍/断相机 四个场景"
BASE_OUT="${OUT_DIR}/baseline"
mkdir -p "${BASE_OUT}"
cexec_t 60 "${NAS} && timeout 40 ros2 topic info -v /cmd_vel" \
    > "${OUT_DIR}/cmd_vel_before_baseline.txt" 2>&1 || true

# 基准跑期间也采一份里程计偏离做**对照**（未劣化时应接近 0）
cexec_bg "${CD}/behavior/odom_deviation_baseline.log" \
    "${NAS} && cd ${SCRIPTS_IN} && python3 odom_deviation_probe.py --duration 300 \
     --json ${CD}/behavior/odom_deviation_baseline.json"
sleep 5
set +e
scripts/rtk-nav2/run-route.sh "${RUN_ID}" --timeout 420 > "${BASE_OUT}/run.log" 2>&1
BASE_RC=$?
set -e
cp "${DIR}/route_exit_code.txt" "${BASE_OUT}/exit_code.txt" 2>/dev/null || true
cp "${DIR}/route_report.json" "${BASE_OUT}/route_report.json" 2>/dev/null || true
log "基准跑退出码 ${BASE_RC}"
if [ "${BASE_RC}" -ne 0 ]; then
    log "** 基准跑不是 0（${BASE_RC}）：后面场景的结果不能当成基准对照用 **"
fi

# 基准跑跑完，全局图应当长出来了：取一次快照 + 几何核对 + 覆盖曲线
run_probe "${RUN_ID}" "behavior/after_lap_geometry.json" 180 \
    "nav2_probe.py geometry --json ${CD}/behavior/after_lap_geometry.json --dump-snapshot ${CD}/behavior/global_after_lap.npz" || true

# ---------------------------------------------------------------- 2) 里程计劣化
# **位置**：放在放障碍/断相机**之前**。理由是断相机是破坏性的（把感知链路打断且不修复），
# 必须留在最后；劣化只改参数、跑完就恢复，所以放在它前面。
# **必须先证明劣化确实生效**——第二阶段有一版参数根本没设进去
# （SetParametersResult 类型写错，节点一设参数就崩），"基准 vs 劣化"两次跑的是同一个配置，
# 结论是空的。所以这里：设参数 -> 读回确认 -> 再量"里程计与真值的偏离"。
log "===== 2) 里程计劣化跑 ====="
DEG_OUT="${OUT_DIR}/degraded"
mkdir -p "${DEG_OUT}"

PARAM_DIR="${SCRIPT_DIR}/../../rtk_nav2_ws/src/robonix_rtk_nav2/scripts"

set +e
{
    cexec_t 40 "${NAS} && timeout 20 python3 ${SCRIPTS_IN}/param_tool.py get /wheel_odom_bridge degrade"
    cexec_t 40 "${NAS} && timeout 20 python3 ${SCRIPTS_IN}/param_tool.py get /wheel_odom_bridge scale"
} > "${OUT_DIR}/params_before_degrade.txt" 2>&1

{
    cexec_t 40 "${NAS} && timeout 20 python3 ${SCRIPTS_IN}/param_tool.py set /wheel_odom_bridge degrade true"
    cexec_t 40 "${NAS} && timeout 20 python3 ${SCRIPTS_IN}/param_tool.py set /wheel_odom_bridge scale 1.35"
    cexec_t 40 "${NAS} && timeout 20 python3 ${SCRIPTS_IN}/param_tool.py set /wheel_odom_bridge noise_stddev_m 0.02"
    cexec_t 40 "${NAS} && timeout 20 python3 ${SCRIPTS_IN}/param_tool.py get /wheel_odom_bridge degrade"
    cexec_t 40 "${NAS} && timeout 20 python3 ${SCRIPTS_IN}/param_tool.py get /wheel_odom_bridge scale"
} > "${OUT_DIR}/params_degrade_set.txt" 2>&1
set -e
log "劣化参数设置与读回见 params_degrade_set.txt（必须能看到 degrade=true、scale=1.35）"
cat "${OUT_DIR}/params_degrade_set.txt"

# 劣化生效的**行为证据**（比读参数更强）：/wheel_odom_used 与真值的偏离。
# **必须在行驶中采**：上一版是跑完基准、机器人静止时采的，`truth_displacement_m` 正好 0.0
# ——自己就说明测的是"没动过"，那种数据证明不了劣化生效（reviewer 指出了这一点）。
# 现在把探针**后台起在整圈行驶期间**，跑完再取结果。
cexec_bg "${CD}/behavior/odom_deviation_degraded.log" \
    "${NAS} && cd ${SCRIPTS_IN} && python3 odom_deviation_probe.py --duration 300 \
     --json ${CD}/behavior/odom_deviation.json"
sleep 5
set +e
scripts/rtk-nav2/run-route.sh "${RUN_ID}" --timeout 420 > "${DEG_OUT}/run.log" 2>&1
DEG_RC=$?
set -e
cp "${DIR}/route_exit_code.txt" "${DEG_OUT}/exit_code.txt" 2>/dev/null || true
cp "${DIR}/route_report.json" "${DEG_OUT}/route_report.json" 2>/dev/null || true
run_probe "${RUN_ID}" "behavior/degraded/trajectory_report.json" 120 \
    "trajectory_report.py ${CD}/trajectory.csv --report ${CD}/route_report.json --output ${CD}/behavior/degraded/trajectory_report.json" || true
log "劣化跑退出码 ${DEG_RC}（与基准 ${BASE_RC} 对照）"

# 恢复
set +e
cexec_t 40 "${NAS} && timeout 20 python3 ${SCRIPTS_IN}/param_tool.py set /wheel_odom_bridge degrade false" \
    > "${OUT_DIR}/param_restore.log" 2>&1
set -e

# ---------------------------------------------------------------- 3) 路线上放障碍
log "===== 3) 在路线上放障碍 (${OBSTACLE_X}, ${OBSTACLE_Y}) ====="
# 说明：先放障碍再重启路线跟随。栈本身不动（代价地图要保留，才能看它进不进图）。
cexec_t 60 "/work/scripts/rtk-nav2/container/spawn_obstacle.sh spawn ${OBSTACLE_X} ${OBSTACLE_Y} ${OBSTACLE_SIDE} 1.0" \
    > "${OUT_DIR}/spawn_obstacle.log" 2>&1 || log "** 放障碍失败，见 spawn_obstacle.log **"
mark_injection "${RUN_ID}" "路线上放障碍 (${OBSTACLE_X},${OBSTACLE_Y})"
sleep 8

# 障碍进图了吗：取一次局部图，看前方 3 m 内有没有占据格
run_probe "${RUN_ID}" "behavior/obstacle_in_map.json" 120 \
    "nav2_probe.py localmap --json ${CD}/behavior/obstacle_in_map.json" || true

OBST_OUT="${OUT_DIR}/obstacle"
mkdir -p "${OBST_OUT}"
cexec_t 40 "${NAS} && timeout 25 python3 ${SCRIPTS_IN}/param_tool.py get /danger danger_cost_threshold" \
    > "${OUT_DIR}/danger_threshold.txt" 2>&1 || true

set +e
scripts/rtk-nav2/run-route.sh "${RUN_ID}" --timeout 200 > "${OBST_OUT}/run.log" 2>&1
OBST_RC=$?
set -e
cp "${DIR}/route_exit_code.txt" "${OBST_OUT}/exit_code.txt" 2>/dev/null || true
cp "${DIR}/route_report.json" "${OBST_OUT}/route_report.json" 2>/dev/null || true
log "有障碍时退出码 ${OBST_RC}（期望 5 = 前方障碍停车）"

# 实际停车距离：用真值轨迹算"整段里到障碍表面的最小距离"（最近到了多远）。
# 不用 trajectory_report.py 的字段——它给的是定位/横向误差，没有末位姿；
# 直接从 trajectory.csv 的 gt_x/gt_y 算，口径最硬。
run_probe "${RUN_ID}" "behavior/obstacle/stop_distance.txt" 120 \
    "stop_distance.py ${CD}/trajectory.csv --obstacle ${OBSTACLE_X} ${OBSTACLE_Y} --side ${OBSTACLE_SIDE} --json ${CD}/behavior/obstacle/stop_distance.json" || true

# 顺带出一份定位/横向误差（判据 6 的另外两项）
run_probe "${RUN_ID}" "behavior/obstacle/trajectory_report.json" 120 \
    "trajectory_report.py ${CD}/trajectory.csv --report ${CD}/route_report.json --output ${CD}/behavior/obstacle/trajectory_report.json" || true

log "移除障碍"
cexec_t 60 "/work/scripts/rtk-nav2/container/spawn_obstacle.sh remove" \
    > "${OUT_DIR}/remove_obstacle.log" 2>&1 || true
sleep 8

# ---------------------------------------------------------------- 4) 断掉相机
log "===== 4) 断掉相机桥接 ====="
mark_injection "${RUN_ID}" "断掉相机桥接（ros_gz_bridge_rtk_camera）"
CAM_PIDS="$(kill_pattern "[r]os_gz_bridge_rtk_camera")"
log "  被杀掉的相机桥接 PID：${CAM_PIDS:-（无）}"
{
    echo "killed_pids=${CAM_PIDS:-none}"
    echo "time=$(date '+%H:%M:%S')"
} > "${OUT_DIR}/camera_kill.txt"
sleep 3
CAMP_OUT="${OUT_DIR}/camera_lost"
mkdir -p "${CAMP_OUT}"
set +e
scripts/rtk-nav2/run-route.sh "${RUN_ID}" --timeout 180 > "${CAMP_OUT}/run.log" 2>&1
CAMP_RC=$?
set -e
cp "${DIR}/route_exit_code.txt" "${CAMP_OUT}/exit_code.txt" 2>/dev/null || true
cp "${DIR}/route_report.json" "${CAMP_OUT}/route_report.json" 2>/dev/null || true
log "断相机后退出码 ${CAMP_RC}（期望 3 = 相机/代价地图失效）"

# 恢复
set +e
cexec_t 40 "${NAS} && timeout 20 python3 ${SCRIPTS_IN}/param_tool.py set /wheel_odom_bridge degrade false" \
    > "${OUT_DIR}/param_restore.log" 2>&1
set -e

cat <<MSG

套件完成（产物在 ${OUT_DIR}）：
  baseline/    基准跑：退出码、route_report.json
  obstacle/    障碍停车：退出码、停车距离、障碍是否进图
  camera_lost/ 断相机：退出码
  degraded/    里程计劣化：退出码（与基准对照）、劣化是否生效的证据
  after_lap_geometry.json / global_after_lap.npz  跑完一圈后的地图快照与几何核对

退出码对照（第一阶段的约定，原样沿用）：
  0 正常跑完 / 3 相机或代价地图失效 / 5 前方障碍停车 / 6 超时 / 7 定位失效
MSG
