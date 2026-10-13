#!/usr/bin/env bash
# 判据 2 复测矩阵（brief §11 返工清单第 1 条）。
#
# **注意 `set -e`**：lib.sh 开了 `set -euo pipefail`，而这里的探针/cex 调用**合法地**会返回非零
# （探针等不到话题返回 1、`ign model --list | grep -c` 找不到时返回 1）。
# 不加 `|| true` 的话脚本会**在第一处静默终止**——日志停在 "[A] 基线"、没有任何报错，
# 我因为这个白跑了两轮（第一次以为在跑，其实 4 分钟前就死了）。下面所有探测调用都带 `|| true`。
#
# 每条要**三份证据同时给**：
#   (i)   移走后该处致命格**逐格变回 free**（局部图快照 A/B 逐格 diff）
#   (ii)  同时给出"传感器已看不到它"的点云证据（该距离段 z>=阈值 的点数归零）
#   (iii) 世界侧确认 model 已移除（ign model --list）
# 并且：某一条之后**再跑一圈退出码 0**（改之前会以退出码 5 停在同一处）。
#
# 复用的是 reviewer 复验时写的探针（`artifacts/rtk-nav2/rev_probe_local.py`、
# `rev_summarize_local.py`）——不重写一套，免得两边口径不同。
#
# 用法：scripts/rtk-nav2/clear-matrix.sh <run-id> [case ...]
#   case ∈ front_tall front_short left_tall right_tall behind_tall（默认前两个 + 整圈）
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"; shift || true
CASES=("$@")
[ "${#CASES[@]}" -eq 0 ] && CASES=(front_tall front_short)
set_paths_run="${RUN_ID}"
OUT="$(artifact_dir "${RUN_ID}")/clearing"
CDIR="$(artifact_dir_c "${RUN_ID}")/clearing"
mkdir -p "${OUT}"
C="${CONTAINER_NAME}"
REV_DIR="${CONTAINER_WORKDIR}/artifacts/rtk-nav2"          # reviewer 的探针在这
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"

# ★★ 2026-10-10（reviewer ①′）：`cex` 原来是**直连** `docker exec`（无分区）⇒ **分区盲** ✗
#   ⇒ `nobs()` 的 `ign model --list` 数的是**默认分区**里那套，**不是本轮那一套** ✗
#   ⇒ "动作到底成没成"会被静默读成 0 ✗ —— 与 `wait_ready.py` 被分区骗死**同族** ✓
#     （`spawn_obstacle.sh` 文件头记的那条："`ign service` 返回成功也可能什么都没做" ✓）。
#   ★ 修法照 `spawn_obstacle.sh`：**把分区注入下移到最底层 helper** ✓
#     ⇒ 本文件所有 `cex`/`ros`/`nobs` 调用（含 line 125/129/139）**一次全覆盖** ✓。
#   ★ 对 ROS 侧命令（`ros`）**无害** ✓：`ros2 topic info` 走 ROS 图（`ROS_DOMAIN_ID`），
#     不看 `GZ_PARTITION` ✓；而**没有任何** `ros2` 子命令会因多这两个环境变量改变行为 ✓。
#   ★ 用 `docker exec -e`（而非把 `VAR=val` 拼进命令串）⇒ **不受引号/分词影响** ✓。
if [ -n "${RUN_ID}" ]; then
    _GZP="rtknav2_${RUN_ID}"
    cex() { docker exec -e GZ_PARTITION="${_GZP}" -e IGN_PARTITION="${_GZP}" "${C}" bash -lc "$*"; }
else
    cex() { docker exec "${C}" bash -lc "$*"; }
fi
ros() { cex "${NAS} && $*"; }
nobs() { cex "ign model --list 2>/dev/null | grep -c test_obstacle" | tr -d '\r' | tail -1; }

# 局部图快照（逐格用）。用 nav2_probe 而不是 reviewer 的探针：我要 npz 做逐格 diff。
snap() { ros "cd ${WS_DIR}/src/robonix_rtk_nav2/scripts && timeout 90 python3 nav2_probe.py localmap \
         --snapshot ${CDIR}/local_$1.npz --json ${CDIR}/local_$1.json" >/dev/null 2>&1 || true; }

# 点云证据：在"障碍应在的那个盒子里"统计 z>=0.12 的点数（0.12 = min_obstacle_height）
# 盒子按**base_link**给：x∈[1.2,1.8]（约 1.5 m 处）、|y|<=0.35。
cloud_ev() {
    # 用 reviewer 的探针取点云证据。**注意 label 的用法**：它会存成
    # `cloud_<label>.npz`，所以 label 直接给场景名（我第一版写成 cloud_$1 又去找
    # cloud_$1.npz，会永远找不到文件——而重定向到 /dev/null 让这个错静默了）。
    ros "cd ${REV_DIR} && timeout 90 python3 rev_probe_local.py --out ${CDIR} \
         --label $1 --secs 5" >/dev/null 2>&1 || true
    if [ -f "${OUT}/probe_$1.json" ]; then
        python3 - "${OUT}/probe_$1.json" "$1" >> "${OUT}/cloud_evidence.txt" <<'PY'
import json, sys
try:
    d = json.load(open(sys.argv[1], encoding='utf-8'))
except Exception:
    raise SystemExit
prof = d.get('cloud_base_link_z_profile')
print('%-22s base_link_z_profile=%s' % (sys.argv[2], json.dumps(prof, ensure_ascii=False) if prof else '(无)'))
PY
    fi
    if [ -f "${OUT}/cloud_$1.npz" ]; then
        python3 - "$OUT/cloud_$1.npz" "$1" >> "${OUT}/cloud_evidence.txt" <<'PY'
import sys
import numpy as np
z = np.load(sys.argv[1])['p']
m = (z[:, 0] > 1.2) & (z[:, 0] < 1.8) & (np.abs(z[:, 1]) < 0.35)
n = int(m.sum())
hi = int((z[m, 2] >= 0.12).sum()) if n else 0
print('%-22s points_in_box=%4d  z>=0.12_pts=%4d  z_min=%s' % (
    sys.argv[2], n, hi, (round(float(z[m, 2].min()), 3) if n else None)))
PY
    fi
}

# 机器人原地转到某个相对航向（弧度），用于"左侧/右侧/后方"三条
turn_to() {
    local yaw="$1"
    ros "timeout 40 python3 - <<PY
import math, time, rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
rclpy.init(); n = Node('turn_probe')
n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
pub = n.create_publisher(Twist, '/cmd_vel', 10)
target = float('${yaw}'); t0 = time.time()
# 开环按时间转：0.5 rad/s，转 target 需要 2*target 秒（含加速段留余量）
tw = Twist(); tw.angular.z = 0.5 if target > 0 else -0.5
dur = abs(target) / 0.5
while time.time() - t0 < dur:
    pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.02)
    time.sleep(0.02)
pub.publish(Twist())
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.05)
rclpy.shutdown()
PY" >/dev/null 2>&1 || true
    sleep 3
}

# 一个场景：把障碍放在相对机器人航向 bearing、距离 1.5 m 处
run_case() {
    local name="$1" bearing="$2" side="$3" height="$4"
    log "===== 场景 ${name}：bearing=${bearing} 侧${side} 高${height} ====="
    [ "${bearing}" != "0" ] && turn_to "${bearing}"
    sleep 2
    # base_link 前向 1.5 m 处的世界坐标
    read -r OX OY <<<"$(ros "timeout 30 python3 - <<PY
import math, rclpy, tf2_ros
rclpy.init(); n = rclpy.node.Node('xy')
n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
b = tf2_ros.Buffer(); tf2_ros.TransformListener(b, n)
import time; t0 = time.time(); p = None
while time.time() - t0 < 15 and p is None:
    rclpy.spin_once(n, timeout_sec=0.1)
    try: p = b.lookup_transform('map', 'base_footprint', rclpy.time.Time())
    except Exception: pass
x = p.transform.translation.x + 1.5 * math.cos(float('${bearing}'))
y = p.transform.translation.y + 1.5 * math.sin(float('${bearing}'))
print('%.3f %.3f' % (x, y))
rclpy.shutdown()
PY" 2>/dev/null | tail -1)"
    log "  障碍世界坐标：(${OX}, ${OY})"
    : > "${OUT}/cloud_evidence.txt"

    log "  [A] 基线（无障碍）"
    snap "${name}_A"; cloud_ev "${name}_A"
    cex "ign model --list 2>/dev/null | grep -c test_obstacle" >/dev/null 2>&1 || true

    log "  [B] 放障碍 0.4×0.4×${height}"
    ros "/work/scripts/rtk-nav2/container/spawn_obstacle.sh spawn ${OX} ${OY} 0.4 ${height}" 2>&1 | tail -1 || true
    echo "world_model_count_B=$(nobs)" >> "${OUT}/${name}.txt"
    sleep 14
    snap "${name}_B"; cloud_ev "${name}_B"

    log "  [C] 移走障碍 → **先转开、再转回同一朝向**（brief §11 的测法要求）"
    # 为什么必须"转开再转回"：射线清除靠的是**新射线穿过该格**。移走障碍之后
    # 如果机器人再也没朝那个方向看，那些格**本来就永远没有机会被清**——那不是缺陷，
    # 是"清除的前提是再次观测到那一带"这个语义。所以这一步既是复现，也是判据的一部分：
    # 只有"重新进视场之后格变 free"才说明清除通路真的工作。
    ros "/work/scripts/rtk-nav2/container/spawn_obstacle.sh remove" >/dev/null 2>&1
    echo "world_model_count_C=$(nobs)" >> "${OUT}/${name}.txt"
    if [ "${bearing}" != "0" ] || true; then
        log "    转开 +0.9 rad（离开那一带）"
        turn_to "0.9"
        sleep 4
        log "    转回原朝向 ${bearing}（让它重新进视场）"
        turn_to "$(python3 -c "print(float('${bearing}') - 0.9)")"
        sleep 8
    fi
    {
        echo "reobserve_plan=bearing=${bearing} -> turn_away(+0.9 rad) -> turn_back(${bearing}) -> probe C"
        echo "reobserve_note=清除的前提是再次观测到那一带；本场景在 probe C 之前显式做了转开+转回"
    } >> "${OUT}/${name}.txt"
    snap "${name}_C"; cloud_ev "${name}_C"

    # 逐格 diff：只看障碍那一点及其邻域
    python3 "$(dirname "${BASH_SOURCE[0]}")/diff_snapshots.py" \
        "${OUT}/local_${name}_B.npz" "${OUT}/local_${name}_C.npz" \
        --at 1.5 0.0 --at 1.5 0.2 --at 1.5 -0.2 --radius-cells 6 \
        > "${OUT}/${name}_diff.json" 2>&1
    python3 - "${OUT}/${name}_diff.json" "${name}" >> "${OUT}/${name}.txt" <<'PY'
import json, sys
try:
    d = json.load(open(sys.argv[1], encoding='utf-8'))
except Exception as exc:
    print('%s diff 读不到：%s' % (sys.argv[2], exc)); raise SystemExit
g = d.get('global', {})
print('%s: lethal B=%s C=%s  cleared_in_C=%s  新增=%s' % (
    sys.argv[2], g.get('lethal_b'), g.get('lethal_c') or g.get('lethal_b'),
    g.get('lethal_cleared_in_b'), g.get('lethal_new_in_b')))
for p in d.get('points', []):
    print('   at %s  B=%s(%s) -> C=%s(%s)  cleared=%s' % (
        p.get('at'), p.get('a_value'), p.get('a_class'),
        p.get('b_value'), p.get('b_class'), p.get('cleared')))
PY

    if [ "${bearing}" != "0" ]; then turn_to "$(python3 -c "print(-float('${bearing}'))")"; fi
    sleep 2
}

stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"
fix_artifacts_ownership --all   # ★ 跨轮／证据类 ⇒ 整棵 ✓（★ 挂载生效的机器上才需要 ✓）
log "判据 2 复测矩阵（run-id: ${RUN_ID}），场景：${CASES[*]}"

for c in "${CASES[@]}"; do
    case "${c}" in
        front_tall)   run_case front_tall   0        0.4 1.0 ;;
        front_short)  run_case front_short  0        0.4 0.25 ;;
        left_tall)    run_case left_tall    1.5708   0.4 1.0 ;;
        right_tall)   run_case right_tall  -1.5708   0.4 1.0 ;;
        behind_tall)  run_case behind_tall  3.1416   0.4 1.0 ;;
        *) log "未知场景 ${c}" ;;
    esac
done

log "===== 移走后再跑一圈（通过标准：退出码 0）====="
set +e
scripts/rtk-nav2/run-route.sh "${RUN_ID}" --timeout 420 2>&1 | tail -6
LAP_RC=${PIPESTATUS[0]}
set -e
echo "lap_after_removal_exit=${LAP_RC}" >> "${OUT}/lap_after_removal.txt"
cp "$(artifact_dir "${RUN_ID}")/route_report.json" "${OUT}/lap_after_removal_report.json" 2>/dev/null || true
log "移走后再跑一圈退出码 ${LAP_RC}（通过标准 0）"

log "产物：${OUT}/（每场景 {A,B,C} 快照 + diff.json + .txt；cloud_evidence.txt；lap_after_removal*）"
for f in "${OUT}"/*.txt; do echo "----- $(basename "$f") -----"; cat "$f"; done
