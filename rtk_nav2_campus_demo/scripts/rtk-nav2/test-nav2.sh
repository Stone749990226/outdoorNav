#!/usr/bin/env bash
# 第三阶段 3.1 的判据套件：地图正确性 / 清除语义 / 边跑边建 / 性能 / 局部图旋转帧。
#
# 判据 1 地图正确性（逐格几何核对 + 近墙回归）
# 判据 2 清除语义（障碍移走后被清掉；未观测的墙保留）
# 判据 3 边跑边建（覆盖增长曲线 + 没去过的区域仍 unknown）
# 判据 7 性能（两张图/点云的发布频率、容器 CPU、宿主 load）
# 判据 8 局部图挂 base_link 的旋转帧实测（原地转一圈）
#
# 用法：scripts/rtk-nav2/test-nav2.sh <run-id> [--skip-spin]
#
# **这个套件会开车**（判据 8 要原地旋转），所以跑之前确认没人在看这台机器。
source "$(dirname "${BASH_SOURCE[0]}")/test-common.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
SKIP_SPIN=0
[ "${2:-}" = "--skip-spin" ] && SKIP_SPIN=1
set_paths "${RUN_ID}"
DIR="${HDIR}"
CD="${CDIR}"
ensure_container
stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"
fix_artifacts_ownership --all   # ★ 跨轮／证据类 ⇒ 整棵 ✓（★ 挂载生效的机器上才需要 ✓）
ensure_run_dirs "${RUN_ID}"

# 旋转是"注入类"动作（会给机器人发指令，地图会长出东西），必须留痕
if [ "${SKIP_SPIN}" -eq 0 ]; then
    mark_injection "${RUN_ID}" "原地旋转（判据 8：局部图旋转帧实测）"
fi

log "===== 判据 7：性能（旋转之前先量，避免旋转把图搅动后再量）====="
run_probe "${RUN_ID}" "perf/perf.json" 180 "nav2_probe.py perf --json ${CD}/perf/perf.json --duration 60" || true

log "===== 判据 3：覆盖增长曲线（静置观察 60 s，图应当基本稳定）====="
run_probe "${RUN_ID}" "coverage/coverage_idle.csv" 180 \
    "nav2_probe.py coverage --csv ${CD}/coverage/coverage_idle.csv --duration 60 --json ${CD}/coverage/coverage_idle.json" || true

if [ "${SKIP_SPIN}" -eq 0 ]; then
    log "===== 判据 8：局部图旋转帧实测（原地转 1 圈，0.6 rad/s）====="
    # 先把 route_follower 停掉：它也在发 /cmd_vel，两个发布者会打架
    kill_pattern "[r]oute_follower" >/dev/null 2>&1 || true
    sleep 2
    run_probe "${RUN_ID}" "spin/spin_summary.json" 300 \
        "spin_probe.py --out-dir ${CD}/spin --rate 0.6 --turns 1 --half-angle 30" || true
    log "旋转后先看有没有把图搅乱（几何核对会用到，放在最后）"
fi

log "===== 判据 1：地图正确性（逐格几何核对 + 近墙回归）====="
run_probe "${RUN_ID}" "nav2/geometry_final.json" 180 \
    "nav2_probe.py geometry --json ${CD}/nav2/geometry_final.json --dump-snapshot ${CD}/nav2/global_snapshot_final.npz" || true

log "===== 判据 3：两张图的元数据与计数 ====="
run_probe "${RUN_ID}" "nav2/map_stats_final.json" 120 \
    "nav2_probe.py stats --json ${CD}/nav2/map_stats_final.json" || true

log "===== 控制权归属（裁定 2 的可核查判据）====="
cexec_t 40 "${NAS} && timeout 25 ros2 topic info -v /cmd_vel" \
    > "${DIR}/nav2/cmd_vel_publishers_final.txt" 2>&1 || true
PUB_NODES="$(grep -A3 'Publisher count' "${DIR}/nav2/cmd_vel_publishers_final.txt" 2>/dev/null | grep 'Node name' | awk '{print $3}' | sort -u | tr '\n' ' ')"
log "  /cmd_vel 的发布者节点：${PUB_NODES:-（无）}"

log "套件完成。产物在 ${DIR}：perf/ coverage/ spin/ nav2/"
cat <<MSG

判读顺序建议：
  1. ${DIR}/nav2/geometry_final.json      —— 占据格到真实表面的距离分布 + 每面墙的三分类
                                              + route_near（路线 2 m 内的墙）的占据比例
  2. ${DIR}/coverage/coverage_idle.csv    —— 覆盖增长曲线（静置段应当基本不涨）
  3. ${DIR}/spin/spin_summary.json        —— 转一圈后"新长出来的幽灵格"数量
  4. ${DIR}/perf/perf.json                —— 发布频率 / 容器 CPU / 宿主 load
  5. ${DIR}/nav2/cmd_vel_publishers_final.txt —— 发布者必须只有 route_follower
MSG
