#!/usr/bin/env bash
# 异常场景验证：故意制造“激光丢失 / 里程计丢失 / 路线上有障碍 / 总时限过短”，
# 逐条记录命令与巡游节点的真实退出码。
#
# 用法：
#   scripts/start-demo.sh safetyrun     # 先在一个全新的 run-id 上启动演示
#   scripts/test-patrol-behavior.sh safetyrun
#
# 判定标准（与 config/patrol.yaml、README 中的退出码表一致）：
#   障碍场景 -> 5    激光丢失 -> 3    里程计丢失 -> 4    超时 -> 6
# 任何一项与预期不符即视为失败，脚本以非零状态退出。
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
RUN_DIR="${PROJECT_DIR}/artifacts/${RUN_ID}"
OUT_DIR="${RUN_DIR}/safety_tests"
fix_artifacts_ownership
ensure_run_dirs "${RUN_ID}"
mkdir -p "${OUT_DIR}"

ensure_container
demo_stack_running || die "演示栈未运行，请先执行 scripts/start-demo.sh ${RUN_ID}"

PASS=0
FAIL=0
SUMMARY="${OUT_DIR}/summary.txt"
: > "${SUMMARY}"

record() {
    local name="$1" expected="$2" actual="$3" note="$4"
    if [ "${expected}" = "${actual}" ]; then
        printf 'PASS  %-22s 期望退出码=%s 实际=%s  %s\n' "${name}" "${expected}" "${actual}" "${note}" | tee -a "${SUMMARY}"
        PASS=$((PASS + 1))
    else
        printf 'FAIL  %-22s 期望退出码=%s 实际=%s  %s\n' "${name}" "${expected}" "${actual}" "${note}" | tee -a "${SUMMARY}"
        FAIL=$((FAIL + 1))
    fi
}

run_case() {
    local name="$1" expected="$2"; shift 2
    log "场景：${name}（期望退出码 ${expected}）"
    capture_screen "${RUN_DIR}/screenshots/1x_${name}_before.png" || true
    set +e
    cexec "/work/scripts/container/run_patrol.sh '${RUN_ID}' $*" \
        2>&1 | tee "${OUT_DIR}/${name}.log"
    local code=${PIPESTATUS[0]}
    set -e
    echo "${code}" > "${OUT_DIR}/${name}.exit_code"
    capture_screen "${RUN_DIR}/screenshots/1x_${name}_after.png" || true
    record "${name}" "${expected}" "${code}" "日志 ${OUT_DIR}/${name}.log"
}

# ---------------------------------------------------------------- 单元测试
log "运行运动控制行为单元测试"
set +e
cexec "cd /work && python3 -m pytest tests/test_patrol_safety.py -v" \
    2>&1 | tee "${OUT_DIR}/unit_tests.log"
UNIT_CODE=${PIPESTATUS[0]}
set -e
record "unit_tests" "0" "${UNIT_CODE}" "34 条纯逻辑用例（数据缺失/过期、近障、超时、零速、时钟守卫）"

# ------------------------------------------------- 场景 1：路线上出现障碍
# 机器人出生在 odom (0,0)（世界 (-3,-3)），第一段路线沿世界 y=-3 向 +x 行驶。
# 在 odom (3.0, 0.0) 即世界 (0.0, -3.0) 放一个 0.30 m 立柱，位于路线正中间。
log "在路线上放置临时障碍"
cexec "/work/scripts/container/spawn_obstacle.sh spawn 0.0 -3.0" \
    | tee "${OUT_DIR}/spawn_obstacle.log"
sleep 3
run_case "obstacle_on_route" "5"

log "移除临时障碍"
cexec "/work/scripts/container/spawn_obstacle.sh remove" \
    | tee "${OUT_DIR}/remove_obstacle.log"
sleep 3

# ------------------------------------------------------- 场景 2：总时限过短
# 路线剩余里程约 20 m，0.18 m/s 需要约 115 s，把总时限压到 20 s 必然触发超时。
run_case "total_timeout" "6" "-p total_timeout:=20.0"

# --------------------------------------------------------- 场景 3：激光丢失
# 把 /scan 重映射到一个不存在的话题，模拟传感器数据消失（仿真时钟仍在推进）。
run_case "scan_lost" "3" "-r /scan:=/scan_unavailable"

# --------------------------------------------------------- 场景 4：里程计丢失
run_case "odom_lost" "4" "-r /odom:=/odom_unavailable"

# ---------------------------------------------------------------- 汇总
log "异常场景汇总："
cat "${SUMMARY}"
log "通过 ${PASS} 项，失败 ${FAIL} 项"

if [ "${FAIL}" -ne 0 ]; then
    exit 1
fi
