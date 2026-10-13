#!/usr/bin/env bash
# 第三阶段的**回归与核对主流程**（在另一台/另一次干净起栈上跑，与 evidence-run 分开）。
#
# 为什么分开：evidence-run 采的是"建图与规划"的判据（1/2/3/5/7/8），
# 这个脚本采的是"不回归"的判据（4/6）与两条核对（控制权归属、来源审计）。
# 两者的**破坏性**不同，混在一起会把前面采的证据毁掉。
#
# **顺序是有依赖的**（每一步的理由都写在下面）：
#   ① plan-checks        —— 判据 4。P2 要"图上基本还全是未知"时的那次规划，
#                           所以必须是**在这个栈上第一个开车/开车之前的动作**。
#   ② check-control-authority —— 裁定 2 的可核查判据：/cmd_vel 发布者只有 route_follower。
#                           它会把 route_follower 后台起起来，所以会开车。
#   ③ audit-sources      —— 来源审计。要在**相机还活着**的时候做（话题订阅者名单才完整）。
#   ④ test-behavior      —— 判据 6。**最后**跑：它末尾会杀掉相机桥接（破坏性），
#                           跑完之后这个栈的感知链路就是残的，不能再采别的证据。
#
# 用法：scripts/rtk-nav2/start-demo.sh <run-id> && scripts/rtk-nav2/regression-run.sh <run-id>
source "$(dirname "${BASH_SOURCE[0]}")/test-common.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
set_paths "${RUN_ID}"
DIR="${HDIR}"
ensure_container
stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"
fix_artifacts_ownership --all   # ★ 跨轮／证据类 ⇒ 整棵 ✓（★ 挂载生效的机器上才需要 ✓）
health_check 0 || die "前置健康检查失败"

log "########## 1) 判据 4：规划（含地图更新后路径随之变化）##########"
scripts/rtk-nav2/plan-checks.sh "${RUN_ID}" --with-lap 2>&1 | tail -25 || true

log "########## 2) 控制权归属：/cmd_vel 发布者必须只有 route_follower ##########"
scripts/rtk-nav2/check-control-authority.sh "${RUN_ID}" --timeout 90 2>&1 | tail -8 || true

log "########## 3) 来源审计（相机还活着时做）##########"
scripts/rtk-nav2/audit-sources.sh "${RUN_ID}" 2>&1 | tail -30 || true

log "########## 4) 判据 6：不回归（基准/劣化/障碍/断相机；末尾会杀相机）##########"
scripts/rtk-nav2/test-behavior.sh "${RUN_ID}" 2>&1 | tail -40 || true

log "########## 5) 汇总 ##########"
python3 "$(dirname "${BASH_SOURCE[0]}")/summarize_evidence.py" "${DIR}" \
    --json "${DIR}/evidence_summary.json" | tail -60 || true

log "回归流程完成，产物在 ${DIR}"
