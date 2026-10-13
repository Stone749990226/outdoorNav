#!/usr/bin/env bash
# 控制权归属核对（brief §10 裁定 2 的可核查判据）：/cmd_vel 的发布者**只有 route_follower**。
#
# 为什么单独一个脚本、而不是塞在某次跑圈的尾巴上：判据要求的是"**运行中**的发布者名单"
# ——route_follower 只在跑的时候才存在，跑完就退出了。所以这里把路线跟随**后台起起来**，
# 在它运行期间取一次话题信息，再让它跑完（或超时打断）。
#
# 用法：scripts/rtk-nav2/check-control-authority.sh <run-id> [--timeout 90]
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
TIMEOUT=90
shift || true
while [ $# -gt 0 ]; do
    case "$1" in
        --timeout) TIMEOUT="$2"; shift 2 ;;
        *) shift ;;
    esac
done
DIR="$(artifact_dir "${RUN_ID}")/nav2"
mkdir -p "${DIR}"
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"

stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"

log "后台起路线跟随（${TIMEOUT} s），运行期间取 /cmd_vel 的话题信息"
cexec_bg "${CONTAINER_WORKDIR}/artifacts/rtk-nav2/${RUN_ID}/logs/authority_route.log" \
    "${NAS} && cd ${WS_DIR} && timeout -s INT -k 10 ${TIMEOUT} ros2 run robonix_rtk_nav2 route_follower \
     --ros-args --params-file ${WS_DIR}/install/robonix_rtk_nav2/share/robonix_rtk_nav2/config/route.yaml \
     -p use_sim_time:=true -p total_timeout:=${TIMEOUT}.0"

# 等它真的起来再取（不然拿到的是"还没发布者"的空名单，看起来像通过）
for _ in $(seq 1 30); do
    if cexec_t 20 "pgrep -f '[r]oute_follower' >/dev/null" >/dev/null 2>&1; then break; fi
    sleep 2
done
sleep 6

cexec_t 60 "${NAS} && timeout 30 ros2 topic info -v /cmd_vel" \
    > "${DIR}/cmd_vel_publishers_running.txt" 2>&1 || true
PUB="$(grep -A4 'Publisher count' "${DIR}/cmd_vel_publishers_running.txt" 2>/dev/null \
       | grep 'Node name' | awk '{print $3}' | sort -u | tr '\n' ' ')"
log "  运行中 /cmd_vel 的发布者：${PUB:-（无）}"

# 顺带把"谁在发 /cmd_vel"的原始行也留下（评审可以直接看）
cexec_t 60 "${NAS} && timeout 30 ros2 node info /route_follower" \
    > "${DIR}/node_info_route_follower.txt" 2>&1 || true
grep -A6 "Publishers" "${DIR}/node_info_route_follower.txt" 2>/dev/null | head -8 \
    > "${DIR}/route_follower_publishers.txt" || true

{
    echo "=== 控制权归属判据 ==="
    echo "brief §10 裁定 2：运行中 /cmd_vel 的发布者**只有** route_follower。"
    echo "本机实测发布者：${PUB:-（无）}"
    echo "原始话题信息：cmd_vel_publishers_running.txt"
    echo "route_follower 的发布者列表：route_follower_publishers.txt"
} > "${DIR}/control_authority_verdict.txt"
cat "${DIR}/control_authority_verdict.txt"
log "等后台的路线跟随自己结束（最长 $((TIMEOUT + 40)) s）"
sleep $((TIMEOUT + 40))
log "完成"
