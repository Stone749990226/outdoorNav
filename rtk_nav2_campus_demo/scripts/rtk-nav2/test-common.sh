#!/usr/bin/env bash
# 第三阶段的两个"自测套件"公共部分（被 test-nav2.sh 与 test-behavior.sh 引入）。
# 只放两边都要用的东西，不做具体场景。

source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

# 容器内跑 ROS 的前缀
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"
SCRIPTS_IN="${WS_DIR}/src/robonix_rtk_nav2/scripts"

# 第三阶段的核心节点：缺一个就不该带着残破环境往下跑
# （第二阶段这一条是硬要求：上一版没做健康检查，脚本在"节点已经崩了"的环境里
#  继续跑了 19 分钟，产出一堆看着正常、其实什么都没测到的产物）
CORE_NODES=(
    "absolute_localizer"
    "danger"
    "ekf_local"
    "wheel_odom_bridge"
    "depth_point_cloud"
    "planner_server"
    "local_costmap_host"
    "route_follower"
)

health_check() {
    local with_route="${1:-1}"
    local missing=""
    for node in "${CORE_NODES[@]}"; do
        if [ "${node}" = "route_follower" ] && [ "${with_route}" -eq 0 ]; then
            continue
        fi
        if ! cexec_t 20 "pgrep -f '[${node:0:1}]${node:1}' >/dev/null" >/dev/null 2>&1; then
            missing="${missing} ${node}"
        fi
    done
    if [ -n "${missing}" ]; then
        log "** 前置健康检查失败，缺节点：${missing} **"
        return 1
    fi
    # **别用 ${with_route:+...}**：with_route=0 时它照样展开（"0" 是非空字符串），
    # 日志会写成"含 route_follower"——而那时恰恰是**故意不检查**它的场景，属假陈述。
    local scope="不含 route_follower（本套件会自己启动它）"
    [ "${with_route}" = "1" ] && scope="含 route_follower"
    log "前置健康检查通过（核心节点：${scope}）"
    return 0
}

# 两张图是不是都还活着（发布者数 > 0）。判据 1/3 都靠这个前提。
require_costmaps() {
    local ok=1
    for t in /global_costmap/costmap /local_costmap/costmap; do
        local n
        n="$(cexec_t 30 "${NAS} && timeout 20 ros2 topic info ${t} 2>/dev/null | awk '/Publisher count/{print \$3}'" 2>/dev/null | tr -d '\r' | tail -1)"
        if [ "${n:-0}" != "1" ]; then
            log "** ${t} 的发布者数 = ${n:-?}（应为 1）**"
            ok=0
        else
            log "  ${t} 发布者数 1 ✓"
        fi
    done
    [ "${ok}" -eq 1 ]
}

# 在容器内跑一个探针并把输出落到产物目录（带超时；探针挂住不能拖死整个套件）。
#
# **命令里的路径要用 CDIR（容器视角）**，不是宿主路径：项目根在容器里挂成 /work。
# 把宿主路径塞进 cexec 会指向不存在的目录，而现象是"探针静默什么都没做"。
CDIR=""          # 由调用方在 run_probe 之前用 set_paths <run-id> 设好
run_probe() {
    local run_id="$1" out_rel="$2" secs="$3"
    shift 3
    local dir
    dir="$(artifact_dir "${run_id}")"
    mkdir -p "$(dirname "${dir}/${out_rel}")"
    # **stdout 不能落到探针自己也要写的那个 .json 上**：很多探针同时用
    # `--json <同一路径>` 写文件、又往 stdout 打同一份 JSON，两条路写同一个文件 →
    # 文件里两份内容接在一起、**不是合法 JSON**。实测受害的是
    # `spin/spin_summary.json`（e1/e3 都坏），而它"文件在、大小也正常"，
    # 只有真的 json.load 才会发现。
    local stdout_to="${dir}/${out_rel}"
    case "${out_rel}" in
        *.json) stdout_to="${dir}/${out_rel%.json}.stdout.txt" ;;
    esac
    set +e
    cexec_t "${secs}" "${NAS} && cd ${SCRIPTS_IN} && python3 $*" \
        > "${stdout_to}" 2>&1
    local rc=$?
    set -e
    log "  探针 ${out_rel} 退出码 ${rc}"
    return "${rc}"
}

# 设好宿主/容器两套路径
set_paths() {
    HDIR="$(artifact_dir "$1")"
    CDIR="$(artifact_dir_c "$1")"
    mkdir -p "${HDIR}"
}
