#!/usr/bin/env bash
# 四个入口脚本共用的辅助函数。本文件不单独执行，用 source 引入。
#
# 设计约定：
#   - 演示全部运行在专用容器 robonix-slam-demo 内，容器只挂载本项目的目录；
#   - 宿主机上不安装、不启动任何 ROS 进程，因此 stop-demo.sh 不可能影响其他项目；
#   - 图形输出到当前用户的远程桌面会话；显示号自动探测，可用 DEMO_DISPLAY 覆盖。

set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-robonix-slam-demo}"
# ★★ 判断依据是「哪条 Gazebo 路线」，不是 CPU 架构 ✗（2026-10-10 移植时改）
#
#   原先：`case "$(uname -m)"` ⇒ aarch64 走 Fortress、其余走 Classic ✗
#   ⇒ 在 x86 上必然选到 **Classic** ✗ —— 而 ★ **本项目阶段三~八全部按 Fortress 写成** ✓
#     （SDF／插件／ros_gz 桥接／launch 文件都是 Fortress ✓）⇒ 选 Classic 等于整条链回改 ✗。
#   ⇒ 根因：★ **同一个键（架构）被用来选三件事** ✗ —— ① 镜像 tag ✓ ② Gazebo 实现 ✓ ③ launch 文件 ✓
#     而「x86 + Fortress」这个组合**历史上不存在** ⇒ ★ 按键选必然至少错一个 ✓。
#   ⇒ 修法：★ **换键** ✓（★ 改形状，不改记性 ✓）—— 路线一处定义、多处消费 ✓。
#
#   ★ 默认取 fortress ✓：本仓库既有成果**全是** Fortress ✓；`classic` 只在显式指定时用 ✓。
GAZEBO_ROUTE="${GAZEBO_ROUTE:-fortress}"     # fortress | classic
case "${GAZEBO_ROUTE}" in
    fortress|classic) ;;
    *) die "GAZEBO_ROUTE 只能是 fortress 或 classic，收到：${GAZEBO_ROUTE}" ;;
esac
# 镜像 tag 按【路线 + 架构】派生：Fortress 路线区分架构 ✓；Classic 路线沿用无后缀名 ✓
ARCH_TAG=""
if [ "${GAZEBO_ROUTE}" = "fortress" ]; then
    case "$(uname -m)" in
        aarch64|arm64) ARCH_TAG="-arm64" ;;
        *)             ARCH_TAG="-amd64" ;;
    esac
fi
IMAGE_NAME="${IMAGE_NAME:-robonix/slam-demo:humble${ARCH_TAG}}"
DEMO_DISPLAY="${DEMO_DISPLAY:-}"        # 留空则自动探测（见 detect_display）
CONTAINER_WORKDIR="/work"
WS_DIR="${CONTAINER_WORKDIR}/demo_ws"
ARTIFACTS_DIR="${CONTAINER_WORKDIR}/artifacts"
CURRENT_RUN_FILE="${ARTIFACTS_DIR}/.current-run"

# 项目根目录 = 本脚本所在目录的上级
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

log() { printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*"; }

# 判断当前用户的活动图形会话是不是 Wayland：
# 6001 是 xrdp + Xorg（可用 ffmpeg x11grab 直接抓屏、wmctrl 可移动窗口）；
# 6002 是 gnome-remote-desktop + GNOME Wayland（X 应用走 Xwayland，
#   x11grab 抓到的是空画面、wmctrl 也移不动窗口，需要换一套办法）。
session_is_wayland() {
    local t=""
    if command -v loginctl >/dev/null 2>&1; then
        for s in $(loginctl list-sessions --no-legend 2>/dev/null | awk -v u="$(id -u)" '$2==u {print $1}'); do
            t="$(loginctl show-session "${s}" -p Type --value 2>/dev/null)"
            [ "${t}" = "wayland" ] && return 0
        done
    fi
    return 1
}

# Wayland 会话下 Xwayland 的授权文件由 mutter 生成，文件名每次会话都不同
detect_xauthority() {
    local f
    f="$(ls -1t /run/user/$(id -u)/.mutter-Xwaylandauth.* 2>/dev/null | head -1)"
    if [ -n "${f}" ]; then echo "${f}"; return 0; fi
    if [ -f "${HOME}/.Xauthority" ]; then echo "${HOME}/.Xauthority"; return 0; fi
    return 1
}

# 自动挑选图形显示：优先当前用户自己的 X socket，其次第一个存在的。
# 不同机器的远程桌面显示号不一定相同（6001 上是 :10），因此不写死。
detect_display() {
    [ -n "${DEMO_DISPLAY}" ] && return 0
    local uid sock n owner
    uid="$(id -u)"
    for sock in /tmp/.X11-unix/X*; do
        [ -S "${sock}" ] || continue
        n="${sock##*X}"
        owner="$(stat -c '%u' "${sock}" 2>/dev/null || echo 0)"
        if [ "${owner}" = "${uid}" ]; then
            DEMO_DISPLAY=":${n}"
            return 0
        fi
    done
    for sock in /tmp/.X11-unix/X*; do
        [ -S "${sock}" ] || continue
        DEMO_DISPLAY=":${sock##*X}"
        return 0
    done
    return 1
}
die() { printf '[错误] %s\n' "$*" >&2; exit 1; }

require_docker() {
    command -v docker >/dev/null 2>&1 || die "找不到 docker 命令"
    docker info >/dev/null 2>&1 || die "当前用户无法访问 docker（需要加入 docker 组）"
}

container_running() {
    [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null || echo false)" = "true" ]
}

ensure_container() {
    require_docker
    detect_display || die "找不到可用的 X 显示。请确认远程桌面会话已建立，或用 DEMO_DISPLAY=:N 指定"
    if container_running; then
        return 0
    fi
    if docker inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
        log "启动已存在的容器 ${CONTAINER_NAME}"
        docker start "${CONTAINER_NAME}" >/dev/null
    else
        [ -S "/tmp/.X11-unix/X${DEMO_DISPLAY#:}" ] || \
            die "显示 ${DEMO_DISPLAY} 不存在。请确认远程桌面会话已建立，或用 DEMO_DISPLAY=:N 指定"
        # aarch64（Jetson）上实测 DDS 组播发现在这个网络环境下不可靠：
        # slam_toolbox 的服务能出现在 ros2 service list 里，但调用永远不回响应
        # （serialize_map 卡死、pause_new_measurements 时有时无）。
        # 本演示所有节点都在同一台机器上，限制只用回环即可，加上这个环境变量后恢复正常。
        # x86_64 机器（6001）保持原样不动，避免影响已验收的环境。
        local xauth
        xauth="$(detect_xauthority)" || die "找不到可用的 X 授权文件（Xauthority）"
        log "X 授权文件：${xauth}"
        local extra_env=()
        if [ "$(uname -m)" = "aarch64" ] || [ "$(uname -m)" = "arm64" ]; then
            extra_env+=(-e "ROS_LOCALHOST_ONLY=1")
        fi
        log "创建容器 ${CONTAINER_NAME}（显示 ${DEMO_DISPLAY}）"
        docker run -d --name "${CONTAINER_NAME}" \
            --network host \
            --hostname "$(hostname)" \
            -e "DISPLAY=${DEMO_DISPLAY}.0" \
            -e "XAUTHORITY=/xauth" \
            -e "TURTLEBOT3_MODEL=burger" \
            -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
            "${extra_env[@]}" \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v "${xauth}:/xauth:ro" \
            -v "${PROJECT_DIR}:${CONTAINER_WORKDIR}" \
            -w "${CONTAINER_WORKDIR}" \
            "${IMAGE_NAME}" sleep infinity >/dev/null
    fi
    # 等待容器内可以执行命令
    for _ in $(seq 1 30); do
        if docker exec "${CONTAINER_NAME}" true >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    die "容器 ${CONTAINER_NAME} 未能就绪"
}

# 在容器内执行命令（非交互）
cexec() {
    docker exec "${CONTAINER_NAME}" bash -lc "$*"
}

# 在容器内后台执行命令，日志写到指定文件
cexec_bg() {
    local logfile="$1"; shift
    docker exec -d "${CONTAINER_NAME}" bash -lc "$* > '${logfile}' 2>&1"
}

workspace_built() {
    docker exec "${CONTAINER_NAME}" test -f "${WS_DIR}/install/setup.bash"
}

ensure_workspace() {
    # 每次都干净重编：先删掉 build/ install/ 与 __pycache__ 再 colcon build，
    # 避免旧的 .pyc 让“改了代码却仍跑旧逻辑”的情况发生（本包编译只需 1~2 秒）。
    if ! workspace_built; then
        log "首次运行，编译工作区"
    fi
    cexec "/work/scripts/container/build_workspace.sh" | tail -3
    log "工作区已重新编译（colcon build）"
}

# 直接在容器里跑 pgrep（不经 bash -lc），否则包装用的 bash 自身命令行会包含
# 匹配串，pgrep -f 会把 bash 也匹配上，导致“永远为真”。
container_process_running() {
    docker exec "${CONTAINER_NAME}" pgrep -f "$1" >/dev/null 2>&1
}

demo_stack_running() {
    # x86_64 用 demo.launch.py（Gazebo Classic），aarch64 用 demo_gz.launch.py（Gazebo Sim）
    container_process_running "ros2 launch robonix_slam_demo demo.launch.py" \
        || container_process_running "ros2 launch robonix_slam_demo demo_gz.launch.py"
}

map_server_running() {
    container_process_running "nav2_map_server/map_server"
}

current_run_id() {
    if [ -f "${PROJECT_DIR}/artifacts/.current-run" ]; then
        cat "${PROJECT_DIR}/artifacts/.current-run"
    fi
}

set_current_run() {
    mkdir -p "${PROJECT_DIR}/artifacts"
    printf '%s' "$1" > "${PROJECT_DIR}/artifacts/.current-run"
}

resolve_run_id() {
    local run_id="${1:-}"
    if [ -z "${run_id}" ]; then
        run_id="$(current_run_id)"
    fi
    if [ -z "${run_id}" ]; then
        run_id="$(date '+%Y%m%d-%H%M%S')"
    fi
    printf '%s' "${run_id}"
}

# 容器以 root 运行，写出的文件在宿主机上属主是 root。为了让宿主机脚本也能
# 在同一个产物目录里写日志，这里统一把 artifacts 目录的属主改回当前用户。
fix_artifacts_ownership() {
    mkdir -p "${PROJECT_DIR}/artifacts"
    # 只检查 artifacts 目录本身是不够的：容器会在里面建出 root 属主的子目录，
    # 之后宿主机脚本往这些子目录里写就会 permission denied。
    # 这里每次都对整棵树做一次 chown（alpine 容器，约 1 秒），不给残留留机会。
    docker run --rm -v "${PROJECT_DIR}/artifacts:/a" alpine:3.22.0 \
        chown -R "$(id -u):$(id -g)" /a >/dev/null 2>&1 || true
}

# 提前用当前用户创建本次运行的目录树，避免容器先建成 root 属主导致宿主机无法写日志
ensure_run_dirs() {
    local run_id="$1"
    mkdir -p "${PROJECT_DIR}/artifacts/${run_id}/logs" \
             "${PROJECT_DIR}/artifacts/${run_id}/screenshots" \
             "${PROJECT_DIR}/artifacts/${run_id}/bag" \
             "${PROJECT_DIR}/artifacts/${run_id}/map"
}

# 远程桌面会话若处于锁定/熄屏状态，先解锁，否则截图只会得到锁屏画面。
# 只解锁当前用户自己的会话（loginctl 也仅允许会话属主这么做），不涉及 X 访问控制。
unblank_display() {
    local uid sid
    uid="$(id -u)"
    command -v loginctl >/dev/null 2>&1 || return 0
    sid=""
    for s in $(loginctl list-sessions --no-legend 2>/dev/null | awk -v u="${uid}" '$2==u {print $1}'); do
        if [ "$(loginctl show-session "${s}" -p LockedHint --value 2>/dev/null)" = "yes" ]; then
            sid="${s}"
        fi
    done
    if [ -n "${sid}" ]; then
        log "远程桌面会话 ${sid} 处于锁定状态，解锁以便显示并截图演示窗口"
        loginctl unlock-session "${sid}" 2>/dev/null || log "解锁失败（可能需要手动解锁）"
        sleep 2
    fi
}

# 脚本退出时把产物目录属主交还给当前用户，避免容器（root）写入后宿主机无法清理
trap 'fix_artifacts_ownership >/dev/null 2>&1 || true' EXIT

# 抓取当前 X 显示画面作为图形证据
capture_screen() {
    local outfile="$1"
    # GNOME 在会话空闲一段时间后会自动锁屏，锁屏状态下抓到的只是锁屏画面，
    # 所以每次截图前先解锁当前用户自己的会话。
    unblank_display

    if session_is_wayland; then
        # Wayland：X 根窗口是空的（窗口由合成器合成），x11grab 只能抓到黑屏，
        # 必须用 GNOME 自己的截图工具抓合成后的画面。
        command -v gnome-screenshot >/dev/null 2>&1 || {
            log "Wayland 会话但没有 gnome-screenshot，跳过截图"; return 1; }
        XDG_RUNTIME_DIR="/run/user/$(id -u)" \
        DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus" \
            gnome-screenshot -f "${outfile}" >/dev/null 2>&1 || return 1
        return 0
    fi

    local display="${DEMO_DISPLAY}.0"
    command -v ffmpeg >/dev/null 2>&1 || { log "未安装 ffmpeg，跳过截图"; return 1; }
    local size="${SCREEN_SIZE:-$(detect_screen_size)}"
    DISPLAY="${display}" ffmpeg -hide_banner -loglevel error \
        -f x11grab -video_size "${size}" -i "${display}" \
        -frames:v 1 -y "${outfile}"
}

# 从 xdpyinfo 读当前显示分辨率，避免不同机器上写死 1920x1080 抓出黑边
detect_screen_size() {
    local display="${DEMO_DISPLAY}.0" size
    size="$(DISPLAY="${display}" xdpyinfo 2>/dev/null | awk '/dimensions:/ {print $2; exit}')"
    if [ -n "${size}" ]; then echo "${size}"; else echo "1920x1080"; fi
}
