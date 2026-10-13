#!/usr/bin/env bash
# RTK 第三阶段（复用 nav2 建图与规划）的公共函数。用 source 引入，不单独执行。
#
# 与 scripts/rtk/lib.sh（第二阶段）**互相独立**：独立容器、独立镜像、独立 ROS_DOMAIN_ID、
# 独立 Gazebo 分区、独立工作区与产物目录。**不动** rtk_ws/、scripts/rtk/、artifacts/rtk/。
# 这里不复用第二阶段的文件，避免"改了一份却跑了另一份"。
#
# 留痕机制照第二阶段继续（run_state.txt / run_kind / injection= / config_fingerprint），
# 并按 brief §10 增加了**镜像钉版**的记录（镜像 ID + depth_image_proc 版本）。

set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-${DEMO_CONTAINER:-rtknav2-demo}}"
# ★★ 镜像名按【架构】派生，✗ 不再写死 `humble-arm64` ✗（2026-10-10 移植时改）
#   写死的代价：★ 在本机（x86_64）起栈时去拉 `…:humble-arm64` ✗ ⇒ `pull access denied` ⇒ **起不来** ✓
#   ⇒ 与 `scripts/lib.sh` 的 `ARCH_TAG` **同一条规则** ✓（★ 那两支 lib 刻意独立 ⇒ 这里自带一份 ✓）
#   ★ 判据：`uname -m` ⇒ amd64 走 `-amd64` ✓、aarch64 走 `-arm64` ✓（★ 与本机/6002 各自的镜像名一致 ✓）
#   ✗ 别写成"固定 -amd64" ✗ —— ★ 6002 跑的是同一份脚本，那样会把 ARM 那边弄坏 ✓
_ARCH_TAG="-amd64"
case "$(uname -m)" in
    aarch64|arm64) _ARCH_TAG="-arm64" ;;
esac
IMAGE_NAME="${IMAGE_NAME:-robonix/rtk-nav2-demo:humble${_ARCH_TAG}}"
DEMO_DISPLAY="${DEMO_DISPLAY:-}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-44}"
GZ_PARTITION="${GZ_PARTITION:-rtknav2}"
CONTAINER_WORKDIR="/work"
# ★★ 2026-10-11（阶段十 · 第 0 刀）：**工作区名可覆盖** ✓ ★★
#   为什么需要：二分要拿**旧树**跑一轮 ✓，而边界是"**✗ 不许就地改工作树** ✗、用**独立副本**跑" ✓。
#   本链路上 `rtk_nav2_ws` 是**写死**的（build/install/launch 全按它拼路径 ✗）
#   ⇒ 与其改十几处，不如**一处变量**把整条链路切到另一个工作区 ✓。
#   ★ 默认值 = 原字面量 ⇒ **现有所有入口行为逐字不变** ✓（这条是可核的：不设该变量时
#     `WS_DIR`／`PKG_DIR` 与原值逐字符相同 ✓）。
WORKSPACE_NAME="${WORKSPACE_NAME:-rtk_nav2_ws}"
WS_DIR="${CONTAINER_WORKDIR}/${WORKSPACE_NAME}"
ARTIFACTS_ROOT="${CONTAINER_WORKDIR}/artifacts/rtk-nav2"

# 本文件在 scripts/rtk-nav2/ 下，项目根目录要往上两级
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PKG_DIR="${PROJECT_DIR}/${WORKSPACE_NAME}/src/robonix_rtk_nav2"

log() { printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*"; }
die() { printf '[错误] %s\n' "$*" >&2; exit 1; }

require_docker() {
    command -v docker >/dev/null 2>&1 || die "找不到 docker 命令"
    docker info >/dev/null 2>&1 || die "当前用户无法访问 docker"
}

detect_display() {
    [ -n "${DEMO_DISPLAY}" ] && return 0
    local uid sock owner
    uid="$(id -u)"
    for sock in /tmp/.X11-unix/X*; do
        [ -S "${sock}" ] || continue
        owner="$(stat -c '%u' "${sock}" 2>/dev/null || echo 0)"
        if [ "${owner}" = "${uid}" ]; then DEMO_DISPLAY=":${sock##*X}"; return 0; fi
    done
    for sock in /tmp/.X11-unix/X*; do
        [ -S "${sock}" ] || continue
        DEMO_DISPLAY=":${sock##*X}"; return 0
    done
    return 1
}

detect_xauthority() {
    local f
    f="$(ls -1t /run/user/$(id -u)/.mutter-Xwaylandauth.* 2>/dev/null | head -1)"
    if [ -n "${f}" ]; then echo "${f}"; return 0; fi
    [ -f "${HOME}/.Xauthority" ] && { echo "${HOME}/.Xauthority"; return 0; }
    return 1
}

# ★★ 2026-10-09（阶段八）**起栈前的 display 自检 + 统一传递** ★★
#
# 为什么要有（一次真实事故，`campus8-a`）：容器是**2026-09-29 建的**，baked
# `DISPLAY=:10.0`；而 `:10` 的 X socket 属**另一个账号**（宿主 `/tmp/.X11-unix/`：
# `X10` 属 `czn`、`X11` 属 `xjy`）⇒ Gazebo GUI 报
# `Invalid MIT-MAGIC-COOKIE-1 key / could not connect to display :10.0` ⇒ Qt 起不来 ⇒
# **整个 `ign gazebo` 进程死掉**（连 server 一起）⇒ 没有模型、没有传感器。
#
# **迷惑点才是真代价**：现象是 `ros2 node list` 有 29 个节点、话题都在、`/rtk/static_map`
# 也有发布者 ⇒ 看起来"栈起来了"，但 TF 里**没有 map**、`/rtk/navsat` 全空 ⇒
# 长得像"定位链上游没起来"，而不像"仿真器死了"。**"栈起来了"与"仿真器活着"是两件事。**
#
# `lib.sh` 里本来就有"按 socket 属主挑 display"的 `detect_display`、`launch/demo.launch.py`
# 的头注释也早写了这件事 —— **缺的只是没有一个入口把它传下去** ✗。
# ⇒ 收敛到这一处：**所有起栈入口**（`start-demo.sh` / `start-campus.sh` / `start-depth-drop.sh`）
#   都调 `ensure_display`，不再各写一遍（否则下次新入口再漏一次）。
#
# 自检三件事（reviewer 2026-10-09 要求"失败直接拒起"）：
#   ① 选中 display 的 socket 必须**属当前 uid**；② `xdpyinfo` 必须成功；
#   ③ 通过后 **export `ROBONIX_DEPTH_DROP_DISPLAY`**（`container/start_stack.sh` 认它，
#      只在显式设置时才覆盖容器里 baked 的那个 DISPLAY）。
ensure_display() {
    detect_display || die "找不到可用的 X display（/tmp/.X11-unix 下没有 socket？）⇒ 请在带桌面的会话里跑"
    local sock="/tmp/.X11-unix/X${DEMO_DISPLAY#:}"
    [ -S "${sock}" ] || die "display ${DEMO_DISPLAY} 的 socket ${sock} 不存在 ⇒ 拒起"
    local owner; owner="$(stat -c '%u' "${sock}" 2>/dev/null || echo '?')"
    if [ "${owner}" != "$(id -u)" ]; then
        die "display ${DEMO_DISPLAY}（socket ${sock}）属 uid ${owner}，**不是当前 uid $(id -u)** ⇒ 拒起。
  这就是 `campus8-a` 那次的病：容器 baked 了别人的 DISPLAY ⇒ Gazebo GUI 起不来 ⇒
  **整个 ign gazebo 进程死掉**、而 `ros2 node list` 照样有一堆节点（看起来像"定位没起来"）。
  处置：export DEMO_DISPLAY=:<你自己的那个号>（本机 /tmp/.X11-unix 下属于 uid $(id -u) 的那个）"
    fi
    local xa; xa="$(detect_xauthority || true)"
    if ! timeout 10 env DISPLAY="${DEMO_DISPLAY}.0" ${xa:+XAUTHORITY="${xa}"} \
         xdpyinfo >/dev/null 2>&1; then
        die "display ${DEMO_DISPLAY}.0 的 xdpyinfo 失败（X 认证/权限问题）⇒ 拒起"
    fi
    export ROBONIX_DEPTH_DROP_DISPLAY="${ROBONIX_DEPTH_DROP_DISPLAY:-${DEMO_DISPLAY}.0}"
    log "display 自检通过：DEMO_DISPLAY=${DEMO_DISPLAY}（socket 属当前 uid ✓、xdpyinfo ✓）；传给 Gazebo/RViz 的 DISPLAY=${ROBONIX_DEPTH_DROP_DISPLAY}"
}

container_running() {
    [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null || echo false)" = "true" ]
}

ensure_container() {
    require_docker
    detect_display || die "找不到可用的 X 显示，请确认远程桌面会话已建立"
    if container_running; then return 0; fi
    # ★★ 2026-10-13（★ 业主实测撞到的**真根因** ✗）：**只判断「存不存在」是不够的** ✗ ★★
    #   ★ 现象 ✓：★ 机器上有一个**旧镜像**建的同名容器 ⇒ ★ `docker start` 把**旧容器**拉起来 ✗
    #     ⇒ ★ 里面**没有新镜像才有的东西** ✗（★ 实测：`/opt/outdoor-patrol-demo` 不在 ⇒
    #       `frontend-bridge.sh` 报「镜像里没有前端依赖」✗ ⇒ 「**一条命令**」这个承诺在那台机器上**不成立** ✗）
    #   ★ 判据 ✓：★ 拿**容器实际用的 image id** 与 **当前 tag 指向的 image id** 比 ✓
    #     —— ★ ✗ **别比 tag 字符串** ✗（★ tag 可以指向别的 id ✓ ⇒ 比字符串会**假通过** ✗）
    _NEED_CREATE=0
    if docker inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
        _cimg="$(docker inspect -f '{{.Image}}' "${CONTAINER_NAME}" 2>/dev/null || true)"
        _timg="$(docker image inspect "${IMAGE_NAME}" --format '{{.Id}}' 2>/dev/null || true)"
        if [ -n "${_timg}" ] && [ "${_cimg}" != "${_timg}" ]; then
            log "★★ 容器 ${CONTAINER_NAME} 是**旧镜像**建的 ✗ ⇒ **重建** ✓（✗ 不静默重建 ✗，理由打出来 ✓）"
            log "   ★ 容器镜像 id = ${_cimg#sha256:}"
            log "   ★ ${IMAGE_NAME} 现在 = ${_timg#sha256:}"
            log "   ★ 处置：docker rm -f ${CONTAINER_NAME} ⇒ 再按当前镜像建 ✓"
            docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 \
                || die "旧容器 ${CONTAINER_NAME} 删不掉 ⇒ 拒起（✗ 不用旧镜像硬起 ✗）"
            _NEED_CREATE=1
        else
            log "启动已存在的容器 ${CONTAINER_NAME}（★ 镜像 id 与当前 ${IMAGE_NAME} 一致 ✓）"
            docker start "${CONTAINER_NAME}" >/dev/null
        fi
    else
        _NEED_CREATE=1
    fi
    if [ "${_NEED_CREATE}" = "1" ]; then
        # ★★ 鉴权参数按【实际能不能连上】来定，✗ 不是按"有没有那个文件" ✗
        #   （2026-10-10 移植到本机 WSL 时改。两条实测都在这台机器上做的 ✓）
        #   ① ★ 原写法 `detect_xauthority || die` ✗ ⇒ 本机**没有** `.mutter-Xwaylandauth.*`、
        #      也**没有** `~/.Xauthority` ⇒ 直接判死 ⇒ **整条起栈根本走不到 docker run** ✓
        #   ② ★ 而本机的 X **根本不需要鉴权** ✓ —— 实测：不带任何 XAUTHORITY 也能连上、
        #      读到根窗口与屏幕尺寸 ✓（`scripts/rtk-nav2/x11_probe.py` 就是那个探针 ✓）
        #   ⇒ ★★ 判据换成 **"不带 auth 能不能连上"** ✓（这是**目标量** ✓，"有没有文件"只是**相邻量** ✗）
        #   ⇒ ★ XAUTHORITY 传**空** ✓ —— ★ 这**正是 WSLg 要的** ✓，
        #     而原写法传 `/xauth` 是 **ARM 机（mutter + Xorg）** 的做法 ✓，在本机是**错的** ✗
        local xauth="" auth_args=()
        if xauth="$(detect_xauthority 2>/dev/null)"; then
            auth_args=(-e "XAUTHORITY=/xauth" -v "${xauth}:/xauth:ro")
            log "X 鉴权：用 ${xauth} ⇒ 容器内 XAUTHORITY=/xauth"
        elif timeout 10 env DISPLAY="${DEMO_DISPLAY}.0" XAUTHORITY= xdpyinfo >/dev/null 2>&1; then
            auth_args=(-e "XAUTHORITY=")
            log "X 鉴权：★ 找不到 Xauthority 文件，但**不带鉴权能连上** ⇒ 容器内 XAUTHORITY=（空）"
        else
            die "显示 ${DEMO_DISPLAY}.0 既找不到 Xauthority 文件、不带鉴权也连不上 ⇒ 拒起"
        fi
        # ★★ 2026-10-12：**场景去重**的外部挂载 ✓（与 `container/start_stack.sh` 里那段**配套** ✓）
        #   并入他们的仓之后，本子树**不带** `worlds/models/campus_scene` ✓，
        #   场景在他们仓的 `<外层>/gazebo_campus/generated/models` ✓
        #   —— ★ 而 `/work` 只挂**本子树** ✗ ⇒ ★ 不单独挂进来，容器里**看不见**它 ✗
        #      ⇒ 世界加载失败（★ 表相是"**看着像卡在起栈**"✗ —— 这个坑我们记过 ✓）
        #   ★ ★ **空操作的形态** ✓：本仓里那层**不存在** ⇒ ★ 数组为空 ⇒ **命令行逐字不变** ✓
        SCENE_MOUNT_ARGS=()
        EXTERNAL_MODELS_C="/opt/rtk_nav2_external_models"
        _outer_scene="$(cd "${PROJECT_DIR}/.." 2>/dev/null && pwd)/gazebo_campus/generated/models"
        if [ -d "${_outer_scene}" ] && [ ! -d "${PROJECT_DIR}/gazebo_campus/generated/models" ]; then
            # ★★ 挂到**容器内的路径** ✗ 不挂进 `/work` ✓ —— ★ 这是实测改的 ✓：
            #   第一版挂到 `${CONTAINER_WORKDIR}/gazebo_campus/generated/models` ✗
            #   ⇒ ★ **Docker 要在宿主的子树里创建那几个中间目录** ✗ ⇒ 建出来的是
            #     **root 属主**的目录 ✗ ⇒ ★ **把宿主的树弄脏了**（连 `rm` 都删不掉 ✗）
            #   ⇒ ★ 挂到容器自己的路径 ⇒ ★ Docker **不碰宿主的树** ✓（只影响容器 ✓）
            SCENE_MOUNT_ARGS=(-v "${_outer_scene}:${EXTERNAL_MODELS_C}:ro")
            log "外层场景：${_outer_scene} ⇒ 只读挂到 ${EXTERNAL_MODELS_C}（★ 容器内路径 ✓，不脏宿主 ✓）"
        fi
        log "创建容器 ${CONTAINER_NAME}（显示 ${DEMO_DISPLAY}，ROS_DOMAIN_ID=${ROS_DOMAIN_ID}，GZ_PARTITION=${GZ_PARTITION}）"
        docker run -d --name "${CONTAINER_NAME}" \
            --init \
            --network host \
            --hostname "$(hostname)" \
            -e "DISPLAY=${DEMO_DISPLAY}.0" \
            ${auth_args[@]+"${auth_args[@]}"} \
            -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
            -e "ROS_LOCALHOST_ONLY=1" \
            -e "GZ_PARTITION=${GZ_PARTITION}" \
            -e "IGN_PARTITION=${GZ_PARTITION}" \
            -e "TURTLEBOT3_MODEL=burger" \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v "${PROJECT_DIR}:${CONTAINER_WORKDIR}" \
            ${SCENE_MOUNT_ARGS[@]+"${SCENE_MOUNT_ARGS[@]}"} \
            -w "${CONTAINER_WORKDIR}" \
            "${IMAGE_NAME}" sleep infinity >/dev/null
    fi
    for _ in $(seq 1 30); do
        docker exec "${CONTAINER_NAME}" true >/dev/null 2>&1 && return 0
        sleep 1
    done
    die "容器未能就绪"
}

# ★ docker 的可执行文件**显式解析一次**：本仓库根下有一个 `docker/` 目录，
#   若某个 shell 的 PATH 里含 `.`，直接写 `docker` 会被解析成**同名的目录**，
#   报 `exec: docker: cannot execute: Is a directory`（实测在本地 WSL 上踩到过一次）。
#   主机上正常解析到 /usr/bin/docker；解析不到时退回裸名 `docker`（保持原行为）。
DOCKER_BIN="${DOCKER_BIN:-}"
if [ -z "${DOCKER_BIN}" ]; then
    if [ -x /usr/bin/docker ]; then
        DOCKER_BIN=/usr/bin/docker
    else
        DOCKER_BIN="$(command -v docker 2>/dev/null || echo docker)"
    fi
fi

# ★★ 2026-10-10（reviewer ③″ 结案）：**凡 `cexec` 出来的进程，默认就该看到"本轮的分区"** ★★
#   **根因（已由代码三处 + 运行时实测确认 ✓）**：栈那一行带前缀 `GZ_PARTITION=rtknav2_${RUN_ID}` ✓，
#   而**探针那一行没带** ✗；`cexec` 又没有 `-e` ✗ ⇒ 它继承**容器创建时**的 `GZ_PARTITION=rtknav2` ✓
#   ⇒ `wait_ready.py` 在 `rtknav2` 里找本轮的 `/world/` ⇒ **必然 0** ✗ ⇒ **假"仿真器不在"** ✗
#   ⇒ 触发重试 ⇒ 重叠 ⇒ 时钟归零 ⇒ ④/3-10/整轮健康 全废 ✓。
#   ★ **只补那一行不够** ✓（下一处还会忘 ✓ —— 今天就忘了一次 ✓）⇒ **改成结构性的**：
#     在**这里**统一注入 ⇒ **任何** caller 都自动拿到本轮分区 ✓（含那些"从外面 cexec 进来碰 gz"的 ✓，
#     它们此前全打在**默认分区** ✗ ⇒ 会**静默什么都不做 / 或改到残留仿真上** ✗）。
# ★★★ 2026-10-10（reviewer 找到 §4 真因）：**注入一律用 `export`，不用"环境前缀"** ✗ ★★★
#   坑：`VAR=x cmd1 && cmd2` 里**赋值只作用于 `cmd1`** ✗ ⇒ 前缀**静默失效**、**零报错** ✗。
#   ★ 实测（各自**独立** shell，避免互相污染 ✓）：`VAR=z true && env|grep -c z` ⇒ **0** ✗；
#     `VAR=z env|grep -c z` ⇒ 2 ✓；`export VAR=z; true && env|grep -c z` ⇒ 2 ✓。
#   ⇒ ★ 本文件原来三处都是**前缀** ✗ ⇒ 命令是**单条**时侥幸对 ✓、是**复合**（`a && b && c`）时
#     **只有第一条拿得到** ✗ —— 阶段八的就绪探针正是复合命令 ✗ ⇒ `python3 wait_ready.py` 拿不到分区
#     ⇒ 继承容器创建时的 `rtknav2` ✗ ⇒ **假"仿真器不在"** ⇒ 超时重试（§4 那个洞的真因 ✓）。
#   ⇒ ★ 已全改成 `export …; $*` ✓ ⇒ **任何命令形状都生效** ✓。
#   ★ 并留一条纪律：**注入手段必须让子进程能自报** ✓（`wait_ready.py` 的 `[probe-env]` ✓ 今晚加的 ✓，
#     正是它把这条揪出来的 ✓ —— 否则"静默失效"永远看不见 ✗）。
# ★★ 这是个**查询** ⇒ ★ **永远返回 0** ✓（2026-10-10 移植到本机时修）
#   病：原写法最后一句是 `[ -n "${_r}" ] && printf …` ✗ ⇒ **没有 `.current-run` 时它返回 1** ✗
#   ⇒ ★★ 而本文件第 11 行是 `set -euo pipefail`（**带 -e** ✓）⇒ **整个脚本被直接杀掉** ✗，
#      而且是**无声的** ✗（`start-campus.sh` 把这一步的输出丢进 /dev/null ⇒ 连 die 的消息都看不到 ✓）
#   触发条件：★ **`.current-run` 还不存在** ✓ —— 而这**恰恰是首次起栈的正常状态** ✓
#     （`ensure_workspace` 跑在 `set_current_run` **之前** ✓）⇒ ★ 本机第一次起栈必挂 ✓
#   ★ 定式：**查询类函数不要用 `A && B` 收尾** ✗ —— ★ 那会让"没找到"变成"报错" ✓
#     （★ 同族于"取不到 ≠ 缺失"✓：★ 这里更进一步 —— ★ **取不到还被当成了致命错误** ✗）
_round_gzp() {
    _r="$(current_run_id 2>/dev/null || true)"
    if [ -n "${_r}" ]; then printf 'rtknav2_%s' "${_r}"; fi
    return 0
}
cexec() {
    _g="$(_round_gzp)"
    if [ -n "${_g}" ]; then
        "${DOCKER_BIN}" exec "${CONTAINER_NAME}" bash -lc "export GZ_PARTITION='${_g}' IGN_PARTITION='${_g}'; $*"
    else
        "${DOCKER_BIN}" exec "${CONTAINER_NAME}" bash -lc "$*"
    fi
}

# 带超时的容器内命令：探测/注入这类应当很快返回的命令用它，避免无限期挂住
# ★ 同样注入本轮分区 ✓（reviewer ③″ 指出的那一族：**凡 cexec 系都要** ✓，只改 cexec/cexec_bg 会漏这条 ✓）
cexec_t() {
    local t="$1"; shift
    _g="$(_round_gzp)"
    if [ -n "${_g}" ]; then
        timeout "${t}" "${DOCKER_BIN}" exec "${CONTAINER_NAME}" bash -lc "export GZ_PARTITION='${_g}' IGN_PARTITION='${_g}'; $*"
    else
        timeout "${t}" "${DOCKER_BIN}" exec "${CONTAINER_NAME}" bash -lc "$*"
    fi
}

cexec_bg() {
    local logfile="$1"; shift
    # 注意：重定向发生在容器内，所以 logfile 必须是**容器路径**（/work/...）
    _g="$(_round_gzp)"
    if [ -n "${_g}" ]; then
        "${DOCKER_BIN}" exec -d "${CONTAINER_NAME}" bash -lc "export GZ_PARTITION='${_g}' IGN_PARTITION='${_g}'; $* > '${logfile}' 2>&1"
    else
        "${DOCKER_BIN}" exec -d "${CONTAINER_NAME}" bash -lc "$* > '${logfile}' 2>&1"
    fi
}

workspace_built() { "${DOCKER_BIN}" exec "${CONTAINER_NAME}" test -f "${WS_DIR}/install/setup.bash"; }

ensure_workspace() {
    if ! workspace_built; then
        log "首次运行，编译 rtk_nav2_ws 工作区"
    fi
    cexec "find ${WS_DIR}/src -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null; \
           source /opt/ros/humble/setup.bash && cd ${WS_DIR} && colcon build --symlink-install" | tail -3
    log "rtk_nav2_ws 已编译"
}

# ---------------------------------------------------------------- 栈归属（防"停掉别人的栈"）
#
# 为什么需要：这一类故障**连续两轮骗过人**——
#   第二阶段：reviewer 正在跑的巡视栈被 developer 的收尾脚本在几十秒内无声杀掉，那轮巡游数据作废；
#   第四阶段：上一次失败运行留下的 `start-demo.sh` 重试循环，把我刚起的栈的相机桥接 SIGINT 掉，
#   于是每一轮"节点不在/没数据"的观察都是**测量环境被自己上一轮扰动**的结果，
#   而我每次都先去怀疑代码。
# 规则：**起栈前若已有别的 `start-demo` 在跑就拒绝**（比"起栈前 pkill"稳——pkill 会误杀别人正常的重试）；
#       **重试循环只允许停"自己起的那个栈"**（用归属文件核对，对不上就停手并报出来）。
OWNER_FILE() { printf '%s/artifacts/rtk-nav2/.stack-owner' "${PROJECT_DIR}"; }

# 返回"别的 start-demo"的 PID（**排除自己和自己的所有祖先进程**），没有则空。
#
# **为什么必须排除祖先**：加固第一版只排除了 `$$`，结果**它自己被调用时反而拒绝启动**——
# 因为调用方的命令行（`bash -c "... scripts/rtk-nav2/start-demo.sh nav4-fix1 ..."`）里也含
# `start-demo.sh`，`pgrep -f` 把**父进程**当成了"另一个 start-demo"。
# 这是本轮"自匹配"的第五次变体（前四次分别在 grep、pkill 上）。教训：
# **凡是拿模式去匹配"进程列表"的检查，都要连自己的祖先一起排除。**
other_start_demo_pids() {
    # **判据必须只看"命令本身就是 bash 在跑 start-demo.sh"的进程。**
    # 我连着两轮起栈被自己这条检测拦住，原因是 pgrep -f 匹配的是**整条命令行**：
    # 用 `ssh 主机 'cd 项目 && scripts/rtk-nav2/start-demo.sh <id> 2>&1 | grep ...'` 调用时，
    # ① 外层 `bash -c` 的 argv 里含 "start-demo.sh"；② bash 为管道左边那种复合命令会
    # fork 一个 **子** subshell，它的 argv 也含那个字符串。②是 $$ 的**后代**而不是祖先，
    # 所以光走 PPID 链排除祖先漏掉它（祖先链只能排除①）。
    # 这里两道都上：排除自己整棵进程树（后代 + 祖先），并要求候选的命令行里
    # **不含 shell 元字符**（`|`&;<>()`）—— `bash -c '…'` 包装的调用一定带元字符。
    local anc=" $$ " desc=" $$ " p pid cpid cppid
    p="$$"
    while [ -n "${p}" ] && [ "${p}" != "0" ] && [ "${p}" != "1" ]; do
        anc="${anc}${p} "
        p="$(ps -o ppid= -p "${p}" 2>/dev/null | tr -d ' ')"
    done
    # 后代**只从 $$ 往下长**（祖先各自的兄弟/子女不算我的——一开始从祖先也扩散，
    # 结果把一个真正并发的 start-demo 当成自己的后代漏掉了，探针实测：假阴性）。
    local all changed=1
    all="$(ps -eo pid=,ppid= 2>/dev/null)"
    while [ "${changed}" -eq 1 ]; do
        changed=0
        while read -r cpid cppid; do
            case "${desc}" in
                *" ${cppid} "*) case "${desc}" in
                    *" ${cpid} "*) ;;
                    *) desc="${desc}${cpid} "; changed=1 ;;
                esac ;;
            esac
        done <<< "${all}"
    done

    local out="" args
    for pid in $(pgrep -f "[s]tart-demo\.sh" 2>/dev/null); do
        case "${anc}" in *" ${pid} "*) continue ;; esac
        case "${desc}" in *" ${pid} "*) continue ;; esac
        args="$(ps -o args= -p "${pid}" 2>/dev/null || true)"
        [ -n "${args}" ] || continue
        case "${args}" in
            *"|"*|*"&"*|*";"*|*"<"*|*">"*|*"("*|*")"*) continue ;;
        esac
        out="${out} ${pid}"
    done
    printf '%s' "${out}"
}

record_stack_owner() {
    mkdir -p "$(dirname "$(OWNER_FILE)")"
    printf '%s %s\n' "$1" "$(date +%s)" > "$(OWNER_FILE)"
}

stack_owner_run_id() { cut -d' ' -f1 "$(OWNER_FILE)" 2>/dev/null || true; }
clear_stack_owner() { rm -f "$(OWNER_FILE)"; }

# 只停"自己的栈"：归属对不上就**不停**，并把原因说出来（这正是那次该有的行为）
stop_stack_if_ours() {
    local run_id="$1" owner
    owner="$(stack_owner_run_id)"
    if [ -n "${owner}" ] && [ "${owner}" != "${run_id}" ]; then
        log "** 不停止栈：它属于 run ${owner}，不是本次的 ${run_id} **（避免把别人的运行无声杀掉）"
        return 1
    fi
    cexec "/work/scripts/rtk-nav2/container/stop_stack.sh '${run_id}' >/dev/null 2>&1 || true"
    return 0
}

stack_running() {
    # ★★ 2026-10-10 修正（reviewer 裁定，带两条）★★
    #   **症状**：栈**明明在跑**（`/clock` 572 Hz、30 节点、`ps` 里 21 个含 `launch` 的进程），
    #   这条守卫却判"没在运行" ✗ ⇒ `geo-route.sh` 被挡在门外。
    #   **根因**：`pgrep -f "ros2 launch robonix_rtk_nav2 demo.launch.py"` 匹配的是
    #   **argv 的整串前缀与顺序** ⇒ 起栈的调用方式一变就失配 ✗。
    #   ① **名字段**：只要求稳定的那一段 `demo.launch.py`（不再要求整串前缀 ✓）。
    #   ② ★ **加二级证据**：放宽匹配后"**误判成在跑**"的风险上升 ✗，而这条守卫正是
    #      **起栈/停栈的判据** ⇒ 再加一条"栈确实在动"的证据（`/clock` 有频率 ✓）——
    #      把"名字匹配"从**充分条件降为必要条件** ✓。
    #   ★ 进一步实测：**连"稳定的名字段"也测不到** ✗ —— 栈在跑时
    #     `pgrep -f "demo.launch.py"` = **0**（launch 父进程的 argv 里没有这个文件名；
    #     子节点只是碰巧因为 `/tmp/launch_params_*` 才含 "launch" 二字 ✗）。
    #   ⇒ 与其继续找一个"碰巧稳定的名字"，不如**换判据**：
    #   **主判据 = 仿真真的在推进**（`/clock` 有频率 ✓）。这比名字匹配**强**：
    #     名字是**代理量**、随调用方式漂移 ✗；`/clock` 是**直接证据** ✓（时间在走 = 栈在跑）。
    #   （reviewer 担心的"放宽后误判成在跑" ✗：`/clock` 有频率**不是**放宽，是换成了更强的证据 ✓。）
    #   ★★ 但**我换成 `/clock` 的版本实测不成立、已回退** ✗ ★★
    #   凭据（`campus8-h/guard_evidence.txt`）：**停栈之后**新判据仍返回 **true** ✗
    #   —— 正是 reviewer 提醒的"**误判成在跑**" ✓。根因：`stop-demo.sh` + 我的清理
    #   **杀不掉 `ros_gz_bridge`**（`parameter_bridge`）✗ ⇒ 它**继续发 `/clock`** ✗
    #   ⇒ "仿真在推进" ≠ "本栈在跑" ✗ —— **我把代理量换成了另一个代理量** ✗（`/clock` 也有别的发布者）。
    #   ⇒ 在找到"**与本栈生命周期绑定**"的判据之前，**先回退到原判据**（它只会**假阴性**、
    #     不会假阳性 ✓ —— 假阴性挡人、假阳性**放行一个坏栈**，后者危险得多 ✗）。
    #   ★★ 裁定（reviewer 2026-10-10）：**按"起栈时记下的 launch PID + lstart"判** ✓ ★★
    #   为什么这样才硬：名字会被 argv 顺序骗 ✗、`/clock` 有别的发布者（`ros_gz_bridge` 的
    #   `parameter_bridge` 杀不掉 ⇒ 停栈后它还在发 ✗）—— **两者都是代理量** ✗。
    #   而 pidfile 是**起栈那一刻自己记下的凭据** ✓，`lstart` 挡住 PID 复用 ✓。
    local _rid _pf _pid _lst _want
    _rid="$(current_run_id)"
    [ -n "${_rid}" ] || return 1
    _pf="${PROJECT_DIR}/artifacts/rtk-nav2/${_rid}/logs/demo_launch.pid"
    [ -s "${_pf}" ] || return 1
    _pid="$(cat "${_pf}" 2>/dev/null)"
    [ -n "${_pid}" ] || return 1
    docker exec "${CONTAINER_NAME}" bash -lc "[ -d /proc/${_pid} ]" >/dev/null 2>&1 || return 1
    _lst="$(docker exec "${CONTAINER_NAME}" bash -lc "ps -o lstart= -p ${_pid} 2>/dev/null" \
            | sed 's/^ *//;s/ *$//')"
    _want="$(cat "${_pf%.pid}.lstart" 2>/dev/null | sed 's/^ *//;s/ *$//')"
    [ -n "${_lst}" ] && [ -n "${_want}" ] && [ "${_lst}" = "${_want}" ]
}

# ★★ 判据拆分（reviewer 2026-10-10）：**别用一个布尔概括两种状态** ✗ ★★
#   旧守卫 `pgrep -f "ros2 launch …"` 返回 0 **不是错** —— 它抓到的是
#   "**launch 父进程不在**"这个**真事实** ✓，只是把它汇报成了"栈没在跑" ✗。
#   ⇒ 拆成两个**分开问、分开报**的事实：
#     ① **`launch` 在不在**（PID + lstart ✓）—— "起栈这套东西"的生死 ✓
#        ⇒ **能不能开下一个栈**看它 ✓；
#     ② **该在的节点齐不齐**（4 个关键节点 ✓）—— 栈的**完整度** ✓
#        ⇒ **这一轮算不算数**看它 ✓。
#   ★ 好处：**"半塌"会当场显示成"① 在、② 缺 3 个"** ✗，而不是含混成一句"在跑/没跑" ✓。
STACK_KEY_NODES="/planner_server /global_costmap/global_costmap /controller_server /lifecycle_manager_nav2"

stack_launch_alive() {
    local _rid _pf _pid _lst _want
    _rid="$(current_run_id)"; [ -n "${_rid}" ] || return 1
    _pf="${PROJECT_DIR}/artifacts/rtk-nav2/${_rid}/logs/demo_launch.pid"
    [ -s "${_pf}" ] || return 1
    _pid="$(cat "${_pf}" 2>/dev/null)"; [ -n "${_pid}" ] || return 1
    docker exec "${CONTAINER_NAME}" bash -lc "[ -d /proc/${_pid} ]" >/dev/null 2>&1 || return 1
    _lst="$(docker exec "${CONTAINER_NAME}" bash -lc "ps -o lstart= -p ${_pid} 2>/dev/null" \
            | sed 's/^ *//;s/ *$//')"
    _want="$(cat "${_pf%.pid}.lstart" 2>/dev/null | sed 's/^ *//;s/ *$//')"
    [ -n "${_lst}" ] && [ -n "${_want}" ] && [ "${_lst}" = "${_want}" ]
}

# 缺失的关键节点（空 = 齐）；★ 取不到清单时报 **UNKNOWN**，**不假装"齐"** ✗
stack_missing_nodes() {
    local _f="/tmp/_stack_nodes.$$" _m=""
    docker exec "${CONTAINER_NAME}" bash -lc \
        "source /opt/ros/humble/setup.bash; source ${WS_DIR}/install/setup.bash; \
         timeout 25 ros2 node list 2>/dev/null" > "${_f}" 2>/dev/null
    if [ ! -s "${_f}" ]; then echo "UNKNOWN(取不到节点清单 ✗)"; rm -f "${_f}"; return 0; fi
    for _n in ${STACK_KEY_NODES}; do grep -qx "${_n}" "${_f}" || _m="${_m} ${_n}"; done
    rm -f "${_f}"
    echo "${_m}"
}

stack_report() {
    printf '  ① launch 进程：%s\n' "$(stack_launch_alive && echo '在 ✓' || echo '不在 ✗')"
    local _m; _m="$(stack_missing_nodes)"
    if [ -z "${_m}" ]; then printf '  ② 关键节点  ：齐 ✓\n'
    else printf '  ② 关键节点  ：缺%s ✗\n' "${_m}"; fi
}

# 兼容旧调用：**① ∧ ②** 才算"栈可用"（两者都过才为真 ✓）
stack_running() { stack_launch_alive && [ -z "$(stack_missing_nodes)" ]; }

current_run_id() {
    [ -f "${PROJECT_DIR}/artifacts/rtk-nav2/.current-run" ] && cat "${PROJECT_DIR}/artifacts/rtk-nav2/.current-run"
}

set_current_run() {
    mkdir -p "${PROJECT_DIR}/artifacts/rtk-nav2"
    printf '%s' "$1" > "${PROJECT_DIR}/artifacts/rtk-nav2/.current-run"
}

resolve_run_id() {
    local run_id="${1:-}"
    [ -z "${run_id}" ] && run_id="$(current_run_id)"
    [ -z "${run_id}" ] && run_id="$(date '+%Y%m%d-%H%M%S')"
    printf '%s' "${run_id}"
}

artifact_dir() { printf '%s/artifacts/rtk-nav2/%s' "${PROJECT_DIR}" "$1"; }
# 同一份产物在**容器内**的路径。项目根在容器里挂成 /work，所以两者不同，
# 把宿主路径直接塞进 cexec 的命令里会指向一个不存在的目录（第三阶段实测踩到）。
artifact_dir_c() { printf '%s/artifacts/rtk-nav2/%s' "${CONTAINER_WORKDIR}" "$1"; }

# ★★ 2026-10-11 实测：★ 在**本机的挂载上 `chown` 根本是 no-op** ✗ ★★
#   ★ 探针 ✓：★ 造一个文件 ⇒ ★ 用**容器（root）** `chown 1:1` ⇒ ★ 属主**仍是 `1000:1000`** ✗
#     ★ 因为 `/mnt/c` 是 **DrvFs** ✓、★ 默认**不启用 metadata** ⇒ ★ 所有文件都报宿主 uid ✓
#   ⇒ ★ 所以这函数**在本机从来没起过作用** ✗ —— ★ 它不是在"交还属主"，★ 是纯开销 ✓
#     （★ 实测：★ 每次退出 **72 s** ✓、★ 大树那轮 **193 s** ✓；★ 入口那一次＝起栈前段 **77 s** ✓）
#   ★★ **但换机／★ 开了 metadata ⇒ `chown` 会生效** ✓（★ 例：★ 6002 那份**不是这个挂载** ✓）
#     ⇒ ★ 所以 **1) 探测挂载** ✓ ＋ ★ **2) `--all` 保底** ✓ 两条都留着 ✓。
#
# ★★ 2026-10-11 收窄范围 ✓（★ reviewer 派的 ✓，★ 先测再改 ✓）★★
#   ★ 原来：`chown -R` **整棵** `artifacts/rtk-nav2`（★ 实测已 **58 GB** ✓）
#     ⇒ ★ 每个 source 本文件的脚本**退出时**多花 **72 s** ✗（★ 实测 ✓；★ 本轮起栈那次 **193 s** ✓）
#     ⇒ ★ **入口也调一次**（`start-demo.sh:50` ✓）⇒ ★ 起栈**前段又 77 s** ✗ ⇒ ★ 两个洞**同源** ✓
#   ★ 现在：★ 只交还 ★ **本轮目录** ＋ ★ **顶层那几个共享文件** ✓
#     ★ 先测过 ✓：★ 一轮写到 run 目录**之外**的**只有** `artifacts/rtk-nav2/.current-run` ✓（★ 实测 ✓）
#   ★★ 两头判据 ✓：★ ① 退出耗时**大降** ✓；★ ② ★ **宿主仍能直接写产物** ✓（★ 属主交还**没坏** ✓）
#     ⇒ ★ **② 坏 ⇒ 退回整棵** ✓（★ 正确性优先 ✗ 不许为省时间把属主交还弄坏 ✗）
#   ★ 需要整棵时（★ 跨轮清理类脚本 ✓）：★ 传 `--all` ✓。
# ★ 探测本挂载上 chown 生不生效 ✓（★ no-op ⇒ **整体跳过** ✓ —— ★ 本机连那次 docker run 都省 ✓）
_fs_chown_effective() {
    local d="${PROJECT_DIR}/artifacts/rtk-nav2"
    mkdir -p "${d}" || return 1
    local probe="${d}/.chown_probe_$$"
    : > "${probe}" 2>/dev/null || return 1
    local b a
    b="$(stat -c '%u:%g' "${probe}" 2>/dev/null)"
    docker run --rm -v "${d}:/a" alpine:3.22.0 chown 1:1 "/a/$(basename "${probe}")" >/dev/null 2>&1 || true
    a="$(stat -c '%u:%g' "${probe}" 2>/dev/null)"
    if [ "${b}" != "${a}" ]; then
        docker run --rm -v "${d}:/a" alpine:3.22.0 chown "${b%:*}:${b#*:}" "/a/$(basename "${probe}")" >/dev/null 2>&1 || true
        rm -f "${probe}" 2>/dev/null || true
        return 0
    fi
    rm -f "${probe}" 2>/dev/null || true
    return 1
}

fix_artifacts_ownership() {
    local root="${PROJECT_DIR}/artifacts/rtk-nav2"
    local uid_gid; uid_gid="$(id -u):$(id -g)"
    # ★ 挂载探测：★ 本机 no-op ⇒ 直接返回 ✓（★ 省掉那次 docker run ✓）
    if ! _fs_chown_effective; then
        mkdir -p "${root}" 2>/dev/null || true
        return 0
    fi
    mkdir -p "${root}"
    local inner="/a"
    if [ "${1:-}" != "--all" ]; then
        local rid="${RUN_ID:-}"
        [ -z "${rid}" ] && rid="$(current_run_id 2>/dev/null || true)"
        inner=".current-run"
        [ -n "${rid}" ] && [ -d "${root}/${rid}" ] && inner="${rid} ${inner}"
    fi
    docker run --rm -v "${root}:/a" alpine:3.22.0 \
        sh -c "chown -R ${uid_gid} ${inner}" >/dev/null 2>&1 || true
}

ensure_run_dirs() {
    local run_id="$1"
    mkdir -p "$(artifact_dir "${run_id}")/logs" \
             "$(artifact_dir "${run_id}")/screenshots" \
             "$(artifact_dir "${run_id}")/bag" \
             "$(artifact_dir "${run_id}")/nav2" \
             "$(artifact_dir "${run_id}")/coverage" \
             "$(artifact_dir "${run_id}")/perf"
}

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
        log "解锁远程桌面会话 ${sid} 以便显示与截图"
        loginctl unlock-session "${sid}" 2>/dev/null || true
        sleep 2
    fi
}

trap 'fix_artifacts_ownership >/dev/null 2>&1 || true' EXIT

# 抓屏。每一步都有超时：X 卡住时 xdpyinfo/ffmpeg 会永久阻塞（实测过一次，
# 整轮测试卡死在截图这一步）。
capture_screen() {
    local outfile="$1"
    local display="${DEMO_DISPLAY}.0"
    command -v ffmpeg >/dev/null 2>&1 || { log "无 ffmpeg，跳过截图"; return 1; }
    unblank_display
    local size
    size="$(timeout 15 env DISPLAY="${display}" XAUTHORITY="$(detect_xauthority)" xdpyinfo 2>/dev/null \
            | awk '/dimensions:/ {print $2; exit}')"
    if ! timeout 20 env DISPLAY="${display}" XAUTHORITY="$(detect_xauthority)" \
            ffmpeg -hide_banner -loglevel error \
            -f x11grab -video_size "${size:-1280x800}" -i "${display}" \
            -frames:v 1 -y "${outfile}"; then
        log "截图失败（X 显示 ${display} 无响应？）：${outfile}"
        return 1
    fi
}

# ------------------------------------------------------------------ 留痕
#
# 把"这次运行是在什么配置/注入下得到的"写进产物。
# 为什么必须落到产物里、而不是写在报告里：第二阶段那一轮 `wheel_odom_bridge` 一改参数就崩，
# "基准跑"与"劣化跑"其实是同一个配置，而**从产物完全看不出来**。
# ★★ 2026-10-12（reviewer ✓）：★ "容器干净"这道闸**必须读两层** ✗ —— ★ 两层各有一个说谎的方式 ✓
#   ★ ① ROS 层（`ros2 daemon stop` 之后再量 `ros2 node list` ✓）：★ 会**报陈旧节点** ✗
#     —— ★ 进程都已 KILL 了、它还列着 ✓（今天实测：`lifecycle_manager` 被 TERM 后仍被列出 ✓）。
#   ★ ② 进程层（`ps -eo comm` ✓）：★ 兜住"活着但 `node list` 读空" ✗（★ 本项目的老坑 ✓）。
#   ★ ⇒ ★★ **两层都空才放行** ✓；★ 而 ★ 拒的时候**必须写出是哪一层脏** ✓（`dirty: process=N node=M` ✓）
#     —— ★ 今天出现过"ROS 层说活、进程层说死" ✓ ⇒ ★ 不写层名，下一个人还得再查一遍 ✗。
#   ★ ★ 判据两头 ✓（reviewer ✓）：★ ① 陈旧 daemon 在场（真进程已 KILL、`node list` 仍报 ✓）
#     ⇒ ★ **放行** ✓（★ 以**进程层**为准 ✓ —— 否则会被永远拒之门外 ✗，今天就拒了我三次 ✓）；
#     ★ ② 有活着但 `node list` 读空的 ✓ ⇒ ★ **拒** ✓（★ 进程层兜住 ✓）。
#   ★ ★ 实现上：★ **进程层是决定者** ✓、★ ROS 层**只记录** ✓（★ 因为它会撒谎 ✓）。
container_clean_or_die() {
    local why="${1:-起栈前必须干净}"
    cexec "source /opt/ros/humble/setup.bash && ros2 daemon stop" >/dev/null 2>&1 || true
    local nodes nproc
    nodes="$(cexec "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && timeout 20 ros2 node list" 2>/dev/null | tr -d '\r' | grep -v '^$' || true)"
    nproc="$(cexec "ps -eo comm --no-headers 2>/dev/null | grep -cxE 'ruby|rviz2|lifecycle_manager|ros2|ign|gz'" 2>/dev/null | tr -d '\r ' || true)"
    nproc="${nproc:-0}"
    local nnode=0
    [ -n "${nodes}" ] && nnode="$(printf '%s\n' "${nodes}" | grep -c . )"
    if [ "${nproc}" -ge 1 ]; then
        die "${why}：★ **进程层脏**（dirty: process=${nproc} node=${nnode}）⇒ 真进程还在跑 ✗
$(printf '%s\n' "${nodes}" | sed 's/^/    ROS 层还报：/')
  ⇒ 先 scripts/rtk-nav2/stop-demo.sh；★ 若它扛住 SIGTERM（lifecycle_manager 实测如此 ✗）⇒ 按 PID 核过 cmdline 后 **KILL** ✓"
    fi
    if [ "${nnode}" -ge 1 ]; then
        log "★ 干净闸：★ **进程层空 ✓、ROS 层还报 ${nnode} 个** ✓（dirty: process=0 node=${nnode}）⇒ ★ 判**放行** ✓（旧 daemon／陈旧节点表 ✓ —— 以进程层为准 ✓）"
    else
        log "★ 干净闸：★ 两层都空 ✓（dirty: process=0 node=0）"
    fi
}

record_state() {
    local run_id="$1" key="$2" value="$3"
    local dir
    dir="$(artifact_dir "${run_id}")"
    mkdir -p "${dir}"
    printf '%s=%s   # %s\n' "${key}" "${value}" "$(date '+%H:%M:%S')" >> "${dir}/run_state.txt"
}

# 标记一次注入（杀节点 / 劣化里程计 / 断流 / 放障碍 / 原地旋转 ...），
# **并把 run_kind 改写为"注入运行"**：注入类运行不带启动参数，光看 launch_args
# 会被推导成"主线（干净启动）"——那样一个被污染的结果在产物里反而长得像干净的主线结果。
mark_injection() {
    local run_id="$1" desc="$2"
    local dir f
    dir="$(artifact_dir "${run_id}")"
    mkdir -p "${dir}"
    f="${dir}/run_state.txt"
    record_state "${run_id}" "injection" "${desc}"
    if [ -f "${f}" ]; then
        if grep -q '^run_kind=' "${f}"; then
            sed -i "s|^run_kind=.*|run_kind=注入运行（非主线结果：${desc}）|" "${f}"
        else
            printf 'run_kind=注入运行（非主线结果：%s）\n' "${desc}" >> "${f}"
        fi
    fi
}

# 镜像钉版：验收基准（brief §10）。镜像 ID 与 depth_image_proc 版本一起进产物，
# 复验时先对这两条——中途重建过镜像、或换了包版本，一眼就能看出来。
record_image_identity() {
    local run_id="$1"
    local img_id diag_ver
    img_id="$(docker image inspect "${IMAGE_NAME}" --format '{{.Id}}' 2>/dev/null || echo '(取不到)')"
    diag_ver="$(docker run --rm --entrypoint bash "${IMAGE_NAME}" \
                -c "dpkg-query -W -f='\${Version}' ros-humble-depth-image-proc 2>/dev/null" 2>/dev/null || echo '(取不到)')"
    record_state "${run_id}" "image" "${IMAGE_NAME}"
    record_state "${run_id}" "image_id" "${img_id}"
    record_state "${run_id}" "depth_image_proc_version" "${diag_ver}"
}

# 配置指纹：把这次运行**实际用到的**参数/场景/模型文件哈希写进 run_state.txt。
# 记"实际生效的东西"而不是手工键列表——手工列表在新增参数时必然漏。
# **nav2 的两张图参数在 nav2_params.yaml 里（brief §4 要求进指纹）**；
# 另外还会把运行中两张图的**实际参数**dump 进产物（比哈希更强：能看出文件与运行时不一致）。
record_config_fingerprint() {
    local run_id="$1"
    # ★ 2026-10-09（阶段八）：第二个参数 = 本次启动参数串。
    #   为什么必须传：下面的固定清单是**阶段三那套**（`nav2_params.yaml` / `rtk_courtyard.sdf` / …）
    #   ⇒ 阶段八真正在用的 `campus_100.sdf` 与 `campus_nav2.yaml` **不在里面** ✗
    #   ⇒ 判据 ⑤"世界与配置 sha256 要进记录"就落空了（reviewer 2026-10-09 抓到）。
    #   做法：从启动参数里抠出 `world:=` / `nav2_config:=` / `static_world:=` 三个**实际生效**的路径
    #   （容器视角 `/work/...` 换回宿主 `${PROJECT_DIR}/...`），按**全量 sha256**另记一组
    #   `config_used_*` 与 `config_fingerprint_sha256_full`。
    #   ★ 不动原来那组 16 位值：那是阶段三/七的既有锚点，改了会**破坏它们的可复跑对照**。
    #   ★ 另外更正一处口径（reviewer 顺手提到的）：原来那串是
    #     `sha256sum | cut -c1-16`，即 **sha256 的前 16 个十六进制字符**，**不是 md5** ——
    #     全项目本来就是 sha256，这一处只是**截断显示**。
    local launch_args="${2:-}"
    local files=(
        "${PKG_DIR}/config/nav2_params.yaml"
        "${PKG_DIR}/config/danger.yaml"
        "${PKG_DIR}/config/route.yaml"
        "${PKG_DIR}/config/navsat.yaml"
        "${PKG_DIR}/config/ekf_local.yaml"
        "${PKG_DIR}/worlds/rtk_courtyard.sdf"
        "${PKG_DIR}/models/robot_rtk_nav2.urdf.xacro"
    )
    local combined="" f h
    for f in "${files[@]}"; do
        [ -f "${f}" ] || continue
        h="$(sha256sum "${f}" 2>/dev/null | cut -c1-16)"
        record_state "${run_id}" "config_$(basename "${f}")" "${h}"
        combined="${combined}${h}"
    done
    record_state "${run_id}" "config_fingerprint" \
        "$(printf '%s' "${combined}" | sha256sum | cut -c1-16)"

    # ---- 阶段八：把**本次实际生效**的世界/配置按全量 sha256 另记一组 ----
    local used="" kmap=""
    local key path host
    # ★★ 2026-10-12 修（reviewer 批准 ✓）：去重标记**必须带上"这是第几次起栈"** ✗★
    #   原来的标记 `.cfg_seen_<key>` 落在 run-id 的产物目录里 ⇒ **同一个 run-id 第 2 次起栈时它还在**
    #   ⇒ 那一轮会 `continue` 掉**不记** ✗ ⇒ ★ ① 字段**静默缺失** ✗、★ ② 读者看到的那个值其实是
    #   **第 1 次起栈**的 ✗✗（★ 实测 2026-10-11 `campus100-look`：第 1 次指主树、第 2 次指
    #   `rtk_nav2_ws_campus9-b1` ⇒ 读到的是**错的那一套** ✗；`config_fingerprint_sha256_full`
    #   由下面的 `used` 串拼出 ⇒ **一起被污染** ✗）。
    #   ⇒ 标记改成 `.cfg_seen_s<起栈序号>_<key>` ✓ ⇒ **每次起栈都记全** ✓
    #     （★ 序号读的是 `start-demo.sh` 刚写下的 `.stack_start_count` ✓；★ 读不到时退 1 ✓）
    local SIDX
    SIDX="$(cat "${PROJECT_DIR}/artifacts/rtk-nav2/${run_id}/.stack_start_count" 2>/dev/null || echo 1)"
    for a in ${launch_args}; do
        case "$a" in
            world:=*|nav2_config:=*|static_world:=*)
                path="${a#*:=}"
                # 容器视角 -> 宿主视角（挂载是 PROJECT_DIR:/work）
                host="${path/#\/work/${PROJECT_DIR}}"
                [ -f "${host}" ] || { kmap="${kmap} ${a}（宿主侧找不到 ${host} ✗）"; continue; }
                key="$(basename "${host}")"
                if [ -f "${PROJECT_DIR}/artifacts/rtk-nav2/${run_id}/.cfg_seen_s${SIDX}_${key}" ]; then
                    # ★ `world` 与 `static_world` 通常**就是同一个文件**（阶段八就是这么传的）
                    #   ⇒ 不去重会在 `run_state.txt` 里出现两行同名同值（reviewer 2026-10-09 指出，
                    #     无害但是噪声）⇒ 记一次就够，并在第一次那行标注同源 ✓
                    continue
                fi
                : > "${PROJECT_DIR}/artifacts/rtk-nav2/${run_id}/.cfg_seen_s${SIDX}_${key}"
                # ★★ 值里**只放裸 sha**；"同源"那句挪到 `#` 之后的**注释行** ★★
                #   来历（reviewer 2026-10-09）：我原来把注记**写进值里**（`sha（world/static_world 同源…）`）
                #   ⇒ 任何按 `key=value` 解析这一行的消费者拿到的都是 `sha（…）` ✗，
                #   拿去比 sha **永远不等**（实测：两个值肉眼完全相同、判定却是 False ✗）。
                #   我一开始只在**读**的那一端剥括号 —— **那是只修一半**：坑仍然留给下一个消费者 ✗。
                #   **规则：给机器读的字段里不要塞给人看的注记** —— 这条要落在**写**上 ✓。
                record_state "${run_id}" "config_used_${key}" "$(sha256sum "${host}" | cut -d' ' -f1)"
                # ★★ 2026-10-12（reviewer 要求 ✓）：把**该文件的 mtime** 也记进产物 ✓
                #   为什么：事后判「跑的时候是不是这一份」时，原来**只能靠"文件现在没变 + mtime 早于本轮"去推** ✗
                #   ⇒ 产物里直接有 mtime ⇒ **下一个人不用推** ✓。
                #   ★ 另记**独立字段** ✗ **不塞进上面那个值里** ✗ —— 机器读的字段里不塞注记（本项目已有此规 ✓）。
                record_state "${run_id}" "config_used_${key}_mtime" \
                    "$(stat -c %y "${host}" 2>/dev/null | cut -c1-19)"
                printf '#   （%s 与 static_world 同源 ⇒ 两个启动参数指向同一个文件，故只记一次）\n' \
                    "${key}" >> "$(artifact_dir "${run_id}")/run_state.txt"
                used="${used}$(sha256sum "${host}" | cut -d' ' -f1)"
                ;;
        esac
    done
    if [ -n "${used}" ]; then
        record_state "${run_id}" "config_fingerprint_sha256_full" \
            "$(printf '%s' "${used}" | sha256sum | cut -d' ' -f1)"
    fi
    # 生成器也进记录（世界是它产出的 ⇒ "世界对不对"要能追到"哪一版生成器产的"）
    # ★ 同时记 **几何指纹**（reviewer 2026-10-09 裁定 (b)）：障碍清单排序后的 sha256。
    #   标定（`geo_calib.json`）与 `geo_route` 的启动校验**要用它、不用世界文件哈希** ——
    #   否则"只加了 `<gui><camera>` 机位"（零几何）也会被判成"标定失效" ✗。
    if [ -f "${PKG_DIR}/scripts/make_campus_world.py" ]; then
        record_state "${run_id}" "config_used_make_campus_world.py" \
            "$(sha256sum "${PKG_DIR}/scripts/make_campus_world.py" | cut -d' ' -f1)"
        local gp
        # ★★ reviewer 2026-10-09：**一并记 `world_matches_model`** ★★
        #   为什么：几何指纹是**从生成器模型**算的，而这一轮**实际跑的是盘上那份世界**
        #   ⇒ 两者不一致时（盘上是旧模型产物 / 被手改过），记录里的指纹描述的是**另一份世界** ✗
        #   —— "记录/校验的对象不是实际在跑的那一份"是最难查的一类（与"在跑的是哪一份代码"同族）。
        #   ⇒ 记一行 `world_matches_model=true/false`；**交付报告里这一项必须为 true** ✓。
        local gout gp wm
        gout="$(python3 "${PKG_DIR}/scripts/make_campus_world.py" --check-only 2>/dev/null || true)"
        gp="$(printf '%s' "${gout}" | python3 -c 'import json,sys; print(json.load(sys.stdin)["geometry_fingerprint"])' 2>/dev/null || true)"
        if [ -n "${gp}" ]; then
            record_state "${run_id}" "geometry_fingerprint" "${gp}"
        fi
        wm="$(printf '%s' "${gout}" | python3 -c 'import json,sys; print(str(json.load(sys.stdin).get("disk_world_matches_model")).lower())' 2>/dev/null || true)"
        if [ -n "${wm}" ]; then
            record_state "${run_id}" "world_matches_model" "${wm}"
        fi
    fi
    # 静态层的三项栅格参数（它们必须是"与全局图逐字一致"的那一组，写进记录才能事后核）
    local sp=""
    for a in ${launch_args}; do
        case "$a" in
            static_resolution:=*|static_size:=*|static_origin:=*|static_topic:=*) sp="${sp} $a" ;;
        esac
    done
    # ★ **这两个必须写成 if、不能写成 `[ ... ] && cmd`**：函数体最后一条若是
    #   `[ -n "" ] && ...`，条件为假时**整条 list 返回 1** ⇒ 调用点那一行是个简单命令
    #   ⇒ `set -e` 在**调用方**触发 ⇒ 起栈脚本**静默中止**（实测：`campus8-d` 只写到
    #   `static_layer_args` 就没了、`run_kind` 整段没执行，而 ssh 那层的退出码被管道吃成 0，
    #   看起来像"什么都没发生"）✗。教训与"判据判事件"同族：**返回码也要看是不是它的**。
    if [ -n "${sp// /}" ]; then
        record_state "${run_id}" "static_layer_args" "${sp# }"
    fi
    if [ -n "${kmap// /}" ]; then
        record_state "${run_id}" "config_used_warning" "${kmap# }"
    fi
    return 0
}

# 按模式在容器内杀进程，并回显命中/剩余 PID 作为证据。
# 模式**必须**用 [x] 括号写法：pkill -f 会匹配到自己的父 shell
# （docker exec 起的 `bash -lc "pkill -f <模式>"` 命令行里就含这个字面模式；
#  第三阶段清第二阶段孤儿进程时实测踩到：pkill 把自己那条 shell 杀了，脚本后半段没跑完）。
kill_pattern() {
    local pattern="$1"
    local pids left p
    pids="$(cexec "pgrep -f '${pattern}' || true" 2>/dev/null | tr -d '\r' | grep -E '^[0-9]+$' | tr '\n' ' ' || true)"
    log "  命中 ${pattern} 的 PID：${pids:-（无）}"
    for p in ${pids}; do
        cexec "kill -9 ${p} >/dev/null 2>&1 || true" >/dev/null 2>&1 || true
    done
    sleep 2
    left="$(cexec "pgrep -f '${pattern}' || true" 2>/dev/null | tr -d '\r' | grep -cE '^[0-9]+$' || true)"
    log "  剩余 ${pattern} 进程数：${left:-0}"
    printf '%s' "${pids}"
}
