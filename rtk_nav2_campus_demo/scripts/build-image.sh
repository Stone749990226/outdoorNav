#!/usr/bin/env bash
# 构建演示用的容器镜像 robonix/slam-demo:humble。
#
# 基础镜像优先用本机已有的 ROS 2 Humble Desktop Full 副本；找不到时按下面的顺序
# 依次尝试，并把最终选中的基础镜像打印出来（换机器时这一步的结论会不同）：
#   1. robonix-osrf-ros:humble-desktop-full   （原机器上已有的本地副本）
#   2. osrf/ros:humble-desktop-full           （官方镜像，可直接用）
#   3. 用 docker pull 拉取 osrf/ros:humble-desktop-full
# 都不行时会明确报错，不会去改宿主环境。
#
# 用法：scripts/build-image.sh [--base <镜像>]
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

BASE_IMAGE_OVERRIDE=""
while [ $# -gt 0 ]; do
    case "$1" in
        --base) BASE_IMAGE_OVERRIDE="${2:?--base 需要参数}"; shift 2 ;;
        -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
        *) die "未知参数 $1" ;;
    esac
done

require_docker

# ★ 按【Gazebo 路线】选 Dockerfile（✗ 不再按架构 ✗ —— 理由见 scripts/lib.sh 的注释）
#   ★ 旧写法在 x86 上会选到 Gazebo Classic 11 ✗，而本项目全部成果都是 Fortress ✓。
case "${GAZEBO_ROUTE}" in
    fortress)
        case "$(uname -m)" in
            aarch64|arm64) DOCKERFILE="${PROJECT_DIR}/docker/Dockerfile.arm64" ;;
            *)             DOCKERFILE="${PROJECT_DIR}/docker/Dockerfile.amd64" ;;
        esac
        log "路线 ${GAZEBO_ROUTE} ／ 架构 $(uname -m)：使用 ${DOCKERFILE##*/}（Gazebo Sim / Fortress）"
        ;;
    *)
        DOCKERFILE="${PROJECT_DIR}/docker/Dockerfile"
        log "路线 ${GAZEBO_ROUTE}：使用 docker/Dockerfile（Gazebo Classic 11）"
        ;;
esac
[ -f "${DOCKERFILE}" ] || die "找不到 ${DOCKERFILE}"

image_exists() {
    docker image inspect "$1" >/dev/null 2>&1
}

pick_base_image() {
    if [ -n "${BASE_IMAGE_OVERRIDE}" ]; then
        image_exists "${BASE_IMAGE_OVERRIDE}" || die "指定的基础镜像 ${BASE_IMAGE_OVERRIDE} 在本机不存在"
        echo "${BASE_IMAGE_OVERRIDE}"; return 0
    fi
    # ★ 走 Fortress 路线（★ 不分架构 ✓）就不依赖现成 ROS 镜像：直接在 ubuntu:22.04 上装官方 apt 包
    if [ "${GAZEBO_ROUTE}" = "fortress" ]; then
        if image_exists "ubuntu:22.04"; then
            echo "ubuntu:22.04"; return 0
        fi
        log "本机没有 ubuntu:22.04，尝试拉取" >&2
        if docker pull ubuntu:22.04 >&2; then echo "ubuntu:22.04"; return 0; fi
        return 1
    fi
    if image_exists "robonix-osrf-ros:humble-desktop-full"; then
        echo "robonix-osrf-ros:humble-desktop-full"; return 0
    fi
    if image_exists "osrf/ros:humble-desktop-full"; then
        log "本机没有 robonix-osrf-ros:humble-desktop-full，改用已在的 osrf/ros:humble-desktop-full" >&2
        echo "osrf/ros:humble-desktop-full"; return 0
    fi
    log "本机没有 Humble 基础镜像，尝试拉取 osrf/ros:humble-desktop-full" >&2
    if docker pull osrf/ros:humble-desktop-full >&2; then
        echo "osrf/ros:humble-desktop-full"; return 0
    fi
    return 1
}

BASE_IMAGE="$(pick_base_image)" || die "找不到可用的 ROS 2 Humble 基础镜像。
请在能联网的机器上 docker pull osrf/ros:humble-desktop-full，
或把已有的同类镜像用 --base <镜像名> 传进来；不要为此修改宿主系统。"

# ★ Fortress 路线需要构建机上的 ROS keyring（★ 官方下发的 ros.key 已过期 ✓ —— 两条路线都要 ✓）
if [ "${GAZEBO_ROUTE}" = "fortress" ]; then
    KEY_SRC="/usr/share/keyrings/ros-archive-keyring.gpg"
    KEY_DST="${PROJECT_DIR}/docker/keys/ros-archive-keyring.gpg"
    mkdir -p "$(dirname "${KEY_DST}")"
    if [ -r "${KEY_SRC}" ]; then
        cp "${KEY_SRC}" "${KEY_DST}"
        log "已复制构建机的 ROS keyring：${KEY_SRC} -> docker/keys/"
    elif [ ! -s "${KEY_DST}" ]; then
        die "找不到 ROS 签名密钥：${KEY_SRC} 不存在，且 docker/keys/ros-archive-keyring.gpg 也没有。
请在有 ROS 源的机器上执行一次 apt 更新以获得最新的 keyring 后重试。"
    fi
fi

log "基础镜像：${BASE_IMAGE}"
log "构建镜像 ${IMAGE_NAME}"
docker build --build-arg "BASE_IMAGE=${BASE_IMAGE}" \
    -t "${IMAGE_NAME}" -f "${DOCKERFILE}" "${PROJECT_DIR}"

log "镜像信息："
docker run --rm "${IMAGE_NAME}" cat /build-info.txt
