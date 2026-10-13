#!/usr/bin/env bash
# 放入/移除演示用的临时障碍物（**主机侧入口**；内部转发到容器里的实现）。
# 用法：scripts/rtk-nav2/spawn-obstacle.sh spawn <x> <y> [边长] [高] ｜ remove
# 说明：**演示当天只用这一条**（主机命令）—— 不要把容器内的
#       `container/spawn_obstacle.sh` 直接抄进现场清单（那是容器侧，直接跑会报
#       `/opt/ros/humble/setup.bash: 没有那个文件或目录`）。
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

if [ $# -eq 0 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    sed -n '2,4p' "$0"; exit 0
fi

# ★ docker 的可执行文件要**显式解析**：本仓库根下有一个 `docker/` 目录，
#   而有些 shell 的 PATH 里含 `.` ⇒ 直接写 `docker` 会被解析成**同名的目录**，
#   报 `exec: docker: cannot execute: Is a directory`（实测踩到）。
DOCKER_BIN="${DOCKER_BIN:-/usr/bin/docker}"
if [ ! -x "${DOCKER_BIN}" ]; then
    DOCKER_BIN="$(command -v docker 2>/dev/null || true)"
fi
if [ -z "${DOCKER_BIN}" ] || [ ! -x "${DOCKER_BIN}" ]; then
    echo "** 找不到可执行的 docker（试过 /usr/bin/docker 与 PATH）：${DOCKER_BIN:-<空>} **" >&2
    echo "   （提示：若报 'Is a directory'，说明 PATH 里的 '.' 命中了仓库根的 docker/ 目录）" >&2
    exit 127
fi

echo "[主机侧] 转发到容器 ${CONTAINER_NAME}：container/spawn_obstacle.sh $*"
exec "${DOCKER_BIN}" exec "${CONTAINER_NAME}" bash -lc \
    "/work/scripts/rtk-nav2/container/spawn_obstacle.sh $(printf '%q ' "$@")"
