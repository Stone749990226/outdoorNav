#!/usr/bin/env bash
# 目标机预检：在**目标机本机**执行（不是宿主这边），把所有会影响复现的事实打出来。
#
# 用法（在目标机上）：
#   bash scripts/preflight.sh [输出文件]
#
# 输出用于填写目标机自己的 docs/environment.md，不要照抄其它机器的结论。

set -o pipefail
OUT="${1:-}"

emit() { printf '%s\n' "$*"; }

report() {
emit "===== 目标机预检 ====="
emit "采集时间：$(date '+%Y-%m-%d %H:%M:%S %Z')"
emit "主机名：$(hostname)"
emit "当前用户：$(id -un) (uid=$(id -u), groups=$(id -Gn))"
emit

emit "----- 操作系统 / 内核 -----"
emit "$(head -2 /etc/os-release 2>/dev/null | tr '\n' ' ')"
emit "内核：$(uname -r) $(uname -m)"
emit

emit "----- 资源 -----"
emit "$(free -h 2>/dev/null | head -2 | tr '\n' '|')"
emit "磁盘：$(df -h / 2>/dev/null | tail -1)"
emit "CPU：$(nproc) 核"
emit

emit "----- GPU / 驱动 -----"
if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>&1 | sed 's/^/  /'
else
    emit "  无 nvidia-smi"
fi
lspci 2>/dev/null | grep -iE 'vga|3d|display' | sed 's/^/  /' || emit "  lspci 不可用"
emit

emit "----- Docker -----"
if command -v docker >/dev/null 2>&1; then
    emit "  版本：$(docker --version 2>&1)"
    if docker info >/dev/null 2>&1; then
        emit "  当前用户可访问 docker：是"
        emit "  存储驱动：$(docker info 2>/dev/null | grep -i 'storage driver' | head -1 | cut -d: -f2- | xargs)"
        emit "  运行时：$(docker info 2>/dev/null | grep -i '^ Runtimes' | cut -d: -f2- | xargs)"
    else
        emit "  当前用户可访问 docker：否（需要加入 docker 组，或提供 sudo 密码）"
    fi
else
    emit "  未安装 docker"
fi
emit "  sudo 免密：$(sudo -n true 2>/dev/null && echo 是 || echo 否)"
emit

emit "----- 宿主 ROS -----"
if [ -d /opt/ros ]; then emit "  /opt/ros: $(ls /opt/ros | tr '\n' ' ')"; else emit "  未安装（/opt/ros 不存在）"; fi
emit

emit "----- 图形会话 -----"
emit "  X socket：$(ls /tmp/.X11-unix/ 2>/dev/null | tr '\n' ' ')"
emit "  DISPLAY 环境：${DISPLAY:-（未设置）}"
emit "  监听端口（远程桌面）："
(ss -tlnp 2>/dev/null || netstat -tlnp 2>/dev/null) | grep -E '5900|5901|3389' | sed 's/^/    /' || emit "    未发现 3389/5900"
emit "  登录会话："
loginctl list-sessions --no-legend 2>/dev/null | sed 's/^/    /' || emit "    loginctl 不可用"
emit "  当前用户的会话锁定状态："
if command -v loginctl >/dev/null 2>&1; then
    for s in $(loginctl list-sessions --no-legend 2>/dev/null | awk -v u="$(id -u)" '$2==u {print $1}'); do
        emit "    $s: LockedHint=$(loginctl show-session "$s" -p LockedHint --value 2>/dev/null) Active=$(loginctl show-session "$s" -p Active --value 2>/dev/null)"
    done
fi
emit

emit "----- 工具 -----"
for c in ffmpeg rsync python3 xauth; do
    printf '  %s: ' "$c"
    command -v "$c" >/dev/null 2>&1 && echo "$(command -v "$c")" || echo "缺失"
done
emit

emit "----- 网络 -----"
for url in https://github.com https://packages.ros.org/ros2/ubuntu/dists/jammy/Release; do
    printf '  %s -> ' "$url"
    curl -sS -o /dev/null -m 15 -w '%{http_code}\n' "$url" 2>&1 | tail -1
done
emit "  apt 源："
grep -rhE '^deb ' /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null | head -8 | sed 's/^/    /'
emit

emit "----- 已有镜像 / 容器 -----"
if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
    emit "  镜像（与本项目相关的）："
    docker images --format '{{.Repository}}:{{.Tag}} {{.Size}}' 2>/dev/null \
        | grep -iE 'ros|gazebo|robonix|slam|osrf' | sed 's/^/    /' || emit "    （无）"
    emit "  运行中的容器："
    docker ps --format '{{.Names}} {{.Image}} {{.Status}}' 2>/dev/null | sed 's/^/    /' || emit "    （无）"
    emit "  是否已存在本项目容器名 robonix-slam-demo："
    docker ps -a --filter name=robonix-slam-demo --format '    {{.Names}} {{.Status}}' 2>/dev/null
fi
emit
emit "===== 预检结束 ====="
}

if [ -n "${OUT}" ]; then
    report | tee "${OUT}"
else
    report
fi
