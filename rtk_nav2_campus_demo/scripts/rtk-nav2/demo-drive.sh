#!/usr/bin/env bash
# ★★★ 演示版 · 走航线（**一条命令**）★★★
#
# ★★ **标注（2026-10-14 加 ✓）：★ 本脚本**不在当前演示流程里** ✗** ——
#   ★ 旧用法：按 `demo-data/` 里的**固定 41 点航线**走 ✓（★ 业主口径已改 ⇒ ★ 现在车由
#     **前端下发目标点**驱动 ✓ —— 见 `README.md` 的「怎么跑」✓：起栈 ⇒ `frontend-bridge.sh`
#     ⇒ 浏览器里选终点 ⇒ 停栈 ✓）。
#   ★ 它**仍然保留** ✓（旧流程与对照还要用它 ✓）；★ ✗ **别删 `demo-data/`** ✗ ——
#     起栈脚本的**出生点**就是从 `demo-data/waypoints_export.csv` **第 1 行**读的 ✓
#     （删了起栈就废 ✗）。
#
#   用法：  scripts/rtk-nav2/demo-drive.sh
#
# ★ 起栈那个终端**另开一个终端**跑本脚本 ✓（起栈是长驻前台 ✓）。
# ★ 固定项全在这里钉死 ⇒ 业主不用带参数 ✓：
#     · 环境变量（同起栈 ✓）· run-id=demo ✓ · 标定与航线取 demo-data/ 里的固定文件 ✓
#     · **--tol 0.40** ✓（业主裁定：到点容差放宽；只动这一个命令行参数 ✓）
# ★ 走完**自动抓图**是 geo-route.sh 内置的步骤 ✓（走完之后、收栈之前 ✓）⇒ 先看到抓图那几行再停栈 ✗
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_DATA="${HERE}/demo-data"

export CONTAINER_NAME="${CONTAINER_NAME:-${DEMO_CONTAINER:-rtknav2-demo}}"
export ROS_DOMAIN_ID="${DEMO_DOMAIN:-56}"
export GZ_PARTITION="${DEMO_PARTITION:-rtknav2demo}"
export RUN_ID="demo"

[ -f "${DEMO_DATA}/geo_calib.json" ] || { echo "✗ 缺固定标定 ${DEMO_DATA}/geo_calib.json"; exit 1; }
[ -f "${DEMO_DATA}/route_geo.json" ] || { echo "✗ 缺固定航线 ${DEMO_DATA}/route_geo.json"; exit 1; }

# ★ 幂等补齐"栈活性凭据"（2026-10-12：业主那次"启动后很久车都没动"就是这个 ✗）★
#   `geo-route.sh` 的闸要求 `logs/demo_launch.pid` **非空** ✓；它若丢了 ⇒ 判"栈没在运行" ⇒ 直接退出 ✗
#   ⇒ 这里在开跑前**按容器里活着的 `ros2 launch` 补一次** ✓（✗ 不猜 PID ✓）
TREE="$(cd "${HERE}/../.." && pwd)"
DEMO_DIR="${TREE}/artifacts/rtk-nav2/demo"
_lpid="$(docker exec "${CONTAINER_NAME}" bash -lc 'ps -eo pid,args' 2>/dev/null | /bin/grep -E 'ros2 launch' | /bin/grep -v grep | head -1 | awk '{print $1}')"
if [ -n "${_lpid}" ]; then
    mkdir -p "${DEMO_DIR}/logs"
    # ★★ 2026-10-12：**先删再写 ＋ 判产物** ✓ —— ★ 与 `demo-start.sh` 里那段**同形** ✓
    #   ★ 为什么两处都要 ✓：★ `logs/` 由**容器（root）先写** ⇒ ★ `>` 原地截断要**文件**写权限 ✗
    #     ⇒ ★ **Permission denied** ✗；★ 而 `rm -f` 走**目录**权限 ⇒ **能删** ✓（已实测 ✓）
    #   ★ 且原写法**不看写入结果** ⇒ ★ 在失败之上打 ✓ ✗（★ 与"发起 ≠ 做成"同族 ✓）
    #   ★ ⇒ ★ **改一处要两处一起改** ✗（★ 这正是本仓"同形处各自一份 ⇒ 容易漏"的老账 ✓）
    rm -f "${DEMO_DIR}/logs/demo_launch.pid" "${DEMO_DIR}/logs/demo_launch.lstart"
    _lstart="$(docker exec "${CONTAINER_NAME}" bash -lc "ps -o lstart= -p ${_lpid}" 2>/dev/null | sed 's/^ *//;s/ *$//')"
    printf '%s' "${_lpid}" > "${DEMO_DIR}/logs/demo_launch.pid" 2>/dev/null || true
    printf '%s' "${_lstart}" > "${DEMO_DIR}/logs/demo_launch.lstart" 2>/dev/null || true
    # ★ 判据落在**目标量**上 ✓：★ 要的是"**凭据 ＝ 容器里的真身**" ✗
    #   ★ ✗ 不能只判 `[ -s ]` ✗ —— ★ 若 `rm -f` 也失败（★ 目录不可写 ✓），
    #     **旧文件还在** ⇒ ★ `-s` 照样成立 ⇒ ★ 又会打假 ✓ ✗（★ 代理量 ✓ 不是目标量 ✗）
    if [ "$(cat "${DEMO_DIR}/logs/demo_launch.pid" 2>/dev/null)" = "${_lpid}" ]; then
        echo "★ 栈活性凭据已核（补/覆盖）：logs/demo_launch.pid = ${_lpid} ✓"
    else
        echo "✗ 栈活性凭据**没写成**：${DEMO_DIR}/logs/demo_launch.pid 的内容 ≠ 真身 ${_lpid} ✗" >&2
        echo "  ★ 三种成因都归这一句 ✓：★ **不存在** ✓／★ **空** ✓／★ **是旧值**（★ 实测过：★ 目录不可写 ⇒ `rm` 失败 ⇒ 旧文件留下 ✓）" >&2
        echo "  ⇒ geo-route.sh 会判「栈没在运行」并直接退出 ✗" >&2
        exit 1
    fi
else
    echo "✗ 容器 ${CONTAINER_NAME} 里没有 ros2 launch ⇒ 栈没起 ⇒ 先跑 demo-start.sh ✓"
    exit 1
fi

echo "★ 演示版走航线：轮=${RUN_ID} ｜ 标定与航线取自 demo-data/（✗ 不拷 ✓）｜ --tol 0.40 ✓"
# ★★ 路径必须是**容器视角**（2026-10-12 端到端验证抓到的 bug ✗）★★
#   `geo-route.sh` 会 `cexec "test -s '${CALIB}'"` —— ★ 那是在**容器里**测的 ✗
#   ⇒ 传宿主路径（/mnt/c/...）一定测不到 ✗ ⇒ 报"没有 geo_calib.json" ⇒ 直接退出 ✓
#   （业主那次"车很久不动"的第二层原因就是这个 ✓）
#   ⇒ 统一用容器视角：/work = 树的挂载点（lib.sh 的 CONTAINER_WORKDIR ✓）
exec bash "${HERE}/geo-route.sh" "${RUN_ID}" \
    --calib "/work/scripts/rtk-nav2/demo-data/geo_calib.json" \
    --route "/work/scripts/rtk-nav2/demo-data/route_geo.json" \
    --tol 0.40
