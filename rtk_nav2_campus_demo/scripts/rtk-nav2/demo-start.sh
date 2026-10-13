#!/usr/bin/env bash
# ★★★ 演示版 · 起栈（**一条命令，什么都不用带**）★★★
#
#   用法：  scripts/rtk-nav2/demo-start.sh
#
# ★ 与按轮脚本（start-northcampus.sh 等）的关系：本脚本是**薄包装** ✓
#   它把"每次都要手敲的那些东西"钉死在这里 ⇒ 业主只敲一条命令 ✓：
#     · 环境变量（CONTAINER_NAME / ROS_DOMAIN_ID / GZ_PARTITION）—— 内部定 ✓
#     · run-id —— 固定为 demo ✓（✗ 不再按次分目录 ✓）
#     · 输入文件 —— 固定在 scripts/rtk-nav2/demo-data/ ✓（✗ 不拷 ✓）
#     · 不渲染小车 —— 固定的 .no_render_fix ✓（原来是"按轮标记" ⇒ 现在固定开 ✓）
#   ★ 行为与交付版**逐条一致** ✓（判据：整理只许改"要敲多少"，不许改行为 ✓）：
#     同世界 ✓ 同配置 ✓ 渲染关 ✓ 经纬度航线 ✓ --tol 0.40 ✓ 走完自动抓图 ✓
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_DATA="${HERE}/demo-data"
TREE="$(cd "${HERE}/../.." && pwd)"

# ── 固定项（这就是"不用带参数"的全部内容）──
export CONTAINER_NAME="${CONTAINER_NAME:-${DEMO_CONTAINER:-rtknav2-demo}}"
export ROS_DOMAIN_ID="${DEMO_DOMAIN:-56}"
export GZ_PARTITION="${DEMO_PARTITION:-rtknav2demo}"
export RUN_ID="demo"
DEMO_DIR="${TREE}/artifacts/rtk-nav2/demo"

# ── 0) 固定输入必须在（✗ 不在这里拷；缺了就报清楚 ✓）──
[ -f "${DEMO_DATA}/geo_calib.json" ]        || { echo "✗ 缺固定标定 ${DEMO_DATA}/geo_calib.json"; exit 1; }
[ -f "${DEMO_DATA}/route_geo.json" ]        || { echo "✗ 缺固定航线 ${DEMO_DATA}/route_geo.json"; exit 1; }
[ -f "${DEMO_DATA}/waypoints_export.csv" ]  || { echo "✗ 缺固定航点导出 ${DEMO_DATA}/waypoints_export.csv"; exit 1; }

# ★★ 2026-10-12：出生点**不依赖生成物** ★★
#   按轮入口 start-northcampus.sh 是从 artifacts/rtk-nav2/outnav1/exports/waypoints_export.csv
#   **现读第 1 行**取出生点的 ✓ —— 但那个在 artifacts/ 下、是**生成物** ✗（被清掉就起不来 ✓）。
#   ⇒ 演示版把它**钉在 demo-data/ 里**（与标定、航线同处 ✓），并在两处兜住：
#     ① 用 SPAWN_X_OVERRIDE / SPAWN_Y_OVERRIDE **显式钉住**出生点 ✓（按轮入口支持这对覆盖 ✓）；
#     ② 万一那个生成物不在 ⇒ 从 demo-data/ 的固定副本**补齐** ✓（✗ 不要求用户做任何拷贝 ✓）。
WP_CSV="${TREE}/artifacts/rtk-nav2/outnav1/exports/waypoints_export.csv"
if [ ! -f "${WP_CSV}" ]; then
    mkdir -p "$(dirname "${WP_CSV}")"
    cp "${DEMO_DATA}/waypoints_export.csv" "${WP_CSV}"
    echo "★ 生成物缺失 ⇒ 已从固定副本补齐：${WP_CSV}（自动 ✓，用户不用管 ✓）"
fi
# ★ 出生点从**签入的那份**现读（✗ 不从生成物读 ✓）
read -r _SX _SY < <(python3 - "${DEMO_DATA}/waypoints_export.csv" <<'PY'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1], encoding='utf-8-sig')))
print(rows[0]['x'], rows[0]['y'])
PY
)
[ -n "${_SX:-}" ] && [ -n "${_SY:-}" ] || { echo "✗ 没能从 ${DEMO_DATA}/waypoints_export.csv 读出第 1 个点"; exit 1; }
export SPAWN_X_OVERRIDE="${_SX}"
export SPAWN_Y_OVERRIDE="${_SY}"
echo "★ 出生点（钉住 ✓）：(${SPAWN_X_OVERRIDE}, ${SPAWN_Y_OVERRIDE}) —— 取自 demo-data/waypoints_export.csv 第 1 行 ✓"

# ── 1) 固定产物目录 ＋ **固定的"不渲染小车"开关** ──
mkdir -p "${DEMO_DIR}/input"
touch "${DEMO_DIR}/.no_render_fix"

echo "★ 演示版起栈：容器=${CONTAINER_NAME} 域=${ROS_DOMAIN_ID} 分区=${GZ_PARTITION} 轮=${RUN_ID}"
echo "★ 渲染：关（固定 ${DEMO_DIR}/.no_render_fix 在 ✓ ⇒ 车体不渲染、车顶红盘仍在 ✓）"
echo "★ 产物目录：${DEMO_DIR}"

# ── 2) 起栈（复用按轮入口；它自己会做干净闸/就绪闸/GUI 配置 ✓）──
#    ★ 标定与航线只被走航线的脚本用（demo-drive.sh 显式指过去 ✓）⇒ 这里不传 ✓
#    ★ 全程不拷文件 ✓：输入固定在 demo-data/ ✓、产物固定在 artifacts/rtk-nav2/demo/ ✓
bash "${HERE}/start-northcampus.sh" "${RUN_ID}"
RC=$?

# ★★ 2026-10-12：**起栈失败必须就地停** ✓（★ 这条是我自己踩出来的真缺陷 ✗）
#   实测（2026-10-12）：我传错了镜像名（`robinox/…` ✗，正确是 `robonix/…` ✓）
#     ⇒ `ensure_container` 里 `docker run` 直接失败 ✗
#     ⇒ ★ 但**本脚本只把 RC 存下来、从不看它** ✗ ⇒ 继续往下走 ✓
#     ⇒ ★ 表相：最后只报一句"**演示 GUI 配置没生成**" ✗ —— **与真因（镜像名）差着二十行** ✗
#   ★ 这与本项目反复记的那条同族：★ **"发起了动作" ≠ "做成了"** ✓
#     （★ 更坏的变体：**调用方不看返回码** ⇒ 失败被**下游的表相**盖住 ✗）
#   ⇒ ★ 定式 ✓：**子脚本返回非零 ⇒ 立刻停 ＋ 把 rc 打出来** ✓
#     （✗ 别让它继续去补凭据/走航线 —— 那会让表相变成"**启动了但车不动**"✗，本项目记过 ✓）
if [ "${RC}" -ne 0 ]; then
    echo "✗ 起栈失败：start-northcampus.sh rc=${RC} ⇒ 就地停下 ✗"
    echo "  ★ 真正的错因在上面（★ 常见：镜像名拼错 / 镜像没建 / docker 没开 ✓）"
    echo "  ★ 不继续补凭据、不起航线 ✓（否则表相会变成「启动了但车不动」✗）"
    exit "${RC}"
fi

# ── 3) ★ 补一个"栈活性凭据"（2026-10-12 实测踩到 ✗）──
#   现象：业主那次"**启动后很久车都没动**" ✗ —— 真相不是车坏了，而是
#   `start-northcampus.sh` 返回 rc=0（栈确实起来了 ✓，`ros2 launch` 活着 ✓），
#   但轮次目录里的 **`logs/demo_launch.pid` 不见了** ✗ ⇒ `lib.sh` 的 `stack_running`
#   要求 `[ -s <pidfile> ]`（**非空** ✓）⇒ 判"栈没在运行" ✗ ⇒ `geo-route.sh` 直接退出 ✗
#   ⇒ 车主那边看到的就是"启动了、但车一动不动" ✓（★ 而栈其实是好的 ✓）。
#   ⇒ 这里**幂等补齐**：拿容器里**活着的** `ros2 launch` 的 PID 写回 `.pid`，并写 `.lstart` ✓。
#   ★ 为什么放在起栈之后：此刻栈一定活着 ✓ ⇒ 拿到的就是真身 ✓（✗ 不猜、不硬编码 ✓）。
_lpid_line="$(docker exec "${CONTAINER_NAME}" bash -lc 'ps -eo pid,args' 2>/dev/null | /bin/grep -E 'ros2 launch' | /bin/grep -v grep | head -1)"
_lpid="$(printf '%s' "${_lpid_line}" | awk '{print $1}')"
if [ -n "${_lpid}" ]; then
    mkdir -p "${DEMO_DIR}/logs"
    # ★★ 2026-10-12：**先删再写 ＋ 判产物** ✗ 别原地 `>` 之后**无条件**打 ✓
    #   ★ 实测（★ **干净克隆**里才暴露 ✓）：`logs/` 目录是**我的** ✓，
    #     ★ 但里面那几个文件是**容器（root）先写的** ✓ ⇒ ★ `>` 是**原地截断** ✗
    #     ⇒ ★ 它要的是**文件**的写权限 ✗ ⇒ ★ **Permission denied** ✗（★ 目录是我的也没用 ✓）
    #   ★ 而 `rm -f` **能删掉 root 的文件** ✓ —— ★ `unlink` 看的是**目录** ✓（已实测 ✓）
    #   ★ ★ 且原写法**不看写入结果** ✗ ⇒ ★ 在**失败之上**打了个 ✓ ✗✗
    #     （★ 与"**发起 ≠ 做成**"同族 ✓；★ 与上面"起栈失败要报出来"是**同一条** ✓）
    #   ★ ★ **本仓另一处同形**：`demo-drive.sh` 里同一段 ✓ ⇒ ★ 改要**两处一起改** ✓
    rm -f "${DEMO_DIR}/logs/demo_launch.pid" "${DEMO_DIR}/logs/demo_launch.lstart"
    _lstart="$(docker exec "${CONTAINER_NAME}" bash -lc "ps -o lstart= -p ${_lpid}" 2>/dev/null | sed 's/^ *//;s/ *$//')"
    printf '%s' "${_lpid}" > "${DEMO_DIR}/logs/demo_launch.pid" 2>/dev/null || true
    printf '%s' "${_lstart}" > "${DEMO_DIR}/logs/demo_launch.lstart" 2>/dev/null || true
    # ★ 判据落在**目标量**上 ✓：★ 要的是"**凭据 ＝ 容器里的真身**" ✗
    #   ★ ✗ 不能只判 `[ -s ]` ✗ —— ★ 若 `rm -f` 也失败（★ 目录不可写 ✓），
    #     **旧文件还在** ⇒ ★ `-s` 照样成立 ⇒ ★ 又会打假 ✓ ✗（★ 代理量 ✓ 不是目标量 ✗）
    if [ "$(cat "${DEMO_DIR}/logs/demo_launch.pid" 2>/dev/null)" = "${_lpid}" ]; then
        echo "★ 已补栈活性凭据：logs/demo_launch.pid = ${_lpid} ✓（+ .lstart ✓）"
    else
        echo "✗ 栈活性凭据**没写成**：${DEMO_DIR}/logs/demo_launch.pid 的内容 ≠ 真身 ${_lpid} ✗" >&2
        echo "  ★ 三种成因都归这一句 ✓：★ **不存在** ✓／★ **空** ✓／★ **是旧值**（★ 实测过：★ 目录不可写 ⇒ `rm` 失败 ⇒ 旧文件留下 ✓）" >&2
        echo "  ⇒ geo-route.sh 会判「栈没在运行」并直接退出 ✗（★ 表相会变成「启动了但车不动」✗）" >&2
        echo "  处置：看 ${DEMO_DIR}/logs/ 的属主与权限（★ 本步已先 rm 再写 ✓ ⇒ 还失败就是别的原因 ✓）" >&2
        exit 1
    fi
else
    echo "✗ 没在容器里找到 ros2 launch ⇒ 不补凭据（栈可能没起来 ✗）"
fi
exit "${RC}"
