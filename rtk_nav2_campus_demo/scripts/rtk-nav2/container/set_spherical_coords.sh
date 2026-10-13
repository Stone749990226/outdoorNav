#!/usr/bin/env bash
# ★ 容器内：尽早把世界的球坐标设成**他们场景的地理基准** ✓（阶段九 ✓）
#
# 用法（容器内）: set_spherical_coords.sh <world_name> <partition> [最大轮询次数]
#
# ★★ 2026-10-10 第三次修正（★ 分区必须**显式传进来** ✓）★★
#   上一版靠调用方 `cexec_bg` 注入分区 ✗ —— 而 `cexec_bg` 的分区来自 `_round_gzp`（读"当前 run" ✓），
#   ★ 那一刻 `start-demo.sh` **还没把本轮 run-id 记上** ✗ ⇒ 注入的是**容器默认 `rtknav2`** ✗
#   ⇒ 本脚本 `ign service -l` **在错分区里数** ⇒ **永远看不到 `set_spherical_coordinates`** ✗✗
#   ⇒ 球坐标从未设上 ✓（实测 NavSat 警告 953 次 ✗）。
#   ★ 与本项目那条定式完全一致：**"环境注入必须让子进程能自报"** ✓ ——
#     实测自报出来才发现拿到的是 `rtknav2` 而不是 `rtknav2_campus9-nav6` ✓✓。
#   ⇒ 改成**参数显式传入并在本脚本里 export** ✓（✗ 不依赖任何外部注入 ✗）。
#   · 我们的世界 `north_campus.sdf` 里**没有** `<spherical_coordinates>` ✗（他们的 Classic world 就没有 ✓），
#     而整条定位链依赖它 ✗ ⇒ 必须给它一个值 ✓；✗ 又不许改世界文件 ✗
#     ⇒ 用运行时服务 `/world/<w>/set_spherical_coordinates` ✓。
#   · ★★ 但**时间点**是关键 ✗ ★★：NavSat 系统在**传感器创建那一刻**就去读球坐标 ✗，
#     读不到就**整拍失败** ✓（日志 `NavSat.cc:259 ... Spherical coordinates not set.` ✓），
#     那几拍 GPS 报的是**绕 (0,0) 的垃圾经纬度** ✗ ⇒ `navsat_transform` 输出**几百万米** ✗
#     ⇒ `absolute_localizer` 的**第一条 fix** 落在垃圾窗口里 ✗ ⇒ 它把垃圾坐标当成基准锁住 ✗
#       （★ 首读没有基准可校 ⇒ `self.fix is None` ⇒ **直接接受** ✓）
#     ⇒ 之后每一条好读数都成"跳 6653 km" ✗ ⇒ 全丢 ✗ ⇒ 永远 `fix_stale` ✗
#     ⇒ 没有可用的 map 帧 ✗ ⇒ TF 闸门过不去 / nav2 起不来 / 就绪超时 ✗✗。
#   · ⇒ **必须在车被生成之前**把球坐标设上 ✓。服务在世界加载完那一刻就能用 ✓，
#     而车要等 `robot_description` ✓ ⇒ 中间有个**窗口** ✓ —— 本脚本就是抢这个窗口 ✓。
#
# ★★ 为什么单独写成文件（✗ 不再内联在 start-northcampus.sh 里 ✗）★★
#   第一版把它内联成 `cexec_bg "…" "<一大段带 \$ 转义的脚本>"` ✗ ⇒
#   ★ 实测**整条命令没跑起来** ✗（容器里连日志文件都没建 ✓），而外层只看到"已发起" ✓
#   —— 典型的"**发出信号 ≠ 做成了**" ✗。⇒ 定式：**复杂脚本一律落成文件、按路径调用** ✓。
set -uo pipefail

W="${1:?用法: set_spherical_coords.sh <world_name> <partition> [max_tries]}"
PART="${2:?缺 partition（★ 必须显式传 ✓ 见文件头）}"
MAX="${3:-1500}"

export GZ_PARTITION="${PART}"
export IGN_PARTITION="${PART}"

# ★★ 2026-10-13 修（★ 业主实测撞到 ✓）：★ 本文件**曾被拼坏** ✗ ——
#   ★ 头部改成了 `(world, partition, max)` 三参 ✓，★ 而**旧的两参尾巴没删** ✗（又一遍 `set -uo pipefail`
#     ＋ `MAX="${2:-1500}"` ✓）⇒ ★ 第二遍把**分区名当成了轮询次数** ✗
#     ⇒ ★ 实测 `seq: invalid floating point argument: 'rtknav2_demo'` ✗ ⇒ ★ **一次都没轮** ⇒
#       ★ 球坐标从未设上 ⇒（★ 见文件头那条链）★ 定位链锁死 ⇒ 没有 map 帧 ⇒ 代价地图不发图 ✗
#   ⇒ ★ 现在：★ **只有一套解析** ✓（三参 ✓）＋ ★ 成功写**凭据文件** ✓ ＋ ★ 失败**喊出来** ✓。
#   ★ 凭据文件的意义 ✓：★ 宿主侧**据此判"设上了没有"** ✓（✗ 不再靠"日志里有没有那句话" ✗ ——
#     ★ 那是"按文本比" ✓，而这条链要的是"**服务真的被调过**" ✓）。
OKFILE="${OKFILE:-/tmp/spherical_ok_${PART}}"
rm -f "${OKFILE}"          # ★ 每轮先清 ✓（✗ 不让上一轮的凭据冒充本轮 ✓）

echo "[$(date '+%H:%M:%S')] 开始轮询球坐标服务（world=${W}，分区=${PART}，最多 ${MAX} 次）"
for i in $(seq 1 "${MAX}"); do
    if ign service -l 2>/dev/null | grep -q 'set_spherical_coordinates'; then
        out="$(ign service -s "/world/${W}/set_spherical_coordinates" \
            --reqtype ignition.msgs.SphericalCoordinates \
            --reptype ignition.msgs.Boolean \
            --timeout 5000 \
            --req 'latitude_deg: 30.22413029, longitude_deg: 120.25979315833334, elevation: 0, heading_deg: 0, surface_model: EARTH_WGS84' 2>&1)"
        echo "[$(date '+%H:%M:%S')] ★ 已设上球坐标（第 ${i} 次轮询）：${out}"
        date '+%Y-%m-%dT%H:%M:%S' > "${OKFILE}"
        exit 0
    fi
    sleep 0.2
done
echo "[$(date '+%H:%M:%S')] ✗✗ 设不上球坐标（${MAX} 次轮询都没等到服务）✗"
echo "   ★ ⇒ ★ 本轮**定位链必坏** ✗（没有 map 帧 ⇒ 代价地图不发图 ✓）⇒ ★ 宿主侧应当**报错停** ✗"
exit 1

