#!/usr/bin/env bash
# frontend-bridge.sh —— 把**我们的车**接到**他们前端那条链**上（★ 接入点 ✓，实测已通 ✓）
#
#   用法：scripts/rtk-nav2/frontend-bridge.sh
#
# ★★ 这条链（★ 已端到端跑通 ✓ —— 侦察见 `docs/frontend-integration-recon.md` ✓）：
#   ① 前端（或任何 HTTP 客户端）★ `POST /api/routes` ✓（★ body 见该文档 §①）
#   ② ★ **他们的 `bridge.py`** ✓（★ 监听 `127.0.0.1:8765` ✓）★ 校验后 ★ 发 `FollowPath` 动作 ✓
#   ③ ★ nav2 `controller_server` ✓ ⇒ `/cmd_vel` ⇒ 我们的车 ✓
#   ④ 车 → ★ **`/odom`** ✓（★ 由 `frontend_adapter.py` 发 ✓，★ 装的是 `map→base_link` 的米值 ✓）
#   ⑤ 桥把 `/odom` 原样回填 ⇒ ★ `GET /api/status` ✓ ⇒ 前端画车标 ✓
#
# ★★ 起动这个脚本**做三件事** ✓（✗ 不碰我们现有演示的行为 ✓）：
#   ★ ① 起 `frontend_adapter.py` ✓（★ 发 `/odom` ✓ ＋ ★ **吞掉 `/outdoor_nav/sim_reset`** ✓
#     —— ★ 业主口径"**起始点就是小车位置**" ✓ ⇒ ★ 绝不让车被瞬移 ✓）
#   ★ ② 起**他们的 `bridge`** ✓，★ ★ **并且必须带 `use_sim_time:=true`** ✗✗
#   ★ ③ ★ **起前端 dev server** ✓（★ 在**容器里** ✓ —— ★ 业主口径："起完这个脚本就能直接
#     访问" ✓ ⇒ ★ 全程**不用切 Windows** ✓）；★ 并把这三样的 **PID 登记下来** ✓
#     （★ 供 `stop-demo.sh` 按 PID 收 ✓ —— ★ 因为"停栈"**收不掉**这条链 ✗，实测过 ✓）
#
# ★★★ 那个参数是**根因的修法** ✓（★ 一行参数、✗ 不改他们代码 ✓）：
#   ★ 他们的桥给**路径**打时间戳用**自己的时钟** ✓；★ 它默认**没开** `use_sim_time` ✗
#     ⇒ ★ 路径时间戳是**墙钟**（≈1.79e9 ✓）★ 而我们的栈与 TF 跑在**仿真时间**上 ✓（几百秒 ✓）
#     ⇒ ★ 控制器报 ★
#       `[tf_help] Transform data too old when converting from map to odom`
#       （Data time: 1791812929s … ｜ Transform time: 2415s …）✗
#     ⇒ ★ `Resulting plan has 0 poses in it` ✗ ⇒ ★ RPP 抛 ⇒ ★ `Controller patience exceeded` ✗
#       ⇒ ★ `FollowPath` **ABORTED**（★ 桥如实报 `status 6` ✓）
#   ★ ⇒ ★ 带上 `use_sim_time:=true` 之后 ★ **两行错误都 0 次** ✓、★ 路径能在 27 秒内**走完** ✓
#     （★ 实测：★ 出生点 → `(3.4476, −11.7593)` ✓、★ **离目标 0.2398 m** ✓、★ **六项速度全 0** ✓）
#
# ★ 前提（★ 两样，都**不在**镜像里 ⇒ 要操作者先备好 ✓ —— ★ 2026-10-13 更正 ✗：
#   原句写"镜像里已有他们的包构建过" ✗，★ **实测不成立** ✓）：
#   ★ ① **他们的桥**要构建过 ✓：★ 源码放 `<仓库>/.external/their_bridge` ✓
#     ⇒ ★ 本脚本按 `$THEIR_INSTALL` 找产物 ✓（默认 `/work/.external/install` ✓）
#     ★ 找不着会**报错停**并把构建命令**原样印出来** ✓（✗ 不"发起即报成功" ✗）
#   ★ ② ★ **前端**要**在镜像里** ✓：★ 源码在仓里的 `outdoor-patrol-demo/` ✓
#     ⇒ ★ `docker/Dockerfile.amd64` **构建时**已 `npm ci` ✓ ⇒ ★ 镜像没带就**报错停** ✓
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TREE="$(cd "${HERE}/../.." && pwd)"
CONTAINER="${CONTAINER_NAME:-${DEMO_CONTAINER:-rtknav2-demo}}"
THEIR_INSTALL="${THEIR_INSTALL:-/work/.external/install}"   # ★ 放**挂载树**里 ⇒ 容器重建也不丢 ✗
#   ★ 教训 ✓：原先默认 /tmp/ti ⇒ ★ 容器一重建就没了 ✗ ⇒ ★ `source` 失败 ⇒ 桥根本没起 ✗
#     ★ 而脚本当时**照样打了"已起"** ✗（★ 与"发起 ≠ 做成"同族 ✓）⇒ ★ 现在起不来就**报错停** ✓
OUT="/work/artifacts/rtk-nav2/demo/logs"

say() { printf '%s\n' "$*"; }

say "★ 接入：容器 ${CONTAINER}"
docker inspect "${CONTAINER}" >/dev/null 2>&1 || { say "✗ 容器不在 ⇒ 先跑 demo-start.sh"; exit 1; }
# ── ① 适配器（发 /odom ＋ 吞 sim_reset）──
if docker exec "${CONTAINER}" bash -lc 'pgrep -f frontend_adapter.py' >/dev/null 2>&1; then
    say "  ★ 适配器已在跑 ✓"
else
    docker exec -d "${CONTAINER}" bash -lc \
        "source /opt/ros/humble/setup.bash && source /work/rtk_nav2_ws/install/setup.bash && \
         python3 /work/scripts/rtk-nav2/container/frontend_adapter.py > ${OUT}/frontend_adapter.log 2>&1"
    say "  ★ 适配器已起 ✓（日志 ${OUT}/frontend_adapter.log ✓）"
fi

# ── ② 他们的桥 ★ 带 use_sim_time ──
if docker exec "${CONTAINER}" bash -lc 'pgrep -f "lib/outdoor_nav_bridge/bridge"' >/dev/null 2>&1; then
    say "  ★ 他们的桥已在跑 ✓（★ 请确认它带 use_sim_time:=true ✗ —— 不带会 0-poses 中止 ✓）"
else
    # ★★ 先**确认能起** ✓（✗ 不再"发起即报成功" ✗）
    if ! docker exec "${CONTAINER}" bash -lc "test -f ${THEIR_INSTALL}/setup.bash && test -f ${THEIR_INSTALL}/outdoor_nav_bridge/lib/outdoor_nav_bridge/bridge"; then
        say "✗ 没找到他们的构建产物：${THEIR_INSTALL}/setup.bash ✗"
        say "  ⇒ ★ 先构建 ✓（产物要放**挂载树**里，✗ 别放 /tmp ✗ —— 容器一重建就没了 ✓）："
        say "     docker exec ${CONTAINER} bash -lc 'source /opt/ros/humble/setup.bash && \\"
        say "       colcon build --base-paths /work/.external/their_bridge --packages-select outdoor_nav_bridge \\"
        say "         --build-base /work/.external/build --install-base ${THEIR_INSTALL}'"
        exit 1
    fi
    docker exec -d "${CONTAINER}" bash -lc \
        "source /opt/ros/humble/setup.bash && source ${THEIR_INSTALL}/setup.bash && \
         python3 ${THEIR_INSTALL}/outdoor_nav_bridge/lib/outdoor_nav_bridge/bridge \
           --ros-args -p use_sim_time:=true > ${OUT}/their_bridge.log 2>&1"
    sleep 3
    if docker exec "${CONTAINER}" bash -lc 'curl -s --max-time 4 http://127.0.0.1:8765/api/status' >/dev/null 2>&1; then
        say "  ★ 他们的桥已起 ✓（★ **已带 use_sim_time:=true** ✓ —— 这就是根因的修法 ✓）"
    else
        say "  ✗ 桥**没能应答** ✗ ⇒ 看 ${OUT}/their_bridge.log"
        docker exec "${CONTAINER}" bash -lc "tail -5 ${OUT}/their_bridge.log" 2>/dev/null | sed 's/^/    /'
        exit 1
    fi
fi

# ── ③ 前端 dev server（★ 在**容器里**跑 ✓ ⇒ ★ "一条命令起完" ✓）──
#   ★ 业主原话 ✓："**怎么还要在 windows 和 wsl 之间切换** ✓ —— 我想起完 frontend-bridge.sh
#     就能直接访问" ✓ ⇒ ★ 前端搬进容器 ✓
#   ★ 为什么这样就能"两侧都开" ✓：★ 容器是 `--network host` ✓（★ 实测 ✓）＋
#     ★ `npm run dev` ⇒ `vite --host 0.0.0.0` ✓ ⇒ ★ 它绑的是**宿主的** `0.0.0.0:5173` ✓
#     ⇒ ★ WSL 侧与 Windows 侧**都能**开 ✓（★ Windows→WSL 的转发今天实测过 ✓）
#   ★ ★ **必须在**容器里**起（✗ 不在宿主）** ✗：★ 宿主 WSL 里**没有可用的 node** ✗
#     （★ 实测：`node` 不在 PATH ✓、`npm` 只是 Windows 的 shim ✓、★ 而那个 `node.exe`
#     从 WSL 调 **`Permission denied (os error 13)`** ✗）⇒ ★ 这就是原来要切 Windows 的根因 ✓
#   ★ 镜像里那份在哪 ✓：`/opt/outdoor-patrol-demo` ✓（★ `Dockerfile.amd64` **构建时**
#     已 `npm ci` ✓ —— ★ 首次运行装会要联网 ＋ 多等几分钟 ⇒ ★ "一条命令"就不成立 ✗）
#   ★ ★ ✗ **不在镜像里就报错停** ✗（✗ 不"发起即报成功" ✗ —— ★ 同一个坑本文件上面记过 ✓）
FRONT_DIR="/opt/outdoor-patrol-demo"
if ! docker exec "${CONTAINER}" bash -lc "test -d ${FRONT_DIR}/node_modules"; then
    say "✗ 镜像里没有前端依赖：${FRONT_DIR}/node_modules ✗"
    say "  ⇒ ★ 这个镜像不是『带前端』的那版 ✓ ⇒ ★ 要重建镜像 ✓"
    say "  （★ 见 docs/frontend-integration.md §五 ✓；★ 构建命令：scripts/build-image.sh ✓）"
    exit 1
fi
if docker exec "${CONTAINER}" bash -lc 'pgrep -f "[v]ite" >/dev/null 2>&1'; then
    say "  ★ 前端已在跑 ✓"
else
    docker exec -d "${CONTAINER}" bash -lc \
        "cd ${FRONT_DIR} && npm run dev > ${OUT}/frontend_dev.log 2>&1"
    say "  ★ 前端已起 ✓（容器内 ${FRONT_DIR} ／ 日志 ${OUT}/frontend_dev.log ✓）"
fi

# ── ③′ ★ **登记 PID**（★ 停的时候按 PID 收 ✓）──
#   ★ 为什么要有它 ✓：★ 实测过"**停栈**收不掉这条链" ✗ ——
#     ★ `stop-demo.sh` 只收"栈"（★ launch ／ gz ／ 节点 ✓），★ **这三样不在其中** ✗
#     ⇒ ★ 业主停完栈，桥**还在** `8765` 上听 ✓（★ 实测：栈停后仿真进程 **0** ✓、
#     而桥 pid **6005** 还活着 ＋ `8765` **LISTEN** ✓ —— ★ 这就是那条反例 ✓）
#   ★ ★ 所以在**起的时候**登记 ✓ ⇒ ★ 停的时候**按 PID 收** ✓（✗ 不按名字片段 ✗ ——
#     名字片段会数错：★ 实测 `[b]ridge\.py` 数到 0 ✗，★ 因为进程名里**没有** `.py` ✓）
#   ★ ★ 写在**容器内** ✓（✗ 不从宿主写 ✗ —— ★ 这个目录由容器（root）创建 ⇒
#     ★ 宿主 `>` 原地截断要**文件**写权限 ⇒ Permission denied ✗；★ 本仓栽过 ✓）
#   ★ ★ 形态与既有的一致 ✓：★ **一个 PID 一个 `*.pid` 文件** ✓（✗ 不写一个列表文件 ✗ ——
#     ★ `stop_stack.sh` 的 `kill_pidfile` 是**按 `*.pid` 逐个读**的 ✓ ⇒ ★ 写成列表它**读不到** ✗）
#     ⇒ ★ 这样 `stop-demo.sh` **不用改就**会连前端一起收 ✓（★ 走的就是它已有的那条路 ✓）
docker exec "${CONTAINER}" bash -lc \
    "rm -f ${OUT}/frontend_chain_*.pid; \
     for p in \$(ps -eo pid,args | /bin/grep -E '[f]rontend_adapter\.py|[o]utdoor_nav_bridge/bridge|[v]ite' | awk '{print \$1}'); do \
         printf '%s' \"\$p\" > ${OUT}/frontend_chain_\${p}.pid; \
     done; \
     ls ${OUT}/frontend_chain_*.pid 2>/dev/null | wc -l" \
    2>/dev/null | tail -1 | sed 's/^/  ★ 已登记前端链 PID 文件数 = /'

# ── ④ 自检（★ 按产物判 ✓）──
sleep 12
say "  ★ 自检："
say "    HTTP :8765 ⇒ $(docker exec "${CONTAINER}" bash -lc 'curl -s --max-time 5 http://127.0.0.1:8765/api/status' 2>/dev/null | head -c 120)"
say "    use_sim_time ⇒ $(docker exec "${CONTAINER}" bash -lc 'source /opt/ros/humble/setup.bash && timeout 20 ros2 param get /outdoor_nav_bridge use_sim_time 2>/dev/null' 2>/dev/null | tr -d '\n')"
say "    /odom 发布者 ⇒ $(docker exec "${CONTAINER}" bash -lc 'source /opt/ros/humble/setup.bash && timeout 20 ros2 topic info /odom 2>/dev/null' 2>/dev/null | /bin/grep -oE 'Publisher count: [0-9]+' || echo '（取不到）')"
say "    :5173 监听（★ **宿主侧**量 ✓ —— 容器是 host 网络 ⇒ 宿主侧量到的就是它 ✓）⇒ $( (ss -ltn 2>/dev/null || netstat -ltn 2>/dev/null) | /bin/grep -c ":5173" )"
say "★ 完 ⇒ 现在可以 POST /api/routes 了 ✓（★ 浏览器开 http://localhost:5173 ✓ —— ★ WSL 侧、Windows 侧都行 ✓、✗ 不用切 Windows ✗）"
say "  ★ ⚠ 若 :5173 是 0 ⇒ 看 ${OUT}/frontend_dev.log ✓（★ 别当『没起』就跳过 ✓）"
