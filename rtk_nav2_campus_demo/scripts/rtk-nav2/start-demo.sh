#!/usr/bin/env bash
# 启动 RTK 第三阶段（复用 nav2 建图与规划）演示。
#
# 用法：scripts/rtk-nav2/start-demo.sh [run-id] [--gui] [--no-gui] [-- 额外的 ros2 launch 参数]
#   --gui     打开 Gazebo 与 RViz 窗口（要投屏/截图时用）
#   --no-gui  无界面（**默认**）。自带 GUI 在软件渲染下会把共享的 X 显示器堵死。
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID=""
LAUNCH_ARGS=()
WITH_GUI=0
while [ $# -gt 0 ]; do
    case "$1" in
        --) shift; LAUNCH_ARGS=("$@"); break ;;
        --gui) WITH_GUI=1; shift ;;
        # ★ 阶段八改名：`--no-gui` 的名字在**骗人** —— 它只关 RViz，Gazebo GUI 照开
        #   （`gz_gui:=false` 会走 `-s`、相机链路整个消失，那一项不能关）。
        #   `--no-rviz` 是**同义词**；旧名保留（`start-depth-drop.sh` 还在用 `--gui`，
        #   向后兼容），但新写的入口一律用 `--no-rviz`。
        --no-rviz) WITH_GUI=0; shift ;;
        --rviz) WITH_GUI=1; shift ;;
        --no-gui) WITH_GUI=0; shift ;;
        -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
        *) RUN_ID="$1"; shift ;;
    esac
done
RUN_ID="$(resolve_run_id "${RUN_ID}")"

if [ "${WITH_GUI}" -eq 0 ]; then
    LAUNCH_ARGS=("rviz:=false" "${LAUNCH_ARGS[@]}")
fi
# **不自动起路线跟随**：它由 run-route.sh 单独启动。栈刚起来时代价地图还在瞬态，
# launch 一并起它会在几秒内以非零码退出，在日志里留下一个"看起来像故障"的退出码。
LAUNCH_ARGS=("route:=false" "${LAUNCH_ARGS[@]}")

# **起栈前先看有没有别的 start-demo 在跑**：它的重试循环会 stop_stack，
# 而那个动作会杀掉"不是它起的栈"（我上两轮就是这么被自己上一轮的残留骗了两次）。
OTHERS="$(other_start_demo_pids)"
if [ -n "${OTHERS// /}" ]; then
    die "已有另一个 start-demo 在运行（PID: ${OTHERS}）。它会按自己的重试逻辑停栈，会破坏不是它起的栈——请等它结束，或先手工处理，不要并发起栈。"
fi

ensure_container
# ★ 2026-10-09：**起栈前的 display 自检**（选中 display 必须是当前 uid 的 socket + xdpyinfo 成功，
#   失败直接拒起）+ **统一把 ROBONIX_DEPTH_DROP_DISPLAY 传给容器**（见 lib.sh 里 ensure_display
#   的长注释：`campus8-a` 那次容器 baked 的是别人的 DISPLAY ⇒ Gazebo GUI 死 ⇒ 整栈看起来像
#   "定位没起来"）。收敛在这一处 ⇒ 所有走 start-demo.sh 的入口都自动拿到 ✓。
ensure_display
ensure_workspace
fix_artifacts_ownership
unblank_display
ensure_run_dirs "${RUN_ID}"
set_current_run "${RUN_ID}"

if stack_running; then
    log "第三阶段演示栈已在运行。先执行 scripts/rtk-nav2/stop-demo.sh 再重启。"
    exit 1
fi

# 记录"这个栈是谁起的"：重试循环据此只停自己的栈（见 lib.sh 的说明）
record_stack_owner "${RUN_ID}"
trap 'clear_stack_owner' EXIT

# ---- 留痕：这次运行实际生效的东西 ----
# 记录"这是同一个 run-id 的第几次起栈"：同一 run-id 重复起栈会**覆盖**该 run 的
# 同名产物（报告/地图/截图），事后必须能看出来。
MARKER="${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/.stack_start_count"
mkdir -p "${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}"
START_N=$(( $(cat "${MARKER}" 2>/dev/null || echo 0) + 1 ))
printf '%s' "${START_N}" > "${MARKER}"
record_state "${RUN_ID}" "stack_start_index" "${START_N}"
if [ "${START_N}" -gt 1 ]; then
    record_state "${RUN_ID}" "stack_restart_note" "同一 run-id 第 ${START_N} 次起栈：同名产物（报告/地图/截图/bag）会被本次覆盖"
fi
record_state "${RUN_ID}" "launch_args" "${LAUNCH_ARGS[*]:-（无）}"
# 内建录制是否被关掉（`SKIP_BAG=1`）：**要能从这一轮的产物里看出来**，
# 否则"这一轮没录 bag"会被读成"录制坏了"。
record_state "${RUN_ID}" "skip_bag" "${SKIP_BAG:-0}"
record_image_identity "${RUN_ID}"
record_config_fingerprint "${RUN_ID}" "${LAUNCH_ARGS[*]:-}"

# run_kind 由**实际的启动参数**推导，不靠人记得标。
# 第三阶段的规则比第二阶段**更严**：第二阶段只有两个清空开关会改变结果，所以按名单判；
# 第三阶段凡是有**任何**透传参数（rviz/route 这两个由本脚本自己加的不算）就一律标成
# 归因运行——因为第三阶段还没有已知的"无害参数"，宁可比实际更保守。
# ★★ 2026-10-09（阶段八）**修正 `run_kind` 的判定** ★★
#   原规则是"**只要带了启动参数就标归因运行**"（那时第三阶段确实没有已知的无害参数）。
#   但阶段八起栈**必然**要带 `world:=` / `nav2_config:=` / `static_*:=` / `x_pose/y_pose:=`
#   —— 它们是**这个阶段的正常参数**，不是"注入"⇒ 照旧规则会让**每一轮 campus 都被标成
#   "归因运行（非主线结果）"** ✗，将来引用"campus8-x 是本轮结果"就与记录自相矛盾
#   （reviewer 2026-10-09 抓到的，正是"判轮次有效性"这条）。
#   ⇒ 判法：**先认这是不是阶段八那条入口**（看 `nav2_config` 是不是 `campus_nav2.yaml`）——
#     是 ⇒ 阶段八那一组参数进白名单（其余参数照旧算注入）；不是 ⇒ **保持原规则一个字不动**
#     （阶段三/七的可复跑判据不受影响）。
IS_STAGE8=0
for a in "${LAUNCH_ARGS[@]}"; do
    case "$a" in
        nav2_config:=*campus_nav2.yaml) IS_STAGE8=1 ;;
    esac
done
ARGS_STR=""
for a in "${LAUNCH_ARGS[@]}"; do
    case "$a" in
        rviz:=*|route:=*) ;;
        world:=*|nav2_config:=*|static_world:=*|static_topic:=*|static_resolution:=*|static_size:=*|static_origin:=*) \
            [ "${IS_STAGE8}" -eq 1 ] || ARGS_STR="${ARGS_STR} $a" ;;
        # ★ 检查档会带 `rviz_config:=…/campus.rviz`（阶段八自己的视图配置，Scale 8 而不是 45）
        #   —— 它同样是**这一阶段的标配**。**只在阶段八那支入口、且值确实是 campus.rviz 时**才放过 ✗
        #   （别的 `rviz_config:=` 是实打实的注入 ⇒ 照旧标归因运行 ✓）
        rviz_config:=*campus.rviz) [ "${IS_STAGE8}" -eq 1 ] || ARGS_STR="${ARGS_STR} $a" ;;
        x_pose:=*|y_pose:=*|z_pose:=*) \
            [ "${IS_STAGE8}" -eq 1 ] || ARGS_STR="${ARGS_STR} $a" ;;
        *) ARGS_STR="${ARGS_STR} $a" ;;
    esac
done
if [ -n "${ARGS_STR// /}" ]; then
    record_state "${RUN_ID}" "run_kind" "归因运行（非主线结果：启动参数里带了${ARGS_STR}）"
elif [ "${IS_STAGE8}" -eq 1 ]; then
    # 阶段八：这里的参数是**这一阶段的标配**（world/nav2_config/static_*/出生点），
    # 只有上面循环里那些**不在白名单**的参数才会置空 ARGS_STR ⇒ 走到这里就是"干净起栈" ✓
    record_state "${RUN_ID}" "run_kind" "主线（阶段八标准起栈：world=campus_100.sdf + nav2_config=campus_nav2.yaml + 静态层栅格 + 生成器 SPAWN）"
else
    record_state "${RUN_ID}" "run_kind" "主线（干净启动）"
fi

READY_RC=1
for attempt in 1 2 3; do
    log "启动第三阶段演示栈（第 ${attempt} 次尝试），run-id = ${RUN_ID}"
    # 清掉上次可能残留的 gz 传输文件：gz 被 SIGKILL 后留下的文件会让新实例起不来
    cexec "rm -rf /tmp/ign-* /tmp/gz-* /dev/shm/ign* /dev/shm/gz* 2>/dev/null; true"
    stop_stack_if_ours "${RUN_ID}"
    # ★ `SKIP_BAG=1` 透传进容器 ⇒ **不起内建 rosbag 录制**（默认 0 = 照旧录制，行为不变）。
    #   容器侧开关与本开关的完整说明在 `scripts/rtk-nav2/container/start_stack.sh`。
    # ★ 2026-10-10（reviewer 裁定）：改用 **`docker exec -d`**（`cexec_bg`）✓ ——
    #   `-d` ⇒ **由 daemon 持有这个 exec 会话** ✓，**客户端的生死与它无关** ✓；
    #   容器侧 `start_stack.sh` 里还有 `setsid` ✓ ⇒ 再断开它自个儿的会话 ✓。
    #   ★ **但要如实说**：这条是**加固**，**不是** `campus8-h` 那次塌栈的解释 ✗ ——
    #     我拿"旧姿势起的 `campus8-i`"实测过：**断开 ssh 之后 launch PID 仍活着** ✓
    #     ⇒ **SIGHUP 假设被否掉了** ✗。所以这条防的是"**将来别踩**"，**不是**"已经查明的凶手" ✓。
    # ★★ 2026-10-10（reviewer 裁定）：**分区随 run-id 唯一化** ✗ ★★
    #   实测真凶：容器里**同时跑着三套 `ign gazebo`** ✗（两套陈旧 ✓），而 `lib.sh:17`
    #   的 `GZ_PARTITION` 是**固定常量 `rtknav2`** ✗ ⇒ 它们**同分区** ⇒
    #   本轮的 bridge **同时收到旧仿真的 gz 话题** ✗ ⇒ **代价图可能被"另一台车"的
    #   观测写** ✗ ⇒ "补丁 + 向北生长 + 楔形"**三件事一次全解释** ✓（生长 = 那台车在动 ✓）。
    #   ⇒ **即使有残留仿真，最坏也只是浪费资源、不会串台** ✓（把"必然串台"降级成"无害残留" ✓）。
    #   ★ 不必重建容器 ✓：`docker exec -e` 即可覆盖（容器创建时的值只对**没显式覆盖的** exec 生效 ✓）
    #     —— 这里用 `bash -lc` 的**环境变量前缀**达到同样效果 ✓。
    GZP="rtknav2_${RUN_ID}"
    # ★ 2026-10-10（阶段九）：多透一个**可选的**服务端配置路径 ✓
    #   （`start_stack.sh` 只在它非空时用 ✓ ⇒ 其它阶段行为**逐字不变** ✓）。
    #   ★ 走 `bash -lc` 的环境变量前缀 ✓ —— `docker exec` **不继承宿主 env** ✗，
    #     所以**必须显式列在这里** ✓（这也是本项目"环境注入要显式、并让子进程自报"的那条 ✓）。
    # ★★ 2026-10-11（阶段十 · 第 0 刀）：多透一个**工作区名** ✓ ★★
    #   容器侧 `start_stack.sh` 里 `WS=` 是**写死**的 ✗（它跑在 `docker exec` 里 ⇒ 拿不到宿主 env ✗）
    #   ⇒ 必须**显式列在这里** ✓（与本文件里 GZ_PARTITION/… 同一族 ✓）。
    #   ★ 默认值 = `rtk_nav2_ws` ⇒ 不设该变量时**行为逐字不变** ✓。
    #   ★ 用**环境变量前缀**（不是复合命令）⇒ 赋值确实作用于那条 start_stack.sh 调用 ✓。
    cexec_bg "/tmp/stack_start_${RUN_ID}.log" "GZ_PARTITION='${GZP}' IGN_PARTITION='${GZP}' SKIP_BAG='${SKIP_BAG:-0}' WORKSPACE_NAME='${WORKSPACE_NAME}' ROBONIX_DEPTH_DROP_DISPLAY='${ROBONIX_DEPTH_DROP_DISPLAY:-}' NORTH_CAMPUS_SERVER_CONFIG='${NORTH_CAMPUS_SERVER_CONFIG:-}' /work/scripts/rtk-nav2/container/start_stack.sh '${RUN_ID}' ${LAUNCH_ARGS[*]:-}"
    log "本轮 gz 分区 = ${GZP}（随 run-id 唯一 ⇒ 与任何残留仿真**不会串台** ✓）"

    log "等待定位链 / 相机与点云 / 危险距离 / nav2 两张图就绪（最长 300 秒）"
    READY_LOG="${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/logs/readiness.log"
    # ★★★ 2026-10-14（步骤②）：★ **不再盲等满 300 s** ✗ ★★★
    #   ★ 现象（业主 2026-10-14 亲报 ✓）：★ 第一次起栈 ⇒
    #     `[lifecycle_manager] … async_send_request failed` ⇒
    #     `[ERROR] Failed to bring up all requested nodes. Aborting bringup.`
    #     ⇒ ★ **nav2 没激活** ⇒ ① 没有代价地图（＝"激光周围没有那个圆盘" ✓）
    #        ② 桥的 `server_is_ready()` 恒 false ⇒ ★ 发路径**一直停在 queued** ⇒ **车不动** ✓
    #     ⇒ ★ 旧行为：waiter 只会**等满 300 s** ⇒ 然后重试 ⇒ ★ 第二次就好了 ✓（★ 慢，但能救回 ✓）
    #   ★ 改法 ✓：★ 把 waiter 丢到**后台** ✓（它自己把退出码写成凭据 ✓，见 `wait_ready_then_rc.sh`），
    #     宿主侧一边等它、一边**盯 `demo_launch.log` 里那条具体特征** ✓：
    #       `Failed to bring up all requested nodes. Aborting bringup.`
    #     ⇒ ★ 一出现就**立刻判未就绪** ✗（✗ 不空等 ✓）⇒ 走**原来那条重试路径** ✓（行为不变 ✓）
    #   ★ ★ 判据拿**具体值**比 ✓（✗ 不拿"有没有 ERROR"这种否定式 ✓ —— 启动期的 ERROR 有一堆 ✓）
    #   ★ ★ 且**不放松就绪闸** ✗：`wait_ready.py` 自己那几条判据一个字没动 ✓；
    #     这条只是"**更早判失败**" ✓（✗ 绝不把"没就绪"变成"就绪" ✗）
    #   ★ `demo_launch.log` 每轮重开（`> ` 截断 ✓，见 `container/start_stack.sh:179`）
    #     ⇒ ★ 不会把上一轮的 abort 误判成本轮的 ✓
    #   ★ ★ 凭据与日志都落在**挂载树**里 ✓ ⇒ 宿主侧直接读 ⇒ ★ 轮询里**不夹 `docker exec`** ✓
    #     （★ 每次 `docker exec` 要 0.2~3 s ✗ ⇒ 会把这 10 s 的预算吃光 ✓）
    READY_RC_HOST="${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/logs/ready_rc"
    READY_STARTED_HOST="${READY_RC_HOST}.started"
    LAUNCH_LOG="${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/logs/demo_launch.log"
    ABORT_PAT='Failed to bring up all requested nodes. Aborting bringup.'
    # ★★ 第三条早识别用（★ 2026-10-14 **在业主眼前那一轮实测到** ✓）：★ lifecycle **卡住**的判据 ✓
    #   ★ 现场 ✓：`demo_launch.log` 里 `lifecycle_manager_nav2` 一共只有 **5 行**，最后一行是
    #     `Configuring controller_server` ⇒ ★ 之后 **215 s 一个字都没有** ✗，
    #     ★ 而 `Aborting bringup` **一行都没出现** ✗（服务调用**既不回、也不超时** ✗）
    #   ⇒ ★ 只盯 `Aborting bringup` **在这条上不会触发** ✗ ⇒ 还得补一条"**停住不动**"的判据 ✓
    #   ★ `LIFECYCLE_STALL_S` 的取法 ✓：★ 取**健康起栈里 lifecycle 相邻两行的最大间隔**的若干倍 ✓
    #     （✗ 不拍脑袋 ✗）—— ★ 健康值实测见交接文档；这里先给一个**保守**的初值 ✓
    #   ★ ★ 阈值**实测标定** ✓（2026-10-14 ✓）：★ 健康那一轮里，lifecycle 相邻两行的
    #     **最大间隔 = 1.1 s** ✓（n=11 行 ✓）⇒ ★ 取 **30 s** ⇒ ★ 27 倍余量 ✓
    #     （★ 卡住那轮实测 ≥215 s ✓ ⇒ 30 s 判得出、且远不到 300 s ✓）
    LIFECYCLE_STALL_S="${LIFECYCLE_STALL_S:-30}"
    rm -f "${READY_RC_HOST}" "${READY_STARTED_HOST}"
    cexec_bg "/work/artifacts/rtk-nav2/${RUN_ID}/logs/readiness.log" \
        "/work/scripts/rtk-nav2/container/wait_ready_then_rc.sh '${WS_DIR}' 300 '/work/artifacts/rtk-nav2/${RUN_ID}/logs/ready_rc'"
    set +e
    READY_RC=""
    T_WAIT0="$(date +%s)"
    WAITER_OK=0
    while :; do
        rc="$(tr -d '\r\n ' < "${READY_RC_HOST}" 2>/dev/null)"
        if [ -n "${rc}" ]; then READY_RC="${rc}"; break; fi
        # ★★ "waiter 到底起没起" —— ★ 2026-10-14 实测加的判据 ✓（✗ 不是想出来的 ✗）：
        #   第一版 waiter 写成 `set -u` ＋ `source /opt/ros/humble/setup.bash`
        #   ⇒ ★ 它 0.0 s 就死（`AMENT_TRACE_SETUP_FILES: unbound variable`）✗
        #   ⇒ ★ rc 文件永远不写，而宿主侧**完全看不出来** ✗ ⇒ ★ 白等 330 s × 3 次 = 1124 s ✗✗
        #   ⇒ ★ 现在 waiter **先写哨兵** ✓ ⇒ ★ 20 s 内没哨兵 ⇒ 立刻判失败 ✓
        if [ "${WAITER_OK}" -eq 0 ] && [ -s "${READY_STARTED_HOST}" ]; then
            WAITER_OK=1
            log "★ 就绪 waiter 已起 ✓（哨兵 ${READY_STARTED_HOST##*/} 出现 ✓）"
        fi
        if [ "${WAITER_OK}" -eq 0 ] && [ $(( $(date +%s) - T_WAIT0 )) -gt 20 ]; then
            READY_RC=125
            log "✗ **就绪 waiter 根本没起来** ✗（20 s 内没写出哨兵 ✓）⇒ 判未就绪 ✗（✗ 不空等 330 s ✗）"
            log "   ★ 依据：2026-10-14 第一版 waiter 因 `set -u` ＋ ROS `setup.bash` 当场死 ✓"
            log "     ⇒ rc 文件永远不写、宿主侧看不出差别 ⇒ 白等 1124 s ✗"
            log "   ⇒ 看 ${READY_LOG} 的尾部："
            tail -5 "${READY_LOG}" 2>/dev/null | sed 's/^/     /'
            break
        fi
        # ★★ 早识别②：★ lifecycle **停住不动** ✗（★ `Aborting bringup` 那条在它上面**永远不出现** ✗）★
        #   ★ 判据落**具体量**上 ✓：★ 最后一条 `lifecycle_manager_nav2` 那行**自带的 epoch 秒** ✓
        #     ⇒ ★ 距今超过 `LIFECYCLE_STALL_S` 秒 ⇒ 判未就绪 ✓（✗ 不拿"有没有 ERROR"这种否定式 ✗）
        n_life="$(/bin/grep -ac 'lifecycle_manager_nav2' "${LAUNCH_LOG}" 2>/dev/null)"
        if [ -n "${n_life}" ] && [ "${n_life}" != "0" ]; then
            last_life="$(/bin/grep -a 'lifecycle_manager_nav2' "${LAUNCH_LOG}" 2>/dev/null | tail -1)"
            t_life="$(printf '%s' "${last_life}" | /bin/grep -oE '\[[0-9]{10}\.[0-9]+\]' | head -1 | tr -d '[]' | cut -d. -f1)"
            # ★★ 还要**认状态** ✗（★ 2026-10-14 实测补的 ✓ —— 差点写成假阳性 ✗）：
            #   ★ 健康轮里 lifecycle **激活完就不再打日志**了 ✓（最后一行是 `Creating bond timer...` ✓，
            #     它到日志末行已 49 s ✓ 且还在涨 ✓）⇒ ★ 只按"最后一行有多旧"判，**健康轮迟早也会命中** ✗✗
            #   ⇒ ★ 判据再收紧一层 ✓：★ 最后那行必须**是一个"起了头没干完"的状态变更** ✓
            #     即含 `Configuring ` 或 `Activating ` ✓ —— ★ 健康轮的末行是 `Creating bond timer...` ✓
            #     /`Managed nodes are active` ✓ ⇒ ✗ 不匹配 ⇒ 不误判 ✓
            if [ -n "${t_life}" ] && [ $(( $(date +%s) - t_life )) -gt "${LIFECYCLE_STALL_S}" ] \
               && printf '%s' "${last_life}" | /bin/grep -qE '(Configuring|Activating) '; then
                READY_RC=126
                log "★★ **早识别②（lifecycle 停住）** ✓：最后一条 lifecycle 行距今 $(( $(date +%s) - t_life ))s > ${LIFECYCLE_STALL_S}s ⇒ 判**未就绪** ✗"
                log "   ★ 最后那条是：$(printf '%s' "${last_life}" | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-160)"
                log "   ★ 共 ${n_life} 行 lifecycle ⇒ ★ 卡在服务调用上（既不回、也不超时）⇒ nav2 不会激活 ✗"
                log "   ★ 没等满 300 s ✓：从开始等到这里只用了 $(( $(date +%s) - T_WAIT0 ))s"
                printf '%s' "$(( $(date +%s) - t_life ))" > "/tmp/lifecycle_stall_${RUN_ID}"
                cexec "pkill -f '[w]ait_ready\.py' >/dev/null 2>&1 || true" >/dev/null 2>&1 || true
                break
            fi
        fi
        n_abort="$(/bin/grep -c -F "${ABORT_PAT}" "${LAUNCH_LOG}" 2>/dev/null)"
        if [ -n "${n_abort}" ] && [ "${n_abort}" != "0" ]; then
            READY_RC=1
            # ★ 那条日志行自带的 epoch 秒 ⇒ ★ 这才是"abort **那行**出现的时刻" ✓
            t_abort="$(/bin/grep -m1 -F "${ABORT_PAT}" "${LAUNCH_LOG}" 2>/dev/null \
                       | /bin/grep -oE '\[[0-9]{10}\.[0-9]+\]' | head -1 | tr -d '[]' | cut -d. -f1)"
            t_now="$(date +%s)"
            log "★★ **早识别** ✓：demo_launch.log 出现 'Aborting bringup'（${n_abort} 处 ✓）⇒ 判**未就绪** ✗"
            log "   ★ 为什么这是硬失败 ✗：nav2 没激活 ⇒ 代价地图不发图 ∧ 桥的 server_is_ready() 恒 false ⇒ **车不动** ✓"
            log "   ★ 没等满 300 s ✓：从开始等到这里只用了 $((t_now - T_WAIT0))s（waiter 已就地停掉 ✓）"
            if [ -n "${t_abort}" ]; then
                log "   ★ abort 那行自身的时刻 = ${t_abort} ⇒ 距「现在就重试」$((t_now - t_abort))s"
                printf '%s' "$((t_now - t_abort))" > "/tmp/abort_to_retry_${RUN_ID}"
            fi
            # ★ 括号写法 ✓（✗ 不用裸 `wait_ready.py` ✗ —— 那条 bash 包装的命令行里也有这个串 ⇒ **自匹配** ✗，本仓记过）
            cexec "pkill -f '[w]ait_ready\.py' >/dev/null 2>&1 || true" >/dev/null 2>&1 || true
            break
        fi
        if [ $(( $(date +%s) - T_WAIT0 )) -gt 330 ]; then READY_RC=124; break; fi
        sleep 0.5
    done
    set -e
    tail -2 "${READY_LOG}" 2>/dev/null || true
    if [ "${READY_RC}" -eq 0 ]; then
        log "定位链、相机/点云、危险距离与 nav2 两张图均已就绪（第 ${attempt} 次尝试）"
        break
    fi
    log "第 ${attempt} 次启动未就绪"
    # **崩了就别重试**：配置类错误（参数类型不对、插件加载失败）重试三次只是把
    # 15 分钟耗在"等一个永远起不来的东西"上。而且它伪装得很好——承载地图的进程崩掉后，
    # lifecycle_manager 会一直打印 "Waiting for service planner_server/get_state..."，
    # 日志里看起来只是"启动慢"。这里直接去日志里找崩溃特征，命中就立即退出。
    CRASH="$(grep -m3 -E "terminate called|process has died|InvalidParameterType|Traceback|what\(\):" \
             "${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/logs/demo_launch.log" 2>/dev/null || true)"
    if [ -n "${CRASH}" ]; then
        log "** 启动期有进程崩溃，不再重试 **（同一 run-id 的日志见 ${READY_LOG} 旁边的 demo_launch.log）"
        printf '%s\n' "${CRASH}" >&2
        cexec "/work/scripts/rtk-nav2/container/stop_stack.sh '${RUN_ID}' >/dev/null 2>&1 || true"
        exit 1
    fi
    log "未发现崩溃特征，按启动慢处理，重试"
    stop_stack_if_ours "${RUN_ID}"
    sleep 5
done
if [ "${READY_RC}" -ne 0 ]; then
    log "连续 3 次都没能就绪，请看 artifacts/rtk-nav2/${RUN_ID}/logs/ 下的日志"
    exit 1
fi

# 运行时参数证据：两张图的**实际**参数 dump。比配置文件哈希更强——
# 能直接看出"文件改了但运行时没生效"这类不一致（brief §4 要求两张图 yaml 进指纹，
# 这里在指纹之外再加一份运行时快照）。
#
# ★ 2026-09-21 改（§9.8：同一份证据只允许一个写入者 + 生产者保证"空结果不落盘"）：
#   1) **换文件名**：CLI 这份写 `params_cli_*.yaml`。局部图的进程内那份由
#      `local_costmap_host` 写在 `params_local_costmap_local_costmap.yaml`
#      （路径由 start_stack.sh 的 `param_dump_path:=` 传入）。改名前**两者同名**，
#      而 CLI 后写 + `>` 截断 ⇒ **把进程内那份好证据覆盖掉**（实测踩到）。
#   2) **先写临时文件 → 非空校验 → 再改名**：这条命令在本接线下**会超时/返回 0 字节**
#      （见 local_costmap_host.cpp 的说明），旧写法 `2>/dev/null` 吞错后照样 `>`
#      截断目标 ⇒ 把好证据静默截成 0 字节、被误读成"没有参数"。
#   3) 四字段头（机器可判，不靠人读）：produced_by / produced_at（**读出那一刻的墙钟**，
#      不是文件 mtime）/ via（固定枚举）/ run_id。
set +e
NAV2_DIR="${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/nav2"
# ★★ 2026-10-11 补：★ 加上 `/controller_server` ✓ ★★
#   ★ 真因 ✓：★ 阶段十第 7 刀改的是 `desired_linear_vel` ／ `max_angular_accel` ✓
#     —— ★ 这两个键**只落在 `controller_server`** ✓ ⇒ ★ 原来那三张快照**一张都盖不到它** ✗
#     （★ 实测：各轮 `nav2/` 下只有 global／local／planner 三张 ＋ `cmd_vel_publishers.txt` ✓）
#   ⇒ ★ 后果实测 ✓：★ "stock 轮 23–29 个/米 vs 改动轮 50–69 个/米"这条批差 ✓
#     ★ 想分"是不是那两键"时，**手上没有运行时证据** ✗ ⇒ ★ 只能靠再跑一轮对照 ✓。
#   ★ 加它的代价 = 一次 `ros2 param dump`（★ 不应答就按既有分支**跳过、不落盘** ✓，✗ 不截断 ✗）。
for n in /global_costmap/global_costmap /local_costmap/local_costmap /planner_server /controller_server; do
    safe="$(printf '%s' "${n}" | tr '/' '_')"
    tmp="$(mktemp)"
    # ★★ 2026-10-14（步骤②）实测：★ 这里原来只写 `timeout 30` ✗ ⇒ **会永挂** ✗✗
    #   ★ 实测：第 3 次连起时，本行在容器里挂了 **32 分钟** ✗ ——
    #     `ros2 param dump` 是 **Python CLI** ✓，卡在 rclpy 里等一个不应答的服务调用时
    #     **不回字节码层** ⇒ ★ **SIGTERM 被挂着不处理** ✗；而 `timeout` **默认只发 TERM、不发 KILL** ✗
    #     ⇒ ★ 它自己也永远等下去 ⇒ ★ **起栈整条链就停在这儿** ✓（业主看到的"起栈卡住"✓）
    #   ★ 修法 ✓：★ `timeout -k 5 30` ⇒ TERM 后 5 s 补 KILL ⇒ ★ 保证能死 ✓（最坏 35 s 返回 ✓）
    cexec "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && \
           timeout -k 5 30 ros2 param dump ${n} 2>/dev/null" > "${tmp}" 2>/dev/null
    if [ -s "${tmp}" ]; then
        {
            printf '# produced_by: ros2_param_dump_cli\n'
            printf '# produced_at: %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
            printf '# via: cli-daemon\n'
            printf '# run_id: %s\n' "${RUN_ID}"
            cat "${tmp}"
        } > "${NAV2_DIR}/params_cli${safe}.yaml"
        log "运行时快照（CLI，via=cli-daemon）：params_cli${safe}.yaml（$(wc -c < "${tmp}") 字节）"
    else
        log "运行时快照跳过（CLI 返回空，未落盘）：${n} —— 该节点参数服务在本接线下不应答；局部图请看带四字段头的进程内那份"
    fi
    rm -f "${tmp}"
done
# 控制权归属证据（brief §10 裁定 2 的可核查判据）：/cmd_vel 的发布者必须只有 route_follower
# ★ 同一族（2026-10-14 ✓）：★ 这条也补上 `-k 5` ✗ —— 理由与上面那条逐字相同 ✓
cexec "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && \
       timeout -k 5 30 ros2 topic info -v /cmd_vel 2>/dev/null" \
    > "${NAV2_DIR}/cmd_vel_publishers.txt" 2>/dev/null
set -e

# ★★ 2026-10-09（阶段八）**截图判据改成"看 Gazebo GUI 在不在"** ★★
#   原来挂在 `WITH_GUI`（= RViz 开不开）上 ⇒ `--no-rviz` 的**取数档完全没有截图**，
#   于是"两档只差一个 RViz 窗口"这句话**不成立**（reviewer `campus8-c` 实测：screenshots/ 空 ✗）。
#   现在：只要启动参数里没有显式 `gz_gui:=false*` 就抓 —— 因为**Gazebo GUI 才是"画面"的来源**，
#   RViz 有没有只影响画面里多不多一个窗口 ✓。两条路径都会把理由打进日志。
TAKE_SHOT=1
for a in "${LAUNCH_ARGS[@]}"; do
    case "$a" in
        gz_gui:=false|gz_gui:=False|gz_gui:=FALSE|gz_gui:=0) TAKE_SHOT=0 ;;
    esac
done
if [ "${TAKE_SHOT}" -eq 1 ]; then
    capture_screen "${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}/screenshots/01_startup.png" \
        && log "已抓取启动截图 01_startup.png（判据 = Gazebo GUI 在；RViz $([ "${WITH_GUI}" -eq 1 ] && echo 开 || echo 关)）"
else
    log "启动参数里显式关了 Gazebo GUI（gz_gui:=false）⇒ 没有画面可抓，跳过截图"
fi

# ★★ 演示便利（reviewer 2026-10-10，业主点"给脚本加上自动排布窗口"）：★ 把 Gazebo 与 RViz
#   **自动并排** ⇒ 一屏就能同时看见"场景 ＋ 实时代价图"（此前每次都得人手摆、两人摆得还不一样 ✗）。
#   ★ **fail-soft** ✓：没装 wmctrl／窗口没出来／合成器不认 ⇒ 只打印一句，**不影响起栈** ✓。
#   ★ 关掉它：`DEMO_ARRANGE=0` ✓；★ 先看它将执行什么：`scripts/rtk-nav2/arrange-windows.sh --dry-run` ✓。
#   ★ 等待给 15 s（★ 此刻两个窗口通常已就绪 ⇒ 立即返回 ✓；✗ 不设长等待，免得白等 ✗）。
if [ "${DEMO_ARRANGE:-1}" = "1" ]; then
    "$(dirname "${BASH_SOURCE[0]}")/arrange-windows.sh" --wait 15 \
        || log "窗口排布未完成（★ 不影响起栈 ✓；可手动跑 arrange-windows.sh ✓）"
else
    log "DEMO_ARRANGE=0 ⇒ 跳过窗口排布（★ 需要时手动跑 arrange-windows.sh ✓）"
fi

cat <<MSG

第三阶段演示已启动（run-id: ${RUN_ID}）
  产物：${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}
  容器：${CONTAINER_NAME}（ROS_DOMAIN_ID=${ROS_DOMAIN_ID}，GZ_PARTITION=${GZ_PARTITION}）
  镜像：${IMAGE_NAME}
  显示：${DEMO_DISPLAY}

下一步：
  scripts/rtk-nav2/run-route.sh ${RUN_ID}       # 绝对定位驱动的路线跟随（边走边建图）
  scripts/rtk-nav2/plan-to-waypoint.sh ${RUN_ID} # 用 NavFn 规划到下一点并落产物
  scripts/rtk-nav2/save-artifacts.sh ${RUN_ID}  # 保存两张图快照 / 轨迹 / 报告
  scripts/rtk-nav2/stop-demo.sh                 # 停止
MSG
