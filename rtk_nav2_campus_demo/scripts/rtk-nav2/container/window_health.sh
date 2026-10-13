#!/usr/bin/env bash
# 窗口体检（reviewer 2026-10-10 的建议）—— **让窗口自己自证** ✓
#
# ★ 为什么要它：今天大半天的成本**全花在"事后重建窗口是否有效"** ✗
#   （mtime 反推、标记对账、/proc 考古、时钟归零… ✓）⇒ 与其事后判，不如**运行中产出** ✓。
#
# ★ 三个字段组（各自回答闸门的一条）：
#   · `t_epoch` + `sim_clock`            ⇒ 采样时刻 ✓ / 时钟是否在推进 ✓
#   · `instances_by_partition`           ⇒ (a) 属本轮的几套 / 别的分区几套 ✓
#     ★ 逐进程读 `/proc/<pid>/environ` ✓ —— **不受分区口径影响** ✓（进程层 ✓）
#   · `clock_backjump`（累计回退次数）   ⇒ (b) 窗口内有没有发生过重启 ✓
#     ★ 重启时任意时刻实例数都 ≤1 ⇒ **光看 (a) 对重启无感** ✓ ⇒ (b) 必须有 ✓
#
# ★ 它自己也要**两头标定**（照今天的规矩 ✓）：① 健康窗口 ⇒ 零告警 ✓；
#   ② 故意重启一次 ⇒ 必须告警且 `clock_backjump ≥ 1` ✓。
#
# 用法（容器内）：window_health.sh <输出.jsonl> [间隔秒]
set -uo pipefail
OUT="${1:?用法: window_health.sh <输出路径.jsonl> [间隔秒]}"
EVERY="${2:-30}"
mkdir -p "$(dirname "${OUT}")"
# ★★ 2026-10-10：**必须自己 source ROS** ✗ ★★
#   实测（手跑）：`setsid nohup` / `bash -lc` 起的进程里 **`ros2` 不在 PATH** ✗ ⇒
#   `sim_clock` **永远是 null** ✗ ⇒ 按三态设计 ⇒ `status` **永远是 `undetermined`** ✗✗ ⇒
#   ★ **判据恒报警 ⇒ 没人再看它** ✗（正是"比没有更坏"那一类 ✓）。
#   ★★ 再一条实测教训：**`set -u` 下 source ROS 会直接把 shell 打死** ✗ ★★
#     （ROS 的 setup.bash 引用了未定义变量 ✓）⇒ 连"开机第一行"都写不出去 ✗（文件根本不存在 ✓）。
#     ⇒ 定式：**source 前后临时 `set +u`** ✓（`|| true` 挡不住"shell 退出"✗）。
set +u
. /opt/ros/humble/setup.bash 2>/dev/null || true
. /work/${WORKSPACE_NAME:-rtk_nav2_ws}/install/setup.bash 2>/dev/null || true
set -u
ROUND="$(cat /work/artifacts/rtk-nav2/.current-run 2>/dev/null || true)"
MINE="rtknav2_${ROUND}"
# ★★ 2026-10-10（reviewer）：**开机第一件事就把"我要写到哪"落盘** ✓ ★★
#   为什么：实测 `campus8-n1` 只留下 `logs/window_health.pid`、**没有 `.jsonl`** ✗ ⇒
#   "**看门狗活着但没产物**" 与 "**它压根没起来**" **无法区分** ✗ ——
#   又一次"看不见被当成没发生" ✓。⇒ 起手先落一行 ✓（**这一行能落盘 ⇒ 脚本确实起来了** ✓）。
# ★★ 2026-10-10（reviewer）：**判据的"约定"必须写下来** ✗ ★★
#   实测教训：reviewer 为了复算 `mine_set_sha256` **穷举了三种约定** ✓
#   （原串无尾换行 ✓ / 带 `\n` ✗ / 带尾空格 ✗）—— **猜中的** ✗。
#   ⇒ ★ 规则：凡落盘一个哈希，**同时写清"对什么、怎么规范化"** ✓（否则下一个复算方还得穷举 ✗）。
#   ⇒ 本字段 `sha256_convention` **随开机声明行落一次** ✓（每行都带太吵 ✗）。
# ★★ 2026-10-10（reviewer）：**启动行要自报本脚本的 sha256** ✓ ★★
#   为什么：实测本轮 `campus8-n1` 跨了两次尝试（`demo_launch.lstart`=05:10:14 ✓ 而驱动起于 05:03:56 ✓），
#   且 `start_stack.sh` mtime=05:09:40 ✓ ⇒ ★ **两次尝试用的是两份不同的代码** ✗ ⇒
#   "在跑的是哪份代码"**从产物看不出来** ✗（reviewer 差点因此误判 ✓）。
#   ★ 铁律：**判据文件自己被改过，判据就当没跑过** ✓ —— 所以它必须**自报哈希** ✓，
#     复算方拿产物里的哈希 vs 盘上哈希一比 ⇒ 当场知道"这一轮跑的是哪一版" ✓。
_SELF_SHA="$(sha256sum /work/scripts/rtk-nav2/container/window_health.sh 2>/dev/null | cut -d' ' -f1)"
printf '{"t_epoch": %s, "status": "startup", "declares_out": "%s", "interval_s": %s, "mine": "%s", "script_sha256": "%s", "sha256_convention": "mine_set_sha256 = sha256(mine_set raw string, no trailing newline, space-separated pids sorted)"}\n' \
    "$(date +%s.%N)" "${OUT}" "${EVERY}" "${MINE}" "${_SELF_SHA}" >> "${OUT}" 2>/dev/null || true
prev_sim=""; backjump=0
# ★★ 2026-10-10：**`set -u` 下未初始化的变量会把整个看门狗打死** ✗ ★★
#   实测（手跑、**stderr 可见** ✓）：`line 84: window_start: unbound variable` ⇒ rc=1 ⇒
#   **第一拍就退出** ✗ ⇒ 本轮**一个 `window_health.jsonl` 都没有** ✗。
#   ★ 而且它是**被"修另一个 bug"引进来的** ✓：`window_start` 是"窗口起点"机制（防"每轮恒报警"）✓，
#     但**只加了读、没加初始化** ✗ ⇒ 修好了一个坑、埋了个更狠的 ✓。
#   ★ 之所以一整天没人发现：stderr 进了 `/dev/null` ✗（**已改**：落 `logs/window_health.err` ✓）。
#   ⇒ 三个"跨拍/窗口"变量一律**先声明** ✓（`window_start` 要"首次看到本分区时"才赋值 ✓，
#     `prev_set`/`prev_epoch` **首拍本来就没有上一拍** ✓）✗。
window_start=""; prev_set=""; prev_epoch=""; gap=0
while :; do
    now="$(date +%s.%N)"
    # sim 时钟：直接读 gz 的 /clock（拿不到就记 null ⇒ 不假装有数 ✓）
    # ★ 从 **ROS 侧**读 `/clock` ✓ —— 桥接话题 ✓，归 `ROS_DOMAIN_ID` 管 ✓（容器级固定 ✓）
    #   ✗ 不用 `ign topic` ✗ —— 那是 **gz 客户端**路，正是 `wait_ready.py` 被分区口径骗死的那条 ✓
    sim="$(timeout 6 ros2 topic echo --once /clock --field clock 2>/dev/null | grep -m1 'sec:' | awk '{print $2}' || true)"
    [ -n "${sim}" ] || sim="$(timeout 6 ros2 topic echo --once /clock 2>/dev/null | grep -m1 'sec:' | awk '{print $2}' || true)"
    [ -n "${sim}" ] || sim="null"
    # 逐进程归属（进程层 ✓，与分区口径无关 ✓）
    bypart="$(for p in $(pgrep -f '[i]gn gazebo' 2>/dev/null); do
                tr '\0' '\n' < "/proc/${p}/environ" 2>/dev/null | sed -n 's/^GZ_PARTITION=//p'
              done | sort | uniq -c | awk '{printf "%s:%s ", $2, $1}')"
    # 回退计数（(b) 的观测量 ✓）
    if [ "${sim}" != "null" ] && [ -n "${prev_sim}" ]; then
        awk -v a="${prev_sim}" -v b="${sim}" 'BEGIN{exit !(b+0 < a+0)}' && backjump=$((backjump+1))
    fi
    # ★ 同一形态（`A && 赋值`）⇒ 改 `if` ✓（行为不变，但不再有"静默"的形状 ✓）
    if [ "${sim}" != "null" ]; then prev_sim="${sim}"; fi
    # ★★ 更硬的量：**那套仿真的"身份"变没变**（reviewer 2026-10-10）★★
    #   为什么 `clock_backjump` 不够：判回退靠比较**相邻两拍** ✗ ⇒
    #   旧实例只跑了 10 s（那拍读到 10 ✓）⇒ 重启 ⇒ 下一拍已在 30 s 后、新实例到 30 ✓
    #   ⇒ **30 > 10 ⇒ 看不到回退** ✗✗ —— 采样间隔 ≪ 最短关心时长 才安全 ✗，
    #   而"重启随时可能发生"恰恰意味着这个时长**没有下界** ✗。
    #   ⇒ ★ 换"**身份变化**"这个**事件**本身 ✓：本轮分区那套的 `starttime`（不可逆、无盲点 ✓）
    # ★★ 判"**集合恒定性**"（reviewer 2026-10-10）★★ —— 它同时覆盖 (a) (b) 与残留 ✓
    #   `mine_pid_starttime` 只是这个集合的**一个投影** ✓；看集合 ⇒ 重启/缝隙/多套一次全覆盖 ✓。
    mine_set=""; other_set=""
    for p in $(pgrep -f '[i]gn gazebo' 2>/dev/null); do
        _part="$(tr '\0' '\n' < "/proc/${p}/environ" 2>/dev/null | sed -n 's/^GZ_PARTITION=//p')"
        # ★ `/proc/<pid>/stat` 第 22 字段 = starttime（jiffies，全精度 ✓）
        #   ✗ 不用 `ps -o lstart`（秒级 ⇒ 同秒两套会被看成"一起生的" ✗ ✓）
        _st="$(awk '{print $22}' "/proc/${p}/stat" 2>/dev/null)"
        if [ "${_part}" = "${MINE}" ]; then
            mine_set="${mine_set}${p}:${_st} "
        elif [ -n "${_part}" ]; then
            other_set="${other_set}${_part}/${p}:${_st} "
        fi
    done
    mine_set="$(printf '%s' "${mine_set}" | tr ' ' '\n' | sort | tr '\n' ' ' | sed 's/ *$//')"
    other_set="$(printf '%s' "${other_set}" | tr ' ' '\n' | sort | tr '\n' ' ' | sed 's/ *$//')"
    # ★★ 读空必须落 `undetermined` ✓ —— ✗ 不许落成"没变" ✗
    #   （那正是把"**看不见**"当成"**没发生**" ✓ —— 今天栽过的那一类 ✓）
    # ★★ "空"有**两个来源**（reviewer 2026-10-10）：★ **启动期也是空** ✗
    #   体检随栈一起起 ✓ ⇒ 仿真还没起来的那几拍 `mine_set` 就是空 ✓
    #   ⇒ 若一律判 empty/undetermined ⇒ **每一轮都恒报警** ✗✗ ⇒ **没人再看它** ✗（比没有更坏 ✓）
    #   ⇒ 给判据一条**明确的窗口起点** ✓：首次看到本分区那套 = `window_start` ✓（之前的空归启动期 ✓）
    if [ -z "${mine_set}" ]; then
        if [ -z "${window_start}" ]; then
            set_state="startup"    # ★ 窗口尚未开始 ⇒ **不参与判据** ✓
        else
            set_state="empty"      # ★ 窗口内的空 ⇒ **缝隙或真死** ⇒ undetermined ✓
        fi
    else
        if [ -z "${window_start}" ]; then
            window_start="${now}"
            set_state="first"
        elif [ "${mine_set}" = "${prev_set}" ]; then
            set_state="same"
        else
            set_state="changed"    # ⇒ restart ✗
        fi
    fi
    # ★★ `A && B` **当赋值用** ⇒ A 为假时静默什么都不做 ✗（与今早那个 `set -e` 坑同族 ✓）
    #   ⇒ 本项目定式：**赋值一律写 `if …; then …; fi`** ✓
    if [ -n "${mine_set}" ]; then prev_set="${mine_set}"; fi
    window_end="${now}"
    identity_restart=0
    if [ "${set_state}" = "changed" ]; then identity_restart=1; fi
    # ★★ (b) 必须**三态**（reviewer 2026-10-10）：★ "没有回退"也可能只是"**没在测**" ✗
    #   · sim_clock 在推进 ∧ 无回退 ⇒ `no_restart` ✓（可用 ✓）
    #   · 出现回退               ⇒ `restart` ✗（窗口作废 ✓）
    #   · t_epoch 在走而 sim_clock 不走 ⇒ ★ `undetermined` ✗（**标红，不许算通过** ✓）
    # ★★ 三态（reviewer）：集合空 ⇒ `undetermined` ✗（"看不见" ≠ "没发生" ✓）
    if [ "${set_state}" = "startup" ]; then status="startup";      # ★ 窗口还没开 ⇒ **不判** ✓
    elif [ "${sim}" = "null" ] || [ "${set_state}" = "empty" ] || [ "${gap}" = "1" ]; then status="undetermined"
    else status="no_restart"; fi
    if [ "${identity_restart}" = "1" ] && [ "${status}" = "no_restart" ]; then status="restart"; fi
    # ★ 交叉核对：集合说"变"而时钟没回退（或反之）⇒ **不硬判** ✓
    if [ "${backjump}" -gt 0 ] && [ "${identity_restart}" != "1" ] && [ "${status}" = "no_restart" ]; then
        status="undetermined"
    fi
    # ★★ 缺口判据（reviewer 2026-10-10）：★ **按相邻两拍的 `t_epoch` 间隔**判 ✓，别用"总行数 vs 区间长度" ✗
    #   （后者要定容差 ⇒ 又多一个旋钮 ✗；按间隔判还能**指出缺在哪** ✓，信息更多 ✓）
    #   不变量：体检每 `EVERY` 秒该落一行 ⇒ 相邻间隔 ≤ 2×EVERY 才算覆盖到 ✓。
    gap=0
    if [ -n "${prev_epoch}" ]; then
        awk -v a="${prev_epoch}" -v b="${now}" -v e="${EVERY}" 'BEGIN{exit !((b-a) > 2*e)}' && gap=1
    fi
    prev_epoch="${now}"
    # ★★ 2026-10-10（reviewer）：**别修那个单值投影，直接删掉它** ✗ ★★
    #   理由（reviewer 读码 + 我 trace 实测）：① 它是**人为降维** ✗；
    #   ② 本轮集合里**天然有两个 `starttime`**（`…786`×2 / `…802`×2 ✓）⇒ "取哪一个"**无定义** ✗
    #      ⇒ 落盘只会给出一个**歧义值** ✓（正是当初改判"集合"而不是"单值"的原因 ✓）。
    #   ★ 真问题不是投影，是 —— **`mine_set` 算出来了却没落盘** ✗ ⇒
    #     "集合恒定性"这条判据**在产物里无法复算** ✗（只看得见 `status=no_restart` ✓、看不见证据 ✗）
    #     = "**算出来了 ≠ 交出去了**" ✗（今天**第二次**现形 ✓）。
    #   ⇒ 改落三件：★ **规范串 `sha256`** ✓ ＋ **成员数** ✓ ＋ **原串** ✓
    #     ⇒ 复算方**比对相邻两拍的哈希**即可独立复算"集合变没变" ✓✓，不必采信我的转述 ✓。
    _ms="${mine_set:-}"
    _ms_n=0
    if [ -n "${_ms}" ]; then _ms_n="$(printf '%s' "${_ms}" | wc -w)"; fi
    _ms_sha="$(printf '%s' "${_ms}" | sha256sum | cut -d' ' -f1)"
    # ★★★ 2026-10-10（reviewer）：**"缺口是单拍的"会让"看末行"得出相反结论** ✗ ★★★
    #   `gap=0` 每拍重置 ✓ ⇒ 缺口**只影响它那一拍** ✓ ⇒ 之后 `status` 回到 `no_restart` ✓
    #   ⇒ ★ **末行读起来是"窗口干净"** ✗，而真相是"**这一轮中间断过**" ✗。
    #   ⇒ 判据本身没错 ✓（清单写的是"`status` **全程**非 `undetermined`" ✓），
    #     ★ 但**最自然的读法（只看末行）会得出相反结论** ✗ ⇒ 这正是"别让读者靠猜" ✓。
    #   ⇒ ★★ 修法：**把"曾经"累积成两个旗标，并**每拍都落** ✓** ⇒
    #     ★ **末行即全程** ✓ ⇒ "这一轮能不能用"变成**一个字段** ✓、甚至不必扫全程 ✓。
    if [ "${status}" = "undetermined" ]; then ever_undetermined=1; fi
    if [ "${gap}" = "1" ]; then ever_gap=1; fi
    printf '{"t_epoch": %.3f, "sim_clock": %s, "instances_by_partition": "%s", "clock_backjump": %d, "mine_set_sha256": "%s", "mine_set_n": %d, "mine_set": "%s", "status": "%s", "ever_undetermined": %s, "ever_gap": %s, "mine": "%s"}\n' \
        "${now}" "${sim}" "${bypart% }" "${backjump}" "${_ms_sha}" "${_ms_n}" "${_ms}" "${status}" "${ever_undetermined:-0}" "${ever_gap:-0}" "${MINE}" >> "${OUT}"
    sleep "${EVERY}"
done
