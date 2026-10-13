#!/usr/bin/env bash
# 局部图 `global_frame: base_link` → `odom` 的轻量验收（2026-09-22，AuYang 拍板那一行）。
#
# 判据（AuYang 原话）：**原地转一圈 ⇒ 局部图上不再出现那个圈** ✓；**给个目标点，车照常能走** ✓；
#                     **指纹变了照例给我** ✓。
#
# ★ 为什么"圈"要落成**图**而不是只落成计数：**圈是个形状**。转一圈本来就会看到新方向上的
#   真障碍 ⇒ 占据格总数**一定会涨一点**，光看总数分不出"看到新东西"和"旧格跟着车体转"。
#   所以本驱动同时落 PNG（给人看形状）+ npz（逐格）+ 计数序列（辅证）。
#
# ★ 另外两条**必须一起测**，否则"圈没了"可以被别的变化冒充：
#   ① **改动真的生效了**：`base_link` 时局部图原点是**结构性常量 (0,0)**；换成 `odom` 后
#      它必须**跟着车走**。这一条从 ring_*.json 的 `local_origin_span` 直接读 ✓（"改过 ≠ 生效"）。
#   ② **没把第五阶段修好的东西弄回去**：R1 的判据是"四张图最长恒定段 = 0"。
#      本驱动后台跑一遍 `longseq.py` 并按**逐帧**重算拐点 ✓（防"换帧把冻结换回来"）。
#
# 用法：scripts/rtk-nav2/odom-ring-check.sh <run-id>
#   **要求栈已经在跑**（先 scripts/rtk-nav2/start-demo.sh <run-id>）。
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"

R="$(artifact_dir "${RUN_ID}")"          # 宿主侧路径
RC="$(artifact_dir_c "${RUN_ID}")"       # 容器内路径
mkdir -p "${R}/logs" "${R}/ring"
START_EPOCH="$(date +%s)"
# ★ `T0` **绝不允许为空串**（2026-09-22 实测踩到）：`.stack_t0` 是 **r1/r2/r4_driver 那一族**写的，
#   按本驱动的起栈路径（`start-demo.sh`）**根本没有这个文件**；更早一版把 `T0` 直接传给
#   `longseq.py --t0`，空串让 argparse 当场报 `expected one argument` —— 而那句报错落在
#   **容器内的日志**里，外层只看到"没有产物" ⇒ 差点被读成"跑了但没冻" ✗。
#   现在：优先 `.stack_t0`；没有就用 `run_state.txt` 的 mtime（= 起栈那一刻）；再兜底当前时间。
if [ -f "${R}/.stack_t0" ]; then
    T0="$(cat "${R}/.stack_t0")"
else
    T0="$(stat -c %Y "${R}/run_state.txt" 2>/dev/null || echo "${START_EPOCH}")"
fi
T0="${T0:-${START_EPOCH}}"
rel() { echo "起栈后 +$(( $(date +%s) - T0 )) s（墙钟）"; }
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"
PROBE="python3 ${CONTAINER_WORKDIR}/scripts/rtk-nav2/container/ring_probe.py"
LS_SECONDS="${LS_SECONDS:-200}"

echo "###### odom 帧轻量验收开始 container-UTC=$(cexec 'date -u +%Y-%m-%dT%H:%M:%SZ')  $(rel)"

# ---------------- 0) 先取本轮的活值：局部图的 global_frame 到底是什么 ----------------
# ★ 本脚本是**宿主侧**脚本 ⇒ **凡是 rclpy 的活都必须包一层 cexec***（宿主上没有 rclpy）。
#   而且 `cexec_bg` 的重定向发生在**容器内** ⇒ 它的日志路径必须是**容器内路径** `${RC}/...`，
#   给宿主路径不会报错，只会把日志写进容器自己的文件系统里 —— **静默丢证据**。
timeout 120 "${DOCKER_BIN}" exec "${CONTAINER_NAME}" bash -lc \
    "${NAS} && timeout 90 python3 ${CONTAINER_WORKDIR}/scripts/rtk-nav2/container/getparams.py \
     /local_costmap/local_costmap global_frame robot_base_frame rolling_window \
     width height resolution" > "${R}/ring/params_local.txt" 2>&1
cat "${R}/ring/params_local.txt"

# ---------------- 1) 后台长序列（冻结回归判据）----------------
cexec_bg "${RC}/logs/longseq.log" \
    "${NAS} && cd ${WS_DIR} && timeout 900 python3 ${CONTAINER_WORKDIR}/scripts/rtk-nav2/container/longseq.py \
     --seconds ${LS_SECONDS} --t0 ${T0:-0} --label 'odom 帧轻量验收：转圈 + 给目标' \
     --out-csv ${RC}/ring/longseq.csv --json ${RC}/ring/longseq.json"
LS_MARK="$(date +%s)"
# ★ 起完**先核一眼它没当场死**（空 `--t0` 那次就是死在这里，而外层只看到"没产物"）：
#   cexec_bg 是 detached，拿不到退出码 ⇒ 用**日志开头几行 + 8 s 后文件在不在**当哨兵。
sleep 8
if [ -f "${R}/logs/longseq.log" ]; then head -3 "${R}/logs/longseq.log"; fi
grep -qiE "error|Traceback|usage:" "${R}/logs/longseq.log" 2>/dev/null && \
    echo "** longseq 的日志里有 error/Traceback/usage ⇒ 它当场死了，冻结回归这一项按『未测』记 **"

probe() {   # probe <阶段名> <秒数> [额外参数...]
    local ph="$1" sec="$2"; shift 2
    local mark; mark="$(date +%s)"
    cexec_bg "${RC}/logs/ring_${ph}.log" \
        "${NAS} && cd ${WS_DIR} && ${PROBE} --run-dir ${RC}/ring --phase ${ph} --seconds ${sec} $*"
    # 等它自己落 JSON（**不靠 sleep 猜**：超时就如实报，不写"通过"）
    local i
    for i in $(seq 1 $(( ${sec%.*} + 60 ))); do
        if [ -f "${R}/ring/ring_${ph}.json" ] && \
           [ "$(stat -c %Y "${R}/ring/ring_${ph}.json")" -ge "${mark}" ]; then
            tail -6 "${R}/logs/ring_${ph}.log"; return 0
        fi
        sleep 1
    done
    echo "** 阶段 ${ph} 超时没落 JSON ⇒ 如实记，本阶段不作数 **"; return 1
}

# ---------------- 2) 静止基线 ----------------
echo; echo "###### 2) 静止基线（车没动，局部图每秒采样两次）$(rel)"
probe baseline 12 --rate-hz 2 || true

# ---------------- 3) 原地转一圈（本驱动自己发 /cmd_vel）----------------
echo; echo "###### 3) 原地转圈：0.4 rad/s × 20 s ≈ 8.0 rad（457°，够一整圈）$(rel)"
echo "    ★ 这一段**没有目标在跑** ⇒ /cmd_vel 只有本驱动一个写入者（nav2 控制器也在发，"
echo "      同时发就成了两个写入者，转速会变成谁后发谁算）"
probe spin 20 --rate-hz 2 --spin-rate 0.4 || true

# ---------------- 4) 转完静止（判"圈有没有留在图上"）----------------
echo; echo "###### 4) 转完静止（车应已停稳；这一段是判圈的主读数）$(rel)"
probe after 14 --rate-hz 2 || true

# ---------------- 5) 给个目标点：车照常能走吗（不回归）----------------
# ★★ 目标点**不能选成车当前位置**（2026-09-22 首轮就踩了这个坑）：车的出生点就是
#   路线起点 `(2.6,-2.6)`，第一版把目标点也写成 `(2.6,-2.6)` ⇒ 导航**当场回 `arrived`、
#    车一米没动**，而屏幕上看起来"通过了"——**判据覆盖的是空集** ✗。
#   ⇒ 现在改成路线**第 4 段的端点 `(-2.6,-2.6)`**（离出生点 5.2 m，净空已验证），
#     并在汇总里**用真值位移判"到底动了没有"**（位移 < 1 m ⇒ 本项记『未覆盖』，不记通过）。
echo; echo "###### 5) 给目标点 (-2.6, -2.6)（路线第 4 段端点，离出生点 5.2 m）$(rel)"
echo "    ★ 转完必须先停：目标阶段 nav2 控制器接管 /cmd_vel，前面那一段的指令已经清零 ✓"
"$(dirname "${BASH_SOURCE[0]}")/nav-to.sh" "${RUN_ID}" -2.6 -2.6 2>&1 | tail -5
probe drive 45 --rate-hz 2 || true
echo "---- 目标阶段结束时的导航状态 ----"
cexec_t 40 "${NAS} && timeout 20 ros2 topic echo --once /rtk/goal_navigator_state" 2>&1 | tail -6

# ---------------- 6) 等长序列结束，重算拐点（逐帧）----------------
echo; echo "###### 6) 长序列结束，重算拐点 $(rel)"
# 判"跑完了没有"用**产物**（longseq.json 是它在收尾时写的），不用 PID：
# cexec_bg 是 `docker exec -d`，拿不到容器内的 PID（宿主上 `kill -0` 那个号是错的进程）。
LS_WAIT_MAX=$(( LS_SECONDS + 90 ))
for _ in $(seq 1 "${LS_WAIT_MAX}"); do
    [ -f "${R}/ring/longseq.json" ] && break
    sleep 2
done
if [ -f "${R}/ring/longseq.json" ]; then
    echo "  longseq.json 已落盘 ✓"
else
    echo "  ** longseq.json 还没落盘 ⇒ 冻结回归这一项按『未测』记（不写成通过）**"
fi
timeout 120 python3 - "${R}/ring" <<'PYEOF' 2>&1 | tee "${R}/ring/onset.txt"
import csv, json, os, sys
R = sys.argv[1]
p = os.path.join(R, "longseq.csv")
if not os.path.isfile(p):
    print("** longseq.csv 不在 ⇒ 冻结回归这一项**未测**，不许写成通过**"); raise SystemExit(0)
rows = list(csv.DictReader(open(p)))

# ★★ "恒定段"判据的**适用前提**（2026-09-22 首轮实测踩到，误报了一次"又冻了"）：
#   该判据问的是"**输入在变的时候，图有没有跟着变**"。**车静止 + 场景无变化**时，
#   图不变是**正确行为** ⇒ 在那种窗口里读出的"恒定段"是**假阳性** ✗。
#   首轮实例：全程唯一的 ≥3 恒定段（11 样本）落在 sim 344.10→350.06，
#   而四个阶段的 sim 跨度是 184→267 ⇒ **它在测试结束后的静置窗里**。
#   ⇒ 修法：把评定**限制在"场景确实在变"的窗口** —— 各阶段 sim 跨度的并集。
WIN = None
spans = []
for ph in ("baseline", "spin", "after", "drive"):
    fp = os.path.join(R, "ring_%s.json" % ph)
    if os.path.isfile(fp):
        s = json.load(open(fp))["samples"]
        if s:
            spans.append((s[0]["sim"], s[-1]["sim"]))
if spans:
    WIN = (min(a for a, _ in spans), max(b for _, b in spans))

def const_runs(sub):
    runs, i, n = [], 0, len(sub)
    while i < n:
        j = i
        while j + 1 < n and sub[j + 1]["hash"] == sub[i]["hash"]:
            j += 1
        runs.append({"n": j - i + 1, "sim0": float(sub[i]["sim_hdr"]),
                     "sim1": float(sub[j]["sim_hdr"])})
        i = j + 1
    return runs

out = {"source": "ring/longseq.csv（逐帧）", "const_min_samples": 3,
       "phase_sim_window": WIN, "tags": {}}
print("=== 冻结回归（逐帧序列；连续 >=3 个相同样本才算恒定段）===")
if WIN:
    print("  ★ 主判据**只看『场景在变』的窗口** sim %.2f→%.2f（= 各阶段跨度并集）；" % WIN)
    print("    整段序列的读数只作参考（含测试结束后的静置窗，那里不变是正确行为）")
for tag in ("g_raw", "g_inf", "l_raw", "l_inf"):
    sub = sorted([r for r in rows if r["tag"] == tag], key=lambda r: float(r["wall"]))
    if not sub:
        out["tags"][tag] = {"error": "该 tag 一帧都没有"}; continue
    all_runs = const_runs(sub)
    inwin = [r for r in sub if WIN and WIN[0] <= float(r["sim_hdr"]) <= WIN[1]] if WIN else []
    win_runs = const_runs(inwin) if inwin else []
    ge3_all = [r for r in all_runs if r["n"] >= 3]
    ge3_win = [r for r in win_runs if r["n"] >= 3]
    longest_win = max(win_runs, key=lambda r: r["n"]) if win_runs else None
    out["tags"][tag] = {
        "frames_all": len(sub), "distinct_all": len({r["hash"] for r in sub}),
        "longest_all": max(r["n"] for r in all_runs),
        "const_ge3_all": len(ge3_all),
        "const_ge3_in_window": len(ge3_win),
        "frames_in_window": len(inwin),
        "longest_in_window": (longest_win["n"] if longest_win else None),
        "verdict": ("**窗口内有恒定段 >=3 ⇒ 又冻了**" if ge3_win else
                    ("**窗口内无恒定段（最长 %d 个样本）**" % longest_win["n"] if longest_win
                     else "窗口内一帧都没有")),
    }
    print("  %-6s 全段：帧数=%-4d 不同哈希=%-4d 最长=%d 连续>=3 段数=%d"
          % (tag, len(sub), len({r["hash"] for r in sub}),
             out["tags"][tag]["longest_all"], len(ge3_all)))
    s = out["tags"][tag]
    print("         窗口内：帧数=%-4d 最长恒等段=%s 连续>=3 段数=%d ⇒ %s"
          % (s["frames_in_window"], s["longest_in_window"], s["const_ge3_in_window"], s["verdict"]))
    for r in ge3_all:
        print("         （全段里的恒定段：%d 样本 sim %.2f→%.2f ⇒ 落在窗口%s）"
              % (r["n"], r["sim0"], r["sim1"],
                 "外（静置窗，属预期，不算冻结）" if WIN and (r["sim1"] < WIN[0] or r["sim0"] > WIN[1]) else "内"))
json.dump(out, open(os.path.join(R, "onset.json"), "w"), ensure_ascii=False, indent=1)
PYEOF

# ---------------- 7) 汇总读数（都从产物里读，不从屏幕上看）----------------
echo; echo "###### 7) 汇总 $(rel)"
timeout 120 python3 - "${R}/ring" <<'PYEOF' 2>&1 | tee "${R}/ring/summary.txt"
import json, math, os, sys
R = sys.argv[1]
def load(ph):
    p = os.path.join(R, "ring_%s.json" % ph)
    return json.load(open(p)) if os.path.isfile(p) else None
PHASES = ("baseline", "spin", "after", "drive", "drive2")
J = {ph: load(ph) for ph in PHASES}
print("=== 逐阶段读数（全部从产物里读，不从屏幕上读）===")
for ph in PHASES:
    j = J[ph]
    if j is None:
        print("  %-9s **该阶段没有产物** ⇒ 未测" % ph); continue
    lo, oc = j["local_origin_span"], j["local_occupied"]
    op = j["odom_pose_span"]
    print("  %-9s frame=%s 样本=%d" % (ph, "、".join(j["local_frame_seen"]), j["samples_n"]))
    print("            局部图原点 x %s  y %s ⇒ **绝对值本身是判据**：base_link 时它恒为 0.000"
          % ("%.3f→%.3f" % (lo["x"][0], lo["x"][-1]) if lo["x"] else "(无)",
             "%.3f→%.3f" % (lo["y"][0], lo["y"][-1]) if lo["y"] else "(无)"))
    print("            局部图占据格 %d → %d（Δ%+d）" % (oc[0], oc[-1], oc[-1] - oc[0]))
    # **原点是否跟着车走**：只在**车真的位移**的阶段才能判（转圈时车位不变 ⇒ 原点本就该不变）
    if lo["x"] and op["x"]:
        dorg = math.dist((lo["x"][0], lo["y"][0]), (lo["x"][-1], lo["y"][-1]))
        dpos = math.dist((op["x"][0], op["y"][0]), (op["x"][-1], op["y"][-1]))
        if dpos >= 0.5:
            print("            **原点跟随核对**：车位移 %.2f m，图原点位移 %.2f m（差 %.2f m，应 ≈0）"
                  % (dpos, dorg, abs(dpos - dorg)))
        else:
            print("            （车本阶段位移 %.2f m < 0.5 m ⇒ 原点跟随**不适用**，不据此下结论）" % dpos)
sp = J["spin"]
if sp is not None:
    print("=== 转圈 ===")
    print("  指令 %s rad（%d 条 /cmd_vel）；**真值解缠总转角 %s rad**；odom 解缠总转角 %s rad"
          % (sp["spin_cmd_total_rad"], sp["spin_cmd_messages"],
             sp["gt_yaw_unwrapped_total_rad"], sp["odom_yaw_unwrapped_total_rad"]))
    if sp["gt_yaw_unwrapped_total_rad"] and sp["odom_yaw_unwrapped_total_rad"]:
        g, o = sp["gt_yaw_unwrapped_total_rad"], sp["odom_yaw_unwrapped_total_rad"]
        if g > 0.5:
            print("  ★ **里程计转角误差**：odom 少报 %.0f%%（真值 %.2f vs odom %.2f rad，差 %.2f rad）"
                  % (100 * (1 - o / g), g, o, g - o))
            print("    ⇒ 含义：局部图的内容是**按 odom 的朝向信念**贴进格子的 ⇒ 转圈过程中同一段墙会被"
                  "贴到相差最多 ~%.2f rad 的格上 —— 这是『帧换了也仍在』的**残留**（比 base_link 的整圈"
                  "闭环小得多，但**不是零**）。判『圈还在不在』仍以 PNG 形状为准。" % abs(g - o))
b, a = J["baseline"], J["after"]
if b is not None and a is not None:
    print("=== 圈（辅证；判据本身是 PNG 的形状）===")
    print("  转前静止末帧占据格 %d → 转后静止末帧 %d（Δ%+d）⇒ 判圈请看 ring_after_last.png："
          "**占据格是否连成一圈环绕车体、且该半径上没有真实障碍**" % (b["local_occupied"][-1], a["local_occupied"][-1], a["local_occupied"][-1] - b["local_occupied"][-1]))
    print("  计数上升**不能**单独判圈：转一圈看到新方向上的真障碍也会增加（本场景南边就有 obstacle_02）")
print("=== 车照常能走（守卫：位移 < 1 m ⇒ 本项**未覆盖**，不记通过）===")
moved = False
for ph in ("drive", "drive2"):
    j = J[ph]
    if j is None: continue
    gt = [s["gt_pose"] for s in j["samples"] if s.get("gt_pose")]
    if not gt:
        print("  %-7s **真值位姿一个样本都没有 ⇒ 未测**" % ph); continue
    d = math.dist((gt[0][0], gt[0][1]), (gt[-1][0], gt[-1][1]))
    ok = d >= 1.0
    moved = moved or ok
    print("  %-7s 真值位移 %.2f m（(%0.2f,%0.2f) → (%0.2f,%0.2f)）⇒ %s"
          % (ph, d, gt[0][0], gt[0][1], gt[-1][0], gt[-1][1],
             "**覆盖到了：车确实走了**" if ok else "**未覆盖：位移 < 1 m（目标点很可能就是车的当前位置）**"))
if not moved:
    print("  **⇒ 本项整体『未覆盖』：没有任何一轮让车真的动起来 ✗ 不许写成通过**")
PYEOF

# ---------------- 8) 产物自检（四件）----------------
# PNG 是二进制 ⇒ **不进 --require**（第 ④ 件结构完整性按"文本"规则查，二进制会被误判），
# 只登记（--optional）✓。JSON/CSV/TXT 进 --require。
echo; echo "###### 8) 产物自检四件 $(rel)"
# ★ 包进容器跑：done_gate.py 只用 stdlib、宿主也能跑，但**本轮所有取证工具都跑在容器里**
#   ⇒ 口径统一（"这个数是在哪个环境里算出来的"要能一句话说清，见 §9.1 的"在哪台机器/哪一侧"）。
cexec_t 300 "${NAS} && cd ${WS_DIR} && timeout 240 python3 ${CONTAINER_WORKDIR}/scripts/rtk-nav2/container/done_gate.py \
    --run-dir ${RC} --run-id ${RUN_ID} --start-epoch ${START_EPOCH} \
    --label '局部图帧 base_link→odom 的轻量验收：原地转圈 + 目标 + 冻结回归' \
    --require ring/params_local.txt \
    --require ring/ring_baseline.json --require ring/ring_spin.json \
    --require ring/ring_after.json --require ring/ring_drive.json \
    --require ring/longseq.csv --require ring/longseq.json \
    --require ring/onset.txt --require ring/onset.json \
    --require ring/summary.txt \
    --optional ring/ring_baseline_first.npz --optional ring/ring_baseline_last.npz \
    --optional ring/ring_after_last.npz --optional ring/ring_drive_last.npz \
    --optional ring/ring_spin_last.npz \
    --optional ring/ring_drive2.json --optional ring/ring_drive2_last.png \
    --optional ring/longseq2.csv --optional ring/longseq2.json \
    --optional ring/summary_drive2.txt \
    --optional ring/ring_baseline_last.png --optional ring/ring_after_last.png \
    --optional ring/ring_drive_last.png --optional ring/ring_spin_last.png" \
    > "${R}/ring/done_gate.txt" 2>&1
GATE_RC=$?
tail -40 "${R}/ring/done_gate.txt"
echo; echo "###### 自检退出码 = ${GATE_RC}（0 = 四件全过）$(rel)"

echo; echo "###### 9) 把图回拉到本地看（只回拉这几张，不动 artifacts 树）$(rel)"
echo "   （PNG/npz 留在 6002 —— artifacts 的事实主副本在远端）"
echo "验收驱动结束 $(rel)"
exit "${GATE_RC}"
