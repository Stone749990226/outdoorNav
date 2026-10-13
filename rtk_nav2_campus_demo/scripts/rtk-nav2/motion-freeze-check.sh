#!/usr/bin/env bash
# 补做"运动窗口"的冻结核对（2026-09-22）。**为什么单独一份**：
# 第一版把它写在多层 heredoc 套嵌的脚本里，`--t0` 取到空串 ⇒ longseq.py 直接 argparse 报错退出，
# 而那句报错落在容器内的日志里、外层只看到"没有产物"⇒ 差点被读成"跑了但没冻" ✗。
# 教训：**多层 heredoc 里拼的脚本，自己就是"又一个写入者"**；短脚本一律本地写好再传。
set +u
P="$HOME/projects/robonix"; C=robonix-rtk-nav2-demo; D=/usr/bin/docker
R="$P/artifacts/rtk-nav2/odomframe-1"; RC=/work/artifacts/rtk-nav2/odomframe-1
NAS='source /opt/ros/humble/setup.bash && source /work/rtk_nav2_ws/install/setup.bash'

# t0：`.stack_t0` 是**别的驱动**写的（r1/r2/r4_driver 那一族），本 run 没有 ⇒ 用 run_state 的 mtime 兜底，
# 且**绝不允许变成空串**（空串正是上一次失败的根因）。
if [ -f "$R/.stack_t0" ]; then T0="$(cat "$R/.stack_t0")"; else T0="$(stat -c %Y "$R/run_state.txt" 2>/dev/null || date +%s)"; fi
T0="${T0:-0}"
echo "t0=$T0（$(date -d @$T0 '+%H:%M:%S' 2>/dev/null || echo '?')）"

echo "###### A) 起 60 s 长序列（运动窗口用）"
$D exec -d $C bash -lc "$NAS && cd /work/rtk_nav2_ws && timeout 200 python3 /work/scripts/rtk-nav2/container/longseq.py --seconds 60 --t0 $T0 --label motion --out-csv $RC/ring/longseq_motion.csv --json $RC/ring/longseq_motion.json > $RC/logs/longseq_motion.log 2>&1"
sleep 8
[ -f "$R/logs/longseq_motion.log" ] && head -3 "$R/logs/longseq_motion.log"
GOAL_EPOCH=$(date +%s)

echo "###### B) 让车开起来：回目标 (2.6,-2.6)（离当前位置 5.2 m）"
"$P/scripts/rtk-nav2/nav-to.sh" odomframe-1 2.6 -2.6 2>&1 | tail -3

echo "###### C) 等长序列落盘"
for _ in $(seq 1 90); do [ -f "$R/ring/longseq_motion.json" ] && break; sleep 2; done
if [ ! -f "$R/ring/longseq_motion.json" ]; then
    echo "** longseq_motion.json 没落盘 ⇒ 运动窗口这一项按『未测』记 **"
    tail -5 "$R/logs/longseq_motion.log" 2>/dev/null
    exit 1
fi
echo "  已落盘 ✓"

echo "###### D) 恒定段（**只在"车在动"的窗口里评**：从发目标那一刻起）"
python3 - "$R/ring" "$GOAL_EPOCH" <<'PY'
import csv, json, os, sys, time
R, goal = sys.argv[1], float(sys.argv[2])
rows = list(csv.DictReader(open(os.path.join(R, "longseq_motion.csv"))))
print("  发目标墙钟 %s；序列墙钟跨度 %s → %s"
      % (time.strftime("%H:%M:%S", time.localtime(goal)),
         time.strftime("%H:%M:%S", time.localtime(float(min(r["wall"] for r in rows)))),
         time.strftime("%H:%M:%S", time.localtime(float(max(r["wall"] for r in rows))))))
for tag in ("g_raw", "g_inf", "l_raw", "l_inf"):
    sub = sorted([r for r in rows if r["tag"] == tag], key=lambda r: float(r["wall"]))
    sub = [r for r in sub if float(r["wall"]) >= goal]      # ← 只保留"车在动"的窗口
    if not sub:
        print("  %-6s 窗口内一帧都没有 ⇒ 未测" % tag); continue
    runs, i, n = [], 0, len(sub)
    while i < n:
        j = i
        while j + 1 < n and sub[j+1]["hash"] == sub[i]["hash"]: j += 1
        runs.append((j-i+1, float(sub[i]["sim_hdr"]), float(sub[j]["sim_hdr"])))
        i = j+1
    ge3 = [r for r in runs if r[0] >= 3]
    longest = max(runs, key=lambda r: r[0])
    print("  %-6s 帧数=%-4d 不同哈希=%-4d 最长恒等段=%d 连续>=3 段数=%d ⇒ %s"
          % (tag, len(sub), len({r["hash"] for r in sub}), longest[0], len(ge3),
             "**有恒定段 ⇒ 又冻了**" if ge3 else "**无恒定段**"))
    for r in ge3:
        print("         恒定段 %d 样本 sim %.2f→%.2f" % r)
PY

echo "###### E) drive2 阶段：真值位移（独立于里程计）与原点跟随"
python3 - "$R/ring" <<'PY'
import json, math, os, sys
R = sys.argv[1]
j = json.load(open(os.path.join(R, "ring_drive2.json")))
gt = [s["gt_pose"] for s in j["samples"] if s.get("gt_pose")]
od = [s["odom_pose"] for s in j["samples"] if s.get("odom_pose")]
lo = j["local_origin_span"]
if gt:
    d = math.dist((gt[0][0], gt[0][1]), (gt[-1][0], gt[-1][1]))
    print("  真值：(%+.2f,%+.2f) → (%+.2f,%+.2f)  位移 %.2f m ⇒ %s"
          % (gt[0][0], gt[0][1], gt[-1][0], gt[-1][1], d,
             "**覆盖到了：车确实走了**" if d >= 1.0 else "**未覆盖**"))
if od and lo["x"]:
    dorg = math.dist((lo["x"][0], lo["y"][0]), (lo["x"][-1], lo["y"][-1]))
    dpos = math.dist((od[0][0], od[0][1]), (od[-1][0], od[-1][1]))
    print("  **原点跟随核对**：odom 车位位移 %.2f m，局部图原点位移 %.2f m（差 %.2f m，应 ≈0）"
          % (dpos, dorg, abs(dpos - dorg)))
PY
