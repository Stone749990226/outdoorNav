#!/usr/bin/env bash
# 全局图"还吃不吃新输入"的 A/B（2026-09-22；对应 §8.8 那条未收口的现象）。
# 判据：**在相机光轴前方 ~2 m 放一个箱**，看四张图的哈希有没有变；
#   **全局图变了 ⇒ 它在消费新输入 ✓**；**只有局部图变 ⇒ 问题只夹在全局图这一侧 ✗**。
# 为什么必须放箱、不能只看"图变不变"：**车不动 + 场景不变时不变是正确行为**
#   （本轮实测：60 s 窗口内车位姿 max_disp 0.096 m ⇒ 那段"不变"是预期的，不是冻结）。
# 全部用**现成工具**（pickbox / longseq / spawn_obstacle），不另写探针。
set +u
P="$HOME/projects/robonix"; C=robonix-rtk-nav2-demo; D=/usr/bin/docker
R="$P/artifacts/rtk-nav2/odomframe-1"; RC=/work/artifacts/rtk-nav2/odomframe-1
F="$R/freeze2"; mkdir -p "$F"
NAS='source /opt/ros/humble/setup.bash && source /work/rtk_nav2_ws/install/setup.bash'
LS="python3 /work/scripts/rtk-nav2/container/longseq.py"

longseq() {  # longseq <名字> [秒数]
    $D exec -d $C bash -lc "$NAS && cd /work/rtk_nav2_ws && timeout 120 $LS --seconds ${2:-20} --t0 0 --label $1 --out-csv $RC/freeze2/ls_$1.csv --json $RC/freeze2/ls_$1.json > $RC/logs/ls_$1.log 2>&1"
    for _ in $(seq 1 90); do [ -f "$F/ls_$1.json" ] && break; sleep 2; done
    [ -f "$F/ls_$1.json" ] && echo "  [$1] 落盘 ✓" || { echo "  [$1] **没落盘**"; tail -3 "$R/logs/ls_$1.log"; }
}

echo "###### 0) 选点（相机光轴前方 ~2 m、图上 free、该方位无回波）"
$D exec $C bash -lc "$NAS && timeout 150 python3 /work/scripts/rtk-nav2/container/pickbox.py --dist 2.0 --max-offset 20 --step 5 --out $RC/freeze2/pickbox.json" > "$F/pickbox.txt" 2>&1
tail -6 "$F/pickbox.txt"
BOX="$(grep -m1 '^BOX_XY=' "$F/pickbox.txt" | cut -d= -f2)"
if [ -z "$BOX" ]; then echo "** 没选到点 ⇒ 本项未测，如实记 **"; exit 1; fi
BX="${BOX%%,*}"; BY="${BOX##*,}"; echo "  箱位：($BX, $BY)"

echo "###### 1) 放箱前 20 s"
longseq before 20

echo "###### 2) 放箱"
$D exec $C bash -lc "$NAS && bash /work/scripts/rtk-nav2/container/spawn_obstacle.sh spawn $BX $BY 0.4 1.0" 2>&1 | tail -1
sleep 12
echo "###### 3) 有箱 20 s"
longseq with 20

echo "###### 4) 移箱"
$D exec $C bash -lc "$NAS && bash /work/scripts/rtk-nav2/container/spawn_obstacle.sh remove" 2>&1 | tail -1
sleep 12
echo "###### 5) 移箱后 20 s"
longseq after 20

echo "###### 6) 三态哈希对比（**按 tag 分列**，不合并）"
python3 - "$F" <<'PY'
import csv, json, os, sys
F = sys.argv[1]
def hs(name, tag):
    p = os.path.join(F, "ls_%s.csv" % name)
    if not os.path.isfile(p): return None
    v = [r["hash"] for r in csv.DictReader(open(p)) if r["tag"] == tag]
    return (len(v), len(set(v)), set(v)) if v else None
for tag in ("g_raw", "g_inf", "l_raw", "l_inf"):
    b, w, a = hs("before", tag), hs("with", tag), hs("after", tag)
    def fmt(x): return "无产物" if x is None else "%d 帧/%d 不同" % (x[0], x[1])
    line = "  %-6s 放箱前 %-14s 有箱 %-14s 移箱后 %-14s" % (tag, fmt(b), fmt(w), fmt(a))
    if b and w:
        changed = bool(b[2] & w[2]) is False
        line += " ⇒ %s" % ("**哈希集合完全不同（图确实变了）**" if changed else "哈希集合有交集（变化不明显）")
    print(line)
PY
echo "  箱位记录：$BOX" | tee "$F/box_xy.txt"
