#!/usr/bin/env bash
# 在容器内取第三阶段的产物：两张图快照 + 全局图 pgm/yaml + 存读一致性核对 + 控制权归属。
# 用法（容器内）：save_artifacts.sh <run_id>
set -o pipefail
set +u

RUN_ID="${1:?用法: save_artifacts.sh <run_id>}"
WS=/work/rtk_nav2_ws
OUT="/work/artifacts/rtk-nav2/${RUN_ID}"
NAV2_DIR="${OUT}/nav2"
COV_DIR="${OUT}/coverage"
mkdir -p "${NAV2_DIR}" "${COV_DIR}"
SCRIPTS="${WS}/src/robonix_rtk_nav2/scripts"
PKG_SHARE="${WS}/install/robonix_rtk_nav2/share/robonix_rtk_nav2"

. /opt/ros/humble/setup.bash
. "${WS}/install/setup.bash"

echo "[容器内] 取第三阶段产物 → ${OUT}"

echo "  - 两张图元数据与计数（存图**前**）"
python3 "${SCRIPTS}/nav2_probe.py" stats --json "${NAV2_DIR}/map_stats_before.json" \
    > "${NAV2_DIR}/map_stats_before.txt" 2>&1 || echo "    （stats 失败，见 map_stats_before.txt）"

echo "  - 几何核对（逐格 + 近墙回归）"
python3 "${SCRIPTS}/nav2_probe.py" geometry --json "${NAV2_DIR}/geometry.json" \
    --dump-snapshot "${NAV2_DIR}/global_snapshot.npz" \
    > "${NAV2_DIR}/geometry.txt" 2>&1 || echo "    （geometry 失败，见 geometry.txt）"

echo "  - 局部图快照"
python3 "${SCRIPTS}/nav2_probe.py" localmap --json "${NAV2_DIR}/local_stats.json" \
    --snapshot "${NAV2_DIR}/local_snapshot.npz" \
    > "${NAV2_DIR}/local_stats.txt" 2>&1 || echo "    （localmap 失败）"

echo "  - 全局图存成 pgm+yaml（nav2 map_saver）"
# **必须显式给 --free 0.196，不能用 map_saver 的默认 0.25**：
# map_saver 把"未知"写成像素 205，而它写进 yaml 的默认 free_thresh 是 0.25；
# nav2 的 map_server 读回来用 occ = (255-pixel)/255 与那两个阈值比，
# 205 -> 0.196 < 0.25 -> **被判成空闲**。也就是说默认参数下存一遍再读回来，
# "没去过的地方"会全部变成"可通行"——正是 brief §7 担心的那个方向。
# 这个结论不是推的，是造了一张已知像素值的小图让 nav2 自己读出来的
# （scripts/pgm_convention_probe.py，产物在 nav2/pgm_convention_*.json）：
#   free_thresh 0.196 -> 205 读回 unknown(-1)   ✓
#   free_thresh 0.25  -> 205 读回 free(0)       ✗
timeout 60 ros2 run nav2_map_server map_saver_cli \
    -f "${NAV2_DIR}/global_map" -t /global_costmap/costmap --free 0.196 --occ 0.65 \
    > "${NAV2_DIR}/map_saver.log" 2>&1
echo "    map_saver 退出码 ${?}"

echo "  - 定死 nav2 的 pgm 读取约定（合成小图，两套阈值各测一次）"
timeout 90 python3 "${SCRIPTS}/pgm_convention_probe.py" --free 0.196 \
    --json "${NAV2_DIR}/pgm_convention_free0196.json" \
    > "${NAV2_DIR}/pgm_convention_free0196.txt" 2>&1
pkill -9 -f "[m]ap_server" 2>/dev/null || true
sleep 2
timeout 90 python3 "${SCRIPTS}/pgm_convention_probe.py" --free 0.25 \
    --node-name pgm_convention_server_025 \
    --json "${NAV2_DIR}/pgm_convention_free025.json" \
    > "${NAV2_DIR}/pgm_convention_free025.txt" 2>&1
pkill -9 -f "[m]ap_server" 2>/dev/null || true

echo "  - 两张图元数据与计数（存图**后**，用于存读比对）"
# 为什么存完还要再取一次：全局图是**边跑边长**的，存图前的快照与存进文件的那一帧
# 天然差几十个格子（第一次实测：free 差 49 格）。拿"存完之后"的快照去比，时间上最贴近；
# 再对比前后两次快照就知道这段时间里图有没有在变（不变 = 比对是干净的）。
python3 "${SCRIPTS}/nav2_probe.py" stats --json "${NAV2_DIR}/map_stats.json" \
    > "${NAV2_DIR}/map_stats.txt" 2>&1 || echo "    （stats 失败，见 map_stats.txt）"

echo "  - 存读一致性：纯文件读回，与实时快照比"
python3 "${SCRIPTS}/check_map_files.py" --yaml "${NAV2_DIR}/global_map.yaml" \
    --live-json "${NAV2_DIR}/map_stats.json" --json "${NAV2_DIR}/save_load_check.json" \
    > "${NAV2_DIR}/save_load_check.txt" 2>&1 || echo "    （不一致或读回失败，见 save_load_check.txt）"

echo "  - 存图前后两张快照是否一致（一致 => 上面那次比对不受边跑边长影响）"
python3 - "${NAV2_DIR}/map_stats_before.json" "${NAV2_DIR}/map_stats.json" \
    > "${NAV2_DIR}/save_load_skew.json" 2>&1 <<'PYEOF'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as fh:
    before = json.load(fh)["global"]["counts"]
with open(sys.argv[2], encoding="utf-8") as fh:
    after = json.load(fh)["global"]["counts"]
delta = {k: after[k] - before[k] for k in ("free", "occupied", "unknown")}
out = {
    "before": before,
    "after": after,
    "delta": delta,
    "map_changed_between_snapshots": any(v != 0 for v in delta.values()),
    "note": "两次 stats 分别取在存图前/后。delta 全 0 表示这段时间图没变，"
            "存读比对不受边跑边长影响；否则按 delta 的量级判读计数差异",
}
print(json.dumps(out, ensure_ascii=False, indent=2))
PYEOF

echo "  - 存读回环（nav2 map_server 亲自读回自己存的图）"
timeout 120 python3 "${SCRIPTS}/map_roundtrip.py" --yaml "${NAV2_DIR}/global_map.yaml" \
    --live-json "${NAV2_DIR}/map_stats.json" --json "${NAV2_DIR}/map_roundtrip.json" \
    --node-name map_roundtrip_server \
    > "${NAV2_DIR}/map_roundtrip.txt" 2>&1 || echo "    （回环不一致或起不来，见 map_roundtrip.txt）"

# **bag 可读性不在这里查**：本脚本跑的时候 bag 通常还在录，metadata.yaml 要到录制
# 结束才写，"现在查"永远是 false（我第一版就这么错了）。改由 stop 之后跑的
# scripts/rtk-nav2/check-bag.sh 负责。
echo "  - （bag 可读性见 stop 后执行的 scripts/rtk-nav2/check-bag.sh）"

echo "  - 控制权归属：/cmd_vel 的发布者"
timeout 30 ros2 topic info -v /cmd_vel > "${NAV2_DIR}/cmd_vel_publishers.txt" 2>&1

echo "  - 两张图与规划器的运行期参数快照"
# 走自写的 param_tool 而不是 `ros2 param dump`：后者在这台机器上受 CLI daemon 影响，
# 实测对 /local_costmap/local_costmap 返回**空文件**（0 字节），而这一步是
# brief §4 要的证据（能看出"文件改了但运行时没生效"），不能静默为空。
for n in /global_costmap/global_costmap /planner_server; do
    safe="$(printf '%s' "${n}" | tr '/' '_')"
    timeout 40 python3 "${SCRIPTS}/param_tool.py" dump "${n}" \
        > "${NAV2_DIR}/params${safe}.yaml" 2>&1
    sz="$(stat -c %s "${NAV2_DIR}/params${safe}.yaml" 2>/dev/null || echo 0)"
    if [ "${sz}" -lt 100 ]; then
        echo "    ** ${n} 的参数快照只有 ${sz} 字节，可能没取到 **"
    else
        echo "    ${n}: ${sz} 字节"
    fi
done
# 局部图那份由宿主节点在 on_activate 时进程内落盘（见 launch 里的说明），
# 这里只检查到没到，不再走外部查询（那条路在本接线下会静默返回空）。
LOCAL_PARAMS="${NAV2_DIR}/params_local_costmap_local_costmap.yaml"
if [ -s "${LOCAL_PARAMS}" ]; then
    echo "    /local_costmap/local_costmap: $(stat -c %s "${LOCAL_PARAMS}") 字节（宿主进程内落盘）"
else
    echo "    ** /local_costmap/local_costmap 的运行期参数没落盘 **"
fi

echo "[容器内] 产物已写入 ${OUT}"
