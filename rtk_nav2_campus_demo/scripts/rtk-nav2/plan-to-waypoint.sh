#!/usr/bin/env bash
# 用 nav2 的 NavFn 规划一条路径，落产物并做几何核对（第三阶段 3.2 的入口）。
#
# 用法：scripts/rtk-nav2/plan-to-waypoint.sh [run-id] [--goal X Y] [--start X Y] [--label 说明]
#   不给 --goal 时默认规划到**路线上的下一个航点**（方环 (2.6,-2.6) -> (2.6,2.6) -> ...）：
#   从机器人当前位姿算出它正在追的那个点，就是"路线下一个航点"。
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID=""
GOAL=()
START=()
LABEL=""
while [ $# -gt 0 ]; do
    case "$1" in
        --goal) GOAL=("$2" "$3"); shift 3 ;;
        --start) START=("$2" "$3"); shift 3 ;;
        --label) LABEL="$2"; shift 2 ;;
        -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
        *) RUN_ID="$1"; shift ;;
    esac
done
RUN_ID="$(resolve_run_id "${RUN_ID}")"
stack_running || die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"
NAV2_DIR="$(artifact_dir "${RUN_ID}")/nav2"
# **`--json` 与 `--csv` 必须给容器视角的路径**（`/work/...`）。
# 这里是宿主路径，容器里那个目录不存在——而 `plan_path.py` 写 CSV 时抛 FileNotFoundError、
# 脚本非零退出，**JSON 与 RViz 的 /rtk/plan_path 却都正常**，所以现象是"只有 plan.csv 从来没产出过"
# （reviewer 复验时指出的）。这与 `run_route.sh` 注释里记的"报告路径必须是容器视角"是同一类错误。
NAV2_DIR_C="$(artifact_dir_c "${RUN_ID}")/nav2"
mkdir -p "${NAV2_DIR}"

# 不给目标点时用路线上的下一个航点（方环四角，与 config/route.yaml 一致）。
# 起点用机器人当前位姿，由 plan_path.py 自己从 TF 取。
ROUTE_X="2.6 -2.6 2.6 2.6 -2.6 2.6 -2.6 -2.6"
if [ "${#GOAL[@]}" -eq 0 ]; then
    read -r CUR_X CUR_Y <<<"$(cexec "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && \
        timeout 20 python3 -c \"
import rclpy, tf2_ros, math
rclpy.init(); n=rclpy.node.Node('pose_probe'); n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
b=tf2_ros.Buffer(); tf2_ros.TransformListener(b,n)
import time; t0=time.time(); p=None
while time.time()-t0<15 and p is None:
    rclpy.spin_once(n, timeout_sec=0.1)
    try: p=b.lookup_transform('map','base_footprint',rclpy.time.Time())
    except Exception: pass
print('%.4f %.4f' % (p.transform.translation.x, p.transform.translation.y))
rclpy.shutdown()\"" 2>/dev/null | tail -1)"
    log "机器人当前位置：(${CUR_X:-?}, ${CUR_Y:-?})"
    # 取路线四角里"还没到过的最近一个"：简单取与当前位置最近但不同的那个角，
    # 也就是说规划到下一点。四角顺序是固定的，取严格大于当前进度的下一个角由
    # route 逻辑决定；这里用**离当前位置最近的、且不在当前位置 0.3 m 内的角**作为目标。
    GOAL_STR="$(python3 - "$CUR_X" "$CUR_Y" <<'PY'
import math, sys
try:
    cx, cy = float(sys.argv[1]), float(sys.argv[2])
except Exception:
    print('2.6 2.6'); raise SystemExit
corners = [(2.6, -2.6), (2.6, 2.6), (-2.6, 2.6), (-2.6, -2.6)]
# 取"按方环顺序、沿环向前的下一个角"：找当前位置最近的角，再取它的下一个
idx = min(range(4), key=lambda i: math.hypot(corners[i][0]-cx, corners[i][1]-cy))
nxt = corners[(idx + 1) % 4]
print('%.2f %.2f' % nxt)
PY
)"
    read -r GOAL_X GOAL_Y <<<"${GOAL_STR}"
    LABEL="${LABEL:-路线下一个航点（方环四角顺序）}"
    log "规划目标（路线下一个航点）：(${GOAL_X}, ${GOAL_Y})"
else
    GOAL_X="${GOAL[0]}"; GOAL_Y="${GOAL[1]}"
fi

ARGS="--goal ${GOAL_X} ${GOAL_Y} --json ${NAV2_DIR_C}/plan.json --csv ${NAV2_DIR_C}/plan.csv --label '${LABEL}'"
[ "${#START[@]}" -gt 0 ] && ARGS="${ARGS} --start ${START[0]} ${START[1]}"
cexec "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash && \
       python3 ${WS_DIR}/src/robonix_rtk_nav2/scripts/plan_path.py ${ARGS}" \
    > "${NAV2_DIR}/plan.txt" 2>&1
RC=$?
cat "${NAV2_DIR}/plan.txt"
# **产物要判存在性**：上面那个路径错误之所以没人发现，就是因为只看退出码不同时看文件。
MISSING=""
for f in plan.json plan.csv; do
    [ -s "${NAV2_DIR}/${f}" ] || MISSING="${MISSING} ${f}"
done
if [ -n "${MISSING}" ]; then
    log "** 产物缺失：${MISSING} **（退出码 ${RC}，见 plan.txt）"
    exit 1
fi
log "规划退出码 ${RC}；产物：${NAV2_DIR}/plan.json、plan.csv、plan.txt"
exit "${RC}"
