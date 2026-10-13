#!/usr/bin/env bash
# 启动路线跟随（在已就绪的栈上跑一圈），等它结束并回显退出码与报告。
# 用法：scripts/rtk-nav2/run-route.sh [run-id] [--timeout 秒]
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

TIMEOUT=420
RUN_ID=""
while [ $# -gt 0 ]; do
    case "$1" in
        --timeout) TIMEOUT="${2:?--timeout 需要参数}"; shift 2 ;;
        -h|--help) sed -n '2,4p' "$0"; exit 0 ;;
        *) RUN_ID="$1"; shift ;;
    esac
done
RUN_ID="$(resolve_run_id "${RUN_ID}")"
DIR="$(artifact_dir "${RUN_ID}")"

if [ ! -d "${DIR}" ]; then
    die "找不到 run ${RUN_ID} 的产物目录，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"
fi
if ! stack_running; then
    die "栈没在运行，请先 scripts/rtk-nav2/demo-start.sh（★ 无参数 ✓）"
fi

log "启动路线跟随（run-id: ${RUN_ID}）"
CDIR_REPORT="$(artifact_dir_c "${RUN_ID}")/route_report.json"
cexec "/work/scripts/rtk-nav2/container/run_route.sh '${RUN_ID}' '${TIMEOUT}' '${CDIR_REPORT}'"

RC_FILE="${DIR}/route_exit_code.txt"
if [ -f "${RC_FILE}" ]; then
    RC="$(cat "${RC_FILE}")"
    log "路线跟随退出码：${RC}"
    [ -f "${DIR}/route_report.json" ] && python3 -c "
import json,sys
d=json.load(open('${DIR}/route_report.json'))
print('  退出码字段：%s   中止原因：%s' % (d.get('exit_code'), d.get('abort_reason') or '（无）'))
print('  到达路线点：%s/%s   圈数：%s' % (len(d.get('reached') or []), len(d.get('route') or []), d.get('loop_count')))
ms, me = d.get('mission_start_sim_s'), d.get('mission_end_sim_s')
if ms is not None and me is not None:
    print('  任务窗口：%.1f – %.1f s（%.1f s）' % (ms, me, me-ms))
print('  停车次数：%s  减速次数：%s  末次危险距离：%s' % (d.get('stop_events'), d.get('slow_events'), d.get('last_danger_m')))
print('  注：横向偏差请用 trajectory_report 按**真值**算的那份（本报告里的 cross_track_estimate 是按估计位姿算的，仅供对照）')
" || true
else
    log "** 没有拿到退出码文件 **（${RC_FILE}）"
fi
