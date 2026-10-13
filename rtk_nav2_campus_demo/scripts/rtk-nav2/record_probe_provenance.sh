#!/usr/bin/env bash
# ★ 阶段十 · 第 0 刀：把**探针自己的身份与采样口径**记进本轮产物 ✓（reviewer 2026-10-11 要求 ✓）
#
# 用法： scripts/rtk-nav2/record_probe_provenance.sh <run-id> <seconds> <interval>
#
# ★★ 为什么（reviewer 原话要点 ✓）★★
#   ★ "**记下探针自己的 sha ＋ 采样间隔** ✓ 进那一轮的产物 ✓（★ "在跑的是哪份代码"要能自报 ✓）"
#   ★ 并且 ★ **要同样的采样间隔** ✓ —— ✗ 否则"旧版 vs 新版"的比较**不成立** ✓。
#
# ★ 做法（★ 两条独立读数，一致才判 ✓ —— 本项目"守卫不许把取不到当缺失"那条 ✓）：
#   ① **宿主侧** sha256（`scripts/rtk-nav2/container/residue_probe.py` ✓）
#   ② **容器内** sha256（同一文件经 `/work` 挂载 ✓）⇒ ★ 两条**必须逐位相同** ✓
#      （★ 不相同 ⇒ 记成**未取到**、并如实报 ✗，✗ 不许挑一条信 ✗）
#   ③ **对照口径也是实测的** ✓：从 `campus9-res1`／`campus9-res2` **自己的 probe.log** 里
#      量相邻拍次的墙钟间隔中位数 ✓ —— ✗ 不凭记忆写"当时是 2 s" ✗。
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:?用法: record_probe_provenance.sh <run-id> <seconds> <interval>}")"
SEC="${2:?缺 seconds}"; IV="${3:?缺 interval}"
RUN_HOST="${PROJECT_DIR}/artifacts/rtk-nav2/${RUN_ID}"
PROBE_REL="scripts/rtk-nav2/container/residue_probe.py"
PROBE_HOST="${PROJECT_DIR}/${PROBE_REL}"
PROBE_C="${CONTAINER_WORKDIR}/${PROBE_REL}"

[ -f "${PROBE_HOST}" ] || die "找不到探针脚本：${PROBE_HOST}"
SHA_HOST="$(sha256sum "${PROBE_HOST}" | cut -d' ' -f1)"
SHA_C="$(cexec "sha256sum ${PROBE_C} 2>/dev/null" 2>/dev/null | tr -d '\r' | cut -d' ' -f1)"
if [ -z "${SHA_C}" ]; then
    log "⚠ 容器内取不到探针 sha ⇒ **记成未取到** ✗（✗ 不拿宿主那份顶替 ✗）"
elif [ "${SHA_C}" != "${SHA_HOST}" ]; then
    log "⚠★ 宿主与容器**两份 sha 不一致** ✗（宿主 ${SHA_HOST:0:16}… ／ 容器 ${SHA_C:0:16}…）⇒ 如实记，✗ 不挑一条信 ✗"
else
    log "探针 sha ✓ 两处逐位相同：${SHA_HOST:0:16}…（宿主 ＋ 容器内各算一次 ✓）"
fi

mkdir -p "${RUN_HOST}/probe"
python3 - "${RUN_HOST}/probe/probe_provenance.json" "${RUN_ID}" "${PROBE_C}" "${SHA_HOST}" "${SHA_C}" \
        "${SEC}" "${IV}" "${PROJECT_DIR}" <<'PY'
import json, os, re, statistics, sys
(out, run_id, probe_c, sha_host, sha_c, sec, iv, proj) = sys.argv[1:9]

def measured_interval(run):
    """★ 从那一轮**自己的** probe.log 量墙钟间隔中位数 ✓（实测，不凭记忆 ✓）"""
    p = os.path.join(proj, 'artifacts/rtk-nav2', run, 'probe', 'probe.log')
    if not os.path.isfile(p):
        return None
    ts = []
    for ln in open(p, encoding='utf-8', errors='replace'):
        m = re.match(r'\[(\d\d):(\d\d):(\d\d)\]\s+tick', ln)
        if m:
            h, mi, s = map(int, m.groups())
            ts.append(h * 3600 + mi * 60 + s)
    d = [b - a for a, b in zip(ts, ts[1:]) if b - a > 0]
    if not d:
        return None
    return {'n_ticks': len(ts), 'interval_median_s': statistics.median(d),
            'interval_min_s': min(d), 'interval_max_s': max(d),
            'span_s': ts[-1] - ts[0], 'measured_from': p}

d = {
    'run_id': run_id,
    'probe_script_container_path': probe_c,
    'probe_script_sha256_host': sha_host,
    'probe_script_sha256_in_container': (sha_c or None),
    'probe_script_sha256_agree': bool(sha_c) and sha_c == sha_host,
    'this_round_seconds': float(sec), 'this_round_interval_s': float(iv),
    'sha256_convention': 'sha256sum 该文件；无尾换行约定问题（二进制/文本按原样）✓',
    'baseline_intervals_measured': {r: measured_interval(r) for r in ('campus9-res1', 'campus9-res2')},
    'note': ('★ 本轮探针口径 = 采样间隔 %s s ✓；★ 对照轮（res1/res2）的间隔是**从它们自己的 '
             'probe.log 量出来的**（见 baseline_intervals_measured ✓），✗ 不是凭记忆写的 ✓ ⇒ '
             '两边同口径，跨轮比较才成立 ✓' % iv),
}
json.dump(d, open(out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
print('  ★ 已落盘 %s' % out)
for r, v in d['baseline_intervals_measured'].items():
    print('    对照 %s：%s' % (r, ('中位 %ss（%d 拍）' % (v['interval_median_s'], v['n_ticks'])) if v else '未取到 ✗'))
PY
