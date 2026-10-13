#!/usr/bin/env bash
# bag 可读性核对（brief §11 口径修订 2：被引用为证据的 rosbag 必须 `ros2 bag info` 可读且消息数 > 0）。
#
# **必须在录制结束之后跑**（一般在 stop-demo.sh 之后）：录制期间 metadata.yaml 还没写，
# 这时候查永远是 false——我第一版把它塞在 save_artifacts.sh 里，因此得出了"metadata 不存在"
# 的假结论，真实原因只是查早了。所以这条单独成脚本，由人在停栈后显式执行。
#
# 用法：scripts/rtk-nav2/stop-demo.sh && scripts/rtk-nav2/check-bag.sh <run-id>
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

RUN_ID="$(resolve_run_id "${1:-}")"
DIR="$(artifact_dir "${RUN_ID}")/nav2"
BAG_HOST="$(artifact_dir "${RUN_ID}")/bag/run"
CDIR="$(artifact_dir_c "${RUN_ID}")/bag/run"
mkdir -p "${DIR}"
NAS="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"

{
    echo "# bag=${BAG_HOST}"
    if [ -f "${BAG_HOST}/metadata.yaml" ]; then
        echo "metadata.yaml: 存在"
        cexec_t 120 "${NAS} && timeout 90 ros2 bag info ${CDIR}" 2>&1
    else
        echo "metadata.yaml: **不存在**"
        ls -la "${BAG_HOST}" 2>&1 | head -8
    fi
} > "${DIR}/bag_info.txt" 2>&1

python3 - "${BAG_HOST}" "${DIR}/bag_info.txt" "${DIR}/bag_readable.json" <<'PY'
import json, os, re, sys
bag_dir, log, out = sys.argv[1], sys.argv[2], sys.argv[3]
text = open(log, encoding='utf-8', errors='replace').read()
counts = [int(c) for c in re.findall(r'Count: (\d+)', text)]
m = re.search(r'Duration:\s*([\d.]+)s', text)
res = {
    'metadata_yaml_exists': os.path.exists(os.path.join(bag_dir, 'metadata.yaml')),
    'ros2_bag_info_readable': ('Bag size' in text) or ('Files:' in text),
    'topic_count': len(re.findall(r'Topic: /', text)),
    'message_counts': counts,
    'total_messages': sum(counts),
    'duration_s': m.group(1) if m else None,
    'note': '通过标准：metadata_yaml_exists 且 ros2_bag_info_readable 且 total_messages > 0',
}
json.dump(res, open(out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
ok = res['metadata_yaml_exists'] and res['ros2_bag_info_readable'] and res['total_messages'] > 0
print('bag_readable: metadata=%s readable=%s topics=%s total_msgs=%s -> %s'
      % (res['metadata_yaml_exists'], res['ros2_bag_info_readable'],
         res['topic_count'], res['total_messages'], '通过' if ok else '不通过'))
PY
