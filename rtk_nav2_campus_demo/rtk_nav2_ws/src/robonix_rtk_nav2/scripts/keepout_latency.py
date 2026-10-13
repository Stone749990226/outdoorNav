#!/usr/bin/env python3
"""★ **合并 keepout 的时延仪表**（reviewer 2026-10-09 裁定：**进轮的工具** ⇒ 开工前必须冒烟一次 ✗）。

**测什么** ✓：`/rtk/drop/mask_map`（检测器那份 ✓）→ `/rtk/drop/mask_merged`（合并节点输出 ✓）
之间**到底加了多少时延** ✓ —— 因为 keepout 的 `filter_info_topic` 已改指合并节点 ✓ ⇒
**下游（keepout filter）现在看到的是合并输出** ✓ ⇒ 要能回答"这一级会不会慢到让禁行区迟到" ✓。

**★ 三个数、三条来历**（**只报一个数必然误标** ✗ —— 这正是要分开报的理由 ✓）：

| 字段 | 定义 | 它含什么 |
| --- | --- | --- |
| `d_stamp_ms` | `merged.header.stamp − mask.header.stamp` | ★ **含检测器自身处理** ✗（检测器把**深度帧的**时间戳原样写进掩膜 ✓ ⇒ 这个差是"**深度帧 → 合并输出**" ✓）⇒ **是上界** ✓ |
| `d_recv_ms` | `收 merged 的钟 − 收 mask 的钟`（**本仪表自己的 sim 时钟** ✓） | **合并这一级新增的时延** ✓（下游视角 ✓）；含本仪表的回调调度抖动 ✓ |
| `node_proc_est_ms` | `merged.header.stamp − 收到 mask 的时刻` | **节点处理耗时**的估计 ✓（前提：检测器→节点 与 检测器→仪表 的传输近似相等 ✓） |

**为什么不用单一判据量的"端到端"** ✗：`merged.stamp` 是**我的节点重打的 `now()`** ✓ ⇒
它**不是**检测器发布时刻 ✓ ⇒ 拿它减 `mask.stamp` 得到的是**上界**、不是节点耗时 ✓；
反过来只报"收→收"又会把**仪表自身调度**算进去 ✓。⇒ **三个都报、各自标清** ✓。

**★ 一个必须处理的坑**（不处理就会把中位数以外的数全污染 ✗）：
`keepout_merge.on_static()` 里对**静态图到达**也会 `republish()` ✓，而 `static_walls` 是 **1 Hz 定时重发** ✓
⇒ **每秒有 1 条"没有对应 mask 的" merged** ✗。它没有因果源 ✓ ⇒ 配对规则必须**只把 merged 配给"戳 ≤ 它、且尚未被配过的 mask"** ✓，
再用 `--max-pair-ms` 卡一道 ✓。**被卡掉的和没配上对的都计数、都进 JSON** ✓（**不藏** ✗）——
且**卡尺子不会放过慢节点** ✓：若合并真的慢，样本会掉到 0 ⇒ **冒烟必失败** ✓（**失败朝"不通过"倒** ✓）。

退出码：`0` = 通过（配对数 ≥ `--want`）；`3` = 没等到 sim 时钟；`4` = 没有 mask；`5` = 有 mask 但**一条都没配上**（合并节点没产出）；`6` = 配上了但**样本不够**；`1` = 崩了；`2` = 用法错。
**无论哪个码，JSON 都照写** ✓（失败也要留下证据 ✓）。

用法：`keepout_latency.py --json out.json [--seconds 60] [--want 10]`
"""
import argparse
import json
import statistics
import sys
import time

# ★ ROS/numpy **懒加载** ✓（本仓库惯例 ✓）：`--self-test` 是纯逻辑 ⇒ 在**没有 ROS 的机器上也要能跑** ✓
#   （首版把 `import rclpy` 放模块顶层 ⇒ 容器里裸 `python3 --self-test` 直接 `ModuleNotFoundError` ✗）
SRC = '/rtk/drop/mask_map'          # 检测器产物（**不动它** ✓，只订阅 ✓）
OUT = '/rtk/drop/mask_merged'       # 合并节点产物
MARKED = 100                        # 检测器源码里就是 100 ✓


def _stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _marked(msg):
    import numpy as np
    return int((np.asarray(msg.data, dtype=np.int16) >= MARKED).sum())


def _summ(xs):
    if not xs:
        return None
    s = sorted(xs)
    return {'n': len(xs), 'min': round(s[0], 2),
            'median': round(statistics.median(s), 2),
            'p90': round(s[int(0.9 * (len(s) - 1))], 2), 'max': round(s[-1], 2)}


def _pair(srcs, outs, max_pair_ms):
    """★ 纯函数、**可离线自检** ✓（不起 ROS ✓）—— 配对规则见文件头"一个必须处理的坑" ✓。

    `srcs`/`outs` = `{'stamp':ns, 'recv':ns, ...}` 列表（`srcs` 会就地标 `used` ✓）。
    **门被踩时不消费候选** ✓：那条 mask 留给它**真正**的 merged ✓；
    被门拒的计数 `rejected_by_gate` ✓、找不到因果源（**1 Hz 静态重发** ✓）计数 `no_candidate` ✓。
    """
    order = sorted(srcs, key=lambda d: d['stamp'])
    d_stamp, d_recv, proc_est = [], [], []
    gate_rej, no_cand = 0, 0
    for o in sorted(outs, key=lambda d: d['stamp']):
        cand = None
        for s in order:                        # 已按戳升序 ⇒ 取"戳 ≤ o 且未用过"里的**最后一个** ✓
            if (not s['used']) and s['stamp'] <= o['stamp']:
                cand = s
        if cand is None:
            no_cand += 1
            continue
        dr = (o['recv'] - cand['recv']) / 1e6
        if abs(dr) > max_pair_ms:
            gate_rej += 1
            continue
        cand['used'] = True
        d_stamp.append((o['stamp'] - cand['stamp']) / 1e6)
        d_recv.append(dr)
        proc_est.append((o['stamp'] - cand['recv']) / 1e6)
    return {'d_stamp': d_stamp, 'd_recv': d_recv, 'proc_est': proc_est,
            'gate_rej': gate_rej, 'no_cand': no_cand,
            'leftover': sum(1 for s in order if not s['used'])}


def _self_test():
    """★ 合成退化例 —— **每一例都可能不过** ✗（今天 `costmap_probe.py` 首跑就崩过 ✓）。

    ★ 时间一律用**毫秒**写给 `mk()` ✓（`MS` 是"毫秒→纳秒"✓）——
      首版我按**秒**写、`mk` 按**毫秒**解释 ⇒ ③ 的"慢节点 500 ms"其实是 **0.5 ms**、
      于是"慢节点"那一例**假装通过**了 ✗。**仪器没错、是自检的夹具错了** ✓（一样得修 ✓）。
    """
    MS = 1_000_000                     # 毫秒 → 纳秒

    def mk(stamps_ms, recv_off_ms=0.0):
        return [{'stamp': int(t * MS), 'recv': int((t + recv_off_ms) * MS), 'used': False}
                for t in stamps_ms]

    fails = []

    # ① 正常：3 对，节点 +5 ms
    s = mk([1000.0, 2000.0, 3000.0]); o = mk([1005.0, 2005.0, 3005.0])
    r = _pair(s, o, 200.0)
    if not (len(r['d_stamp']) == 3 and abs(r['d_stamp'][0] - 5.0) < 1e-3 and r['gate_rej'] == 0):
        fails.append('①正常 配对=%d d=%s' % (len(r['d_stamp']), r['d_stamp']))

    # ② ★ 1 Hz 无源 merged（静态重发）：应进 no_candidate、**不进** 配对、**不**污染 d_recv
    s = mk([1000.0, 2000.0, 3000.0]); o = mk([1005.0, 1500.0, 2005.0, 2500.0, 3005.0])
    r = _pair(s, o, 200.0)
    if not (len(r['d_stamp']) == 3 and r['no_cand'] == 2 and max(r['d_recv']) < 20):
        fails.append('②无源 配对=%d no_cand=%d max_recv=%.1f'
                     % (len(r['d_stamp']), r['no_cand'], max(r['d_recv'])))

    # ③ ★ 慢节点（+500 ms）⇒ **必须一条都配不上** ✓（失败朝"不通过"倒 ✓，不能被门悄悄放过 ✗）
    s = mk([1000.0, 2000.0, 3000.0]); o = mk([1500.0, 2500.0, 3500.0])
    r = _pair(s, o, 200.0)
    if not (len(r['d_stamp']) == 0 and r['gate_rej'] == 3):
        fails.append('③慢节点 配对=%d gate_rej=%d' % (len(r['d_stamp']), r['gate_rej']))

    # ④ latched 重放：out 的戳早于任何 mask ⇒ no_candidate、不崩
    s = mk([5000.0]); o = mk([1000.0])
    r = _pair(s, o, 200.0)
    if not (len(r['d_stamp']) == 0 and r['no_cand'] == 1 and r['leftover'] == 1):
        fails.append('④迟到 replay 配对=%d no_cand=%d' % (len(r['d_stamp']), r['no_cand']))

    # ⑤ 仪表自身调度抖动 ⇒ d_recv 可以为负 ⇒ **不许崩、也不许悄悄当成 0** ✓
    s = mk([1000.0]); o = [{'stamp': int(1005.0 * MS), 'recv': int(999.0 * MS)}]
    r = _pair(s, o, 200.0)
    if not (len(r['d_stamp']) == 1 and r['d_recv'][0] < 0):
        fails.append('⑤负 d_recv 配对=%d' % len(r['d_stamp']))

    for f in fails:
        print('  ✗ %s' % f)
    print('自检：%d/%d 例通过' % (5 - len(fails), 5))
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', help='读数落这里 ✓（`--self-test` 时可省 ✓）')
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--want', type=int, default=10, help='通过所需的最少配对数')
    ap.add_argument('--max-pair-ms', type=float, default=200.0,
                    help='配对门：|d_recv| 超过它就判为"不是这条 mask 触发的" ✓')
    ap.add_argument('--self-test', action='store_true', help='只跑合成退化例、不起 ROS ✓')
    a = ap.parse_args()
    if a.self_test:
        return _self_test()
    if not a.json:
        ap.error('--json 必填（除非 --self-test）')

    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    Q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE)

    rclpy.init()
    # ★ `Parameter(name, value)` 的**第二个位置参数是 `type`、不是值** ✗ ——
    #   首版写成 `Parameter('use_sim_time', True)` ⇒ `TypeError: type must be an instance of 'Type'` ✗
    #   （**一跑就崩** ✓，正是"进轮的工具必须先冒烟"那条规矩要拦的东西 ✓）⇒ 必须用 `value=` 关键字 ✓
    n = Node('keepout_latency', parameter_overrides=[Parameter('use_sim_time', value=True)])
    srcs, outs = [], []

    def now_ns():
        return n.get_clock().now().nanoseconds

    def on_src(m):
        srcs.append({'stamp': _stamp_ns(m), 'recv': now_ns(), 'count': _marked(m), 'used': False})

    def on_out(m):
        outs.append({'stamp': _stamp_ns(m), 'recv': now_ns(), 'count': _marked(m)})

    n.create_subscription(OccupancyGrid, SRC, on_src, Q)
    n.create_subscription(OccupancyGrid, OUT, on_out, Q)

    # ── 等 sim 时钟 ✓（`use_sim_time: True` 下 `/clock` 没来之前 `now()` 恒 0 ⇒ 差值会全是假的 ✗）──
    t0 = time.time()
    while now_ns() == 0 and time.time() - t0 < 20.0:
        rclpy.spin_once(n, timeout_sec=0.2)
    if now_ns() == 0:
        _write(a.json, {'verdict': 'FAIL', 'reason': 'no_clock',
                        'note': '20 s 内没等到 `/clock` ⇒ sim 时钟没起来 ⇒ **时延不可测** ✗'})
        print('✗ 没等到 sim 时钟 ⇒ exit 3')
        return 3

    end = time.time() + a.seconds
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node()
    rclpy.shutdown()

    # ── 配对（纯函数 ✓，退化例已离线自检 ✓）──────────────────────────────────
    pr = _pair(srcs, outs, a.max_pair_ms)
    d_stamp, d_recv, proc_est = pr['d_stamp'], pr['d_recv'], pr['proc_est']
    gate_rej, no_cand, left = pr['gate_rej'], pr['no_cand'], pr['leftover']
    out = {
        'verdict': 'PASS' if len(d_stamp) >= a.want else 'FAIL',
        'window_s': a.seconds,
        'want': a.want,
        'max_pair_ms_gate': a.max_pair_ms,
        'mask_msgs': len(srcs), 'merged_msgs': len(outs),
        'paired': len(d_stamp),
        'unpaired_merged(no_candidate)': no_cand,
        'rejected_by_gate': gate_rej,
        'unpaired_mask(leftover)': left,
        # ★ 独立于 costmap_probe 的**第二处掩膜计数** ✓（时点不同 ⇒ 可交叉验证"回退有没有被触发" ✓）
        'mask_count_last': srcs[-1]['count'] if srcs else None,
        'merged_count_last': outs[-1]['count'] if outs else None,
        'merged_count_first': outs[0]['count'] if outs else None,
        'd_stamp_ms': _summ(d_stamp),
        'd_recv_ms': _summ(d_recv),
        'node_proc_est_ms': _summ(proc_est),
        'defs': {
            'd_stamp_ms': 'merged.stamp − mask.stamp ⇒ **含检测器自身处理**（掩膜戳 = 深度帧戳）⇒ **上界**',
            'd_recv_ms': '收 merged 的钟 − 收 mask 的钟（本仪表 sim 时钟）⇒ **合并级新增时延**（下游视角）',
            'node_proc_est_ms': 'merged.stamp − 收到 mask 的时刻 ⇒ 节点处理耗时**估计**',
        },
        'rejected_note': '`rejected_by_gate` = 有候选但 |d_recv| 超门的 ⇒ 判为"非这条 mask 触发" ✓（不藏 ✗）',
    }
    if not srcs:
        out['reason'] = 'no_mask'
    elif not d_stamp:
        out['reason'] = 'no_pair'
    elif len(d_stamp) < a.want:
        out['reason'] = 'too_few'
    _write(a.json, out)

    print('mask=%d merged=%d 配对=%d（无源=%d 超门=%d 剩=%d）'
          % (len(srcs), len(outs), len(d_stamp), no_cand, gate_rej, left))
    for k in ('d_stamp_ms', 'd_recv_ms', 'node_proc_est_ms'):
        print('  %-17s %s' % (k, out[k]))
    print('  掩膜计数：mask=%s merged=%s' % (out['mask_count_last'], out['merged_count_last']))

    if not srcs:
        print('✗ 一条 mask 都没收到 ⇒ exit 4'); return 4
    if not d_stamp:
        print('✗ 有 mask 但一条都没配上 ⇒ **合并节点没产出** ⇒ exit 5'); return 5
    if len(d_stamp) < a.want:
        print('✗ 只配上 %d 条（< %d）⇒ exit 6' % (len(d_stamp), a.want)); return 6
    return 0


def _write(path, obj):
    with open(path, 'w') as f:
        json.dump(obj, f, indent=1, ensure_ascii=False)


if __name__ == '__main__':
    sys.exit(main())
