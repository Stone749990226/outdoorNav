#!/usr/bin/env python3
"""从 run 的 **rosbag** 重建标定样本 CSV（`t, gt_x, gt_y, loc_x, loc_y, dt`）。

**为什么要有它**（一次真事故，2026-10-09）：我给取数器加 `--from-csv`（离线重拟合）时，
`_emit_fit()` 在**校验之前**就把目标 CSV 用 `open(..., 'w')` **截断了**，紧接着因为一个列序
不一致抛了 `IndexError` ⇒ **12198 对样本的 CSV 归零** ✗。
（原始 json 还在，但 reviewer 要的"逐样本独立复算"输入没了。）

⇒ **bag 里两个话题都在**（`/ground_truth/odom` 11873 条、`/odometry/global` 12105 条）
⇒ 用本脚本按**与取数器同一条配对规则**重建 ⇒ 数据可**原样恢复**，而且**比 CSV 更强**：
CSV 只是"我算出来的中间产物"，bag 是**原始记录** ⇒ 现在任何人可以从 bag **一路重算到底** ✓。

★ 配对规则与 `geo_calib_map_world.py::Pairer.pair_all` **逐字相同**（最近时间戳 + `--tol`），
  否则"重建的样本"与"当时算的样本"会是两份东西（"同一份数据两个口径"同族）✗。

用法（容器内，需要 `rosbag2_py`）：
  python3 scripts/bag_to_frame_fit_samples.py --bag /work/artifacts/rtk-nav2/<run>/bag/run \
      --out /work/artifacts/rtk-nav2/<run>/frame_fit --tol 0.05
产出：`<out>_samples.csv`（列序与取数器一致）+ 屏幕摘要。
"""

import argparse
import csv
import sys


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', default='', help='run 的 bag 目录（与 --from-trajectory 二选一）')
    # ★ 第二条恢复源：`trajectory.csv`（`rtk_trajectory_logger` 每拍同时记 **真值与定位**）
    #   —— 它**覆盖整轮**，而 bag 有时间上限（`BAG_MAX_SECONDS=600` ⇒ 10 min 就封顶）。
    #   实测：`campus8-d` 的 bag 只到 21:54 ⇒ 只覆盖 x∈[-44, 3]（47 m），
    #   而 `trajectory.csv` 覆盖整条 ±40 m 路线 ✓ ⇒ **恢复优先用它**。
    ap.add_argument('--from-trajectory', default='',
                    help='`<run>/trajectory.csv`：每行同时含 gt_x/gt_y 与 est_x/est_y ⇒ '
                         '**同一拍**（dt=0），覆盖整轮（不受 bag 的 10 min 上限影响）')
    ap.add_argument('--out', required=True, help='输出前缀（写 <out>_samples.csv）')
    ap.add_argument('--tol', type=float, default=0.05, help='配对容差（秒），与取数器同值')
    a = ap.parse_args(argv)

    if a.from_trajectory:
        rows = [r for r in csv.DictReader(open(a.from_trajectory, encoding='utf-8'))
                if r.get('est_x')]
        if not rows:
            print('!! trajectory.csv 里没有 est_x ⇒ 拿不到定位侧', file=sys.stderr)
            return 1
        out = [(float(r['t']), float(r['gt_x']), float(r['gt_y']),
                float(r['est_x']), float(r['est_y']), 0.0) for r in rows]
        return _dump(a, out, src=a.from_trajectory)

    if not a.bag:
        print('!! 必须给 --bag 或 --from-trajectory', file=sys.stderr)
        return 1
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='mcap' if a.bag.endswith('.mcap')
                                     else 'sqlite3'),
           rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                       output_serialization_format='cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = {'/ground_truth/odom': 'gt', '/odometry/global': 'loc'}
    for t in want:
        if t not in types:
            print('!! bag 里没有 %s ⇒ 无法重建（bag 里的话题：%s）' % (t, sorted(types)),
                  file=sys.stderr)
            return 1
    msgcls = get_message('nav_msgs/msg/Odometry')

    gt, loc = [], []
    while r.has_next():
        topic, data, _t = r.read_next()
        if topic not in want:
            continue
        m = deserialize_message(data, msgcls)
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pos = m.pose.pose.position
        (gt if topic == '/ground_truth/odom' else loc).append((ts, pos.x, pos.y))

    print('bag 读出：真值 %d 条、定位 %d 条' % (len(gt), len(loc)), file=sys.stderr)

    # ---- 配对：与 Pairer.pair_all 同一规则（两边排序 + 最近时间戳 + tol）----
    gt.sort()
    loc.sort()
    out = []
    j = 0
    for (tg, xg, yg) in gt:
        if not loc:
            break
        while j + 1 < len(loc) and abs(loc[j + 1][0] - tg) <= abs(loc[j][0] - tg):
            j += 1
        tl, xl, yl = loc[j]
        dt = abs(tl - tg)
        if dt <= a.tol:
            out.append((tg, xg, yg, xl, yl, dt))

    return _dump(a, out, src=a.bag)


def _dump(a, out, src=''):
    """写 `<out>_samples.csv`（列序 = 取数器一致：t, gt_x, gt_y, loc_x, loc_y, dt）。"""
    if not out:
        print('!! 一对都没配上 ⇒ 检查两条流的时间戳是否同一时基', file=sys.stderr)
        return 3
    with open(a.out + '_samples.csv', 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'gt_x', 'gt_y', 'loc_x', 'loc_y', 'dt'])
        for row in out:
            w.writerow(['%.6f' % row[0], '%.6f' % row[1], '%.6f' % row[2],
                        '%.6f' % row[3], '%.6f' % row[4], '%.6f' % row[5]])
    xs = [r[1] for r in out]
    ys = [r[2] for r in out]
    print('重建 %d 对（源：%s）⇒ %s_samples.csv' % (len(out), src, a.out))
    print('覆盖：x [%.2f, %.2f]（%.2f m）｜ y [%.2f, %.2f]（%.2f m）'
          % (min(xs), max(xs), max(xs) - min(xs), min(ys), max(ys), max(ys) - min(ys)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
