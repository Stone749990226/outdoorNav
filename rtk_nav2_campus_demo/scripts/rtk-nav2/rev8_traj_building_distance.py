#!/usr/bin/env python3
"""rev8_traj_building_distance.py —— 轨迹到**静态障碍**的最小距离（第 3 步的判据②）

用法: rev8_traj_building_distance.py <run目录> [--world <campus_100.sdf>] [--need 0.355]

数据来源（**都是权威产物** ✓）：
  <run>/trajectory.csv  —— 列 `t,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw`（用 **gt** 列 ✓）
  worlds/campus_100.sdf —— `bld_*` / `green_*` 的 `<pose>` + `<box><size>`（轴对齐盒 ✓）

★ 判据（reviewer 2026-10-10）：**轨迹与楼面的最小距离 ≥ 0.355 m**（0.105 车半径 ＋ 0.25 膨胀）✓
★ 并**成对报**（★ 评审自己的要求："该近的时候真的近过" ✓）：
  只给一个"最小值"是**会恒过**的 ✗ —— 若那轮**根本没靠近过任何楼**，最小值天然很大 ✓
  ⇒ 所以这里同时给 **分位数（p1/p5/p50）＋ 最小值的落点(t,x,y) ＋ 进入各阈值带的样本数** ✓
    ⇒ "最小值大" 与 "全程都离得远" 能分开 ✓（前者是结果、后者说明这一轮**没有检验到** ✗）。
退出码: 0 = 最小值 ≥ need ｜ 1 = 最小值 < need ｜ 2 = 读文件/用法错
"""
import csv
import math
import os
import sys
import xml.etree.ElementTree as ET


def load_boxes(world):
    root = ET.parse(world).getroot()
    out = []
    for m in root.iter('model'):
        nm = m.get('name') or ''
        if not (nm.startswith('bld') or nm.startswith('green')):
            continue
        pose = m.find('pose')
        size = None
        for g in m.iter('geometry'):
            b = g.find('box')
            if b is not None:
                size = b.find('size').text
        if pose is None or size is None:
            continue
        x, y = [float(v) for v in pose.text.split()[:2]]
        sx, sy = [float(v) for v in size.split()[:2]]
        out.append((nm, x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2))
    return out


def dist_to_box(px, py, box):
    """点到轴对齐盒的距离；**盒内为 0** ✓（且返回 inside 标志）"""
    _, x0, x1, y0, y1 = box
    inside = (x0 <= px <= x1) and (y0 <= py <= y1)
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    return math.hypot(dx, dy), inside


def main():
    args = sys.argv[1:]
    if not args or args[0] in ('-h', '--help'):
        print(__doc__)
        return 2
    run = args[0]
    world = 'rtk_nav2_ws/src/robonix_rtk_nav2/worlds/campus_100.sdf'
    need = 0.355
    if '--world' in args:
        world = args[args.index('--world') + 1]
    if '--need' in args:
        need = float(args[args.index('--need') + 1])

    traj = os.path.join(run, 'trajectory.csv')
    if not os.path.isfile(traj):
        print('✗ 找不到 %s' % traj)
        return 2
    boxes = load_boxes(world)
    # ★★ 2026-10-10（reviewer 抓到 ✗）：**必须钉住读的是哪一份文件** ★★
    #   实测：栈**还在跑**时 `trajectory.csv` **还在长** ✓ ⇒ 我 / reviewer / 再读一次，
    #   同一目录同一工具竟给出**三组不同的分位数** ✗（p1 0.8511 / 0.8935 / 0.9098 ✓），
    #   ★ 而"最小距离"三次一致 ✓（最小值出现在**早期**，早就写进去了 ✓）。
    #   ⇒ ★ 定式：**分位数这类"整段统计"必须在文件冻结后读** ✓（栈停了再读 ✓），
    #     并且**读数必须带 (sha256, 行数)** ✓ —— 否则复算方只能采信转述 ✗。
    import hashlib
    blob = open(traj, 'rb').read()
    traj_sha = hashlib.sha256(blob).hexdigest()
    traj_lines = blob.count(b'\n')
    rows = []
    with open(traj, encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                t = float(r['t']); x = float(r['gt_x']); y = float(r['gt_y'])
            except (KeyError, TypeError, ValueError):
                continue
            best, who, ins = 1e9, None, False
            for b in boxes:
                d, i = dist_to_box(x, y, b)
                if i:
                    ins = True
                if d < best:
                    best, who = d, b[0]
            rows.append((t, x, y, best, who, ins))

    if not rows:
        print('✗ trajectory.csv 里没有可用样本')
        return 2
    ds = sorted(r[3] for r in rows)
    mn = min(rows, key=lambda r: r[3])
    n = len(ds)

    def q(p):
        return ds[min(n - 1, int(p * n))]

    print('=== 轨迹 ↔ 静态障碍 最小距离 ===')
    print('  目录: %s' % run)
    print('  世界: %s（%d 个静态障碍盒）' % (world, len(boxes)))
    # ★ 2026-10-10（reviewer 更正 ✓）：★ 行数必须**带标签** ✓ ——
    #   ★ 裸数字会掩盖真因 ✗：`wc -l` 与"数据行（不含表头）"在**有尾换行**的文件上**差 1** ✓，
    #   而那个 1 是**表头**（`csv.DictReader` 不含表头 ✓）、✗ 不是换行 ✗（实测两者同为 17106 ✓）。
    print('  ★ 读的那份文件（★ 复算要钉住这两个 ✓）：trajectory.csv sha256=%s'
          % traj_sha[:16])
    print('    ★ `wc -l` = %d ｜ ★ 数据行（不含表头）= %d（★ 差 1 时那个 1 是**表头** ✓，✗ 不是换行 ✗）'
          % (traj_lines, len(rows)))
    print('  样本数 = %d ｜ t ∈ [%.1f, %.1f] s' % (n, rows[0][0], rows[-1][0]))
    print('  ★ **最小距离 = %.4f m**（判据 need = %.3f m ⇒ %s）'
          % (mn[3], need, '过 ✓' if mn[3] >= need else '不过 ✗'))
    print('     ★ 最小值出现在 t=%.2f s  点 (%.3f, %.3f)  最近障碍 = %s' % (mn[0], mn[1], mn[2], mn[4]))
    print('  ---- 分位数（★ 判"是不是压根没靠近过" ✓）----')
    print('    p1 = %.4f m ｜ p5 = %.4f m ｜ p50 = %.4f m ｜ 最大 = %.4f m'
          % (q(0.01), q(0.05), q(0.50), ds[-1]))
    for band in (0.355, 0.5, 1.0, 2.0, 5.0):
        c = sum(1 for d in ds if d < band)
        print('    距离 < %.3f m 的样本数 = %d（%.1f%%）' % (band, c, 100.0 * c / n))
    ins = sum(1 for r in rows if r[5])
    print('  ★ **落在盒内（= 切进楼里）的样本数 = %d**%s' % (ins, '' if ins == 0 else '  ✗'))
    print('  ⇒ %s' % ('最小值达标 ✓（★ 并看上一行分位数：若全程都离得远，这一轮**没检验到** ✗）'
                      if mn[3] >= need else '最小值不达标 ⇒ 视为"切进楼里" ✗'))
    return 0 if (mn[3] >= need and ins == 0) else 1


if __name__ == '__main__':
    sys.exit(main())
