#!/usr/bin/env python3
"""rev8_stagnation_check.py —— 判据③：**t 单调** ＋ **有没有长时间停滞** ✓

用法: rev8_stagnation_check.py <run目录> [--v-still 0.02] [--long 20]

数据来源：`<run>/trajectory.csv`（列 `t,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw`，用 **gt** ✓）
★ **必须在栈停了之后读** ✓ —— 栈还在跑时该文件**一直在增长** ✗ ⇒ 分位数/时长类读数不可复算 ✓
  （2026-10-10 reviewer 抓到：同一目录同一工具三个时刻三组数 ✓）
⇒ 所以输出里**钉住** (sha256, 行数) ✓。

判据：
  ① **`t` 单调不减** ✓（回跳数 = 0 ✓；回跳 = "两套仿真/时钟归零"的特征 ✓）
  ② **无长时间停滞** ✓：把"逐拍速度 < v_still"的**连续段**找出来 ✓，
     报**最长的一段**（时长 ＋ 起止时刻 ＋ 位置）✓ —— 由人对着 `--long`（默认 20 s）看 ✓
退出码: 0 = ①过且最长停滞 < long ｜ 1 = 有回跳或停滞超限 ｜ 2 = 读文件错
        3 = ★ **不足以判**（这一轮**没跑完** 且 末拍**车还在动** ⇒ 产物被截断 ✗，不许当"过" ✗）
"""
import csv
import hashlib
import math
import os
import sys


def main():
    args = sys.argv[1:]
    if not args or args[0] in ('-h', '--help'):
        print(__doc__)
        return 2
    run = args[0]
    v_still = float(args[args.index('--v-still') + 1]) if '--v-still' in args else 0.02
    long_s = float(args[args.index('--long') + 1]) if '--long' in args else 20.0

    p = os.path.join(run, 'trajectory.csv')
    if not os.path.isfile(p):
        print('✗ 找不到 %s' % p)
        return 2
    blob = open(p, 'rb').read()
    sha = hashlib.sha256(blob).hexdigest()
    rows = []
    with open(p, encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append((float(r['t']), float(r['gt_x']), float(r['gt_y'])))
            except (KeyError, TypeError, ValueError):
                continue
    if len(rows) < 2:
        print('✗ 样本太少')
        return 2

    # ★★ 窗口必须限定到**驾驶区间** ✗★★（2026-10-10 实测踩到 ✓）：
    #   `trajectory.csv` 从**起栈**就开始记 ✓，到**停栈**才结束 ✓ —— 两头都有一段"没在开车"：
    #     · 头：起栈后等就绪 / 等闸门（`midB` 实测 t 0~78 s 静止 ✓）
    #     · 尾：**路线跑完之后**车停在终点、栈还开着（`midB` 实测 **252.8 s** 静止 ✗）
    #   ⇒ ★ 不切窗口的话，最长"停滞"必然是**尾巴那段** ✗ ⇒ 判据**恒不通过** ⇒ 没人再看它 ✗
    #     （这正是本项目那条"**判据两头坏法是一对**" ✓）
    #   ⇒ 窗口 = **驾驶区间的首末**：**驱动自己落的逐点产物** 里 **最早的首拍 → 最晚的末拍** ✓
    #     （★ 不靠猜 ✓）；★ 拿不到就退化成全程 **并明说** ✓。
    #   ★★ 2026-10-10 修正 ✗：**两种驱动**落的产物**文件名不同** ✓ ——
    #     批处理 `rev9-drive.sh` → `geo_route_point_*.csv` ✓；
    #     流式 `rev9-stream.sh`（阶段九的演示就走的这条 ✓）→ `stream_point_*.csv` ✗。
    #   ⇒ ★ 只认前一种的话，**流式轮次的窗口会静默退化成"全程"** ✗ ⇒
    #     尾巴那段静止（跑完之后车停在终点、栈还开着）被算进去 ⇒ **判据恒不通过** ✗
    #     （★ 实测 `campus9-nav15` 就是这样：全程 ⇒ 报"最长停滞 ≥ 20 s 不过 ✗"，
    #      而按**正确窗口**重算是 **2.1 s ✓** —— ★ 又一次"**坏的是尺子**" ✓）
    #   ⇒ ★ 两种都认 ✓，并把**实际用了哪些文件**印出来 ✓（✗ 不写死一种 ✗）。
    #   ★★ 2026-10-10 **第二次**修正 ✗★★：★ 右边界**也要两头标定** ✓ ——
    #     ★ 第一版把右边界钉在"**最后一个逐点产物的末拍**" ✓，★ "恒不通过"那一头修对了 ✓，
    #     ★ 但换来了**另一头** ✗：★ **没到达的点根本没有产物** ✗ ⇒ 卡住的那一整段
    #     正好落在窗口**之外** ✗ ⇒ ★ 判据**看不见它本该抓的那类事故** ✗ ⇒ ★ **恒过** ✗✗
    #     （★ reviewer 2026-10-10 回放实测 ✓：`nav12` 整轮 wp41 真卡 **420 s** ✓，
    #      而窗口只到 **1475.7** ✗、后面 **~729 s 的不动**被切掉 ⇒ ★ 报"过" ✗。）
    #   ⇒ ★ 右边界看**这一轮到底跑完没有** ✓：
    #       · 跑完（逐点记录数 **≥** 航线点数）⇒ 按**末点末拍** ✓（把跑完后的静止尾巴排除 ✓）
    #       · 没跑完 ⇒ ★ **右边界延伸到 `trajectory.csv` 末尾** ✓（★ 失败那段必须**在窗口内** ✓）
    #       · ★ **判不出来 ⇒ 也延伸到末尾** ✓（✗ 绝不默认"跑完了" ✗ —— 宁可多报 ✗ 不肯漏报 ✗）
    #     ★ 这正是那条"**判据两头坏法是一对**" ✓ —— ★ **恒过比恒不通过更坏** ✗
    #       （★ 恒过会在产物里印一个**绿色的"过"** ✓，而那一轮**肉眼可见地失败了** ✗）。
    import glob
    import json
    PATS = ('geo_route_point_*.csv', 'stream_point_*.csv')
    t_lo, t_last_art = None, None
    used = []
    for pat in PATS:
        for f in sorted(glob.glob(os.path.join(run, pat))):
            ts = []
            with open(f, encoding='utf-8') as fh:
                for r in csv.DictReader(fh):
                    try:
                        ts.append(float(r['t']))
                    except (KeyError, TypeError, ValueError):
                        pass
            if ts:
                t_lo = ts[0] if t_lo is None else min(t_lo, ts[0])
                t_last_art = ts[-1] if t_last_art is None else max(t_last_art, ts[-1])
                used.append(os.path.basename(f))
    n_geo = sum(1 for u in used if u.startswith('geo_route_point_'))
    n_stream = len(used) - n_geo

    # ★ **航线共几点**：★ 取**所有候选航线文件的最大值** ✓
    #   （✗ 不能只信一个 ✗：★ 实测 `nav12` 的 `input/route_stream.json` **只有 8 点** ✗
    #    —— 追加没落到它上面 ✓；而同一轮的 `route_geo.json` 是 **41** ✓、三段和也是 41 ✓）
    cands = []
    for pat in ('input/*route*.json', '*route*.json'):
        for f in sorted(glob.glob(os.path.join(run, pat))):
            try:
                d = json.load(open(f, encoding='utf-8'))
            except Exception:
                continue
            pts = d.get('points') if isinstance(d, dict) else d
            if isinstance(pts, list) and pts:
                cands.append((os.path.relpath(f, run), len(pts)))
    cands = sorted(set(cands))
    route_n = max((n for _, n in cands), default=None)
    cand_txt = '，'.join('%s=%d' % (b, n) for b, n in cands) if cands else '（一个都读不到）'

    # ★ 这一轮**跑完没有** = 逐点记录数 vs 航线点数 ✓（★ 两个数都印出来 ✓ 肉眼可查 ✓）
    rec_n, rec_src = None, '（找不到逐点记录文件）'
    for f in sorted(glob.glob(os.path.join(run, '*records*.json'))):
        try:
            d = json.load(open(f, encoding='utf-8'))
        except Exception:
            continue
        if isinstance(d, dict):
            rec_n = d.get('n_records', len(d.get('records', [])))
        elif isinstance(d, list):
            rec_n = len(d)
        rec_src = os.path.basename(f)
        break

    full = rows
    t_hi = t_last_art
    if not used:
        t_lo = None
        why = '★ 一个逐点产物都没有 ⇒ ★ 窗口 = **全程** ✓'
    elif route_n is None or rec_n is None:
        t_hi = full[-1][0]
        why = ('★ **判不出跑没跑完** ✗（航线点数 或 逐点记录 缺）⇒ '
               '★ **右边界延伸到末尾** ✓（★ 宁可多报 ✗ 不肯漏报 ✗）')
    elif rec_n >= route_n:
        why = ('★ 判定 = **跑完了** ✓（逐点记录 %d ≥ 航线 %d）⇒ ★ 右边界 = **末点末拍** ✓'
               '（把跑完后的静止尾巴排除 ✓）' % (rec_n, route_n))
    else:
        t_hi = full[-1][0]
        why = ('★ 判定 = **没跑完** ✗（逐点记录 %d < 航线 %d）⇒ ★ **右边界延伸到末尾** ✓'
               '（★ 失败那一整段必须**在窗口内** ✓）' % (rec_n, route_n))
    if t_lo is not None:
        rows = [r for r in full if t_lo - 0.05 <= r[0] <= t_hi + 0.05]
        if len(rows) < 2:
            rows = full
            why += '（★ 但窗口内样本 < 2 ⇒ 退回全程 ✓）'

    print('=== 判据③：t 单调 ＋ 停滞检查 ===')
    print('  目录: %s' % run)
    print('  ★ 读的那份文件：trajectory.csv sha256=%s' % sha[:16])
    # ★ 2026-10-10 修正 ✗：这里 `rows` **已经被窗口筛过** ✓ ⇒ 印它会印成"窗口内行数" ✗
    #   （★ 我第一版就印错了：写成"数据行"，实际是 12212 那个**窗口内**的数 ✗ ——
    #    ★ 与"裸数字掩盖真因"同一族 ✓）⇒ ★ 要印**未筛的 `full`** ✓，并**各自带标签** ✓。
    print('    ★ `wc -l`（整份文件）= %d ｜ ★ 数据行（整份、不含表头）= %d'
          % (blob.count(b'\n'), len(full)))
    print('      （★ 差 1 时那个 1 是**表头** ✓，✗ 不是换行 ✗；★ 窗口内的行数另见下一行 ✓）')
    print('  ★ 逐点产物 = **%d** 个（geo_route_point_* ×%d ＋ stream_point_* ×%d）'
          % (len(used), n_geo, n_stream))
    print('  ★ 航线共 %s 点 ｜ 候选来源：%s'
          % (route_n if route_n is not None else '★取不到★', cand_txt))
    print('  ★ 逐点记录 = %s 条（%s）' % (rec_n if rec_n is not None else '★取不到★', rec_src))
    print('  ★ 窗口口径 = %s' % why)
    print('  ★ 分析窗口 t ∈ [%.1f, %.1f] s（样本 %d）｜ ★ 全程样本 %d'
          % (rows[0][0], rows[-1][0], len(rows), len(full)))

    # ★★ 三态：★ "**没跑完**"还要再分两种 ✗★★ ——
    #   ★ 没跑完 **且末拍车还在动** ⇒ ★ 那是**产物被截断**（栈被提前停了 / 记录器先死 ✓）
    #     ⇒ ★ **不足以判** ✗ —— ★ 它**不能**印成"过" ✗（★ 本轮实测 `nav13` 就是这一态 ✓：
    #       车 13:42:01 起、**13:45:41 栈就被停了** ✓，末拍仍在往末点开 ✗ ⇒
    #       "最长停滞 1.9 s ⇒ 过" ✗ **是错的结论** ✗ —— ★ 它只是"**没记到**" ✓）。
    #   ★ 没跑完 **而末拍是静止的** ⇒ ★ 那才是"**卡住了**" ✓（`nav12` 就是这一态 ✓：卡 708 s ✓）。
    #   ★ 这正是本项目那条"**字段读空 ⇒ undetermined，✗ 不许落成"没变"**" ✓。
    complete = (route_n is not None and rec_n is not None and rec_n >= route_n)
    tail_s = 5.0
    tail = [r for r in rows if r[0] >= rows[-1][0] - tail_s]
    tail_v = 0.0
    if len(tail) >= 2:
        dsum = sum(math.hypot(tail[i][1] - tail[i - 1][1], tail[i][2] - tail[i - 1][2])
                   for i in range(1, len(tail)))
        dtsum = tail[-1][0] - tail[0][0]
        tail_v = dsum / dtsum if dtsum > 1e-9 else 0.0
    truncated = (not complete) and tail_v >= v_still

    # ① t 单调
    back = [i for i in range(1, len(rows)) if rows[i][0] < rows[i - 1][0]]
    print('  ★① `t` 回跳次数 = %d %s' % (len(back), '✓' if not back else '✗'))
    if back:
        for i in back[:3]:
            print('       t[%d] %.3f → %.3f' % (i - 1, rows[i - 1][0], rows[i][0]))

    # ② 停滞段（逐拍速度 < v_still 的连续段）
    segs = []
    cur = None
    path_len = 0.0
    for i in range(1, len(rows)):
        t0, x0, y0 = rows[i - 1]
        t1, x1, y1 = rows[i]
        dt = t1 - t0
        d = math.hypot(x1 - x0, y1 - y0)
        path_len += d
        v = d / dt if dt > 1e-9 else 1e9
        if v < v_still:
            if cur is None:
                cur = [t0, t1, x0, y0]
            else:
                cur[1] = t1
        else:
            if cur is not None:
                segs.append(cur)
                cur = None
    if cur is not None:
        segs.append(cur)
    segs = [s for s in segs if s[1] - s[0] > 0.5]           # 忽略零头
    segs.sort(key=lambda s: s[1] - s[0], reverse=True)
    longest = segs[0] if segs else None
    print('  ★② 逐拍速度 < %.3f m/s 的连续段：%d 段；最长 = %s'
          % (v_still, len(segs),
             ('%.1f s（t %.1f → %.1f，位置 (%.2f, %.2f)）'
              % (longest[1] - longest[0], longest[0], longest[1], longest[2], longest[3]))
             if longest else '无'))
    if segs[:5]:
        print('     前 5 长：%s' % '，'.join('%.1fs@%.0fs' % (s[1] - s[0], s[0]) for s in segs[:5]))
    ok2 = (longest is None) or (longest[1] - longest[0] < long_s)
    if truncated:
        print('  ★★ 但 **这一轮的产物被截断** ✗ —— 没跑完（逐点记录 %s < 航线 %s）'
              % (rec_n, route_n))
        print('      **且末拍车还在动**（末 %.0f s 平均 %.3f m/s ≥ %.3f）✗'
              % (tail_s, tail_v, v_still))
        print('      ⇒ ★ 判定 = **不足以判** ✗（✗ 不许印成"过" ✗ —— ★ 这是"**没记到**" ✓，')
        print('         ✗ 不是"没停滞" ✗；★ 要判就得**把这一轮跑完**再量 ✓）')
    else:
        print('  ★② 最长停滞 %s %.1f s 的阈值 ⇒ %s'
              % ('<' if ok2 else '>=', long_s, '过 ✓' if ok2 else '不过 ✗'))
    span = max(1e-9, rows[-1][0] - rows[0][0])
    still_s = sum(s[1] - s[0] for s in segs)          # ★ 静止**时长**（不是段数 ✗）
    print('  ---- 参考 ----')
    print('  路径总长 = %.2f m ｜ 平均速度 = %.3f m/s ｜ 静止时长合计 = %.1f s（占全程 %.1f%%）'
          % (path_len, path_len / span, still_s, 100.0 * still_s / span))
    if back:
        return 1
    if truncated:
        return 3                                      # ★ 3 = 不足以判（产物被截断 ✓）
    return 0 if ok2 else 1


if __name__ == '__main__':
    sys.exit(main())
