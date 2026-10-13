#!/usr/bin/env python3
"""rev9_scene_probe.py —— 用**同一套口径**量截图：背景色 / 主色（含归一化）/ 地面板占比 ✓

用法: rev9_scene_probe.py <图片> [--ground "r,g,b"] [--tol 12] [--bg-anchor "x,y"]

★ 为什么要有它（2026-10-10 阶段九·第 2 步 ✓）：
  ★ 要判"我们的渲染和他们的像不像"，就必须**两侧用同一把尺子**量 ✓ ——
    ✗ 不能一边肉眼看、一边报数字 ✗（本项目"读数有效性五维"里的**比对**维 ✓）。
  ★ 三条量：
    ① **背景色**：取图上一块锚点区域的**唯一色**（世界 `<scene><background>` 的落点 ✓）
    ② **主色表**：整图直方图 top-N，每行给 **RGB ＋ 计数 ＋ 归一化比值（最大分量=1）** ✓
       —— ★ 归一化比值是**不受整体明暗影响**的量 ✓ ⇒ 能分开"色相变了"与"整体变亮变暗" ✓
    ③ **地面板占比**：把"接近地面绿"的像素框出来，报**包围盒占整图宽/高的百分比** ✓
       —— ★ 这是"取景口径"的量 ✓（机位/视场一变它就变 ✓）

★ 约定：直方图按**精确 RGB** 计数（不做色彩量化 ✓）；`--depth 8` ✓。
"""
import subprocess
import sys


def sh(*a):
    return subprocess.run(a, capture_output=True, text=True).stdout


def main():
    a = sys.argv[1:]
    if not a:
        print(__doc__)
        return 2
    img = a[0]
    tol = int(a[a.index('--tol') + 1]) if '--tol' in a else 12
    ground = None
    if '--ground' in a:
        ground = tuple(int(v) for v in a[a.index('--ground') + 1].split(','))

    ident = sh('identify', '-format', '%w %h', img).split()
    if len(ident) != 2:
        print('✗ 读不了图 %s' % img)
        return 2
    W, H = int(ident[0]), int(ident[1])
    print('=== %s（%dx%d）===' % (img.split('/')[-1], W, H))

    # ---- ① 整图直方图 top-N ----
    raw = sh('convert', img, '-format', '%c', '-depth', '8', 'histogram:info:-')
    rows = []
    for line in raw.splitlines():
        line = line.strip()
        if ':' not in line:
            continue
        try:
            cnt = int(line.split(':')[0].strip())
        except ValueError:
            continue
        import re
        m = re.search(r'\((\d+),(\d+),(\d+)\)', line)
        if not m:
            m = re.search(r'#([0-9A-Fa-f]{6})', line)
            if not m:
                continue
            h = m.group(1)
            rgb = (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))
        else:
            rgb = tuple(int(x) for x in m.groups())
        rows.append((cnt, rgb))
    rows.sort(reverse=True)
    tot = W * H

    # ★ `--auto-ground`：自动挑"地面绿" ✓ —— ✗ 别把颜色写死 ✗
    #   （★ 两侧的地面绿**不是同一个值** ✓：他们 (105,145,88)、我们 (145,166,134) ——
    #    ★ 这正是要比的东西 ✓ ⇒ 写死就等于只量得动一侧 ✓）
    #   判据：**绿分量最大** 且 **在色相上偏绿**（g−r、g−b 都为正 ✓），取满足者里**计数最多**的 ✓
    if ground is None and '--auto-ground' in a:
        cands = [(c, rgb) for c, rgb in rows
                 if rgb[1] > rgb[0] + 8 and rgb[1] > rgb[2] + 8]
        if cands:
            ground = cands[0][1]
            print('  ★ 自动认出的地面绿 = %s（计数 %d ✓）' % (str(ground), cands[0][0]))
        else:
            print('  ✗ 自动认不出地面绿 ⇒ 用 --ground 显式给 ✓')

    print('  --- 主色 top8（★ 归一化比值 = 各分量 / 最大分量 ✓，✗ 与整体明暗无关 ✓）---')
    for cnt, rgb in rows[:8]:
        mx = max(rgb) or 1
        print('    (%3d,%3d,%3d)  %9d px  %5.1f%%  归一 %s'
              % (rgb[0], rgb[1], rgb[2], cnt, 100.0 * cnt / tot,
                 ' '.join('%.3f' % (v / mx) for v in rgb)))

    # ---- ② 背景色：取"最常出现的那个颜色"当候选，并用四角投票确认 ----
    corners = {}
    for (fx, fy) in ((0.02, 0.02), (0.97, 0.02), (0.02, 0.97), (0.97, 0.97)):
        # ★ 别把 `%[pixel:…]` 塞进 `%` 格式化里 ✗ —— `%[` 会被当格式符 ⇒ `ValueError` ✓（刚踩过 ✓）
        spec = '%[pixel:p{' + str(int(fx * W)) + ',' + str(int(fy * H)) + '}]'
        out = sh('convert', img, '-format', spec, 'info:').strip()
        corners[(fx, fy)] = out
    print('  --- 四角取样（★ 背景通常是**天空/底色**，占一整片 ✓）---')
    for k, v in corners.items():
        print('    (%.2f,%.2f)  %s' % (k[0], k[1], v))

    # ---- ③ 地面板包围盒占比 ----
    # ★ 做法：把图**降采样**到 720 宽再逐像素扫 ✓ —— ✗ 不靠 ImageMagick 的 `-opaque/-trim` ✗
    #   （那套在"颜色不在边缘"时会给出让人误读的框 ✓；降采样后扫描是**确定**的 ✓，
    #    百分比量本来也不需要全分辨率 ✓）
    if ground:
        r, g, b = ground
        tgt = sh('convert', img, '-colorspace', 'sRGB', '-depth', '8',
                 '-resize', '720x', 'txt:-')
        xs, ys, n, tot = [], [], 0, 0
        import re
        for line in tgt.splitlines():
            m = re.match(r'(\d+),(\d+):\s*\((\d+),(\d+),(\d+)', line)
            if not m:
                continue
            px, py, pr, pg, pb = (int(v) for v in m.groups())
            tot += 1
            if abs(pr - r) <= tol and abs(pg - g) <= tol and abs(pb - b) <= tol:
                xs.append(px)
                ys.append(py)
                n += 1
        print('  --- 地面绿 ≈ (%d,%d,%d)（±%d）---' % (r, g, b, tol))
        if not xs:
            print('    ✗ 一个都没匹配到（★ 先查尺子：这个颜色对不对 ✓）')
        else:
            # ★ 降采样后的图宽 = 720，高按比例 ⇒ 用采样网格自己的宽高算百分比 ✓
            #   ★ 注意别把局部变量叫 `sh` ✗ —— 会把模块级的 `sh()` 函数**遮蔽**掉 ✓（刚踩过 ✓）
            pts = [l.split(':')[0].split(',') for l in tgt.splitlines()
                   if re.match(r'^\d+,\d+:', l)]
            sw = max(int(p[0]) for p in pts) + 1 if pts else 1
            grid_h = max(int(p[1]) for p in pts) + 1 if pts else 1
            print('    包围盒 x∈[%d,%d] y∈[%d,%d]（采样网格 %dx%d）'
                  % (min(xs), max(xs), min(ys), max(ys), sw, grid_h))
            print('    ⇒ ★ 占宽 %.1f%% ｜ 占高 %.1f%% ｜ 面积占比 %.1f%%'
                  % (100.0 * (max(xs) - min(xs) + 1) / sw,
                     100.0 * (max(ys) - min(ys) + 1) / grid_h,
                     100.0 * n / max(1, tot)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
