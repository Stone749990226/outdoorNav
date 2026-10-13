#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""residue_selftest.py —— ★ 给 `residue_analyze.py` 做**两头标定**（两份合成输入 ＋ 已知真值 ✓）

用法: residue_selftest.py <输出目录>

★ 为什么必须有它（本项目定式 ✓）：★ **"正向那次通过"信息量最低** ✗ ——
  一个**恒过**的分析器在真实数据上照样报一堆"残留" ✓ ⇒ ★ 两头都要验 ✓：
    ★ **该报的必须报出来** ✓；★ **不该报的必须不报** ✗。

★ 两份合成输入（★ 缺哪一份，标定都不算数 ✓）：
  ★ **A 有残留** ✓：埋 3 格"后半段每拍都占用" ＋ 1 格"早先占用后被清空" ✗ ＋ 1 格"只在末段" ✗
  ★ **B 干净** ✓：★ **每一格占用后都回到自由** ⇒ ★ 残留集合必须**恰好为 0** ✗
    （★ 没有这份 ⇒ ★ 一个恒过、见谁都报的分析器**照样通过 A** ✗）

★★ **埋点一律取"非对角"格** ✗（★ 本例 `(10,30)`／`(12,31)`… ✓）—— ★ 理由：
  ★ 本项目 `rev8_dumpdiff.py` **行列转置**栽过 ✓（★ 差点把"通过"报成"不通过" ✗）；
  ★ 若埋点落在**对角**上（i==j ✓），★ 转置**照样对** ✗ ⇒ ★ 那份自检**验不出转置** ✗。

★★★ **但要写清楚：★ "非对角"本身**不**是转置的守卫 ✗**（2026-10-11 实测更正 ✗）★：
  ★ 本自检的合成输入**是用 `cell_of`／`world_of` 同一套换算造的** ✓ ⇒ ★ "埋点"与"找点"**一起转** ⇒
    ★ **自洽** ✗ ⇒ ★ A／B 两组**不保证**报 ✗。
  ★ 实测（`residue_negcalib.sh` 三种变体 ✓）：★ 只转 `cell_of` ⇒ 红的是 **A⑦＋C①＋C③** ✓；
    ★ 只转 `world_of` ⇒ 红的是 **A④＋A⑤＋C②** ✓；★ 两个都转 ⇒ 六项**全红** ✓。
  ★ ⇒ ★ **没有"某一项"能单独叫守卫** ✗；★ 唯一能说的是 ★ **C 组（直接打函数那三项）每次至少一项变红** ✓。
  ★ 所以 ★ **别把"非对角"记成转置守卫** ✗ —— ★ 它防的是"**埋点退化到对角**"这一件事 ✗，是另一回事 ✓。

退出码: 0 = 全部符合预期 ｜ 1 = 有不符合
"""
import csv
import json
import os
import subprocess
import sys

W = H = 40
RES = 0.1
OX = OY = 0.0
N_TICK = 12
HERE = os.path.dirname(os.path.abspath(__file__))
ANALYZE = os.path.join(HERE, 'residue_analyze.py')

# ★ 真值（★ 全部非对角 ✓）
RES_SCAN, RES_CAM, RES_NONE = (10, 30), (12, 31), (14, 32)
TRANSIENT, LATE = (20, 21), (31, 20)
M0 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]


def ctr(ij, res=RES, ox=OX, oy=OY):
    i, j = ij
    return ox + (i + 0.5) * res, oy + (j + 0.5) * res


def build(d, case):
    probe = os.path.join(d, 'probe')
    os.makedirs(probe, exist_ok=True)

    def grid(seq):
        g = [0] * (W * H)
        for j in range(H):                       # 自由区 + 未知区（两头都要有 ✓）
            for i in range(W):
                g[j * W + i] = 0 if (i < 35 and j < 35) else -1
        if case == 'A':
            if seq >= 4:                         # ★ 后半段每拍都占用 ⇒ 稳定残留 ✓
                for ij in (RES_SCAN, RES_CAM, RES_NONE):
                    g[ij[1] * W + ij[0]] = 100
            if seq <= 4:                         # ★ 早先占用、之后清空 ⇒ 不许进残留 ✗
                g[TRANSIENT[1] * W + TRANSIENT[0]] = 99
            if seq >= N_TICK - 2:                # ★ 只末段出现 ⇒ 不许进"稳定残留" ✗
                g[LATE[1] * W + LATE[0]] = 90
        else:                                    # ★ B 干净：每一格都会被清掉 ✓
            g[RES_SCAN[1] * W + RES_SCAN[0]] = 100 if seq <= N_TICK - 3 else 0
            g[TRANSIENT[1] * W + TRANSIENT[0]] = 99 if seq <= 4 else 0
        return g

    for seq in range(1, N_TICK + 1):
        with open(os.path.join(probe, 'map_%04d.bin' % seq), 'wb') as fh:
            fh.write(bytes((v & 0xFF) for v in grid(seq)))
        info = {'stamp': '%d.0' % seq, 'resolution': RES, 'width': W, 'height': H,
                'origin': {'x': OX, 'y': OY}, 'cells_total': W * H}
        json.dump(info, open(os.path.join(probe, 'map_%04d.json' % seq), 'w'), indent=2)
        rx = round(1.0 + 0.3 * min(seq, 8), 3)
        json.dump({'seq': seq, 'wall_epoch': 1e9 + seq, 'sim_t': float(seq * 2),
                   'map_recv': seq, 'map': info, 'robot_xy_map': [rx, 1.0],
                   'robot_tf': {'M': [[1, 0, 0, rx], [0, 1, 0, 1.0], [0, 0, 1, 0]],
                                'stamp_or_err': '%d.0' % seq},
                   'scan': {'frame_id': 'base_footprint', 'stamp': '%d.0' % seq,
                            'n_total': 3, 'n_kept': 3, 'tf': {'M': M0, 'stamp_or_err': '0.0'}},
                   'cam': {'frame_id': 'camera_optical_frame', 'stamp': '%d.0' % seq,
                           'n_total': 1, 'n_kept': 1, 'tf': {'M': M0, 'stamp_or_err': '0.0'}}},
                  open(os.path.join(probe, 'tick_%04d.json' % seq), 'w'),
                  ensure_ascii=False, indent=2)
        sx, sy = ctr(RES_SCAN)
        tx, ty = ctr(TRANSIENT)
        open(os.path.join(probe, 'scan_%04d.xyz' % seq), 'w').write(
            '%.4f %.4f %.4f\n%.4f %.4f %.4f\n%.4f %.4f %.4f\n'
            % (sx, sy, 0.02, tx, ty, 0.02, 5.0, 5.0, 0.02))   # ★ 末点是"落在自由格"那一头 ✓
        cx, cy = ctr(RES_CAM)
        open(os.path.join(probe, 'cam_%04d.xyz' % seq), 'w').write('%.4f %.4f %.4f\n' % (cx, cy, 0.85))
    return probe


def run(d):
    r = subprocess.run([sys.executable, ANALYZE, d], capture_output=True, text=True)
    return r, os.path.join(d, 'probe')


def read(probe):
    a = json.load(open(os.path.join(probe, 'analysis.json'), encoding='utf-8'))
    cells = list(csv.DictReader(open(os.path.join(probe, 'residue_cells.csv'), encoding='utf-8')))
    got = {(int(c['i']), int(c['j'])) for c in cells}
    src = {(int(c['i']), int(c['j'])): c['src'] for c in cells}
    return a, got, src, cells


def main():
    root = sys.argv[1]
    checks = []

    def chk(name, cond, detail=''):
        checks.append((name, bool(cond), detail))

    # ---------- ★ 输入 A：该出残留 ----------
    dA = os.path.join(root, 'caseA')
    probeA = build(dA, 'A')
    rA, _ = run(dA)
    print(rA.stdout)
    if rA.returncode != 0:
        print('✗ A 分析器退出码 %d\n%s' % (rA.returncode, rA.stderr[-600:]))
        return 1
    aA, gotA, srcA, cellsA = read(probeA)
    chk('A① 三格"一直占用"都判出来', {RES_SCAN, RES_CAM, RES_NONE} <= gotA, str(sorted(gotA)))
    chk('A② 早先占用、后被清空的格**没**进残留', TRANSIENT not in gotA, str(TRANSIENT))
    chk('A③ 只在末段出现的格**没**进"稳定残留"', LATE not in gotA, str(LATE))
    chk('A④ 归因 scan 正确', srcA.get(RES_SCAN) == 'scan', str(srcA.get(RES_SCAN)))
    chk('A⑤ 归因 cam 正确', srcA.get(RES_CAM) == 'cam', str(srcA.get(RES_CAM)))
    chk('A⑥ 无源那片归 none（✗ 不许硬塞给某个源 ✗）', srcA.get(RES_NONE) == 'none',
        str(srcA.get(RES_NONE)))
    chk('A⑦ 正向对照两头都出现', aA.get('align_ok') is True)
    chk('A⑧ 高度记到了（scan z≈0.02）',
        all(abs(float(c['z_scan']) - 0.02) < 1e-6 for c in cellsA
            if c['src'] == 'scan' and c['z_scan'] not in ('', 'None')))
    chk('A⑨ 集合恰好等于埋的三格',
        gotA == {RES_SCAN, RES_CAM, RES_NONE}, str(sorted(gotA)))

    # ---------- ★ 输入 B：该**没有**残留 ----------
    dB = os.path.join(root, 'caseB')
    probeB = build(dB, 'B')
    rB, _ = run(dB)
    print(rB.stdout)
    if rB.returncode != 0:
        print('✗ B 分析器退出码 %d\n%s' % (rB.returncode, rB.stderr[-600:]))
        return 1
    aB, gotB, _, _ = read(probeB)
    chk('★B① 干净输入 ⇒ 稳定残留**恰好 0** ✗（★ 这条挡"恒过" ✓）', len(gotB) == 0, str(sorted(gotB)))
    chk('★B② 干净输入 ⇒ 两时刻差分也是 0', aB.get('two_time_cells') == 0,
        str(aB.get('two_time_cells')))

    # ---------- ★ 行列对齐（★ 直接打函数，不靠图 ✓）----------
    sys.path.insert(0, HERE)
    import residue_analyze as RA                                        # noqa: E402
    info = {'resolution': RES, 'width': W, 'height': H, 'origin': {'x': OX, 'y': OY}}
    wx, wy = ctr(RES_SCAN)                                              # (1.05, 3.05)
    chk('★C① 世界→格 = (i=列↔x, j=行↔y) 正确', RA.cell_of(info, wx, wy) == RES_SCAN,
        str(RA.cell_of(info, wx, wy)))
    chk('★C② 反向 格→世界 一致',
        tuple(round(v, 4) for v in RA.world_of(info, *RES_SCAN)) == (round(wx, 4), round(wy, 4)),
        str(RA.world_of(info, *RES_SCAN)))
    chk('★C③ 转置写法**必须**判为不同（★ 两头都标定 ✓）',
        RA.cell_of(info, wx, wy) != (RES_SCAN[1], RES_SCAN[0]))

    print('=== 自检结果（★ 两份输入 ＋ 对齐 ✓）===')
    bad = 0
    for name, ok, detail in checks:
        print('  %s %s %s' % ('✓' if ok else '✗', name, ('｜' + detail) if detail else ''))
        bad += 0 if ok else 1
    print('  ★ A 稳定残留 = %s（真值 %s）' % (sorted(gotA), sorted({RES_SCAN, RES_CAM, RES_NONE})))
    print('  ★ B 稳定残留 = %s（真值 [] 空 ✓）' % sorted(gotB))
    print('  ★ 合计 %d 项，%s' % (len(checks), '全过 ✓' if bad == 0 else '有 %d 项不过 ✗' % bad))
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
