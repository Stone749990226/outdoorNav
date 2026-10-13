#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arc_pitch_watcher —— ★ 俯仰**超阈值**时**确定性**地去抓 RViz 代价图那一窗 ✓

用法（★ 宿主侧 ✓，由 `cutS8-run.sh` 在开车那一刻后台拉起 ✓）：
  arc_pitch_watcher.py <run-id> [--threshold-deg 2.0] [--interval 1.0]
                       [--max-shots 8] [--min-gap 8] [--max-seconds 3600] [--out <json>]

★★ 为什么要它（★ reviewer 2026-10-11 派 ✓）★★
  ★ 业主那个待定问题② = ★ **"代价图上那个弧"** ✓ —— ★ 弧**只画在 RViz 的代价图上** ✗
    （★ Gazebo 里看不见 ✓）⇒ ★ 关 RViz 的档**根本没有这个证据** ✗。
  ★ 而弧是**瞬态**的 ✗（★ "启停时出现、姿态恢复后被清空" ✓）⇒ ★ **靠抓拍碰运气必然抓不到** ✗
    ⇒ ★★ 触发必须是**确定性的** ✓：★ **用该拍实测的俯仰**（★ 探针逐拍就写着 ✓）当扳机 ✓。

★★ 扳机用的尺（★ 与离线那把**逐位同一把** ✓）★★
  ★ 该拍 `tick_*.json` 的 `scan.tf.M` ✓（★ map ← near_lidar ✓，✗ 不用 `robot_tf` ✗ —— 它 roll／pitch 恒 0 ✓）
  ★ 法向 `n = R @ [0,0,1]` ⇒ ★ `n_z = M[2][2]` ✓ ⇒ ★ **面倾角** `tilt = acos(|n_z|)` ✓
  ★ 标定 ✓：★ 用它在 cutS7 上算，★ `>1°` 得 **35 拍** ✓ —— ★ 与早先离线那把**报的数一样** ✓
    （★ 分布：★ cutS7 p50 **0.299°** ✓／max **5.347°** ✓；★ res1 p50 0.300／max 13.135 ✓）
  ★ 阈值取 **2.0°**（★ reviewer 给的例子 ✓）：★ 各轮实测 `>2°` 有 **23／47／34 拍** ✓ ⇒ ✗ 不会"一次都不触发" ✗。

★★ 它**不进判据** ✓ ★★
  ★ 本轮的判据走**探针 ＋ 逐拍 dump** ✓；★ 截图只是**附加证据** ✓（★ 别拿它当判据 ✓）。

★★ 抓的方法 ✓：★ 直接调**已验证的那个工具** `window_capture.sh` ✓（★ ✗ 不在这里再写第二份 ✗）
  —— 它自带结构判据（★ 几何互斥 ＋ 内容归属 ＋ 亮度 ✓），★ 正是"这张图是不是真截到了"那一问 ✓。
"""
import argparse
import glob
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = '/mnt/c/Users/xjy/projects/robonix'


def seq_of(p):
    m = re.search(r'_(\d+)\.', os.path.basename(p))
    return int(m.group(1)) if m else -1


def tilt_deg(tick_path):
    """★ 该拍的面倾角（度）✓；★ 取不到 ⇒ None（★ ✗ 不许填 0 ✗）"""
    try:
        d = json.load(open(tick_path, encoding='utf-8'))
    except Exception:
        return None, None
    sc = d.get('scan') or {}
    M = (sc.get('tf') or {}).get('M')
    if not M or len(M) < 3 or len(M[2]) < 3:
        return None, d.get('seq')
    nz = max(-1.0, min(1.0, float(M[2][2])))
    return math.degrees(math.acos(abs(nz))), d.get('seq')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_id')
    ap.add_argument('--threshold-deg', type=float, default=2.0)
    ap.add_argument('--interval', type=float, default=1.0)
    ap.add_argument('--max-shots', type=int, default=8)
    ap.add_argument('--min-gap', type=float, default=8.0)
    ap.add_argument('--max-seconds', type=float, default=3600.0)
    ap.add_argument('--out', default=None)
    ap.add_argument('--no-arrange-on-occlusion', action='store_true',
                    help='★ 默认：★ 抓失败且原因是"窗口被压住"时，★ 调一次 arrange-windows.sh 再重试 ✓')
    a = ap.parse_args()

    RUN = os.path.join(PROJ, 'artifacts/rtk-nav2', a.run_id)
    PROBE = os.path.join(RUN, 'probe')
    SHOTS = os.path.join(RUN, 'screenshots')
    JSONL = os.path.join(RUN, 'arc_trigger_log.jsonl')
    out = a.out or os.path.join(RUN, 'arc_trigger_log.json')
    os.makedirs(SHOTS, exist_ok=True)

    # ★ 环境：★ 与 lib.sh 的 capture_screen **同一套** ✓（★ window_capture.sh 自己**不设** DISPLAY ✗）
    env = dict(os.environ)
    disp = env.get('DEMO_DISPLAY', ':0')
    env['DISPLAY'] = disp if disp.endswith('.0') else disp + '.0'
    xauth = env.get('XAUTHORITY')
    if not xauth:
        for cand in ('/mnt/wslg/.Xauthority', os.path.expanduser('~/.Xauthority')):
            if os.path.isfile(cand):
                xauth = cand
                break
    if xauth:
        env['XAUTHORITY'] = xauth

    rec = {'run_id': a.run_id, 'threshold_deg': a.threshold_deg, 'interval_s': a.interval,
           'max_shots': a.max_shots, 'min_gap_s': a.min_gap, 'max_seconds': a.max_seconds,
           'ruler': ('★ `scan.tf.M` ⇒ `n_z = M[2][2]` ⇒ `tilt = acos(|n_z|)` ✓'
                     '（★ 与离线那把同一把 ✓：★ 用它算 cutS7 的 `>1°` 得 35 拍 ✓）'),
           'display': env['DISPLAY'], 'xauthority': env.get('XAUTHORITY'),
           'note': '★ 截图**只是附加证据** ✓、★ 不进判据 ✗（★ 判据走探针 ＋ 逐拍 dump ✓）',
           'trigger_rule': '★ tilt > %.1f° ∧ 距上一张 ≥ %.0f s ∧ 张数 < %d ✓' % (a.threshold_deg, a.min_gap, a.max_shots),
           'shots': [], 'n_ticks_seen': 0, 'tilt_max_deg': None, 'started_at': time.time()}
    # ★★ 收尾必须**落盘** ✓（★ 2026-10-11 实测的坑 ✗）
    #   ★ 我第一版只在**循环正常结束**时才写 `arc_trigger_log.json` ✗ ⇒ ★ 而 `cutS8-run.sh` 是
    #     **按 PID 收它**（★ 处置三件套 ✓）⇒ ★ 收到 TERM 就直接死了 ✗ ⇒ ★ **汇总产物一份都没有** ✗
    #     （★ 只剩逐条追加的 `.jsonl` ✓ ⇒ ★ 汇总只能事后自己拼 ✗ ＝ 与"算出来了 ≠ 交出去了"同族 ✓）。
    #   ⇒ ★ 现在：★ 收到 TERM／INT ⇒ ★ 置旗 ⇒ ★ **跳出循环 ⇒ 照常落盘** ✓。
    stop = {'flag': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('flag', True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__('flag', True))
    t0 = time.time()
    last_shot = -1e9
    seen = set()

    while not stop['flag'] and time.time() - t0 < a.max_seconds:
        files = glob.glob(os.path.join(PROBE, 'tick_*.json'))
        if files:
            newest = max(files, key=seq_of)
            if newest not in seen:
                seen.add(newest)
                t, s = tilt_deg(newest)
                rec['n_ticks_seen'] += 1
                if t is not None:
                    if rec['tilt_max_deg'] is None or t > rec['tilt_max_deg']:
                        rec['tilt_max_deg'] = round(t, 4)
                    now = time.time()
                    if t > a.threshold_deg and (now - last_shot) >= a.min_gap \
                            and len(rec['shots']) < a.max_shots:
                        tag = 'arc_seq%04d_tilt%.2f' % (s if s is not None else -1, t)
                        d = os.path.join(SHOTS, tag)
                        os.makedirs(d, exist_ok=True)
                        shot = {'tick_seq': s, 'tilt_deg': round(t, 4), 'at': time.strftime('%H:%M:%S'),
                                'out_dir': os.path.relpath(d, PROJ)}

                        def _cap():
                            return subprocess.run([os.path.join(HERE, 'window_capture.sh'), d, 'rviz', '1'],
                                                  capture_output=True, text=True, env=env, timeout=300)

                        r = _cap()
                        # ★★ 2026-10-11 实测（cutS8 ✓）：★ 触发**成功**了 8 次 ✓ 而**每一张都 rc=1 ✗** ——
                        #   ★ 原因是 `window_capture.sh` 的**判据②（内容归属）** ✗：★ RViz 那几窗被
                        #   **外来窗口压住** ✓ ⇒ ★ `xwd -id` 拿到的是**别人的像素** ✗ ⇒ ★ **它拒绝出图** ✓
                        #   （★ 这正是它该做的 ✓ —— ★ "出图了 ≠ 截到了" ✓）。
                        #   ★ 它自己给的补救就在输出里："★ **先排布再抓**" ✓ ⇒ ★ 这里照做 ✓：
                        #   ★ **排一次**（★ 一次就够 ✓，✗ 不每张都排 ✗）→ ★ **重试这一张** ✓，★ 并记账 ✓。
                        shot['rc'] = r.returncode
                        shot['capture_tail'] = (r.stdout or '')[-800:]
                        occ = [ln.strip() for ln in (r.stdout or '').split('\n') if '压住' in ln]
                        if r.returncode != 0 and occ:
                            shot['failure_reason'] = occ[0]
                            if not a.no_arrange_on_occlusion and not rec.get('arranged'):
                                ar = subprocess.run([os.path.join(HERE, 'arrange-windows.sh')],
                                                    capture_output=True, text=True, env=env, timeout=600)
                                rec['arranged'] = True
                                rec['arrange_rc'] = ar.returncode
                                rec['arrange_tail'] = (ar.stdout or '')[-800:]
                                r = _cap()
                                shot['retried_after_arrange'] = True
                                shot['rc_after_arrange'] = r.returncode
                                shot['capture_tail_after_arrange'] = (r.stdout or '')[-800:]
                        shot['rc'] = r.returncode
                        pngs = sorted(glob.glob(os.path.join(d, '*.png')))
                        shot['n_png'] = len(pngs)
                        shot['png'] = [os.path.relpath(p, PROJ) for p in pngs]
                        rec['shots'].append(shot)
                        with open(JSONL, 'a', encoding='utf-8') as fh:
                            fh.write(json.dumps(shot, ensure_ascii=False) + '\n')
                        last_shot = now
        time.sleep(a.interval)

    rec['elapsed_s'] = round(time.time() - t0, 1)
    rec['n_shots'] = len(rec['shots'])
    json.dump(rec, open(out, 'w', encoding='utf-8'), ensure_ascii=False, indent=2)
    print('★ 触发轮：看过 %d 拍 ｜ tilt max %s° ｜ 截图 %d 张 ⇒ %s'
          % (rec['n_ticks_seen'], rec['tilt_max_deg'], rec['n_shots'], out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
