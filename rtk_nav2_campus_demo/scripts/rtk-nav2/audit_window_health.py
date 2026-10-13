#!/usr/bin/env python3
"""audit_window_health.py —— 复算 `window_health.jsonl` 的**重启判据**（★ 全程，不用单点 ✗）

★ 为什么要有它（2026-10-10 实测 ✓）：★ 在 `campus8-demo2` 那轮上，
  ★ **`sim_clock` 全程回退过 1 次**（第 24→35 行，447 → 158 ✓），而
  ★★ **产物自记的 `clock_backjump` 全程都是 0** ✗ ⇒ ★ **产物自己的判据漏掉了一次真重启** ✗。

★ 它为什么漏（★ 2026-10-10 **读代码更正过一版** ✗ —— 第一版说"把读不到当成没变"，★ 那是错的 ✗）：
  ★ 实测 ✓：★ **没有该键的行只有 2 行**（全是 `startup` ✓ = 设计态 ✓）；★ **`null` 的有 9 行**，
    ★ 而它们**全部**标着 `status: "undetermined"` ✓ ⇒ ★ 产物**没有**把读不到当成没变 ✗（★ 它遵守了三态 ✓）。
  ★★ 真因 = ★ **计数器进程自己随栈一起重启** ✗**：
    ★ 本脚本用的 `window_health.sh` 写法是对的 ✓（★ 读不到时**保留**上一拍 ✓，✗ 不重置 ✗）；
    ★ 而 ★ **记录器与栈同生共死** ✓ ⇒ ★ 新实例开机时"上一拍"是空的 ✓
    ⇒ ★ **它从来没有可比的上一拍** ✓ ⇒ ★ 回退计数在自己的重启上**天然盲** ✗
  ⇒ ★★ 不是实现 bug，是**设计边界** ✓ ⇒ ★ 已记档那条正对：★ "**比较基准若活在内存里，
     一次重启会既制造异常、又擦掉证据 ⇒ 基准要落盘、从产物末行恢复**" ✓✓
  ⇒ ★ 修法 = ★ **`prev_sim` 落盘、下一拍从产物末行恢复** ✓（★ ✗ 不是"别在 null 上重置"——它没重置 ✗）。

★ 所以本脚本给**三路**读数，并**只对可靠的那一路下断言** ✓：
  ① ★ `sim_clock` **全程单调** ✓（★ 回退 = 事件，★ 连**发生时刻**一起报 ✓）—— ★ 这是**重启的充分证据** ✓
  ② ★ **身份**：`mine_set_sha256` 的取值集合与**变化点** ✓ —— ★ **不依赖时钟** ✓，
     实测那次它的中间值 = **空串的 sha256**（`e3b0c442…` ✓）⇒ ★ 集合**空过** ✓ ⇒ ★ 身份判据抓住了 ✓
  ③ ★ `t_epoch − sim_clock` 的漂移 —— ★ **它不是重启证据** ✗（★ sim 慢于墙钟/被暂停是正常形态 ✓）：
     实测同一份产物漂了 **2487 s** ✓、★ 而真回退只有 **289 s** ✓ ⇒ ★ 只报数、**不下重启结论** ✓。

用法: audit_window_health.py <window_health.jsonl>
★ 退出码：0 = 跑完（★ 不代表"没问题"✗）｜ 2 = 用法错
"""
import json
import sys


def main(path):
    rows = []
    for ln in open(path, encoding='utf-8'):
        ln = ln.strip()
        if ln:
            try:
                rows.append(json.loads(ln))
            except Exception:
                pass
    print("  行数 = %d" % len(rows))
    have = [r for r in rows if r.get('sim_clock') is not None]
    # ★★ 两件事必须**分开数** ✗（2026-10-10 我并成"缺字段"报过一次，是错的 ✗）：
    #   ① ★ **没有这个键** ⇒ ★ 设计态（`startup` 行 ✓，实测 2 行 ✓）
    #   ② ★ **键在、值为 null** ⇒ ★ "**读不到**" ⇒ ★ 应带 `status: undetermined` ✓（实测 9 行 ✓，全部带 ✓）
    nokey = sum(1 for r in rows if 'sim_clock' not in r)
    isnull = sum(1 for r in rows if 'sim_clock' in r and r['sim_clock'] is None)
    st_null = sorted(set(str(r.get('status')) for r in rows
                         if 'sim_clock' in r and r['sim_clock'] is None))
    print("  有 sim_clock 的行 = %d" % len(have))
    print("  ★ 没有该键的行 = %d（★ 设计态：startup 行 ✓）" % nokey)
    print("  ★★ 键在、值为 null 的行 = %d ⇒ ★ 这是「读不到」✓，★ 其 status = %s"
          % (isnull, st_null))
    if isnull and st_null != ['undetermined']:
        print("      ✗ **值 null 却没标 undetermined** ⇒ ★ 那才是「把读不到当没变」✗")

    print("  --- ① sim_clock 全程回退（★ 事件 ＋ 时刻 ✓）---")
    bj = 0
    for i in range(1, len(have)):
        a, b = have[i - 1], have[i]
        if b['sim_clock'] < a['sim_clock']:
            bj += 1
            print("    ★ sim_clock %s → %s（回退 %s s）｜ t_epoch %s → %s（Δ=%.1f s）"
                  % (a['sim_clock'], b['sim_clock'], a['sim_clock'] - b['sim_clock'],
                     a.get('t_epoch'), b.get('t_epoch'),
                     (b.get('t_epoch') or 0) - (a.get('t_epoch') or 0)))
    print("    ⇒ 全程回退次数 = %d  ⇒ %s" % (bj, "无回退 ✓" if bj == 0 else "★★ 有回退 ⇒ 重启过 ✗"))

    cb = sorted(set(str(r.get('clock_backjump')) for r in rows if 'clock_backjump' in r))
    n_cb = sum(1 for r in rows if 'clock_backjump' in r)
    print("  --- ② 产物自记 clock_backjump 取值集合 = %s（%d 行有）" % (cb, n_cb))
    if bj > 0 and cb == ['0']:
        print("      ★★ ①有回退而这里只有 ['0'] ⇒ ★ **产物自己的判据漏掉了这次重启** ✗")
        print("      ⇒ ★ **别用 `clock_backjump == 0` 当「无重启」那条臂** ✗；★ 用下面的身份判据 ✓")

    ms = [(r.get('t_epoch'), str(r.get('mine_set_sha256'))) for r in rows if r.get('mine_set_sha256')]
    print("  --- ③ 身份 mine_set_sha256（★ 不依赖时钟 ✓，%d 行有）---" % len(ms))
    prev = None
    for t, v in ms:
        if v != prev:
            print("    t_epoch=%s ⇒ %s  ★ 变化点 ✓" % (t, v[:24]))
            prev = v
    print("      不同取值数 = %d" % len(set(v for _, v in ms)))
    ib = sorted(set(str(r.get('instances_by_partition')) for r in rows
                    if r.get('instances_by_partition') is not None))
    print("      instances_by_partition 取值集合 = %s" % ib)

    off = [(r['t_epoch'] - r['sim_clock']) for r in have if r.get('t_epoch') is not None]
    if len(off) >= 2:
        print("  --- ④ t_epoch − sim_clock 漂移（★ 只报数，✗ 不下重启结论 ✗）---")
        print("    首 = %.1f s ｜ 末 = %.1f s ｜ 漂移 = %.1f s" % (off[0], off[-1], off[-1] - off[0]))
        print("    ★ 漂移 = sim 比墙钟慢/被暂停的正常形态 ✓ ⇒ ★ **它不是重启证据** ✗（只有①是 ✓）")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    main(sys.argv[1])
