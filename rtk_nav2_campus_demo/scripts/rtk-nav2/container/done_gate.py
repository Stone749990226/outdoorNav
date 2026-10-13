#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**产物自检四件**通过之后，才允许写 `MEASUREMENTS_DONE`；并把**产物清单**附在标记旁边。

## 为什么要有这个（有实例，不是理论风险）

- `nav10-r3` 报"长窗口复验 = 不冻"，但**① 长窗口那一档一帧产物都没有**
  （`onset_log.txt` 全文只有 `NameError`；`onset_cost.csv/json` 不存在），
  **而 `MEASUREMENTS_DONE` 照样被写下、栈照样按流程停了** ✗
  ⇒ `touch MEASUREMENTS_DONE` 只证明"驱动走到了写标记那一步"，**不证明每一档都有产物**。
- `MEASUREMENTS_DONE` 当时是 **0 字节** ⇒ 连"谁写的、什么时候写的、走的哪条路"都读不出来。

⇒ 两条互补的判据，**缺一个就还能被同一形态骗第二次**：
    · **标记**管"**到没到那一步**"；
    · **产物路径 + 行数**管"**那一步有没有东西**"。
本脚本把这两条**绑成一步**：标记只在**四件**全过之后才写，且标记自身携带清单。

## 判据（**四件**，缺一不可）

  1. **存在**   —— 该档的产物文件在不在；
  2. **非空**   —— 字节数 > `--min-bytes`（默认 1；**0 字节不算产物**）；
  3. **带生产者头** —— 头里必须有**生产者 + 时刻 + 途径**三要素。
     · 原生带头的（如 `params_*.yaml` 的 `# produced_by:` 四字段）**直接核**；
     · 原生不带头的（CSV/JSON 这种数据文件，加 `#` 头会破坏它们的可读性）
       ⇒ 本脚本给它写一份**旁挂头** `<产物>.provenance`，内容**由该产物的实际字节算出来**
       （sha256 + 字节数 + 行数 + 生产者 + 时刻 + 途径 + run_id），
       再核这份旁挂头的三要素。**不是"假装有头"**：头里的 sha256 对不上就算不过。

  4. **结构完整性** —— 行数 / **末行完整** / **字段数**（关键产物再加"**能解析**"）。
     ★ 为什么补这一件（2026-09-21 实测，有实例）：**前三件查不出"被写坏"** ——
       `nav11-r1/gates.txt` **存在、非空、并有旁挂生产者头、mtime 也新鲜 ⇒ 前三件全过 ✓**，
       但**内容其实是坏的**：驱动用 `> gates.txt` 把 stdout 重定向到**工具自己也在写**的同一个
       文件 ⇒ 两个写入者互相截断/错位，文件里出现**半截行**
       （实测：`  ⇒ 四张图各真收到一帧（判据：收帧，不是 ``topic list`` 里` 被切断，
        末行只剩 `.py 直连活值替代）`）。
       ⇒ **「能读」≠「读全」**。这一件就是为这个边界加的。
     ★ 但**光加自检不够**：这一类坏掉**根子在"同一份证据有两个写入者"**，
       所以**修法必须落到机制层** ⇒ 驱动**不许对「工具自己会写的文件」做 `>` 重定向**
       （`r1_driver.sh` 里那行 `> "${R}/gates.txt"` 已删，stdout 改落到 `gates_stdout.txt`）。
       自检是**兜底**：万一以后又有人加了一条 `>`，这里会当场拦住。

     具体查法**按格式分工**（第一版不分格式，结果满屏假阳性 —— 见 `_structure` docstring）：
       · **`.json`** ⇒ **`json.loads` 能过**即通过（解析器给确切出错位置）；
         **不**做逐行配对、**不**要求尾换行（我们自己 `json.dump` 就不写尾换行）；
       · **`.csv`** ⇒ `csv.reader` 全表**字段数与表头一致** ⇒ 记录 **行数 × 列数**；
         **不**做逐行配对、**不**要求尾换行；
       · **其它文本** ⇒ ① 无 NUL 字节 ② **以换行结尾**（写坏的常缺尾换行）
         ③ **逐行括号配对**（`（）`/`[]`/`{}`）—— **行被截断最典型的特征**；
       · **行数**一律记录；可用 `--expect-lines <相对路径>=<精确行数>` 明确判（可选）。

★ 另有一条**同样要机器可判**的：**产物必须是本轮产出的**（mtime ≥ `--start-epoch`）。
  光"存在且非空"拦不住"上一轮留下的同名文件"——而本项目的产物目录是**按 run-id 分**的、
  同一 run-id 重复起栈会**覆盖**同名产物（见 `start-demo.sh` 的 `stack_start_index`），
  所以"同一个 run-id 里混进上一轮的文件"是真实可能的，不能只靠人记得。

用法（容器内）：
  done_gate.py --run-dir /work/artifacts/rtk-nav2/<run-id> \
               --start-epoch <驱动开始的墙钟 epoch> \
               --require r4_longseq.csv --require r4_longseq.json ...
退出码：0 = **四件**全过（标记已写）；1 = 有不过的（标记**未**写，写了 INCOMPLETE）。
"""

import argparse
import csv
import hashlib
import json
import os
import re
import sys
import time


def _sha16(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


def _lines(path):
    """文本产物的行数；二进制 / 读不动就返回 None（**不猜**）。"""
    try:
        with open(path, "rb") as f:
            raw = f.read()
    except OSError:
        return None
    if b"\x00" in raw[:4096]:
        return None
    return raw.count(b"\n") + (0 if raw.endswith(b"\n") else 1)


def _native_header(path):
    """核前 6 行里的 `# produced_by/produced_at/via` 三要素。返回 (bool, 缺失项列表)。"""
    try:
        with open(path, "r", errors="replace") as f:
            head = [next(f, "") for _ in range(6)]
    except OSError:
        return False, ["读不到文件"]
    txt = "".join(head)
    missing = [k for k in ("produced_by:", "produced_at:", "via:")
               if ("# %s" % k) not in txt]
    return (not missing), missing


_PAIRS = [("（", "）"), ("(", ")"), ("[", "]"), ("{", "}")]
_TEXT_EXT = (".txt", ".log", ".yaml", ".yml", ".md", "")


def _structure(path, rel):
    """第 4 件：**结构完整性**。返回 (bad列表, notes列表, detail)。**bad 为空 = 通过**。

    ## 为什么要按格式分开查（第一版踩到的坑，记下来别再犯）

    第一版对所有文件都"逐行查括号配对 + 必须换行结尾"，结果**满屏假阳性**：
      · `.json` 里 `{` 在第 1 行、`}` 在最后一行是**正常的** ⇒ 逐行配对**根本不成立**；
      · 我们自己的 `json.dump(...)` **不写尾换行** ⇒ 所有 JSON 都被判"末行不完整"。
    ⇒ **结构化格式有自己的权威解析器，别用文本启发式去猜**。分工是：

      `.json` ⇒ **`json.loads` 能过**就是硬判据（解析器会指出确切的出错位置）；
                **不做**逐行配对、**不做**尾换行要求。
      `.csv`  ⇒ `csv.reader` 全表字段数与表头一致；**不做**逐行配对、**不做**尾换行要求
                （CSV 的"末行截断"表现为字段数变化，用字段数查更准）。
      **其它文本**（`.txt`/`.log`/`.yaml`/无扩展名）⇒ 才用文本启发式：
                ① 不许有 NUL 字节；② **必须以换行结尾**（被两个写入者交错写坏的文件常常缺尾换行）；
                ③ **逐行括号配对**——**这是"行被截断"最典型的特征**
                   （`nav11-r1/gates.txt` 那两行正是如此，实测被这条当场抓到）。
    """
    notes, bad = [], []
    try:
        raw = open(path, "rb").read()
    except OSError as e:
        return ["读不到文件：%s" % e], [], {}
    detail = {"bytes": len(raw)}
    if b"\x00" in raw:
        bad.append("文件里有 %d 个 NUL 字节 ⇒ **稀疏洞/被交错写坏**" % raw.count(b"\x00"))
    txt = raw.decode("utf-8", "replace")
    lines = txt.split("\n")
    if lines and lines[-1] == "":
        lines = lines[:-1]
    detail["lines"] = len(lines)
    ext = os.path.splitext(rel)[1].lower()

    if ext == ".json":
        # ---- 权威判据：能不能解析 ----
        try:
            obj = json.loads(txt)
            notes.append("JSON 能解析 ✓（顶层 %s）" % type(obj).__name__)
        except Exception as e:
            bad.append("**JSON 解析失败**：%s" % e)
        # 结构化格式不查逐行配对/尾换行（见 docstring）
        return bad, notes, detail

    if ext == ".csv":
        try:
            rows = list(csv.reader(txt.splitlines()))
            if not rows:
                bad.append("CSV 没有表头行")
            else:
                n = len(rows[0])
                detail["csv_fields"] = n
                off = [(i + 1, len(r)) for i, r in enumerate(rows) if len(r) != n]
                if off:
                    bad.append("**CSV 字段数不一致**（表头 %d 列）：行号/实际列数 %s" % (n, off[:5]))
                else:
                    notes.append("CSV 字段数一致：**%d 行 × %d 列**（表头 + %d 条记录）✓"
                                 % (len(rows), n, len(rows) - 1))
        except Exception as e:
            bad.append("**CSV 解析失败**：%s" % e)
        return bad, notes, detail

    # ---- 其余按文本处理 ----
    if ext in _TEXT_EXT:
        if not raw.endswith(b"\n"):
            bad.append("**末行不完整**（文件不以换行结尾，最后一行为 %r）"
                       % raw.split(b"\n")[-1][:60])
        if lines and not lines[-1].strip():
            bad.append("末行是空行（可能被截断后补了换行）")
        unbal = []
        for i, ln in enumerate(lines, 1):
            for o, c in _PAIRS:
                if ln.count(o) != ln.count(c):
                    unbal.append((i, o, c, ln.count(o), ln.count(c), ln[:70]))
                    break
        if unbal:
            detail["unbalanced_lines"] = len(unbal)
            for i, o, c, no, nc, head in unbal[:5]:
                bad.append("第 %d 行 **括号不配对**（%s=%d / %s=%d）⇒ 行被截断：%s"
                           % (i, o, no, c, nc, head))
        else:
            notes.append("文本结构：%d 行，括号全平衡，以换行结尾 ✓" % len(lines))
    return bad, notes, detail


def write_provenance(path, rel, run_id, label, extra=""):
    """给"原生不带头"的产物写一份由**实际字节**算出的旁挂头，并返回它的三要素核验结果。"""
    prov = path + ".provenance"
    n = os.path.getsize(path)
    with open(prov, "w") as f:
        f.write("# produced_by: done_gate.py（旁挂头：本文件不是产物本身，是它的出处记录）\n")
        f.write("# produced_at: %s\n" % time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
        f.write("# via: product-selfcheck-sidecar\n")
        f.write("# run_id: %s\n" % run_id)
        f.write("# label: %s\n" % label)
        f.write("# subject: %s\n" % rel)
        f.write("# subject_bytes: %d\n" % n)
        f.write("# subject_lines: %s\n" % _lines(path))
        f.write("# subject_sha256_16: %s\n" % _sha16(path))
        f.write("# subject_mtime_utc: %s\n"
                % time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(os.path.getmtime(path))))
        if extra:
            f.write("# note: %s\n" % extra)
    return prov


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--require", action="append", default=[],
                    help="必须存在的产物（相对 --run-dir 的路径），可多次")
    ap.add_argument("--optional", action="append", default=[],
                    help="只登记、不作为判据的产物")
    ap.add_argument("--start-epoch", type=float, default=0.0,
                    help="驱动开始的墙钟 epoch；产物 mtime 早于它 ⇒ 判为陈货")
    ap.add_argument("--min-bytes", type=int, default=1)
    ap.add_argument("--label", default="")
    ap.add_argument("--run-id", default="")
    ap.add_argument("--expect-lines", action="append", default=[],
                    help="精确行数判据，格式 <相对路径>=<行数>，可多次")
    ap.add_argument("--no-write", action="store_true", help="只自检、不写标记（演练用）")
    a = ap.parse_args()

    run_id = a.run_id or os.path.basename(os.path.normpath(a.run_dir))
    expect_lines = {}
    for item in a.expect_lines:
        k, _, v = item.rpartition("=")
        if not k or not v.strip().isdigit():
            print("** --expect-lines 格式错：%r（应为 <相对路径>=<行数>）**" % item)
            return 2
        expect_lines[k] = int(v.strip())
    rows, fails = [], []

    def check(rel, required):
        p = os.path.join(a.run_dir, rel)
        row = {"path": rel, "required": required}
        if not os.path.exists(p):
            row.update(exists="否", nonempty="—", header="—", verdict="不存在")
            if required:
                fails.append("%s：**不存在**" % rel)
            rows.append(row)
            return
        n = os.path.getsize(p)
        mt = os.path.getmtime(p)
        row["bytes"] = n
        row["lines"] = _lines(p)
        row["sha256_16"] = _sha16(p)
        row["mtime_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(mt))
        ok_e = True
        ok_n = n > a.min_bytes
        ok_f = (mt >= a.start_epoch) if a.start_epoch else True
        ok_h, missing = _native_header(p)
        if not ok_h:
            # 原生不带头 ⇒ 写旁挂头再核（旁挂头的三者由本脚本生成，必然齐；
            # 关键是它绑定了 subject 的 sha256 ⇒ 产物一变，头就对不上）
            write_provenance(p, rel, run_id, a.label)
            ok_h = True
            row["header_kind"] = "旁挂（%s.provenance，含 subject_sha256_16）" % rel
        else:
            row["header_kind"] = "原生"
        bad_s, notes_s, det_s = _structure(p, rel)
        row["structure"] = "过" if not bad_s else "**不过**"
        row["structure_notes"] = notes_s
        row.update(det_s)
        # 可选的精确行数判据
        exp = expect_lines.get(rel)
        ok_l = True
        if exp is not None:
            ok_l = (row.get("lines") == exp)
            if not ok_l:
                bad_s = list(bad_s) + ["**行数不符**：实际 %s 行，判据要求 %d 行"
                                       % (row.get("lines"), exp)]
                row["structure"] = "**不过**"
        row.update(exists="是", nonempty="是" if ok_n else "**否**",
                   fresh="是" if ok_f else "**否（mtime 早于驱动起点）**",
                   header="有三要素" if ok_h else "**缺 %s**" % missing)
        bad = []
        if not ok_n:
            bad.append("空文件（%d 字节）" % n)
        if not ok_f:
            bad.append("陈货（mtime %s < 驱动起点）" % row["mtime_utc"])
        if not ok_h:
            bad.append("头缺 %s" % missing)
        bad.extend(bad_s)
        row["verdict"] = "过" if not bad else "**不过**：" + "；".join(bad)
        if bad and required:
            fails.append("%s：%s" % (rel, "；".join(bad)))
        rows.append(row)

    for rel in a.require:
        check(rel, True)
    for rel in a.optional:
        check(rel, False)

    now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    verdict = "PASS" if not fails else "FAIL"
    lines = []
    lines.append("# produced_by: done_gate.py")
    lines.append("# produced_at: %s" % now)
    lines.append("# via: product-selfcheck")
    lines.append("# run_id: %s" % run_id)
    # ★ 绑定"这份清单描述的是哪一版输入"（reviewer 2026-09-22 提，成本近零、用途明确）：
    #   没有这两行，"这份证据说的是哪一版"就只能靠时间先后猜 ✗（我们今天已因此错过一次）。
    #   值直接取该轮 run_state.txt 里已记的实测值——**不现算当前文件**（现算是"现在这一版"，不是"那轮的版本" ✗）。
    _st = os.path.join(a.run_dir, "run_state.txt")
    _vals = {}
    try:
        with open(_st, encoding="utf-8", errors="ignore") as _fh:
            for _ln in _fh:
                _m = re.match(r"^([A-Za-z0-9_.\-]+)=([0-9a-f]+)", _ln.strip())
                if _m:
                    _vals[_m.group(1)] = _m.group(2)
    except OSError:
        pass
    # ★★ 2026-10-10（reviewer ①′）：原来**只**读阶段三那个键 `config_rtk_courtyard.sdf` ✗
    #   阶段八的 world 是 `campus_100.sdf` ⇒ 该键查不到；而**阶段三那个键往往还在**
    #   （`lib.sh:509` 有意保留旧锚点 ✓ —— 那里保留是**对的**）⇒ 结果是：
    #   这份清单会把"描述的世界"标成**另一个世界**的哈希 ✗✗ ——
    #   ★ 比"(run_state 未记录)"**更坏**：看上去**有值** ✓、值还**合法** ✓ ⇒ 没人会去查 ✗。
    #   修法：**先找阶段八的 `config_used_*.sdf|.world`**（本次**实际生效**的那份 ✓），再退回旧键。
    #   （`config_used_make_campus_world.py` 以 `.py` 结尾 ⇒ 被下面过滤排除 ✓，不会误当成世界 ✓）
    _world_key = next((k for k in _vals
                       if k.startswith("config_used_") and k.endswith((".sdf", ".world"))), None)
    if _world_key is None:
        _world_key = "config_rtk_courtyard.sdf"
    lines.append("# describes_world: %s"
                 % _vals.get(_world_key, "(run_state 未记录)"))
    lines.append("# describes_world_key: %s" % _world_key)
    lines.append("# describes_config: %s"
                 % _vals.get("config_fingerprint", "(run_state 未记录)"))
    if a.label:
        lines.append("# label: %s" % a.label)
    lines.append("# 判据（**四件**，缺一不可）："
                 "① 存在 ② 非空 ③ 带生产者头（生产者+时刻+途径）"
                 "④ **结构完整性**（行数 / 末行完整 / 字段数 / 能解析）")
    lines.append("#   ★ 第 ④ 件是 2026-09-21 补的：前三件**查不出「被写坏」** ——"
                 "`nav11-r1/gates.txt` 就是这样通过了前三件，而内容是半截行。"
                 "（「能读」≠「读全」；根子在「同一份证据两个写入者」，"
                 "机制层修法见驱动脚本注释。）")
    lines.append("# 另加：产物 mtime 必须 ≥ 驱动起点 %s（挡同名陈货）"
                 % (time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(a.start_epoch))
                    if a.start_epoch else "（未给，跳过）"))
    lines.append("# ── 产物清单（相对 %s）──" % a.run_dir)
    for r in rows:
        if r.get("exists") == "否":
            lines.append("#   [%s] %s：不存在" % ("必须" if r["required"] else "登记", r["path"]))
            continue
        lines.append("#   [%s] %s：%d 字节，%s 行，sha256_16=%s，%s，头=%s，结构=%s ⇒ %s"
                     % ("必须" if r["required"] else "登记", r["path"], r["bytes"],
                        r["lines"], r["sha256_16"], r.get("fresh", "?"),
                        r.get("header_kind", r["header"]), r.get("structure", "?"),
                        r["verdict"]))
        for nt in r.get("structure_notes", []):
            lines.append("#        · %s" % nt)
    lines.append("# 结论：%s" % verdict)
    if fails:
        lines.append("# **不过的项**：")
        for f in fails:
            lines.append("#   - %s" % f)

    text = "\n".join(lines) + "\n"
    man = os.path.join(a.run_dir, "MEASUREMENTS_MANIFEST.txt")
    with open(man, "w") as f:
        f.write(text)

    done = os.path.join(a.run_dir, "MEASUREMENTS_DONE")
    bad_mark = os.path.join(a.run_dir, "MEASUREMENTS_INCOMPLETE")
    if verdict == "PASS":
        if not a.no_write:
            with open(done, "w") as f:
                f.write(text)
            if os.path.exists(bad_mark):
                os.remove(bad_mark)
        print(text)
        print("⇒ **四件全过 ⇒ 已写 MEASUREMENTS_DONE（标记自带清单，不再是 0 字节）**：%s" % done)
        print("   清单同时落盘：%s" % man)
        print("   自检机器可读副本：%s" % json.dumps(
            {"verdict": verdict, "run_id": run_id, "products": rows},
            ensure_ascii=False))
        return 0

    if not a.no_write:
        # ★ 不过时**不写** MEASUREMENTS_DONE，但写一个 INCOMPLETE：
        #   栈还是要停（否则容器被占住），但**必须能看出这一轮不作数**。
        with open(bad_mark, "w") as f:
            f.write(text)
        if os.path.exists(done):
            os.remove(done)
    print(text)
    print("⇒ **产物自检四件有不过的 ⇒ 不写 MEASUREMENTS_DONE，写了 MEASUREMENTS_INCOMPLETE**")
    print("   **本轮不作数**：报告里不许出现「这一档已通过」这种写法。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
