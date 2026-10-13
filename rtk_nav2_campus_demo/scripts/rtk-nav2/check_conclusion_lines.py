#!/usr/bin/env python3
"""结论行检查器：Python 文件里每个 print(...) 是不是【在条件分支里】（= 由数据算出）？

★ 为什么要有它：有人把**结论句写死在 echo/print 里** ✗ ⇒ 数据说 A、输出说 B、脚本一句错都不报 ✓。
★ 设计口径（★ 都是被实测打出来的 ✓）：
  ① ★ **优选结构，不选文本** ✗ —— 这是 Python 文件 ⇒ ★ 直接走 `ast` 的**祖先链** ✓（★ 答的才是"目标量"✓）
     ✗ 不用"看上一行是不是 `if`" ✗ —— 实测那个启发式**误报 7/11、9/18**（★ 过半 ✓），★ 因为它只看紧邻一行 ✓。
  ② ★ **只断言可靠的一侧** ✓：
     · 祖先里有 `If` ⇒ ★ 报「受保护」✓（★ 可靠 ✓）
     · 祖先里没有 `If` ⇒ ★ **只报事实**（★ 最内层是模块级／for／while／try 里的哪一种 ✓）
       ✗ **不写「可能无条件」** ✗ —— 那句话在实测里**过半是假警报** ✓，
       而 ★ 恒误报的检查器会**逼人去给 print 套一个无用的 if** ✗（★ 与"恒不通过的判据否掉每条正确对象"同族 ✓）。
★ 用法：check_conclusion_lines.py <文件.py> [更多文件…]  ／  ★ 退出码：0 = 跑完（不代表"都没问题"✗）
"""
import ast
import sys


def enclosing_chain(tree, target):
    """返回 target 节点的【语句祖先链】（从外到内），只含 stmt 与 Module/FunctionDef。"""
    parents = {}
    for node in ast.walk(tree):
        for child in ast.iter_child_nodes(node):
            parents[child] = node
    chain = []
    cur = parents.get(target)
    while cur is not None:
        if isinstance(cur, (ast.stmt, ast.Module)):
            chain.append(cur)
        cur = parents.get(cur)
    return list(reversed(chain))


def kind(node):
    return type(node).__name__


def check(path):
    try:
        src = open(path, encoding="utf-8").read()
        tree = ast.parse(src)
    except Exception as e:
        # ★ 解析不了 ⇒ 未判（★ 不许落成"没有"✗）
        print("  UNDETERMINED %s: %s: %s" % (path, type(e).__name__, e))
        return
    prints = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
              and isinstance(n.func, ast.Name) and n.func.id == "print"]
    n_prot, n_fact = 0, 0
    print("=== %s （ast 找到 print 调用 %d 个）===" % (path, len(prints)))
    for p in sorted(prints, key=lambda n: n.lineno):
        chain = enclosing_chain(tree, p)
        ifs = [c for c in chain if isinstance(c, ast.If)]
        # ★ 最近的一个"带条件的祖先"（不含 FunctionDef/Module）
        inner = None
        for c in reversed(chain):
            if isinstance(c, (ast.If, ast.For, ast.While, ast.Try, ast.With)):
                inner = c
                break
        if ifs:
            n_prot += 1
            try:
                cond_src = ast.unparse(ifs[-1].test)
            except Exception:
                cond_src = "<unparse 失败>"
            # ★★ "受保护"只答"它在一个 if 里"✗ —— ★ 不答"它是算出来的"✗（★ 实测被指出 ✓）
            #   ★ 例：`if os.path.exists(p): print("…检查通过 ✓…")` ⇒ ★ 门是"文件在不在"✓，
            #     而结论**仍是写死的** ✗ ⇒ ★ 与无条件**没有区别** ✓
            #   ⇒ ★ 断言的那一侧也要**带凭据** ✓（★ 同"报哈希必带算法"✓、"报计数必带命令"✓）
            const = isinstance(ifs[-1].test, ast.Constant)
            flag = ("  ⇒ ★ 常量条件 ⇒ **与无条件等价** ✗" if const
                    else "  ⇒ ★ 注意：这只说明「它在 if 里」✗，★ 不说明「结论是算出来的」✗")
            print("  L%-4d ★ 受保护 ✓ 最近条件 = L%d `%s`（祖先链里 If 共 %d 层）"
                  % (p.lineno, ifs[-1].lineno, kind(ifs[-1]), len(ifs)))
            print("         条件源码: `%s`%s" % (cond_src, flag))
        else:
            n_fact += 1
            where = ("模块级" if inner is None else
                     "在 L%d 的 `%s` 里（★ 不是 If ⇒ 本检查器不断言它是否由数据算出 ✗）"
                     % (inner.lineno, kind(inner)))
            # ★ 只报事实 ✓ —— ✗ 不写"可能无条件" ✗
            print("  L%-4d 事实：%s" % (p.lineno, where))
    print("  ⇒ 受保护 %d ／ 仅报事实 %d ／ 共 %d" % (n_prot, n_fact, len(prints)))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    for f in sys.argv[1:]:
        check(f)
