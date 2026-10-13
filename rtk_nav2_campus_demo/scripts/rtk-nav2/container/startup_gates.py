#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""起栈后自检三关（0/1/2/3）——**跑实验之前**先过，任一不过就当场作废。

为什么要这个脚本（实测代价）：
  2026-09-21 的清空实验（run-id `nav10-clr`）跑了 3 分钟、产物里四列全 NA。
  事后查清：**Gazebo 世界没起来** ⇒ `map` / `odom` 两个帧都不存在 ⇒
  导航栈被 TF 闸门拦住 ⇒ 根本没有 costmap。
  ⇒ 与其"跑完再发现没数据"，不如**起栈后 30 秒内把上游逐环点亮**。

三关的判据一律是"**活节点真收到一帧**"，**不是** `ros2 topic list` / `service list`
里有没有（那两个走 daemon，可能是 11 小时前的图 —— §9.6："缓存里有" ≠ "现在有"）：

  第 0 关｜daemon 不陈旧：扫描 /proc 找 `ros2-daemon`，记它的**存活时长**。
        陈旧（> 阈值）说明后面所有"清单类"读数都不可信 —— 先重启 daemon 再往下。
  第 1 关｜时钟在走：活节点订 `/clock`，**连续两帧 stamp 递增**且**墙钟间隔 ≥ 1 s**。
        时钟不动 = Gazebo 没在跑（或世界没 load），下游全是死的。
  第 2 关｜TF 链可解析：`map → base_footprint` **真查到一帧**（活节点 tf2 Buffer）。
        查不到就按"上游哪一环"分类报出来（仿真上游 / 定位链），别当成 nav2 的问题。
  第 3 关｜四张图都有数据：`/global_costmap/costmap_raw`、`/global_costmap/costmap`、
        `/local_costmap/costmap_raw`、`/local_costmap/costmap` **各真收到一帧**，
        并打印每帧的 stamp 与尺寸（宽×高）—— 尺寸能顺带暴露"图起了但是空的"。

用法（容器内）：
  startup_gates.py --run-dir /work/artifacts/rtk-nav2/<run-id> [--daemon-max-age 900]
输出：stdout 逐关 PASS/FAIL，同时把同样的文本写进 <run-dir>/gates.txt；
      退出码 = **第一个不过的关号 + 1**（0 表示全过），方便 shell 侧当场判。

★ 2026-09-21 改（**关 0 与"全过"撞车**，已实测踩到）：
  原来 `sys.exit(first_fail)`，而 `first_fail` 的取值就是**关号** ——
  **关 0 不过时 `first_fail = 0`，于是"关 0 失败"与"全过"退出码相同（都是 0）** ✗
  驱动写的判据是 `[ "${GATE_RC}" -ne 0 ]`，所以**关 0 这一关等于没接上**：
  nav10-r4 那轮第 0 关实测 FAIL（daemon 存活 3972 s > 900 s 阈值），
  驱动照样往下跑完了全部测定（`docs/rtk-nav2.md` §9.8 已记，当时"未改脚本"）。
  ⇒ 改成 **`退出码 = 关号 + 1`**，于是：**1=关 0 / 2=关 1 / 3=关 2 / 4=关 3**，
    **任一关不过都非 0**，且**关号可以从退出码反推**（`gates.json` 里另存了明确的关号）。
    改这个映射会让旧的"退出码就是关号"读法失效 ⇒ `r4_driver.sh` 的提示语已同步改。

★ 同一次改动的第二条（**关 0 失败 ⇒ 走 daemon 的 CLI 读数不许单独引用**）：
  关 0 不过意味着 `topic list` / `service list` / `param dump` 这类**走 daemon 的读数**
  可能回的是旧图 ⇒ 本脚本会**写下标记文件** `<run-dir>/CLI_READS_UNTRUSTED`，
  并在 `gates.json` 里置 `daemon_reads_usable: false`。
  需要"运行时参数真的生效"这类证据时，**改用 `getparams.py` 的直连活值**
  （它直接调 `<node>/get_parameters` 服务，不经 daemon）。
  关 0 通过时会**删掉**这个标记（避免上一轮的标记被误当成这一轮的结论）。
"""

import argparse
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

RESULTS = []          # [(gate_no, name, ok, lines), ...]


def _log(msg):
    print(msg, flush=True)


def _record(gate_no, name, ok, lines, dur_s=None):
    """`dur_s` = 这一关**自己的墙钟用时**（秒）。**这是"今天行不行"要的那个数**：
    起栈之后每一关各花多久、整条链多久能出结论，都靠它分段给出来。"""
    RESULTS.append((gate_no, name, ok, lines, dur_s))
    _log("")
    _log("==== 第 %d 关｜%s：%s（用时 %s）===="
         % (gate_no, name, "PASS" if ok else "FAIL",
            "—" if dur_s is None else "%.1f s" % dur_s))
    for ln in lines:
        _log("  " + ln)


# ---------- 第 0 关：daemon 存活时长 ----------

def _proc_uptime_s(pid):
    """从 /proc/<pid>/stat 的 starttime（第 22 字段，单位 clock tick）+ /proc/uptime 算存活秒数。"""
    try:
        with open("/proc/%d/stat" % pid, "r") as f:
            raw = f.read()
        # comm 字段可能含空格/括号，取最后一个 ')' 之后再切
        rest = raw[raw.rfind(")") + 2:].split()
        starttime_ticks = float(rest[19])          # 第 22 字段 => rest[19]
        ticks = os.sysconf("SC_CLK_TCK")
        with open("/proc/uptime", "r") as f:
            uptime = float(f.read().split()[0])
        return uptime - starttime_ticks / ticks
    except Exception:
        return None


def find_daemons():
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        try:
            with open("/proc/%d/cmdline" % pid, "rb") as f:
                cmd = f.read().replace(b"\x00", b" ").decode("utf-8", "replace").strip()
        except Exception:
            continue
        if "ros2cli.daemon" in cmd:
            out.append((pid, cmd, _proc_uptime_s(pid)))
    return out


def gate0(args, ev):
    _t_start = time.monotonic()
    max_age = args.daemon_max_age
    daemons = find_daemons()
    lines = []
    if not daemons:
        lines.append("容器内没有 ros2-daemon 在跑（每次 ros2 CLI 调用会临时起一个，属正常）")
        lines.append("⇒ 关 0 视为 PASS（无陈旧缓存可读）")
        _record(0, "daemon 不陈旧", True, lines, time.monotonic() - _t_start)
        ev["daemon"] = []
        return True
    worst = 0.0
    for pid, cmd, age in daemons:
        age_s = "未知" if age is None else "%.0f s（%.2f h）" % (age, age / 3600.0)
        lines.append("pid=%d 存活=%s" % (pid, age_s))
        if age is not None and age > worst:
            worst = age
    ok = worst <= max_age
    if ok:
        lines.append("⇒ 最老的 daemon 存活 %.0f s ≤ 阈值 %d s，清单类读数可接受" % (worst, max_age))
        lines.append("   注：本脚本的三关判据本来就不走 daemon（活节点收帧），此关只记录时长。")
    else:
        lines.append("⇒ **最老的 daemon 存活 %.0f s > 阈值 %d s**：此后 `topic list`/`service list`/"
                     "`param get` 都可能回旧图。要么先重启 daemon，要么只用活节点判据。" % (worst, max_age))
    _record(0, "daemon 不陈旧", ok, lines, time.monotonic() - _t_start)
    ev["daemon"] = [{"pid": p, "age_s": a, "cmd": c} for p, c, a in daemons]
    return ok


# ---------- 第 1 关：/clock 真的在走 ----------

def gate1(args, ev):
    _t_start = time.monotonic()
    node = Node("gate_clock")
    got = []
    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT     # gazebo 的 /clock 常见 BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE
    qos.history = HistoryPolicy.KEEP_LAST

    from rosgraph_msgs.msg import Clock

    def cb(msg):
        got.append((time.monotonic(), msg.clock.sec + msg.clock.nanosec * 1e-9))

    node.create_subscription(Clock, "/clock", cb, qos)
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.clock_timeout and len(got) < 2:
        rclpy.spin_once(node, timeout_sec=0.2)
    # 收到 2 帧后再多收几帧，好看出"墙钟 1 s 内 sim 走了多少"
    t1 = time.monotonic()
    while time.monotonic() - t1 < args.clock_settle:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()

    lines = []
    if len(got) < 2:
        lines.append("** %d s 内只收到 %d 帧 /clock ** ⇒ Gazebo 没在跑，或世界没 load" % (args.clock_timeout, len(got)))
        lines.append("⇒ 处置：停栈，先看 demo_launch.log 的 ign gazebo 段与 create-3 的 \"Requesting list of world names.\"")
        _record(1, "/clock 在走", False, lines, time.monotonic() - _t_start)
        ev["clock"] = {"count": len(got)}
        return False
    wall_span = got[-1][0] - got[0][0]
    sim_span = got[-1][1] - got[0][1]
    increasing = all(got[i + 1][1] > got[i][1] for i in range(len(got) - 1))
    ok = increasing and wall_span >= 1.0
    lines.append("收到 %d 帧；首帧 sim=%.3f 末帧 sim=%.3f" % (len(got), got[0][1], got[-1][1]))
    lines.append("墙钟跨度 %.2f s；sim 跨度 %.3f s；比值 sim/墙钟=%.4f" %
                 (wall_span, sim_span, (sim_span / wall_span) if wall_span > 0 else 0.0))
    lines.append("逐帧单调递增：%s" % ("是" if increasing else "**否**"))
    if not ok:
        lines.append("⇒ 判据不满足（需：递增 且 墙钟跨度 ≥ 1 s）")
    _record(1, "/clock 在走", ok, lines, time.monotonic() - _t_start)
    ev["clock"] = {"count": len(got), "wall_span_s": wall_span, "sim_span_s": sim_span,
                   "increasing": increasing, "first_sim": got[0][1], "last_sim": got[-1][1]}
    return ok


# ---------- 第 2 关：map → base_footprint 真查到一帧 ----------

def gate2(args, ev):
    _t_start = time.monotonic()
    from tf2_ros import Buffer, TransformListener
    node = Node("gate_tf")
    buf = Buffer()
    TransformListener(buf, node, spin_thread=False)
    ok = False
    tf = None
    last_exc = None
    lines = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.tf_timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tf = buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            ok = True
            break
        except Exception as exc:            # noqa: BLE001 —— 就是要分类报错
            last_exc = exc
    lines.append("等待上限 %d s，实际用时 %.1f s" % (args.tf_timeout, time.monotonic() - t0))
    if ok:
        tr = tf.transform.translation
        lines.append("**真查到一帧** map→base_footprint："
                     "x=%.3f y=%.3f z=%.3f（stamp %d.%09d）" %
                     (tr.x, tr.y, tr.z, tf.header.stamp.sec, tf.header.stamp.nanosec))
        lines.append("Buffer 里的帧：%s" % (_frames(buf) or "(未取到)"))
    else:
        lines.append("**%d s 内查不到 map→base_footprint**：%s" % (args.tf_timeout, last_exc))
        frames = _frames(buf)
        lines.append("Buffer 里的帧：%s" % (", ".join(frames) if frames else "(空)"))
        if "map" not in frames:
            lines.append("分类：**定位链没就绪** —— `map` 帧不在，`absolute_localizer` 没发出 map→odom")
        else:
            lines.append("分类：`map` 在但链断在中间某环，逐环查 absolute_localizer / ekf_local")
    _record(2, "TF map→base_footprint 可解析", ok, lines, time.monotonic() - _t_start)
    ev["tf"] = {"ok": ok, "waited_s": time.monotonic() - t0}
    node.destroy_node()
    return ok


# ---------- 第 3 关：四张图各收到一帧 ----------

def _costmap_topics():
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import OccupancyGrid
    return [
        ("/global_costmap/costmap_raw", Costmap, "raw"),
        ("/global_costmap/costmap", OccupancyGrid, "inflated"),
        ("/local_costmap/costmap_raw", Costmap, "raw"),
        ("/local_costmap/costmap", OccupancyGrid, "inflated"),
    ]


def _frames(buf):
    """Buffer 里已知的帧名（tf2 Buffer 只给 YAML 文本，这里只取顶层键）。"""
    try:
        txt = buf.all_frames_as_yaml()
    except Exception:
        txt = ""
    if not txt:
        try:
            txt = buf.all_frames_as_string()
        except Exception:
            txt = ""
    out = []
    for ln in txt.split("\n"):
        s = ln.strip()
        if s.startswith("- "):
            out.append(s[2:].rstrip(":").strip())
        elif s.endswith(":") and not s.startswith("#"):
            out.append(s.rstrip(":").strip())
    return sorted(set(x for x in out if x))


def gate3(args, ev):
    # ★ 计时标记**不能叫 `_t`**：本函数下面的 `for topic, _t, kind in _costmap_topics()`
    #   已经把 `_t` 用作**消息类型**了 ⇒ 撞名 ⇒ `time.monotonic() - _t` 会变成
    #   "float - 类对象" 的 TypeError（**2026-09-21 彩排实测炸在这里**，
    #   而 `py_compile` 对此**无感** —— 又是"编译期工具管不了运行期类型错"的同一个族）。
    _t_start = time.monotonic()
    node = Node("gate_maps")
    seen = {}
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL      # nav2 图是 latched 的
    for topic, msgtype, _kind in _costmap_topics():
        def make_cb(t):
            def cb(msg):
                if t not in seen:
                    seen[t] = msg
            return cb
        node.create_subscription(msgtype, topic, make_cb(topic), qos)
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.maps_timeout and len(seen) < 4:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()

    lines = []
    ok = True
    detail = {}
    for topic, _t, kind in _costmap_topics():
        m = seen.get(topic)
        if m is None:
            ok = False
            lines.append("✗ %s（%s）：**%d s 内一帧都没收到**" % (topic, kind, args.maps_timeout))
            detail[topic] = None
            continue
        st = m.header.stamp
        # 两种消息两套字段：raw 是 nav2_msgs/Costmap（metadata.size_x/y），
        # inflated 是 nav_msgs/OccupancyGrid（info.width/height）。
        if type(m).__name__ == "Costmap":
            md = m.metadata
            w, h = int(md.size_x), int(md.size_y)
        else:
            info = m.info
            w, h = int(info.width), int(info.height)
        detail[topic] = {"w": w, "h": h, "stamp": st.sec + st.nanosec * 1e-9}
        lines.append("✓ %s（%s）：%dx%d，stamp=%d.%09d" %
                     (topic, kind, w, h, st.sec, st.nanosec))
    if ok:
        lines.append("⇒ 四张图各真收到一帧（判据：收帧，不是 ``topic list`` 里有没有）")
    else:
        lines.append("⇒ 缺图的先看第 1/2 关：时钟不走 / TF 不通时，nav2 会被闸门拦住而不起图")
    _record(3, "四张代价地图各收到一帧", ok, lines, time.monotonic() - _t_start)
    ev["maps"] = detail
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True, help="产物目录（gates.txt 写到这里）")
    ap.add_argument("--daemon-max-age", type=float, default=7200.0,
                    help="第 0 关阈值（秒）。判的是'**上一个会话留下的**陈旧 daemon'："
                         "默认 2 h。注意它**不是**'本轮起栈前必须刚重启过'——"
                         "只要不是上一个会话的残留就行（本轮所有判据都不走 daemon）。")
    ap.add_argument("--clock-timeout", type=float, default=30.0)
    ap.add_argument("--clock-settle", type=float, default=3.0)
    ap.add_argument("--tf-timeout", type=float, default=90.0)
    ap.add_argument("--maps-timeout", type=float, default=60.0)
    ap.add_argument("--skip", default="", help="要跳过的关，逗号分隔，例如 '0,1'")
    args = ap.parse_args()

    skip = {s.strip() for s in args.skip.split(",") if s.strip()}
    env = {
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", ""),
        "GZ_PARTITION": os.environ.get("GZ_PARTITION", ""),
        "wall_clock": time.strftime("%Y-%m-%d %H:%M:%S"),
        "utc": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()),
    }
    _log("起栈自检（活节点判据）：run-dir=%s" % args.run_dir)
    _log("ROS_DOMAIN_ID=%s  GZ_PARTITION=%s  容器本地时间(UTC)=%s" %
         (env["ROS_DOMAIN_ID"], env["GZ_PARTITION"], env["utc"]))

    rclpy.init()
    ev = {"env": env}
    # ★★ 2026-09-21 实测踩到（**这是"关 0 失败被漏掉"的真正根因**，比"退出码映射"更深一层）：
    #   原来这里是 `first_fail = 0`，把 **0 同时当成"没有关不过"的哨兵**，
    #   而 **关号 0 本身是合法关号** ⇒ 关 0 跑出 FAIL 时执行 `first_fail = no = 0`
    #   ⇒ `if first_fail:` 为假（后面的关照跑）、`exit_code = 0`（驱动以为全过）✗
    #   ⇒ 只改"退出码 = 关号+1"**不够**，哨兵必须换成 **None**。
    #   （自测：`--daemon-max-age 1 --skip 1,2,3` 在只改映射时仍打印"全过"、退出码 0。）
    first_fail = None          # None = 没有关不过；否则是**关号**（0 是合法值）
    try:
        plan = [(0, "daemon 不陈旧", gate0), (1, "/clock 在走", gate1),
                (2, "TF map→base_footprint 可解析", gate2), (3, "四张代价地图各收到一帧", gate3)]
        for no, name, fn in plan:
            if str(no) in skip:
                _log("\n==== 第 %d 关｜%s：SKIP ====" % (no, name))
                continue
            if first_fail is not None:
                _log("\n==== 第 %d 关｜%s：未跑（前一关没过） ====" % (no, name))
                continue
            if not fn(args, ev):
                first_fail = no
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # ---- 四关合计用时（"多少秒知道今天行不行"的那个数）----
    _durs = [(no, dur) for no, _n, _ok, _l, dur in RESULTS if dur is not None]
    gates_total = sum(d for _no, d in _durs)
    _log("")
    _log("**四关合计用时 %.1f s**（分段：%s）"
         % (gates_total, "、".join("关%d %.1f s" % (no, d) for no, d in _durs)))

    # ---- 退出码：**关号 + 1**（见模块 docstring 的说明；0 = 全过）----
    # ★ 这一段必须在**任何写盘之前**算出来：`gates.txt` 里也要记这两个值
    #   （2026-09-21 实测踩到：先写 gates.txt、后算 gate0_ok ⇒ UnboundLocalError，
    #    而它正好抛在"关 0 不过"这条路径上，把一次真失败伪装成脚本崩溃）。
    exit_code = 0 if first_fail is None else first_fail + 1
    gate0_ok = True
    for no, _name, ok, _lines, _dur in RESULTS:      # 5 元组（多了"用时"）
        if no == 0:
            gate0_ok = ok
    daemon_reads_usable = gate0_ok

    # ---- 关 0 失败 ⇒ CLI（走 daemon）读数不可单独引用：落一个机器可判的标记 ----
    flag = os.path.join(args.run_dir, "CLI_READS_UNTRUSTED")
    if daemon_reads_usable:
        if os.path.exists(flag):
            os.remove(flag)
    else:
        with open(flag, "w") as f:
            f.write("关 0（daemon 不陈旧）未过 ⇒ 走 daemon 的 CLI 读数"
                    "（ros2 topic list / service list / param dump / param get）"
                    "**一律不许单独作为证据引用**。\n"
                    "替代：用 getparams.py 直连 <node>/get_parameters 取活值；"
                    "或只引用**活节点收帧**类判据（本脚本第 1/2/3 关就是这个口径）。\n"
                    "写入时刻（UTC）：%s\n" % env["utc"])

    import json
    os.makedirs(args.run_dir, exist_ok=True)
    txt = os.path.join(args.run_dir, "gates.txt")
    with open(txt, "w") as f:
        f.write("# 起栈自检 0/1/2/3 关（活节点判据，不看 daemon 缓存）\n")
        f.write("# 时间：本地 %s / UTC %s；ROS_DOMAIN_ID=%s GZ_PARTITION=%s\n" %
                (env["wall_clock"], env["utc"], env["ROS_DOMAIN_ID"], env["GZ_PARTITION"]))
        for no, name, ok, lines, dur in RESULTS:
            f.write("\n==== 第 %d 关｜%s：%s（用时 %s）====\n"
                    % (no, name, "PASS" if ok else "FAIL",
                       "—" if dur is None else "%.1f s" % dur))
            for ln in lines:
                f.write("  " + ln + "\n")
        f.write("\n四关合计用时：%.1f s（分段：%s）\n"
                % (gates_total,
                   "、".join("关%d %.1f s" % (no, d) for no, d in _durs)))
        f.write("\n结论：%s\n"
                % ("全过（0/1/2/3）" if first_fail is None
                   else "卡在第 %d 关" % first_fail))
        f.write("退出码：%d（规则：0=全过；n>=1 ⇒ 第 (n-1) 关是第一个不过的关）\n"
                % exit_code)
        f.write("daemon_reads_usable：%s（false ⇒ 走 daemon 的 CLI 清单类读数"
                "不许单独引用；用 getparams.py 直连活值替代）\n"
                % ("true" if gate0_ok else "false"))
    _log("")
    _log("关 0 = %s ⇒ daemon_reads_usable=%s；标记 %s"
         % ("PASS" if gate0_ok else "FAIL", daemon_reads_usable,
            "已删除" if daemon_reads_usable else "已写入 " + flag))
    if not daemon_reads_usable:
        _log("** 关 0 未过：此后走 daemon 的 CLI 读数（topic/service/param 清单类）"
             "一律不许单独引用 ⇒ 用 getparams.py 的直连活值替代 **")

    with open(os.path.join(args.run_dir, "gates.json"), "w") as f:
        json.dump({"first_fail": first_fail, "first_fail_gate": first_fail,
                   "gate_durations_s": {("gate%d" % no): round(d, 2) for no, d in _durs},
                   "gates_total_s": round(gates_total, 2),
                   "first_fail_none_means_all_pass": True,
                   "exit_code": exit_code, "daemon_reads_usable": daemon_reads_usable,
                   "exit_code_rule": "0=全过；n>=1 表示第 (n-1) 关是第一个不过的关",
                   "env": env, "evidence": ev}, f, ensure_ascii=False, indent=2)
    _log("")
    _log("结论：%s（已写入 %s）"
         % ("**全过 0/1/2/3**" if first_fail is None
            else "**卡在第 %d 关**" % first_fail, txt))
    _log("退出码 = %d（规则：0=全过；n>=1 ⇒ 第 (n-1) 关是第一个不过的关）" % exit_code)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
