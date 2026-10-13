#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""**不经 daemon** 地读活节点的运行时参数：直接调 <node>/get_parameters 服务。

为什么要这个（§9.6 / §9.8）：`ros2 param dump` 走 CLI daemon，而本机 daemon 可能是
**一小时前的**（起栈自检第 0 关实测 3972 s）⇒ 它回的可能是**旧图**
⇒ **不能拿它当"这一轮运行时参数生效"的证据** ✗。这里直连服务，绕开 daemon ✓。

## ★ 参数名的正确写法：**挂在层名下面**（2026-09-21 实测踩到，会反复咬人的坑）

nav2 的 `ObstacleLayer` 把每个观测源的参数注册成 **`<层名>.<源名>.<键>`**，不是 `<源名>.<键>`：

    ✓  obstacle_layer.scan.topic        ✗  scan.topic
    ✓  obstacle_layer.scan.data_type    ✗  scan.data_type
    ✓  obstacle_layer.camera_cloud.topic
    ✓  inflation_layer.inflation_radius

**名字写错时的现象极具误导性**：`<node>/get_parameters` **回一个空列表** ——
脚本不报错、**什么都不打印** ⇒ 读的人会得到"**参数服务不应答**"或"**参数没生效**"这种
完全错误的印象（`nav11-r1/r1_getparams.txt` 就是这么来的：两张图各打印了一行表头、
下面一个参数都没有）。**其实服务好得很，是名字不对。**

## 判据（2026-09-21 改硬）

**"请求的键不存在"必须报错退出**，不许"安静地什么都不打印"：
  · 返回值的**个数**与请求的名字个数不一致 ⇒ 有名字不存在 ⇒ **退出码 1**，并列出缺哪些；
  · 某个值的 `type == 0`（`PARAMETER_NOT_SET`）⇒ 名字在、但**没有值** ⇒ 默认**退出码 1**
    （要放行用 `--allow-notset`，并把"未设置"如实打出来）。

用法：getparams.py <节点全名> <参数名...> [--allow-notset]
退出码：0 = 全部名字都有值；1 = 有名字不存在 / 没有值；2 = 用法错；其它 = 服务不可达
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


def _fmt(v):
    t = v.type
    if t == 1:
        return str(bool(v.bool_value))
    if t == 2:
        return str(v.integer_value)
    if t == 3:
        return str(v.double_value)
    if t == 4:
        return repr(v.string_value)
    if t == 7:
        return str([x for x in v.integer_array_value])
    if t == 8:
        return str([x for x in v.double_array_value])
    if t == 9:
        return str([x for x in v.string_array_value])
    return "type=%s（该类型未在本探针里解码）" % t


def main():
    argv = sys.argv[1:]
    allow_notset = "--allow-notset" in argv
    argv = [a for a in argv if a != "--allow-notset"]
    if len(argv) < 2:
        print("用法：getparams.py <节点全名> <参数名...> [--allow-notset]")
        return 2
    node_name = argv[0]
    names = argv[1:]
    rclpy.init()
    n = Node("getparams_probe")
    cli = n.create_client(GetParameters, node_name + "/get_parameters")
    if not cli.wait_for_service(timeout_sec=20.0):
        print("** 服务 %s/get_parameters 20 s 内不可达 ⇒ 该节点不在，或参数服务不应答 **" % node_name)
        n.destroy_node()
        rclpy.shutdown()
        return 1
    req = GetParameters.Request()
    req.names = names
    fut = cli.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < 25:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not fut.done():
        print("** 直连调用了 %.0f s 没返回 **" % (time.time() - t0))
        n.destroy_node()
        rclpy.shutdown()
        return 1
    res = fut.result()
    print("直连 <node>/get_parameters（**不经 daemon**）节点=%s  应答 %.2f s 后返回："
          % (node_name, time.time() - t0))

    # ★ 硬判据（2026-09-21 改）：名字不存在 / 没有值 ⇒ 报错退出，**不许安静地什么都不打印**
    vals = list(res.values)
    if len(vals) != len(names):
        missing = names[len(vals):] if len(vals) < len(names) else []
        print("  ** 返回值的个数(%d)与请求的名字个数(%d)不一致 ⇒ 有名字在这个节点上不存在 ✗ **"
              % (len(vals), len(names)))
        if missing:
            print("  ** 没拿到值的名字：%s **" % ", ".join(missing))
        print("  ★ 提醒：nav2 的源参数挂在**层名**下 —— 写 `obstacle_layer.scan.topic`，"
              "不是 `scan.topic`（写成后者就会得到这个空结果）")
        n.destroy_node()
        rclpy.shutdown()
        return 1

    notset = []
    for nm, v in zip(names, vals):
        print("  %-44s = %s" % (nm, _fmt(v)))
        if v.type == 0:
            notset.append(nm)
    if notset:
        print("  ** 下列参数 type=0（PARAMETER_NOT_SET）⇒ 名字在、但没有值：%s **"
              % ", ".join(notset))
        if not allow_notset:
            print("  ⇒ 判为**不过**（要放行请显式加 --allow-notset）")
            n.destroy_node()
            rclpy.shutdown()
            return 1
        print("  ⇒ 本次带 --allow-notset，如实记为「未设置」、不作判据")
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
