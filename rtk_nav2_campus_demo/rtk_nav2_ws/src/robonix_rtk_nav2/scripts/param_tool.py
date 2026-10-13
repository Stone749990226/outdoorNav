#!/usr/bin/env python3
"""直接读写远端节点的参数，**绕开 `ros2 param` 的 CLI daemon**。

为什么要自己写一个：`ros2 param set/get` 走的是 ROS CLI daemon，这台机器上它经常
状态陈旧——节点明明活着（`pgrep` 看得到），`ros2 param set` 却报 "Node not found"。
后果不是"报个错"这么轻：

  - 里程计劣化实验里设置劣化参数失败 → 劣化根本没生效 → "基准 vs 劣化"两次跑的是同一个配置，
    "行为不变"什么都证明不了（**实测踩过**）；
  - 冒烟测试里把 CLI 的失败误判成"节点崩了" → 假失败。

本工具直接调节点的 `set_parameters` / `get_parameters` 服务，并用服务返回值确认结果。

用法：
  param_tool.py set <节点名> <参数名> <值>     # 值按 bool/int/float/str 自动判型
  param_tool.py get <节点名> <参数名>
  param_tool.py dump <节点名>                 # 全部参数（按 YAML 形式打印）
成功退出码 0，失败非零（调用方据此判定，不靠人看输出）。

`dump` 是第三阶段加的：`ros2 param dump <节点>` 在这台机器上同样受 CLI daemon 影响，
实测对 `/local_costmap/local_costmap` 返回**空**（文件 0 字节）。而"两张图的实际参数"
是 brief §4 要求的证据（判据里要能看出"文件改了但运行时没生效"），不能靠一个
会静默返回空的命令。dump 走 `list_parameters` + `get_parameters` 两个服务，直接拿。
"""

import sys
import time

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node


def infer_value(text):
    low = text.strip().lower()
    if low in ('true', 'false'):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL,
                              bool_value=(low == 'true'))
    try:
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                              integer_value=int(text))
    except ValueError:
        pass
    try:
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                              double_value=float(text))
    except ValueError:
        pass
    return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=text)


def main(argv=None):
    if len(argv if argv is not None else sys.argv) < 3:
        print(__doc__)
        return 2
    args = argv if argv is not None else sys.argv
    action, node_name = args[1], args[2]
    param_name = args[3] if len(args) > 3 else ''

    rclpy.init()
    node = Node('param_tool')
    try:
        if action == 'set':
            value = infer_value(args[4])
            client = node.create_client(SetParameters, f'{node_name}/set_parameters')
            if not client.wait_for_service(timeout_sec=5.0):
                print('服务不可用：%s/set_parameters' % node_name)
                return 1
            req = SetParameters.Request()
            req.parameters = [ParameterMsg(name=param_name, value=value)]
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if future.result() is None:
                print('设置超时'); return 1
            ok = future.result().results[0].successful
            print('set %s.%s = %s -> %s' % (node_name, param_name,
                                            args[4], 'ok' if ok else '失败'))
            return 0 if ok else 1

        if action == 'get':
            client = node.create_client(GetParameters, f'{node_name}/get_parameters')
            if not client.wait_for_service(timeout_sec=5.0):
                print('服务不可用：%s/get_parameters' % node_name)
                return 1
            req = GetParameters.Request()
            req.names = [param_name]
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if future.result() is None:
                print('读取超时'); return 1
            v = future.result().values[0]
            if v.type == ParameterType.PARAMETER_BOOL:
                print(v.bool_value)
            elif v.type == ParameterType.PARAMETER_INTEGER:
                print(v.integer_value)
            elif v.type == ParameterType.PARAMETER_DOUBLE:
                print(v.double_value)
            else:
                print(v.string_value)
            return 0

        if action == 'dump':
            from rcl_interfaces.srv import ListParameters
            client = node.create_client(ListParameters, f'{node_name}/list_parameters')
            if not client.wait_for_service(timeout_sec=5.0):
                print('服务不可用：%s/list_parameters' % node_name)
                return 1
            req = ListParameters.Request()
            req.depth = 0            # 0 = 递归列出全部（含嵌套前缀）
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            if future.result() is None:
                print('列参数超时')
                return 1
            names = sorted(future.result().result.names)
            if not names:
                print('节点 %s 没有参数（或节点不在）' % node_name)
                return 1
            gclient = node.create_client(GetParameters, f'{node_name}/get_parameters')
            if not gclient.wait_for_service(timeout_sec=5.0):
                print('服务不可用：%s/get_parameters' % node_name)
                return 1
            # 分批取，避免一次请求过大
            values = []
            for i in range(0, len(names), 200):
                greq = GetParameters.Request()
                greq.names = names[i:i + 200]
                gfut = gclient.call_async(greq)
                rclpy.spin_until_future_complete(node, gfut, timeout_sec=10.0)
                if gfut.result() is None:
                    print('读参数超时')
                    return 1
                values.extend(gfut.result().values)
            print('%s:' % node_name)
            print('  ros__parameters:')
            for name, v in zip(names, values):
                if v.type == ParameterType.PARAMETER_BOOL:
                    text = 'true' if v.bool_value else 'false'
                elif v.type == ParameterType.PARAMETER_INTEGER:
                    text = str(v.integer_value)
                elif v.type == ParameterType.PARAMETER_DOUBLE:
                    text = repr(v.double_value)
                elif v.type == ParameterType.PARAMETER_STRING_ARRAY:
                    text = '[' + ', '.join(repr(x) for x in v.string_array_value) + ']'
                elif v.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    text = '[' + ', '.join(repr(x) for x in v.double_array_value) + ']'
                elif v.type == ParameterType.PARAMETER_INTEGER_ARRAY:
                    text = '[' + ', '.join(str(x) for x in v.integer_array_value) + ']'
                elif v.type == ParameterType.PARAMETER_BYTE_ARRAY:
                    text = '<byte array %d>' % len(v.byte_array_value)
                else:
                    text = repr(v.string_value)
                print('    %s: %s' % (name, text))
            print('  # 共 %d 个参数' % len(names))
            return 0

        print('未知动作 %s' % action)
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
