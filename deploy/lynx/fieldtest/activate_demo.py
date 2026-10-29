"""Explicit stand/nav-mode operation, then activate HTTP; not a startup side effect."""
import json
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import lynx_quadruped.main as q


def main():
    rclpy.init()
    node = Node('fieldtest_activate')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    q.nav_cmd_pub = node.create_publisher(q.NavCmd, '/NAV_CMD', qos)
    q.motion_state_pub = node.create_publisher(q.MotionState, '/MOTION_STATE', qos)
    q.gait_pub = node.create_publisher(q.Gait, '/GAIT', qos)
    subscription = node.create_subscription(q.MotionInfo, '/MOTION_INFO', q._on_motion_info, qos)
    thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    thread.start()
    report = {}
    try:
        deadline = time.monotonic()+6
        while q._motion_state is None and time.monotonic() < deadline:
            time.sleep(.1)
        if q._motion_state is None or time.monotonic()-q._motion_stamp >= 1:
            raise RuntimeError('No fresh chassis status; no stand command sent')
        report['before'] = q._motion_state
        if q._motion_state != 17 or q._gait not in (0x3002, 0x3003):
            result = q.set_posture(q.SetPosture_Request(posture_name='stand'))
            report['stand'] = {'success': result.success, 'message': result.message}
            if not result.success:
                raise RuntimeError('Standing/navigation gait not confirmed')
        mode, _ = q._read_usage_mode()
        if mode != 1:
            success, message = q._switch_usage_mode(1)
            if not success:
                raise RuntimeError(message)
            mode, _ = q._read_usage_mode()
        if mode != 1 or q._motion_state != 17 or q._gait not in (0x3002, 0x3003):
            raise RuntimeError('Navigation mode did not pass readback')
        request = urllib.request.Request('http://127.0.0.1:8765/api/activation',
            data=b'{"active":true}', headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=5) as response:
            report['activation'] = json.load(response)
        report.update(state=q._motion_state, gait=q._gait, usage_mode=mode)
        print(json.dumps(report, ensure_ascii=False))
    finally:
        q.nav_cmd_pub.publish(q.NavCmd())
        rclpy.shutdown()
        thread.join(timeout=2)
        node.destroy_node()


if __name__ == '__main__':
    main()
