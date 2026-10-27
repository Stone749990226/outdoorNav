"""Stand the chassis into RL nav mode, then activate the outdoor service."""
import json, os, sys, time, urllib.request
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import lynx_quadruped.main as q

rclpy.init()
node = Node('fieldtest_activate')
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
q.nav_cmd_pub = node.create_publisher(q.NavCmd, '/NAV_CMD', qos)
q.motion_state_pub = node.create_publisher(q.MotionState, '/MOTION_STATE', qos)
q.gait_pub = node.create_publisher(q.Gait, '/GAIT', qos)
node.create_subscription(q.MotionInfo, '/MOTION_INFO', q._on_motion_info, qos)
import threading
threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
deadline = time.monotonic() + 6
while q._motion_state is None and time.monotonic() < deadline:
    time.sleep(0.1)
report = {'before': q._motion_state}
if q._motion_state != 17 or q._gait != q._nav_gait:
    result = q.set_posture(q.SetPosture_Request(posture_name='stand'))
    report['stand'] = {'success': result.success, 'message': result.message}
if q._motion_state == 17:
    mode, message = q._read_usage_mode()
    report['usage_mode'] = mode
    report['usage_message'] = message
body = json.dumps({'active': True}).encode()
request = urllib.request.Request('http://127.0.0.1:8765/api/activation', data=body,
                                 headers={'Content-Type': 'application/json'})
with urllib.request.urlopen(request, timeout=5) as response:
    report['activation'] = json.load(response)
report['state'] = q._motion_state
report['gait'] = q._gait
print(json.dumps(report, ensure_ascii=False))
node.destroy_node()
rclpy.shutdown()
