import json, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
rclpy.init(); node = Node('cmd_probe')
seen = {}
for topic in ('/navigation/controller_command', '/mock_base/cmd_vel'):
    def make(topic):
        def cb(msg):
            row = seen.setdefault(topic, {'n': 0, 'max_v': 0.0, 'max_w': 0.0, 'last': None})
            row['n'] += 1
            row['max_v'] = max(row['max_v'], abs(msg.linear.x))
            row['max_w'] = max(row['max_w'], abs(msg.angular.z))
            row['last'] = time.time()
            if abs(msg.linear.x) + abs(msg.angular.z) > 1e-6:
                row['nonzero'] = row.get('nonzero', 0) + 1
        return cb
    node.create_subscription(Twist, topic, make(topic), 10)
end = time.time() + float(__import__('sys').argv[1] if len(__import__('sys').argv) > 1 else 20)
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
print(json.dumps(seen))
node.destroy_node(); rclpy.shutdown()
