"""Read-only ROS observations for the existing navigation chain. No publishers."""

import math
import time


class ExistingObservations:
    def __init__(self, backend, config):
        import rclpy
        import rclpy.time
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener
        self.rclpy, self.rclpy_time, self.backend, self.config = rclpy, rclpy.time, backend, config
        self.node = Node('outdoornav_existing_observer')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.odom = None
        self.odom_at = self.scan_at = -math.inf
        self.odom_stamp = None
        self.odom_subscription = None
        self.odom_edge = None
        self.generation = None
        self.last_discovery = -math.inf
        self.node.create_subscription(LaserScan, config['scan_topic'], self.scan,
                                      qos_profile_sensor_data)
        self.node.create_timer(0.05, self.update)
        self.Odometry, self.qos = Odometry, qos_profile_sensor_data

    def scan(self, message):
        age = self.node.get_clock().now().nanoseconds / 1e9 - (
            message.header.stamp.sec + message.header.stamp.nanosec / 1e9)
        if -0.1 <= age < 1 and any(math.isfinite(r) and message.range_min < r < message.range_max for r in message.ranges):
            self.scan_at = time.monotonic()

    def on_odom(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        age = self.node.get_clock().now().nanoseconds / 1e9 - stamp
        if self.odom_stamp is not None and stamp <= self.odom_stamp:
            return
        if not -0.1 <= age < 1:
            return
        if message.header.frame_id != 'odom' or message.child_frame_id != 'base_link':
            return
        self.odom_stamp = stamp
        self.odom = message
        self.odom_at = time.monotonic() - max(0, age)

    def discover(self):
        from robonix_api import ATLAS
        from robonix_api.atlas_types import LifecycleState
        providers = ATLAS.query_services(id=self.config['mapping_provider_id'])
        if len(providers) != 1 or providers[0].state != LifecycleState.ACTIVE:
            self.generation = None
            return
        generation = providers[0].registration_id
        if generation != self.generation:
            self.generation = generation
            self.odom_stamp = None
        if self.odom_subscription is None:
            cap = ATLAS.find_unique_capability(contract_id='robonix/primitive/chassis/odom',
                provider_id=self.config['odometry_provider_id'], transport='ros2')
            edge = ATLAS.connect_capability(consumer_id=self.config.get('consumer_id', 'outdoor_navigation'),
                provider_id=cap.provider_id, contract_id=cap.contract_id, transport='ros2')
            self.odom_edge = edge
            self.odom_subscription = self.node.create_subscription(self.Odometry,
                edge.endpoint, self.on_odom, self.qos)

    def update(self):
        now = time.monotonic()
        if now - self.last_discovery > 1:
            self.last_discovery = now
            try:
                self.discover()
            except Exception:
                self.generation = None
        pose = None
        try:
            tf = self.buffer.lookup_transform('map', 'base_link', self.rclpy_time.Time())
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
            age = self.node.get_clock().now().nanoseconds / 1e9 - stamp
            p, q = tf.transform.translation, tf.transform.rotation
            norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
            if 0 <= age < 1 and abs(norm-1) < 0.05 and self.generation and all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)):
                pose = {'x': p.x, 'y': p.y, 'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                        'stamp_s': stamp, 'age_s': age}
        except Exception:
            pass
        speed = None
        if self.odom and now - self.odom_at < 1:
            t = self.odom.twist.twist
            speed = (math.hypot(t.linear.x, t.linear.y), t.angular.z)
        self.backend.observe(pose, speed, now-self.scan_at < 1, self.generation, now)

    def close(self):
        if self.odom_edge:
            self.odom_edge.close()
        self.node.destroy_node()
