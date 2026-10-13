#!/usr/bin/env python3
"""**模拟 RTK** 节点。

它做的事只有一件：把 Gazebo 那个**理想**的 GPS 定位，加工成"看起来像 RTK"的定位——
加高斯噪声与一个固定偏差，然后转发出去。

为什么单独做一个节点、而不是把噪声写进 Gazebo 传感器：

1. **"这里在模拟"这件事必须在代码和话题名里一眼可见**。Gazebo 的 GPS 传感器是理想值，
   噪声写在 SDF 里没人看得出来；本节点的名字是 `rtk_sim`，输出话题是 `/rtk/sim_fix`，
   `ros2 node list` / `ros2 topic list` 一眼就能看出这一环是模拟的。
2. **噪声量级、固定偏差要能被审计**：验收方要拿话题实测噪声去对照文档声明值，
   参数化之后就有一个明确的比对对象。

**再次强调：这是模拟 RTK，不代表真实 RTK 的精度、也不模拟失锁/浮点解/多路径。**
真实 RTK 在遮挡、多路径、基站距离变化下的表现与本模拟无关。

订阅：/rtk/navsat      （Gazebo GPS，理想值，无噪声）
发布：/rtk/sim_fix     （NavSatFix，加了噪声与固定偏差）
      /rtk/sim_status  （String，latched；写明这是模拟源，便于审计时一眼确认）
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

# 1 度纬度 ≈ 111320 m；经度按 cos(纬度) 收缩（本场景尺度小，等距投影足够）
M_PER_DEG_LAT = 111320.0


class RtkSim(Node):

    def __init__(self):
        super().__init__('rtk_sim')

        self.declare_parameter('input_topic', '/rtk/navsat')
        self.declare_parameter('output_topic', '/rtk/sim_fix')
        # 噪声：位置标准差（米）。RK 标称 1~2 cm，这里默认 0.02 m（2 cm 1σ）
        self.declare_parameter('sigma_m', 0.02)
        # 固定偏差（米，东/北方向）。真实系统里由天线相位中心、基准站坐标等系统性因素造成，
        # 这里给一个小的固定值，用来验证"系统不会凭空把有偏的来源纠正成正确"。
        self.declare_parameter('bias_east_m', 0.05)
        self.declare_parameter('bias_north_m', -0.03)
        self.declare_parameter('publish_rate_hz', 10.0)
        # 把 GPS 消息的 frame_id 改成机器人 TF 里真实存在的坐标系。
        # Gazebo 的 navsat 传感器发出来的 frame_id 是 "robot_rtk/base_footprint/navsat_sensor"
        # 这种 gz 内部命名，TF 里没有这一帧，navsat_transform 会一直报
        # "Could not obtain base_footprint -> ... transform" 并且算不出 map 系位置
        # （实测 /odometry/gps 恒为 0）。改成 URDF 里真实存在的 gps_link 即可。
        self.declare_parameter('frame_id_override', 'gps_link')

        self.sigma = float(self.get_parameter('sigma_m').value)
        self.bias_e = float(self.get_parameter('bias_east_m').value)
        self.bias_n = float(self.get_parameter('bias_north_m').value)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(NavSatFix, out_topic, 10)
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/rtk/sim_status', latched)

        status = String()
        status.data = ('模拟 RTK（rtk_sim）：输出 %s = Gazebo GPS + 高斯噪声(σ=%.3f m) '
                       '+ 固定偏差(东 %+.3f m, 北 %+.3f m)。'
                       '不代表真实 RTK 精度；不模拟失锁/浮点解/多路径。'
                       % (out_topic, self.sigma, self.bias_e, self.bias_n))
        self.status_pub.publish(status)

        self.create_subscription(NavSatFix, in_topic, self.on_fix, 10)
        self.count = 0
        self.last_log = self.get_clock().now()

        self.get_logger().info(
            '**模拟 RTK** 启动：%s -> %s，σ=%.3f m，固定偏差 东%+.3f / 北%+.3f m'
            % (in_topic, out_topic, self.sigma, self.bias_e, self.bias_n))

    def on_fix(self, msg: NavSatFix):
        out = NavSatFix()
        out.header.stamp = msg.header.stamp
        override = str(self.get_parameter('frame_id_override').value)
        out.header.frame_id = override if override else msg.header.frame_id

        # 把米制噪声与偏差换成经纬度增量（本场景尺度下等距投影足够）
        de = self.bias_e + (self.nogauss() * self.sigma)
        dn = self.bias_n + (self.nogauss() * self.sigma)
        m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(msg.latitude))

        out.latitude = msg.latitude + dn / M_PER_DEG_LAT
        out.longitude = msg.longitude + de / m_per_deg_lon
        out.altitude = msg.altitude

        out.status.status = msg.status.status
        # service 字段保持和来源一致；"这是模拟的"由话题名与 /rtk/sim_status 表达，
        # 不伪造 RTK 的服务类型（那会变成对下游撒谎）。
        out.status.service = msg.status.service
        # 协方差按我们注入的噪声填，供 EKF 直接使用
        var = self.sigma * self.sigma
        out.position_covariance = [var, 0.0, 0.0,
                                   0.0, var, 0.0,
                                   0.0, 0.0, var * 4.0]
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub.publish(out)
        self.count += 1

    def nogauss(self):
        """标准正态样本。用 Box-Muller，避免为了一个随机数引入 numpy 依赖到回调热路径。"""
        import random
        return random.gauss(0.0, 1.0)


def main(argv=None):
    rclpy.init(args=argv)
    node = RtkSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
