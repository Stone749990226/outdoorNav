#!/usr/bin/env python3
"""★ 失败诊断的活体记录器 ✓ —— 20 Hz 落 CSV ✓（★ 只为取证 ✗ 不影响控制 ✓）

★ 为什么要有它 ✓（reviewer 2026-10-13 ✓）：
  ★ 失败是**偶发**（10 遍里 1 遍 ✓）⇒ ★ 事后翻日志**量不到那一刻的原始量** ✗
  ★ ⇒ ★ 必须在**它发生时**就有记录 ✓
★ 记四路 ✓（★ 判 ③ 要用 ✓）：
  ★ `/ground_truth/odom`  ⇒ ★ 车的**真值**阵位 ✓（★ 尺子：仿真真值 ✓）
  ★ `/odometry/local`     ⇒ ★ **喂给 controller_server 的**那条 ✓（`campus_nav2.yaml:352/415` ✓）
  ★ TF `map→base_footprint` ⇒ ★ 控制器把路径变换到车体时**实际用**的位姿 ✓（`:247` ✓）
  ★ `/outdoor_nav/planned_path` ⇒ ★ retained ✓ ⇒ ★ 每次一发就存一份带时间戳的文件 ✓
"""
import csv, math, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

OUT = "/work/artifacts/rtk-nav2/demo/logs/live_records"
import os; os.makedirs(OUT, exist_ok=True)

class Rec(Node):
    def __init__(self):
        super().__init__("run_recorder")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.gt = None; self.loc = None; self.gt_v = (0.0, 0.0); self.loc_stamps = []; self.paths = 0
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_gt, 50)
        self.create_subscription(Odometry, "/odometry/local", self.on_loc, 50)
        self.create_subscription(RosPath, "/outdoor_nav/planned_path", self.on_path, 1)
        self.cmd = None
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 50)
        self.f = open(OUT + "/live.csv", "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow(["t_sim", "gt_x", "gt_y", "loc_x", "loc_y", "tf_x", "tf_y",
                         "gt_minus_loc", "gt_minus_tf", "loc_age_s",
                         "cmd_lx", "cmd_az", "gt_vx", "gt_wz",
                         "map_odom_x", "map_odom_y", "map_odom_age", "odom_base_x", "odom_base_y",
                         "odom_base_age", "map_base_age"])
        self.create_timer(0.05, self.tick)
        self.rows = 0
        self.get_logger().info("★ 记录器起 ✓ 20 Hz ⇒ %s/live.csv" % OUT)

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)

    def on_gt(self, m):
        self.gt = (m.pose.pose.position.x, m.pose.pose.position.y, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        self.gt_v = (m.twist.twist.linear.x, m.twist.twist.angular.z)
    def on_loc(self, m):
        self.loc = (m.pose.pose.position.x, m.pose.pose.position.y)
        self.loc_stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        if len(self.loc_stamps) > 4000: self.loc_stamps = self.loc_stamps[-2000:]
    def on_path(self, m):
        self.paths += 1
        # ★★ 2026-10-13 实测踩到 ✗：★ 话题是 TRANSIENT_LOCAL（retained ✓）⇒
        #   ★ 记录器**一起来**就先收到**上一条残留路径** ✓（头戳是旧的 ✓）
        #   ★ 用**头戳**命名 ⇒ ★ 文件名与"这一遍的路径"**对不上** ✗ ⇒ 我拿它比错过一次 ✓
        #   ★ ⇒ ★ 改用 ★ **收到时刻**（仿真时间 ✓）命名 ＋ 同时记下头戳 ✓
        now = self.get_clock().now().nanoseconds * 1e-9
        recv = int(round(now * 10))
        hdr = m.header.stamp.sec
        fn = "%s/path_%03d_recv%06d_hdr%d.csv" % (OUT, self.paths, recv, hdr)
        with open(fn, "w", encoding="utf-8") as fh:
            fh.write("i,x,y\n")
            for i, p in enumerate(m.poses):
                fh.write("%d,%.6f,%.6f\n" % (i, p.pose.position.x, p.pose.position.y))
        self.get_logger().info("★ 存下路径 #%d ✓ 点数 %d ⇒ %s" % (self.paths, len(m.poses), fn))

    def tick(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        def lk(a, b):
            try:
                tr = self.tf.lookup_transform(a, b, rclpy.time.Time())
                t = tr.transform.translation
                # ★ 同时取这条 TF 的**时间戳** ✗ ⇒ ★ 判它是不是停了 ✓
                return t.x, t.y, tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            except Exception:
                return float("nan"), float("nan"), float("nan")
        tfx, tfy, tf_stamp = lk("map", "base_footprint")
        mox, moy, mo_stamp = lk("map", "odom")
        obx, oby, ob_stamp = lk("odom", "base_footprint")
        if self.gt is None or self.loc is None:
            return
        d_loc = math.hypot(self.gt[0] - self.loc[0], self.gt[1] - self.loc[1])
        d_tf = math.hypot(self.gt[0] - tfx, self.gt[1] - tfy) if tfx == tfx else float("nan")
        age = (t - self.loc_stamps[-1]) if self.loc_stamps else float("nan")
        self.w.writerow(["%.4f" % t, "%.4f" % self.gt[0], "%.4f" % self.gt[1],
                         "%.4f" % self.loc[0], "%.4f" % self.loc[1],
                         "%.4f" % tfx, "%.4f" % tfy, "%.4f" % d_loc, "%.4f" % d_tf, "%.4f" % age,
                         "%.4f" % (self.cmd[0] if self.cmd else 0.0),
                         "%.4f" % (self.cmd[2] if self.cmd else 0.0),
                         "%.4f" % self.gt_v[0], "%.4f" % self.gt_v[1],
                         "%.4f" % mox, "%.4f" % moy, "%.4f" % ((t - mo_stamp) if mo_stamp == mo_stamp else float("nan")),
                         "%.4f" % obx, "%.4f" % oby,
                         "%.4f" % ((t - ob_stamp) if ob_stamp == ob_stamp else float("nan")),
                         "%.4f" % ((t - tf_stamp) if tf_stamp == tf_stamp else float("nan"))])
        self.rows += 1
        if self.rows % 100 == 0: self.f.flush()

def main():
    rclpy.init(); n = Rec()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        n.f.flush(); n.f.close(); n.get_logger().info("共 %d 行 ✓ 路径 %d 份" % (n.rows, n.paths))
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 0

if __name__ == "__main__":
    sys.exit(main())
