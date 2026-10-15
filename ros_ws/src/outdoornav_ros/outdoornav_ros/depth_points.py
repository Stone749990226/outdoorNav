#!/usr/bin/env python3
"""Project depth into point clouds; no-return rays clear beyond the marking range."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

# 相机水平视场（worlds/rtk_courtyard.sdf 与 URDF 里写死 1.0472 rad = 60°）。
# **只在 camera_info 还没到时**用来兜底算 fx —— 正常路径一律用 camera_info 的内参。
HFOV = 1.0472

FIELDS = (
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
)


class DepthPoints(Node):
    def __init__(self):
        super().__init__("depth_points")
        self.declare_parameter("input_topic", "/rtk/camera/depth_image")
        self.declare_parameter("info_topic", "/rtk/camera/camera_info")
        self.declare_parameter("output_topic", "/rtk/camera/points")
        # 远裁剪距离：与 URDF 的 <clip><far>8.0</far> 同值（改一处必须改另一处）
        self.declare_parameter("far_clip", 8.0)
        self.declare_parameter("emit_far_points", True)
        # 统计日志周期（帧）。10 Hz 下 50 帧 = 5 s 一条，够做前后计数对照。
        self.declare_parameter("log_every", 50)

        p = self.get_parameter
        self.input_topic = p("input_topic").value
        self.output_topic = p("output_topic").value
        self.far_clip = float(p("far_clip").value)
        self.emit_far = p("emit_far_points").value
        self.log_every = int(p("log_every").value)

        self.intr = None  # (fx, fy, cx, cy)
        self.frames = 0
        self.valid_total = 0
        self.invalid_total = 0
        self.bad_encoding_logged = False
        self.no_info_logged = False

        self.pub = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, p("info_topic").value, self.on_info, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, self.input_topic, self.on_depth, qos_profile_sensor_data
        )
        self.get_logger().info(
            "深度->点云：%s -> %s（far_clip=%.2f m，无效像素补远点=%s）"
            % (
                self.input_topic,
                self.output_topic,
                self.far_clip,
                "开" if self.emit_far else "关",
            )
        )

    # ------------------------------------------------------------------ 内参
    def on_info(self, msg: CameraInfo):
        k = msg.k
        if k[0] <= 0.0 or k[4] <= 0.0:
            return
        self.intr = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))

    def _intrinsics(self, width, height):
        """优先用 camera_info；没到就按水平视场兜底（并**只告警一次**）。"""
        if self.intr is not None:
            return self.intr
        if not self.no_info_logged:
            self.no_info_logged = True
            self.get_logger().warn(
                "还没收到 camera_info，暂按水平视场 %.4f rad 兜底算内参（主点取图像中心）；"
                "正常路径应当用 camera_info 的内参。" % HFOV
            )
        fx = width / (2.0 * np.tan(HFOV / 2.0))
        return (fx, fx, width / 2.0, height / 2.0)

    # ------------------------------------------------------------------ 主循环
    def on_depth(self, msg: Image):
        if msg.encoding != "32FC1" and not self.bad_encoding_logged:
            # 不 return：先按 float32 试一次，真读不出来再报（Gazebo 的 depth 就是 32FC1）。
            self.bad_encoding_logged = True
            self.get_logger().error(
                "深度图编码是 %r，不是 32FC1；本节点按 float32 解释，结果可能是错的。"
                % msg.encoding
            )
        w, h = int(msg.width), int(msg.height)
        try:
            z = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        except ValueError:
            if not self.bad_encoding_logged:
                self.bad_encoding_logged = True
                self.get_logger().error(
                    "深度图 %dx%d 的数据长度与 32FC1 对不上（%d 字节），已跳过该帧。"
                    % (w, h, len(msg.data))
                )
            return

        fx, fy, cx, cy = self._intrinsics(w, h)
        far = self.far_clip

        # 有效 = 有限、正、且不超远裁剪。其余（NaN/inf/0/超远）都算"这条射线量程内没看到东西"。
        valid = np.isfinite(z) & (z > 0.0) & (z <= far)

        # 无效像素沿它自己的射线落到**远裁剪面**上：z_n = far（与 point_cloud_xyz 的
        # "深度=沿光轴"约定一致），x/y 按同一比例缩放。
        z_use = np.where(valid, z, np.float32(far))
        xn = ((np.arange(w, dtype=np.float32) - cx) / fx)[None, :]
        yn = ((np.arange(h, dtype=np.float32) - cy) / fy)[:, None]
        pts = np.empty((h, w, 3), dtype=np.float32)
        pts[..., 0] = xn * z_use
        pts[..., 1] = yn * z_use
        pts[..., 2] = z_use
        if not self.emit_far:
            nan = np.float32("nan")
            pts[~valid, 0] = nan
            pts[~valid, 1] = nan
            pts[~valid, 2] = nan

        out = PointCloud2()
        out.header = msg.header  # frame_id 沿用深度图（光学帧），与改前一致
        out.height = 1
        out.width = w * h
        out.fields = list(FIELDS)
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * w * h
        out.is_dense = False
        out.data = pts.reshape(-1, 3).tobytes()
        self.pub.publish(out)

        self.frames += 1
        self.valid_total += int(valid.sum())
        self.invalid_total += int((~valid).sum())
        if self.log_every > 0 and self.frames % self.log_every == 0:
            n = w * h
            self.get_logger().info(
                "已发 %d 帧：有效像素 %d / 无效像素 %d（%.1f%%）"
                "；无效像素%s"
                % (
                    self.frames,
                    self.valid_total,
                    self.invalid_total,
                    100.0 * self.invalid_total / max(1, n * self.frames),
                    ("已补远裁剪面 %.2f m 上的点" % far)
                    if self.emit_far
                    else "按 NaN 处理",
                )
            )


def main(argv=None):
    rclpy.init(args=argv)
    node = DepthPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
