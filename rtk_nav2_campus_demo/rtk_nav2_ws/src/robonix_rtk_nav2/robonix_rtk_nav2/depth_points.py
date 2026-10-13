#!/usr/bin/env python3
"""深度图 → 点云（**替换 `depth_image_proc/point_cloud_xyz`**），并给**无回波像素**
沿它自己的射线补一个落在**远裁剪面**上的点，让 nav2 能把那些方向清成空闲。

## 为什么必须在上游做（brief §1.1）

nav2 的清除**只沿"有端点的那条射线"走**，而 `point_cloud_xyz` **只给有效深度发点**
（无效像素发 NaN）。像素没有回波（看天空 / 出远裁剪 / 无效）时，那个方向**一个有效点都没有**
→ 没有端点 → 不清空 → 格子保持未知。
**nav2 对 PointCloud2 源没有 `inf_is_valid` 那种处理**（那是 LaserScan 源的能力），
所以这件事只能在"深度→点云"这一步做。

## 本节点相对 `point_cloud_xyz` 的唯一差别

| 像素 | point_cloud_xyz | 本节点 |
| --- | --- | --- |
| 有效深度 | `(x_n·Z, y_n·Z, Z)` | 同 |
| 无回波（NaN / inf / 0 / 超远裁剪） | `(NaN, NaN, NaN)` —— **没有端点** | `(x_n·far, y_n·far, far)`，`far=8.0` —— **有端点** |

其中 `x_n = (u-cx)/fx`、`y_n = (v-cy)/fy`，与 `point_cloud_xyz` 的**沿光轴**约定一致
（不是径向：第一阶段的 `Z_FLOOR` 标定是在"深度=沿光轴距离"下标的）。
**点数与改前完全一样**（都是一像素一个点），所以带宽/CPU 不因为这一项上涨。

## 为什么远点取在"远裁剪面"（8.0 m）

  · **必须 > `obstacle_max_range`（相机源是 6.0）**：否则这个远点会被当成**障碍物**标记，
    在开阔方向凭空立起一圈假墙。8.0 > 6.0 ✓（这是本方案能成立的一个关键前提，
    别把 `far_clip` 调到 6.0 以下）。
  · **必须落在相机自己的量程内**（`<clip><far>` = 8.0 m），否则"在量程内判空闲"名不副实。
  · 清除射线最终会被该源的 `raytrace_max_range`（6.0）截住 —— 也就是"清到 6 m 为止"。

## 代价与治不了的三块（必须写进已知限制，不许省略）

这是**用"没看到"推断"没有"**：对**透明 / 镜面 / 极暗**物体不成立（仿真里基本没有，真机要注意）。
它治不了：
  ① **相机水平视场 60° 之外**（侧后方）——那些方向根本没有像素，无从生成端点；
  ② **约 0.46 m 以内**看不到的贴地区域（相机离地 0.19 m、垂直半视场 23.4°，
     最下一条射线打到地面的距离 ≈ 0.462 m）；
  ③ 超过 `raytrace_max_range`（6.0 m）的格子。

## 实测到的第二条限制（2026-09-21，读 nav2 二进制 + 头文件确认，别当成"应该没问题"）

`nav2_costmap_2d::ObservationBuffer::bufferCloud()` 把点云变换到地图帧后，
**按该源的 `min_obstacle_height` / `max_obstacle_height` 逐点过滤并"丢弃"**
（`observation_buffer.hpp` 里这两个成员 + `libnav2_costmap_2d_core.so` 里
`bufferCloud` 对 `[x28,#216]`/`[x28,#224]` 的 `fcmpe` 带过滤与紧凑搬运可证）。
**被丢掉的点既不标记也不清除**——它们在射线清除里也不存在。于是（相机源 min=0.30 / max=1.2）：

  · **地面点（z≈0）全部被丢弃**：相机**本来就不清地面**（这与"幽灵格被 0.30 拦住"
    是同一枚硬币的两面，不是两件事）；
  · 本节点补的远点，**只有落在地图帧 z ∈ [0.30, 1.2] 的才留得下**：8 m 处对应
    仰角约 **0.7°~7.2°**（240 行里约 32 行，仰角更高的行远点 z>1.2 被丢）。

**这仍然够用，但要说清为什么**：清空一个方位**只需要该方位有一条带端点的射线**，
而那 32 行覆盖 ±30° 内的**每一个方位**（每一列都有）。所以"视场内无回波方位被清到 6 m"
成立；而"整幅图的每个像素方向都被清"**不成立**，也不该这么写。

订阅：`input_topic`(32FC1 深度图) + `info_topic`(CameraInfo，取内参)
发布：`output_topic`(PointCloud2 x/y/z float32)，frame_id 沿用深度图（光学约定帧）

参数：
  `emit_far_points`（默认 true）：**关掉它就是改前的行为**（无效像素发 NaN）。
    留这个开关是为了做"无回波⇒空闲"的**同一次运行内 A/B 对照**（判据 ①的前后计数），
    不是给演示用的第二个模式。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

# 相机水平视场（worlds/rtk_courtyard.sdf 与 URDF 里写死 1.0472 rad = 60°）。
# **只在 camera_info 还没到时**用来兜底算 fx —— 正常路径一律用 camera_info 的内参。
HFOV = 1.0472

FIELDS = (
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
)


def _as_bool(value, default=False):
    """把参数值解析成 bool，**字符串与布尔都要能收**。

    为什么需要：`emit_far_points` 要能被 launch 传（做判据 ①的 A/B 对照），
    而 **`LaunchConfiguration` 代入到节点参数里的是字符串**（'true'/'false'），
    命令行 `-p emit_far_points:=true` 传的却是**布尔**。
    如果把它声明成 bool，launch 传字符串会在**声明那一刻**抛类型错、节点直接起不来
    ——现象是"这个话题没有发布者"，很容易被当成"传感器没数据"去查。
    所以这里不声明类型（`declare_parameter` 不给默认值），两种都收，再统一解析。
    （`far_clip` 在 launch 里写的是 Python 字面量，不走这条路，保持 float 声明。）
    """
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class DepthPoints(Node):

    def __init__(self):
        super().__init__('depth_points')
        self.declare_parameter('input_topic', '/rtk/camera/depth_image')
        self.declare_parameter('info_topic', '/rtk/camera/camera_info')
        self.declare_parameter('output_topic', '/rtk/camera/points')
        # 远裁剪距离：与 URDF 的 <clip><far>8.0</far> 同值（改一处必须改另一处）
        self.declare_parameter('far_clip', 8.0)
        # 无效像素是否补"远裁剪面上的点"（关掉 = 改前行为，供 A/B）。
        # **不声明类型**：launch 传字符串、命令行传布尔，两种都要能收（见 `_as_bool`）。
        self.declare_parameter('emit_far_points')
        # 统计日志周期（帧）。10 Hz 下 50 帧 = 5 s 一条，够做前后计数对照。
        self.declare_parameter('log_every', 50)

        p = self.get_parameter
        self.input_topic = p('input_topic').value
        self.output_topic = p('output_topic').value
        self.far_clip = float(p('far_clip').value)
        self.emit_far = _as_bool(p('emit_far_points').value, default=True)
        self.log_every = int(p('log_every').value)

        self.intr = None            # (fx, fy, cx, cy)
        self.frames = 0
        self.valid_total = 0
        self.invalid_total = 0
        self.bad_encoding_logged = False
        self.no_info_logged = False

        self.pub = self.create_publisher(PointCloud2, self.output_topic,
                                         qos_profile_sensor_data)
        self.create_subscription(CameraInfo, p('info_topic').value,
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.input_topic, self.on_depth,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            '深度->点云：%s -> %s（far_clip=%.2f m，无效像素补远点=%s）'
            % (self.input_topic, self.output_topic, self.far_clip,
               '开' if self.emit_far else '关（改前行为）'))

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
                '还没收到 camera_info，暂按水平视场 %.4f rad 兜底算内参（主点取图像中心）；'
                '正常路径应当用 camera_info 的内参。' % HFOV)
        fx = width / (2.0 * np.tan(HFOV / 2.0))
        return (fx, fx, width / 2.0, height / 2.0)

    # ------------------------------------------------------------------ 主循环
    def on_depth(self, msg: Image):
        if msg.encoding != '32FC1' and not self.bad_encoding_logged:
            # 不 return：先按 float32 试一次，真读不出来再报（Gazebo 的 depth 就是 32FC1）。
            self.bad_encoding_logged = True
            self.get_logger().error(
                '深度图编码是 %r，不是 32FC1；本节点按 float32 解释，结果可能是错的。'
                % msg.encoding)
        w, h = int(msg.width), int(msg.height)
        try:
            z = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        except ValueError:
            if not self.bad_encoding_logged:
                self.bad_encoding_logged = True
                self.get_logger().error(
                    '深度图 %dx%d 的数据长度与 32FC1 对不上（%d 字节），已跳过该帧。'
                    % (w, h, len(msg.data)))
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
            # 关掉时**逐字节等价于改前**：无效像素发 NaN（nav2 的 buffer 会丢 NaN）
            nan = np.float32('nan')
            pts[~valid, 0] = nan
            pts[~valid, 1] = nan
            pts[~valid, 2] = nan

        out = PointCloud2()
        out.header = msg.header          # frame_id 沿用深度图（光学帧），与改前一致
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
                '已发 %d 帧：有效像素 %d / 无效像素 %d（%.1f%%）'
                '；无效像素%s'
                % (self.frames, self.valid_total, self.invalid_total,
                   100.0 * self.invalid_total / max(1, n * self.frames),
                   ('已补远裁剪面 %.2f m 上的点' % far) if self.emit_far
                   else '按改前行为发 NaN'))


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


if __name__ == '__main__':
    raise SystemExit(main())
