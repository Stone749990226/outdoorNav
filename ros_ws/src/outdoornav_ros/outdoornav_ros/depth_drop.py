#!/usr/bin/env python3
"""Depth-based negative obstacle detection and keepout masks."""

import json
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


from outdoornav.perception import rotation, edge_columns, near_ground_z


class DropDetector(Node):
    def __init__(self):
        super().__init__("depth_drop")
        self.declare_parameter("camera_height", 0.19)
        self.declare_parameter("drop_threshold", 0.15)
        self.declare_parameter("ground_tolerance", 0.06)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("min_columns", 12)
        self.declare_parameter("evidence_dir", "")
        self.declare_parameter("mask_stale_timeout", 1.0)
        # 禁行区按车体足迹外扩的量（米）。0.15 = 内切半径 0.105 + 5 cm 余量；
        # 目的：让"规划器避开的地方"与"控制器拒绝的地方"是同一条口径（见 publish_mask 的说明）。
        self.declare_parameter("hazard_margin", 0.15)

        self.declare_parameter("floor_consistency_tol", 0.25)

        self.declare_parameter("ground_ref", "plane")

        # ⇒ ② 把 T 降到 0.10 后必须重跑该 A/B，否则"边界翻转"只是换个位置再出现）。
        self.declare_parameter("ref_truncate", False)
        self.camera_height = float(self.get_parameter("camera_height").value)
        self.drop_threshold = float(self.get_parameter("drop_threshold").value)
        self.ground_tolerance = float(self.get_parameter("ground_tolerance").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.min_columns = int(self.get_parameter("min_columns").value)
        self.evidence_dir = str(self.get_parameter("evidence_dir").value or "")
        self.mask_stale_timeout = float(self.get_parameter("mask_stale_timeout").value)
        self.hazard_margin = float(self.get_parameter("hazard_margin").value)
        self.floor_consistency_tol = float(
            self.get_parameter("floor_consistency_tol").value
        )
        self.ground_ref = str(self.get_parameter("ground_ref").value)
        self.ref_truncate = bool(self.get_parameter("ref_truncate").value)
        self.last_confirmed_wall = None
        # Persistent map-frame hazard store. Confirmed edges only add cells; loss of
        # view never removes them. Local masks are projected from this same store.
        self.hazard_points = []
        self.frame_log = None
        self.merge_log = None
        self.edges_log = None
        if self.evidence_dir:
            os.makedirs(self.evidence_dir, exist_ok=True)
            self.frame_log = open(
                os.path.join(self.evidence_dir, "frames.jsonl"), "a", encoding="utf-8"
            )

            self.merge_log = open(
                os.path.join(self.evidence_dir, "merge_events.jsonl"),
                "a",
                encoding="utf-8",
            )

            #   若那些边不在南沿上，放宽门只是把别处的短边放进图 = 新增假阳性。
            self.edges_log = open(
                os.path.join(self.evidence_dir, "frame_edges.jsonl"),
                "a",
                encoding="utf-8",
            )
        self.intr = None
        self.info_stamp = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(
            OccupancyGrid, "/rtk/drop/mask_map", latched
        )
        self.odom_pub = self.create_publisher(
            OccupancyGrid, "/rtk/drop/mask_odom", latched
        )
        self.map_info = self.create_publisher(
            CostmapFilterInfo, "/rtk/drop/filter_info_map", latched
        )
        self.odom_info = self.create_publisher(
            CostmapFilterInfo, "/rtk/drop/filter_info_odom", latched
        )
        for frame, pub in (("map", self.map_info), ("odom", self.odom_info)):
            info = CostmapFilterInfo()
            info.header.frame_id = frame
            info.type = 0  # nav2 KeepoutFilter
            info.filter_mask_topic = "/rtk/drop/mask_" + frame
            info.base = 0.0
            info.multiplier = 1.0
            pub.publish(info)
        self.display_pub = self.create_publisher(Image, "/rtk/drop/classification", 5)
        self.frames = 0
        self.create_subscription(
            CameraInfo, "/rtk/camera/camera_info", self.on_info, qos_profile_sensor_data
        )
        self.pending_depth = None
        self.pending_since = None
        self.pending_attempts = 0
        self.create_subscription(
            Image, "/rtk/camera/depth_image", self.on_depth, qos_profile_sensor_data
        )
        # The camera bridge can deliver an image a few milliseconds before /tf.
        # Retry the same frame instead of replacing it with the next frame.
        self.create_timer(0.03, self.process_depth)

    def on_info(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self.intr = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.info_stamp = msg.header.stamp

    def on_depth(self, msg):
        if msg.encoding == "32FC1" and self.intr is not None:
            if self.pending_depth is None:
                self.pending_since = self.get_clock().now()
                self.pending_attempts = 0
            self.pending_depth = msg

    def process_depth(self):
        msg = self.pending_depth
        if msg is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        depth_stamp = stamp.nanoseconds * 1e-9
        retry_count = self.pending_attempts
        self.pending_attempts += 1
        try:
            if (
                abs((stamp - Time.from_msg(self.info_stamp)).nanoseconds)
                > 1_000_000_000
            ):
                self.get_logger().warn(
                    "CameraInfo older than 1 s; frame undecidable",
                    throttle_duration_sec=5.0,
                )
                return
            # TF often lags the depth image by 2–20 ms. Ask for that image's stamp
            # with a bounded wait; do not substitute the latest transform.
            tf_map = self.tf.lookup_transform("map", msg.header.frame_id, stamp)
            tf_odom = self.tf.lookup_transform("odom", "map", stamp)
            self.pending_depth = None
            self.pending_since = None
            self.pending_attempts = 0
            tf_stamp = stamp.nanoseconds * 1e-9
            data = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.step // 4
            )[:, : msg.width]
        except (Exception,) as exc:
            # Keep the pending frame for up to 0.25 s. A bounded retry makes a
            # 12–20 ms TF lag a recoverable transport skew, not a false detector
            # failure; after the bound the frame becomes undecidable and is dropped.
            if (
                self.pending_since is not None
                and (self.get_clock().now() - self.pending_since).nanoseconds
                < 250_000_000
            ):
                return
            self.pending_depth = None
            self.pending_since = None
            self.get_logger().warn(
                "Depth/TF unavailable after bounded retry: %s" % exc,
                throttle_duration_sec=5.0,
            )
            return
        ref_stats = {}

        #   上一版把它初始化放在日志块之后 ⇒ 节点启动即 `UnboundLocalError` 崩掉
        #   （`ramp7-ab` 连拦三次就是它；合成测试只调 `edge_columns`，不经过这里，所以没暴露）。
        #   教训：`py_compile` 只查语法，**运行路径必须真跑一次**（就绪闸门就是这个作用）。
        ptp_across = None

        #   这样 `ros2 param set /depth_drop drop_threshold 0.10` 当场生效、不必重启栈。
        #   与 `hazard_margin` 那处同一写法；**参数不变时行为与改前逐位相同**（只是多读一次）。
        drop_th = float(self.get_parameter("drop_threshold").value)
        edges, upper, lower, heading = edge_columns(
            data,
            self.intr,
            tf_map,
            self.camera_height,
            drop_th,
            self.ground_tolerance,
            ground_ref=self.ground_ref,
            ref_truncate=self.ref_truncate,
            ref_stats=ref_stats,
        )

        floor = float(tf_map.transform.translation.z) - self.camera_height
        near_z, near_n = near_ground_z(data, self.intr, tf_map)
        delta = None if near_z is None else floor - near_z
        if delta is not None and abs(delta) > self.floor_consistency_tol:
            self.get_logger().warn(
                "地面基准与近场实测地面差 %.3f m（floor %.3f / 实测 %.3f，%d 点）"
                "——发布 z 有偏置时检测器会失明，这条是活性告警（**不改行为**）"
                % (delta, floor, near_z, near_n),
                throttle_duration_sec=5.0,
            )
        # 记录用：**未过滤**的列数（与历史口径一致，不要改成过滤后的）+ 宽松预筛结果
        n_cols_raw = int(len(edges))
        if self.edges_log is not None and n_cols_raw:
            self.edges_log.write(
                json.dumps(
                    {
                        "depth_stamp": depth_stamp,
                        "wall": time.time(),
                        "drop_th": drop_th,
                        "ground_ref": self.ground_ref,
                        "n_edges": n_cols_raw,
                        "edges": [
                            [round(float(q[0]), 3), round(float(q[1]), 3)]
                            for q in np.asarray(edges).reshape(-1, 2)
                        ],
                    }
                )
                + "\n"
            )
            self.edges_log.flush()
        prefilter_ok = bool(n_cols_raw >= self.min_columns)
        display = np.zeros((msg.height, msg.width), dtype=np.uint8)
        display[::2, ::2][upper] = 110
        display[::2, ::2][lower] = 210
        confirmed = len(edges) >= self.min_columns
        if confirmed:
            # Reject isolated columns without assuming the edge is x=constant.
            axis = np.array([-heading[1], heading[0]])
            along = edges @ heading[:2]
            across = edges @ axis
            median = np.median(along)
            edges = edges[np.abs(along - median) <= 0.20]
            ptp_across = float(np.ptp(across))  # 只记录（判据仍用同一个值，未改）
            confirmed = len(edges) >= self.min_columns and np.ptp(across) >= 0.5
        if confirmed:
            self.last_confirmed_wall = time.time()
            # Merge only measured, confirmed edge points in fixed map coordinates.
            # No clearing path is allowed to remove this store during view loss.
            if self.hazard_points:
                merged = np.vstack([np.asarray(self.hazard_points), edges])
            else:
                merged = edges
            before = (
                np.asarray(self.hazard_points)
                if self.hazard_points
                else np.zeros((0, 2))
            )
            keep = []
            for point in merged:
                if not any(np.linalg.norm(point - other) < 0.08 for other in keep):
                    keep.append(point)

            #   判据 = "与入库前的任何格距离 ≥0.08 m"（与上面的去重口径一致）。
            added = [
                list(map(float, p_))
                for p_ in keep
                if not len(before)
                or np.min(np.linalg.norm(before - np.asarray(p_), axis=1)) >= 0.08
            ]
            self.hazard_points = keep
            if self.merge_log is not None:
                self.merge_log.write(
                    json.dumps(
                        {
                            "wall": time.time(),
                            "n_before": int(len(before)),
                            "n_after": int(len(keep)),
                            "n_added": int(len(added)),
                            "added_cells": added,
                            "drop_th": drop_th,
                            "ground_ref": self.ground_ref,
                        }
                    )
                    + "\n"
                )
                self.merge_log.flush()
        if self.frame_log:
            self.frame_log.write(
                json.dumps(
                    {
                        "wall": time.time(),
                        "depth_stamp": depth_stamp,
                        "tf_requested_stamp": depth_stamp,
                        "tf_return_stamp": tf_stamp,
                        "tf_retry_count": retry_count,
                        "tf_delay_ms": round((tf_stamp - depth_stamp) * 1000, 3),
                        "valid_pixels": int(np.isfinite(data).sum()),
                        # 口径不变：仍是**未过滤**的列数（历史可比）
                        "edge_columns": n_cols_raw,
                        #   不是入库确认 ⇒ 现名 `prefilter_ok`，真正的入库条件单列 `merged`。
                        "prefilter_ok": prefilter_ok,
                        "merged": bool(
                            confirmed
                        ),  # = 列数≥12 **且** ptp(across)≥0.5 且随后入库
                        "ptp_across": ptp_across,
                        "upper_px": int(upper.sum()),
                        "lower_px": int(lower.sum()),
                        "floor": round(floor, 4),
                        "near_ground_z": None if near_z is None else round(near_z, 4),
                        "near_ground_n": near_n,
                        "floor_minus_near_ground": None
                        if delta is None
                        else round(delta, 4),
                        "live": bool(int(upper.sum()) > 0),
                        "ground_ref": self.ground_ref,
                        "drop_th": drop_th,
                        "ref_source": ref_stats or None,
                    }
                )
                + "\n"
            )
            self.frame_log.flush()
        if self.hazard_points:
            map_edges = np.asarray(self.hazard_points)
            self.publish_mask("map", map_edges, msg.header.stamp, self.map_pub)
            q = tf_odom.transform.rotation
            p = tf_odom.transform.translation
            odom_edges = map_edges @ rotation(q)[:2, :2].T + np.array([p.x, p.y])
            self.publish_mask("odom", odom_edges, msg.header.stamp, self.odom_pub)
            # Magenta/bright edge in the classification image (valid ground=110, lower=210).
            for u in range(0, msg.width, 2):
                if np.any(upper[:, u // 2]) and np.any(lower[:, u // 2]):
                    v = np.where(upper[:, u // 2])[0][0] * 2
                    display[max(0, v - 2) : min(msg.height, v + 3), u : u + 2] = 255
        # Lifecycle evidence: do not keep publishing a stale keepout mask forever.
        # The filter receives no synthetic free mask; after TTL the mask remains the
        # last measured mask, while the evidence stream records STALE_UNKNOWN.
        if self.frame_log and self.last_confirmed_wall is not None:
            age = time.time() - self.last_confirmed_wall
            if age > self.mask_stale_timeout:
                self.frame_log.write(
                    json.dumps(
                        {
                            "wall": time.time(),
                            "lifecycle": "STALE_UNKNOWN",
                            "last_confirmed_age_s": age,
                        }
                    )
                    + "\n"
                )
                self.frame_log.flush()
        image = Image()
        image.header = msg.header
        image.height = msg.height
        image.width = msg.width
        image.encoding = "mono8"
        image.step = msg.width
        image.data = display.tobytes()
        self.display_pub.publish(image)
        self.frames += 1
        if self.frames % 30 == 0:
            self.get_logger().info(
                "upper=%d lower=%d edge_columns=%d confirmed=%s"
                % (upper.sum(), lower.sum(), len(edges), confirmed)
            )

    def publish_mask(self, frame, points, stamp, publisher):
        # Sparse local window. Unknown (-1) everywhere outside measured edge samples:
        # no synthetic free ground is emitted by this detector.
        origin = np.min(points, axis=0) - 0.40
        max_xy = np.max(points, axis=0) + 0.40
        width, height = np.ceil((max_xy - origin) / self.resolution).astype(int) + 1
        if width > 240 or height > 240:
            return
        grid = np.full((height, width), -1, dtype=np.int8)
        # Neighboring camera columns can land more than one 5 cm cell apart.
        # Connect only adjacent, coherent measurements; never fill a missing span.
        cells = np.floor((points - origin) / self.resolution).astype(int)
        order = np.argsort(points[:, 1])
        for index in order:
            ix, iy = cells[index]
            if 0 <= ix < width and 0 <= iy < height:
                grid[iy, ix] = 100
        for left, right in zip(order[:-1], order[1:]):
            a, b = points[left], points[right]
            if np.linalg.norm(b - a) > 0.16:
                continue
            steps = max(1, int(np.ceil(np.linalg.norm(cells[right] - cells[left]))))
            for ix, iy in np.rint(
                np.linspace(cells[left], cells[right], steps + 1)
            ).astype(int):
                if 0 <= ix < width and 0 <= iy < height:
                    grid[iy, ix] = 100

        margin = float(self.get_parameter("hazard_margin").value)
        margin_cells = int(np.ceil(margin / self.resolution))
        if margin_cells > 0:
            dilated = grid >= 100
            for _ in range(margin_cells):
                shifted = [
                    np.roll(dilated, 1, axis=0),
                    np.roll(dilated, -1, axis=0),
                    np.roll(dilated, 1, axis=1),
                    np.roll(dilated, -1, axis=1),
                ]
                for s in shifted:
                    dilated = dilated | s
            grid[dilated] = 100
        out = OccupancyGrid()
        out.header.stamp = stamp
        out.header.frame_id = frame
        out.info.resolution = self.resolution
        out.info.width = int(width)
        out.info.height = int(height)
        out.info.origin.position.x = float(origin[0])
        out.info.origin.position.y = float(origin[1])
        out.info.origin.orientation.w = 1.0
        out.data = grid.reshape(-1).astype(np.int8).tolist()
        publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DropDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.frame_log:
            node.frame_log.close()
        if node.merge_log:
            node.merge_log.close()
        if node.edges_log:
            node.edges_log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
