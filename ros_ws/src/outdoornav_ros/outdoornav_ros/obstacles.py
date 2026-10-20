"""Gazebo obstacle control shared by HTTP, Web and the command line."""

import csv
import math
from pathlib import Path
import threading

from outdoornav.api import ApiError
from outdoornav.bundle import is_free, world_cell
from outdoornav.obstacles import box_sdf, obstacle_spec, position_at


class Obstacles:
    def __init__(self, node, run_dir):
        from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
        self.node = node
        self.lock = threading.RLock()
        self.items, self.pending, self.deleting = {}, set(), set()
        self.create = node.create_client(
            SpawnEntity, "/world/north_campus/create", callback_group=node.group
        )
        self.remove = node.create_client(
            DeleteEntity, "/world/north_campus/remove", callback_group=node.group
        )
        self.move = node.create_client(
            SetEntityPose, "/world/north_campus/set_pose", callback_group=node.group
        )
        self.log = (Path(run_dir) / "obstacles.csv").open("w", newline="", buffering=1)
        self.writer = csv.writer(self.log)
        self.writer.writerow(
            ["sim_s", "event", "name", "x", "y", "width", "depth", "height"]
        )
        self.timer = node.create_timer(0.1, self.tick, callback_group=node.group)

    def record(self, item, event):
        p = item["position"]
        self.writer.writerow(
            [
                self.now(),
                event,
                item["name"],
                p["x"],
                p["y"],
                item["width"],
                item["depth"],
                item["height"],
            ]
        )

    def now(self):
        return self.node.get_clock().now().nanoseconds / 1e9

    def snapshot(self):
        with self.lock:
            return [dict(item) for item in self.items.values()]

    def call(self, client, request):
        if not client.service_is_ready():
            raise ApiError(503, "Gazebo obstacle service is unavailable")
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(6):
            raise ApiError(
                504,
                "Gazebo obstacle operation timed out; check the scene before retrying",
            )
        if not future.result().success:
            raise ApiError(502, "Gazebo rejected the obstacle operation")

    def add(self, payload):
        spec = obstacle_spec(payload)
        p = spec["start"]
        if not is_free(self.node.free, self.node.bundle["grid"], p):
            raise ValueError("Obstacle center must be inside static free space")
        pose = self.node.pose()
        if (
            pose
            and math.hypot(pose["x"] - p["x"], pose["y"] - p["y"])
            < math.hypot(spec["width"], spec["depth"]) / 2 + 0.405
        ):
            raise ValueError("Obstacle is too close to the robot")
        # Scripted motion must stay on static free ground, including the full box footprint.
        distance = math.dist(tuple(p.values()), tuple(spec["end"].values()))
        count = max(1, math.ceil(distance / 0.1))
        if spec["kind"] == "moving":
            for index in range(count + 1):
                x, y = (
                    p[k] + index / count * (spec["end"][k] - p[k]) for k in ("x", "y")
                )
                r0, c0 = world_cell(
                    self.node.bundle["grid"],
                    x - spec["width"] / 2,
                    y - spec["depth"] / 2,
                )
                r1, c1 = world_cell(
                    self.node.bundle["grid"],
                    x + spec["width"] / 2,
                    y + spec["depth"] / 2,
                )
                free = self.node.free
                if (
                    r0 < 0
                    or c0 < 0
                    or r1 >= free.shape[0]
                    or c1 >= free.shape[1]
                    or not free[r0 : r1 + 1, c0 : c1 + 1].all()
                ):
                    raise ValueError(
                        "Moving obstacle footprint must stay inside static free space"
                    )
        with self.lock:
            if spec["name"] in self.items or spec["name"] in self.pending:
                raise ApiError(409, "Obstacle name already exists")
            if len(self.items) + len(self.pending - self.items.keys()) >= 5:
                raise ApiError(409, "At most five obstacles may be active")
            self.pending.add(spec["name"])
        try:
            from ros_gz_interfaces.srv import SpawnEntity
            request = SpawnEntity.Request()
            request.entity_factory.name = spec["name"]
            request.entity_factory.sdf = box_sdf(spec)
            request.entity_factory.allow_renaming = False
            request.entity_factory.pose.position.x = float(p["x"])
            request.entity_factory.pose.position.y = float(p["y"])
            request.entity_factory.pose.orientation.w = 1.0
            self.call(self.create, request)
            with self.lock:
                item = dict(spec, position=p, started_sim_s=self.now(), error=None)
                self.items[spec["name"]] = item
                self.record(item, "added")
                return dict(item)
        finally:
            with self.lock:
                self.pending.discard(spec["name"])

    def delete(self, name):
        with self.lock:
            if name not in self.items:
                raise KeyError(name)
            if name in self.deleting:
                raise ApiError(409, "Obstacle is already being removed")
            self.deleting.add(name)
            item = self.items[name]
        try:
            from ros_gz_interfaces.srv import DeleteEntity
            request = DeleteEntity.Request()
            request.entity.name, request.entity.type = name, 2
            self.call(self.remove, request)
            with self.lock:
                self.record(item, "removed")
                del self.items[name]
            return {"removed": name}
        finally:
            with self.lock:
                self.deleting.discard(name)

    def tick(self):
        with self.lock:
            for name, item in list(self.items.items()):
                if (
                    item["kind"] != "moving"
                    or name in self.pending
                    or name in self.deleting
                    or item["error"]
                ):
                    continue
                self.pending.add(name)
                point = position_at(item, self.now() - item["started_sim_s"])
                from ros_gz_interfaces.srv import SetEntityPose
                request = SetEntityPose.Request()
                request.entity.name, request.entity.type = name, 2
                request.pose.position.x, request.pose.position.y = (
                    float(point["x"]),
                    float(point["y"]),
                )
                request.pose.orientation.w = 1.0
                future = self.move.call_async(request)

                def done(future, name=name, item=item, point=point):
                    with self.lock:
                        self.pending.discard(name)
                        if self.items.get(name) is not item:
                            return
                        try:
                            if not future.result().success:
                                raise RuntimeError("Gazebo rejected motion")
                            item["position"] = point
                            self.record(item, "moved")
                        except Exception as error:
                            item["error"] = str(error)
                            self.node.get_logger().error(f"Obstacle {name}: {error}")

                future.add_done_callback(done)

    def close(self):
        self.timer.cancel()
        self.log.close()
