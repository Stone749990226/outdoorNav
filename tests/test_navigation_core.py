import asyncio
from contextlib import ExitStack
import importlib.util
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, Mock, patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np

from outdoornav.api import serve
from outdoornav.bundle import is_free, load_bundle, masks, route_mask
from outdoornav.diagnostics import load_grid, seq_of
from outdoornav.routing import RouteGuidance, plan_route, validate_point
from outdoornav.tasks import TaskStore, ProgressBudget


class RoutingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle = load_bundle()
        cls.network = cls.bundle["network"]
        cls.free = masks()

    def test_same_segment_midpoints_and_reverse(self):
        road = self.network["roads"][2]
        a, b = road["points"][:2]
        mid = lambda t: {k: a[k] + t * (b[k] - a[k]) for k in ("x", "y")}
        route = plan_route(self.network, mid(0.2), mid(0.8))
        self.assertAlmostEqual(
            route["length_m"],
            math.dist(tuple(mid(0.2).values()), tuple(mid(0.8).values())),
        )
        self.assertEqual(route["road_ids"], ["branch"])
        reverse = plan_route(self.network, mid(0.8), mid(0.2))
        self.assertAlmostEqual(reverse["length_m"], route["length_m"])

    def test_junction_closed_arc_and_disconnected_branch(self):
        nodes = {n["id"]: n for n in self.network["nodes"]}
        direct = plan_route(self.network, nodes["S"], nodes["J"])
        detour = plan_route(self.network, nodes["S"], nodes["J"], ["loop-a"])
        self.assertGreater(detour["length_m"], direct["length_m"])
        self.assertEqual(detour["road_ids"], ["loop-b"])
        with self.assertRaises(ValueError):
            plan_route(self.network, nodes["S"], nodes["P"], ["branch"])

    def test_actual_survey_geometry_and_quality_equal_baseline(self):
        root = Path(__file__).resolve().parents[1]
        baseline = json.loads(
            (root / "artifacts/baseline/road_network.json").read_text()
        )
        self.assertEqual(self.network, baseline)
        self.assertEqual(
            self.bundle["survey_report"]["logs"]["loop"]["quality_counts"], {"2": 2135}
        )

    def test_robot_in_junction_arrival_neighborhood_can_enter_open_arc(self):
        road = self.network["roads"][0]
        a, b = road["points"][:2]
        length = math.dist((a["x"], a["y"]), (b["x"], b["y"]))
        start = {key: a[key] + 0.12 / length * (b[key] - a[key]) for key in ("x", "y")}
        goal = next(node for node in self.network["nodes"] if node["id"] == "J")
        route = plan_route(self.network, start, goal, ["loop-a"])
        self.assertEqual(route["road_ids"], ["loop-b"])
        allowed = route_mask(self.bundle, self.free, route, start) < 100
        self.assertTrue(is_free(allowed, self.bundle["grid"], start))
        # A goal on the closed arc still cannot silently move to another road.
        with self.assertRaises(ValueError):
            plan_route(self.network, goal, start, ["loop-a"])

    def test_road_mask_allows_detour_without_erasing_static_obstacles(self):
        road = self.network["roads"][0]
        a, b = road["points"][:2]
        route = plan_route(self.network, a, b)
        mask = route_mask(self.bundle, self.free, route, a)
        self.assertEqual(mask.dtype, np.int8)
        self.assertTrue(np.all(mask[~self.free] == 100))
        self.assertGreater(np.count_nonzero(mask < 100), 20)
        self.assertTrue(is_free(mask < 100, self.bundle["grid"], a))
        self.assertTrue(np.any((mask > 0) & (mask < 100)))
        shifted_start = {"x": a["x"] + 0.5, "y": a["y"]}
        self.assertTrue(
            np.array_equal(
                mask, route_mask(self.bundle, self.free, route, shifted_start)
            )
        )
        with self.assertRaises(ValueError):
            route_mask(self.bundle, self.free, route, {"x": 40, "y": -25})

    def test_bundle_detects_modified_assets(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bundle.json"
            path.write_text(
                json.dumps({**self.bundle, "files": {"mask.bin": "incorrect"}})
            )
            (Path(tmp) / "mask.bin").write_bytes(b"changed")
            with self.assertRaisesRegex(ValueError, "changed"):
                load_bundle(path)

    def test_bad_points_and_unknown_roads_rejected(self):
        for value in ({"x": True, "y": 1}, {"x": math.nan, "y": 0}, None):
            with self.assertRaises(ValueError):
                validate_point(value)
        with self.assertRaises(ValueError):
            plan_route(
                self.network,
                self.network["nodes"][0],
                self.network["nodes"][1],
                ["missing"],
            )


class RouteGuidanceTests(unittest.TestCase):
    @staticmethod
    def point(x, y=0):
        return {"x": x, "y": y}

    def guide(self, coordinates):
        points = [self.point(*p) for p in coordinates]
        return RouteGuidance(points, points[0], points[-1])

    def test_sparse_L_retains_bend_and_bounds_legs(self):
        guide = self.guide([(0, 0), (10, 0), (10, 10)])
        self.assertIn(self.point(10, 0), guide.points)
        self.assertLessEqual(max(math.dist(tuple(a.values()), tuple(b.values())) for a, b in zip(guide.points, guide.points[1:])), 2.0)
        window = guide.window(self.point(0))
        self.assertEqual([guide.points[i] for i in window], [self.point(x) for x in (2, 4, 6)])
        guide.window(self.point(8.5))
        # The near corner remains in the path, not a chord to the final target.
        window = guide.window(self.point(8.5))
        self.assertEqual(guide.points[window[0]], self.point(10))
        self.assertEqual(guide.points[window[1]], self.point(10, 2))

    def test_U_with_close_parallel_arm_never_jumps_to_nearby_final(self):
        guide = self.guide([(0, 0), (10, 0), (10, 0.4), (0, 0.4)])
        window = guide.window(self.point(0, 0.4))
        self.assertEqual(guide.index, 1)
        self.assertFalse(guide.final_leg)
        self.assertTrue(all(guide.points[i]["y"] == 0 for i in window))
        for point in guide.points[1:-1]:
            guide.window(point)
        self.assertTrue(guide.final_leg)
        self.assertEqual(guide.points[-1], self.point(0, 0.4))

    def test_dense_line_duplicates_and_reversal_have_independent_progress(self):
        dense = [self.point(i / 100) for i in range(1001)]
        guide = RouteGuidance(dense + [dense[-1]], dense[0], dense[-1])
        self.assertEqual(len(guide.points), 6)
        guide.window(self.point(6.5))
        index = guide.index
        guide.window(self.point(0))
        self.assertEqual(guide.index, index)  # Localization jitter cannot rewind.
        reverse = RouteGuidance(list(reversed(dense)), dense[-1], dense[0])
        self.assertEqual(reverse.index, 1)
        self.assertEqual(reverse.points[reverse.window(dense[-1])[0]], self.point(8))
        self.assertEqual(reverse.points[-1], dense[0])

    def test_actual_start_and_exact_goal_and_zero_length(self):
        start, end = self.point(-3, 1), self.point(8, 0.12)
        guide = RouteGuidance([self.point(0), self.point(8)], start, end)
        self.assertEqual(guide.points[0], start)
        self.assertEqual(guide.points[-1], end)
        one = RouteGuidance([end, end], end, end)
        self.assertEqual(one.window(end), (0,))
        self.assertTrue(one.final_leg)

    def test_window_advances_early_but_is_bounded_for_dense_bends(self):
        guide = self.guide([(0, 0), (20, 0)])
        before = guide.window(self.point(0))
        after = guide.window(self.point(1.5))
        self.assertNotEqual(before, after)
        self.assertEqual(guide.index, 2)  # 0.5 m BEFORE reaching the first station.
        self.assertGreater(guide.points[after[-1]]["x"] - 1.5, 6)
        zigzag = self.guide([(i / 10, i % 2) for i in range(100)])
        self.assertLessEqual(len(zigzag.window(self.point(0))), 12)


class SensorOnlyNavigationTests(unittest.TestCase):
    """Exercise ROS consumers without a graph, simulator or live service calls."""

    @classmethod
    def setUpClass(cls):
        if importlib.util.find_spec("rclpy") is None:
            raise unittest.SkipTest("ROS environment is not sourced")
        from outdoornav_ros import navigation

        cls.navigation = navigation

    def setUp(self):
        nav = self.navigation
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(nav.Node, "__init__", return_value=None))
        for name in (
            "declare_parameter", "get_parameter", "get_clock", "create_publisher",
            "create_subscription", "create_client", "create_timer", "get_logger",
        ):
            self.stack.enter_context(patch.object(nav.Navigation, name))
        for name in ("Clock", "Buffer", "TransformListener", "ActionClient", "ActionServer", "Obstacles", "serve"):
            self.stack.enter_context(patch.object(nav, name))
        bundle_path = Path(__file__).resolve().parents[1] / "scenes/north_campus/generated/bundle.json"
        nav.Navigation.get_parameter.side_effect = lambda name: SimpleNamespace(
            value=str(bundle_path) if name == "bundle" else ""
        )
        from builtin_interfaces.msg import Time

        nav.Navigation.get_clock.return_value.now.return_value = SimpleNamespace(
            nanoseconds=10_000_000_000, to_msg=lambda: Time(sec=10)
        )
        self.publishers = {}

        def publisher(message_type, topic, qos):
            result = Mock()
            self.publishers[topic] = (message_type, result)
            return result

        nav.Navigation.create_publisher.side_effect = publisher
        bundle = load_bundle()
        self.position = dict(x=bundle["spawn"][0], y=bundle["spawn"][1], yaw=0, age_s=0)
        self.stack.enter_context(patch.object(nav.Navigation, "pose", return_value=self.position))
        self.stack.enter_context(patch.object(nav.Navigation, "status", return_value={"ready": True, "pose": self.position}))
        self.node = nav.Navigation()

    def test_initialization_publishes_only_sensor_prediction_not_scene_or_roads(self):
        nav = self.navigation
        self.assertEqual(
            set(self.publishers),
            {"/cmd_vel", "/navigation/execution_path", "/navigation/predicted_obstacles", "/navigation/prediction_info"},
        )
        grids = [(topic, publisher) for topic, (kind, publisher) in self.publishers.items() if kind is nav.OccupancyGrid]
        self.assertEqual([topic for topic, _ in grids], ["/navigation/predicted_obstacles"])
        # No observations means no prediction, regardless of known scene geometry.
        values = grids[0][1].publish.call_args.args[0].data
        self.assertTrue(values)
        self.assertEqual(set(values), {-1})
        self.assertTrue(np.any(~self.node.free))
        info = self.publishers["/navigation/prediction_info"][1].publish.call_args.args[0]
        self.assertEqual(info.filter_mask_topic, "/navigation/predicted_obstacles")

    def test_grid_messages_preserve_unknown_and_observed_cells_without_scene_prefill(self):
        from outdoornav_ros.maps import Maps

        # A message-only Maps constructor must not create publishers or timers.
        node = Mock()
        node.get_clock.return_value = self.node.get_clock()
        maps = Maps(node, self.node.bundle)
        self.assertEqual(node.mock_calls, [])
        grid = dict(width=3, height=2, origin=[-1.0, 2.0], resolution=0.1)
        values = np.array([[-1, 0, 100], [-1, 45, -1]], dtype=np.int8)
        message = maps.message(values, grid)
        self.assertEqual(list(message.data), [-1, 0, 100, -1, 45, -1])
        self.assertEqual((message.info.width, message.info.height), (3, 2))
        node.create_publisher.assert_not_called()
        node.create_timer.assert_not_called()

    def test_preview_and_submission_retain_reference_routing_without_map_writes(self):
        goal = next(n for n in self.node.bundle["network"]["nodes"] if n["id"] == "J")
        payload = dict(
            frame_id="map", goal=goal, blocked_road_ids=["loop-a"],
            scene_version=self.node.bundle["scene_version"],
            route_version=self.node.bundle["route_version"],
        )
        before = {topic: publisher.publish.call_count for topic, (_, publisher) in self.publishers.items()}
        route = self.node.preview(payload)
        self.assertNotIn("loop-a", route["road_ids"])
        self.assertTrue(route["reference_path"])
        task = self.node.submit(payload)
        self.assertEqual(task["reference_path"], route["reference_path"])
        self.assertEqual(before, {topic: publisher.publish.call_count for topic, (_, publisher) in self.publishers.items()})

    def test_execution_plans_on_unknown_without_removed_map_acknowledgement(self):
        nav = self.navigation
        node = self.node
        goal = node.bundle["network"]["roads"][0]["points"][1]
        task = node.tasks.submit(goal)
        task_id = task["task_id"]
        handle = Mock(goal_id=SimpleNamespace(uuid=bytes.fromhex(task_id)), is_cancel_requested=True)
        unknown = node.maps.message(np.full(node.free.shape, -1, dtype=np.int8))
        node.global_map = node.local_map = unknown
        original = unknown.data[:]
        node.maps = SimpleNamespace(message=node.maps.message)
        self.install_planner()
        follower = Mock(accepted=True)
        follower.get_result_async.return_value.done.return_value = False
        node.follow_client = Mock()
        node.follow_client.send_goal_async.return_value = Mock(done=lambda: True, result=lambda: follower)
        node.release_controller = AsyncMock(return_value=True)
        node.confirm_stop = AsyncMock(return_value=True)

        async def pause(*args):
            node.tasks.cancel(task_id)

        node.pause = pause
        asyncio.run(node.execute(handle))
        self.assertGreater(node.path_client.send_goal_async.call_count, 1)
        node.follow_client.send_goal_async.assert_called_once()
        state = node.tasks.snapshot(task_id)
        self.assertEqual(state["path_revision"], 1)
        self.assertTrue(state["reference_path"])
        points = state["execution_path"]
        self.assertEqual(points[0], {key: self.position[key] for key in ("x", "y")})
        self.assertNotEqual(points[-1], goal)  # A rolling horizon, not a shortcut.
        self.assertEqual(state["state"], "canceled")
        self.assertEqual(unknown.data, original)
        self.assertFalse(node.path_obstructed(state["execution_path"], self.position))
        unknown.data[0] = 100  # Only sensed occupied cells, not unknown, block a path.
        unknown.info.origin.position.x = self.position["x"]
        unknown.info.origin.position.y = self.position["y"]
        self.assertTrue(node.path_obstructed(state["execution_path"], self.position))

    def test_retained_app_still_gates_motion_on_task_and_readiness(self):
        node, nav = self.node, self.navigation
        command = nav.Twist()
        command.linear.x = 0.5
        for ready, stopping, state, moves in (
            (True, False, "navigating", True),
            (False, False, "navigating", False),
            (True, True, "navigating", False),
            (True, False, "succeeded", False),
            (True, False, "failed", False),
        ):
            with self.subTest(ready=ready, stopping=stopping, state=state):
                task = node.tasks.submit(dict(x=1., y=0.))
                node.tasks.update(task["task_id"], state=state)
                node.owner, node.stopping = task["task_id"], stopping
                node.status.return_value = {"ready": ready}
                node.raw_command, node.raw_received = command, nav.time.monotonic()
                node.safety_tick()
                sent = node.command_pub.publish.call_args.args[0]
                self.assertEqual(sent.linear.x, 0.5 if moves else 0.)
                self.assertEqual(sent.angular.z, 0.)

    def test_release_reuses_terminal_futures_across_many_replans(self):
        requests, results = [], {}
        for index in range(250):
            follower = Mock(accepted=True)
            follower.get_result_async.side_effect = AssertionError("must not requery old goals")
            pending = Mock(done=lambda: True, result=lambda f=follower: f)
            result = Mock(done=lambda: True)
            result.result.return_value.status = 4 if index == 249 else 6
            requests.append(pending)
            results[pending] = result
        self.assertTrue(asyncio.run(self.node.release_controller(requests, results)))
        # finally() can safely run cleanup again without another action request.
        self.assertTrue(asyncio.run(self.node.release_controller(requests, results)))
        for pending in requests:
            pending.result().get_result_async.assert_not_called()
            pending.result().cancel_goal_async.assert_not_called()

    def test_release_waits_for_late_acceptance_and_cancels_active_goal_once(self):
        from rclpy.task import Future

        accepted, result = Future(), Future()
        follower = Mock(accepted=True)
        follower.get_result_async.return_value = result
        follower.cancel_goal_async.side_effect = lambda: result.set_result(SimpleNamespace(status=5))

        async def pause(*args):
            accepted.set_result(follower)

        self.node.pause = pause
        cache = {}
        self.assertTrue(asyncio.run(self.node.release_controller([accepted], cache)))
        self.assertTrue(asyncio.run(self.node.release_controller([accepted], cache)))
        follower.get_result_async.assert_called_once()
        follower.cancel_goal_async.assert_called_once()

    def test_release_does_not_turn_unknown_or_exception_into_success(self):
        follower = Mock(accepted=True)
        pending = Mock(done=lambda: True, result=lambda: follower)
        result = Mock(done=lambda: True)
        result.result.return_value.status = 0
        self.assertFalse(asyncio.run(self.node.release_controller([pending], {pending: result})))
        result.result.side_effect = RuntimeError("result transport failed")
        self.assertFalse(asyncio.run(self.node.release_controller([pending], {pending: result})))
        follower.cancel_goal_async.assert_not_called()

    def test_release_timeout_keeps_ownership_unconfirmed(self):
        now = [0.]
        pending = Mock(done=lambda: False)

        async def pause(*args):
            now[0] += 2.

        self.node.pause = pause
        with patch.object(self.navigation.time, "monotonic", side_effect=lambda: now[0]):
            self.assertFalse(asyncio.run(self.node.release_controller([pending], {})))
        pending.result.assert_not_called()

    def install_planner(self, fail=None, detour=None):
        nav = self.navigation
        self.node.path_client = Mock()

        def send(request):
            target = request.goal.pose.position
            start = request.start.pose.position
            failed = fail is not None and fail(target)
            points = [(start.x, start.y)]
            if detour is not None:
                points.extend(detour(start, target))
            points.append((target.x, target.y))
            path = nav.Path()
            for x, y in points:
                pose = nav.PoseStamped()
                pose.pose.position.x, pose.pose.position.y = x, y
                path.poses.append(pose)
            result = SimpleNamespace(
                status=nav.GoalStatus.STATUS_ABORTED if failed else nav.GoalStatus.STATUS_SUCCEEDED,
                result=SimpleNamespace(path=path),
            )
            planner = Mock(accepted=True)
            planner.get_result_async.return_value = Mock(done=lambda: True, result=lambda: result)
            return Mock(done=lambda: True, result=lambda: planner)

        self.node.path_client.send_goal_async.side_effect = send

    def planned_window(self, coordinates, fail=None, detour=None):
        points = [dict(x=x, y=y) for x, y in coordinates]
        task = self.node.tasks.submit(points[-1])
        guide = RouteGuidance(points, points[0], points[-1])
        self.install_planner(fail, detour)
        path = asyncio.run(self.node.guided_path(guide, guide.window(points[0]), points[0], task["task_id"]))
        return path, guide

    def test_guided_NavFn_legs_follow_L_and_preserve_obstacle_detours(self):
        path, guide = self.planned_window([(0, 0), (4, 0), (4, 4)], detour=lambda a, b: [((a.x + b.x) / 2, (a.y + b.y) / 2 + 0.3)])
        requests = [call.args[0] for call in self.node.path_client.send_goal_async.call_args_list]
        targets = [(r.goal.pose.position.x, r.goal.pose.position.y) for r in requests]
        self.assertEqual(targets, [(2, 0), (4, 0), (4, 2)])
        self.assertTrue(all(r.use_start for r in requests))
        for a, b in zip(requests, requests[1:]):
            self.assertEqual(a.goal.pose.position, b.start.pose.position)
        self.assertEqual(len(path.poses), 7)  # 3 detours retained in one RPP path.
        self.assertIn((4, 0), [(p.pose.position.x, p.pose.position.y) for p in path.poses])
        self.assertEqual(path.poses[1].pose.position.y, 0.3)

    def test_occupied_straight_station_can_be_bypassed_but_never_a_bend(self):
        path, _ = self.planned_window([(0, 0), (10, 0)], fail=lambda p: p.x == 2)
        self.assertIsNotNone(path)
        requests = [call.args[0] for call in self.node.path_client.send_goal_async.call_args_list]
        self.assertEqual(requests[1].start.pose.position.x, 0)
        self.assertEqual(requests[1].goal.pose.position.x, 4)
        path, _ = self.planned_window([(0, 0), (4, 0), (4, 4)], fail=lambda p: p.x == 4 and p.y == 0)
        self.assertIsNone(path)
        self.assertEqual(self.node.path_client.send_goal_async.call_count, 2)

    def test_final_guided_target_is_the_exact_requested_goal(self):
        path, _ = self.planned_window([(0, 0), (2, 0), (2, 0.12)])
        end = path.poses[-1].pose.position
        self.assertEqual((end.x, end.y), (2, 0.12))

    def test_segment_update_bypasses_jitter_gate_and_preempted_result_is_ignored(self):
        nav, node = self.navigation, self.node
        points = [dict(x=0., y=0.), dict(x=10., y=0.)]
        self.position.update(x=0., y=0.)
        task = node.tasks.submit(points[-1])
        task_id = task["task_id"]
        handle = Mock(goal_id=SimpleNamespace(uuid=bytes.fromhex(task_id)), is_cancel_requested=True)
        self.install_planner()
        followers, results = [], []

        def follow(command):
            if results:
                results[-1].done.return_value = True
                results[-1].result.return_value.status = nav.GoalStatus.STATUS_CANCELED
            follower = Mock(accepted=True)
            result = Mock()
            result.done.return_value = False
            follower.get_result_async.return_value = result
            followers.append(follower)
            results.append(result)
            return Mock(done=lambda: True, result=lambda: follower)

        node.follow_client = Mock()
        node.follow_client.send_goal_async.side_effect = follow
        node.release_controller = AsyncMock(return_value=True)
        node.confirm_stop = AsyncMock(return_value=True)

        async def pause(*args):
            if len(followers) == 1:
                self.position["x"] = 1.5
            else:
                node.tasks.cancel(task_id)

        node.pause = pause
        with patch.object(nav, "plan_route", return_value=dict(reference_path=points)):
            asyncio.run(node.execute(handle))
        self.assertEqual(len(followers), 2)  # Sim clock unchanged; no 1 s / 6 s wait.
        self.assertEqual(node.tasks.snapshot(task_id)["path_revision"], 2)
        self.assertEqual(node.tasks.snapshot(task_id)["state"], "canceled")
        self.assertEqual(len(node.release_controller.call_args.args[0]), 2)
        self.assertIsNone(node.owner)
        self.assertTrue(node.stopping)
        self.assertNotIn(task_id, node.requests)
        self.assertNotIn(task_id, node.handles)
        self.assertNotIn(task_id, node.cancel_sent)

    def execute_fixture(self, points, goal=None):
        nav, node = self.navigation, self.node
        points = [dict(x=float(x), y=float(y)) for x, y in points]
        self.position.update(points[0])
        task = node.tasks.submit(goal or points[-1])
        task_id = task["task_id"]
        handle = Mock(goal_id=SimpleNamespace(uuid=bytes.fromhex(task_id)), is_cancel_requested=False)
        self.install_planner()
        self.result = Mock()
        self.result.done.return_value = False
        follower = Mock(accepted=True)
        follower.get_result_async.return_value = self.result
        node.follow_client = Mock()
        node.follow_client.send_goal_async.return_value = Mock(done=lambda: True, result=lambda: follower)
        node.release_controller = AsyncMock(return_value=True)
        node.confirm_stop = AsyncMock(return_value=True)
        self.stack.enter_context(patch.object(nav, "plan_route", return_value=dict(reference_path=points)))
        return task_id, handle

    def test_nearby_final_on_U_cannot_complete_before_traversing_reference(self):
        node = self.node
        task_id, handle = self.execute_fixture([(0, 0), (10, 0), (10, 0.1), (0, 0.1)])

        async def pause(*args):
            node.tasks.cancel(task_id)

        node.pause = pause
        asyncio.run(node.execute(handle))
        node.follow_client.send_goal_async.assert_called_once()
        handle.succeed.assert_not_called()
        self.assertEqual(node.tasks.snapshot(task_id)["state"], "canceled")
        # Remaining distance includes the U-shaped road rather than the nearby goal shortcut.
        self.assertAlmostEqual(handle.publish_feedback.call_args.args[0].distance_remaining, 20.1)

    def test_only_final_goal_completes_after_stop_confirmation(self):
        node, nav = self.node, self.navigation
        final = dict(x=2., y=0.12)
        task_id, handle = self.execute_fixture([(0, 0), (2, 0)], goal=final)

        async def pause(*args):
            self.position.update(final)
            self.result.done.return_value = True
            self.result.result.return_value.status = nav.GoalStatus.STATUS_SUCCEEDED

        node.pause = pause
        asyncio.run(node.execute(handle))
        state = node.tasks.snapshot(task_id)
        self.assertEqual(state["state"], "succeeded")
        self.assertEqual(state["distance_remaining_m"], 0.)
        self.assertEqual(state["completion_pose"]["y"], final["y"])
        node.confirm_stop.assert_awaited_once()
        handle.succeed.assert_called_once()
        sent = node.follow_client.send_goal_async.call_args.args[0].path.poses[-1].pose.position
        self.assertEqual((sent.x, sent.y), (2., 0.12))

    def test_failed_final_stop_is_not_reported_as_success(self):
        node, nav = self.node, self.navigation
        task_id, handle = self.execute_fixture([(0, 0), (2, 0)])
        node.confirm_stop = AsyncMock(return_value=False)

        async def pause(*args):
            self.position.update(x=2., y=0.)
            self.result.done.return_value = True
            self.result.result.return_value.status = nav.GoalStatus.STATUS_SUCCEEDED

        node.pause = pause
        asyncio.run(node.execute(handle))
        self.assertEqual(node.tasks.snapshot(task_id)["state"], "failed")
        self.assertIn("Could not confirm", node.stop_failure)
        handle.succeed.assert_not_called()

    def test_current_controller_abort_still_uses_failure_budget(self):
        node, nav = self.node, self.navigation
        task_id, handle = self.execute_fixture([(0, 0), (10, 0)])
        stamp = [10.]
        from builtin_interfaces.msg import Time
        node.get_clock.return_value.now.side_effect = lambda: SimpleNamespace(
            nanoseconds=int(stamp[0] * 1e9), to_msg=lambda: Time(sec=int(stamp[0]))
        )

        async def pause(*args):
            stamp[0] += 10
            self.result.done.return_value = True
            self.result.result.return_value.status = nav.GoalStatus.STATUS_ABORTED

        node.pause = pause
        asyncio.run(node.execute(handle))
        self.assertEqual(node.tasks.snapshot(task_id)["state"], "failed")
        self.assertIn("Controller could not make progress", node.tasks.snapshot(task_id)["reason"])
        handle.succeed.assert_not_called()

    def test_effective_launch_configuration_has_no_scene_or_road_costmap_layers(self):
        from launch import LaunchContext
        import yaml

        root = Path(__file__).resolve().parents[1]
        share = root / "ros_ws/src/outdoornav_ros"
        spec = importlib.util.spec_from_file_location("navigation_launch_test", share / "launch/navigation.launch.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            context = LaunchContext()
            context.launch_configurations.update(
                bundle=str(root / "scenes/north_campus/generated/bundle.json"), run_dir=directory
            )
            with patch.object(module, "get_package_share_directory", return_value=str(share)), patch.object(module, "Node") as factory:
                module.nodes(context)
            params = yaml.safe_load((Path(directory) / "navigation.yaml").read_text())
            self.assertNotIn("nav2_map_server", [call.kwargs["package"] for call in factory.call_args_list])
        for name in ("global_costmap", "local_costmap"):
            costmap = params[name][name]["ros__parameters"]
            self.assertTrue(costmap["track_unknown_space"])
            self.assertEqual(costmap["plugins"], ["obstacle_layer", "inflation_layer"])
            self.assertEqual(costmap["obstacle_layer"]["observation_sources"].split(), ["camera_cloud", "scan"])
            for source in ("camera_cloud", "scan"):
                self.assertTrue(costmap["obstacle_layer"][source]["clearing"])
                self.assertTrue(costmap["obstacle_layer"][source]["marking"])
            # Local filters are absent: rcl rejects an untyped `filters: []`.
            # Costmap2DROS declares an empty string-vector default instead.
            self.assertEqual(costmap.get("filters", []), ["motion_prediction"] if name == "global_costmap" else [])
            if name == "local_costmap":
                self.assertNotIn("filters", costmap)
            for value in costmap.values():
                if isinstance(value, dict) and "plugin" in value:
                    self.assertIn(value["plugin"], {"nav2_costmap_2d::ObstacleLayer", "nav2_costmap_2d::InflationLayer", "nav2_costmap_2d::KeepoutFilter"})
            if name == "global_costmap":
                self.assertEqual(costmap["motion_prediction"]["filter_info_topic"], "/navigation/prediction_info")
        self.assertTrue(params["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"])


class TaskTests(unittest.TestCase):
    def test_evicted_task_callback_is_ignored(self):
        store = TaskStore()
        first = store.submit({"x": 0, "y": 0})
        store.update(first["task_id"], state="succeeded")
        for index in range(201):
            task = store.submit({"x": index, "y": 0})
            store.update(task["task_id"], state="succeeded")
        with self.assertRaises(KeyError):
            store.snapshot(first["task_id"])
        self.assertFalse(store.update(first["task_id"], state="navigating"))
        self.assertEqual(store.snapshot()["task_id"], task["task_id"])

    def test_replacement_and_late_results_cannot_overwrite_new_task(self):
        store = TaskStore()
        first = store.submit({"x": 1, "y": 2})
        second = store.submit({"x": 3, "y": 4})
        self.assertTrue(store.snapshot(first["task_id"])["cancel_requested"])
        self.assertFalse(store.update(first["task_id"], state="navigating"))
        store.update(first["task_id"], state="canceled")
        self.assertFalse(store.update(first["task_id"], state="succeeded"))
        self.assertEqual(store.snapshot()["task_id"], second["task_id"])
        self.assertEqual(store.snapshot()["state"], "accepted")

    def test_cancel_idempotent_and_snapshots_detached(self):
        store = TaskStore()
        task = store.submit({"x": 1, "y": 2})
        task["goal"]["x"] = 999
        self.assertEqual(store.snapshot()["goal"]["x"], 1)
        store.cancel(task["task_id"])
        store.cancel(task["task_id"])
        self.assertEqual(store.snapshot()["state"], "canceling")
        pose = {"frame_id": "map", "x": 1, "y": 2, "stamp_s": 12.5}
        store.update(task["task_id"], state="canceled", completion_pose=pose)
        pose["x"] = 999
        completed = store.snapshot()
        completed["completion_pose"]["x"] = -999
        self.assertEqual(store.snapshot()["completion_pose"]["x"], 1)
        self.assertFalse(store.update(task["task_id"], completion_pose=pose))


class ProgressTests(unittest.TestCase):
    def test_successful_plans_reset_transient_failures_on_long_routes(self):
        budget = ProgressBudget()
        for _ in range(20):
            self.assertTrue(budget.planned(False))
            self.assertTrue(budget.planned(True))
        self.assertTrue(budget.planned(False))
        self.assertTrue(budget.planned(False))
        self.assertFalse(budget.planned(False))

    def test_controller_retry_requires_actual_progress(self):
        budget = ProgressBudget()
        budget.moved(0, 0, 0)
        self.assertTrue(budget.controller_ended(1))
        budget.planned(True)
        budget.moved(0.1, 0, 10)
        self.assertTrue(budget.controller_ended(10))
        self.assertTrue(budget.controller_ended(11))
        self.assertFalse(budget.controller_ended(21))
        budget.moved(0.6, 0, 22)
        self.assertTrue(budget.controller_ended(22))


class ProbeTests(unittest.TestCase):
    def test_numeric_order_and_paired_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            for seq in (2, 10):
                (Path(tmp) / f"map_{seq}.json").write_text(
                    json.dumps({"width": 1, "height": 1})
                )
                (Path(tmp) / f"map_{seq}.bin").write_bytes(bytes([seq]))
            (Path(tmp) / "map_11.json").write_text("{}")
            self.assertEqual(load_grid(tmp)[1], bytes([10]))
        self.assertEqual(seq_of("tick_0010.json"), 10)


class TransportTests(unittest.TestCase):
    def test_goal_contract_and_unknown_endpoints(self):
        class Backend:
            tasks = TaskStore()

            def status(self):
                return {"ready": True, "schema_version": 1}

            def map_snapshot(self):
                return {"frame_id": "map"}

            def preview(self, body):
                return {"goal": validate_point(body["goal"])}

            def submit(self, body):
                return self.tasks.submit(validate_point(body["goal"]))

        server = serve(Backend(), port=0)
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            request = Request(
                base + "/api/tasks",
                data=json.dumps({"goal": {"x": 1, "y": 2}}).encode(),
                headers={"Content-Type": "application/json"},
            )
            with urlopen(request) as response:
                task = json.load(response)
                self.assertEqual(response.status, 202)
            with urlopen(base + "/api/tasks/" + task["task_id"]) as response:
                self.assertEqual(json.load(response)["state"], "accepted")
            with self.assertRaises(HTTPError) as error:
                urlopen(base + "/api/not-an-endpoint")
            self.assertEqual(error.exception.code, 404)
        finally:
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
