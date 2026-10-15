import json
import math
from pathlib import Path
import tempfile
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np

from outdoornav.api import serve
from outdoornav.bundle import is_free, load_bundle, masks, route_mask
from outdoornav.diagnostics import load_grid, seq_of
from outdoornav.routing import plan_route, validate_point
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
