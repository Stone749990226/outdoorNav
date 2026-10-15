import json
import math
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import xml.etree.ElementTree as ET

from outdoornav.api import serve
from outdoornav.obstacles import box_sdf, obstacle_spec, position_at


class ObstacleTests(unittest.TestCase):
    def spec(self, **kwargs):
        return obstacle_spec(
            dict(
                name="test_case",
                kind="moving",
                start={"x": 0, "y": 0},
                end={"x": 2, "y": 0},
                speed_mps=0.5,
                **kwargs,
            )
        )

    def test_motion_is_continuous_and_reverses_at_endpoints(self):
        spec = self.spec()
        for seconds, x in ((0, 0), (2, 1), (4, 2), (6, 1), (8, 0), (10, 1)):
            self.assertEqual(position_at(spec, seconds), {"x": x, "y": 0})
        self.assertLess(
            math.dist(
                tuple(position_at(spec, 3.999).values()),
                tuple(position_at(spec, 4.001).values()),
            ),
            0.002,
        )
        static = obstacle_spec({"name": "test_static", "start": {"x": 3, "y": 4}})
        self.assertEqual(position_at(static, 999), {"x": 3, "y": 4})

    def test_rejects_unsafe_input_and_keeps_real_collision(self):
        valid = dict(
            name="test_box", kind="moving", start={"x": 0, "y": 0}, end={"x": 1, "y": 0}
        )
        for change in (
            {"name": "robot_rtk"},
            {"speed_mps": math.nan},
            {"width": True},
            {"height": -1},
            {"end": {"x": 0, "y": 0}},
            {"kind": "unknown"},
        ):
            with self.assertRaises(ValueError):
                obstacle_spec({**valid, **change})
        model = ET.fromstring(box_sdf(obstacle_spec(valid))).find("model")
        self.assertEqual(
            model.find("link/collision/geometry/box/size").text, "0.6 0.6 0.8"
        )
        self.assertIsNotNone(model.find("link/visual"))

    def test_http_add_list_remove_and_version_validation(self):
        class Obstacles:
            items = {}

            def snapshot(self):
                return list(self.items.values())

            def add(self, payload):
                spec = obstacle_spec(payload)
                self.items[spec["name"]] = spec
                return spec

            def delete(self, name):
                return {"removed": self.items.pop(name)["name"]}

        class Backend:
            bundle = {"scene_version": "scene", "route_version": "route"}
            obstacles = Obstacles()

        server = serve(Backend(), 0)
        base = f"http://127.0.0.1:{server.server_port}/api/obstacles"
        payload = dict(
            self.spec(), frame_id="map", scene_version="scene", route_version="route"
        )

        def post(body):
            return urlopen(
                Request(
                    base,
                    data=json.dumps(body).encode(),
                    headers={"Content-Type": "application/json"},
                )
            )

        try:
            with post(payload) as response:
                self.assertEqual(response.status, 201)
            with urlopen(base) as response:
                self.assertEqual(len(json.load(response)["obstacles"]), 1)
            with self.assertRaises(HTTPError) as error:
                post({**payload, "scene_version": "stale"})
            self.assertEqual(error.exception.code, 409)
            with urlopen(Request(base + "/test_case", method="DELETE")) as response:
                self.assertEqual(json.load(response), {"removed": "test_case"})
            with urlopen(base) as response:
                self.assertEqual(json.load(response)["obstacles"], [])
        finally:
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
