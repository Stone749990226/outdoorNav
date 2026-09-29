import unittest

from outdoor_nav_bridge.bridge import dense_points, validate_route


CALIBRATION = {"version": "survey-test", "map_version": "measured-test", "status": "survey_dgps_unverified",
               "origin_wgs84": {"latitude": 30.2, "longitude": 120.2}}
ROUTE = {
    "frame_id": "map",
    "calibration_version": CALIBRATION["version"],
    "calibration_status": CALIBRATION["status"],
    "map_version": CALIBRATION["map_version"],
    "origin_wgs84": CALIBRATION["origin_wgs84"],
    "executable_on_robot": False,
    "path": [{"x": 0, "y": 0}, {"x": 0, "y": -3}],
    "waypoints": [{"x": 0, "y": 0, "yaw": -1.57}, {"x": 0, "y": -3, "yaw": -1.57}],
}


class BridgeValidationTests(unittest.TestCase):
    def test_accepts_simulation_route(self):
        validate_route(ROUTE, CALIBRATION)

    def test_rejects_wrong_frame_and_version(self):
        for key, value in (("frame_id", "demo_map"), ("calibration_version", "old"), ("map_version", "old"),
                           ("calibration_status", "provisional"), ("origin_wgs84", {"latitude": 0, "longitude": 0})):
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_route({**ROUTE, key: value}, CALIBRATION)

    def test_rejects_robot_executable_or_invalid_points(self):
        with self.assertRaises(ValueError):
            validate_route({**ROUTE, "executable_on_robot": True}, CALIBRATION)
        with self.assertRaises(ValueError):
            validate_route({**ROUTE, "path": [{"x": 0, "y": 0}, {"x": float("nan"), "y": 1}]}, CALIBRATION)

    def test_densifies_short_browser_route_without_changing_endpoints(self):
        points = dense_points(ROUTE["path"])
        self.assertGreaterEqual(len(points), 13)
        self.assertEqual(points[0], ROUTE["path"][0])
        self.assertEqual(points[-1], ROUTE["path"][-1])

    def test_rejects_zero_length_and_skips_duplicate_control_points(self):
        with self.assertRaises(ValueError):
            validate_route({**ROUTE, "path": [ROUTE["path"][0]] * 2}, CALIBRATION)
        points = dense_points([ROUTE["path"][0], *ROUTE["path"]])
        self.assertEqual(len(points), len(dense_points(ROUTE["path"])))


if __name__ == "__main__":
    unittest.main()
