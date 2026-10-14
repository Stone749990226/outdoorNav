import json
import math
from pathlib import Path
import tempfile
import unittest

from outdoornav.survey import (
    build,
    distance,
    filter_track,
    resample,
    simplify,
    timed_fixes,
)


def track(n=80):
    return [
        {
            "x": i * 0.2,
            "y": 0,
            "time_s": i * 0.2,
            "index": i,
            "utc": str(i),
            "quality": 2,
        }
        for i in range(n)
    ]


class SurveyMapTests(unittest.TestCase):
    def test_isolated_spike_is_rejected_without_breaking_valid_motion(self):
        points = track()
        points[30] = {**points[30], "y": 20}
        chunks, report = filter_track(points)
        self.assertEqual(len(chunks), 1)
        self.assertEqual(report["rejected"][0]["index"], 30)
        self.assertLess(max(abs(p["y"]) for p in chunks[0]), 0.001)

    def test_missing_data_and_persistent_jump_remain_separate(self):
        points = track()
        chunks, report = filter_track(points[:20] + points[40:])
        self.assertEqual(len(chunks), 2)
        self.assertTrue(report["gaps"])
        points[40:] = [{**p, "y": 30} for p in points[40:]]
        chunks, report = filter_track(points)
        self.assertEqual(len(chunks), 2)
        self.assertFalse(report["rejected"])

    def test_unusable_quality_does_not_create_even_a_short_connection(self):
        points = track()
        points[30] = {**points[30], "quality": 0}
        chunks, report = filter_track(points)
        self.assertEqual(len(chunks), 2)
        self.assertTrue(report["gaps"])

    def test_stops_are_compressed_and_corner_geometry_is_preserved(self):
        points = [{**p, "x": 2, "y": 3} for p in track()]
        chunks, report = filter_track(points)
        self.assertEqual(len(chunks[0]), 1)
        self.assertEqual(len(report["stationary_segments"]), 1)
        corner = [{"x": 0, "y": 0}, {"x": 10.2, "y": 0}, {"x": 10.2, "y": 10}]
        self.assertEqual(simplify(resample(corner)), corner)

    def test_time_order_midnight_and_duplicate_handling(self):
        fixes = [
            {"utc": utc}
            for utc in ("235959.80", "000000.00", "000000.00", "000000.20", "000000.10")
        ]
        points, rejected = timed_fixes(fixes)
        self.assertEqual(len(points), 3)
        self.assertEqual(len(rejected), 2)
        self.assertAlmostEqual(points[1]["time_s"] - points[0]["time_s"], 0.2)

    def test_actual_logs_form_one_loop_and_one_confirmed_branch(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            network, report = build(output=output)
            rebuilt, _ = build(output=output)
            self.assertEqual(network, rebuilt)
            self.assertEqual(len(network["nodes"]), 3)
            self.assertEqual(len(network["roads"]), 3)
            nodes = {n["id"]: n for n in network["nodes"]}
            for road in network["roads"]:
                self.assertLess(distance(road["points"][0], nodes[road["from"]]), 1e-8)
                self.assertLess(distance(road["points"][-1], nodes[road["to"]]), 1e-8)
                self.assertTrue(
                    all(
                        math.isfinite(p[axis])
                        for p in road["points"]
                        for axis in ("x", "y")
                    )
                )
            self.assertLess(report["joins"]["closure"]["distance_m"], 2)
            self.assertLess(report["joins"]["junction"]["distance_m"], 5)
            self.assertEqual(report["logs"]["loop"]["quality_counts"], {"2": 2135})
            self.assertEqual(report["logs"]["branch"]["quality_counts"], {"2": 368})
            self.assertEqual(network["quality_status"], "survey_dgps_unverified")
            self.assertFalse(network["executable_on_robot"])
            self.assertTrue((output / "map.svg").exists())
            self.assertEqual(
                json.loads((output / "origin.json").read_text()),
                network["origin_wgs84"],
            )


if __name__ == "__main__":
    unittest.main()
