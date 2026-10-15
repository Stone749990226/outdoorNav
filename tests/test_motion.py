import unittest
import numpy as np
from outdoornav.motion import MotionTracker, prediction_mask, scan_clusters


class MotionTests(unittest.TestCase):
    def test_stationary_returns_are_not_predicted_and_moving_body_is_swept(self):
        tracker = MotionTracker()
        face = np.array([[2.0, 1.0], [2.0, 1.2], [2.0, 1.4]])
        for stamp in np.arange(0, 1, 0.1):
            tracker.observe([face], float(stamp))
        self.assertEqual(tracker.envelopes(1.0), [])
        for stamp in np.arange(1, 2, 0.1):
            tracker.observe([face + np.array([0.0, 0.2 * (stamp - 1)])], float(stamp))
        regions = tracker.envelopes(2.0)
        self.assertEqual(len(regions), 1)
        low, high = regions[0]
        self.assertLess(low[0], 2.0)
        self.assertGreater(high[1], 2.1)
        self.assertEqual(tracker.envelopes(4.0), [])
        tracker.observe([face], 0.1)
        self.assertEqual(tracker.envelopes(0.1), [])

    def test_separate_clusters_and_raster_bounds(self):
        groups = scan_clusters([[0, 0], [0.1, 0], [2, 1], [2, 1.1]])
        self.assertEqual(len(groups), 2)
        grid = {"width": 10, "height": 10, "origin": [0, 0], "resolution": 0.1}
        mask = prediction_mask(grid, [(np.array([-0.5, -0.5]), np.array([0.3, 0.3]))])
        self.assertGreater(mask[0, 0], 60)
        self.assertLess(mask[0, 0], 99)
        self.assertEqual(mask[-1, -1], 0)
        # An off-map prediction must not wrap around into the opposite grid edge.
        empty = prediction_mask(
            grid, [(np.array([-2.0, -2.0]), np.array([-0.1, -0.1]))]
        )
        self.assertEqual(np.count_nonzero(empty), 0)


if __name__ == "__main__":
    unittest.main()
