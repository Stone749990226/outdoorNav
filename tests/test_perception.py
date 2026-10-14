"""Synthetic depth surfaces preserve obstacle detection without importing ROS."""

from types import SimpleNamespace as Namespace
import unittest
import numpy as np
from outdoornav.perception import edge_columns


class DepthGeometryTests(unittest.TestCase):
    def test_flat_ground_and_missing_returns_are_not_negative_obstacles(self):
        for depth in (self.surface(False), np.full((240, 320), np.nan)):
            self.assertEqual(len(self.detect(depth)), 0)

    def test_measured_thirty_centimeter_drop_is_detected_across_robot_width(self):
        edges = self.detect(self.surface(True))
        self.assertGreaterEqual(len(edges), 12)
        self.assertGreaterEqual(np.ptp(edges[:, 1]), 0.5)

    @staticmethod
    def surface(drop):
        v, _ = np.mgrid[:240, :320]
        down = (v - 120) / 120
        with np.errstate(divide="ignore", invalid="ignore"):
            near, far = 0.55 / down, 0.85 / down
            if not drop:
                return np.where((down > 0) & (near < 7.95), near, np.nan)
            return np.where(
                (down > 0) & (near <= 2),
                near,
                np.where((down > 0) & (far >= 2) & (far < 7.95), far, np.nan),
            )

    @staticmethod
    def detect(depth):
        transform = Namespace(
            transform=Namespace(
                rotation=Namespace(x=-0.5, y=0.5, z=-0.5, w=0.5),
                translation=Namespace(x=0, y=0, z=0.55),
            )
        )
        return edge_columns(
            depth, (120, 120, 160, 120), transform, 0.55, 0.15, 0.06, ground_ref="plane"
        )[0]
