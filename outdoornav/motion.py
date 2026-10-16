"""Short-horizon occupancy prediction from sensor clusters, independent of Gazebo controls."""

from collections import deque
import math
import numpy as np


def scan_clusters(points, gap=0.3):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) < 2:
        return []
    breaks = np.flatnonzero(np.linalg.norm(np.diff(points, axis=0), axis=1) > gap) + 1
    groups = np.split(points, breaks)
    if len(groups) > 1 and np.linalg.norm(groups[0][0] - groups[-1][-1]) < gap:
        groups = [np.concatenate((groups[-1], groups[0])), *groups[1:-1]]
    return [
        group
        for group in groups
        if len(group) >= 2 and np.ptp(group, axis=0).max() <= 2.0
    ]


class MotionTracker:
    def __init__(self, horizon=2.0, stale_after=1.0):
        self.horizon, self.stale_after = horizon, stale_after
        self.tracks = []
        self.last_stamp = None

    def observe(self, clusters, stamp):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.tracks.clear()
        self.last_stamp = stamp
        available = [t for t in self.tracks if stamp - t["stamp"] < self.stale_after]
        assigned = set()
        for points in clusters:
            center = np.mean(points, axis=0)
            candidates = [
                (np.linalg.norm(center - t["center"]), i, t)
                for i, t in enumerate(available)
                if i not in assigned
            ]
            distance, index, track = (
                min(candidates, key=lambda x: x[0])
                if candidates
                else (math.inf, None, None)
            )
            if distance > 0.65:
                track = {"history": deque(), "velocity": np.zeros(2)}
                self.tracks.append(track)
            else:
                assigned.add(index)
            history = track["history"]
            history.append((stamp, center))
            while len(history) > 1 and stamp - history[0][0] > 0.7:
                history.popleft()
            if len(history) > 1 and stamp - history[0][0] >= 0.35:
                velocity = (center - history[0][1]) / (stamp - history[0][0])
                speed = np.linalg.norm(velocity)
                track["velocity"] = velocity if 0.08 <= speed <= 1.2 else np.zeros(2)
            track.update(
                center=center,
                lower=points.min(axis=0),
                upper=points.max(axis=0),
                stamp=stamp,
            )
        self.tracks = [t for t in self.tracks if stamp - t["stamp"] < self.stale_after]

    def envelopes(self, stamp):
        regions = []
        for track in self.tracks:
            age = stamp - track["stamp"]
            if (
                not 0 <= age < self.stale_after
                or np.linalg.norm(track["velocity"]) < 0.08
            ):
                continue
            delta = track["velocity"] * (self.horizon + age)
            # A single lidar sees the near face. Cover the unseen body and matching uncertainty.
            regions.append(
                (
                    track["lower"] + np.minimum(delta, 0) - 0.35,
                    track["upper"] + np.maximum(delta, 0) + 0.35,
                )
            )
        return regions


def prediction_mask(grid, regions):
    # No prediction means no information, not measured free space. KeepoutFilter
    # skips -1 but replaces UNKNOWN in the master map even with a mask value of 0.
    mask = np.full((grid["height"], grid["width"]), -1, dtype=np.int8)
    resolution = grid["resolution"]
    origin = np.asarray(grid["origin"])
    for lower, upper in regions:
        c0, r0 = np.floor((lower - origin) / resolution).astype(int)
        c1, r1 = np.ceil((upper - origin) / resolution).astype(int)
        mask[
            max(0, r0) : max(0, min(mask.shape[0], r1)),
            max(0, c0) : max(0, min(mask.shape[1], c1)),
        ] = 80
    return mask
