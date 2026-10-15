"""Run repeatable physical navigation scenarios through the public HTTP contract."""

import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import time
from urllib.request import Request, urlopen

from .bundle import ROOT, is_free, masks, route_mask
from .routing import plan_route, snap
from .tasks import TERMINAL
from .validation import NAVIGATION_SCENARIOS


class Acceptance:
    def __init__(self, run, timeout):
        self.run, self.timeout = run, timeout
        self.bundle = self.request("/api/map")
        self.free = masks()
        self.results = []
        self.active = set()

    def request(self, endpoint, body=None, method=None):
        request = Request(
            "http://127.0.0.1:8765" + endpoint,
            data=json.dumps(body).encode() if body is not None else None,
            headers={"Content-Type": "application/json"},
            method=method,
        )
        attempts = 3 if body is None else 1
        for attempt in range(attempts):
            try:
                with urlopen(request, timeout=5) as response:
                    return json.load(response)
            except (TimeoutError, ConnectionResetError):
                if attempt + 1 == attempts:
                    raise
                time.sleep(0.2)

    def pose(self):
        status = self.request("/api/status")
        if not status["ready"]:
            raise RuntimeError("System is not healthy: " + str(status["health"]))
        return status["pose"]

    def nearby(self, distance):
        current = snap(self.bundle["network"], self.pose())
        road = next(
            r for r in self.bundle["network"]["roads"] if r["id"] == current["road_id"]
        )
        target = current["offset_m"] + distance
        if target > road["length_m"]:
            target = current["offset_m"] - distance
        if target < 0:
            raise ValueError(
                "This scenario needs a longer road ahead; choose a starting point farther from a junction"
            )
        offset = 0
        for a, b in zip(road["points"], road["points"][1:]):
            segment = math.dist((a["x"], a["y"]), (b["x"], b["y"]))
            if offset + segment >= target:
                t = (target - offset) / segment
                return {k: a[k] + t * (b[k] - a[k]) for k in ("x", "y")}
            offset += segment
        return road["points"][-1]

    def submit(self, goal, blocked=()):
        start = self.pose()
        payload = {k: self.bundle[k] for k in ("scene_version", "route_version")}
        payload.update(frame_id="map", goal=goal, blocked_road_ids=list(blocked))
        task = self.request("/api/tasks", payload)
        self.active.add(task["task_id"])
        return {
            "task": task,
            "start": start,
            "began_s": start["stamp_s"],
            "blocked": list(blocked),
        }

    def cancel(self, record):
        self.request("/api/tasks/" + record["task"]["task_id"] + "/cancel", {})

    def wait_moving(self, record):
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            task = self.request("/api/tasks/" + record["task"]["task_id"])
            if task["state"] in ("navigating", "replanning"):
                position = self.pose()
                if (
                    math.hypot(
                        position["x"] - record["start"]["x"],
                        position["y"] - record["start"]["y"],
                    )
                    >= 0.5
                ):
                    return
            if task["state"] in TERMINAL:
                raise RuntimeError(
                    "Task ended before navigation: " + str(task["reason"])
                )
            time.sleep(0.2)
        raise RuntimeError("Navigation did not begin")

    def finish(self, record, expected="succeeded", obstacle=None, require_rejoin=False):
        deadline, previous = time.monotonic() + self.timeout, None
        while time.monotonic() < deadline:
            task = self.request("/api/tasks/" + record["task"]["task_id"])
            if task["state"] != previous:
                print(task["task_id"][:8] + ": " + task["state"], flush=True)
                previous = task["state"]
            if task["state"] in TERMINAL:
                break
            time.sleep(0.5)
        else:
            self.cancel(record)
            raise RuntimeError("Acceptance deadline exceeded")
        self.active.discard(task["task_id"])
        pose = self.pose()
        completion_pose = task["completion_pose"] or pose
        ended = task["completed_sim_s"] or pose["stamp_s"]
        result = {
            "task_id": task["task_id"],
            "state": task["state"],
            "reason": task["reason"],
            "path_revision": task["path_revision"],
            "goal": task["goal"],
            "estimated_error_m": math.dist(
                (completion_pose["x"], completion_pose["y"]),
                (task["goal"]["x"], task["goal"]["y"]),
            ),
            "expected": expected,
            "began_s": record["began_s"],
            "ended_s": ended,
        }
        time.sleep(1.5)  # Allow buffered truth samples to be flushed by the recorder.
        samples = []
        with (self.run / "trajectory.csv").open() as stream:
            for row in csv.DictReader(stream):
                if record["began_s"] <= float(row["t"]) <= ended:
                    samples.append({k: float(row[k]) for k in ("t", "gt_x", "gt_y")})
        if not samples:
            raise RuntimeError("No independent ground-truth trajectory was recorded")
        goal = task["goal"]
        result["truth_error_m"] = math.hypot(
            samples[-1]["gt_x"] - goal["x"], samples[-1]["gt_y"] - goal["y"]
        )
        last = samples[-1]
        stopped = [r for r in samples if r["t"] >= last["t"] - 0.5]
        result["truth_stop_displacement_m"] = max(
            math.hypot(r["gt_x"] - last["gt_x"], r["gt_y"] - last["gt_y"])
            for r in stopped
        )
        route = plan_route(
            self.bundle["network"], record["start"], goal, record["blocked"]
        )
        allowed = route_mask(self.bundle, self.free, route, record["start"]) < 100
        result["truth_outside_corridor_samples"] = sum(
            not is_free(allowed, self.bundle["grid"], {"x": r["gt_x"], "y": r["gt_y"]})
            for r in samples
        )
        from shapely.geometry import Point, box, shape

        obstacles = [
            shape(o["footprint"]) for o in self.bundle["geometry"]["obstacles"]
        ]
        if obstacle:
            x, y, w, d = obstacle
            obstacles.append(box(x - w / 2, y - d / 2, x + w / 2, y + d / 2))
        result["minimum_truth_obstacle_clearance_m"] = min(
            Point(r["gt_x"], r["gt_y"]).distance(o) for r in samples for o in obstacles
        )
        # Pair ground truth with acknowledged Gazebo obstacle poses by simulation time.
        # Newly injected fixtures do not exist before their added event.
        log = self.run / "obstacles.csv"
        if log.is_file():
            with log.open() as stream:
                events = sorted(
                    csv.DictReader(stream), key=lambda row: float(row["sim_s"])
                )
            cursor, fixtures, clearance = 0, {}, math.inf
            for sample in samples:
                while (
                    cursor < len(events)
                    and float(events[cursor]["sim_s"]) <= sample["t"]
                ):
                    event = events[cursor]
                    if event["event"] == "removed":
                        fixtures.pop(event["name"], None)
                    else:
                        fixtures[event["name"]] = event
                    cursor += 1
                for fixture in fixtures.values():
                    x, y, w, d = (
                        float(fixture[key]) for key in ("x", "y", "width", "depth")
                    )
                    gap = Point(sample["gt_x"], sample["gt_y"]).distance(
                        box(x - w / 2, y - d / 2, x + w / 2, y + d / 2)
                    )
                    clearance = min(clearance, gap)
            result["minimum_injected_obstacle_clearance_m"] = (
                clearance if math.isfinite(clearance) else None
            )
            result["minimum_truth_obstacle_clearance_m"] = min(
                result["minimum_truth_obstacle_clearance_m"], clearance
            )
        reference = route["reference_path"]
        if len(reference) > 1:
            from shapely.geometry import LineString

            line = LineString([(p["x"], p["y"]) for p in reference])
            result["final_reference_distance_m"] = Point(
                last["gt_x"], last["gt_y"]
            ).distance(line)
            tail = [
                sample
                for sample in samples
                if math.hypot(sample["gt_x"] - goal["x"], sample["gt_y"] - goal["y"])
                <= 2
            ]
            result["last_two_meters_reference_distance_max_m"] = max(
                (
                    Point(sample["gt_x"], sample["gt_y"]).distance(line)
                    for sample in tail
                ),
                default=None,
            )
        # The configured Nav2/physical robot radius is 0.105 m; the scene's 0.5 m
        # survey-clearance assumption is a different geometric quantity.
        result["passed"] = (
            task["state"] == expected
            and result["truth_outside_corridor_samples"] == 0
            and result["minimum_truth_obstacle_clearance_m"] >= 0.105
            and result["truth_stop_displacement_m"] <= 0.02
            and (expected != "succeeded" or result["estimated_error_m"] <= 0.25)
            and (
                not require_rejoin
                or (
                    result.get("last_two_meters_reference_distance_max_m") is not None
                    and result["last_two_meters_reference_distance_max_m"] <= 0.6
                )
            )
        )
        self.results.append(result)
        if not result["passed"]:
            raise RuntimeError("Scenario acceptance failed: " + json.dumps(result))
        return result

    def obstacle(self, action, point=None, width=0.6, depth=0.6):
        command = [str(ROOT / "nav"), "obstacle", action, "--name", "test_acceptance"]
        if point:
            command += [
                "--x",
                str(point["x"]),
                "--y",
                str(point["y"]),
                "--width",
                str(width),
                "--depth",
                str(depth),
                "--height",
                ".8",
            ]
        subprocess.run(command, check=True)

    def scenario(self, name):
        print("Scenario: " + name, flush=True)
        if name == "short":
            self.finish(self.submit(self.nearby(2)))
        elif name == "turnaround":
            start = snap(self.bundle["network"], self.pose())["point"]
            self.finish(self.submit(self.nearby(4)))
            self.finish(self.submit(start))
        elif name == "cancel":
            task = self.submit(self.nearby(6))
            self.wait_moving(task)
            self.cancel(task)
            self.finish(task, "canceled")
        elif name == "replace":
            near, far = self.nearby(3), self.nearby(6)
            first = self.submit(far)
            self.wait_moving(first)
            second = self.submit(near)
            self.finish(first, "canceled")
            self.finish(second)
        elif name in ("detour", "blocked"):
            distance = 6 if name == "detour" else 24
            start = self.pose()
            goal = self.nearby(distance)
            center = self.nearby(distance / 2)
            dx, dy = goal["x"] - start["x"], goal["y"] - start["y"]
            width, depth = (
                (0.6, 0.6)
                if name == "detour"
                else ((1, 12) if abs(dx) > abs(dy) else (12, 1))
            )
            self.obstacle("add", center, width, depth)
            try:
                time.sleep(2)
                result = self.finish(
                    self.submit(goal),
                    "succeeded" if name == "detour" else "failed",
                    (center["x"], center["y"], width, depth),
                )
                if name == "blocked" and "No traversable path" not in (
                    result["reason"] or ""
                ):
                    raise RuntimeError(
                        "Blockade failure did not report a constrained planning failure"
                    )
            finally:
                self.obstacle("remove")
        elif name in ("injected-static", "moving"):
            goal, center, start = self.nearby(12), self.nearby(6), self.pose()
            dx, dy = goal["x"] - start["x"], goal["y"] - start["y"]
            norm = math.hypot(dx, dy)
            nx, ny = -dy / norm, dx / norm
            moving = name == "moving"
            payload = {
                key: self.bundle[key] for key in ("scene_version", "route_version")
            }
            payload.update(
                frame_id="map",
                name="test_injected",
                kind="moving" if moving else "static",
                start={
                    "x": center["x"] - (0.8 * nx if moving else 0),
                    "y": center["y"] - (0.8 * ny if moving else 0),
                },
                end={"x": center["x"] + 0.8 * nx, "y": center["y"] + 0.8 * ny},
                width=0.6,
                depth=0.6,
                height=0.8,
                speed_mps=0.2,
            )
            task = self.submit(goal)
            self.wait_moving(task)
            self.request("/api/obstacles", payload)
            try:
                self.finish(task, require_rejoin=True)
            finally:
                self.request("/api/obstacles/test_injected", method="DELETE")
        elif name in ("ring", "branch"):
            nodes = {n["id"]: n for n in self.bundle["network"]["nodes"]}
            goal = lambda node: {k: nodes[node][k] for k in ("x", "y")}
            if name == "branch":
                self.finish(self.submit(goal("P")))
            else:
                self.finish(self.submit(goal("S")))
                self.finish(self.submit(goal("J"), ["loop-a"]))
                self.finish(self.submit(goal("S"), ["loop-b"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument(
        "--scenario",
        choices=[*NAVIGATION_SCENARIOS, "all"],
        default="short",
    )
    parser.add_argument("--timeout", type=float, default=900)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("Timeout must be finite and positive")
    runner = Acceptance(args.run, args.timeout)
    report = {
        "schema_version": 1,
        "scenario": args.scenario,
        "run_id": args.run.name,
        "scene_version": runner.bundle["scene_version"],
        "route_version": runner.bundle["route_version"],
    }
    try:
        names = NAVIGATION_SCENARIOS if args.scenario == "all" else [args.scenario]
        for name in names:
            runner.scenario(name)
        report["passed"] = True
    except (Exception, KeyboardInterrupt) as error:
        report.update(passed=False, error=str(error))
        for task in runner.active:
            try:
                runner.request("/api/tasks/" + task + "/cancel", {})
            except OSError:
                pass
    finally:
        report["tasks"] = runner.results
        destination = args.run / (
            "acceptance-" + args.scenario + "-" + time.strftime("%H%M%S") + ".json"
        )
        destination.write_text(json.dumps(report, indent=2) + "\n")
        print("Acceptance report: " + str(destination), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
