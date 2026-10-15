"""Exercise version, simulation-clock and sensor failures in the owned simulation."""

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from urllib.error import HTTPError

from .acceptance import Acceptance
from .runtime.environment import ros_environment
from .runtime.processes import matches, owned
from .tasks import TERMINAL
from .validation import FAULT_SCENARIOS


class FaultAcceptance(Acceptance):
    def __init__(self, run, timeout):
        super().__init__(run, timeout)
        self.manifest = json.loads((run / "run.json").read_text())
        self.paused = False

    def control(self, request):
        env = ros_environment(True)
        env.update(
            IGN_PARTITION=self.manifest["partition"],
            GZ_PARTITION=self.manifest["partition"],
            OUTDOORNAV_RUN_ID=self.manifest["run_id"],
        )
        response = subprocess.run(
            [
                "ign",
                "service",
                "-s",
                "/world/north_campus/control",
                "--reqtype",
                "ignition.msgs.WorldControl",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                request,
            ],
            env=env,
            capture_output=True,
            text=True,
            timeout=7,
        )
        if response.returncode or "data: true" not in response.stdout:
            raise RuntimeError(
                "Gazebo control failed: " + response.stdout + response.stderr
            )

    def run_scenario(self, scenario):
        if scenario == "version-mismatch":
            payload = dict(
                frame_id="map",
                scene_version="invalid",
                route_version=self.bundle["route_version"],
                goal=self.nearby(3),
            )
            before = self.request("/api/status")["task"]
            try:
                self.request("/api/tasks", payload)
            except HTTPError as error:
                if error.code != 409:
                    raise
                after = self.request("/api/status")["task"]
                identity = (
                    lambda task: (
                        task["task_id"],
                        task["goal"],
                        task["blocked_road_ids"],
                    )
                    if task
                    else None
                )
                if identity(before) != identity(after):
                    raise RuntimeError("Rejected version changed the current task")
                return {"passed": True, "http_status": 409, "task_unchanged": True}
            raise RuntimeError("Mismatched scene was accepted")
        record = self.submit(self.nearby(8))
        self.wait_moving(record)
        if scenario == "pause":
            self.control("pause: true")
            self.paused = True
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                status = self.request("/api/status")
                if not status["health"]["clock"]:
                    break
                time.sleep(0.1)
            else:
                raise RuntimeError("Paused clock was not detected")
            self.control("pause: false")
            self.paused = False
        elif scenario == "rewind":
            self.control("reset { time_only: true }")
        elif scenario == "sensor-loss":
            candidates = []
            for process in owned(self.manifest["run_id"]):
                try:
                    command = (
                        Path(f"/proc/{process['pid']}/cmdline")
                        .read_bytes()
                        .split(b"\0")
                    )
                    if any(
                        part.endswith(b"/outdoornav_ros/depth_points")
                        for part in command
                    ):
                        candidates.append(process)
                except OSError:
                    pass
            if len(candidates) != 1 or not matches(candidates[0]):
                raise RuntimeError("Could not identify this run's depth processor")
            os.kill(candidates[0]["pid"], signal.SIGTERM)
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            status = self.request("/api/status")
            task = self.request("/api/tasks/" + record["task"]["task_id"])
            if task["state"] in TERMINAL:
                break
            time.sleep(0.2)
        else:
            raise RuntimeError("Fault did not produce a terminal task result")
        self.active.discard(task["task_id"])
        time.sleep(1.2)
        rows = list(csv.DictReader((self.run / "trajectory.csv").open()))
        # A time-only reset starts a new monotonic segment of simulation timestamps.
        if scenario == "rewind":
            boundary = max(
                (
                    i
                    for i in range(1, len(rows))
                    if float(rows[i]["t"]) < float(rows[i - 1]["t"])
                ),
                default=0,
            )
            rows = rows[boundary:]
        completed = task["completed_sim_s"]
        rows = [
            row
            for row in rows
            if completed is not None and float(row["t"]) <= completed
        ]
        if not rows:
            raise RuntimeError("No independent truth samples after the fault")
        final = rows[-1]
        tail = [row for row in rows if float(row["t"]) >= float(final["t"]) - 0.5]
        displacement = max(
            math.hypot(
                float(row["gt_x"]) - float(final["gt_x"]),
                float(row["gt_y"]) - float(final["gt_y"]),
            )
            for row in tail
        )
        result = {
            "task": task,
            "health": status["health"],
            "truth_stop_displacement_m": displacement,
            "passed": task["state"] == "failed" and displacement <= 0.02,
        }
        result["task"] = {
            key: value
            for key, value in task.items()
            if key not in ("execution_path", "reference_path")
        }
        if scenario == "rewind":
            result["passed"] &= "clock moved backward" in (task["reason"] or "")
        if not result["passed"]:
            raise RuntimeError("Fault acceptance failed: " + json.dumps(result))
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--scenario", required=True, choices=FAULT_SCENARIOS)
    parser.add_argument("--timeout", type=float, default=30)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("Timeout must be finite and positive")
    runner = FaultAcceptance(args.run, args.timeout)
    report = {
        "schema_version": 1,
        "run_id": args.run.name,
        "scenario": args.scenario,
        "scene_version": runner.bundle["scene_version"],
        "route_version": runner.bundle["route_version"],
    }
    try:
        report.update(runner.run_scenario(args.scenario))
    except (Exception, KeyboardInterrupt) as error:
        report.update(passed=False, error=str(error))
    finally:
        if runner.paused:
            try:
                runner.control("pause: false")
            except Exception:
                pass
        for task_id in runner.active:
            try:
                runner.request("/api/tasks/" + task_id + "/cancel", {})
            except OSError:
                pass
        target = args.run / (
            "acceptance-" + args.scenario + "-" + time.strftime("%H%M%S") + ".json"
        )
        target.write_text(json.dumps(report, indent=2) + "\n")
        print("Acceptance report: " + str(target), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
