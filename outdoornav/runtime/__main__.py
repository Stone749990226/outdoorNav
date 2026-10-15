"""One CLI for survey data, simulation, navigation and diagnostics."""

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from outdoornav.validation import NAVIGATION_SCENARIOS, FAULT_SCENARIOS
from outdoornav.bundle import ROOT, load_bundle
from .build import build
from .environment import data_python, doctor, ros_environment
from .processes import matches
from .run import RenderStartupError, start, down
from .state import RUNS, active


def main():
    parser = argparse.ArgumentParser(prog="./nav", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("doctor", "build", "status", "down"):
        commands.add_parser(name)
    up = commands.add_parser("up")
    up.add_argument("--rviz", action="store_true")
    up.add_argument(
        "--render", choices=["auto", "hardware", "software"], default="auto"
    )
    up.add_argument("--port", type=int, default=5174)
    logs = commands.add_parser("logs")
    logs.add_argument("component", nargs="?")
    survey = commands.add_parser("survey")
    survey.add_argument("--loop", type=Path, default=ROOT / "1.ubx")
    survey.add_argument("--branch", type=Path, default=ROOT / "2.ubx")
    scene = commands.add_parser("scene")
    scene.add_argument("action", choices=["build"])
    obstacle = commands.add_parser("obstacle")
    obstacle.add_argument("action", choices=["add", "remove"])
    obstacle.add_argument("--name", default="test_obstacle")
    obstacle.add_argument("--kind", choices=["static", "moving"], default="static")
    obstacle.add_argument("--end-x", type=float)
    obstacle.add_argument("--end-y", type=float)
    obstacle.add_argument("--speed", type=float, default=0.3)
    for key, value in [
        ("x", 0.0),
        ("y", -8.0),
        ("width", 0.4),
        ("depth", 0.4),
        ("height", 1.0),
    ]:
        obstacle.add_argument("--" + key, type=float, default=value)
    diagnose = commands.add_parser("diagnose")
    diagnose.add_argument("report")
    diagnose.add_argument("arguments", nargs=argparse.REMAINDER)
    record = commands.add_parser("record")
    record.add_argument("--output", type=Path)
    capture = commands.add_parser(
        "capture", help="Capture paired costmaps and point clouds"
    )
    capture.add_argument("--output", type=Path)
    capture.add_argument("--seconds", type=float, default=60)
    capture.add_argument("--interval", type=float, default=2)
    validate = commands.add_parser(
        "validate", help="Run physical navigation acceptance scenarios"
    )
    validate.add_argument(
        "--scenario",
        choices=[*NAVIGATION_SCENARIOS, *FAULT_SCENARIOS, "all"],
        default="short",
    )
    validate.add_argument("--timeout", type=float, default=900)
    args = parser.parse_args()
    try:
        if args.command == "doctor":
            checks = doctor()
            print(json.dumps(checks, ensure_ascii=False, indent=2))
            return 0 if all(c["ok"] for c in checks) else 1
        if args.command == "build":
            if active():
                raise RuntimeError("Stop the current run before rebuilding")
            build()
        elif args.command == "up":
            if not 1 <= args.port <= 65535:
                raise ValueError("Web port must be 1..65535")
            try:
                start(args)
            except RenderStartupError as error:
                if args.render != "auto":
                    raise
                print(f"{error}\nRetrying with software rendering.", flush=True)
                start(argparse.Namespace(**{**vars(args), "render": "software"}))
        elif args.command == "down":
            down()
        elif args.command == "status":
            state = active()
            if state:
                state["component_health"] = {
                    k: matches(v) for k, v in state["components"].items()
                }
                try:
                    with urlopen(
                        "http://127.0.0.1:8765/api/status", timeout=2
                    ) as response:
                        state["navigation"] = json.load(response)
                        task = state["navigation"].get("task")
                        if task:
                            for field in ("execution_path", "reference_path"):
                                task.pop(field, None)
                except OSError:
                    state["navigation"] = {"ready": False}
            print(
                json.dumps(
                    state or {"ready": False, "message": "stopped"},
                    ensure_ascii=False,
                    indent=2,
                )
            )
        elif args.command == "logs":
            state = active()
            directories = sorted(p for p in RUNS.glob("*/logs") if p.is_dir())
            folder = (
                Path(state["directory"]) / "logs"
                if state
                else directories[-1]
                if directories
                else None
            )
            if folder is None:
                raise RuntimeError("No run logs")
            files = (
                [folder / (args.component + ".log")]
                if args.component
                else sorted(folder.glob("*.log"))
            )
            if not files or not all(p.is_file() and p.parent == folder for p in files):
                raise ValueError("Unknown log component")
            subprocess.run(["tail", "-n", "80", *map(str, files)], check=True)
        elif args.command in ("survey", "scene"):
            if active():
                raise RuntimeError("Stop the current run before replacing scene data")
            subprocess.run(
                [
                    *data_python(),
                    "-m",
                    "outdoornav.generate",
                    args.command,
                    *(
                        ["--loop", str(args.loop), "--branch", str(args.branch)]
                        if args.command == "survey"
                        else []
                    ),
                ],
                check=True,
            )
        elif args.command == "obstacle":
            state = active()
            if not state or not state["ready"]:
                raise RuntimeError("Start the campus with ./nav up first")
            if args.action == "add":
                bundle = load_bundle()
                payload = {
                    key: bundle[key] for key in ("scene_version", "route_version")
                }
                payload.update(
                    frame_id="map",
                    name=args.name,
                    kind=args.kind,
                    start={"x": args.x, "y": args.y},
                    end={"x": args.end_x, "y": args.end_y},
                    width=args.width,
                    depth=args.depth,
                    height=args.height,
                    speed_mps=args.speed,
                )
                request = Request(
                    "http://127.0.0.1:8765/api/obstacles",
                    method="POST",
                    data=json.dumps(payload).encode(),
                    headers={"Content-Type": "application/json"},
                )
            else:
                import re

                if not re.fullmatch(r"test_[A-Za-z0-9_]{1,48}", args.name):
                    raise ValueError("Invalid obstacle name")
                request = Request(
                    "http://127.0.0.1:8765/api/obstacles/" + args.name, method="DELETE"
                )
            with urlopen(request, timeout=8) as response:
                print(json.dumps(json.load(response), ensure_ascii=False, indent=2))
        elif args.command == "validate":
            state = active()
            if not state or not state["ready"]:
                raise RuntimeError("Start the campus with ./nav up first")
            subprocess.run(
                [
                    *data_python(),
                    "-m",
                    "outdoornav.faults"
                    if args.scenario in FAULT_SCENARIOS
                    else "outdoornav.acceptance",
                    "--run",
                    state["directory"],
                    "--scenario",
                    args.scenario,
                    "--timeout",
                    str(args.timeout),
                ],
                check=True,
            )
        elif args.command == "diagnose":
            subprocess.run(
                [*data_python(), "-m", "diagnostics", args.report, *args.arguments],
                check=True,
            )
        elif args.command in ("record", "capture"):
            state = active()
            if not state or not state["ready"]:
                raise RuntimeError("Start the campus with ./nav up first")
            env = ros_environment(True)
            env.update(
                ROS_DOMAIN_ID=state["ros_domain_id"], OUTDOORNAV_RUN_ID=state["run_id"]
            )
            env["RMW_IMPLEMENTATION"] = state["rmw_implementation"]
            if args.command == "capture":
                subprocess.run(
                    [
                        "ros2",
                        "run",
                        "outdoornav_ros",
                        "capture",
                        "--output",
                        str(
                            args.output
                            or Path(state["directory"])
                            / "captures"
                            / (time.strftime("%H%M%S") + "-" + uuid.uuid4().hex[:4])
                            / "probe"
                        ),
                        "--seconds",
                        str(args.seconds),
                        "--interval",
                        str(args.interval),
                    ],
                    env=env,
                    check=True,
                )
                return 0
            destination = args.output or Path(
                state["directory"]
            ) / "bags" / time.strftime("%H%M%S")
            subprocess.run(
                [
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    str(destination),
                    "/clock",
                    "/tf",
                    "/tf_static",
                    "/rtk/scan",
                    "/rtk/camera/points",
                    "/rtk/camera/depth_image",
                    "/rtk/imu",
                    "/rtk/sim_fix",
                    "/odometry/local",
                    "/odometry/global",
                    "/ground_truth/odom",
                    "/global_costmap/costmap",
                    "/local_costmap/costmap",
                    "/cmd_vel",
                    "/navigation/execution_path",
                ],
                env=env,
                check=True,
            )
        return 0
    except HTTPError as error:
        try:
            detail = json.load(error).get("error", str(error))
        except (ValueError, OSError):
            detail = str(error)
        print(f"OutdoorNav: {detail}", file=sys.stderr)
        return 1
    except (RuntimeError, ValueError, OSError, subprocess.SubprocessError) as error:
        print(f"OutdoorNav: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
