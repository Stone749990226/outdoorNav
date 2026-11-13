"""Run actual services against isolated mock primitives, with no hardware startup."""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError

ROOT = Path(__file__).resolve().parent


def request(url, body=None, timeout=8):
    req = Request(url, data=json.dumps(body).encode() if body is not None else None,
                  headers={"Content-Type": "application/json"})
    try:
        with urlopen(req, timeout=timeout) as response:
            return json.load(response)
    except HTTPError as error:
        raise RuntimeError(f"{url}: HTTP {error.code}: {error.read().decode(errors='replace')}") from error


def wait_for(fn, timeout=25):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            result = fn()
            if result:
                return result
        except (OSError, ValueError, RuntimeError) as error:
            last = error
        time.sleep(0.1)
    raise RuntimeError(f"Timed out: {last}")


def dependencies(root):
    packages = root / "packages" if (root / "packages").is_dir() else root
    framework = root / "framework/robonix" if (root / "framework/robonix").is_dir() else root / "robonix"
    return {"framework": framework, "scene": framework / "system/scene",
            "mapping": packages / "service-map-outdoor-rbnx",
            "localization": packages / "service-localization-outdoor-rbnx",
            "navigation": Path(os.environ.get("SIM_NAVIGATION_ROOT", packages / "service-navigation-outdoor-rbnx")).resolve(),
            "gnss": packages / "primitive-g70-gnss-rbnx",
            "imu": packages / "primitive-wit-imu-rbnx"}


def environment(paths, cfg, run):
    env = dict(os.environ)
    for key in ("OUTDOOR_DEPLOY_ROOT", "NTRIP_PASSWORD", "RBNX_PACKAGE_ROOT", "RBNX_INSTANCE_NAME",
                "RBNX_DEPLOY_MANAGED", "ROBONIX_DRIVER_CONTRACT_ID", "CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE"):
        env.pop(key, None)
    pythonpaths = [str(ROOT.parent), str(paths["framework"] / "pylib/robonix-api")]
    # All services must use bindings generated from the same framework version.
    for name in ("scene", "mapping", "localization", "navigation"):
        folder = paths[name]
        pythonpaths.extend([str(folder), str(folder / "rbnx-build/codegen/proto_gen"),
                            str(folder / "rbnx-build/codegen/robonix_mcp_types")])
    pythonpaths.append(env.get("PYTHONPATH", ""))
    env.update(PYTHONPATH=os.pathsep.join(pythonpaths), ROS_DOMAIN_ID=str(cfg["domain"]),
               ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
               ROBONIX_ATLAS=f"127.0.0.1:{cfg['atlas_port']}",
               ROBONIX_PROVIDER_BIND_HOST="127.0.0.1", ROBONIX_PROVIDER_ADVERTISE_HOST="127.0.0.1",
               SCENE_PROFILE="annotate", SCENE_WEB_VIEWER="off", SCENE_GRAPH_ENABLED="false",
               SCENE_OBJECT_MEMORY_ENABLED="false", SCENE_OBJECT_WATCHDOG="0",
               SCENE_MAP_BINDING_WAIT_S="0", SCENE_ANNOTATIONS_DIR=str(run / "annotations"),
               SCENE_MAP_META_DIR=str(run / "meta"), SCENE_GRAPH_CACHE_DIR=str(run / "graph"),
               ROS_LOG_DIR=str(run / "ros_logs"), OPENBLAS_NUM_THREADS="1")
    return env


def prepare_map(run, cfg, imported=None):
    from outdoor_mapping.network import build, version
    from outdoor_mapping.storage import write_json
    from .world import coordinates
    if imported:
        value = json.loads(imported.read_text())
        if value.get("map_type") != "outdoor" or not value.get("network", {}).get("roads"):
            raise ValueError("--map must point to an outdoor map.json with roads")
        cfg["origin"] = value["origin_wgs84"]
    else:
        samples = []
        route = [(i / 4, 0.0, 0x3002) for i in range(13)]
        route += [(3.0, i / 4, 0x3003) for i in range(1, 9)]
        route += [(3.0 + i / 4, 2.0, 0x3002) for i in range(1, 13)]
        for i, (x, y, gait) in enumerate(route):
            lat, lon = coordinates(x, y, cfg["origin"])
            samples.append(dict(latitude=lat, longitude=lon, altitude=0.0, quality=4, grade="fixed",
                                gait_code=gait, gait="stairs" if gait == 0x3003 else "flat",
                                time_s=float(i), utc=str(i), segment=0, index=i))
        built = build(samples, "mock-route", cfg["origin"])
        value = dict(**built, schema_version=1, map_id="mock-demo", map_type="outdoor",
                     frame_id="map", units="meters", example_data=True, recordings=[],
                     parent_map_id=None, note="SIMULATION ONLY", merge_reviewed=True,
                     version=version(built["network"]), saved_at=time.time())
    # Keep imported maps immutable, including their original identity and version.
    from outdoor_mapping.storage import MapStore
    destination = MapStore(run / "maps").path(value["map_id"])
    if destination.exists() and json.loads(destination.read_text()).get("version") != value["version"]:
        raise ValueError("Run directory already contains another map version; use a new --run-dir")
    write_json(destination, value)
    return value


def configurations(paths, cfg, run):
    localizer = {
        "origin": cfg["origin"], "provider_ids": {"gnss": "mock_hardware", "imu": "mock_hardware"},
        "calibrated": True, "antenna_offset_confirmed": True, "feedback_confirmed": True,
        "velocity_accuracy_confirmed": False, "estimator_mode": "eskf", "require_map_reference": True,
        "imu": {"base_from_imu_rotation": [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
                "mounting_confirmed": True, "reference_confirmed": True,
                "magnetic_declination_rad": 0.0, "quality_confirmed": True, "yaw_std_deg": 1.0,
                "course_heading": False, "timeout_s": 0.5},
        "fusion": {"stationary_s": 5.0},
        "output_topics": {"odom": "/mock/localization/odom", "pose": "/mock/localization/pose",
                          "status": "/mock/localization/status", "tf": "/mock/tf", "tf_static": "/mock/tf_static"},
        "run_dir": str(run / "localization"), "http_port": cfg["localization_port"],
        "worker_python": sys.executable,
    }
    navigation = {
        "params_file": str(paths["navigation"] / "config/nav2_params.example.yml"),
        "provider_ids": {"cloud": "mock_hardware", "localization": "localization_outdoor", "chassis": "mock_hardware"},
        "topic_remap": {"tf": "/mock/tf", "tf_static": "/mock/tf_static", "chassis": "/mock/chassis_status"},
        "velocity_output_topic": "/mock/cmd_vel", "gait_output_topic": "/mock/gait_request",
        "gait_switching_enabled": True, "global_grid": {"origin": [-cfg["grid_size_m"] / 2] * 2,
            "width": cfg["grid_size_m"], "height": cfg["grid_size_m"], "resolution": 0.1, "rolling": True},
        "http_port": cfg["navigation_port"], "run_dir": str(run / "navigation"),
        "worker_python": sys.executable, "action_wait_s": 30.0,
        "policy": {"motion_enabled": True, "calibrated": True, "antenna_offset_confirmed": True,
                   "footprint_confirmed": True, "cloud_frame_confirmed": True, "feedback_confirmed": True,
                   "require_chassis_ready": True, "max_speed_mps": cfg["max_speed_mps"],
                   "fixed_arrival_m": cfg["arrival_m"], "float_arrival_m": cfg["arrival_m"],
                   "stopped_linear_mps": 0.10, "stop_hold_s": 1.0},
    }
    return {
        "mock_hardware": cfg,
        "localization_outdoor": localizer,
        "outdoor_mapping": {"data_dir": str(run / "maps"), "gnss_provider_id": "mock_hardware",
            "gait": {"topic": "/mock/actual_gait", "msg_type": "std_msgs/msg/UInt32", "field": "data"},
            "example_data": True, "lifecycle_topic": "/mock/map/lifecycle"},
        "outdoor_execution": navigation,
        "scene": {"perception": {"profile": "annotate"}, "web_host": "127.0.0.1",
            "web_port": cfg["scene_port"], "web_viewer": "off", "observations": [],
            "map_providers": {"outdoor": "outdoor_mapping"},
            "outdoor_navigation": {"run_dir": str(run / "scene_requests"), "execution_enabled": True,
                "localization_config_path": str(run / "localization/config.json"),
                "policy": {"fixed_arrival_m": cfg["arrival_m"], "float_arrival_m": cfg["arrival_m"]}}},
    }


def validate_settings(cfg):
    if type(cfg["domain"]) is not int or not 1 <= cfg["domain"] <= 232 or cfg["domain"] == 80:
        raise ValueError("Simulation domain must be 1..232, excluding real robot domains 0/80")
    ports = [cfg[k] for k in ("atlas_port", "scene_port", "localization_port", "navigation_port", "control_port")]
    if len(set(ports)) != len(ports) or any(type(p) is not int or not 1024 <= p <= 65535 for p in ports):
        raise ValueError("Simulation requires five distinct unprivileged ports")
    for port in ports:
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind(("127.0.0.1", port))
            except OSError as error:
                raise ValueError(f"Simulation port {port} is unavailable: {error}") from error
    for key, lower, upper in (("max_speed_mps", 0, 0.4), ("arrival_m", 0, 3), ("grid_size_m", 10, 80)):
        if type(cfg[key]) not in (float, int) or not math.isfinite(cfg[key]) or not lower < cfg[key] <= upper:
            raise ValueError("Invalid " + key)
    if any(type(cfg["start"][key]) not in (float, int) or not math.isfinite(cfg["start"][key])
           for key in ("x", "y", "yaw_deg")):
        raise ValueError("Invalid initial pose")


def run(args):
    cfg = json.loads(args.config.read_text())
    run_dir = args.run_dir.resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    with (run_dir / "simulation.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return serve(args, cfg, run_dir)


def serve(args, cfg, run):
    (run / "ready.json").unlink(missing_ok=True)
    if args.smoke:
        (run / "smoke-result.json").unlink(missing_ok=True)
    validate_settings(cfg)
    paths = dependencies(Path(os.environ.get("SIM_DEPS_ROOT", ROOT.parent)).resolve())
    env = environment(paths, cfg, run)
    os.environ.update(env)
    sys.path[:0] = env["PYTHONPATH"].split(os.pathsep)
    from robonix_api import ATLAS
    import grpc
    import lifecycle_pb2
    import robonix_contracts_pb2_grpc as contracts

    value = prepare_map(run, cfg, args.map)
    configs = configurations(paths, cfg, run)
    (run / "configurations.json").write_text(json.dumps(configs, ensure_ascii=False, indent=2))
    (run / "session.json").write_text(json.dumps({"simulated": True, "pid": os.getpid(),
        "config": cfg, "map_id": value["map_id"], "dependencies": {k: str(v) for k, v in paths.items()}}, indent=2))
    processes, initialized, logs = [], [], []
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.set())

    def launch(label, command):
        log = (run / (label + ".log")).open("w")
        logs.append(log)
        child_env = dict(env)
        # Scene's existing map-binding client calls this exact lifecycle contract.
        if label == "localization_outdoor":
            child_env["ROBONIX_DRIVER_CONTRACT_ID"] = "robonix/service/localization/driver"
        p = subprocess.Popen(command, cwd=ROOT.parent, env=child_env, stdin=subprocess.DEVNULL,
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((label, p))

    def driver(provider, command, config=None):
        contract = ("robonix/service/localization/driver" if provider == "localization_outdoor"
                    else "robonix/lifecycle/driver")
        def discover():
            found = ATLAS.find_capability(contract_id=contract, provider_id=provider, transport="grpc")
            return found
        caps = wait_for(discover)
        ref = ATLAS.connect_capability(consumer_id="outdoor_simulation", provider_id=caps[0].provider_id,
                                      contract_id=contract, transport="grpc")
        try:
            with grpc.insecure_channel(ref.endpoint, options=[("grpc.enable_http_proxy", 0)]) as channel:
                stub = (contracts.RobonixServiceLocalizationDriverStub if provider == "localization_outdoor"
                        else contracts.RobonixLifecycleDriverStub)
                response = stub(channel).Driver(
                    lifecycle_pb2.Driver_Request(command=command, config_json=json.dumps(config or {})), timeout=25)
                if not response.ok:
                    raise RuntimeError(f"{provider}: {response}")
        finally:
            ref.close()

    try:
        cap_dirs = [paths["framework"] / "capabilities"]
        cap_dirs.extend(paths[k] / "capabilities" for k in ("gnss", "imu", "localization", "mapping", "navigation"))
        launch("atlas", ["robonix-atlas", "--listen", env["ROBONIX_ATLAS"], "--capabilities",
                         ",".join(map(str, cap_dirs))])
        wait_for(lambda: ATLAS.query_contract("robonix/primitive/gnss/fix"))
        modules = [("mock_hardware", "simulation.mock_hardware"),
                   ("localization_outdoor", "localization_outdoor.atlas_bridge"),
                   ("outdoor_mapping", "outdoor_mapping.main"),
                   ("outdoor_execution", "outdoor_nav.atlas_bridge"), ("scene", "scene_service.service")]
        for provider, module in modules:
            launch(provider, [sys.executable, "-m", module])
            driver(provider, 0, configs[provider])
            initialized.append(provider)
            driver(provider, 1)
            print(provider + " ACTIVE", flush=True)
        base = f"http://127.0.0.1:{cfg['scene_port']}"
        wait_for(lambda: request(base + "/api/outdoor/state"))
        loaded = request(base + "/api/maps/load", {"map_type": "outdoor", "map_id": value["map_id"]}, timeout=30)
        if not loaded.get("ok"):
            raise RuntimeError(str(loaded))
        goal = value["network"]["roads"][-1]["points"][-1]
        state = request(base + "/api/outdoor/state")
        if not any(p["name"] == "快递点" for p in state.get("places", [])):
            request(base + "/api/regions", {"kind": "poi", "name": "快递点", "points": [[goal["x"], goal["y"]]],
                    "map_id": value["map_id"], "version": value["version"]})
        wait_for(lambda: request(base + "/api/outdoor/state")["navigation"].get("ready"), timeout=35)
        print(f"SIMULATION READY — no robot connection\nControl: http://127.0.0.1:{cfg['control_port']}/"
              f"\nScene: {base}/outdoor\nLogs: {run}", flush=True)
        (run / "ready.json").write_text(json.dumps({"simulated": True, "scene": base,
            "control": f"http://127.0.0.1:{cfg['control_port']}", "map_id": value["map_id"]}, indent=2))
        if args.smoke:
            from .smoke import check
            check(base, f"http://127.0.0.1:{cfg['control_port']}", run)
            return
        while not stopped.wait(0.5):
            for label, p in processes:
                if p.poll() is not None:
                    raise RuntimeError(f"{label} exited ({p.returncode}); see {run}/{label}.log")
    finally:
        (run / "ready.json").unlink(missing_ok=True)
        # Shut down navigation while its feedback sources are still alive.
        for provider in reversed(initialized):
            try:
                driver(provider, 3)
            except Exception as error:
                print(f"shutdown {provider}: {error}", file=sys.stderr)
        for label, p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGTERM)
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait(timeout=2)
        for log in logs:
            log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/default.json")
    parser.add_argument("--run-dir", type=Path, default=ROOT / "run")
    parser.add_argument("--map", type=Path, help="Copy an existing outdoor map.json into isolated simulation data")
    parser.add_argument("--smoke", action="store_true", help="Check navigation, cancellation and IMU dropout, then exit")
    args = parser.parse_args()
    try:
        run(args)
    except Exception as error:
        raise SystemExit(str(error)) from error


if __name__ == "__main__":
    main()
