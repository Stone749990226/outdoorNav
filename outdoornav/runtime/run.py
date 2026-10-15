"""Dependency-ordered startup and ownership-checked shutdown."""

import fcntl
import errno
import json
import os
import signal
import socket
import subprocess
import time
import uuid
from urllib.request import urlopen
from outdoornav.bundle import DEFAULT_BUNDLE, ROOT, load_bundle
from .build import STAMP, build, ensure_scene, fingerprints
from .environment import INSTALL, render_environment, ros_environment
from .processes import identity, matches, stop_owned
from .state import ACTIVE, RUNS, active, write_json


class RenderStartupError(RuntimeError):
    """The GL probe passed, but Gazebo could not create its actual context."""


def check_port(port):
    # WSL port forwarding may release a stopped listener just after its PID exits.
    for attempt in range(11):
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("127.0.0.1", port))
                return
            except OSError as error:
                if error.errno != errno.EADDRINUSE:
                    raise RuntimeError(
                        f"Cannot bind port {port}: {error.strerror}"
                    ) from error
                if attempt == 10:
                    raise RuntimeError(
                        f"Port {port} is busy; inspect its owner with ss -ltnp"
                    ) from error
        time.sleep(0.2)


def domain_available(domain):
    """Local DDS participant ports detect running instances without joining their graph."""
    for port in range(7400 + 250 * domain + 10, 7400 + 250 * domain + 130):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            try:
                probe.bind(("127.0.0.1", port))
            except OSError:
                return False
    return True


def select_domain(requested=None):
    if requested is not None:
        try:
            domain = int(requested)
        except (ValueError, TypeError) as error:
            raise ValueError(
                "ROS_DOMAIN_ID must be an integer within 0..101"
            ) from error
        if not 0 <= domain <= 101:
            raise ValueError(
                "ROS_DOMAIN_ID must be within 0..101 to avoid ephemeral ports"
            )
        if not domain_available(domain):
            raise RuntimeError(
                f"ROS domain {domain} has occupied DDS ports; choose an unused domain"
            )
        return str(domain)
    offset = uuid.uuid4().int % 70
    for index in range(70):
        domain = 10 + (offset + index) % 70
        if domain_available(domain):
            return str(domain)
    raise RuntimeError("No unused local ROS domain within 10..79")


def start(args):
    RUNS.mkdir(parents=True, exist_ok=True)
    lock = (RUNS / "lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        raise RuntimeError(
            "A project run is already active; use ./nav status or ./nav down"
        ) from error
    check_port(args.port)
    check_port(8765)
    domain = select_domain(os.environ.get("ROS_DOMAIN_ID"))
    ensure_scene()
    bundle = load_bundle()
    try:
        built_fingerprint = json.loads(STAMP.read_text())
    except (OSError, ValueError):
        built_fingerprint = None
    if not INSTALL.is_file() or built_fingerprint != fingerprints():
        build()
    run_id = time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8]
    directory = RUNS / run_id
    logs = directory / "logs"
    logs.mkdir(parents=True)
    env, renderer = render_environment(ros_environment(True), args.render, logs)
    # The common entrypoint carries this domain to every component and CLI tool.
    # A new run must not inherit the old demo's hard-coded domain and mixed clocks.
    env["ROS_DOMAIN_ID"] = domain
    env.update(
        OUTDOORNAV_RUN_ID=run_id,
        GZ_PARTITION="outdoornav_" + run_id,
        IGN_PARTITION="outdoornav_" + run_id,
    )
    models = str(DEFAULT_BUNDLE.parent / "models") + ":/opt/ros/humble/share"
    env["IGN_GAZEBO_RESOURCE_PATH"] = models
    env["GZ_SIM_RESOURCE_PATH"] = models
    state = {
        "run_id": run_id,
        "supervisor": identity(os.getpid()),
        "components": {},
        "directory": str(directory),
        "web_url": f"http://localhost:{args.port}",
        "renderer": renderer,
        "scene_version": bundle["scene_version"],
        "route_version": bundle["route_version"],
        "source_fingerprint": json.loads(STAMP.read_text()),
        "git_revision": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip(),
        "ready": False,
        "ros_domain_id": env["ROS_DOMAIN_ID"],
        "rmw_implementation": env["RMW_IMPLEMENTATION"],
        "partition": env["GZ_PARTITION"],
    }
    write_json(ACTIVE, state)
    write_json(directory / "run.json", state)
    write_json(
        directory / "config.json",
        {
            **vars(args),
            "bundle": str(DEFAULT_BUNDLE),
            "ros_domain_id": env["ROS_DOMAIN_ID"],
            "partition": env["GZ_PARTITION"],
        },
    )
    children, handles = {}, []
    stopped = False

    def request_stop(*_):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    def spawn(name, command):
        handle = (logs / f"{name}.log").open("w")
        handles.append(handle)
        child_env = env
        if name == "web":
            # Node may come from nvm; ROS's system-Python PATH must not select
            # an older distro Node than the one checked by doctor and build.
            child_env = {**env, "PATH": os.environ.get("PATH", env["PATH"])}
        child = subprocess.Popen(
            command,
            env=child_env,
            cwd=ROOT,
            stdout=handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        children[name] = child
        state["components"][name] = identity(child.pid)
        write_json(ACTIVE, state)
        write_json(directory / "run.json", state)
        return child

    render_offset, render_tail = 0, ""

    def alive():
        nonlocal render_offset, render_tail
        simulation_log = logs / "simulation.log"
        if simulation_log.is_file():
            with simulation_log.open(errors="replace") as stream:
                stream.seek(render_offset)
                output = render_tail + stream.read()
                render_offset = stream.tell()
            render_tail = output[-256:]
            if any(
                message in output
                for message in (
                    "Failed to create OpenGL context",
                    "D3D12: Removing Device",
                    "Unable to create the rendering window",
                )
            ):
                error = (
                    RenderStartupError
                    if not state["ready"] and renderer == "hardware"
                    else RuntimeError
                )
                raise error(f"Gazebo rendering failed; see {simulation_log}")
        for name, child in children.items():
            if name != "health" and child.poll() is not None:
                raise RuntimeError(
                    f"{name} exited ({child.returncode}); see {logs / name}.log"
                )

    try:
        deadline = time.monotonic() + 180
        spawn(
            "simulation",
            [
                "ros2",
                "launch",
                "outdoornav_ros",
                "simulation.launch.py",
                "bundle:=" + str(DEFAULT_BUNDLE),
                "run_dir:=" + str(directory),
            ],
        )

        def wait_ready(simulation=False):
            remaining = max(1, deadline - time.monotonic())
            probe = spawn(
                "health",
                [
                    "ros2",
                    "run",
                    "outdoornav_ros",
                    "health",
                    "--timeout",
                    str(remaining),
                    *(["--simulation"] if simulation else []),
                ],
            )
            while probe.poll() is None:
                alive()
                if stopped or time.monotonic() > deadline:
                    raise RuntimeError(
                        "Startup interrupted or readiness deadline exceeded"
                    )
                time.sleep(0.2)
            if probe.returncode:
                raise RuntimeError(
                    "Components are not ready; see " + str(logs / "health.log")
                )
            children.pop("health")
            state["components"].pop("health")

        wait_ready(simulation=True)
        spawn(
            "navigation",
            [
                "ros2",
                "launch",
                "outdoornav_ros",
                "navigation.launch.py",
                "bundle:=" + str(DEFAULT_BUNDLE),
                "run_dir:=" + str(directory),
                "rviz:=" + str(args.rviz).lower(),
            ],
        )
        wait_ready()
        spawn(
            "web",
            [
                "npm",
                "--prefix",
                str(ROOT / "web"),
                "run",
                "dev",
                "--",
                "--host",
                "127.0.0.1",
                "--port",
                str(args.port),
                "--strictPort",
            ],
        )
        while time.monotonic() < deadline and not stopped:
            alive()
            try:
                with urlopen(
                    f"http://127.0.0.1:{args.port}/api/status", timeout=1
                ) as response:
                    if json.load(response)["ready"]:
                        break
            except (OSError, ValueError, KeyError):
                time.sleep(0.2)
        else:
            raise RuntimeError("Web proxy did not become ready")
        state["ready"] = True
        write_json(ACTIVE, state)
        write_json(directory / "run.json", state)
        print(
            f"OutdoorNav is ready: {state['web_url']}\nLogs: {logs}\nCtrl+C or ./nav down to stop.",
            flush=True,
        )
        unhealthy_since, checked = None, 0.0
        while not stopped:
            alive()
            now = time.monotonic()
            if now - checked >= 1:
                checked = now
                try:
                    with urlopen(
                        "http://127.0.0.1:8765/api/status", timeout=1
                    ) as response:
                        observed = json.load(response)
                except (OSError, ValueError):
                    observed = {"ready": False, "health": {"api": False}}
                unhealthy_since = (
                    (now if unhealthy_since is None else unhealthy_since)
                    if not observed["ready"]
                    else None
                )
                # Controller cancellation and stop confirmation have a bounded
                # 20-second cleanup budget. Keep the API alive to publish its
                # terminal result before shutting down an unhealthy stack.
                task_stopping = (observed.get("task") or {}).get("state") == "canceling"
                health_deadline = 30 if task_stopping else 10
                if (
                    unhealthy_since is not None
                    and now - unhealthy_since >= health_deadline
                ):
                    state["last_navigation_status"] = observed
                    raise RuntimeError(
                        "Component health lost: "
                        + str(observed.get("health"))
                        + "; see "
                        + str(logs)
                    )
            time.sleep(0.2)
    except Exception as error:
        state["failure"] = str(error)
        raise
    finally:
        try:
            with urlopen("http://127.0.0.1:8765/api/status", timeout=1) as response:
                state["last_navigation_status"] = json.load(response)
        except (OSError, ValueError):
            pass
        stop_owned(run_id)
        for child in children.values():
            try:
                child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass
        for handle in handles:
            handle.close()
        state["ready"] = False
        state["stopped"] = True
        write_json(directory / "run.json", state)
        if ACTIVE.is_file() and json.loads(ACTIVE.read_text()).get("run_id") == run_id:
            ACTIVE.unlink()
        lock.close()


def down():
    state = active()
    if state:
        os.kill(state["supervisor"]["pid"], signal.SIGTERM)
        deadline = time.monotonic() + 20
        while matches(state["supervisor"]) and time.monotonic() < deadline:
            time.sleep(0.1)
        if matches(state["supervisor"]):
            raise RuntimeError("Supervisor did not exit; inspect this run's logs")
        print("OutdoorNav stopped")
    elif ACTIVE.is_file():
        # A crashed supervisor may have left detached children with its exact run token.
        try:
            stale = json.loads(ACTIVE.read_text())
            run_id = stale["run_id"]
            if not isinstance(run_id, str) or not run_id:
                raise ValueError("Invalid run ID")
        except (ValueError, KeyError, TypeError):
            ACTIVE.unlink()
            print("Removed invalid runtime record")
            return
        stop_owned(run_id)
        ACTIVE.unlink()
        print("Cleaned the interrupted run")
    else:
        print("OutdoorNav is stopped")
