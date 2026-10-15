"""Process-local Ubuntu/WSL adaptation. Never changes the user's shell or drivers."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from outdoornav.bundle import ROOT

ROS_SETUP = Path("/opt/ros/humble/setup.bash")
INSTALL = ROOT / ".ros-install/setup.bash"
ROS_PACKAGES = (
    "ros_gz_sim",
    "ros_gz_bridge",
    "ros_gz_interfaces",
    "robot_localization",
    "nav2_planner",
    "nav2_navfn_planner",
    "nav2_regulated_pure_pursuit_controller",
    "sensor_msgs_py",
    "nav2_controller",
    "nav2_lifecycle_manager",
    "nav2_costmap_2d",
    "robot_state_publisher",
    "turtlebot3_description",
    "xacro",
    "laser_geometry",
)


def is_wsl():
    return "microsoft" in Path("/proc/sys/kernel/osrelease").read_text().lower()


def ros_environment(installed=False):
    if not ROS_SETUP.is_file():
        raise RuntimeError("ROS 2 Humble is missing: /opt/ros/humble/setup.bash")
    base = os.environ.copy()
    # ROS executables use the distro Python; preserve Node and normal user tools.
    base["PATH"] = "/opt/ros/humble/bin:/usr/bin:/bin:" + base.get("PATH", "")
    base["PYTHONPATH"] = "/usr/lib/python3/dist-packages:" + str(ROOT)
    paths = [str(ROS_SETUP)]
    if installed:
        if not INSTALL.is_file():
            raise RuntimeError("Build is missing; run ./nav build")
        paths.append(str(INSTALL))
    result = subprocess.run(
        [
            "/bin/bash",
            "-c",
            'for file in "$@"; do source "$file" || exit; done; env -0',
            "bash",
            *paths,
        ],
        env=base,
        capture_output=True,
        check=True,
    )
    env = dict(
        item.decode().split("=", 1)
        for item in result.stdout.split(b"\0")
        if b"=" in item
    )
    env.pop("IGN_GAZEBO_SERVER_CONFIG_PATH", None)
    env.update(
        ROS_LOCALHOST_ONLY="1",
        IGN_IP="127.0.0.1",
        GZ_IP="127.0.0.1",
        PYTHONUNBUFFERED="1",
    )
    # Keep middleware identical across all components and CLI tools.
    env.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    # Reliable full costmaps must not block synchronous service responses.
    env.setdefault("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS")
    return env


def data_python():
    for folder in (".venv", "venv"):
        interpreter = ROOT / folder / "bin/python"
        if interpreter.is_file():
            if not shutil.which("uv"):
                raise RuntimeError("A repository venv exists but uv is missing")
            return ["uv", "run", "--no-project", "--python", str(interpreter), "python"]
    if not shutil.which("conda"):
        raise RuntimeError(
            "Create a repository uv venv or install the conda torch environment"
        )
    return ["conda", "run", "--no-capture-output", "-n", "torch", "python"]


def render_environment(env, mode, log_dir):
    env = env.copy()
    env.setdefault("QT_QPA_PLATFORM", "xcb")
    env["QT_X11_NO_MITSHM"] = "1"
    if is_wsl():
        prefix = os.environ.get(
            "WSL_MESA_PREFIX", str(Path.home() / ".local/share/outdoornav/mesa-25.0.7")
        )
        if prefix != "system" and Path(prefix).is_dir():
            lib = Path(prefix) / "usr/lib/x86_64-linux-gnu"
            required = [
                lib / "libGLX_mesa.so.0",
                lib / "dri/d3d12_dri.so",
                Path(prefix) / "usr/share/glvnd/egl_vendor.d/50_mesa.json",
            ]
            if not all(p.is_file() for p in required):
                raise RuntimeError(f"Incomplete private Mesa installation: {prefix}")
            env.update(
                LD_LIBRARY_PATH=str(lib)
                + ":/usr/lib/wsl/lib:"
                + env.get("LD_LIBRARY_PATH", ""),
                LIBGL_DRIVERS_PATH=str(lib / "dri"),
                __EGL_VENDOR_LIBRARY_FILENAMES=str(required[-1]),
                GALLIUM_DRIVER="d3d12",
            )
        env.setdefault("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA")
    if not shutil.which("glxinfo", path=env["PATH"]):
        raise RuntimeError(
            "Install mesa-utils (glxinfo) to check the actual OpenGL context"
        )
    for candidate in ["hardware", "software"] if mode == "auto" else [mode]:
        selected = env.copy()
        selected["LIBGL_ALWAYS_SOFTWARE"] = "1" if candidate == "software" else "0"
        if candidate == "software":
            selected["GALLIUM_DRIVER"] = "llvmpipe"
        check = subprocess.run(
            ["glxinfo"], env=selected, capture_output=True, text=True, timeout=20
        )
        (log_dir / f"render-{candidate}.log").write_text(check.stdout + check.stderr)
        software_context = any(
            name in check.stdout.lower()
            for name in ("llvmpipe", "softpipe", "software rasterizer")
        )
        if (
            check.returncode == 0
            and "GL_ARB_copy_image" in check.stdout
            and not (candidate == "hardware" and software_context)
        ):
            return selected, candidate
    raise RuntimeError(
        "No usable OpenGL context with GL_ARB_copy_image; see render logs and ./nav doctor"
    )


def doctor():
    findings = []

    def record(name, ok, detail):
        findings.append({"component": name, "ok": bool(ok), "detail": detail})

    release = Path("/etc/os-release").read_text()
    record(
        "system",
        "ID=ubuntu\n" in release and 'VERSION_ID="22.04"' in release,
        "WSL2 Ubuntu 22.04" if is_wsl() else "Native Ubuntu 22.04 required",
    )
    for command, hint in [
        ("ign", "Install Gazebo Fortress"),
        ("colcon", "Install python3-colcon-common-extensions"),
        ("node", "Install Node >=22.12"),
        ("npm", "Install npm"),
        ("glxinfo", "Install mesa-utils"),
    ]:
        record(command, shutil.which(command), shutil.which(command) or hint)
    if shutil.which("node"):
        version = subprocess.check_output(["node", "--version"], text=True).strip()
        parts = tuple(int(v) for v in version.lstrip("v").split(".")[:2])
        record("node_version", parts >= (22, 12), version + "; Node >=22.12 required")
    try:
        env = ros_environment()
        for package in ROS_PACKAGES:
            check = subprocess.run(
                ["ros2", "pkg", "prefix", package],
                env=env,
                capture_output=True,
                timeout=10,
            )
            record(
                package,
                check.returncode == 0,
                "available"
                if check.returncode == 0
                else "Install ROS Humble package " + package,
            )
        service_check = subprocess.run(
            [
                "/usr/bin/python3",
                "-c",
                "from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity, SetEntityPose",
            ],
            env=env,
            capture_output=True,
            text=True,
        )
        record(
            "gazebo_entity_services",
            service_check.returncode == 0,
            "available"
            if service_check.returncode == 0
            else "Update ros-humble-ros-gz-interfaces and ros-humble-ros-gz-bridge",
        )
        check = subprocess.run(
            ["glxinfo", "-B"], env=env, capture_output=True, text=True, timeout=10
        )
        record(
            "display",
            check.returncode == 0,
            check.stdout.strip() if check.returncode == 0 else check.stderr.strip(),
        )
        with tempfile.TemporaryDirectory(prefix="outdoornav-render-") as folder:
            _, selected = render_environment(env, "auto", Path(folder))
            record("render", True, "Usable OpenGL context: " + selected)
    except (RuntimeError, subprocess.SubprocessError) as error:
        record("ROS/display", False, str(error))
    try:
        command = data_python()
        check = subprocess.run(
            [*command, "-c", "import numpy, shapely, PIL, yaml"],
            capture_output=True,
            text=True,
            timeout=30,
        )
        record(
            "data_python",
            check.returncode == 0,
            " ".join(command) if check.returncode == 0 else check.stderr.strip(),
        )
    except RuntimeError as error:
        record("data_python", False, str(error))
    from outdoornav.bundle import load_bundle

    try:
        bundle = load_bundle()
        record("scene", True, bundle["scene_version"])
    except (ValueError, OSError) as error:
        record("scene", False, str(error) + "; run ./nav scene build")
    return findings
