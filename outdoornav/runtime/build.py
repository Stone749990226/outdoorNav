"""Source/dependency fingerprints and reproducible ROS/Web builds."""

import hashlib
import json
import shutil
import subprocess
from outdoornav.bundle import ROOT, load_bundle
from .environment import data_python, ros_environment
from .state import write_json

STAMP = ROOT / ".ros-build/fingerprint.json"


def fingerprints():
    roots = [
        ROOT / "outdoornav",
        ROOT / "ros_ws/src",
        ROOT / "scenes/north_campus",
        ROOT / "web/src",
    ]
    files = [
        ROOT / n
        for n in (
            "setup.py",
            "package.xml",
            "pyproject.toml",
            "web/package.json",
            "web/package-lock.json",
            "web/vite.config.ts",
            "web/tsconfig.json",
        )
    ]
    for root in roots:
        files += [
            p
            for p in root.rglob("*")
            if p.is_file()
            and "__pycache__" not in p.parts
            and p.suffix not in (".pyc", ".png")
        ]
    digest = hashlib.sha256()
    for path in sorted(files):
        digest.update(str(path.relative_to(ROOT)).encode())
        digest.update(path.read_bytes())
    for command in (
        ["/usr/bin/python3", "--version"],
        ["node", "--version"],
        ["npm", "--version"],
    ):
        digest.update(subprocess.check_output(command))
    # Environment package updates invalidate the build as well as source updates.
    if shutil.which("dpkg-query"):
        digest.update(
            subprocess.run(
                ["dpkg-query", "-W", "ros-humble-*", "libignition-gazebo*"],
                capture_output=True,
                check=False,
            ).stdout
        )
    return {"sha256": digest.hexdigest()}


def ensure_scene():
    """Generate missing products and rebuild scenes whose layout or survey changed."""
    scene = ROOT / "scenes/north_campus"
    if not (scene / "survey/road_network.json").is_file():
        subprocess.run(
            [*data_python(), "-m", "outdoornav.generate", "survey"],
            cwd=ROOT,
            check=True,
        )
    network = json.loads((scene / "survey/road_network.json").read_text())
    layout_hash = hashlib.sha256((scene / "layout.json").read_bytes()).hexdigest()
    try:
        bundle = load_bundle()
        current = (
            bundle["route_version"] == network["version"]
            and bundle.get("layout_sha256") == layout_hash
        )
    except (OSError, ValueError):
        current = False
    if not current:
        subprocess.run(
            [*data_python(), "-m", "outdoornav.generate", "scene"], cwd=ROOT, check=True
        )


def build():
    ensure_scene()
    load_bundle()
    env = ros_environment()
    subprocess.run(
        [
            "colcon",
            "--log-base",
            str(ROOT / ".ros-log"),
            "build",
            "--paths",
            str(ROOT),
            str(ROOT / "ros_ws/src/outdoornav_ros"),
            "--build-base",
            str(ROOT / ".ros-build"),
            "--install-base",
            str(ROOT / ".ros-install"),
            "--symlink-install",
        ],
        env=env,
        cwd=ROOT,
        check=True,
    )
    # npm ci always uses the lockfile; build must not retain stale dependency installations.
    subprocess.run(["npm", "ci"], cwd=ROOT / "web", check=True)
    subprocess.run(["npm", "run", "build"], cwd=ROOT / "web", check=True)
    write_json(STAMP, fingerprints())
