from glob import glob
from pathlib import Path
from setuptools import setup

entries = {
    "lynx_lio": "lynx_lio",
    "lynx_localizer": "lynx_localizer",
    "lynx_navigation": "lynx_navigation",
    "lynx_watchdog": "lynx_watchdog",
    "rtk_sim": "rtk_sim",
    "wheel_odom_bridge": "odom_bridge",
    "absolute_localizer": "absolute_localizer",
    "tilt_frames": "tilt_frames",
    "trajectory_logger": "trajectory_logger",
    "depth_points": "depth_points",
    "scan_points": "scan_points",
    "depth_drop": "depth_drop",
    "navigation": "navigation",
    "health": "health",
    "capture": "capture",
    "depth_display": "depth_display",
}
setup(
    name="outdoornav_ros",
    version="1.0.0",
    packages=["outdoornav_ros"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/outdoornav_ros"]),
        ("share/outdoornav_ros", ["package.xml"]),
    ]
    + [
        ("share/outdoornav_ros/" + d, [p for p in glob(d + "/*") if Path(p).is_file()])
        for d in ("launch", "config", "models")
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            f"{k} = outdoornav_ros.{v}:main" for k, v in entries.items()
        ]
    },
)
