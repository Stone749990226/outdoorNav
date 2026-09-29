from glob import glob
from setuptools import setup


setup(
    name="outdoor_nav_bridge",
    version="0.1.0",
    packages=["outdoor_nav_bridge"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/outdoor_nav_bridge"]),
        ("share/outdoor_nav_bridge", ["package.xml"]),
        ("share/outdoor_nav_bridge/launch", glob("launch/*.launch.py")),
        ("share/outdoor_nav_bridge/config", glob("config/*.yaml")),
        ("share/outdoor_nav_bridge/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "bridge = outdoor_nav_bridge.bridge:main",
            "sim_robot = outdoor_nav_bridge.sim_robot:main",
        ]
    },
)
