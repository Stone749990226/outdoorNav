from setuptools import find_packages, setup

setup(name="outdoornav_core", version="1.0.0",
      packages=find_packages(include=["outdoornav", "outdoornav.*"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/outdoornav_core"]),
                  ("share/outdoornav_core", ["package.xml"])],
      install_requires=["setuptools"], zip_safe=False)
