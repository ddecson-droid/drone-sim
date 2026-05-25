from setuptools import setup
import os
from glob import glob

package_name = "drone_utils"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Utility nodes for PX4-Gazebo-ROS2 drone simulation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "px4_tf_broadcaster = drone_utils.px4_tf_broadcaster:main",
            "px4_status_monitor = drone_utils.px4_status_monitor:main",
        ],
    },
)
