from setuptools import setup
import os
from glob import glob

package_name = "drone_control"

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
    description="Offboard control examples for PX4 drone simulation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "offboard_control = drone_control.offboard_control:main",
            "keyboard_teleop = drone_control.keyboard_teleop:main",
        ],
    },
)
