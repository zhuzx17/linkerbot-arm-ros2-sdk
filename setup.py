#!/usr/bin/env python3
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "linker_arm_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "LICENSE"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "linkerbot-arm>=0.6.0,<0.7.0"],
    extras_require={"test": ["pytest", "pytest-timeout", "PyYAML"]},
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="linker-robot",
    maintainer_email="linker-robot@todo.todo",
    description="ROS2 wrapper for linkerbot-arm to control LinkerBot robotic arms",
    license="MIT",
    license_files=["LICENSE"],
    entry_points={
        "console_scripts": [
            "arm_node = linker_arm_ros2.arm_node:main",
        ],
    },
)
