#!/usr/bin/env python3
"""
双臂启动文件。

从 config/arm_config.yaml 读取全部配置（机型、CAN 接口、运动参数等），
自动启动左臂和右臂两个 ArmNode 节点。

用法：
  ros2 launch linker_arm_ros2 dual_arm.launch.py
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def _load_config() -> dict:
    pkg_dir = get_package_share_directory("linker_arm_ros2")
    config_path = os.path.join(pkg_dir, "config", "arm_config.yaml")
    with open(config_path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    cfg = _load_config()

    shared_params = {
        "arm_type": cfg["arm_type"],
        "auto_enable": cfg.get("auto_enable", True),
        "calibrate_zero_on_startup": cfg.get("calibrate_zero", False),
        "publish_rate": float(cfg.get("publish_rate", 100.0)),
        "feedback_timeout": float(cfg.get("feedback_timeout", 0.5)),
        "cmd_timeout": float(cfg.get("cmd_timeout", 3.0)),
        "slow_max_velocity": float(cfg.get("slow_max_velocity", 0.5)),
        "normal_max_velocity": float(cfg.get("normal_max_velocity", 3.0)),
        "startup_delay": float(cfg.get("startup_delay", 6.0)),
        "filter_window": int(cfg.get("filter_window", 5)),
        "position_kp": float(cfg.get("position_kp", 20.0)),
    }

    left_node = Node(
        package="linker_arm_ros2",
        executable="arm_node",
        name="left_arm_node",
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            {
                **shared_params,
                "side": "left",
                "can_interface": cfg.get("left_arm", "can0"),
            }
        ],
    )

    right_node = Node(
        package="linker_arm_ros2",
        executable="arm_node",
        name="right_arm_node",
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            {
                **shared_params,
                "side": "right",
                "can_interface": cfg.get("right_arm", "can1"),
            }
        ],
    )

    return LaunchDescription([left_node, right_node])
