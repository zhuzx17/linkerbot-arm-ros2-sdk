#!/usr/bin/env python3
"""
单臂启动文件。

从 config/arm_config.yaml 读取配置，启动单个 ArmNode 节点。
唯一的 launch 参数是 side（选择左臂或右臂）。

用法：
  # 启动左臂（默认）
  ros2 launch linker_arm_ros2 single_arm.launch.py

  # 启动右臂
  ros2 launch linker_arm_ros2 single_arm.launch.py side:=right
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def _load_config() -> dict:
    pkg_dir = get_package_share_directory("linker_arm_ros2")
    config_path = os.path.join(pkg_dir, "config", "arm_config.yaml")
    with open(config_path) as f:
        return yaml.safe_load(f)


def _launch_setup(context):
    cfg = _load_config()
    side = LaunchConfiguration("side").perform(context)

    can_key = f"{side}_arm"
    can_interface = cfg.get(can_key, "can0")

    node = Node(
        package="linker_arm_ros2",
        executable="arm_node",
        name=f"{side}_arm_node",
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            {
                "arm_type": cfg["arm_type"],
                "side": side,
                "can_interface": can_interface,
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
        ],
    )

    return [node]


def generate_launch_description():
    side_arg = DeclareLaunchArgument(
        "side",
        default_value="left",
        description="Arm side: left or right",
    )

    return LaunchDescription(
        [
            side_arg,
            OpaqueFunction(function=_launch_setup),
        ]
    )
