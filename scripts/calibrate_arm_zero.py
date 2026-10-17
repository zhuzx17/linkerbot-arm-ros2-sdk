#!/usr/bin/env python3
"""A7 / A7 Lite zero calibration tool using linkerbot-arm directly.

Verification status:
    - A7 Lite: 真机验证 2 次通过，未出现异常。
    - A7:      **尚未真机验证**。首次跑 A7 时请单臂 --side 逐个测试，
               确认低速回零阶段安全后再放心使用。
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import yaml

SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent  # src/linker_arm_ros2
WORKSPACE_ROOT = PACKAGE_ROOT.parent.parent  # workspace root
DEFAULT_CONFIG_PATH = PACKAGE_ROOT / "config" / "arm_config.yaml"
TASK_CONFIG_PATH = WORKSPACE_ROOT / "task_config.json"
ARM_NODE_NAMES = ("arm_node", "left_arm_node", "right_arm_node")
SIDES = ("left", "right")
SUPPORTED_ARM_TYPES = ("a7", "a7lite")


@dataclass(frozen=True)
class ArmConfig:
    arm_type: str
    left_arm: str
    right_arm: str

    def can_for(self, side: Literal["left", "right"]) -> str:
        return self.left_arm if side == "left" else self.right_arm


def load_arm_config(config_path: Path) -> ArmConfig:
    if not config_path.is_file():
        raise FileNotFoundError(f"arm config not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    arm_type = str(data.get("arm_type", "")).strip().lower()
    if arm_type not in SUPPORTED_ARM_TYPES:
        raise ValueError(
            f"unsupported arm_type={arm_type or '(empty)'}, "
            f"expected one of {SUPPORTED_ARM_TYPES}"
        )

    config = ArmConfig(
        arm_type=arm_type,
        left_arm=str(data.get("left_arm", "") or "").strip(),
        right_arm=str(data.get("right_arm", "") or "").strip(),
    )

    missing = [side for side in SIDES if not config.can_for(side)]
    if missing:
        raise ValueError(f"missing CAN interface for: {', '.join(missing)}")
    return config


def read_optional_task_arm_type(path: Path = TASK_CONFIG_PATH) -> str:
    if not path.is_file():
        return ""

    try:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except json.JSONDecodeError:
        return ""
    return str(data.get("armType", "")).strip().lower()


def find_config_mismatches(config: ArmConfig) -> list[str]:
    mismatches: list[str] = []
    task_arm_type = read_optional_task_arm_type()
    if task_arm_type and task_arm_type != config.arm_type:
        mismatches.append(
            f"[WARN] task_config.json armType={task_arm_type}，"
            f"但 arm_config.yaml arm_type={config.arm_type}。"
        )
    return mismatches


def ensure_config_consistency(config: ArmConfig, *, ignore_mismatch: bool) -> None:
    mismatches = find_config_mismatches(config)
    for message in mismatches:
        print(message)

    if mismatches and not ignore_mismatch:
        raise RuntimeError(
            "机械臂配置不一致，已拒绝执行标定。请先确认/更新 task_config.json 与 "
            "arm_config.yaml。若你已人工确认 arm_config.yaml 就是当前真实配置，"
            "可加 --ignore-config-mismatch 强制继续。"
        )


def find_running_arm_nodes(node_list_output: str) -> list[str]:
    """Recognize the default and launch controller names in any namespace.

    This graph snapshot cannot exclude renamed nodes, other ROS domains or
    standalone SDK processes; it is not an exclusive lock on a CAN device.
    """
    nodes = {line.strip() for line in node_list_output.splitlines() if line.strip()}
    return sorted(name for name in nodes if name.rsplit("/", 1)[-1] in ARM_NODE_NAMES)


def read_ros_node_list() -> str:
    if shutil.which("ros2") is None:
        raise RuntimeError("无法查询 ROS2 节点：找不到 ros2 命令，已拒绝标定")

    try:
        result = subprocess.run(
            ["ros2", "node", "list"],
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (subprocess.SubprocessError, OSError) as error:
        raise RuntimeError("无法查询 ROS2 节点，已拒绝标定") from error
    if result.returncode != 0:
        raise RuntimeError(
            f"ROS2 节点查询失败（exit={result.returncode}），已拒绝标定: {result.stderr.strip()}"
        )
    return result.stdout


def ensure_arm_nodes_stopped() -> None:
    running = find_running_arm_nodes(read_ros_node_list())
    if running:
        raise RuntimeError(
            "机械臂 ROS2 节点正在运行，标定前请先停止机械臂服务: " + ", ".join(running)
        )


def ensure_sdk_importable() -> None:
    # Allow execution by absolute script path without installing the ROS2 package.
    package_path = str(PACKAGE_ROOT)
    if package_path not in sys.path:
        sys.path.insert(0, package_path)
    from linker_arm_ros2.sdk_loader import ensure_sdk_importable as load_sdk

    load_sdk()


def create_arm(arm_type: str, side: Literal["left", "right"], interface_name: str):
    ensure_sdk_importable()
    from linker_arm_ros2.sdk_loader import load_sdk

    sdk = load_sdk()
    if arm_type == "a7lite":
        return sdk.A7lite(side=side, interface_name=interface_name)
    if arm_type == "a7":
        return sdk.A7(side=side, interface_name=interface_name)
    raise ValueError(f"unsupported arm_type: {arm_type}")


def confirm_zero_pose(side: str, arm_type: str) -> None:
    print("")
    print(f"[{side}] 请手动将 {arm_type.upper()} 机械臂摆放到物理零位。")
    first = input(f"[{side}] 确认机械臂已经处于物理零位？输入 yes 继续: ").strip()
    if first.lower() != "yes":
        raise RuntimeError(f"[{side}] 用户取消标定")

    second = input(f"[{side}] 零位将写入电机参数。再次输入 yes 确认: ").strip()
    if second.lower() != "yes":
        raise RuntimeError(f"[{side}] 用户取消标定")


def verify_zero_angles(
    side: str, angles: list[float], tolerance: float, label: str
) -> None:
    if (
        isinstance(tolerance, bool)
        or not isinstance(tolerance, (int, float))
        or not math.isfinite(tolerance)
        or tolerance <= 0
    ):
        raise ValueError("tolerance must be finite and positive")
    if len(angles) != 7:
        raise RuntimeError(f"[{side}] expected 7 joint angles, got {len(angles)}")
    try:
        finite = all(
            not isinstance(value, bool) and math.isfinite(value) for value in angles
        )
    except (TypeError, OverflowError):
        finite = False
    if not finite:
        raise RuntimeError(f"[{side}] {label}关节角必须全部为有限数值")

    max_abs = max(abs(v) for v in angles)
    formatted = ", ".join(f"{v:.4f}" for v in angles)
    print(f"[{side}] {label}关节角(rad): [{formatted}], max_abs={max_abs:.4f}")
    if max_abs > tolerance:
        raise RuntimeError(
            f"[{side}] {label}验证失败: max_abs={max_abs:.4f} > tolerance={tolerance:.4f}"
        )


def force_disable_arm(side: str, arm, *, attempts: int, wait_s: float) -> None:
    """Send disable commands conservatively before closing the CAN dispatcher."""
    print(f"[{side}] 强制失能所有关节: attempts={attempts}, wait={wait_s:.2f}s")
    for attempt in range(1, attempts + 1):
        try:
            motors = getattr(arm, "_motors", None)
            if motors:
                for motor in motors:
                    motor.disable()
                    time.sleep(0.03)
            else:
                arm.disable()
        except Exception as exc:
            print(f"[{side}] 第 {attempt} 次失能命令异常: {exc}")
        time.sleep(wait_s)
    print(f"[{side}] 已发送强制失能命令")


def close_arm_quietly(arm) -> None:
    try:
        arm.close()
    except Exception:
        pass


def calibrate_zero_arm(
    side: str, arm, *, joint_wait_s: float, flash_wait_s: float
) -> None:
    """Calibrate each motor with spacing so every CAN command can be processed."""
    print(f"[{side}] 逐关节写入零位标定，joint_wait={joint_wait_s:.2f}s")
    try:
        arm.reset_error()
    except Exception as exc:
        print(f"[{side}] reset_error 异常，继续标定: {exc}")
    time.sleep(0.2)

    motors = getattr(arm, "_motors", None)
    if not motors:
        arm.calibrate_zero()
    else:
        total = len(motors)
        for index, motor in enumerate(motors, start=1):
            motor_id = getattr(motor, "_id", "?")
            print(f"[{side}] 写入第 {index}/{total} 关节零位 (motor_id={motor_id})")
            motor.calibrate_zero()
            time.sleep(joint_wait_s)

    print(f"[{side}] 等待 Flash 写入 {flash_wait_s:.1f}s")
    time.sleep(flash_wait_s)


def calibrate_side(
    side: Literal["left", "right"],
    can_interface: str,
    arm_type: str,
    args,
) -> None:
    print(f"[{side}] 创建 {arm_type.upper()} SDK 实例, CAN={can_interface}")
    arm = create_arm(arm_type, side, can_interface)
    try:
        if args.disable_only:
            force_disable_arm(
                side,
                arm,
                attempts=args.disable_attempts,
                wait_s=args.disable_wait,
            )
            return

        print(f"[{side}] 先发送失能命令，准备手动摆放零位")
        force_disable_arm(side, arm, attempts=2, wait_s=0.2)

        confirm_zero_pose(side, arm_type)

        calibrate_zero_arm(
            side,
            arm,
            joint_wait_s=args.calibrate_joint_wait,
            flash_wait_s=args.flash_wait,
        )

        force_disable_arm(
            side,
            arm,
            attempts=args.disable_attempts,
            wait_s=args.disable_wait,
        )
        close_arm_quietly(arm)
        arm = None
        print(f"[{side}] 重新连接 SDK，读取标定后的零位状态")
        time.sleep(args.reconnect_wait)
        arm = create_arm(arm_type, side, can_interface)
        verify_zero_angles(
            side, list(arm.get_angles()), args.tolerance, "标定后当前位置"
        )

        print(f"[{side}] 低速回零验证，velocity={args.home_velocity:.3f} rad/s")
        arm.enable()
        time.sleep(0.2)
        arm.set_velocities([args.home_velocity] * 7)
        arm.home(blocking=True)
        if hasattr(arm, "wait_motion_done"):
            arm.wait_motion_done()

        verify_zero_angles(side, list(arm.get_angles()), args.tolerance, "回零后")
        print(f"[{side}] 零位标定成功")
    finally:
        if arm is not None:
            try:
                force_disable_arm(
                    side,
                    arm,
                    attempts=args.disable_attempts,
                    wait_s=args.disable_wait,
                )
            except Exception:
                pass
            time.sleep(0.5)
            close_arm_quietly(arm)


def parse_args(argv: list[str] | None = None):
    parser = argparse.ArgumentParser(
        description="A7 / A7 Lite 独立零位标定工具。标定前必须停止机械臂 ROS2 节点。"
    )
    parser.add_argument(
        "--side",
        choices=["left", "right", "both"],
        default="both",
        help="选择标定左臂、右臂或双臂，默认 both",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help=f"arm_config.yaml 路径，默认 {DEFAULT_CONFIG_PATH}",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.05,
        help="回零后每个关节允许的最大绝对误差(rad)，默认 0.05",
    )
    parser.add_argument(
        "--home-velocity",
        type=float,
        default=0.3,
        help="回零验证速度(rad/s)，默认 0.3",
    )
    parser.add_argument(
        "--flash-wait",
        type=float,
        default=2.0,
        help="写入零位后等待 Flash 保存的时间(秒)，默认 2.0",
    )
    parser.add_argument(
        "--calibrate-joint-wait",
        type=float,
        default=0.5,
        help="逐关节写入零位时每个关节后的等待时间(秒)，默认 0.5",
    )
    parser.add_argument(
        "--reconnect-wait",
        type=float,
        default=1.0,
        help="写入零位并关闭连接后，重新连接前的等待时间(秒)，默认 1.0",
    )
    parser.add_argument(
        "--ignore-config-mismatch",
        action="store_true",
        help="忽略 task_config.json 与 arm_config.yaml 的不一致并继续",
    )
    parser.add_argument(
        "--disable-attempts",
        type=int,
        default=5,
        help="退出前重复发送失能命令次数，默认 5",
    )
    parser.add_argument(
        "--disable-wait",
        type=float,
        default=0.4,
        help="每轮失能命令后的等待时间(秒)，默认 0.4",
    )
    parser.add_argument(
        "--disable-only",
        action="store_true",
        help="只连接指定机械臂并强制失能，不执行标定/回零",
    )
    args = parser.parse_args(argv)
    if not math.isfinite(args.tolerance) or args.tolerance <= 0:
        parser.error("--tolerance must be finite and positive")
    if not math.isfinite(args.home_velocity) or not 0 < args.home_velocity <= 50:
        parser.error("--home-velocity must be finite and in (0, 50] rad/s")
    for name in (
        "flash_wait",
        "calibrate_joint_wait",
        "reconnect_wait",
        "disable_wait",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and non-negative")
    if args.disable_attempts < 1:
        parser.error("--disable-attempts must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        config = load_arm_config(args.config)
        ensure_config_consistency(config, ignore_mismatch=args.ignore_config_mismatch)
        ensure_arm_nodes_stopped()

        sides = SIDES if args.side == "both" else (args.side,)
        print(f"=== {config.arm_type.upper()} 独立零位标定 ===")
        print(f"配置文件: {args.config}")
        print(f"标定侧: {', '.join(sides)}")
        print(f"误差阈值: {args.tolerance:.4f} rad")
        print("")
        print("!!! 安全提示 !!!")
        print("  - 请所有人退出机械臂运动范围，不要站在扫掠区内")
        print("  - 确保急停按钮/断电开关随手可及")
        print("  - 标定后会低速回零，若出现异常运动请立即急停或断电")
        print("")

        for side in sides:
            calibrate_side(side, config.can_for(side), config.arm_type, args)

        print("")
        print("全部选择的机械臂零位标定完成并通过验证")
        return 0
    except KeyboardInterrupt:
        print("\n用户中断标定")
        return 130
    except Exception as exc:
        print(f"\n标定失败: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
