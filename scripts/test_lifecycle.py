#!/usr/bin/env python3
"""
纯 Python SDK 生命周期测试脚本（不依赖 ROS2）。

模拟 arm_node.py 的完整生命周期，用于隔离定位 SDK 问题和 ROS2 集成问题：
  1. 创建双臂 SDK 实例
  2. disable → 0.1s → enable → 0.2s（与 arm_node.py 时序一致）
  3. set_velocities(slow) → home_cancellable(shutdown_event)
  4. 状态读取（get_angles / get_velocities / get_torques）
  5. set_angles 流式控制模拟
  6. disable → 0.5s → close

用法：
  python3 scripts/test_lifecycle.py                   # 使用 arm_config.yaml 配置
  python3 scripts/test_lifecycle.py --left-can can1 --right-can can2 --arm-type a7lite
"""

import argparse
import os
import signal
import sys
import threading
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.dirname(SCRIPT_DIR)
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from linker_arm_ros2.sdk_loader import load_sdk  # noqa: E402

_sdk = load_sdk()
A7, A7lite = _sdk.A7, _sdk.A7lite

ARM_CLASSES = {"a7": A7, "a7lite": A7lite}
NUM_JOINTS = 7
SLOW_VEL = 0.5
NORMAL_VEL = 3.0

shutdown_requested = False
_shutdown_event = threading.Event()


def on_signal(sig, frame):
    global shutdown_requested
    shutdown_requested = True
    _shutdown_event.set()
    print(f"\n[SIGNAL] 收到 {signal.Signals(sig).name}，准备关闭...")


signal.signal(signal.SIGINT, on_signal)
signal.signal(signal.SIGTERM, on_signal)


def load_config():
    config_path = os.path.join(SCRIPT_DIR, "..", "config", "arm_config.yaml")
    if os.path.exists(config_path):
        import yaml

        with open(config_path) as f:
            return yaml.safe_load(f)
    return None


def create_arm(arm_type, side, can_interface):
    cls = ARM_CLASSES.get(arm_type.lower())
    if cls is None:
        raise ValueError(f"不支持的 arm_type: {arm_type}")
    if not callable(getattr(cls, "home_cancellable", None)):
        raise RuntimeError(
            "SDK must provide home_cancellable; update the paired arm source"
        )
    print(f"  [{side}] 创建 {arm_type} 实例, CAN={can_interface} ...")
    t0 = time.perf_counter()
    arm = cls(side=side, interface_name=can_interface)
    dt = time.perf_counter() - t0
    print(f"  [{side}] 创建成功 ({dt:.3f}s)")
    return arm


def test_enable(arm, side):
    if shutdown_requested:
        return
    print(f"  [{side}] disable (清除旧状态) ...")
    arm.disable()
    time.sleep(0.1)
    if shutdown_requested:
        return

    print(f"  [{side}] enable ...")
    t0 = time.perf_counter()
    arm.enable()
    dt = time.perf_counter() - t0
    print(f"  [{side}] enable 完成 ({dt:.3f}s)")

    time.sleep(0.2)
    try:
        angles = arm.get_angles()
        print(f"  [{side}] enable 后读取角度: {[f'{a:.4f}' for a in angles]}")
    except Exception as e:
        print(f"  [{side}] enable 后读取角度失败: {e}")


def test_home(arm, side):
    if shutdown_requested:
        return
    print(f"  [{side}] set_velocities({SLOW_VEL}) ...")
    arm.set_velocities([SLOW_VEL] * NUM_JOINTS)

    if shutdown_requested:
        return
    print(f"  [{side}] home_cancellable ...")
    t0 = time.perf_counter()
    arm.home_cancellable(_shutdown_event)
    dt = time.perf_counter() - t0
    result = "取消" if shutdown_requested else "完成"
    print(f"  [{side}] home {result} ({dt:.3f}s)")


def test_state_read(arm, side, count=5):
    print(f"  [{side}] 连续读取 {count} 次状态 ...")
    for i in range(count):
        if shutdown_requested:
            break
        try:
            angles = arm.get_angles()
            arm.get_velocities()
            arm.get_torques()
            print(f"    [{side}] #{i + 1} pos={[f'{a:.3f}' for a in angles]}")
        except Exception as e:
            print(f"    [{side}] #{i + 1} 读取失败: {e}")
        time.sleep(0.01)


def test_streaming(arm, side, duration=3.0, hz=100):
    if shutdown_requested:
        return
    print(f"  [{side}] 流式控制测试: set_angles 到当前位置, {hz}Hz, {duration}s ...")
    try:
        target = arm.get_angles()
    except Exception as e:
        print(f"  [{side}] 获取当前角度失败: {e}")
        return

    arm.set_velocities([NORMAL_VEL] * NUM_JOINTS)
    interval = 1.0 / hz
    count = 0
    errors = 0
    t_start = time.perf_counter()

    while time.perf_counter() - t_start < duration:
        if shutdown_requested:
            break
        try:
            arm._set_angles(target)
            count += 1
        except Exception as e:
            errors += 1
            if errors <= 3:
                print(f"    [{side}] set_angles 错误 #{errors}: {e}")
        time.sleep(interval)

    dt = time.perf_counter() - t_start
    actual_hz = count / dt if dt > 0 else 0
    print(
        f"  [{side}] 流式控制完成: {count} 帧 / {dt:.2f}s = {actual_hz:.1f}Hz, 错误={errors}"
    )


def test_disable(arm, side):
    print(f"  [{side}] disable ...")
    t0 = time.perf_counter()
    arm.disable()
    dt = time.perf_counter() - t0
    print(f"  [{side}] disable 完成 ({dt:.3f}s)")
    print(f"  [{side}] sleep 0.1s 等待 disable 帧发送 ...")
    time.sleep(0.1)


def test_close(arm, side):
    print(f"  [{side}] close ...")
    t0 = time.perf_counter()
    arm.close()
    dt = time.perf_counter() - t0
    print(f"  [{side}] close 完成 ({dt:.3f}s)")


def _cleanup_arms(arms):
    """Attempt every disable and close, even if a previous arm fails cleanup."""
    errors = []
    for side, arm in arms.items():
        try:
            test_disable(arm, side)
        except BaseException as error:
            errors.append(error)
            print(f"  [{side}] 退出失能失败: {error}", file=sys.stderr)
    # Cancellation ends SDK waiting; disable still has to reach the hardware.
    try:
        time.sleep(0.5)
    except BaseException as error:
        errors.append(error)
        print(f"  退出发送等待异常: {error}", file=sys.stderr)
    for side, arm in arms.items():
        try:
            test_close(arm, side)
        except BaseException as error:
            errors.append(error)
            print(f"  [{side}] 关闭连接失败: {error}", file=sys.stderr)
    return errors


def run_single_arm(arm_type, side, can_interface):
    arm = None
    try:
        arm = create_arm(arm_type, side, can_interface)
        test_enable(arm, side)
        if shutdown_requested:
            return arm
        test_home(arm, side)
        if shutdown_requested:
            return arm
        test_state_read(arm, side)
        if shutdown_requested:
            return arm
        test_streaming(arm, side)
        return arm
    except Exception as e:
        print(f"  [{side}] 异常: {e}")
        return arm
    finally:
        if arm is not None:
            _cleanup_arms({side: arm})


def main(argv=None):
    parser = argparse.ArgumentParser(description="SDK 生命周期测试")
    parser.add_argument("--arm-type", type=str, default=None)
    parser.add_argument("--left-can", type=str, default=None)
    parser.add_argument("--right-can", type=str, default=None)
    parser.add_argument(
        "--single", choices=["left", "right"], default=None, help="只测试单臂"
    )
    args = parser.parse_args(argv)

    cfg = load_config()
    arm_type = args.arm_type or (cfg["arm_type"] if cfg else "a7lite")
    left_can = args.left_can or (cfg["left_arm"] if cfg else "can1")
    right_can = args.right_can or (cfg["right_arm"] if cfg else "can2")

    print("=" * 60)
    print("SDK 生命周期测试")
    print(f"  机型: {arm_type}")
    print(f"  左臂: {left_can}")
    print(f"  右臂: {right_can}")
    print(f"  模式: {'单臂 ' + args.single if args.single else '双臂'}")
    print("=" * 60)

    arms = {}

    try:
        # ── Step 1: 创建实例 ──────────────────────────────────────
        print("\n[Step 1] 创建 SDK 实例")
        if args.single != "right" and not shutdown_requested:
            arms["left"] = create_arm(arm_type, "left", left_can)
        if args.single != "left" and not shutdown_requested:
            arms["right"] = create_arm(arm_type, "right", right_can)

        if shutdown_requested:
            print("\n[中断] 跳到关闭流程")
        else:
            # ── Step 2: enable ────────────────────────────────────
            print("\n[Step 2] disable → enable")
            for side, arm in arms.items():
                if shutdown_requested:
                    break
                test_enable(arm, side)

        if not shutdown_requested:
            # ── Step 3: home ──────────────────────────────────────
            print("\n[Step 3] 慢速回零")
            for side, arm in arms.items():
                if shutdown_requested:
                    break
                test_home(arm, side)

        if not shutdown_requested:
            # ── Step 4: 状态读取 ──────────────────────────────────
            print("\n[Step 4] 状态读取")
            for side, arm in arms.items():
                if shutdown_requested:
                    break
                test_state_read(arm, side)

        if not shutdown_requested:
            # ── Step 5: 流式控制 ──────────────────────────────────
            print("\n[Step 5] 流式控制 (set_angles)")
            for side, arm in arms.items():
                if shutdown_requested:
                    break
                test_streaming(arm, side)

    finally:
        print("\n[Cleanup] disable → close")
        cleanup_errors = _cleanup_arms(arms)
    if cleanup_errors:
        raise RuntimeError("Lifecycle cleanup failed; see per-arm errors")

    print("\n" + "=" * 60)
    print("测试已中断并完成清理" if shutdown_requested else "测试完成")
    print("=" * 60)


if __name__ == "__main__":
    main()
