#!/usr/bin/env python3
"""Adapt linkerbot_arm to the ROS2 arm protocol.

SDK discovery is shared with standalone tools through sdk_loader.
"""

from typing import Literal

from .arm_protocol import ArmProtocol
from .sdk_loader import SDK_ENV_VAR as _SDK_ENV_VAR  # noqa: F401
from .sdk_loader import ensure_sdk_importable as _ensure_sdk_importable  # noqa: F401
from .sdk_loader import find_sdk_source as _find_sdk_source  # noqa: F401
from .sdk_loader import load_sdk

_sdk = load_sdk()
A7, A7lite, P7 = _sdk.A7, _sdk.A7lite, _sdk.P7

_ARM_CLASSES: dict[str, type] = {
    "a7": A7,
    "p7": P7,
    "a7lite": A7lite,
}


class LinkerBotSDKAdapter(ArmProtocol):
    """将 linkerbot_arm SDK (A7 / A7lite / P7) 适配为 ArmProtocol 统一接口。

    直接调用 SDK 创建实例，不做额外的 wake-up / verify / retry。
    SDK 创建失败则直接抛出异常，由 launch 的 respawn 机制处理重启。
    """

    def __init__(
        self,
        arm_type: str,
        side: Literal["left", "right"],
        interface_name: str,
        **kwargs,
    ) -> None:
        arm_type_lower = arm_type.lower()
        cls = _ARM_CLASSES.get(arm_type_lower)
        if cls is None:
            raise ValueError(
                f"不支持的 arm_type '{arm_type}'，可选：{list(_ARM_CLASSES)}"
            )
        if not callable(getattr(cls, "home_cancellable", None)):
            raise RuntimeError(
                "Install linkerbot-arm source with cancellable home support "
                "from requirements/source-versions.txt before starting this node"
            )

        if not callable(getattr(cls, "get_feedback", None)):
            raise RuntimeError(
                "Install linkerbot-arm source with feedback freshness support "
                "from requirements/source-versions.txt before starting this node"
            )

        self._arm = cls(side=side, interface_name=interface_name, **kwargs)
        self._arm_type = arm_type_lower

    # ── 生命周期 ─────────────────────────────────────────────────

    def enable(self) -> None:
        self._arm.enable()

    def disable(self) -> None:
        self._arm.disable()

    def reset_error(self) -> None:
        self._arm.reset_error()

    # ── 运动控制 ─────────────────────────────────────────────────

    def set_angles(self, target_joints: list[float]) -> None:
        self._arm._set_angles(target_joints)

    def move_j(self, target_joints: list[float], *, blocking: bool = True) -> None:
        self._arm.move_j(target_joints, blocking=blocking)

    def home(self, *, blocking: bool = True) -> None:
        self._arm.home(blocking=blocking)

    def home_cancellable(self, cancel_event) -> None:
        method = getattr(self._arm, "home_cancellable", None)
        if method is None:
            raise RuntimeError(
                "Install linkerbot-arm source with cancellable home support"
            )
        method(cancel_event)

    def emergency_stop(self) -> None:
        self._arm.emergency_stop()

    # ── 参数配置 ─────────────────────────────────────────────────

    def set_velocities(self, velocities: list[float]) -> None:
        self._arm.set_velocities(velocities)

    def set_accelerations(self, accelerations: list[float]) -> None:
        self._arm.set_accelerations(accelerations)

    def set_position_kps(self, kps: list[float]) -> None:
        self._arm.set_position_kps(kps)

    # ── 状态读取 ─────────────────────────────────────────────────

    def get_feedback(self):
        return self._arm.get_feedback()

    def get_angles(self) -> list[float]:
        return self._arm.get_angles()

    def get_velocities(self) -> list[float]:
        return self._arm.get_velocities()

    def get_torques(self) -> list[float]:
        return self._arm.get_torques()

    # ── 标定 ─────────────────────────────────────────────────────

    def calibrate_zero(self) -> None:
        self._arm.calibrate_zero()

    # ── 资源释放 ─────────────────────────────────────────────────

    def close(self) -> None:
        if hasattr(self._arm, "close"):
            self._arm.close()

    # ── 扩展访问 ─────────────────────────────────────────────────

    @property
    def raw_sdk(self):
        """返回底层 SDK 实例（A7、A7lite 或 P7），供高级用户直接调用 SDK 特有方法。"""
        return self._arm
