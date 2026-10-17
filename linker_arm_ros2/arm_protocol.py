#!/usr/bin/env python3
"""
机械臂控制抽象协议（ArmProtocol）。

定义了 ROS2 通信层所需的全部机械臂操作接口，将 ROS2 节点与底层 SDK 实现彻底解耦。

设计目的：
  - linkerbot-arm 持续迭代时，只需更新 sdk_adapter.py 中的适配代码，
    arm_node.py 和 launch 等上层文件无需改动。
  - 若未来需要接入其他品牌/型号的机械臂，只需实现一个新的 Adapter 即可。
  - 部署到数百台机器时，上层 ROS2 逻辑完全一致，仅 Adapter 层处理硬件差异。

接口层次：
  ArmProtocol (抽象接口)
       │
       ├── LinkerBotSDKAdapter  (linkerbot SDK 适配器，见 sdk_adapter.py)
       └── 其他 Adapter ...     (未来可扩展)
"""

import threading
from abc import ABC, abstractmethod


class ArmProtocol(ABC):
    """机械臂控制抽象基类。

    所有方法均为 arm_node.py 实际调用的操作，是 ROS2 通信层与硬件之间的唯一契约。
    新增操作时只需在此处添加抽象方法，然后在对应 Adapter 中实现即可。
    """

    # ── 生命周期控制 ─────────────────────────────────────────────

    @abstractmethod
    def enable(self) -> None:
        """使能所有关节电机，准备接收运动指令。"""

    @abstractmethod
    def disable(self) -> None:
        """失能所有关节电机，电机进入自由状态。"""

    @abstractmethod
    def reset_error(self) -> None:
        """清除所有关节的错误/故障状态。"""

    # ── 运动控制 ─────────────────────────────────────────────────

    @abstractmethod
    def set_angles(self, target_joints: list[float]) -> None:
        """直接下发目标角度到各电机（流式控制用）。

        不触发 motion timer，可高频调用（100 Hz 外部控制指令）。
        电机按已设定的 velocity/acceleration profile 趋近目标。
        """

    @abstractmethod
    def move_j(self, target_joints: list[float], *, blocking: bool = True) -> None:
        """关节空间点到点运动（有 motion guard，运动中禁止再次调用）。"""

    @abstractmethod
    def home(self, *, blocking: bool = True) -> None:
        """回零：将所有关节移动到零位 [0, 0, 0, 0, 0, 0, 0]。"""

    def home_cancellable(self, cancel_event: threading.Event) -> None:
        """生命周期任务回零；适配器必须显式支持合作取消，不能退回阻塞回零。"""
        raise NotImplementedError("This adapter does not support cancellable home")

    @abstractmethod
    def emergency_stop(self) -> None:
        """急停：立即停止运动，锁定当前位姿。"""

    # ── 参数配置 ─────────────────────────────────────────────────

    @abstractmethod
    def set_velocities(self, velocities: list[float]) -> None:
        """设置各关节速度上限 (rad/s)。范围 [0, 50]。"""

    @abstractmethod
    def set_accelerations(self, accelerations: list[float]) -> None:
        """设置各关节加速度上限 (rad/s²)。范围 [1, 50]。"""

    @abstractmethod
    def set_position_kps(self, kps: list[float]) -> None:
        """设置各关节位置环 KP（刚度）。"""

    # ── 状态读取 ─────────────────────────────────────────────────

    def get_feedback(self):
        """Return immutable joint feedback with host monotonic reception times.

        Must support ``joints``, ``connected``, ``oldest_received_at`` and
        ``is_fresh(max_age_s, now=...)`` (see linkerbot_arm.ArmFeedback).
        Never emulate freshness by timestamping the legacy numeric getters.
        """
        raise NotImplementedError("This adapter does not support feedback freshness")

    @abstractmethod
    def get_angles(self) -> list[float]:
        """获取 7 个关节的当前角度 (rad)。"""

    @abstractmethod
    def get_velocities(self) -> list[float]:
        """获取 7 个关节的当前速度 (rad/s)。"""

    @abstractmethod
    def get_torques(self) -> list[float]:
        """获取 7 个关节的当前力矩 (Nm)。"""

    # ── 标定 ─────────────────────────────────────────────────────

    @abstractmethod
    def calibrate_zero(self) -> None:
        """将当前位置设为关节零位。

        不是所有机型都支持此操作，不支持时应抛出 NotImplementedError。
        """

    # ── 资源释放 ─────────────────────────────────────────────────

    @abstractmethod
    def close(self) -> None:
        """关闭连接，释放 CAN 等底层资源。"""
