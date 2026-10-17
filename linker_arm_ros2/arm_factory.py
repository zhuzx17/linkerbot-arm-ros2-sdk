#!/usr/bin/env python3
"""
机械臂实例工厂。

负责根据参数创建 ArmProtocol 实例，是 arm_node.py 与具体 Adapter 之间的桥梁。

扩展方式：
  若未来接入新的机械臂品牌/型号，只需：
    1. 在 arm_protocol.py 中确认接口是否覆盖（通常不需改动）
    2. 编写新的 Adapter（如 new_brand_adapter.py）实现 ArmProtocol
    3. 在本文件的 create_arm() 中添加分支即可
  arm_node.py 和 launch 文件完全不需要修改。
"""

from .arm_protocol import ArmProtocol
from .sdk_adapter import LinkerBotSDKAdapter


def create_arm(
    arm_type: str,
    side: str,
    interface_name: str,
    **kwargs,
) -> ArmProtocol:
    """根据参数创建机械臂实例。

    当前支持 linkerbot SDK（A7 / P7 / A7lite），未来可在此扩展其他 Adapter。

    Args:
        arm_type: 机型名称，"a7" / "p7" / "a7lite"。
        side: 臂侧，"left" 或 "right"。
        interface_name: CAN 接口名称，如 "can0"。
        **kwargs: 透传给 Adapter 构造函数（tcp_offset, world_frame 等）。

    Returns:
        ArmProtocol 实例。
    """
    return LinkerBotSDKAdapter(
        arm_type=arm_type,
        side=side,
        interface_name=interface_name,
        **kwargs,
    )
