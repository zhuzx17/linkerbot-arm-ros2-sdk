"""Real rclpy, messages and adapter with fake SDK arms; never open a CAN bus."""

import time
from unittest.mock import create_autospec

import pytest
from linkerbot_arm import ArmFeedback, JointFeedback

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor  # noqa: E402 - after importorskip
from sensor_msgs.msg import JointState  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from linker_arm_ros2 import arm_node, sdk_adapter  # noqa: E402


@pytest.mark.parametrize("arm_type", ["a7", "a7lite", "p7"])
@pytest.mark.parametrize("side,channel", [("left", "can0"), ("right", "can1")])
def test_node_adapter_sdk_call_chain(
    monkeypatch, arm_type, side, channel, wait_for_tasks
):
    cls = create_autospec(sdk_adapter._ARM_CLASSES[arm_type])
    arm = cls.return_value
    arm.get_angles.return_value = [0.0] * 7
    arm.get_velocities.return_value = [0.0] * 7
    arm.get_torques.return_value = [0.0] * 7
    received = time.monotonic()
    arm.get_feedback.return_value = ArmFeedback(
        (JointFeedback(0, 0, 0, received, received, received),) * 7, True
    )
    monkeypatch.setitem(sdk_adapter._ARM_CLASSES, arm_type, cls)
    monkeypatch.setattr(arm_node.time, "sleep", lambda duration: None)
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "auto_enable:=false",
            "-p",
            f"arm_type:={arm_type}",
            "-p",
            f"side:={side}",
            "-p",
            f"can_interface:={channel}",
        ]
    )
    node = None
    try:
        node = arm_node.ArmNode()
        cls.assert_called_once_with(
            side=side,
            interface_name=channel,
            world_frame="urdf",
            tcp_offset=[0.0, 0.0, 0.0],
        )
        assert node.arm.raw_sdk is arm
        assert node.state_pub.topic_name == f"/{side}_arm_joint_state"
        assert node.cmd_sub.topic_name == f"/{side}_arm_joint_control"
        node._publish_state()
        arm.get_feedback.assert_called_once()
        arm.get_angles.assert_not_called()
        arm.get_velocities.assert_not_called()
        arm.get_torques.assert_not_called()
        node.enabled = True
        message = JointState()
        message.position = [0.1] * 7
        node._cmd_callback(message)
        wait_for_tasks(node)
        arm._set_angles.assert_called_once_with([0.1] * 7)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            future = executor.create_task(
                node._srv_home(Trigger.Request(), Trigger.Response())
            )
            executor.spin_until_future_complete(future, timeout_sec=2.0)
            assert future.result().success
            arm.home_cancellable.assert_called_once()
            future = executor.create_task(
                node._srv_emergency_stop(Trigger.Request(), Trigger.Response())
            )
            executor.spin_until_future_complete(future, timeout_sec=2.0)
            assert future.result().success
            arm.emergency_stop.assert_called_once()
        finally:
            executor.shutdown()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    arm.disable.assert_called_once()
    arm.close.assert_called_once()
