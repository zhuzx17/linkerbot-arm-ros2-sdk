"""Real ROS messages/node and SDK decoders with simulated feedback and clocks."""

import math
import struct
from dataclasses import replace
from unittest.mock import Mock

import can
import numpy as np
import pytest

pytest.importorskip("rclpy")
import rclpy
from linkerbot_arm import A7, P7, A7lite, ArmFeedback, JointFeedback
from linkerbot_arm.arm.a7.motor import A7Motor
from linkerbot_arm.arm.a7_lite.motor import A7liteMotor
from linkerbot_arm.arm.p7.motor_v2 import MotorV2
from rclpy.time import Time

from linker_arm_ros2 import arm_node
from linker_arm_ros2.feedback_clock import FeedbackClock


def sample(received, **kwargs):
    return ArmFeedback(
        (JointFeedback(1, 2, 3, received, received, received),) * 7,
        **({"connected": True} | kwargs),
    )


@pytest.fixture
def node(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("Freshness tests must not open physical CAN")

    monkeypatch.setattr(can, "Bus", forbidden)
    monkeypatch.setattr(can.interface, "Bus", forbidden)
    fake = Mock()
    monkeypatch.setattr(arm_node, "create_arm", lambda **kwargs: fake)
    rclpy.init(args=["--ros-args", "-p", "auto_enable:=false"])
    instance = arm_node.ArmNode()
    instance.state_pub.publish = Mock()
    instance.host_time = 100.0
    instance.ros_ns = 10_000_000_000
    clock = Mock(ros_time_is_active=False)
    clock.now.side_effect = lambda: Time(nanoseconds=instance.ros_ns)
    instance.test_clock = clock
    monkeypatch.setattr(instance, "get_clock", lambda: clock)
    monkeypatch.setattr(instance, "get_logger", lambda: Mock())
    monkeypatch.setattr(arm_node.time, "monotonic", lambda: instance.host_time)
    # Seed the first clock observation, before the next received sample.
    fake.get_feedback.return_value = sample(99.9)
    instance._publish_state()
    assert instance.state_pub.publish.call_count == 0
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def advance(node, delta=0.01, ros_delta=None):
    node.host_time += delta
    node.ros_ns += round((delta if ros_delta is None else ros_delta) * 1e9)


def stamp(node):
    msg = node.state_pub.publish.call_args.args[0]
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def test_repeated_cache_keeps_stamp_then_expires_and_recovers(node):
    node.arm.get_feedback.return_value = sample(100.005)
    advance(node)
    node._publish_state()
    assert node.state_pub.publish.call_count == 1, node._feedback_problem
    first_stamp = stamp(node)
    msg = node.state_pub.publish.call_args.args[0]
    assert list(msg.position) == [1.0] * 7
    assert list(msg.velocity) == [2.0] * 7
    assert list(msg.effort) == [3.0] * 7
    assert first_stamp == 10_000_000_000  # clock observation preceding reception
    advance(node, 0.2)
    node._publish_state()
    assert stamp(node) == first_stamp
    assert node.state_pub.publish.call_count == 2
    advance(node, 0.31)
    node._publish_state()
    assert node.state_pub.publish.call_count == 2
    assert "stale" in node._feedback_problem
    node.arm.get_feedback.return_value = sample(node.host_time)
    advance(node)
    node._publish_state()
    assert node.state_pub.publish.call_count == 3
    assert stamp(node) > first_stamp
    assert node._feedback_problem is None
    node.arm.get_angles.assert_not_called()
    node.arm.get_state.assert_not_called()


@pytest.mark.parametrize("number", [int, float, np.float32])
def test_numeric_feedback_is_published_as_ros_float64(node, number):
    joint = JointFeedback(number(1), number(2), number(3), 100.005, 100.005, 100.005)
    node.arm.get_feedback.return_value = ArmFeedback((joint,) * 7, True)
    advance(node)
    node._publish_state()
    assert node.state_pub.publish.call_count == 1, node._feedback_problem
    message = node.state_pub.publish.call_args.args[0]
    for field, expected in (
        (message.position, 1.0),
        (message.velocity, 2.0),
        (message.effort, 3.0),
    ):
        assert list(field) == [expected] * 7
        assert all(isinstance(value, float) for value in field)


@pytest.mark.parametrize(
    "fault",
    [
        "closed",
        "short",
        "angle_stale",
        "velocity_stale",
        "torque_stale",
        "nan",
        "missing",
        "future",
        "error",
    ],
)
def test_invalid_feedback_suppresses_complete_message(node, fault):
    received = sample(100.005)
    joint = received.joints[-1]
    if fault == "closed":
        received = replace(received, connected=False)
    elif fault == "short":
        received = replace(received, joints=received.joints[:-1])
    elif fault.endswith("_stale"):
        joint = replace(joint, **{fault.removesuffix("_stale") + "_received_at": 99.0})
    elif fault == "nan":
        joint = replace(joint, torque=math.nan)
    elif fault == "missing":
        joint = replace(joint, velocity_received_at=None)
    elif fault == "future":
        joint = replace(joint, angle_received_at=101.0)
    elif fault == "error":
        node.arm.get_feedback.side_effect = RuntimeError("receive failure")
    if len(received.joints) == 7:
        received = replace(received, joints=received.joints[:-1] + (joint,))
    node.arm.get_feedback.return_value = received
    advance(node)
    node._publish_state()
    node.state_pub.publish.assert_not_called()
    assert node._feedback_problem


@pytest.mark.parametrize("simulated,jump", [(False, 5.0), (False, -5.0), (True, -5.0)])
def test_clock_jumps_require_new_feedback_from_every_field(node, simulated, jump):
    node.test_clock.ros_time_is_active = simulated
    # Switching clock type resets history, then establish a new sample.
    advance(node)
    node._publish_state()
    node.arm.get_feedback.return_value = sample(node.host_time + 0.005)
    advance(node)
    node._publish_state()
    count = node.state_pub.publish.call_count
    assert count == 1
    advance(node, 0.01, jump)
    node._publish_state()
    assert node.state_pub.publish.call_count == count
    assert "clock epoch" in node._feedback_problem
    advance(node)
    node._publish_state()  # unchanged pre-jump cache is still refused
    assert node.state_pub.publish.call_count == count
    node.arm.get_feedback.return_value = sample(node.host_time + 0.005)
    advance(node)
    node._publish_state()
    assert node.state_pub.publish.call_count == count + 1


def test_paused_sim_time_does_not_pause_feedback_expiry(node):
    node.test_clock.ros_time_is_active = True
    advance(node)
    node._publish_state()
    node.arm.get_feedback.return_value = sample(node.host_time + 0.005)
    advance(node, ros_delta=0.0)
    node._publish_state()
    first = stamp(node)
    advance(node, 0.6, 0.0)
    node._publish_state()
    assert node.state_pub.publish.call_count == 1
    assert "stale" in node._feedback_problem
    node.arm.get_feedback.return_value = sample(node.host_time)
    advance(node, 0.01, 0.0)
    node._publish_state()
    assert node.state_pub.publish.call_count == 2
    assert stamp(node) == first  # paused ROS time cannot be made to advance


def test_connection_closed_while_reading_does_not_publish(node):
    def read_then_close():
        node._connection_closed = True
        return sample(100.005)

    node.arm.get_feedback.side_effect = read_then_close
    advance(node)
    node._publish_state()
    node.state_pub.publish.assert_not_called()


@pytest.mark.parametrize("cls", [A7, A7lite, P7])
def test_real_sdk_decoder_to_ros_chain_and_bus_fault(node, cls):
    sdk = cls.__new__(cls)
    sdk._closed = False
    sdk._bus_error = None
    kinds = [A7liteMotor] * 7 if cls is A7lite else [A7Motor] * 7
    if cls is P7:
        kinds[-2:] = [MotorV2] * 2
    sdk._motors = [kind(mid, Mock()) for kind, mid in zip(kinds, range(61, 68))]
    advance(node, 0.005)
    for motor in sdk._motors:
        if isinstance(motor, A7Motor):
            for field in [0x06, 0x05, 0x03]:
                motor._on_message(
                    can.Message(
                        arbitration_id=motor._id,
                        is_extended_id=False,
                        data=bytes([field]) + struct.pack("<f", 0.5),
                    )
                )
        elif isinstance(motor, A7liteMotor):
            motor._on_message(
                can.Message(
                    arbitration_id=0x020000FD | (motor._id << 8),
                    is_extended_id=True,
                    data=bytes.fromhex("ffffffffffff012c"),
                )
            )
        else:
            motor._on_message(
                can.Message(
                    arbitration_id=motor._id,
                    is_extended_id=False,
                    data=bytes.fromhex("01ffffffff00071e"),
                )
            )
    node.arm.get_feedback.side_effect = sdk.get_feedback
    advance(node, 0.005)
    node._publish_state()
    assert node.state_pub.publish.call_count == 1
    sdk._on_bus_error(RuntimeError("lost bus"))
    advance(node)
    node._publish_state()
    assert node.state_pub.publish.call_count == 1
    assert "connection failed" in node._feedback_problem


def test_clock_history_is_bounded_and_does_not_freshen_repeats():
    clock = FeedbackClock(0.5)
    for i in range(10000):
        host = 100.0 + i / 100
        clock.observe(host, round(host * 1e9), False)
        if i >= 49:
            assert clock.stamp_ns(host - 0.49) <= (host - 0.49) * 1e9 + 1
    assert len(clock._ticks) <= 52
    clock.observe(200.0, 0, True)
    assert clock.stamp_ns(200.0) is None


def test_scaled_sim_time_uses_observations_not_wall_age():
    clock = FeedbackClock(0.5)
    clock.observe(100.0, 10_000_000_000, True)
    clock.observe(100.1, 10_200_000_000, True)
    assert clock.stamp_ns(100.05) == 10_000_000_000
    assert clock.stamp_ns(100.1) == 10_200_000_000


def test_faster_fields_do_not_advance_the_oldest_sample_stamp(node):
    node.arm.get_feedback.return_value = sample(100.005)
    advance(node)
    node._publish_state()
    first = stamp(node)
    advance(node, 0.1)
    updated = sample(node.host_time)
    last = replace(updated.joints[-1], torque_received_at=100.005)
    node.arm.get_feedback.return_value = replace(
        updated, joints=updated.joints[:-1] + (last,)
    )
    node._publish_state()
    assert stamp(node) == first


@pytest.mark.parametrize("timeout", ["0.0", "-0.5", ".nan", ".inf"])
def test_invalid_timeout_is_rejected_before_opening_sdk(monkeypatch, timeout):
    constructor = Mock(side_effect=AssertionError("Unexpected SDK construction"))
    monkeypatch.setattr(arm_node, "create_arm", constructor)
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "auto_enable:=false",
            "-p",
            "feedback_timeout:=" + timeout,
        ]
    )
    try:
        with pytest.raises(ValueError, match="feedback_timeout"):
            arm_node.ArmNode()
        constructor.assert_not_called()
    finally:
        rclpy.shutdown()


def test_large_forward_sim_steps_keep_old_stamp_and_accept_new_samples(node):
    node.test_clock.ros_time_is_active = True
    advance(node)
    node._publish_state()
    node.arm.get_feedback.return_value = sample(node.host_time + 0.005)
    advance(node)
    node._publish_state()
    first = stamp(node)
    advance(node, 0.01, 5.0)
    node._publish_state()
    assert stamp(node) == first  # old sample does not acquire the forward step
    for _ in range(5):
        previous_ros = node.ros_ns
        node.arm.get_feedback.return_value = sample(node.host_time + 0.005)
        advance(node, 0.01, 5.0)
        node._publish_state()
        assert node._feedback_problem is None
        assert stamp(node) == previous_ros


def test_slow_sim_callbacks_accept_fresh_samples_without_starvation():
    clock = FeedbackClock(0.5)
    for i in range(10):
        clock.observe(100 + i, (10 + i) * 1_000_000_000, True)
        if i:
            # A recent sample can map conservatively to the preceding tick,
            # even when clock observations are slower than the age threshold.
            assert clock.stamp_ns(100 + i - 0.1) == (9 + i) * 1_000_000_000
