"""Exercise the actual adapter against autospecs of all three SDK classes."""

from unittest.mock import create_autospec

import pytest

from linker_arm_ros2 import sdk_adapter
from linker_arm_ros2.arm_factory import create_arm
from linker_arm_ros2.arm_protocol import ArmProtocol

# The adapter must discover a source-only SDK before importing it directly.
# isort: split
import linkerbot_arm

OPERATIONS = [
    ("enable", "enable", (), {}),
    ("disable", "disable", (), {}),
    ("reset_error", "reset_error", (), {}),
    ("set_angles", "_set_angles", ([0.1] * 7,), {}),
    ("move_j", "move_j", ([0.1] * 7,), {"blocking": False}),
    ("home", "home", (), {"blocking": False}),
    ("emergency_stop", "emergency_stop", (), {}),
    ("set_velocities", "set_velocities", ([0.5] * 7,), {}),
    ("set_accelerations", "set_accelerations", ([1.0] * 7,), {}),
    ("set_position_kps", "set_position_kps", ([20.0] * 7,), {}),
    ("get_angles", "get_angles", (), {}),
    ("get_velocities", "get_velocities", (), {}),
    ("get_torques", "get_torques", (), {}),
    ("calibrate_zero", "calibrate_zero", (), {}),
    ("close", "close", (), {}),
]


def test_adapter_uses_canonical_sdk_classes():
    assert sdk_adapter.A7 is linkerbot_arm.A7
    assert sdk_adapter.A7lite is linkerbot_arm.A7lite
    assert sdk_adapter.P7 is linkerbot_arm.P7


@pytest.mark.parametrize("arm_type", ["a7", "a7lite", "p7"])
@pytest.mark.parametrize("public,sdk,args,kwargs", OPERATIONS)
def test_adapter_contract(monkeypatch, arm_type, public, sdk, args, kwargs):
    cls = create_autospec(sdk_adapter._ARM_CLASSES[arm_type])
    monkeypatch.setitem(sdk_adapter._ARM_CLASSES, arm_type, cls)
    arm = create_arm(
        arm_type.upper(),
        "right",
        "can1",
        world_frame="maestro",
        tcp_offset=[0.0, 0.1, 0.0],
    )
    assert isinstance(arm, ArmProtocol)
    cls.assert_called_once_with(
        side="right",
        interface_name="can1",
        world_frame="maestro",
        tcp_offset=[0.0, 0.1, 0.0],
    )
    assert arm.raw_sdk is cls.return_value
    method = getattr(cls.return_value, sdk)
    if public.startswith("get_"):
        method.return_value = [0.0] * 7
    result = getattr(arm, public)(*args, **kwargs)
    method.assert_called_once_with(*args, **kwargs)
    if public.startswith("get_"):
        assert result is method.return_value


def test_every_protocol_method_is_covered():
    assert ArmProtocol.__abstractmethods__ == {op[0] for op in OPERATIONS}


@pytest.mark.parametrize("operation,args", [("home", ()), ("move_j", ([0.0] * 7,))])
def test_default_blocking_is_preserved(monkeypatch, operation, args):
    cls = create_autospec(sdk_adapter.A7)
    monkeypatch.setitem(sdk_adapter._ARM_CLASSES, "a7", cls)
    arm = create_arm("a7", "left", "can0")
    getattr(arm, operation)(*args)
    getattr(cls.return_value, operation).assert_called_once_with(*args, blocking=True)


def test_sdk_errors_reach_callers(monkeypatch):
    cls = create_autospec(sdk_adapter.A7)
    cls.return_value._set_angles.side_effect = ValueError("invalid angle")
    monkeypatch.setitem(sdk_adapter._ARM_CLASSES, "a7", cls)
    with pytest.raises(ValueError, match="invalid angle"):
        create_arm("a7", "left", "can0").set_angles([99.0] * 7)


def test_old_sdk_is_rejected_before_opening_or_enabling(monkeypatch):
    class OldSDK:
        def __init__(self, **kwargs):
            pytest.fail("Incompatible SDK must be rejected before opening CAN")

    monkeypatch.setitem(sdk_adapter._ARM_CLASSES, "a7", OldSDK)
    with pytest.raises(RuntimeError, match="cancellable home support"):
        create_arm("a7", "left", "can0")


@pytest.mark.parametrize("arm_type", ["a7", "a7lite", "p7"])
def test_cancellable_home_passes_the_same_signal(monkeypatch, arm_type):
    import threading

    cls = create_autospec(sdk_adapter._ARM_CLASSES[arm_type])
    monkeypatch.setitem(sdk_adapter._ARM_CLASSES, arm_type, cls)
    arm = create_arm(arm_type, "left", "can0")
    cancel = threading.Event()
    arm.home_cancellable(cancel)
    cls.return_value.home_cancellable.assert_called_once_with(cancel)


@pytest.mark.parametrize("arm_type", ["a7", "a7lite", "p7"])
def test_feedback_passthrough(monkeypatch, arm_type):
    cls = create_autospec(sdk_adapter._ARM_CLASSES[arm_type])
    monkeypatch.setitem(sdk_adapter._ARM_CLASSES, arm_type, cls)
    arm = create_arm(arm_type, "left", "can0")
    assert arm.get_feedback() is cls.return_value.get_feedback.return_value
    cls.return_value.get_feedback.assert_called_once_with()
    cls.return_value.get_state.assert_not_called()


def test_sdk_without_feedback_is_rejected_before_can_open(monkeypatch):
    class OldSDK:
        def __init__(self, **kwargs):
            pytest.fail("Incompatible SDK must be rejected before opening CAN")

        def home_cancellable(self, cancel):
            pass

    monkeypatch.setitem(sdk_adapter._ARM_CLASSES, "a7", OldSDK)
    with pytest.raises(RuntimeError, match="feedback freshness support"):
        create_arm("a7", "left", "can0")
