"""Installation must reject a profile for a different ROS Python/OS ABI."""

import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "check_ros_environment",
    Path(__file__).parents[1] / "scripts/check_ros_environment.py",
)
environment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(environment)


@pytest.mark.parametrize(
    "distro,ubuntu,python",
    [
        ("humble", "22.04", (3, 10, 12)),
        ("jazzy", "24.04", (3, 12, 3)),
    ],
)
def test_supported_profiles_exist(distro, ubuntu, python):
    profile = environment.select_profile(distro, "ubuntu", ubuntu, python)
    assert profile.name == f"ros-{distro}.txt"
    assert profile.is_file()


@pytest.mark.parametrize(
    "distro,os_id,ubuntu,python",
    [
        (None, "ubuntu", "22.04", (3, 10)),
        ("rolling", "ubuntu", "24.04", (3, 12)),
        ("humble", "ubuntu", "22.04", (3, 12)),
        ("jazzy", "ubuntu", "24.04", (3, 10)),
        ("humble", "ubuntu", "24.04", (3, 10)),
        ("jazzy", "ubuntu", "22.04", (3, 12)),
        ("humble", "debian", "22.04", (3, 10)),
    ],
)
def test_mixed_profiles_are_rejected(distro, os_id, ubuntu, python):
    with pytest.raises(ValueError):
        environment.select_profile(distro, os_id, ubuntu, python)
