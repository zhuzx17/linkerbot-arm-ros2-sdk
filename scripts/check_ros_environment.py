#!/usr/bin/env python3
"""Check a ROS Python environment and kinematics without opening CAN devices."""

import argparse
import importlib
import math
import os
import sys
from importlib.metadata import version
from pathlib import Path

PROFILES = {
    "humble": ("22.04", (3, 10)),
    "jazzy": ("24.04", (3, 12)),
}


def select_profile(distro, os_id, os_version, python_version):
    """Reject mixed OS/ROS/Python combinations before installing dependencies."""
    if distro not in PROFILES:
        raise ValueError("Source ROS 2 Humble or Jazzy before selecting a profile")
    ubuntu_version, expected_python = PROFILES[distro]
    if (os_id, os_version) != ("ubuntu", ubuntu_version):
        raise ValueError(
            f"ROS 2 {distro} requires Ubuntu {ubuntu_version} for this profile"
        )
    if tuple(python_version[:2]) != expected_python:
        raise ValueError(
            f"ROS 2 {distro} requires Python {expected_python[0]}.{expected_python[1]}"
        )
    return Path(__file__).resolve().parents[1] / "constraints" / f"ros-{distro}.txt"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--print-constraints",
        action="store_true",
        help="validate OS, ROS and venv, then print the matching pip constraints path",
    )
    args = parser.parse_args()
    try:
        os_release = {}
        for line in Path("/etc/os-release").read_text().splitlines():
            key, separator, value = line.partition("=")
            if separator:
                os_release[key] = value.strip('"')
        profile = select_profile(
            os.environ.get("ROS_DISTRO"),
            os_release.get("ID"),
            os_release.get("VERSION_ID"),
            sys.version_info,
        )
        if sys.prefix == sys.base_prefix:
            raise ValueError(
                "Activate a venv created with /usr/bin/python3 -m venv --system-site-packages"
            )
        # Require real ROS imports even when tests would otherwise skip them.
        for name in (
            "rclpy",
            "sensor_msgs.msg",
            "std_srvs.srv",
            "launch",
            "launch_ros",
        ):
            importlib.import_module(name)
        if args.print_constraints:
            print(profile)
            return 0

        print(f"ROS_DISTRO: {os.environ['ROS_DISTRO']}")
        print(f"Python: {sys.version.split()[0]} ({sys.executable})")
        for name in ("numpy", "scipy", "pinocchio", "eigenpy"):
            module = importlib.import_module(name)
            print(f"{name}: {module.__version__} ({module.__file__})")
        for distribution, name in (
            ("linkerbot-arm", "linkerbot_arm"),
            ("linkerbot-transport", "linkerbot_transport"),
        ):
            module = importlib.import_module(name)
            print(f"{distribution}: {version(distribution)} ({module.__file__})")

        from linkerbot_arm import A7, P7, A7lite
        from linkerbot_arm.arm.kinetix import ArmKinetix

        for cls in (A7, A7lite, P7):
            for capability in ("home_cancellable", "get_feedback"):
                if not callable(getattr(cls, capability, None)):
                    raise ValueError(
                        f"{cls.__name__} lacks {capability}; install the paired SDK"
                    )
        for model in ("a7", "a7_lite", "p7"):
            for side in ("left", "right"):
                pose = ArmKinetix.from_builtin(model, side).forward_kinematics(
                    [0.0] * 7
                )
                if not all(math.isfinite(value) for value in pose.to_list()):
                    raise ValueError(f"Non-finite FK for {model}/{side}")
                print(f"FK {model}/{side}: {pose.to_list()}")
        print(
            "ROS imports, paired SDK capabilities and all six arm models passed (no CAN opened)"
        )
        return 0
    except (ValueError, ImportError) as error:
        print(f"Environment check failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
