"""Verify production SDK loading in fresh processes without opening devices."""

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from linker_arm_ros2.sdk_loader import ensure_sdk_importable

ensure_sdk_importable()
import linkerbot_arm  # noqa: E402 - production discovery must run first.

REPO_ROOT = Path(__file__).resolve().parents[1]
SDK_PACKAGE = Path(linkerbot_arm.__file__).resolve().parent


def copy_real_sdk(source):
    """Give the loader a distinct source tree rather than its installed package."""
    shutil.copytree(
        SDK_PACKAGE,
        source / "linkerbot_arm",
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"),
    )
    return source


def run_without_legacy_sdk(code, tmp_path, *args, repo_root=REPO_ROOT):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    # colcon's quiet/JUnit flags belong to the parent test run. Nested pytest
    # must keep its own output and must not overwrite the parent's XML report.
    env.pop("PYTEST_ADDOPTS", None)
    env.pop("LINKERBOT_ARM_SRC", None)
    env.pop("LINKERBOT_SDK_SRC", None)
    result = subprocess.run(
        [sys.executable, "-B", "-c", code, str(repo_root), *map(str, args)],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return result


REJECT_LEGACY_IMPORTS = """
import importlib.abc
import sys

class RejectLegacySDK(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'linkerbot' or fullname.startswith('linkerbot.'):
            raise AssertionError('Unexpected legacy SDK import: ' + fullname)

sys.meta_path.insert(0, RejectLegacySDK())
sys.path.insert(0, sys.argv[1])
"""

# Block every installed/editable SDK candidate while leaving dependency paths
# untouched. Even after fallback, every linkerbot_arm module must use this copy.
REQUIRE_SELECTED_SDK_SOURCE = """
import importlib.machinery
import pathlib

sdk_source = pathlib.Path(sys.argv[2]).resolve()
assert str(sdk_source) not in sys.path

class RequireSelectedSDKSource(importlib.abc.MetaPathFinder):
    blocked_imports = 0

    def find_spec(self, fullname, path=None, target=None):
        if fullname != 'linkerbot_arm' and not fullname.startswith('linkerbot_arm.'):
            return None
        if str(sdk_source) not in sys.path:
            self.blocked_imports += 1
            raise ModuleNotFoundError("No module named 'linkerbot_arm'", name='linkerbot_arm')
        search_path = [str(sdk_source)] if fullname == 'linkerbot_arm' else path
        spec = importlib.machinery.PathFinder.find_spec(fullname, search_path)
        assert spec is not None and spec.origin, ('Missing selected SDK module', fullname)
        assert pathlib.Path(spec.origin).resolve().is_relative_to(sdk_source), spec.origin
        return spec

source_guard = RequireSelectedSDKSource()
sys.meta_path.insert(0, source_guard)
"""

VERIFY_CANONICAL_SDK = """
from linker_arm_ros2 import sdk_adapter
import linkerbot_arm
from linkerbot_arm import A7, A7lite, P7
# Base SDK dependencies remain available when its wheel shares their site-packages.
import pathlib
import numpy
import scipy

assert sdk_adapter._ARM_CLASSES == {'a7': A7, 'a7lite': A7lite, 'p7': P7}
assert all(cls.__module__.startswith('linkerbot_arm.') for cls in (A7, A7lite, P7))
assert pathlib.Path(numpy.__file__).is_file()
assert pathlib.Path(scipy.__file__).is_file()
assert 'linkerbot' not in sys.modules
assert 'linkerbot.arm' not in sys.modules
"""


@pytest.mark.parametrize("source_fallback", [False, True])
def test_real_sdk_adapter_loads_without_legacy_namespace(tmp_path, source_fallback):
    code = REJECT_LEGACY_IMPORTS
    args = []
    if source_fallback:
        source = copy_real_sdk(tmp_path / "standalone-arm/src")
        args.append(source)
        code += (
            REQUIRE_SELECTED_SDK_SOURCE
            + """
import os
os.environ['LINKERBOT_ARM_SRC'] = str(sdk_source)
"""
        )
    code += VERIFY_CANONICAL_SDK
    if source_fallback:
        code += """
assert source_guard.blocked_imports >= 1
assert pathlib.Path(linkerbot_arm.__file__).resolve().is_relative_to(sdk_source)
"""
    run_without_legacy_sdk(code, tmp_path, *args)


@pytest.mark.parametrize(
    "test_file", ["test_sdk_adapter.py", "test_sdk_integration.py", "test_tools.py"]
)
@pytest.mark.parametrize("layout", ["sibling", "workspace", "environment"])
def test_sdk_test_file_collects_independently_from_source(tmp_path, test_file, layout):
    """Each target must bootstrap itself without test_arm_node collection."""
    workspace = tmp_path / "workspace"
    ros_root = workspace / (
        "linker_arm_ros2" if layout == "sibling" else "src/linker_arm_ros2"
    )
    source = (
        tmp_path / "explicit-arm/src"
        if layout == "environment"
        else ros_root.parent / "linkerbot-arm/src"
    )
    copy_real_sdk(source)
    shutil.copytree(
        REPO_ROOT / "linker_arm_ros2",
        ros_root / "linker_arm_ros2",
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"),
    )
    target = ros_root / "tests" / test_file
    target.parent.mkdir(parents=True)
    shutil.copy2(REPO_ROOT / "tests" / test_file, target)

    code = (
        REJECT_LEGACY_IMPORTS
        + REQUIRE_SELECTED_SDK_SOURCE
        + """
import os
import pytest

if sys.argv[3] == 'environment':
    os.environ['LINKERBOT_ARM_SRC'] = str(sdk_source)
# Only this file is present and collected; no other test can preload the SDK.
result = pytest.main([sys.argv[4], '--collect-only', '-q', '-p', 'no:cacheprovider'])
assert result == 0, result
assert source_guard.blocked_imports >= 1
"""
        + VERIFY_CANONICAL_SDK
        + """
assert pathlib.Path(linkerbot_arm.__file__).resolve().is_relative_to(sdk_source)
"""
    )
    result = run_without_legacy_sdk(
        code, tmp_path, source, layout, target, repo_root=ros_root
    )
    assert "tests collected" in result.stdout


@pytest.mark.parametrize("script", ["calibrate_arm_zero.py", "test_lifecycle.py"])
def test_standalone_help_uses_new_sdk_without_opening_can(tmp_path, script):
    code = (
        REJECT_LEGACY_IMPORTS
        + """
import pathlib
import runpy
from linker_arm_ros2.sdk_loader import ensure_sdk_importable

ensure_sdk_importable()
from linkerbot_arm import A7, A7lite, P7

def forbid_device_creation(*args, **kwargs):
    raise AssertionError('Help must not construct an arm or open CAN')

for cls in (A7, A7lite, P7):
    cls.__init__ = forbid_device_creation

script = pathlib.Path(sys.argv[1]) / 'scripts' / sys.argv[2]
sys.argv = [str(script), '--help']
try:
    runpy.run_path(str(script), run_name='__main__')
except SystemExit as exc:
    assert exc.code == 0
else:
    raise AssertionError('Expected argparse help to exit')

assert 'linkerbot' not in sys.modules
"""
    )
    result = run_without_legacy_sdk(code, tmp_path, script)
    assert "usage:" in result.stdout.lower()
