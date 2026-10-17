#!/usr/bin/env bash
# Container entry point: no devices, CAN interfaces or host networking required.
set -eo pipefail
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/arm-venv/bin/activate
set -u

mkdir -p /tmp/arm-ws/src /results
cp -r /opt/src/ros2 /tmp/arm-ws/src/linkerbot-arm-ros2
ros2_source=/tmp/arm-ws/src/linkerbot-arm-ros2
python "$ros2_source/scripts/check_ros_environment.py"
python -m pip check

cd /tmp/arm-ws
python /usr/bin/colcon --log-base /results/colcon-log build \
    --packages-select linker_arm_ros2 --symlink-install
node=install/linker_arm_ros2/lib/linker_arm_ros2/arm_node
test "$(head -n 1 "$node")" = "#!$(python -c 'import sys; print(sys.executable)')"
set +u
source install/setup.bash
set -u
python "$(command -v ros2)" pkg prefix linker_arm_ros2
test -f "$(python "$(command -v ros2)" pkg prefix linker_arm_ros2)/share/linker_arm_ros2/LICENSE"
# Argument discovery evaluates launch descriptions without starting arm nodes.
python "$(command -v ros2)" launch linker_arm_ros2 single_arm.launch.py --show-args
python "$(command -v ros2)" launch linker_arm_ros2 dual_arm.launch.py --show-args

python /usr/bin/colcon --log-base /results/colcon-log test \
    --packages-select linker_arm_ros2 --event-handlers console_direct+ \
    --pytest-args -q -p no:cacheprovider -p pytest_timeout -p pytest_cov.plugin --timeout=60 \
    --cov=linker_arm_ros2 --cov-report=term-missing:skip-covered \
    --cov-report=xml:/results/coverage.xml
python /usr/bin/colcon test-result --test-result-base build --verbose
cp build/linker_arm_ros2/pytest.xml /results/ros2.xml

cd /opt/src/sdk
# The SDK config excludes hardware and interactive tests by default.
python -m pytest -q -p no:cacheprovider -p pytest_timeout tests \
    --timeout=60 --junitxml=/results/sdk.xml
