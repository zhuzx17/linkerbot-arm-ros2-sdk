ARG ROS_DISTRO=humble
ARG ROS_IMAGE=ros
FROM ${ROS_IMAGE}:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
RUN case "$ROS_DISTRO" in humble|jazzy) ;; *) exit 1 ;; esac \
    && apt-get update -qq \
    && apt-get install -y --no-install-recommends \
       python3-venv python3-pip python3-colcon-common-extensions \
       ros-${ROS_DISTRO}-pinocchio git \
    && rm -rf /var/lib/apt/lists/*

COPY ros2/constraints /opt/arm-constraints
COPY ros2/requirements/ci-test.txt /opt/arm-ci-test.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m venv --system-site-packages /opt/arm-venv \
    && /opt/arm-venv/bin/python -m pip install --upgrade \
       -c /opt/arm-constraints/ros-${ROS_DISTRO}.txt 'pip<26' setuptools wheel 'packaging>=24.2' \
    && /opt/arm-venv/bin/python -m pip install \
       -c /opt/arm-constraints/ros-${ROS_DISTRO}.txt -r /opt/arm-ci-test.txt

COPY transport /opt/src/transport
COPY sdk /opt/src/sdk
RUN --mount=type=cache,target=/root/.cache/pip \
    /opt/arm-venv/bin/python -m pip install --no-build-isolation \
    -c /opt/arm-constraints/ros-${ROS_DISTRO}.txt \
    '/opt/src/transport[can]' '/opt/src/sdk[test]'

COPY ros2 /opt/src/ros2
RUN --mount=type=cache,target=/root/.cache/pip \
    source /opt/ros/${ROS_DISTRO}/setup.bash \
    && /opt/arm-venv/bin/python /opt/src/ros2/scripts/check_ros_environment.py --print-constraints \
    && /opt/arm-venv/bin/python -m pip install --no-build-isolation \
       -c /opt/arm-constraints/ros-${ROS_DISTRO}.txt \
       '/opt/src/ros2[test]' \
    && /opt/arm-venv/bin/python -m pip check

ENV PATH=/opt/arm-venv/bin:${PATH} \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    PIP_CACHE_DIR=/tmp/arm-pip-cache \
    ROS_LOCALHOST_ONLY=1 \
    ROS_DOMAIN_ID=87 \
    ROS_HOME=/tmp/arm-ros-home
ENTRYPOINT ["bash", "/opt/src/ros2/docker/run_tests.sh"]
