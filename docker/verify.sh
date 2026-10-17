#!/usr/bin/env bash
# Build/test local sources; repository credentials never enter the image.
set -euo pipefail
if [[ $# -lt 3 || $# -gt 4 ]]; then
    echo "Usage: bash docker/verify.sh humble|jazzy SDK_CHECKOUT TRANSPORT_CHECKOUT [RESULTS_DIR]" >&2
    exit 2
fi
ros_distro=$1
case "$ros_distro" in humble|jazzy) ;; *) echo "Choose humble or jazzy" >&2; exit 2 ;; esac
ros2_source=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
sdk_source=$(cd "$2" && pwd)
transport_source=$(cd "$3" && pwd)
results_dir=${4:-"$ros2_source/docker/results/$ros_distro"}
mkdir -p "$results_dir"
results_dir=$(cd "$results_dir" && pwd)
context_dir=$(mktemp -d /tmp/linkerbot-arm-docker.XXXXXX)
container_name="linkerbot-arm-${ros_distro}-test-$$"
cleanup() {
    docker rm -f "$container_name" >/dev/null 2>&1 || true
    rm -rf "$context_dir"
}
trap cleanup EXIT

mkdir -p "$context_dir"/{ros2,sdk,transport}
# Explicit source allowlists avoid copying .git, local environments or credentials.
for item in setup.py setup.cfg package.xml LICENSE resource linker_arm_ros2 launch config scripts tests constraints requirements; do
    cp -r "$ros2_source/$item" "$context_dir/ros2/"
done
mkdir -p "$context_dir/ros2/docker"
cp "$ros2_source/docker/run_tests.sh" "$context_dir/ros2/docker/"
for item in pyproject.toml README.md LICENSE src tests tools; do
    cp -r "$sdk_source/$item" "$context_dir/sdk/"
done
for item in pyproject.toml README.md LICENSE src; do
    cp -r "$transport_source/$item" "$context_dir/transport/"
done
# Ignore local Python build metadata and bytecode even inside allowed directories.
find "$context_dir" -type d \( -name __pycache__ -o -name '*.egg-info' -o -name results \) \
    -prune -exec rm -rf {} +
image_name="linkerbot-arm-ros2-test:${ros_distro}"
build_flags=()
if [[ ${ARM_DOCKER_NO_CACHE:-0} == 1 ]]; then
    build_flags+=(--no-cache)
fi
{
    for source_dir in "$ros2_source" "$sdk_source" "$transport_source"; do
        git -C "$source_dir" log -1 --format='%H %s'
        git -C "$source_dir" status --short
    done
} | tee "$results_dir/sources.log"
docker build "${build_flags[@]}" --build-arg "ROS_DISTRO=$ros_distro" \
    -t "$image_name" -f "$ros2_source/docker/Dockerfile" "$context_dir" \
    2>&1 | tee "$results_dir/build.log"
# Tests run without network access, device passthrough or published ports.
docker run --rm --name "$container_name" --network none \
    --user "$(id -u):$(id -g)" \
    --mount "type=bind,src=$results_dir,dst=/results" \
    "$image_name" 2>&1 | tee "$results_dir/tests.log"
