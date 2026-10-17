# Testing and Docker Validation

## Environment Check

Run inside the ROS venv after installing the SDK:

```bash
ARM_ROS_SOURCE="$HOME/linkerbot_ws/src/linkerbot-arm-ros2"
python "$ARM_ROS_SOURCE/scripts/check_ros_environment.py"
python -m pip check
```

The check imports real ROS modules, reports numerical-library and SDK origins, verifies cancellable homing and feedback capabilities, and computes finite FK with all six left/right A7, A7 Lite, and P7 URDFs. It does not construct arm instances or open CAN.

## Local Regression Tests

```bash
cd "$ARM_ROS_SOURCE"
ARM_CONSTRAINTS=$(python scripts/check_ros_environment.py --print-constraints)
python -m pip install -c "$ARM_CONSTRAINTS" -r requirements/ci-test.txt -e '.[test]'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -p pytest_timeout -p pytest_cov.plugin tests \
  --timeout=60 --cov=linker_arm_ros2 --cov-report=xml:reports/coverage.xml \
  --junitxml=reports/pytest.xml
```

Only `tests/` is collected by default; the physical lifecycle tool is not run. Some tests skip when ROS2 is missing, which is not a passing ROS2 integration result. Timeout and coverage plugins are explicitly enabled to avoid interference from unrelated system pytest plugins.

## Docker Validation

The host needs access to the Docker daemon. Image building needs APT/PyPI network access, but repository credentials are not copied into the image. Run from the ROS2 checkout; the example dependency directories must not already exist:

```bash
cd ~/linkerbot_ws/src/linkerbot-arm-ros2
mkdir -p ../arm-docker-deps
git clone --branch main https://github.com/linker-bot/linkerbot-arm-python-sdk.git ../arm-docker-deps/sdk
ARM_SOURCE_COMMIT=$(sed -n 's/^linkerbot-arm @ .*@//p' requirements/source-versions.txt)
git -C ../arm-docker-deps/sdk checkout "$ARM_SOURCE_COMMIT"
git clone --branch main https://github.com/linker-bot/linkerbot-transport.git ../arm-docker-deps/transport
bash docker/verify.sh humble ../arm-docker-deps/sdk ../arm-docker-deps/transport
bash docker/verify.sh jazzy ../arm-docker-deps/sdk ../arm-docker-deps/transport
```

Test containers use no network, device passthrough, or published ports. Results default to `docker/results/<distribution>/`; the fourth argument selects another directory. They include source commits and changes, build/test logs, JUnit XML, and colcon results. Failures return a nonzero exit code. Temporary containers and build contexts are removed automatically; images are retained.

ROS2 package coverage XML is also saved, and the installed license is checked.
Test tools are pinned in `requirements/ci-test.txt`; the existing numerical
library constraints for each ROS distribution still apply.

Set `ARM_DOCKER_NO_CACHE=1` before the command when APT layers need refreshing. The transport commit in logs records a test snapshot, not an installation constraint.

## Scope of Validation

Checks cover Python/ROS compatibility, dependencies, all six URDF FKs, colcon build, node interpreter, package discovery, both launch descriptions, and ROS2/SDK tests without hardware. Integration regressions use real ROS2 → adapter → A7 SDK → transport, replacing only the CAN driver, to check send failures, feedback age, asynchronous configuration, stop cancellation, and target ordering.

APT packages, image tags, and transport `main` may change. Use the actual logs when rerunning.

Real CAN communication, motion, stopping distance, and calibration require separate physical-device validation. Other CPU architectures require validation on that platform.

## GitHub CI

Pull requests, pushes to `main`, weekly runs, and manual runs execute Ruff
lint/format, ShellCheck, actionlint, and the Humble/Jazzy Docker checks above.
For local lint, run `python -m pip install -r requirements/ci-lint.txt`,
`ruff check .`, and `ruff format --check .`; ShellCheck and actionlint also
check scripts and workflows.

Normal and manual runs use the paired Arm SDK commit in
`requirements/source-versions.txt`; weekly runs use GitHub `main` to detect
compatibility changes in the latest SDK. Transport always uses GitHub `main`.
Logs record the actual source commits. No Gitea access or device credentials
are required. Results in `reports/` are retained as Actions artifacts for
14 days. Coverage is recorded without a minimum threshold. CI does not
publish releases or move hardware.

[Back to overview](environment_setup.md)

## Validate CI in personal forks

Manual CI runs accept `dependency_owner` and `dependency_ref`. By default they
keep using `linker-bot` dependencies and the usual main/paired-commit rules.
For a coordinated change, push all five repositories to matching branches in
your forks, then select your account and that branch in each manual run.
An explicit dependency ref overrides the Arm SDK paired commit for that run.
Both Humble and Jazzy are tested; logs record the actual source revisions.

Enable Actions in new forks. Manual dispatch requires the workflow on the
fork's default branch; setting only the fork's default branch to the test branch
makes this available without changing the upstream default branch.
