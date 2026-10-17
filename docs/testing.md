# 测试与 Docker 验证

## 环境检查

在已安装 SDK 的 ROS venv 中执行：

```bash
ARM_ROS_SOURCE="$HOME/linkerbot_ws/src/linkerbot-arm-ros2"
python "$ARM_ROS_SOURCE/scripts/check_ros_environment.py"
python -m pip check
```

检查真实 ROS 导入、数值库与 SDK 来源、可取消回零和反馈能力，并用 A7 / A7 Lite / P7 的左右臂六个真实 URDF 计算有限值 FK。不创建机械臂实例，不打开 CAN。

## 本机回归

```bash
cd "$ARM_ROS_SOURCE"
ARM_CONSTRAINTS=$(python scripts/check_ros_environment.py --print-constraints)
python -m pip install -c "$ARM_CONSTRAINTS" -r requirements/ci-test.txt -e '.[test]'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -p pytest_timeout -p pytest_cov.plugin tests \
  --timeout=60 --cov=linker_arm_ros2 --cov-report=xml:reports/coverage.xml \
  --junitxml=reports/pytest.xml
```

默认只收集 `tests/`，不会执行实机生命周期工具。缺少 ROS2 时部分测试会跳过，不能将这种结果当作 ROS2 集成通过。显式启用超时和覆盖率插件，避免系统第三方 pytest 插件干扰。

## Docker 验证

宿主机须可访问 Docker daemon；镜像构建需要 APT/PyPI 网络，仓库凭据不进入镜像。以下命令从 ROS2 仓库根目录执行，示例依赖目录须尚不存在：

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

测试容器无网络、设备透传或端口映射。默认结果位于 `docker/results/<发行版>/`；第四个参数可指定目录。记录源码提交和修改、构建/测试日志、JUnit XML 与 colcon 结果。失败返回非零退出码；临时容器和构建目录自动清理，镜像保留。

同时保存 ROS2 包的覆盖率 XML，检查安装目录中的许可证。测试工具版本固定在
`requirements/ci-test.txt`，继续使用各 ROS 发行版已有的数值库约束。

需要重新获取 APT 层时，在命令前设置 `ARM_DOCKER_NO_CACHE=1`。transport 日志中的提交只记录本次测试快照，不是安装版本约束。

## 验证范围

覆盖 Python/ROS 环境、依赖、六个 URDF FK、colcon 构建、节点解释器、包发现、两个 launch 参数和 ROS2/SDK 无硬件测试。集成回归使用真实 ROS2 → adapter → A7 SDK → transport，仅替换 CAN 驱动，检查发送失败、反馈年龄、异步配置、停止取消及目标顺序。

APT、镜像标签与 transport `main` 可能变化，复跑以实际日志为准。

真实 CAN、运动、停止距离和标定需单独实机验证，其他 CPU 架构需在对应平台验证。

## GitHub CI

PR、`main` 更新、每周及手动运行执行 Ruff lint/format、ShellCheck、actionlint，
并在 Humble 和 Jazzy Docker 环境中执行上述构建及离线验证。
本地 lint 使用 `python -m pip install -r requirements/ci-lint.txt`、
`ruff check .` 和 `ruff format --check .`；ShellCheck 与 actionlint 同时检查脚本和工作流。

普通与手动运行沿用 `requirements/source-versions.txt` 中配对的 Arm SDK 提交；
每周运行改用 GitHub `main` 验证最新 SDK 的兼容性。Transport 始终使用 GitHub `main`，
日志记录实际源码提交，无需 Gitea 访问或设备凭据。
`reports/` 结果作为 Actions artifacts 保留 14 天；覆盖率先记录基线，不设置最低阈值。
CI 不执行发布或实机动作。

[返回总览](environment_setup.md)

## 在个人 fork 验证 CI

Actions 的手动运行入口支持 `dependency_owner` 和 `dependency_ref`：默认仍使用
`linker-bot` 依赖，以及原有的 main/配对提交规则。跨仓库改动可先将 Transport、
Hand/Arm Python SDK 和两个 ROS 仓库推送到个人 fork 的同名测试分支，再统一指定
个人账号与该分支，避免混用上游尚未更新的测试脚本。日志记录实际使用的源码提交。

新 fork 的工作流需要启用 Actions，且默认分支上需要包含工作流文件，才能手动运行；
可以仅将个人 fork 的默认分支设为测试分支。上游默认分支不受影响。

手动运行同时验证 Humble 与 Jazzy。Arm ROS2 指定 `dependency_ref` 时会覆盖 SDK 配对提交，仅用于明确选择的兼容验证。
