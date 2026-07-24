# ROS 2 实时任务控制框架

[English](README.md) | **简体中文**

面向移动操作机器人的可插拔 ROS 2 框架：任务编排、安全门控、导航/操作端口，以及位于 `ros2_control` 之后的软实时控制环。

通过修改 YAML 与插件类名即可在 Mock → Nav2 / FollowJointTrajectory / 自研 `HardwareBus` 之间切换，**无需改业务代码**。

> **范围说明。** 本仓库提供可落地的通用栈（Mock、ReferenceSim、适配器、规划器）。**不内置、不 vendoring EC-Master / IgH。** 真实主站请放在独立模块，经薄 `HardwareBus` 插件接入 — 见 [EtherCAT 对接](docs/ETHERCAT_INTEGRATION.zh-CN.md)。

## 特性

- **混合扩展模型** — `pluginlib` 扩展硬件与安全策略；`ProviderRegistry` 工厂扩展导航 / 操作 / 规划器；Action 端口对接跨进程后端；Launch + profile/runtime YAML 负责部署
- **任务栈** — `/task/execute` Action、`TaskOrchestrator`（超时 / 取消）、`ITaskPlanner`（`simple` 关键词或 `yaml_graph` DSL）
- **安全** — `SafetyGate` 默认拒绝；策略经 `pluginlib` 加载（如 `VelocityLimitPolicy`）
- **控制** — `ros2_control` System 内的 `RtControlLoop` + `HardwareBus::exchange`；内置 Mock / Skeleton / ReferenceSim
- **健康** — `/robot/health`、`/robot/rt_loop_stats`、FaultManager 软件门控
- **软实时基线** — 文档化的 1 kHz 用户态尽力调度验收测试

## 环境要求

| 项目 | 说明 |
|------|------|
| 操作系统 | Ubuntu 22.04（推荐） |
| ROS 2 | Humble（默认）；可通过 `ROS_DISTRO=…` 尝试其他发行版 |
| 构建 | `colcon`、C++17 |
| 可选 | Nav2 端口需 `nav2_msgs`；现场总线需外置 EtherCAT 主站 |

## 快速开始

```bash
# 1. 依赖（apt / pip / rosdep）
./scripts/setup_environment.sh

# 2. 编译
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 3. 启动任务栈（默认 Mock Action 后端）
ros2 launch robot_bringup task_stack.launch.py

# 可选：进程内 Mock（不启独立 Action Server）
ros2 launch robot_bringup task_stack_inprocess_mock.launch.py

# 4. 下发任务
ros2 action send_goal /task/execute robot_interfaces/action/Task \
  "{instruction: 'go to table and pick red cup'}" --feedback

# 5. 测试
colcon test --packages-select robot_testkit --event-handlers console_direct+
```

Nav2 后端（需编译期已安装 `nav2_msgs`）：

```bash
ros2 launch robot_bringup task_stack.launch.py runtime_config:=runtime_nav2.yaml
```

## 文档

| 文档 | 说明 |
|------|------|
| [架构说明](docs/ARCHITECTURE.zh-CN.md) | 扩展边界、实时划分、配置模型 |
| [接入指南](docs/INTEGRATION_GUIDE.zh-CN.md) | 对接 Nav2、轨迹、硬件插件与验收清单 |
| [扩展模板](docs/EXTENSION_TEMPLATES.zh-CN.md) | Skeleton 总线、ReferenceSim、健康与适配器 |
| [EtherCAT 对接](docs/ETHERCAT_INTEGRATION.zh-CN.md) | 外置主站契约（本仓不实现） |
| [软实时基线](docs/SOFT_RT_BASELINE.zh-CN.md) | 软实时验收准则 |
| [发布说明](docs/RELEASING.md) | 版本与发布检查清单（英文） |
| [变更日志](CHANGELOG.md) | 重要变更记录（英文） |

English docs: start from [README.md](README.md).

## 包一览

| 包名 | 职责 |
|------|------|
| `robot_interfaces` | 消息 / Action 定义 |
| `robot_core_api` / `robot_capability_api` / `robot_hardware_api` | 对外 C++ 契约 |
| `robot_profile` / `robot_runtime` | 强类型配置 + `RuntimeBuilder` / `ProviderRegistry` |
| `robot_task` / `robot_safety` | 编排器、规划器、SafetyGate |
| `robot_navigation_adapters` / `robot_manipulation_adapters` | 端口实现 |
| `robot_ros2_control` / `robot_hardware_plugins` | RT 环 + Mock/Skeleton/ReferenceSim |
| `robot_ethercat_adapters`（可选） | EC-Master / IgH 薄 `HardwareBus`（需外置主站 underlay） |
| `robot_bringup` / `robot_description` | Launch、YAML、URDF/xacro |
| `robot_testkit` | 单元 / Launch / 软实时测试 |

## 配置模型

- **`profile.yaml`** — 仅机器人事实（型号、坐标系、限位、能力）
- **`runtime.yaml`** — 插件类名、provider endpoint、规划器类型、安全策略类名

同一 Profile 可通过更换 Runtime 配置与 `hardware_bus_plugin`，搭配 Mock、仿真或现场总线。

## 许可证

Apache License 2.0 — 见 [LICENSE](LICENSE)。
