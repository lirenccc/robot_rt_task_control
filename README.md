# ROS 2 RT Task-Control Framework

**English** | [简体中文](README.zh-CN.md)

A pluggable ROS 2 framework for mobile manipulators: task orchestration, safety gating, navigation/manipulation ports, and a soft real-time control loop behind `ros2_control`.

Swap Mock → Nav2 / FollowJointTrajectory / your own `HardwareBus` by changing YAML and plugin class names—not business code.

> **Scope.** This repo ships a landable generic stack (Mock, ReferenceSim, adapters, planners). **EC-Master / IgH are not vendored.** Host masters in a separate module and consume them through a thin `HardwareBus` plugin — see [EtherCAT integration](docs/ETHERCAT_INTEGRATION.md).

## Features

- **Hybrid extension model** — `pluginlib` for hardware & safety; `ProviderRegistry` factories for nav / manip / planner; Action ports for cross-process backends; Launch + profile/runtime YAML for deployment
- **Task stack** — `/task/execute` Action, `TaskOrchestrator` (timeout / cancel), `ITaskPlanner` (`simple` keywords or `yaml_graph` DSL)
- **Safety** — `SafetyGate` with default-deny; policies loaded via `pluginlib` (e.g. `VelocityLimitPolicy`)
- **Control** — `RtControlLoop` + `HardwareBus::exchange` inside a `ros2_control` System; Mock / Skeleton / ReferenceSim buses included
- **Health** — `/robot/health`, `/robot/rt_loop_stats`, FaultManager software gating
- **Soft RT baseline** — documented acceptance test for best-effort 1 kHz user-space loop

## Requirements

| Item | Notes |
|------|--------|
| OS | Ubuntu 22.04 (recommended) |
| ROS 2 | Humble (default); other distros may work with `ROS_DISTRO=…` |
| Build | `colcon`, C++17 |
| Optional | `nav2_msgs` for Nav2 port; external EtherCAT master for fieldbus |

## Quick start

```bash
# 1. Dependencies (apt / pip / rosdep)
./scripts/setup_environment.sh

# 2. Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 3. Run task stack (Mock Action backends by default)
ros2 launch robot_bringup task_stack.launch.py

# Optional: in-process Mock (no separate Action servers)
ros2 launch robot_bringup task_stack_inprocess_mock.launch.py

# 4. Send a task
ros2 action send_goal /task/execute robot_interfaces/action/Task \
  "{instruction: 'go to table and pick red cup'}" --feedback

# 5. Tests
colcon test --packages-select robot_testkit --event-handlers console_direct+
```

Nav2 backend (requires `nav2_msgs` at build time):

```bash
ros2 launch robot_bringup task_stack.launch.py runtime_config:=runtime_nav2.yaml
```

## Documentation

| Document | Description |
|----------|-------------|
| [Architecture](docs/ARCHITECTURE.md) | Extension boundaries, RT split, config model |
| [Integration guide](docs/INTEGRATION_GUIDE.md) | Wire Nav2, trajectories, hardware plugins, checklist |
| [Extension templates](docs/EXTENSION_TEMPLATES.md) | Skeleton bus, ReferenceSim, health, adapters |
| [EtherCAT integration](docs/ETHERCAT_INTEGRATION.md) | External master contract (not vendored here) |
| [Soft RT baseline](docs/SOFT_RT_BASELINE.md) | Soft real-time acceptance criteria |
| [Releasing](docs/RELEASING.md) | Versioning and release checklist |
| [Changelog](CHANGELOG.md) | Notable changes |

中文文档见各页底部链接，或从 [README.zh-CN.md](README.zh-CN.md) 进入。

## Packages (overview)

| Package | Role |
|---------|------|
| `robot_interfaces` | Messages / actions |
| `robot_core_api` / `robot_capability_api` / `robot_hardware_api` | Public C++ contracts |
| `robot_profile` / `robot_runtime` | Typed config + `RuntimeBuilder` / `ProviderRegistry` |
| `robot_task` / `robot_safety` | Orchestrator, planners, SafetyGate |
| `robot_navigation_adapters` / `robot_manipulation_adapters` | Port implementations |
| `robot_ros2_control` / `robot_hardware_plugins` | RT loop + Mock/Skeleton/ReferenceSim |
| `robot_ethercat_adapters` (optional) | Thin EC-Master / IgH `HardwareBus` (needs external master underlay) |
| `robot_bringup` / `robot_description` | Launch, YAML, URDF/xacro |
| `robot_testkit` | Unit / launch / soft-RT tests |

## Configuration model

- **`profile.yaml`** — robot facts only (model, frames, limits, capabilities)
- **`runtime.yaml`** — plugins, provider endpoints, planner type, safety policy class names

Same profile can pair with Mock, sim, or fieldbus runtimes by swapping runtime config and `hardware_bus_plugin`.

## License

Apache License 2.0 — see [LICENSE](LICENSE).
