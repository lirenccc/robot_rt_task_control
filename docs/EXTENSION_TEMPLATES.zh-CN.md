# 扩展模板说明

本仓库附带的、与厂商无关的构建块：硬件总线骨架、接近量产形态的参考仿真植物、Runtime 生命周期、健康/故障面，以及导航/操作适配器。

这些是**模板与参考实现**，不绑定具体 OEM 或机型。

English: [EXTENSION_TEMPLATES.md](EXTENSION_TEMPLATES.md)

## HardwareBus 模板

| 插件 | 用途 |
|------|------|
| `SkeletonHardwareBus` | 在 `TODO(vendor)` 处填入真实总线 IO |
| `ReferenceSimHardwareBus` | 无现场总线时的序列号 / 看门狗 / fault 注入参考实现 |
| `MockHardwareBus` | CI 与演示用的轻量成功路径 |

通过 URDF `hardware_bus_plugin` 切换（并与 `runtime.yaml` 保持一致）。EtherCAT 主站请在仓外实现薄适配层 — 见 [ETHERCAT_INTEGRATION.zh-CN.md](ETHERCAT_INTEGRATION.zh-CN.md)。

## Runtime 统一生命周期

`RobotRuntime::{configure, activate, deactivate, cleanup}` 由 `RuntimeLifecycle` 按序驱动已注册参与者（当前含 `HealthMonitor`）。

`task_orchestrator_node` 启动时自动执行 `configure` + `activate`。

## 健康 / FAULT

| Topic | 消息 | 说明 |
|-------|------|------|
| `/robot/health` | `robot_interfaces/msg/HardwareHealth` | `mode` ∈ OK / DEGRADED / FAULT / ESTOP；`fault_code`；`allows_motion` |
| `/robot/rt_loop_stats` | 控制栈环统计 | 软实时可观测性 |

`FaultManager` 提供**软件**门控：编排失败可 `raise(Fault)`。硬件急停仍在框架之下。

## Nav2 适配器

- 配置：`providers.navigation.type: nav2`
- 实现：`Nav2NavigationPort` → `nav2_msgs/NavigateToPose`
- 编译：检测到 `nav2_msgs` 时启用；否则 CMake 跳过该源文件，仍可用 `ros_action`

```bash
ros2 launch robot_bringup task_stack.launch.py runtime_config:=runtime_nav2.yaml
```

示例：[`runtime_nav2.yaml`](../src/robot_bringup/config/runtime_nav2.yaml)

## FollowJointTrajectory / MoveIt 风格适配器

- 配置：`providers.manipulation.type: follow_joint_trajectory` 或 `moveit`
- 实现：`FollowJointTrajectoryManipulationPort` → `control_msgs/FollowJointTrajectory`
- **框架内不做 IK：** `ManipulationGoal` 必须带 `joint_positions`（及 `joint_names`）。笛卡尔规划在外部完成后再填入关节目标。

示例：[`runtime_moveit.yaml`](../src/robot_bringup/config/runtime_moveit.yaml)

## 相关文档

- [接入指南](INTEGRATION_GUIDE.zh-CN.md) — 端到端接线与验收清单
- [架构说明](ARCHITECTURE.zh-CN.md) — 模板在栈中的位置
