# 架构说明

本文描述 ROS 2 实时任务控制框架的扩展模型、实时边界划分、数据流与配置契约。

English: [ARCHITECTURE.md](ARCHITECTURE.md)

## 设计目标

- **任务编排与安全门控** 放在非实时 ROS 2 侧
- **总线交换 / 限幅 / 看门狗** 放在 `ros2_control` 之后的专用控制线程
- 通过 **插件与工厂** 扩展硬件、后端与策略，业务代码中不写 `if (simulation)`

## 扩展边界（混合方案）

| 边界 | 机制 | 典型用途 |
|------|------|----------|
| 二进制插件 | `pluginlib` | `HardwareBus`、`ISafetyPolicy` |
| 进程内 Provider | `ProviderRegistry` 字符串键工厂 | 导航 / 操作端口、规划器、内置安全接线 |
| 跨进程执行 | ROS 2 Action / Service / Topic | Nav2、轨迹控制器、自定义任务后端 |
| 部署组合 | Launch + YAML | 进程布局、参数、选用哪份 runtime |

**组合根：** `RuntimeBuilder` 读取强类型 `RuntimeConfig`，经 `ProviderRegistry` 解析工厂，组装 `TaskOrchestrator`、端口、规划器与安全门。

```text
runtime.yaml / profile.yaml
        │
        v
  RuntimeBuilder  ──► ProviderRegistry（nav / manip / planner / safety）
        │
        v
  task_orchestrator_node
```

## 非实时 / 实时划分

```text
非实时（ROS 2 / 框架侧）
  - TaskOrchestrator（超时 / 取消）+ ITaskPlanner
  - SafetyGate（admit / monitor；默认拒绝）
  - NavigationPort / ManipulationPort 适配器
  - RobotProfile + RuntimeConfig（强类型；业务代码禁止自行解析 YAML）
  - controller_manager 更新路径

实时侧（ros2_control System 内部）
  - RtControlLoop（配置的 loop_hz）
  - HardwareBus::exchange（pluginlib）
  - 限幅 / 看门狗 / 状态采集 → AtomicStateBuffer
```

ROS 侧 executor 抖动不应直接阻塞电机总线周期：RT 环位于硬件 System 的 write/read 路径下，而非普通节点回调中。

## 数据流

```text
/task/execute（robot_interfaces/action/Task）
      │
      v
TaskOrchestrator（+ SafetyGate + FaultManager）
      │
      ├──► INavigationPort   → Action / Nav2 适配器
      └──► IManipulationPort → Action / FollowJointTrajectory 适配器

/cmd_vel 或轨迹控制器指令
      │
      v
ros2_control controller_manager
      │
      v
RtMobileManipulatorSystem::write()
      │
      v
RealtimeBuffer<CommandSnapshot>
      │
      v
RtControlLoop
      │
      v
HardwareBus::exchange()     # 例如 Mock / Skeleton / ReferenceSim / 外置 EtherCAT 插件
      │
      v
AtomicStateBuffer / StateSnapshot
      │
      v
RtMobileManipulatorSystem::read() → controllers + /joint_states
```

## 配置二分

| 文件 | 应包含 | 禁止包含 |
|------|--------|----------|
| `profile.yaml` | 机器人事实：型号、坐标系、限位、能力 | 插件类名、Action endpoint、Launch 布局 |
| `runtime.yaml` | `hardware.*.plugin`、`providers.*.type` / endpoint、`planner.type`、`safety.policies` | 应属于 Profile 的运动学事实 |

同一 Profile 可通过更换 Runtime 与 URDF `hardware_bus_plugin`，搭配 Mock、进程内仿真或现场总线。

## 硬件抽象

- 接口：`robot_hardware_api::HardwareBus`
- 加载：`pluginlib`（URDF 参数 `hardware_bus_plugin` 和/或 runtime YAML）
- 本仓内置：`MockHardwareBus`、`SkeletonHardwareBus`、`ReferenceSimHardwareBus`
- 任务 / 安全业务逻辑中**不写** `if (simulation)`

EC-Master / IgH **不在本仓实现或 vendoring**。见 [ETHERCAT_INTEGRATION.zh-CN.md](ETHERCAT_INTEGRATION.zh-CN.md)。

## 相关文档

- [接入指南](INTEGRATION_GUIDE.zh-CN.md) — 对接真实后端
- [扩展模板](EXTENSION_TEMPLATES.zh-CN.md) — Skeleton、ReferenceSim、健康与适配器
- [软实时基线](SOFT_RT_BASELINE.zh-CN.md) — 软实时验收
