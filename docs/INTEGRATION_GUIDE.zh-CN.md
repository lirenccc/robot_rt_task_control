# 接入指南

如何在真实项目中落地本框架：替换 Mock 后端、对接 Nav2 / MoveIt 风格轨迹、接入外置硬件总线——且不修改任务业务代码。

相关文档：[架构](ARCHITECTURE.zh-CN.md) · [扩展模板](EXTENSION_TEMPLATES.zh-CN.md) · [EtherCAT 边界](ETHERCAT_INTEGRATION.zh-CN.md) · [软实时基线](SOFT_RT_BASELINE.zh-CN.md)

English: [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md)

## 扩展对照表

| 边界 | 机制 | 你怎么扩展 |
|------|------|------------|
| 硬件 | `pluginlib` + `HardwareBus` | 新包导出插件；设置 URDF `hardware_bus_plugin` |
| 导航 / 操作 | 端口 + 适配器 | 改 `runtime.yaml` 的 `providers.*.type` / `endpoint` |
| 规划器 | `ITaskPlanner` | `planner.type: simple \| yaml_graph` |
| 安全 | `pluginlib` + `ISafetyPolicy` | `safety.policies` 中填写类名 |
| 部署 | Launch + YAML | 组合进程与配置；不写 `if (simulation)` |

```text
应用
  │  /task/execute
  ▼
task_orchestrator_node  ← RuntimeBuilder 组合根
  │  SafetyGate + FaultManager
  ├─► INavigationPort  → mock / ros_action / nav2
  └─► IManipulationPort → mock / ros_action / follow_joint_trajectory
                │
                ▼
        ros2_control + RtControlLoop
                │
                ▼
        HardwareBus (pluginlib)  ← Mock / Skeleton / 外置 EtherCAT 插件
```

## 配置

- **Profile**（机器人事实）：[`robot_bringup/config/profile.yaml`](../src/robot_bringup/config/profile.yaml)
- **Runtime**（插件与 endpoint）：
  - [`runtime.yaml`](../src/robot_bringup/config/runtime.yaml) — 默认 Action 后端
  - [`runtime_nav2.yaml`](../src/robot_bringup/config/runtime_nav2.yaml)
  - [`runtime_moveit.yaml`](../src/robot_bringup/config/runtime_moveit.yaml)
  - [`runtime_mock.yaml`](../src/robot_bringup/config/runtime_mock.yaml)

禁止在 Profile 中写插件 / Action / Launch。禁止业务模块自行解析 YAML，只使用强类型加载器。

## 应用侧任务接口

稳定对外接口：

| 接口 | 类型 |
|------|------|
| `/task/execute` | `robot_interfaces/action/Task` |
| `/task/state` | 状态上报 |
| `/task/set_mode` | 模式切换 |

```bash
ros2 launch robot_bringup task_stack.launch.py

# Nav2 后端（需编译期已安装 nav2_msgs）:
ros2 launch robot_bringup task_stack.launch.py runtime_config:=runtime_nav2.yaml

ros2 action send_goal /task/execute robot_interfaces/action/Task \
  "{instruction: 'go to table and pick red cup'}" --feedback
```

进程内 Mock（不启独立 Action Server）：

```bash
ros2 launch robot_bringup task_stack_inprocess_mock.launch.py
```

仅控制栈：

```bash
ros2 launch robot_bringup control.launch.py \
  hardware_bus_plugin:=robot_hardware_plugins/MockHardwareBus
```

YAML 任务图（`planner.type: yaml_graph`，并设置 `tasks_dir`）：

```text
instruction: "graph:demo_pick"
```

示例图：[`config/tasks/demo_pick.yaml`](../src/robot_bringup/config/tasks/demo_pick.yaml)

## 硬件总线插件

1. 实现 `robot_hardware_api::HardwareBus`
2. 使用 `PLUGINLIB_EXPORT_CLASS` 导出，并在 plugin XML 中注册
3. 在 URDF 中指定（并与 runtime YAML 保持一致）：

```xml
<param name="hardware_bus_plugin">your_pkg/YourHardwareBus</param>
```

本仓参考：`MockHardwareBus`、`SkeletonHardwareBus`、`ReferenceSimHardwareBus`。

**不要**把 EC-Master / IgH 拷进本仓 — 见 [EtherCAT 对接](ETHERCAT_INTEGRATION.zh-CN.md)。

**`exchange()` 的 RT 约束：** 禁止堆分配、日志、ROS API、阻塞 IO。

## 导航 / 操作 / 规划器

### 规划器

| `planner.type` | 行为 |
|----------------|------|
| `simple` | 关键词 `SimpleTaskPlanner` |
| `yaml_graph` | 从 `tasks_dir` 加载 `graph:<name>.yaml` |

### 导航

| `providers.navigation.type` | 行为 |
|-----------------------------|------|
| `mock` | 进程内成功 |
| `ros_action` | 框架 Action（默认 mock server） |
| `nav2` | `Nav2NavigationPort` → `nav2_msgs/NavigateToPose`（需 `nav2_msgs`） |

推荐 Nav2 配置：

```yaml
providers:
  navigation:
    type: nav2
    endpoint: navigate_to_pose
```

### 操作

| `providers.manipulation.type` | 行为 |
|-------------------------------|------|
| `mock` / `ros_action` | Mock 或框架 Action |
| `follow_joint_trajectory` / `moveit` | 将**显式关节目标**下发到 `FollowJointTrajectory` |

`FollowJointTrajectoryManipulationPort` **不做 IK**。`ManipulationGoal` 必须带 `joint_positions`（及匹配的 `joint_names`）。笛卡尔规划在框架外（如 MoveIt）完成后再填入关节目标。

## 健康与 RT 统计

| Topic | 来源 |
|-------|------|
| `/robot/rt_loop_stats` | 控制栈（非 RT 发布路径） |
| `/robot/health` | `HealthMonitor`（+ FaultManager） |

RT 侧故障可将健康状态置为 FAULT，并设置 `allows_motion=false`。

## SafetyGate

所有编排步骤经 `admit`；执行中可 `monitor`。无策略或状态未知时**默认拒绝**。

内置：`robot_safety/VelocityLimitPolicy`。软件门控 **不等于** 硬件急停。

## 验收清单

- [ ] `colcon test --packages-select robot_testkit` 通过
- [ ] `task_stack.launch.py` 可完成导航 + 操作反馈
- [ ] 已装 Nav2 时，`runtime_config:=runtime_nav2.yaml` 可选中 `Nav2NavigationPort`
- [ ] 更换 `hardware_bus_plugin` 无需改业务代码
- [ ] Profile 非法字段启动失败
- [ ] Safety 无策略时拒绝执行
- [ ] `/robot/health` 可 echo；控制栈运行时可见 RT 统计
