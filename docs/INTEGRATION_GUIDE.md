# Integration Guide

How to land this framework on a real project: replace Mock backends, connect Nav2 / MoveIt-style trajectories, and plug in an external hardware bus—without changing task business code.

Related: [Architecture](ARCHITECTURE.md) · [Extension templates](EXTENSION_TEMPLATES.md) · [EtherCAT boundary](ETHERCAT_INTEGRATION.md) · [Soft RT baseline](SOFT_RT_BASELINE.md)

中文: [INTEGRATION_GUIDE.zh-CN.md](INTEGRATION_GUIDE.zh-CN.md)

## Extension map

| Boundary | Mechanism | How you extend it |
|----------|-----------|-------------------|
| Hardware | `pluginlib` + `HardwareBus` | Export a plugin; set URDF `hardware_bus_plugin` |
| Nav / manip | Ports + adapters | `providers.*.type` / `endpoint` in `runtime.yaml` |
| Planner | `ITaskPlanner` | `planner.type: simple \| yaml_graph` |
| Safety | `pluginlib` + `ISafetyPolicy` | `safety.policies` class names |
| Deploy | Launch + YAML | Compose processes; no `if (simulation)` |

```text
Application
  │  /task/execute
  ▼
task_orchestrator_node  ← RuntimeBuilder composition root
  │  SafetyGate + FaultManager
  ├──► INavigationPort  → mock / ros_action / nav2
  └──► IManipulationPort → mock / ros_action / follow_joint_trajectory
                │
                ▼
        ros2_control + RtControlLoop
                │
                ▼
        HardwareBus (pluginlib)  ← Mock / Skeleton / external EtherCAT plugin
```

## Configuration

- **Profile** (robot facts): [`robot_bringup/config/profile.yaml`](../src/robot_bringup/config/profile.yaml)
- **Runtime** (plugins & endpoints):
  - [`runtime.yaml`](../src/robot_bringup/config/runtime.yaml) — default Action backends
  - [`runtime_nav2.yaml`](../src/robot_bringup/config/runtime_nav2.yaml)
  - [`runtime_moveit.yaml`](../src/robot_bringup/config/runtime_moveit.yaml)
  - [`runtime_mock.yaml`](../src/robot_bringup/config/runtime_mock.yaml)

Do not put plugin / Action / Launch details in Profile. Do not parse YAML inside business modules—use typed loaders only.

## Application task API

Stable surface:

| Interface | Type |
|-----------|------|
| `/task/execute` | `robot_interfaces/action/Task` |
| `/task/state` | topic / state reporting |
| `/task/set_mode` | mode switching |

```bash
ros2 launch robot_bringup task_stack.launch.py

# Nav2 backend (nav2_msgs required at build time):
ros2 launch robot_bringup task_stack.launch.py runtime_config:=runtime_nav2.yaml

ros2 action send_goal /task/execute robot_interfaces/action/Task \
  "{instruction: 'go to table and pick red cup'}" --feedback
```

In-process Mock (no separate Action servers):

```bash
ros2 launch robot_bringup task_stack_inprocess_mock.launch.py
```

Control stack only:

```bash
ros2 launch robot_bringup control.launch.py \
  hardware_bus_plugin:=robot_hardware_plugins/MockHardwareBus
```

YAML task graph (`planner.type: yaml_graph`, set `tasks_dir`):

```text
instruction: "graph:demo_pick"
```

Example graph: [`config/tasks/demo_pick.yaml`](../src/robot_bringup/config/tasks/demo_pick.yaml)

## Hardware bus plugins

1. Implement `robot_hardware_api::HardwareBus`
2. Export with `PLUGINLIB_EXPORT_CLASS` and register in plugin XML
3. Point URDF (and keep runtime YAML consistent):

```xml
<param name="hardware_bus_plugin">your_pkg/YourHardwareBus</param>
```

References in-repo: `MockHardwareBus`, `SkeletonHardwareBus`, `ReferenceSimHardwareBus`.

Do **not** copy EC-Master / IgH into this repository — see [EtherCAT integration](ETHERCAT_INTEGRATION.md).

**RT rules for `exchange()`:** no heap allocation, no logging, no ROS API, no blocking I/O.

## Navigation / manipulation / planner

### Planner

| `planner.type` | Behavior |
|----------------|----------|
| `simple` | Keyword `SimpleTaskPlanner` |
| `yaml_graph` | Load `graph:<name>.yaml` from `tasks_dir` |

### Navigation

| `providers.navigation.type` | Behavior |
|-----------------------------|----------|
| `mock` | In-process success |
| `ros_action` | Framework Action (default mock server) |
| `nav2` | `Nav2NavigationPort` → `nav2_msgs/NavigateToPose` (needs `nav2_msgs`) |

Recommended Nav2 wiring:

```yaml
providers:
  navigation:
    type: nav2
    endpoint: navigate_to_pose
```

### Manipulation

| `providers.manipulation.type` | Behavior |
|-------------------------------|----------|
| `mock` / `ros_action` | Mock or framework Action |
| `follow_joint_trajectory` / `moveit` | Send **explicit joint targets** to `FollowJointTrajectory` |

`FollowJointTrajectoryManipulationPort` does **not** run IK. `ManipulationGoal` must include `joint_positions` (and matching `joint_names`). Cartesian planning stays outside the framework (e.g. MoveIt), then fill joint targets.

## Health and RT stats

| Topic | Source |
|-------|--------|
| `/robot/rt_loop_stats` | Control stack (non-RT publish path) |
| `/robot/health` | `HealthMonitor` (+ FaultManager) |

RT-side faults can drive health to FAULT and set `allows_motion=false`.

## SafetyGate

Every orchestration step goes through `admit`; execution may call `monitor`. Missing policies or unknown state → **deny**.

Built-in: `robot_safety/VelocityLimitPolicy`. Software gating is **not** a hardware e-stop.

## Acceptance checklist

- [ ] `colcon test --packages-select robot_testkit` passes
- [ ] `task_stack.launch.py` completes nav + manip feedback
- [ ] `runtime_config:=runtime_nav2.yaml` selects `Nav2NavigationPort` when Nav2 is installed
- [ ] Changing `hardware_bus_plugin` needs no business-code edits
- [ ] Illegal Profile fields fail at startup
- [ ] Safety with no policies refuses execution
- [ ] `/robot/health` is echoable; RT stats visible when the control stack runs
