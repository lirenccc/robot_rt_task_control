# Architecture

This document describes the extension model, real-time split, data paths, and configuration contracts of the ROS 2 RT Task-Control Framework.

中文: [ARCHITECTURE.zh-CN.md](ARCHITECTURE.zh-CN.md)

## Goals

- Keep **tasking and safety** on the non-RT ROS 2 side
- Keep **exchange / limiting / watchdog** on a dedicated control thread behind `ros2_control`
- Extend hardware, backends, and policies by **plugins and factories**, not by branching on “sim vs real” in business code

## Extension boundaries (hybrid)

| Boundary | Mechanism | Typical use |
|----------|-----------|-------------|
| Binary plugins | `pluginlib` | `HardwareBus`, `ISafetyPolicy` |
| In-process providers | `ProviderRegistry` factories (string-keyed) | Navigation / manipulation ports, planners, built-in safety wiring |
| Cross-process | ROS 2 Action / Service / Topic | Nav2, trajectory controllers, custom task backends |
| Deployment | Launch + YAML | Process layout, parameters, which runtime file to load |

**Composition root:** `RuntimeBuilder` reads typed `RuntimeConfig`, resolves factories through `ProviderRegistry`, and wires `TaskOrchestrator`, ports, planner, and safety.

```text
runtime.yaml / profile.yaml
        │
        v
  RuntimeBuilder  ──► ProviderRegistry (nav / manip / planner / safety)
        │
        v
  task_orchestrator_node
```

## Non-RT vs RT split

```text
Non-RT (ROS 2 / framework)
  - TaskOrchestrator (timeout / cancel) + ITaskPlanner
  - SafetyGate (admit / monitor; default deny)
  - NavigationPort / ManipulationPort adapters
  - RobotProfile + RuntimeConfig (typed; business code must not parse YAML)
  - controller_manager update path

RT (inside ros2_control System)
  - RtControlLoop @ configured loop_hz
  - HardwareBus::exchange (pluginlib)
  - Limiting / watchdog / state capture into AtomicStateBuffer
```

Executor jitter on the ROS side must not block the bus period: the RT loop runs under the hardware System’s write/read path, not inside ordinary node callbacks.

## Data flow

```text
/task/execute (robot_interfaces/action/Task)
      │
      v
TaskOrchestrator (+ SafetyGate + FaultManager)
      │
      ├──► INavigationPort   → Action / Nav2 adapter
      └──► IManipulationPort → Action / FollowJointTrajectory adapter

/cmd_vel or trajectory controller commands
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
HardwareBus::exchange()     # e.g. Mock / Skeleton / ReferenceSim / external EtherCAT plugin
      │
      v
AtomicStateBuffer / StateSnapshot
      │
      v
RtMobileManipulatorSystem::read() → controllers + /joint_states
```

## Configuration split

| File | Contents | Must not contain |
|------|----------|------------------|
| `profile.yaml` | Robot facts: model, frames, limits, capabilities | Plugin class names, Action endpoints, Launch layout |
| `runtime.yaml` | `hardware.*.plugin`, `providers.*.type` / endpoint, `planner.type`, `safety.policies` | Robot kinematics facts that belong in profile |

The same profile can pair with Mock, in-process sim, or fieldbus by swapping runtime config and URDF `hardware_bus_plugin`.

## Hardware abstraction

- Interface: `robot_hardware_api::HardwareBus`
- Loading: `pluginlib` (URDF param `hardware_bus_plugin` and/or runtime YAML)
- Built-ins in this repo: `MockHardwareBus`, `SkeletonHardwareBus`, `ReferenceSimHardwareBus`
- **No** `if (simulation)` in task or safety business logic

EC-Master / IgH are **not** implemented or vendored here. See [ETHERCAT_INTEGRATION.md](ETHERCAT_INTEGRATION.md).

## Related documents

- [Integration guide](INTEGRATION_GUIDE.md) — how to wire real backends
- [Extension templates](EXTENSION_TEMPLATES.md) — Skeleton, ReferenceSim, health, adapters
- [Soft RT baseline](SOFT_RT_BASELINE.md) — soft real-time acceptance
