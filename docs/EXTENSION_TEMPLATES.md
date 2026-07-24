# Extension Templates

Vendor-agnostic building blocks shipped with this repository: hardware bus stubs, a production-shaped reference plant, runtime lifecycle, health/fault surfaces, and navigation/manipulation adapters.

These are **templates and reference implementations**, not bindings to a specific OEM or robot model.

中文: [EXTENSION_TEMPLATES.zh-CN.md](EXTENSION_TEMPLATES.zh-CN.md)

## HardwareBus templates

| Plugin | Purpose |
|--------|---------|
| `SkeletonHardwareBus` | Fill `TODO(vendor)` for real bus I/O |
| `ReferenceSimHardwareBus` | Sequence / watchdog / fault inject without a fieldbus |
| `MockHardwareBus` | Lightweight success path for CI and demos |

Switch via URDF `hardware_bus_plugin` (and keep `runtime.yaml` consistent). For EtherCAT masters, implement a thin adapter outside this repo — see [ETHERCAT_INTEGRATION.md](ETHERCAT_INTEGRATION.md).

## Runtime lifecycle

`RobotRuntime::{configure, activate, deactivate, cleanup}` is driven in order by `RuntimeLifecycle` for registered participants (currently including `HealthMonitor`).

`task_orchestrator_node` calls `configure` + `activate` on startup.

## Health / FAULT

| Topic | Message | Notes |
|-------|---------|-------|
| `/robot/health` | `robot_interfaces/msg/HardwareHealth` | `mode` ∈ OK / DEGRADED / FAULT / ESTOP; `fault_code`; `allows_motion` |
| `/robot/rt_loop_stats` | loop stats from control stack | Soft-RT observability |

`FaultManager` provides **software** gating: orchestration failures may `raise(Fault)`. Hardware e-stop remains below the framework.

## Nav2 adapter

- Config: `providers.navigation.type: nav2`
- Implementation: `Nav2NavigationPort` → `nav2_msgs/NavigateToPose`
- Build: enabled when `nav2_msgs` is found; otherwise CMake skips the source and you can still use `ros_action`

```bash
ros2 launch robot_bringup task_stack.launch.py runtime_config:=runtime_nav2.yaml
```

Example: [`runtime_nav2.yaml`](../src/robot_bringup/config/runtime_nav2.yaml)

## FollowJointTrajectory / MoveIt-style adapter

- Config: `providers.manipulation.type: follow_joint_trajectory` or `moveit`
- Implementation: `FollowJointTrajectoryManipulationPort` → `control_msgs/FollowJointTrajectory`
- **No IK in-framework:** `ManipulationGoal` must carry `joint_positions` (+ `joint_names`). Run Cartesian planning externally, then fill joint targets.

Example: [`runtime_moveit.yaml`](../src/robot_bringup/config/runtime_moveit.yaml)

## Related documents

- [Integration guide](INTEGRATION_GUIDE.md) — end-to-end wiring and checklist
- [Architecture](ARCHITECTURE.md) — where templates sit in the stack
