# EtherCAT Master Integration Contract (External)

This repository does **not** implement or vendor EC-Master, IgH, OEM ENI files, or motor profiles. Host masters in a separate GitHub module; consume them through a thin `HardwareBus` plugin.

| Artifact | Location |
|----------|----------|
| Interface | `robot_hardware_api::HardwareBus` |
| Fill-in template | `SkeletonHardwareBus` |
| Reference plant (no fieldbus) | `ReferenceSimHardwareBus` |

中文: [ETHERCAT_INTEGRATION.zh-CN.md](ETHERCAT_INTEGRATION.zh-CN.md)

## Repository boundaries

| Repo | Role |
|------|------|
| This framework | Tasking, safety, ports, `HardwareBus` API, Mock/Skeleton/ReferenceSim, `ros2_control` RT loop |
| External master repos (sibling workspaces) | EC-Master / IgH init, cycle, PDO/ENI, CiA402 |
| Thin adapter (optional package) | Implements `HardwareBus`, calls master API inside `exchange()` |

Extracted external master workspaces (do **not** re-vendor master implementations in the product overlay). Layout under `/home/mscape/workspaces/robot_rt_task_control_ws/`:
Product overlay (sibling): `robot_arm_control` (7DoF bringup / `Arm*HardwareBus` / Web).


| Workspace (sibling) | Package | Stable API |
|-----------|---------|------------|
| `../robot_ethercat_master_ecmaster` | `ethercat_master_ecmaster` | `ethercat_master_ecmaster::Master` (`init` / `map_joints` / `start` / `cycle` / `shutdown`) |
| `../robot_ethercat_master_igh` | `ethercat_master_igh` | `ethercat_master_igh::Master` (same surface) |

Vendor SDKs stay on the machine (`ECMASTER_ROOT`, `/opt/etherlab`); they are not vendored into those repos or this framework.

Thin adapters in this workspace (`robot_ethercat_adapters`, optional QUIET build):

| Plugin class | Library | Requires underlay |
|--------------|---------|-------------------|
| `robot_ethercat_adapters/EcMasterHardwareBus` | `robot_ethercat_adapters_ecmaster` | `ethercat_master_ecmaster` |
| `robot_ethercat_adapters/IghHardwareBus` | `robot_ethercat_adapters_igh` | `ethercat_master_igh` |

Joint → slave map uses `ETHERCAT_MOTOR_MODEL` (default `NH17-100-BT-48E`) and `MotorProfileRegistry`; axes are ordered as URDF `joint_names`.

```text
RtControlLoop @ loop_hz
      │
      v
HardwareBus::exchange()          ← pluginlib class name is swappable
      │
      ├── Mock / Skeleton / ReferenceSim     (robot_hardware_plugins)
      ├── EcMasterHardwareBus               (robot_ethercat_adapters, optional)
      └── IghHardwareBus                    (robot_ethercat_adapters, optional)
                │
                ├── ethercat_master_ecmaster::Master::cycle()
                └── ethercat_master_igh::Master::cycle()
```

Switch without touching business code:

- URDF: `hardware_bus_plugin:=robot_ethercat_adapters/EcMasterHardwareBus` (or `.../IghHardwareBus`)
- Keep the same plugin name recorded in [`runtime.yaml`](../src/robot_bringup/config/runtime.yaml)

Build with masters on `CMAKE_PREFIX_PATH` (source their `install/setup.bash` first). If a master package is missing, that adapter is skipped and the rest of this workspace still builds.

## Suggested external master API

Stable, ROS-decoupled C++ surface (sketch):

1. **init** — open NIC / load ENI or IgH domain; bring up slaves
2. **map_joints** — `joint_names[]` → PDO / axis index (from config, not hard-coded models)
3. **start** — reach OP / start Timing+Job (cycle ownership lives in the master)
4. **cycle** — one period: push setpoints + read cached state (**no** PDO exchange inside `exchange`)
5. **request_safety_reset** — clear comm latch and arm healthy dwell (**no** auto re-enable / **no** implicit `0x0080`)
6. **motion_reenable_allowed** — gate `setEnable(true)` after dwell
7. **shutdown** — disable drives and close transport

Do not expose raw CAN/EtherCAT frames upward. The adapter only fills `CommandSnapshot` / `StateSnapshot`.

EC-Master and IgH expose `apply_command_contention_fallback()` for adapter `try_to_lock` misses (CST/CSV safe degrade for that beat).

## Reliability capabilities

Details and env vars live in each master repo; this table only maps framework / adapter duties.

| Capability | EC-Master | IgH | Framework / adapter |
|------------|-----------|-----|---------------------|
| In-Job safe-output | ✅ [`docs/INTEGRATION.md`](../../robot_ethercat_master_ecmaster/docs/INTEGRATION.md) | ✅ same semantics | `cycle()` fault → `exchange` returns false |
| Skip-slot + deadline metrics | ✅ | ✅ | File trajectories advance on **executed** beats |
| `mlockall` + RT fail-closed | ✅ `ECMASTER_*` | ✅ `IGH_*` | Deploy checks in master scripts / systemd examples |
| PDO / `0x60C2` evidence gate | ✅ | ✅ | Surface master error string on enable refusal |
| Anomaly + healthy dwell | ✅ | ✅ | Product `/request_safety_reset` → `Master::request_safety_reset` |
| CST command freshness | ✅ | ✅ | Thin adapter injects `effort`; contention fallback |
| DC out-of-sync monitor | ✅ | ✅ | Diagnostics only; stop is in-Job |
| IgH hard-RT Job ownership | — | ✅ | `IghHardwareBus` / `ArmIghHardwareBus` |

**HIL**: capabilities above still pending pull-cable / overrun / reset-dwell / CST-timeout bench tests.

## Observation and recovery (product)

| Step | Action |
|------|--------|
| 1 | Comm / anomaly latch → Job safe-output; `Master::cycle` stops injecting |
| 2 | Call `/request_safety_reset` (or C++ API) to clear latch and start healthy dwell |
| 3 | Wait for `motion_reenable_allowed`, then `/set_enable` |
| 4 | Do **not** expect automatic re-enable after link loss |

See product `robot_arm_control/docs/troubleshooting.md` and each master’s `config/env.example`.

## Mapping from Skeleton TODOs

| Skeleton `TODO(vendor)` | External module responsibility |
|-------------------------|--------------------------------|
| open transport | `init` + NIC / master instance |
| joint_name → device id | `map_joints` / config table |
| validate PDO layout | config check (dims = `joint_names.size()`) |
| enable drives / watchdog | start path |
| pack / cycle / unpack | `exchange` → `cycle` (no heap allocation) |
| disable / close | stop → `shutdown` |

## Adapter package conventions

1. `find_package(<external_master> QUIET)`; if missing, **do not build** that plugin (same optional pattern as Nav2)
2. `PLUGINLIB_EXPORT_CLASS(..., robot_hardware_api::HardwareBus)` + plugin XML
3. `package.xml`: `exec_depend` / optional on the master package; core packages here must **not** hard-depend on it
4. RT: inside `exchange()` — no logs, no ROS API, no blocking locks, no dynamic allocation

## Explicit non-goals

- Do not copy EC-Master SDK, IgH sources, vendor ENI, or motor profiles into this repository
- Do not bind this repo to a specific joint module SKU; model differences stay in external config / separate repos
