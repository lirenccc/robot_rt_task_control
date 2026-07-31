# EtherCAT 主站对接契约（外置）

本仓库**不实现、不 vendoring** EC-Master、IgH、厂商 ENI 或电机 profile。真实主站放在独立 GitHub 模块，本框架只通过薄 `HardwareBus` 插件消费。

| 产物 | 位置 |
|------|------|
| 接口 | `robot_hardware_api::HardwareBus` |
| 填空模板 | `SkeletonHardwareBus` |
| 参考植物（无现场总线） | `ReferenceSimHardwareBus` |

English: [ETHERCAT_INTEGRATION.md](ETHERCAT_INTEGRATION.md)

## 仓库边界

| 仓库 | 职责 |
|------|------|
| 本仓 | 任务编排、Safety、Ports、`HardwareBus` 接口、Mock/Skeleton/ReferenceSim、`ros2_control` RT 环 |
| 外部主站仓（同级工作区） | EC-Master / IgH 初始化、周期、PDO/ENI、CiA402 |
| 薄适配包（可选，可另仓） | 实现 `HardwareBus`，在 `exchange()` 中调用外部主站 API |

已抽出的外置主站工作区（消费时**不要**在产品仓再维护主站实现）。目录布局在 `/home/mscape/workspaces/robot_rt_task_control_ws/`：
产品 overlay（同级）：`robot_arm_control`（7DoF bringup / `Arm*HardwareBus` / Web）。


| 工作区（同级） | 包名 | 稳定 API |
|--------|------|----------|
| `../robot_ethercat_master_ecmaster` | `ethercat_master_ecmaster` | `ethercat_master_ecmaster::Master`（`init` / `map_joints` / `start` / `cycle` / `shutdown`） |
| `../robot_ethercat_master_igh` | `ethercat_master_igh` | `ethercat_master_igh::Master`（同一面） |

厂商 SDK 仍安装在本机（`ECMASTER_ROOT`、`/opt/etherlab`），不进入上述外置仓或本框架仓。

本仓薄适配（`robot_ethercat_adapters`，QUIET 可选编译）：

| 插件类名 | 库 | 需要 underlay |
|----------|-----|---------------|
| `robot_ethercat_adapters/EcMasterHardwareBus` | `robot_ethercat_adapters_ecmaster` | `ethercat_master_ecmaster` |
| `robot_ethercat_adapters/IghHardwareBus` | `robot_ethercat_adapters_igh` | `ethercat_master_igh` |

关节映射：`ETHERCAT_MOTOR_MODEL`（默认 `NH17-100-BT-48E`）+ `MotorProfileRegistry`；轴顺序与 URDF `joint_names` 一致。

```text
RtControlLoop @ loop_hz
      │
      v
HardwareBus::exchange()          ← pluginlib 类名可切换
      │
      ├── Mock / Skeleton / ReferenceSim     （robot_hardware_plugins）
      ├── EcMasterHardwareBus               （robot_ethercat_adapters，可选）
      └── IghHardwareBus                    （robot_ethercat_adapters，可选）
                │
                ├── ethercat_master_ecmaster::Master::cycle()
                └── ethercat_master_igh::Master::cycle()
```

切换方式（无需改业务代码）：

- URDF：`hardware_bus_plugin:=robot_ethercat_adapters/EcMasterHardwareBus`（或 `.../IghHardwareBus`）
- 在 [`runtime.yaml`](../src/robot_bringup/config/runtime.yaml) 中记录同名插件（与 URDF 保持一致）

编译前先 `source` 外置主站仓的 `install/setup.bash`。缺主站包时跳过对应适配器，本仓其余包仍可编译。

## 外部主站仓建议导出的 API

稳定、与 ROS 解耦的 C++ 面（示意）：

1. **init** — 打开网卡 / 加载 ENI 或 IgH 域，建立从站
2. **map_joints** — 按 `AxisConfig` 映射 PDO / 轴索引（由配置注入，不写死机型）。
   双主站公共字段为扁平身份：`joint_name`、`alias`、`position`、`vendor_id`、
   `product_code`、`model_id`、`pdo_layout`（**无**嵌套 `MotorConfig`；运动学由主站
   profile/SDO 与产品 YAML overlay 负责）
3. **start** — 进 OP / 启动 Timing+Job（周期所有权在主站）；成功后默认保持 safe-output，
   直至显式 `release_safe_output`
4. **cycle** — 单周期：只灌 setpoint + 读缓存状态（**不**在 `exchange` 里重做 PDO）
5. **request_safety_reset** — 清通信闩锁并 arm healthy dwell（**不**自动再使能 / **不**隐式 `0x0080`）；
   dwell 期间仍保持 safe-output
6. **motion_reenable_allowed** / **health** — dwell 与策略授权后才允许再使能；
   监督面用 `Master::health()`（`communication_fault`、`safe_output_active`、
   `motion_reenable_allowed` 等），勿仅凭 `cycle()` 成功判定可动
7. **release_safe_output** — 在 `SupervisedMotion` + 证据门 + healthy dwell 满足后释放灌入
8. **request_fault_reset** — 显式 CiA402 Fault Reset（`0x0080`）；`0xFF` = 故障轴全体。
   EC-Master 在 safe-output / 条件不满足时拒绝；IgH 当前恒返回 `false`
9. **cycle_raw** — **仅 EC-Master**：宿主已完成单位换算的 raw PDO 周期（IgH 未移植）
10. **shutdown** — 下使能并关闭传输

构造参数 **`MotionPolicy`**（默认 `ObservationOnly`）：

| Policy | 驱动配置（SDO） | 周期命令灌入 |
|--------|-----------------|--------------|
| `ObservationOnly` | 否 | 否 |
| `Commissioning` | 是（主站支持时） | 否 |
| `SupervisedMotion` | 是（主站支持时） | 证据门通过后可（仍需 `release_safe_output`） |

产品 / 薄适配实机路径使用 `SupervisedMotion`。

不要向上暴露原始 CAN/EtherCAT 帧；适配层只填充 `CommandSnapshot` / `StateSnapshot`。

EC-Master 与 IgH 均提供 `apply_command_contention_fallback()`：适配层 `try_to_lock` 失败时对已武装 CST/CSV 做本拍安全退化。

## 可靠性能力对照

实现细节与环境变量以各主站仓文档为准；本表只标框架/适配层职责。

| 能力 | EC-Master | IgH | 框架 / 适配层 |
|------|-----------|-----|----------------|
| Job 内 safe-output | ✅ | ✅ 同语义 | `cycle()` fault → `exchange` 返回 false；产品急停另钉位 |
| `MotionPolicy` + `health()` | ✅ | ✅ | 适配/产品 Bus：`SupervisedMotion`；`FeatureState` 读 health |
| `release_safe_output` | ✅ | ✅ | `exchange` 在 `motion_reenable_allowed` 后调用 |
| `request_fault_reset` | ✅ | API 存在，恒 `false` | 产品 `/request_fault_reset` → FeatureBridge → Bus |
| `cycle_raw` | ✅ | 未移植 | 仅宿主 raw 路径；SI 薄适配用 `cycle` |
| 跳拍 + deadline 度量 | ✅ | ✅ | 文件轨迹按**实际执行拍**推进（墙钟可能压缩） |
| `mlockall` + RT fail-closed | ✅ `ECMASTER_*` | ✅ `IGH_*` | 部署检查在主站脚本 / systemd example |
| PDO / `0x60C2` 证据门 | ✅ | ✅ | 使能失败时读主站错误串 |
| Anomaly + healthy dwell | ✅ | ✅ | 产品 `/request_safety_reset` → `Master::request_safety_reset` |
| CST 命令新鲜度 | ✅ | ✅ | 薄适配灌 `effort`；contention fallback |
| DC 失步监控 | ✅ | ✅ | 仅诊断；停机在主站 Job |
| IgH 硬实时 Job 所有权 | — | ✅ | `IghHardwareBus` / `ArmIghHardwareBus` |

**HIL**：上表能力均待台架拔线 / 超期 / 复位 dwell / Fault Reset / CST 超时验收；未测项不得对外宣称生产合格。

## 观测与故障恢复（产品侧）

| 步骤 | 说明 |
|------|------|
| 1 | 通信 / 异常闩锁 → Job safe-output；`Master::cycle` 停灌入 |
| 2 | 调 `/request_safety_reset`（或等价 C++ API）清闩锁并开始 healthy dwell |
| 3 | dwell 满足后 Bus 调用 `release_safe_output`；待 `motion_reenable_allowed` 再 `/set_enable` |
| 4 | 驱动 Fault 态需显式 `/request_fault_reset`（`axis_id`，`255`=全体）；**不**与 safety reset 混用 |
| 5 | **不**期望断线后全自动重使能 |

排障：产品仓 `robot_arm_control/docs/troubleshooting.md`；主站 env 见各仓 `config/env.example`。
EC-Master 部署须提供 `ECMASTER_ENI_FILE`，并以批准 PCI BDF 解析 `ECMASTER_INTELGBE_INSTANCE`（勿猜死网卡序号）。

## 与 Skeleton TODO 的对应

| Skeleton `TODO(vendor)` | 外部模块职责 |
|-------------------------|--------------|
| open transport | `init` + 网卡 / 主站实例 |
| joint_name → device id | `map_joints` / 配置表 |
| validate PDO layout | 配置校验（维度 = `joint_names.size()`） |
| enable drives / watchdog | start 路径 |
| pack / cycle / unpack | `exchange` → `cycle`（禁止堆分配） |
| disable / close | stop → `shutdown` |

## 本仓适配包约定（实现时）

1. `find_package(<外部主站包> QUIET)`；未安装则**不编**该插件（与 Nav2 可选编译同策略）
2. `PLUGINLIB_EXPORT_CLASS(..., robot_hardware_api::HardwareBus)` + plugin XML
3. `package.xml` 对主站仓为 `exec_depend` / optional；本仓核心包**不**硬依赖
4. RT：`exchange()` 内禁止日志、ROS API、阻塞锁、动态分配

## 明确不做

- 不把 EC-Master SDK、IgH 源码、厂商 ENI、电机 profile 拷入本仓
- 本仓不绑定具体关节模组型号；机型差异留在外部配置 / 独立仓
