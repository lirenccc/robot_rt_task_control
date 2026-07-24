# 软实时验收基线

本基线验证 **用户态软实时**（尽力调度），**不是** PREEMPT_RT 认证，也不是现场总线硬实时证明。

English: [SOFT_RT_BASELINE.md](SOFT_RT_BASELINE.md)

## 方法

| 项目 | 取值 |
|------|------|
| 植物 | 进程内 `MockHardwareBus` + `RtControlLoop` |
| 目标频率 | `loop_hz = 1000` |
| 运行时长 | ≥ 2 s |
| 测试源码 | [`robot_testkit/test/test_soft_rt_baseline.cpp`](../src/robot_testkit/test/test_soft_rt_baseline.cpp) |

## 通过准则

| 指标 | 本机期望 | CI 宽松 |
|------|----------|---------|
| `missed_deadlines` | 0 | ≤ 2（约 &lt; 0.1% @ 2 s） |
| `measured_frequency_hz` | ∈ [900, 1100] | 同左 |
| `loop_count` | &gt; 1500 | 同左 |

```bash
colcon test --packages-select robot_testkit --ctest-args -R soft_rt --event-handlers console_direct+
```

## 说明

- 未设置 FIFO / `rt_priority` 时，结果取决于主机负载与后台进程
- 硬实时应在**外置**总线仓 + RT 内核上另行验收
- 运行时还可观察 `/robot/rt_loop_stats` 的抖动与丢截止

## 相关文档

- [架构说明](ARCHITECTURE.zh-CN.md) — RT 环为何放在 `ros2_control` 之下
- [EtherCAT 对接](ETHERCAT_INTEGRATION.zh-CN.md) — 硬实时主站所在位置
