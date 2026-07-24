# Soft Real-Time Acceptance Baseline

This baseline checks **best-effort user-space soft RT**. It is **not** PREEMPT_RT certification or hard real-time proof for a fieldbus stack.

中文: [SOFT_RT_BASELINE.zh-CN.md](SOFT_RT_BASELINE.zh-CN.md)

## Method

| Item | Value |
|------|--------|
| Plant | In-process `MockHardwareBus` + `RtControlLoop` |
| Target rate | `loop_hz = 1000` |
| Duration | ≥ 2 s |
| Test source | [`robot_testkit/test/test_soft_rt_baseline.cpp`](../src/robot_testkit/test/test_soft_rt_baseline.cpp) |

## Pass criteria

| Metric | Local expectation | CI (relaxed) |
|--------|-------------------|--------------|
| `missed_deadlines` | 0 | ≤ 2 (~&lt; 0.1% over 2 s) |
| `measured_frequency_hz` | ∈ [900, 1100] | same |
| `loop_count` | &gt; 1500 | same |

```bash
colcon test --packages-select robot_testkit --ctest-args -R soft_rt --event-handlers console_direct+
```

## Notes

- Without FIFO scheduling / `rt_priority`, results depend on host load and background processes
- Hard RT belongs with an **external** fieldbus stack plus an RT kernel; validate there separately
- At runtime, also watch `/robot/rt_loop_stats` for jitter and deadline misses

## Related documents

- [Architecture](ARCHITECTURE.md) — why the RT loop sits under `ros2_control`
- [EtherCAT integration](ETHERCAT_INTEGRATION.md) — where hard RT masters live
