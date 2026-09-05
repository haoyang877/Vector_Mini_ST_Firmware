# 20 kHz 快环计算时间优化与实机验证

## 目标与测试条件

- MCU：STM32G431，主频 170 MHz；
- 快环频率：20 kHz，单周期硬截止时间 50 us，即 8500 cycles；
- 供电：母线约 28 V；
- 机械负载：约 1.5 Nm 阻尼器，使用 `MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM`；
- 当前镜像仍定义 `HARDWARE_VALIDATION_SKIP_TEMPERATURE_PROTECTION`，温度保护项目按本次台架约定跳过，不能作为量产镜像；
- 最坏时间由快环入口/出口的 DWT CYCCNT 统计，`deadline_overrun_count` 统计超过 8500 cycles 的次数。

## 优化结果

| 场景 | 指标 | 优化前 | 优化后 | 变化 |
| --- | ---: | ---: | ---: | ---: |
| Standby | 滤波周期 | 4682 | 4200 | -482（-10.3%） |
| Standby | 最大周期 | 4857 | 4368 | -489（-10.1%） |
| Mode 13 活跃路径 | 滤波周期 | 8941 | 7848 | -1093（-12.2%） |
| Mode 13 活跃路径 | 最大周期 | 9861 | 8063 | -1798（-18.2%） |
| Mode 13 活跃路径 | 截止期超时 | 持续发生 | 0 | 消除 |

优化后 Mode 13 最坏值距离截止期还有 437 cycles，按 170 MHz 折算约 2.57 us，
占 50 us 周期的 5.1%。测试期间另一次带调试停核的运行出现过 1 次 9153-cycle
异常值；由于停核同时造成 USB 采样超时，该数据不作为无扰动验收结果。最终验收采用
运行态 SWD 内存读取，不停止内核。

停核读数后继续进行高频 USB 浮点遥测时还曾出现一次 HardFault，故障点位于 C 库
`_printf_fp_dec_real`。复位后使用不停核读数重新执行 Mode 13、Mode 15 和速度闭环，
未再复现。`tools/read_fast_loop_metrics.ps1` 因此只做运行态内存读取，不发送 halt；
后续若需要复现该异常，应作为“调试器停核与 USB 格式化交互”单独测试，不能混入
20 kHz 截止期验收。

另验证过 ARMCC 全局 `Optimize for Time`。它把最大周期进一步降到约 7996 cycles，
但同一次 Mode 13 回归产生 `MOTOR_FAULT_SENSORLESS(13)`，因此已经撤回；最终工程
仍使用原 `-O2` 配置，只保留下述源代码级优化。

## 代码调整

1. 新增 `FastMath_SinCos()`：正余弦共用 LUT 和角度归一化；对于快环常见的单圈角度，使用一次条件回绕，避免通用浮点取整路径。
2. Clarke 变换在测量完成后只计算一次，观察器和电流控制共用同一组 alpha/beta 电流。
3. 电流 PI 的 Kp/Ki/Ts 只在初始化或配置提交时刷新；每周期只更新随母线变化的输出限幅。
4. 观察器的有效相电阻、平均电感、磁链平方、磁链倒数和受限 gamma 在初始化/配置提交时预计算。
5. `atan2` 的输出范围已知为 `[-pi, pi]`，观察器角度只需一次负值回绕。
6. 测量配置复制和合法性检查从每个 20 kHz 周期移到初始化/配置提交时。
7. 同一快环周期只读取一次命令应用前的生命周期快照，供保护、编码器和观察器路径共用。

配置提交后仍会同步刷新 PI、观察器和测量派生参数，因此运行时写入 R/L/磁链或
标定得到的新电流零偏不会使用旧缓存；Board Profile 保护参数在启动初始化时装载。

## 实机回归结果

| 项目 | 结果 |
| --- | --- |
| Keil 全量编译 | `0 Error(s), 0 Warning(s)`；Code 96644，RO-data 5848，RW-data 504，ZI-data 28752 bytes |
| 架构检查 | `project_entries=122 failures=0 warnings=0` |
| Mode 13 | 完整运行后自动返回 `mode=0/error=0`；活跃路径 7848 filtered / 8063 max cycles，0 overrun |
| Mode 15 | 1.6714 s 完成；母线最低 27.57 V；最大 Id 4.51 A；最大绝对 Iq 0.02 A；返回 `mode=0/error=0` |
| 编码器速度闭环 Mode 2 | 目标 0.10 rev/s（0.628 rad/s）；3~5 s 平均 0.631 rad/s；母线最低 27.75 V；最大绝对 Iq 1.72 A；50 个样本无故障；停机后 `mode=0/error=0` |
| 最终安全状态 | `mode=0/error=0`，速度与 Iq 回到 0，编码器 Online |

## 复测命令

Keil 编译后，可在固件运行时读取 DWT 统计；脚本会从 AXF 调试信息与 map 文件自动
解析 `MotorControlRuntime.fast_loop_metrics` 地址，不依赖硬编码 RAM 地址：

```powershell
pwsh -NoProfile -File tools/read_fast_loop_metrics.ps1
```

重点验收字段：

- `maximum_cycles < deadline_cycles`；
- `deadline_overrun_count == 0`；
- 活跃控制阶段的 `filtered_cycles` 应保持在 8500 cycles 以下；
- 每次测试结束必须确认 `mode=0`、`error=0`、电流和速度回零。
