# Pitch mode 3：30 秒保护及重复性排查

日期：2026-09-09。接续 [上一轮记录](pitch_mode3_burst_20260909.md)。用户先提出相电流 6 A 最长 100 s，随后明确改为 **30 s 保护**，本轮以最后的 30 s 指令为准。

## 本轮结果

共执行 **12 轮、54 段、累计 687 s 指令观察时间**，其中 11 轮自动验收通过；HOLD 恢复修正前的 pitch39 末段 HOLD 占比不足，失败记录保留。修正后 pitch43～46 共四轮全部通过：包含微小角度、8 s 短距离重复、−12°～+46°两次往返及每段 20 s 微小角度观察。

- 修正后，连续 HOLD 确认之后的最大绝对位置误差 **0.2641°**；各段最后 2 s HOLD 占比均为 100%。
- 本轮 13,734,719 个已统计运行中断周期中，≥50 μs 次数为 0。包括启动/STOP 的最大记录为 8415 cycles（49.50 μs）；最终镜像四轮最大为 8311 cycles（48.89 μs）。这是当前台架负载下的观测，不是所有通信/异常负载的 WCET 证明。
- 实测相电流最大 **5.3993 A**；一次 ARM 内超过 4 A 的最大累计时间 **4.4599 s**，未触发 6 A/30 s 附加保护。
- 没有复现上一轮约 0.66°的保持后滑动。它在本轮逻辑修正之前的几轮也未复现，所以不能把“后续通过”直接解释为滑动根因已完全证明并消除。
- 最后回读：mode 0、error 0、HIL inactive、三相 PWM 关闭；位置约 −0.088°。Flash 参数区及 pitch 校准字节保持不变。运行增益仍为 8/2、0.5/2，候选增益未写入 Flash。

## 保护和参数

- 相电流采样峰值达到 6 A，或电流采样为非有限数，锁存台架停止。
- 任一相电流绝对值超过 4 A 的时间，在一次 ARM 内累计至 **30,000,000 μs** 后停止；低于阈值不清零。下一次 ARM 清零本次计数，不代表热状态已复位或已冷却。普通镜像没有这项 HIL 附加保护。
- 主机在 ARM 前核对板端 6 A、4 A、30 s 契约。旧 4.5 s 镜像不能用新的 `--phase-burst` 后端继续运动。
- native C 回归验证 29.999 s 不触发、30 s 触发，包含短暂回落、三相分别达到 6 A、重新 ARM、非有限数等情况。30 s 累计时间保护在主机测试中验证；未通过实机强制维持大电流到超时来验证热极限。
- pitch 的 Flash 轴信息仍为 −0.3～+0.9 rad、45°/s；目标保留约 5°边界余量。增益保持 RAM 候选：位置 Kp/Kd=8/2，速度 Kp/Ki=0.5/2，Iq 指令上限 6 A。

## 中断开销修正

STM32G4 的共用 ADC IRQ 原先无条件调用两个 `HAL_ADC_IRQHandler()`。HAL 内部对各事件本就检查 ISR 标志与 IER 使能，因此在板级 IRQ 入口先检查 `ISR & IER`，跳过没有已使能挂起事件的 ADC，继续把实际事件交给原 HAL 处理函数。ADC1、ADC2 及所有启用的事件/故障路径均保留。

此改动限定于现有 MCU 硬件中断文件，未在算法、调试服务中引入 Vendor 寄存器。现有 `Core/Src/stm32g4xx_it.c` 是板级生成代码所在位置，后续 CubeMX 重生成须保留该分发修改。主机回归编译实际 IRQ 函数、用寄存器桩覆盖 11 个事件位的停用/挂起组合以及两个 ADC 同时挂起；普通和 HIL 工程也分别编译通过。

## 排查方法

保持上一轮控制算法和增益不变，先固定接近方向，再重复微小指令和延长观察时间。每轮结束自动 STOP，并核对 mode/error 和三相 PWM 已关闭。每轮分析后才决定下一轮；不靠目测来判定电机端的位置保持。

验收包含：最后 2 s 误差 ≤0.3°、HOLD 100%；指令后连续 50 ms HOLD 一旦成立，随后直到下一指令的全部采样误差仍须 ≤0.3°；无丢帧、无饱和、电流/时间预算未触发；启动、运行及停止的 IRQ 最大耗时均 <8500 cycles（50 μs）。

此前微小试验的异常是可见于数据的事件：保持后偏离约 0.66°。其前约 0.3 s，积分支撑约 −1.87 A、Iq 约 −1.65 A，随后发生负向滑动。积分电流与静摩擦是待检验假设，不能仅靠同一张波形认定因果。

## HOLD 丢失后的纠偏状态修正

代码检查发现：进入 HOLD 时清除 `target_transition_active`；随后因位置/速度偏离退出 HOLD 时，原实现没有重新置位。`PositionCascade_UpdateIntegralTransport()` 却以该标志为前提，因此已存在的“同时反对位置、速度纠偏时卸载积分电流”逻辑在这条恢复路径被跳过，直到下一次新目标指令。

修正只在 HOLD→SETTLE 时重新标记纠偏进行中，使用既有卸载速率/上限；正常 HOLD 继续保留支撑电流，轨迹、迟滞窗口和 PID 增益保持原值。

新增实际 C 回归先建立非零支撑并进入 HOLD，然后关闭普通积分与静摩擦附加积分以隔离被测逻辑：沿有利方向的支撑保持不变；重新 HOLD 后向反方向偏离，须按既有限斜率卸载；正负两向均覆盖。修正前失败于未卸载断言（`host_hold_transport_red2`），修正后全部通过（`host_hold_transport_green`）。这证明恢复路径的缺口，并未独立证明此前自发滑动的起因。

对照过位置 Kp=10 与 Kp=8。两者在相同的 `0,-1.5,0,-1.5,0` 序列上均通过；Kp=10 也出现约 0.259°的末段误差，没有足够证据判定更优，最终继续使用 Kp=8。Kp=10 配置仅作为历史试验对照，不写入 Flash。

## 逐轮记录

| 试验目录 | 目标 / ° | 每段时间 | 结果 |
| --- | --- | ---: | --- |
| pitch35_negative_approach | −3,0,0.09,−0.09,0 | 12 s | PASS |
| pitch36_negative_repeat | 同上 | 12 s | PASS |
| pitch37_positive_approach | 3,0,0.09,−0.09,0 | 12 s | PASS |
| pitch38_short_approach | −1.5,0,0.09,−0.09,0 | 12 s | PASS |
| pitch39_prepare_rearm | −1.5 | 8 s | FAIL：末段误差 0.259°，HOLD 62.99%；电流/IRQ 正常 |
| pitch40_rearm_micro | 0,0.09,−0.09,0 | 15 s | PASS；从上轮停用后的位置重新 ARM |
| pitch41_kp10_small | 0,−1.5,0,−1.5,0 | 12 s | PASS，Kp=10；没有证明稳定优势 |
| pitch42_kp8_control | 同上 | 12 s | PASS，恢复 Kp=8 |
| pitch43_hold_fix_micro | 0,0.09,−0.09,0,−1.5,0 | 12 s | PASS；HOLD 恢复修正后的镜像 |
| pitch44_hold_fix_short8 | −1.5,0,−1.5,0 | 8 s | PASS |
| pitch45_hold_fix_range | 46,−12,46,−12,0 | 15 s | PASS；相电流峰值 5.3993 A |
| pitch46_hold_fix_long_micro | 0,0.09,−0.09,0 | 20 s | PASS；保持后偏差最大 0.2109° |

每段时间包含运动与保持，不等于在目标静止保持的净时间。前一次运动和停用后的实际起点保存在原始记录中；同一序列也不是完全相同的机械初态。

## 重复执行示例

在仓库根目录、同一已校零的 pitch 台架与匹配 session 下执行。命令会实际使能和运动，结束后自动 STOP；每轮名称必须唯一。以下重放最后的长观察微小角度用例，其他已测路径可使用上表目标和时间。

```powershell
$taskPython = 'C:/Users/yang yang/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
$taskSession = 'outputs/pitch_validation_20260909/session'
$taskProfile = 'tests/mode3/pitch_candidate_20260909.json'
$taskName = 'pitch_repeat_' + [Guid]::NewGuid().ToString('N')
& $taskPython tools/servo_hil_run.py --name $taskName --session-dir $taskSession --axis-profile $taskProfile --phase-burst --kp 8 --kd 2 --speed-kp .5 --speed-ki 2 --targets=0,0.09,-0.09,0 --seconds 20
if ($LASTEXITCODE -ne 0) { throw 'Trial failed: inspect shutdown status' }
& $taskPython tools/run_mode3_validation.py analyze --profile $taskProfile --trial (Join-Path $taskSession $taskName)
if ($LASTEXITCODE -ne 0) { throw 'Acceptance failed: inspect data before continuing' }
```

## 追溯

本轮初始镜像：`outputs/pitch_validation_20260909/session/image_pitch_guard30_irq/`。每次下载校验完整镜像及原参数/校准保留。每轮目录包含 `capture.tsv`、`trial.json`、`mode3_acceptance.json`；`trial.json` 保存实际增益、保护回读、IRQ 统计、配置快照和镜像哈希。

HOLD 恢复修正后的镜像：`image_pitch_hold_transport`；对应构建日志 `build_hold_transport.log`、`build_normal_hold_transport.log`。正常固件仅构建，不下载。

编译日志：`outputs/pitch_validation_20260909/build_guard30_irq.log`、`build_normal_guard30_irq.log`。主机回归：`outputs/pitch_validation_20260909/host_guard30_irq/`。正常镜像只构建，本次台架运行 HIL 镜像。

最终主机回归为 `host_hold_transport_green/`：26 组伺服场景（含 72 组组合）、独立轨迹/编码器/ADC IRQ/HIL/AXS1 检查，以及 5 项边界、13 项验收分析、2 项轴记录 Python 检查通过。普通及 HIL 最终构建均 0 错误 0 警告。

汇总：`outputs/pitch_validation_20260909/repeat_30s_summary.json`。最终停用/镜像/参数回读：`repeat_30s_final_disabled_state.json`，重新核对 103180 个镜像字节、完整参数区 SHA-256 `160ba507965334ab3bc9b0327cc55e1705653b41767a71ffef6e86db9c076c6e`。最终 AXF SHA-256：`15cd3cd201b463fa1aba9108b7f5c650f72210cdc302c7e929ea36de53afa00b`。

## 提交范围与独立构建验证

本次提交包含上述 mode 3 修正、启动/遥测时序优化、HIL 相电流保护、pitch 配置与测试工具。纳入台架实际使用的 `POSITION_LOOP_FREQ=2000U`，该公共频率也影响模式 18 的调用周期；模式 18 算法、schema 10 参数迁移和上位机界面修改仍保留在工作区，未混入本次提交。

从 Git 暂存区导出独立源码至 `outputs/pitch_commit_20260909/source/` 后验证：26 组 C 伺服场景（含 72 组组合），独立轨迹、编码器、ADC IRQ、HIL、AXS1 测试，以及 25 项 Python 检查全部通过。普通固件与 HIL 固件均 0 错误、0 警告；普通 Code=94804 B、ZI=31704 B，HIL Code=96204 B、ZI=31716 B。日志分别为该目录下的 `../build_normal.log`、`build_hil.log`，主机报告为 `../host/result.json`。

此独立提交快照仍为 schema 9，板上已验证镜像为包含工作区 schema 10 修改的构建；两者不是相同二进制。独立快照仅构建和离线测试，未下载到 pitch 板卡，不应直接用于解释当前 schema 10 参数区。原始波形、镜像和 Flash 备份保留在本地 `outputs/`，不纳入 Git。
