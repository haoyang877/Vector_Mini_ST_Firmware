# 电气零位标定PWM提交超时定位（2026-10-29）

## 结论
定位到维护受理路径的全局临界区：前台同步准备并启相时PRIMASK仍置位，ADC完成后无法响应，零位第一个控制拍被推迟；之后执行正常长度的计算，到PWM检查时恰好落入更新边沿前的256计数禁止区。
本轮只测量、定位，没有修改生产代码、控制算法、PI或保护阈值。诊断代码全部在outputs独立副本。

## 证据链
第一版分段诊断第6次捕获失败，zero_count=0；ADC硬件完成相位参考为4700..4809，失败IRQ入口相位7707，推迟约16.10..16.71µs。三相提交检查时向上计数4292、周期4500，剩余208计数，约1.156µs；要求大于256计数，约1.422µs，因此返回失败，尚未写入三相比较寄存器。
失败首拍从IRQ入口到窗口检查31.322µs；另一正常零位首拍31.311µs，两者计算长度接近。失败不是FOC计算突然多耗十几微秒，也不是直接证明整次IRQ超过50µs。失败帧的exit字段沿用上一拍，不用于任何耗时计算。
第二版在维护受理锁前后补充3个时间点。多次测得外层锁18.894µs，其中PWM启用至解锁0.606µs。first4记录到：锁结束后0.228µs进入零位第0拍ADC，入口相位8201；对应ADC完成至入口延迟约18.84..19.45µs，表明该次ADC一直等待外层锁释放。
第二版first4较迟提交跨过更新边沿后窗口检查通过；因此延迟与故障不是单调关系，是否拒绝取决于当时载波相位。不能用重试成功否定锁导致的首拍推迟。
上述硬件完成参考以关相启动时两次轮询夹持得到，区间包含轮询/读寄存器量化；分段计时有插桩开销，绝对值不是无插桩WCET。

## 实际过程链
1. CAN Type160、kind2经协议作业服务进入MotorStateMachine_RequestIdentifiedMaintenance。此路径在前台同步受理，并非等待2kHz监督后才启相。
2. RequestIdentifiedMaintenance进入BspSystem_EnterCritical，PRIMASK=1；调用request_maintenance_locked。
3. 锁内读取状态、clear_running_data清理控制状态，begin_operation迁移到MAINTENANCE、发布maintenance_runnable，并完成动态守卫。
4. begin_operation调用enable_owned_power，再调用BspPwm_Enable。enable_owned_power的内部临界区是嵌套锁，退出它并不解除外层PRIMASK。
5. 前台继续写受理票据，直到RequestIdentifiedMaintenance的BspSystem_ExitCritical才重新允许ADC。ADC优先级0也无法抢占全局关中断区。
6. ADC1_2_IRQHandler → Adc_CompleteFast → BspApp_OnFastSample → App_FastTick → MotorAxis_FastTick。
7. 快拍先做模拟量、编码器和保护处理，run_fast_task → MotorMaintenance_Step → EncoderCalibrationTask_StepElectricalZero。第0拍复位电流控制器并爬升Id目标，Iq=0，调用FOC_Current。
8. FOC_CurrentWithReference → Clarke/Park → d/q PI及电压调制 → foc_apply_modulation → 逆Park/SVM → BspPwm_SetDuty → McuPwm_CommitThree → pwm_can_commit。
9. 剩余208≤256时不写CCR，pwm_commit_missed锁存s_output_fault并关相；FOC报告CONTROL_OVERRUN，快故障路径确认关闭，监督和CAN再报告故障。

## 计算段耗时
以下为第一版失败首拍的诊断值，阶段包含相邻调用和计时开销，不等于单个底层函数独立基准。

| 阶段 | µs |
| --- | ---: |
| IRQ入口→完整ADC帧 | 4.394 |
| 完整帧→编码器请求结束（含APP入口） | 4.061 |
| 模拟量、电流及采样准备 | 5.617 |
| 观测器 | 2.650 |
| 编码器回读/解码/测速 | 6.028 |
| 快故障及调度 | 0.906 |
| 零位首拍准备 | 2.244 |
| FOC→占空比参数 | 4.844 |
| BSP参数转换→定时器检查 | 0.578 |
| 合计 | 31.322 |

计算链仍有优化余地，但本次异常增加的是进入IRQ前的等待。优先处理维护受理的全局锁范围及首拍启相时序，保留最终状态/代际/STOP/故障复核和256计数守卫；本轮未实施该修改。

## 原始证据
- outputs/bsp-v1-implementation-20261026/hil-zero-deadline-20261029/trial6-result.json：首次拒绝帧。
- 同目录timing-analysis.json、combined-analysis.json：分段和区间计算。
- outputs/bsp-v1-implementation-20261026/hil-zero-lock-20261029/first1-result.json：正常首拍对照。
- 同目录first4-result.json、lock-causality.json：受理锁与待处理中断直接对应。
- 两版program.json、builds.json及探针日志保留下载、完整读回和参数区证据；所有限时诊断均确认持久参数未变。
已恢复原无插桩Commissioning镜像，完整镜像读回通过；运行复核ADC约20kHz、监督约2kHz、故障0、输出关闭、看门狗未调试冻结。所有成功标定和参数保持，CRC及RAM/Flash一致。
恢复证据：outputs/bsp-v1-implementation-20261026/hil-zero-restore-20261029/program.json、final-stop-check.json。
本轮汇总：outputs/bsp-v1-implementation-20261026/hil-zero-deadline-20261029/summary.json。quick验证及文档检查通过。本轮完成定位，未实施修复。
