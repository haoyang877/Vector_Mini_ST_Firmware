# AT32 ADC中断关相实测（2026-10-29）

## 范围与版本
用户要求进行实机ADC中断计时。平台为已核对UID的AT32F456，运行180MHz，使用ESC-APP-AT32F456永久关相目标。
基线为上一轮双板及完整PR已验证的adc-temperature-final-20261029；当前工作区另一项CAN修改没有混入本次测量。
诊断代码只在outputs/bsp-v1-implementation-20261026/adc-irq-diagnostic-20261029中增加；生产BSP文件不改。

## 测量边界
沿用ADC1_2_IRQHandler原DWT起点和BspApp_OnFastSample返回后的DWT终点，包含ADC完整帧检查/发布、编码器和上层快任务。
原APP_FAST_MAX_CYCLES=9000检查保留。最小/最大/累计统计在保护检查之后执行；不包含异常硬件入栈/退栈、C函数序言/尾声和统计收尾。
这不是单独BspAdc函数耗时，也不是启相标定或闭环状态的最坏执行时间证明。

## 结果

| 项目 | 实测 |
| --- | ---: |
| 中断样本数 | 605,095 |
| 最小 | 4281周期 / 23.783µs |
| 平均 | 4921.344周期 / 27.341µs |
| 最大 | 5587周期 / 31.039µs |
| 快采样频率 | 20000.067Hz |
| 统计收尾最大开销 | 54周期 / 0.300µs |
| 超时、ADC故障、电机故障 | 0 |

连续观察约30秒，所有轮询均确认功率输出关闭，时基单调，没有发现复位；运行时看门狗未调试冻结。
相对50µs中断主体软件预算，所测最大值剩余约18.961µs；不将其解释为PWM提交截止的余量。

## 下载、恢复与证据
下载前备份全部256KiB Flash及16KiB参数区。诊断镜像89,608个有效字节均独立读回一致，未写页及参数保持。
实测原始数据：outputs/bsp-v1-implementation-20261026/hil-adc-irq-20261029/measurement.json。
诊断构建、镜像SHA、J-Link日志、下载读回：同目录的production-build.log、diagnostic-build.log、candidate.json、program.json。
无插桩固件恢复证据：outputs/bsp-v1-implementation-20261026/hil-adc-restore-20261029/。
已恢复并整片独立读回验证；89,516个镜像字节无差异，参数区与初始备份完全一致。复位后ADC约20kHz、监督任务约2kHz；故障码为0，CPU运行、输出关闭、看门狗未调试冻结。
汇总：outputs/bsp-v1-implementation-20261026/hil-adc-irq-20261029/summary.json。
首次监测脚本因时钟符号名与AT32实际符号不匹配，在采样前退出；已修正并重新复位计时，其记录保留为measurement-attempt1.json，不计入结果。
