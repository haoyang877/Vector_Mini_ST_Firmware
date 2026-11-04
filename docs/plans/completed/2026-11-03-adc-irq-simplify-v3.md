# ADC入口分离采样和控制 v3

## 最终目标

按用户确认的“采样 → 控制 → 计数”组织ADC快中断，并纠正直接调用MCU内部接口的层级问题。
AT32 main只调用公开BspAdc_CompleteFast、MotorAxis_SampleFast、MotorAxis_ControlFast，
最后递增g_app_fast_samples。完成确认函数移入BSP公开契约，直接由MCU实现，不另加转发包装。
STM32保留HAL完成回调位置，按相同次序执行两个轴阶段，保证回调结束后再清JEOS。

SampleFast包含原快环的编码器Begin、电压、限流参数、零偏校正、三相/母线电流读取、
Clarke和观测器计算，以及编码器Read/Update；ControlFast保留原生命周期、控制/标定和结果发布。
三相实际读取仍由MotorCurrent_Update通过BspAdc_ReadChannel进行，没有遗漏或重复。
零偏未完成时仍读原码校正，不提前使用未校正的相电流。两个阶段之间不增加状态、副本或标志。

## 基线与不变项

基线函数保存在outputs/adc-irq-simplify-20261103/v3-before/，完整历史候选见v1和v2记录。
对比采样阶段与控制阶段的合并语句序列，要求与拆分前MotorAxis_FastTick完全一致。
保持调用顺序、原码/物理量单位、保护/状态/输出、硬件配置、20kHz频率、中断优先级、
零偏发布以及参数/协议ABI。计数仍只在控制完成后增加，供看门狗识别快环停滞。
AT32整个ADC入口保持SRAM布局；不为计数保留快路径包装函数。

## 验证与回退

定向原生检查覆盖无事件/缺辅ADC不调度、清标志后采样、采样后控制、控制后计数、
三相电流/母线量化、零偏窗口与失败冻结、ADC/SPI重叠及40种状态分支、两板APP轨迹对照。
完整PR/release和三套固件在独立源码副本运行，避免并行会话构建争用输出。
各阶段失败日志保留；不烧录，不操作硬件，不将离线结果当作实测时序结论。
回退只撤销此方案改动，不覆盖同时进行的协议/服务绑定修改。

## 结果

已完成。完整release门禁通过，含格式、lint、124项Python单元、16项集成、43组原生检查、
分层/接口/文档检查、Keil普通STM32 APP重建和AXF/HEX/MAP哈希；同一冻结源码的AT32 APP、
STM32 Loader APP也均0错误0警告。最终代码、测试、工程输入与冻结候选逐文件哈希一致。

拆分前后合并的轴快环语句序列一致，四组APP完整事件轨迹一致；三相电流量化、零偏窗口、
失败冻结、无事件/缺辅ADC、清标志后采样、控制后计数及40种状态分支检查均通过。
两板main允许有各自ADC适配；测试继续验证各工程只链接本板入口。
同期两板工程的通信源显示顺序不同，比较改为按路径排序，仍严格核对源码与编译选项。

AT32反汇编确认ADC向量直接调用BspAdc_CompleteFast、MotorAxis_SampleFast、MotorAxis_ControlFast，
然后写入健康计数；三个被调函数和中断入口全部位于0x2000开头的SRAM段，没有额外APP转发。
未测量实机最坏执行时间或栈峰值，不以构建成功代替实机验收。

实现过程中motor_axis.c被VS Code C/C++ IntelliSense锁定；经用户授权只结束该语言服务进程后
完成修改，没有关闭编辑器。较早v2的格式/旧断言/并行构建失败记录保留；最终独立副本通过。

证据：

- [最终摘要](../../../outputs/adc-irq-simplify-20261103/v3-summary.json)
- [合并语句顺序对照](../../../outputs/adc-irq-simplify-20261103/v3-source-comparison.json)
- [SRAM入口反汇编](../../../outputs/adc-irq-simplify-20261103/v3-binary-inspection.json)
- [完整release摘要](../../../outputs/adc-v3-source/outputs/runs/adc-irq-v3-release/summary.json)
- [AT32与Loader APP构建](../../../outputs/adc-v3-source/outputs/extra-builds/builds.json)
- [冻结源码清单](../../../outputs/adc-irq-simplify-20261103/v3-source-manifest.json)
