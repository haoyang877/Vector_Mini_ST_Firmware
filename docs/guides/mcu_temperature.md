# MCU温度来源与保护

核对日期：2026-10-21。当前测量来源是STM32G431内部温度传感器和VREFINT，
不是绕组、MOS或环境温度。硬件入口见[`mcu_temperature.h`](../../firmware/platform/api/mcu_temperature.h)，
监督和保护见[`foc_sensing.c`](../../firmware/motor/foc/foc_sensing.c)。

ADC1的软件触发注入组读取TS和VREFINT；平台侧使用每颗芯片的校准数据换算。
2kHz监督读取完整结果并启动后续采样，不以温度转换延长20kHz控制业务。
显示温度使用低通，过温判断使用原始有效值；90°C触发MCU过温故障，
恢复门限为80°C且仍需显式CLEAR。持续无效/过期样本触发温度传感器故障。

当前CAN温度位于108/124的45B状态payload偏移34，i16、0.01°C；
有效位bit8表示MCU温度。无真实电机温度来源时偏移36无效，不能复制MCU温度。
旧标准CAN GET0x43/0x6E/0x6F、旧48B状态帧与HIL工程均已退役。
具体字段见[状态线路](../protocols/yg_protocol_motor_wire_v1.md)。

绕组热保护是另一条基于相电流平方与时间的模型路径，不是温度传感器；
其调度、失效关断和清错条件见[保护架构](../architecture/fault_protection.md)。
旧换算推导与当时实机记录保留在[历史说明](../archive/guides/mcu_temperature.md)及
[温度验证报告](../reports/2026-09/mcu_temperature_20260918.md)。
