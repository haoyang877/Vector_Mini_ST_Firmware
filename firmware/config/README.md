# 固件配置

| 文件 | 职责 |
| --- | --- |
| config.h | 板型编号、工程输入校验、默认电机型号和快慢环时基 |
| board_config.h | 实际接线能力、采样电路、时钟参数和 Flash 范围 |
| motor_profile_config.h | 电机物理模型、标定输入及热参数 |
| application_config.h | 控制、保护和标定策略，以及派生参数 |

板卡由各 Keil 工程的 `BSP_BOARD` 选择：STM32G4 为 1，AT32F456 为 2。
电机通过 `FOC_ACTIVE_MOTOR_PROFILE` 独立选择，不由板型自动推断。
AT32F456 产品 APP 按节点及 AXS2 修订 2 从同一固件选择 HT8115-4 或 YQ5708。型号表与单位见[五轴配置](../../docs/guides/at32f456_five_axes.md)。STM32 及原有独立夹具沿用编译期默认型号。AT32F456 未装配或记录不匹配时保持关相。

两板共用 20 kHz 快环、2 kHz 监督拍；零偏窗口为 2048 个快拍。
板卡能力直接决定 BSP_ENABLED，不再提供逐项功能开关。
引脚、外设、ADC 结果通道、DMA 和 IRQ 映射分别在对应 MCU 的 `mcu_resources.h`。
修改 STM32 映射还需同步检查 CubeMX 输入和生成代码。

命令/标定电流上限为 10 A，通用默认活动限流为 6 A，AT32F456 产品轴运动上限 7 A；18 A 过流宏是保留的策略输入，
其存在不代表当前运行链路已实现统一三相跳闸。详见[软件架构](../../docs/architecture/software_architecture.md)。

YQ-5708 的热参数仍为空，配置校验要求热参数有效；关闭热调度不会免除该校验。
AT32 的 MCU 温度校准参数也未提供。有功率启动/标定须满足实际准入条件，不能用占位值绕过。
