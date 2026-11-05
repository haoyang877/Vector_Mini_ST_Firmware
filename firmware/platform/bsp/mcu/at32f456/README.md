# AT32F456 BSP实现

当前范围：保留PWM、ADC、Encoder（SPI）、CAN与Flash及必要系统支撑。LED、UART已删除，
不保留接口、占位实现或工程引用。见[删除记录](../../../../../docs/plans/active/2026-10-30-remove-led-uart-v1.md)。

公开功能接口在../../include。本目录保存实际MCU操作，不新增转发层。

| 文件 | 职责 |
| --- | --- |
| mcu_resources.h | PWM/TMR、ADC输入与结果、GPIO、SPI、CAN、DMA及IRQ资源映射；登记未启用资源 |
| system.c | 系统时钟、时基、看门狗、BSP启动顺序及系统IRQ |
| adc.c | ADC初始化、原码读取、启停、中断及单位换算 |
| pwm.c | PWM初始化、周期读取、比较值写入、启停与通道查询 |
| encoder_spi.c | GPIO/SPI2/DMA2初始化及收发，直接实现统一Begin/ReadAngle |
| can.c | CAN初始化、寄存器收发、启动及接收IRQ；初始化声明集中在bsp_internal.h |
| flash.c | 固定区域读写、控制器等待、采样暂停恢复及关相检查 |
| boot.c | 此板现有APP/Loader交接能力 |
| at32f45x_conf.h | 厂商驱动编译选择 |

引脚、外设通道与IRQ在本目录mcu_resources.h；比例、时钟数值和Flash范围仍在
firmware/config/board_config.h，控制周期在firmware/config/config.h。
ADC硬件读取、启停与IRQ集中在本目录adc.c。
ADC零偏、单位换算和NTC公式也在本目录adc.c内完成。
PWM占空比换算和硬件操作全部在本目录pwm.c，不保留转发文件、状态或保护。
PA5/PA6 的 sin/cos 与相电流共用 TMR1 的 20 kHz 触发，每路只转换一次。快环将 ADC2 的两个结果一起发布给 4 kHz 外环。
实现中不保留临时诊断入口、测试快照和测试统计。
先前ADC/PWM接口精简未运行测试和编译；资源配置收拢的独立离线验证见
[资源配置计划](../../../../../docs/plans/active/2026-10-30-mcu-resources-v1.md)。
两板编码器在各自MCU的encoder_spi.c直接实现Begin和ReadAngle，可选16/21位单圈计数；
现有控制环使用65536计数域，无CRC/超时/事务状态。
PA7 电机 NTC 使用 2 kHz 普通组，直接从 ADC2 普通结果寄存器读取。

AT32 CAN硬件运行和RX IRQ在本目录can.c，直接实现统一BspCan接口。
DLC/字节数换算直接在CAN收发函数中完成；没有独立文件、票据、状态枚举或自动恢复。

详细设计见[当前BSP架构](../../../../../docs/architecture/bsp_architecture_v1.md)。
