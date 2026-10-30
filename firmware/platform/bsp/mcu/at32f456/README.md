# AT32F456 BSP实现

当前范围：保留PWM、ADC、Encoder（SPI）、CAN与Flash及必要系统支撑。LED、UART已删除，
不保留接口、占位实现或工程引用。见[删除记录](../../../../../docs/plans/active/2026-10-30-remove-led-uart-v1.md)。

公开功能接口在../../include。本目录保存实际MCU操作，不新增转发层。

| 文件 | 职责 |
| --- | --- |
| system.c | 系统时钟、时基、看门狗、BSP启动顺序及系统IRQ |
| adc.c | ADC初始化、DMA、原码读取、启停、IRQ及片内校准量换算 |
| pwm.c | PWM初始化、周期读取、比较值写入、启停与通道查询 |
| encoder_spi.c | GPIO/SPI2/DMA2初始化及收发，直接实现统一Begin/ReadAngle |
| can.c | CAN初始化、寄存器收发、启动及接收IRQ；初始化声明集中在bsp_internal.h |
| flash.c | 固定区域读写、控制器等待、采样暂停恢复及关相检查 |
| boot.c | 此板现有APP/Loader交接能力 |
| at32f45x_conf.h | 厂商驱动编译选择 |

引脚、比例和Flash范围在firmware/config/board_config.h；控制周期和测试配置在firmware/config。
ADC硬件读取、启停与IRQ集中在本目录adc.c；读取直接返回float，没有帧状态或采样缓存。
ADC零偏、单位换算和NTC公式也在本目录adc.c内完成。
PWM占空比换算和硬件操作全部在本目录pwm.c，不保留转发文件、状态或保护。
sin/cos在adc.c私有s_aux_dma中观察；顺序按启用的NTC、sin、cos排列，每组两半。
实现中不保留临时诊断入口、测试快照和测试统计。
本轮AT32 ADC/PWM按用户要求仅提供实现，测试和编译均未运行。
两板编码器在各自MCU的encoder_spi.c直接实现Begin和ReadAngle，使用相同65536计数域；无CRC/超时/事务状态。
DMA缓冲为本文件私有，BSP不增加空转发层。

AT32 CAN硬件运行和RX IRQ在本目录can.c，直接实现统一BspCan接口。
DLC/字节数换算直接在CAN收发函数中完成；没有独立文件、票据、状态枚举或自动恢复。

详细设计见[当前BSP架构](../../../../../docs/architecture/bsp_architecture_v1.md)。
