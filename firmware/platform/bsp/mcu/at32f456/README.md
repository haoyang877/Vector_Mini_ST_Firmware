# AT32F456 BSP实现

公开功能接口在../../include。本目录保存实际MCU操作，不新增转发层。

| 文件 | 职责 |
| --- | --- |
| system.c | 系统时钟、时基、看门狗、BSP启动顺序及系统IRQ |
| adc.c | ADC配置、DMA、IRQ、原码缓存、故障和Flash暂停/恢复；唯一私有状态 |
| pwm.c | PWM定时器、三相比较值、六路输出、门极GPIO准备与复用 |
| encoder_spi.c | 编码器SPI/CS/DMA事务、超时及失败收尾；不解析器件协议 |
| can.c | CAN控制器、硬件收发槽、错误和IRQ |
| flash.c | 固定区域读写、控制器等待、采样暂停恢复及关相检查 |
| led.c | 实际LED GPIO与亮度输出，只接受共用LED逻辑 |
| temperature.c | MCU温度/VREF和外部NTC的换算及有效性 |
| boot.c | 此板现有APP/Loader交接能力 |
| at32f45x_conf.h | 厂商驱动编译选择 |

引脚、比例和Flash范围在platform/config/boards/at32f456.h；控制周期和测试配置在app/config。
ADC状态不跨文件传递；没有adc_sampling_*或system_setup私有层。
实现中不保留临时诊断入口、测试快照和测试统计。
两板使用相同功能命名，实际器件协议在platform/devices。

详细设计见[当前BSP架构](../../../../../docs/architecture/bsp_architecture_v1.md)。
