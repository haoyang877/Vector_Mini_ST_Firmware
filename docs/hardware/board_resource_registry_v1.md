# YG-ESC-板卡资源与适配电机配置

更新：2026-10-23。统一记录 Vector Mini ST 和 YG_ESC_V1.0（原R40）的资源及适配电机。

适配对象、修改边界和后置事项见[统一框架适配范围草案](../architecture/dual_board_adaptation_scope_v1.md)。
本表登记资源和参数，不以型号已列出表示完整APP或实机验收通过。

## 板卡资源

| 项目 | Vector Mini ST 旧板 | YG_ESC_V1.0 新板 |
| --- | --- | --- |
| MCU | STM32G431CBUx | AT32F456CCU7 |
| 主频 / 外部晶振 | 170MHz / 8MHz | 180MHz / 8MHz |
| Flash / RAM | 128KiB / 32KiB | 256KiB / 108KiB |
| 参数存储 | 0x0801C000，16KiB | 0x0803C000，16KiB |
| PWM / 控制频率 | TIM1，20kHz | TMR1，20kHz |
| U相PWM（上 / 下桥） | PA8 / PA7，CH1 / CH1N | PA9 / PB0，CH2 / CH2C |
| V相PWM（上 / 下桥） | PA9 / PB0，CH2 / CH2N | PA10 / PB1，CH3 / CH3C |
| W相PWM（上 / 下桥） | PA10 / PB1，CH3 / CH3N | PA11 / PB4，CH4 / CH4C |
| ADC触发 / 监督时基 | TIM1_CH4 / TIM7，2kHz | TMR1_CH1 / TMR6，2kHz |
| 三相电流ADC | PA5、PA6、PC4 / ADC2 | PA0、PA1、PA2 / ADC1 |
| 母线电压ADC | PA4 / ADC2 | PA3 / ADC1 |
| 母线电流ADC | 无独立采样 | PA4 / ADC2 |
| 电机温度ADC | PA3预留，当前未用 | PA7 / ADC2 |
| MCU温度 | ADC1内部温度及VREF | 内部温度未启用 |
| 电机编码器SPI | SPI2；CS/SCK/MISO/MOSI：PB12/13/14/15 | SPI2；CS/SCK/MISO/MOSI：PB12/13/14/15 |
| 输出端反馈资源 | SPI1：PB3/4/5，CS：PC11 | PA5、PA6 / ADC2，两路sin/cos |
| CAN引脚 / 速率 | RX：PB8，TX：PB9；1M/5M | RX：PB8，TX：PB9；1M/5M |
| UART资源 | TX：PB6，RX：PB7，当前APP未使用 | TX：PA15，RX：PB3，尚未适配 |
| DMA当前用途 | DMA1_CH1：RGB | DMA1_CH2：辅助ADC；CH3预留内部温度 |
| SWD | PA13 / PA14 | PA13 / PA14 |
| LED资源（使用逻辑后续适配） | PA15红、PC10绿；PA2：RGB | PC13红、PC14蓝；PB5/PB6：双色灯 |
| 栅极驱动 / MOS | FD6288Q / CSD18540Q5B | FD6288Q / WSD6035DN33 |
| 相电流采样 | 低侧分流；当前6mΩ、10倍增益 | 相线分流；4mΩ、INA240A1、20倍增益 |
| 母线电流采样 | 无 | 2mΩ、INA240A1、20倍增益 |
| 母线分压 | 10kΩ / 1kΩ | 100kΩ / 3.3kΩ |

## 适配电机配置

板卡和电机独立选择；YG_ESC_V1.0支持配置多个电机型号，后续继续扩展。
电流统一为幅值A，所有型号的电流环默认带宽为1kHz。

| 项目 | HT8115-4 | YQ-5708 | YQ-5740 | YQ-8116 |
| --- | --- | --- | --- | --- |
| 当前配套板卡 | Vector Mini ST | YG_ESC_V1.0 | YG_ESC_V1.0 | YG_ESC_V1.0 |
| 配置文件 | [ht8115_4.h](../../firmware/motor/profiles/ht8115_4.h) | [yq_5708.h](../../firmware/motor/profiles/yq_5708.h) | [yq_5740.h](../../firmware/motor/profiles/yq_5740.h) | [yq_8116.h](../../firmware/motor/profiles/yq_8116.h) |
| 选择标识后缀 `FOC_MOTOR_PROFILE_` | `HT8115_4`（1） | `YQ_5708`（2） | `YQ_5740`（3） | `YQ_8116`（4） |
| 装配配置 | 无阻尼环 / 有阻尼环 | 当前装配 | 当前装配 | 暂定HT基线 |
| 极对数 | 21 | 14 | 14 | 暂同HT |
| 接法 | 星形 | 星形 | 星形 | 暂同HT |
| 传动 | 直驱1:1 | 8:1 | **40:1** | 暂同HT |
| 每相电阻 | 1.905Ω | 0.55Ω | 同5708 | 暂同HT |
| Ld / Lq | 各1.635mH | 暂估各142.5µH | 同5708 | 暂同HT |
| 磁链 | 0.0175025Wb | 0.003814286Wb | 同5708 | 暂同HT |
| 转矩常数（电机轴 / Iq幅值） | 约0.55133N·m/A，由磁链推导 | 0.0801N·m/A | 同5708 | 暂同HT，待确认 |
| 转子惯量 | 待提供，两种装配分别确认 | 0.0005kg·m² | 同5708 | 待提供 |
| 额定电流（幅值） | **3A** | 约3.54A | 同5708 | 暂同HT |
| 短时电流（幅值） | **7A**，时长待确认 | 约10.61A、30s | 同5708 | 暂同HT |
| 电机轴最高转速 | **300rpm** | 2500rpm | 同5708 | 暂同HT |
| 电流环默认带宽 | **1kHz** | **1kHz** | **1kHz** | **1kHz** |
| 完整APP默认配置 | 已有；两种阻尼环分别选择 | 热参数待补 | 热参数待补 | 详细参数待确认 |

HT阻尼环通过现有`MOTOR_HAS_DAMPING_RING=0/1`选择；机械参数后续分别完善。
YQ-5740复用5708电气参数，只更改型号和减速比。YQ-8116全部继承值均为暂定。
5708/5740磁链按`ψ=Kt/(1.5×极对数)`反算；惯量按用户确认的SI值，不再乘截图的e-4。
惯量先作为型号配置数据保存，未新增通信/Flash字段，也未接入控制算法。
5708/5740电流幅值由原2.5/7.5Arms折算；电机规格不自动改变当前台架的3A保护。
新默认值不覆盖已存PI；配置选择和后续新增型号的方法见[配置说明](../../firmware/motor/profiles/README.md)。

## 配置入口与备注

- 旧板：[板卡配置](../../firmware/platform/config/boards/stm32g4.h)、[电机选择入口](../../firmware/motor/motor_profile_selection.h)。
- YG_ESC_V1.0：[板卡配置](../../firmware/platform/config/boards/at32f456.h)、[台架电机配置](../../tests/bench/at32f456/closedloop_bench.c)。现有文件名及宏暂保留R40。
- 旧板图纸为2mΩ，当前默认工程为6mΩ；新板PA3电压、PA4电流按实板确认。PB5有对地0Ω，使用LED前需确认实装。
- 详细参数依据与电流能力核算见[新板输入记录](r40_at32f456_configuration_v1.md)；过程和测试结果见[适配计划](../plans/active/2026-10-22-at32f456-port-v1.md)。

后续更换板卡或电机时，分别更新对应列及其配置入口；不修改中间层与BSP公共接口。
