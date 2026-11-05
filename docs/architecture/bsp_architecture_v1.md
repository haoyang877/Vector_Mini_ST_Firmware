# BSP 架构与文件归属

更新：2026-11-03。当前有效接口以[职责约定](bsp_design_rules.md)和源码为准。

```text
firmware/platform/
├─ bsp/
│  ├─ include/               公共能力、单位、生命周期和失败语义
│  ├─ bsp_internal.h        初始化、采样启停和台架声明
│  └─ mcu/
│     ├─ at32f456/           adc、pwm、encoder_spi、can、flash、system、boot
│     └─ stm32g4/            同类实现、boot_hw、generated/Core
└─ sdk/                     原样厂商源码
```

板卡和应用配置位于 `firmware/config`，板级 main/Keil/scatter 位于 `projects`，
台架入口位于 `tests/bench`。没有公共转发源、devices 层或独立平台配置树。

| 接口/文件 | 责任 |
| --- | --- |
| bsp.h / system.c / main.c / app.c | Bsp_Init、Bsp_Start、静态应用事件、时钟和系统初始化 |
| bsp_system.h / system.c | 时基与监督中断处理、毫秒时间、短临界区、内存屏障、UID、生产身份和看门狗 |
| bsp_adc.h / adc.c | ADC 初始化、触发、中断事件处理、原码读取、零偏及单位换算 |
| bsp_pwm.h / pwm.c | PWM 初始化、占空比到比较值、直接启停和通道查询 |
| bsp_encoder.h / encoder_spi.c | 编码器初始化、SPI/DMA 传输和单圈原始角度 |
| bsp_can.h / can.c | 引脚、位时序、DLC 换算、单帧收发、硬件忙/完成和 RX IRQ |
| bsp_flash.h / flash.c | 固定存储区、关相复核、暂停采样、擦写、回读和恢复 |
| boot_hw.h / boot*.c | 已有 APP/Loader 交接；不支持的板明确报告不可用 |
| mcu_resources.h | 本 MCU 使用的引脚、外设、DMA、IRQ编号/优先级和寄存器映射 |
| mcu/<平台>/interrupt_map.h | 本平台ADC、监督和SysTick向量符号映射，无SDK类型 |
| projects/<平台>/application/main.c | ADC、监督和SysTick真实向量入口及监督事件接线 |
| stm32g4/generated | CubeMX 生成外设/MSP/IRQ；三个APP向量在USER CODE区映射为BSP处理函数 |

## 启动与执行

两板 main 调用 Bsp_Init、App_Init、Bsp_Start、App_Start，随后循环 App_Process。
BSP 只准备硬件与时钟，不持有电机或协议业务对象，也不自行恢复电机输出。

AT32 ADC 向量 → main中BspAdc_CompleteFast → MotorAxis_SampleFast → MotorAxis_ControlFast → g_app_fast_samples递增。
STM32 ADC 向量 → BspAdc_HandleInterrupt → HAL完成回调 → BspApp_OnFastSample → MotorAxis_SampleFast → MotorAxis_ControlFast。
监督向量 → main入口 → BspSystem_HandleControlInterrupt → BspApp_OnControlTick → 2 kHz APP 监督。
SysTick向量 → main入口 → BspSystem_HandleTickInterrupt → 本板1ms时基。
CAN 接收事件 → BspApp_OnCanReceive → 有界搬运入队；业务解码在前台。
快慢健康计数均由App相应任务在完成后累加，硬件事件不成立时不推进计数。
App_Process前台推进标定发布、状态刷新、通信及Loader交接。
Keil可写调试邮箱和命令分派已删除；BSP不解释应用命令。

控制器和配置负责电流有效性、故障与功率许可。BSP 的 ADC 不提供帧缓存/新鲜度状态，
PWM 不提供提交窗口检查，编码器不提供 CRC/超时，CAN 不提供软件票据和自动恢复。
这是当前接口能力边界，不能沿用旧阶段的保护保证。

## 板间差异

AT32的系统外设初始化按用户约定默认成功，McuSystem_Init/Start不返回错误，
时钟就绪等待保留；BSP公共入口不再维护启动状态，main负责按序单次调用。
独立s_watchdog_started仍管理看门狗启动与喂狗。STM32初始化策略不在本轮变更范围。

| 项目 | STM32G4 | AT32F456 |
| --- | --- | --- |
| 编码器 | TLE5012B，SPI 换向读取 | MT6835，SPI/DMA |
| 相电流/母线电压 | ADC 硬件四次累计 | ADC 硬件四次累计 |
| 独立母线电流 | 无 | ADC2 单次域 |
| 电机 NTC / 输出 sin/cos | 无 | 有采样；输出角度未接入控制 |
| MCU 温度 | 片内校准值换算 | 关闭片内温度采样 |
| 应用看门狗 | 未提供；Loader 继承路径独立 | APP 已提供 |
| Loader | 独立 APP 和 LoaderApp 选择 | 当前不可用 |

不支持通道不伪造有效值。硬件与应用配置详情见[配置说明](../../firmware/config/README.md)。

## 验证入口

`python tools/run.py check_project_layout --all` 检查所有工程文件和依赖路径。
PR 验证不操作硬件；构建和实机验证结论分别记录。
当前清理证据见[清理计划](../plans/active/2026-10-30-firmware-cleanup-v1.md)。
