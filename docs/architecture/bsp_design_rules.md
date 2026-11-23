# BSP 职责约定

核对日期：2026-11-17。这里统一维护职责、依赖、接口语义与板型差异。
正式声明见[生产公开头](../../firmware/platform/bsp/include)；本页不维护头文件副本。

## 公开能力与实现

BSP 统一管理实际使用的时钟、看门狗、ADC、GPIO、PWM、SPI、DMA、CAN、定时器和 Flash。
应用使用功能接口，硬件差异集中在板卡资源、引脚、通道、比例、器件类型和对应 MCU 实现。

- `bsp/include` 只有八个公共头：bsp、bsp_system、bsp_adc、bsp_pwm、bsp_encoder、bsp_can、bsp_flash、boot_hw。
- `bsp_internal.h` 只保存跨文件初始化、采样启停和台架专用声明，没有函数实现、可变状态定义或板卡值。
- 各 MCU 的 adc/pwm/encoder_spi/can/flash/system 实现直接定义统一接口，不增加转发层或驱动注册表。
- ADC 零偏和物理换算、PWM 占空比换算、CAN DLC 换算随硬件实现保留在对应 MCU 文件。
- 单文件辅助函数为 static；只有真实多消费者共享完整契约时才提取共用逻辑。
- AT32 APP 在 BSP 初始化时将 PC13、PC14 置高熄灭，通过 BspSystem_WriteStatusLights 同步输出两灯。
  [灯语](../hardware/at32f456_configuration_v1.md)由应用灯语模块在 2 kHz 监督拍根据轴状态与硬件检查决定。
- 公共 LED 框架、UART、独立器件包装、功率窗口和旧诊断入口已删除。

## 当前接口语义

| 功能 | 契约 |
| --- | --- |
| ADC | float原码/物理量读取，不提供通用帧状态、时间戳或同拍保证；另有快完成确认与单读者NTC新采样消费接口 |
| 零偏 | void 直接设置，调用者负责有效范围、关相及原子发布 |
| PWM | void 直接设置占空比和启停；不验证许可、不限幅、不锁存故障 |
| PWM 查询 | 直接读取六路通道位，不把主输出位作为通道关相证据 |
| 编码器 | void Begin 与 uint32_t ReadAngle；显式选择16/21位单圈计数，控制环用16位；无方向/LUT/零位修正 |
| 编码器等待 | 控制角度读取不验证CRC、状态或超时；AT32另有已完成帧读取供前台诊断，不能混作控制准入 |
| CAN | void Start；bool 收发与忙/完成查询；不提供软件票据、恢复或取消 |
| Flash | 固定记录区，前台关相擦写、回读、失败后禁止不确定重试 |

PWM 三个比较寄存器分别写入，不承诺跨更新事件原子提交。
两板编码器和 CAN 的 GPIO、SPI/DMA、寄存器操作及 IRQ 都归对应 MCU。
协议队列、升级握手、状态准入、故障策略和标定资格由其业务所有者维护。

## 配置与启动

`firmware/config/config.h` 提供板型编号、工程输入要求、电机默认值和公共快慢时基。
`board_config.h` 提供能力、电路、时钟和存储范围；功能直接由实际板卡能力决定。
每个 MCU 的 `mcu_resources.h` 统一管理引脚、外设、采样结果、DMA、IRQ编号和优先级，
仅供该 MCU 使用。STM32 改动资源必须同步审查 `.ioc` 与生成区，手改限 USER CODE。
各MCU的 `firmware/platform/bsp/mcu/<平台>/interrupt_map.h` 只映射ADC、监督与SysTick的向量符号，
各板main.c通过工程选定的MCU包含目录引用它，并定义三个真实中断入口。
映射头不包含SDK类型或寄存器，应用引用权限仅限该头；资源变更时须同步核对向量映射。

main 使用 `Bsp_Init → App_Init → Bsp_Start → App_Start`，BSP 初始化不启相。
AT32外设初始化按默认成功处理：McuSystem_Init/Start为void，仅等待时钟就绪并按序配置，
不维护启动状态或上报初始化异常。Bsp_Init/Start也不缓存启动阶段，调用者负责按顺序各调用一次。
Flash_Init不参与AT32初始化失败分支；Flash运行期读写、看门狗和电机故障仍保留原检查。
AT32 main的ADC入口通过BspAdc_CompleteFast确认并清标志，直接执行MotorAxis_SampleFast → MotorAxis_ControlFast，
快拍进展由生命周期登记。STM32 ADC入口仍调用BspAdc_HandleInterrupt，保留HAL回调后清标志。
监督与SysTick入口调用BspSystem_HandleControlInterrupt和BspSystem_HandleTickInterrupt；
CAN事件由APP组合层实现，监督回调仍由main连接APP。
MCU保留完成确认、清标志、HAL派发及1ms时基；main不读取寄存器、句柄或引入SDK类型。
STM32生成文件只在USER CODE区把原三个函数映射为BSP处理函数，保留原函数体和再生成入口。
快采样可抢占低频监督，原有中断优先级和短临界区语义不因整理改变。

当前通道、采样窗口和应用消费频率统一见[采样链路](sampling_chain_v1.md)，
PWM边沿和比较值装载见[驱动时序](at32f456_pwm_adc_driver.md)。
变更时必须同时核对板级配置、实际寄存器设置与控制预算，不能仅根据同名接口推断硬件行为。

## 验证

离线检查覆盖统一声明、调用者、工程输入、数值范围和错误语义；实际波形、控制时序和电机响应另行验收。
不得把目录一致、头文件存在或单板构建成功当作双板替代证据。
用户已取消逐项接口评审要求，后续可在授权范围内直接完善实现、文档和验证。
接口变更同步实现、调用者、配置和验证；不以同名函数推定两板硬件能力一致。

## 两相电阻测量输出

BspPwm_SelectPhases仅在已许可的相电阻会话、占空比均为中点且电流消退后调用；逻辑位0/1/2为A/B/C，选择3、6、5。未选相上下桥均关闭，保留触发通道、极性和死区。Enable仍恢复全部三相，Disable关闭全部三相。应用不能用此接口绕过状态机启相。

## 公共接口索引

### 生命周期和应用事件

[bsp.h](../../firmware/platform/bsp/include/bsp.h) 声明 `Bsp_Init`、`Bsp_Start`，返回 `BspResult`，
以及 `BspApp_OnFastSample`、`BspApp_OnControlTick`、`BspApp_OnCanReceive` 三个应用回调。
两板实现的失败和等待行为不因同名接口而相同；AT32外设初始化按当前约定默认成功，保留硬件就绪等待。
初始化不等于启相。中断开放前须完成应用对象装配。

### 系统原语

[bsp_system.h](../../firmware/platform/bsp/include/bsp_system.h) 提供毫秒时基、监督/SysTick处理、
UID、生产身份、复位、临界区、内存屏障和32位比较交换。
AT32另提供状态灯、PendSV外环请求和CPU周期计数；这些条件接口不能套用到所有板型。

`EnterControlCritical` 仅隔离低频监督，允许ADC和采样DMA抢占，不能保护快环拥有的数据。
`EnterCritical` 保存PRIMASK并须配对恢复。普通运行不得用全局屏蔽包住通信、等待或整个业务流程。
关相后的Flash完整事务由存储实现保存并恢复中断状态。复位前由调用方确认关相、禁止启动及回复发送完成。

### 模拟信号

[bsp_adc.h](../../firmware/platform/bsp/include/bsp_adc.h) 的两类通用读取为
`BspAdc_ReadRawChannel`、`BspAdc_ReadChannel`，返回 `float`。
三相和母线电压的累加结果归一化到12位计数；母线电流、Hall和NTC按各自实际采样域处理。
物理量单位为A、V、摄氏度；未支持、缺少偏置或换算无定义时返回NaN。Hall只提供原码，物理量入口返回NaN。
读取不保证多个通道同拍，也不自行验证样本年龄或控制资格。

| 专用接口 | 使用者与语义 |
| --- | --- |
| `BspAdc_CompleteFast` | AT32 ADC中断确认主/辅完成并清标志；辅ADC未完成则跳过本拍，不等待。 |
| `BspAdc_HandleInterrupt` | STM32共享ADC中断入口，保持HAL派发和完成回调顺序。 |
| `BspAdc_ReadMotorTemperature` | 监督单读者消费新完成的NTC采样；true表示有新样本，坏值可为NaN；false保留输出。 |
| `BspAdc_SetPhaseOffsets` / `SetBusCurrentOffset` | 直接保存偏置；调用者负责范围、关相及发布条件。 |

当前校零由关相 IDLE 快拍累计2048次，成功后发布偏置。它不再是旧INIT主状态的独立流程。
NTC硬件触发频率和应用消费频率不同；完整通道、时序、0.4 V偏置与新鲜度见[采样链路](sampling_chain_v1.md)。

### 三相功率输出

[bsp_pwm.h](../../firmware/platform/bsp/include/bsp_pwm.h) 提供 `SetDuty`、`Enable`、`Disable`、
`ArePhasesDisabled` 和 `SelectPhases`。设置及启停返回void，三相比较值独立写入，无提交窗口或故障状态包装。
功率资格由状态机维护，BSP不代替准入检查。
选相接口仅用于获准的相电阻检查，在中点占空比且电流消退后选择双相，不是普通运行的任意输出入口。
时序和关相边界见[PWM链路](at32f456_pwm_adc_driver.md)。

### 编码器

[bsp_encoder.h](../../firmware/platform/bsp/include/bsp_encoder.h) 提供成对的
`BspEncoder_Begin`、`BspEncoder_ReadAngle(BspEncoderResolution)`，后者返回 `uint32_t`。
16位表示65536 counts/圈，21位表示2097152 counts/圈；当前控制算法选择16位。
角度读取等待硬件传输完成，但不验证CRC/状态，不设置超时，不应用方向、零位或LUT。
AT32的 `BspEncoder_ReadCompletedFrame` 取得同一已完成SPI帧，用于前台诊断；
这不等于控制读取增加了传感器诊断。调用时机与位布局见[编码器链路](encoder_chain_v1.md)。

### CAN帧

[bsp_can.h](../../firmware/platform/bsp/include/bsp_can.h) 提供位速率配置、Start、TrySend、Receive及忙/完成查询。
Start返回void，收发与查询返回bool；长度使用字节，调用方保证ID、帧格式和DLC对应关系合法。
TrySend成功表示硬件接受，不代表物理发送完成；升级交接另等完成查询。
协议CRC、重发、超时、队列与业务票据归通信层。BSP不提供软件取消、恢复或错误细分状态。
两板硬件行为见[CAN链路](bsp_design_rules.md#can帧)，线路契约见[协议](../protocols/README.md)。

### 参数存储区

[bsp_flash.h](../../firmware/platform/bsp/include/bsp_flash.h) 提供固定记录区读写，返回 `BspResult`。
写入前确认关相，在前台处理擦写、回读和中断恢复；它不允许业务在运行中直接写Flash。
记录版本、CRC、参数与输出轴扩展由[参数管理](parameter_ownership_v1.md)维护。

### Loader交接

[boot_hw.h](../../firmware/platform/bsp/include/boot_hw.h) 提供可用性、确认、镜像CRC、继承看门狗和交接入口。
`BootHw_EnterLoader(node, started_ms, timeout_ms)` 在写邮箱前复核期限；成功复位不返回。
业务票据发送完成、停止条件和禁止重新启相由AppLoader先建立，不能用只读可用性查询替代这些检查。
两板独立Loader及配套APP入口见[Loader设计](loader/loader_product_v3.md)。

## 两板资源差异

| 功能 | AT32F456 | STM32G431 |
| --- | --- | --- |
| 快ADC | ADC1三相电流和末路PA3母线电压，均四倍累加；ADC2单次采母线电流及Hall | ADC2三相电流和母线电压，均四倍累加 |
| 辅助ADC | ADC2普通组采NTC；片内温度不启用 | ADC1采片内温度与VREF |
| 编码器 | MT6835，SPI2/DMA | TLE5012B，SPI2三线换向 |
| PWM | TMR1 CH2/3/4，CH1触发ADC | TIM1 CH1/2/3，CH4触发ADC |
| CAN | CAN1 PTB/RX FIFO | FDCAN1消息RAM/FIFO |
| Loader | 独立APP、Loader和配套LoaderApp | 独立APP、Loader和配套LoaderApp |

具体使能条件见[板级能力](bsp_design_rules.md#文件与运行所有权)；运行有效性仍由真实样本及所属业务检查。

## 文件与运行所有权

| 位置 | 职责 |
| --- | --- |
| `bsp/include` | 八个公开功能头，不暴露厂商类型与寄存器 |
| `bsp/bsp_internal.h` | 跨文件初始化、采样启停和必要台架声明 |
| `bsp/mcu/<mcu>/system.c` | 时钟、时基、同步原语与硬件初始化编排 |
| `bsp/mcu/<mcu>/adc.c` | ADC触发、完成确认、原码、偏置与物理换算 |
| `bsp/mcu/<mcu>/pwm.c` | 占空比换算、硬件启停和六路输出查询 |
| `bsp/mcu/<mcu>/encoder_spi.c` | SPI/DMA、片选与单圈角度 |
| `bsp/mcu/<mcu>/can.c` | DLC、消息RAM/FIFO与硬件收发 |
| `bsp/mcu/<mcu>/flash.c`、`boot*.c` | 存储事务与Loader交接 |
| `projects/<board>/application/main.c` | 板级对象与向量接线，调用公共接口 |

`BSP_SUPPORTED_*`描述实际硬件与驱动，`BSP_ENABLED_*`选择构建功能。
不可用数值返回NaN，其他返回值遵循所属头文件；不能统一假设为NOT_SUPPORTED。
AT32具备独立母线电流、NTC与Hall输入，STM32无对应外部传感器；
STM32提供片内温度/VREF，AT32当前不启用片内温度。能力不等于样本有效、标定完成或功率许可。

CAN的TrySend成功只表示接受，IsTransmitComplete才表示最近一帧物理发送完成。
升级握手在通信层等待同一事务回复完成；BSP不实现业务票据、重发、取消和自动恢复。
输出轴数据与标定状态属于MotorAxis，不属于硬件驱动。
