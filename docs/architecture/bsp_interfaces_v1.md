# BSP功能接口与实现v1（修订6）

当前范围：保留PWM、ADC、Encoder（SPI）、CAN与Flash及必要系统支撑。LED、UART已删除，
不保留接口、占位实现或工程引用。见[删除记录](../plans/active/2026-10-30-remove-led-uart-v1.md)。

状态：修订6接口已接入两板固件，双板构建与离线发布检查通过；统一实机验收尚未完成。日期：2026-10-26。
电流零偏现由INIT阶段的独立校正函数统一处理三相与母线，详见[校正调整](../plans/active/2026-10-30-current-offset-init-v2.md)。
完整文件树和所有权见[BSP架构总图](bsp_architecture_v1.md)。
本页是职责与函数索引，参数、单位、失败和上下文的权威说明位于[公开头文件](../../firmware/platform/bsp/include)。
`bsp_api_v1`保留同步的接口说明副本；修改接口直接实现并同步文档和测试，不再等待逐项评审。

## 分层及所有权

```text
firmware/platform/
├─ bsp/
│  ├─ include/                  八个统一功能契约
│  ├─ bsp_internal.h           统一私有声明，不含实现或状态定义
│  └─ mcu/{at32f456,stm32g4}/   系统、IRQ及ADC/PWM/SPI/CAN/GPIO/Flash真实操作
├─ config/                     唯一板选择、资源、比例、硬件能力与组合校验
└─ sdk/                        厂商原目录，不统一厂商目录格式
```

应用入口在`projects/<board>/application/main.c`，流程统一为
`Bsp_Init → App_Init → Bsp_Start → App_Start → App_Process`。
两板main.c集中定义ADC、监督及SysTick向量，通过工程包含路径引用各MCU目录的interrupt_map.h。
AT32 ADC向量通过BspAdc_CompleteFast确认并清标志后，在main直接执行轴快环和健康计数；
STM32 ADC仍通过BspAdc_HandleInterrupt派发HAL回调，由BspApp_OnFastSample执行相同任务；
监督事件仍经main中的BspApp_OnControlTick执行应用慢环。
硬件标志、HAL派发和时基维护仍归MCU，main不访问寄存器。
BSP初始化时钟及外设、发布采样和中断事件；应用初始化对象、处理协议、控制和健康判定。
BSP不调用具体协议处理器，不持有电机对象；Axis只提供初始化和快慢环，禁止跨模块extern Watch对象。
Flash写入的暂停采样、擦写、回读和中断恢复由BSP拥有；应用外层临界区只保护保存任务身份不被取消后继续使用。

## 模块接口

当前公开面为8个头，包含3个应用实现的事件回调及ADC、监督、SysTick的硬件处理入口。
BootHw_Init和单相PWM仅在根目录内部头声明；已删除闲置微秒时基、复位入口和阶段诊断。
本轮未编译或运行测试，见[公共接口最简化](../plans/active/2026-10-30-bsp-public-minimal-v1.md)。

全部外设实现放在对应MCU目录，include保留统一接口声明。
核对记录见[全BSP硬件边界核对](../plans/active/2026-10-30-bsp-hardware-boundary-audit-v1.md)。

### 生命周期和应用事件

详细契约：[bsp.h](../../firmware/platform/bsp/include/bsp.h)。

```c
BspResult Bsp_Init(void);
BspResult Bsp_Start(void);
void BspApp_OnFastSample(void);
void BspApp_OnControlTick(void);
void BspApp_OnCanReceive(void);
```

### 系统原语

详细契约：[bsp_system.h](../../firmware/platform/bsp/include/bsp_system.h)。

```c
uint32_t BspSystem_TimeMs(void);
void BspSystem_Reset(void); /* 停机配置的显式复位：关相、禁止启动、回复发送完成后调用。 */
uint32_t BspSystem_EnterCritical(void);
void BspSystem_ExitCritical(uint32_t saved_state);
uint32_t BspSystem_EnterControlCritical(void);
void BspSystem_ExitControlCritical(uint32_t saved_state);
void BspSystem_MemoryBarrier(void);
bool BspSystem_CompareExchange32(volatile uint32_t *address, uint32_t expected, uint32_t desired);
BspResult BspSystem_ReadUid(uint8_t uid[BSP_SYSTEM_UID_BYTES]);
BspResult BspSystem_ReadIdentity(uint8_t record[BSP_SYSTEM_IDENTITY_BYTES]);
```

### 模拟信号

两板共用[ADC接口](../../firmware/platform/bsp/include/bsp_adc.h)：

```c
float BspAdc_ReadRawChannel(BspAdcChannel channel);
float BspAdc_ReadChannel(BspAdcChannel channel);
void BspAdc_SetPhaseOffsets(const float offsets_counts[3]);
void BspAdc_SetBusCurrentOffset(float offset_counts);
```

直接读取原码或物理量；无结果码、帧类型、有效位、序号或缓存。
缺参数、未提供的信号或数学无定义返回NAN。硬件原码、触发、DMA和中断由各MCU实现；
换算及零偏也在各MCU实现，公开接口不因板型改变。

### 三相功率输出

两板共用[PWM接口](../../firmware/platform/bsp/include/bsp_pwm.h)：

```c
void BspPwm_SetDuty(float phase_a, float phase_b, float phase_c);
void BspPwm_Enable(void);
void BspPwm_Disable(void);
bool BspPwm_ArePhasesDisabled(void);
```

直接写比较值/输出位，或读取六路通道是否全关；不校验、不返回状态或事件计数。
占空比换算、周期读取、比较值写入、启停和通道查询均由各MCU实现。
应用与电机代码不根据芯片选择调用形式，不新增状态或转发包装。
此统一接口于2026-10-30按用户要求实施，测试与编译尚未运行。

### 编码器

两板使用同一[公开头](../../firmware/platform/bsp/include/bsp_encoder.h)：

```c
void BspEncoder_Begin(void);
uint32_t BspEncoder_ReadAngle(BspEncoderResolution resolution);
```

Begin发起本拍读取，ReadAngle等待传输完成；BSP_ENCODER_16BIT为每圈65536计数，
BSP_ENCODER_21BIT为每圈2097152计数，均用uint32_t返回。现有控制环固定选择16位。
MT6835返回高16位或完整21位；TLE5012B原15位左移补零；不施加方向、零位或LUT。
工程只编译本板MCU的encoder_spi.c，由其直接实现统一接口；不再保留事务/诊断/配置缓存。
硬件读写和启停归MCU，BSP没有额外Encoder转发实现。
两板均无CRC、状态和超时检查。位宽选择的离线验证和工程构建见
[验证记录](../plans/completed/2026-11-03-encoder-output-bits-v1.md)。

### CAN帧

两板共用[bsp_can.h](../../firmware/platform/bsp/include/bsp_can.h)，板型只在工程和BSP内部选择。

```c
bool BspCan_ConfigureBitrate(uint8_t profile);
void BspCan_Start(void);
bool BspCan_TrySend(const BspCanFrame *frame);
bool BspCan_Receive(BspCanFrame *frame);
bool BspCan_IsTransmitBusy(void);
bool BspCan_IsTransmitComplete(void);
```

length统一为字节数，ID为11位标准ID或29位扩展ID；调用方保证ID、格式和DLC对应长度合法。
两板均只允许一帧在途，TrySend的true表示硬件已接受请求，IsTransmitComplete才表示发送成功。
Receive以false表示FIFO空，数据帧写入length个字节，远程帧只写头字段。
不提供BspResult、软件状态、票据、超时/取消或自动恢复；协议握手仍在通信层。
DLC转换直接位于各MCU收发函数；统一入口在mcu/*/can.c实现，包含启停、收发和IRQ。
本轮按用户要求未测试、未编译，不能沿用页面顶部的历史验证结果。

### 参数存储区

详细契约：[bsp_flash.h](../../firmware/platform/bsp/include/bsp_flash.h)。

```c
BspResult BspFlash_Read(void *data, size_t length);
BspResult BspFlash_Write(const void *data, size_t length);
```

### Loader交接

详细契约：[boot_hw.h](../../firmware/platform/bsp/include/boot_hw.h)。

```c
bool BootHw_EnterLoader(uint8_t node, uint32_t started_ms, uint32_t timeout_ms);
void BootHw_Watchdog(bool healthy);
bool BootHw_Confirm(uint32_t version, uint32_t crc32);
bool BootHw_IsConfirmed(void);
uint32_t BootHw_ImageCrc32(void);
bool BootHw_IsAvailable(void);
```

## 两板资源差异

| 功能 | AT32F456 | STM32G4 |
| --- | --- | --- |
| 三相电流/母线电压 | TMR1 CH1 同步触发。ADC1 三相电流各四次累加，ADC2 在抢占序列末位单采 PA3 母线电压 | ADC2注入四rank、各四次累加，原TIM1 CH4触发 |
| 独立母线电流 | ADC2 PA4注入序列，与ADC1同次PWM事件 | 未布置，返回NOT_SUPPORTED |
| 辅助模拟量 | PA7 NTC、可选sin/cos、内部温度/VREF | 内部温度/VREF；其它未布置 |
| 编码器 | MT6835、SPI2四线，APP沿用DMA | TLE5012B、SPI2三线换向 |
| PWM | TMR1，保留原六路使能、死区、映射 | TIM1，直接读写比较值和六路/主输出位 |
| CAN | CAN1单PTB，单帧在途 | FDCAN1硬件FIFO，单帧在途 |
| Loader 试运行保护 | 配套 LoaderApp 维护继承的硬件看门狗 | 配套 LoaderApp 维护继承的硬件看门狗 |

功能用`BSP_SUPPORTED_*`描述实板能力，用`BSP_ENABLED_*`选择本目标功能，用应用的
`BSP_REQUIRED_*`阻止缺少保护所需功能。禁用功能保留同一公开函数，明确返回NOT_SUPPORTED；
不把未安装传感器、缺少校准或旧慢缓存伪装成有效物理量。
STM32生成初始化须与板配置一致，换接线要同步生成配置；不能只改宏而假称任意资源组合都可运行。

## 控制行为和增强边界

三相采样四次累加、算术顺序、母线电压保护、编码器65536计数和协议/参数ABI保持。
编码器不再提供原始诊断字、错误枚举或坏帧计数。
AT32母线电流仍为PWM同步采样；关相 IDLE 零偏校正与三相共用2048个快拍，正常快拍读取物理量并滤波，ADC不提供快序号。
两板都不提供ADC快帧或有效位；未设置零偏的物理电流为NAN，原码直接用于校零。
AT32内部温度/VREF没有实测校准时只发布原码，不把典型值诊断当作保护资格。

两板PWM均按用户要求删除原提交超时、启停读回和故障状态包装。
CAN不报告溢出或维护软件终态；发送完成查询直接读取硬件。
Flash双方沿用各自存储处理；两板编码器均不提供配置缓存和读取错误状态。
CAN按调用方提供的合法经典/FD元数据读写硬件；上层协议仍只使用原扩展FD+BRS及原过滤器。

一次性通电窗口及相关功率契约已删除；电流零点由`MotorCurrentOffsets_Calibrate`统一处理，
三相与母线电流由 `MotorCurrent_Update` 处理；母线电压由 [MotorVoltage_Update](../../firmware/motor/sensing/motor_voltage.c) 读取和滤波。
没有驱动注册表、运行时板检测、虚表或为对称而制造的MCU空文件。

## 验证入口

[执行约定](bsp_api_v1/EXECUTION.md)、[覆盖矩阵](bsp_api_v1/COVERAGE.md)与
[实施计划](../plans/active/2026-10-26-bsp-peripheral-ownership-v1.md)共同记录验收要求。
旧评审记录仅解释变更来源，不作为当前仍存在缺陷的断言。
本轮输出位于`outputs/bsp-v1-implementation-20261026/`；最终实测报告尚待完成，不复用旧实测作为新实现的证据。
