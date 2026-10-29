# BSP功能接口与实现v1（修订6）

状态：修订6接口已接入两板固件，双板构建与离线发布检查通过；统一实机验收尚未完成。日期：2026-10-26。
相电流零偏的应用收尾已移至2kHz监督任务，属于明确记录的调度调整，详见执行约定第2节。
完整文件树和所有权见[BSP架构总图](bsp_architecture_v1.md)。
本页是职责与函数索引，参数、单位、失败和上下文的权威说明位于[公开头文件](../../firmware/platform/bsp/include)。
`bsp_api_v1`保留同步的接口说明副本；修改接口直接实现并同步文档和测试，不再等待逐项评审。

## 分层及所有权

```text
firmware/platform/
├─ bsp/
│  ├─ include/                  十个统一功能契约及既有FastLoopProfile诊断
│  ├─ bsp_adc.c                 快帧、原码/物理量、独立有效位和零偏提交
│  ├─ bsp_pwm.c                 占空比校验、提交保护、输出门控与关相序号
│  ├─ bsp_encoder.c             非阻塞事务、控制样本、器件诊断和配置读取
│  ├─ bsp_can.c                 软件票据、终态、溢出报告与恢复节流
│  ├─ bsp_led.c                 统一闪烁与呼吸节拍
│  ├─ bsp_uart.c                当前两板未绑定，统一返回NOT_SUPPORTED
│  ├─ can_frame_codec.c/.h      物理DLC转换，不解释应用协议
│  ├─ bsp_internal.h           统一私有声明，不含实现或状态定义
│  ├─ devices/                 MT6835/TLE5012B命令、校验及角度解码
│  └─ mcu/{at32f456,stm32g4}/   系统、IRQ及ADC/PWM/SPI/CAN/GPIO/Flash真实操作
├─ config/                     唯一板选择、资源、比例、硬件能力与组合校验
└─ sdk/                        厂商原目录，不统一厂商目录格式
```

应用入口在`projects/<board>/application/main.c`，流程统一为
`Bsp_Init → App_Init → Bsp_Start → App_Start → App_Process`。
BSP初始化时钟及外设、发布采样和中断事件；应用初始化对象、处理协议、控制和健康判定。
BSP不调用具体协议处理器，不持有电机对象；Axis通过自身接口发布诊断，禁止跨模块extern Watch对象。
Flash写入的暂停采样、擦写、回读和中断恢复由BSP拥有；应用外层临界区只保护保存任务身份不被取消后继续使用。

## 模块接口

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
uint32_t BspSystem_TimeUs(void);
uint32_t BspSystem_CoreClockHz(void);
uint32_t BspSystem_EnterCritical(void);
void BspSystem_ExitCritical(uint32_t saved_state);
BspResult BspSystem_ReadUid(uint8_t uid[BSP_SYSTEM_UID_BYTES]);
BspResult BspSystem_ReadIdentity(uint8_t record[BSP_SYSTEM_IDENTITY_BYTES]);
void BspSystem_Reset(void);
BspResult BspWatchdog_Start(void);
void BspWatchdog_Feed(void);
```

### 模拟信号

详细契约：[bsp_adc.h](../../firmware/platform/bsp/include/bsp_adc.h)。

```c
BspResult BspAdc_ReadFast(BspAdcFastSample *sample);
BspResult BspAdc_ReadRawCurrent(BspAdcRawCurrent *sample);
BspResult BspAdc_ReadRawChannel(BspAdcChannel channel, BspAdcRawValue *sample);
BspResult BspAdc_SetPhaseOffsets(const float offsets_counts[3]);
BspResult BspAdc_SetBusCurrentOffset(float offset_counts);
BspResult BspAdc_ReadChannel(BspAdcChannel channel, BspAdcValue *sample);
BspResult BspAdc_UpdateAuxiliary(void);
```

### 三相功率输出

详细契约：[bsp_pwm.h](../../firmware/platform/bsp/include/bsp_pwm.h)。

```c
BspResult BspPwm_SetDuty(float phase_a, float phase_b, float phase_c);
BspResult BspPwm_SetPhaseDuty(BspPwmPhase phase, float duty);
BspResult BspPwm_Enable(void);
void BspPwm_Disable(void);
bool BspPwm_ArePhasesDisabled(void);
BspPwmStatus BspPwm_GetStatus(void);
```

### 编码器

详细契约：[bsp_encoder.h](../../firmware/platform/bsp/include/bsp_encoder.h)。

```c
BspResult BspEncoder_Begin(void);
BspResult BspEncoder_BeginReadback(void);
BspResult BspEncoder_Complete(BspEncoderSample *sample);
BspEncoderType BspEncoder_GetType(void);
BspResult BspEncoder_ReadDiagnostics(BspEncoderDiagnostics *diagnostics);
BspResult BspEncoder_CaptureConfig(void);
BspResult BspEncoder_ReadConfigWord(uint8_t index, uint16_t *word);
```

### 指示器

详细契约：[bsp_led.h](../../firmware/platform/bsp/include/bsp_led.h)。

```c
BspResult BspLed_SetCode(BspLedCodeChannel channel, uint8_t blink_count);
BspResult BspLed_SetColor(BspLedColor color);
```

### CAN帧

详细契约：[bsp_can.h](../../firmware/platform/bsp/include/bsp_can.h)。

```c
BspResult BspCan_Start(void);
BspResult BspCan_TrySend(const BspCanFrame *frame);
BspResult BspCan_TrySendTracked(const BspCanFrame *frame, uint64_t ticket);
BspResult BspCan_Receive(BspCanFrame *frame);
BspResult BspCan_TakeCompletion(BspCanCompletion *completion);
BspCanState BspCan_GetState(void);
BspResult BspCan_Recover(void);
```

### UART字节流

详细契约：[bsp_uart.h](../../firmware/platform/bsp/include/bsp_uart.h)。

```c
BspResult BspUart_TryWrite(const uint8_t *data, size_t length);
BspResult BspUart_TakeTxResult(BspUartTxResult *result);
BspResult BspUart_Read(uint8_t *data, size_t capacity, size_t *received);
BspUartState BspUart_GetState(void);
```

### 参数存储区

详细契约：[bsp_flash.h](../../firmware/platform/bsp/include/bsp_flash.h)。

```c
BspResult BspFlash_Read(void *data, size_t length);
BspResult BspFlash_Write(const void *data, size_t length);
```

### Loader交接

详细契约：[boot_hw.h](../../firmware/platform/bsp/include/boot_hw.h)。

```c
bool BootHw_Init(void);
bool BootHw_EnterLoader(uint8_t node);
void BootHw_Watchdog(bool healthy);
bool BootHw_Confirm(uint32_t version, uint32_t crc32);
bool BootHw_IsConfirmed(void);
uint32_t BootHw_ImageCrc32(void);
bool BootHw_IsAvailable(void);
```

## 两板资源差异

| 功能 | AT32F456 | STM32G4 |
| --- | --- | --- |
| 三相电流/母线电压 | ADC1注入四rank、各四次累加，PWM同步 | ADC2注入四rank、各四次累加，原TIM1 CH4触发 |
| 独立母线电流 | ADC2 PA4注入序列，与ADC1同次PWM事件 | 未布置，返回NOT_SUPPORTED |
| 辅助模拟量 | PA7 NTC、可选sin/cos、内部温度/VREF | 内部温度/VREF；其它未布置 |
| 编码器 | MT6835、SPI2四线，APP沿用DMA | TLE5012B、SPI2三线换向 |
| PWM | TMR1，保留原六路使能、死区、映射 | TIM1，保留原HAL通道状态及六路顺序 |
| CAN | CAN1单PTB，普通帧也须等待槽释放 | FDCAN1硬件FIFO；跟踪票据独占软件槽 |
| LED | PC13/PC14共用闪烁和颜色 | 状态GPIO和TIM2 CH3灯带分别显示 |
| UART/硬件版本ADC | 未绑定 | 未绑定 |
| 看门狗 | 原AT32应用看门狗 | 独立APP无应用看门狗，Loader继承路径保持 |

功能用`BSP_SUPPORTED_*`描述实板能力，用`BSP_ENABLED_*`选择本目标功能，用应用的
`BSP_REQUIRED_*`阻止缺少保护所需功能。禁用功能保留同一公开函数，明确返回NOT_SUPPORTED；
不把未安装传感器、缺少校准或旧慢缓存伪装成有效物理量。
STM32生成初始化须与板配置一致，换接线要同步生成配置；不能只改宏而假称任意资源组合都可运行。

## 控制行为和增强边界

三相采样四次累加、算术顺序、母线电压保护、编码器65536计数和原诊断字、协议/参数ABI保持。
AT32母线电流改为真正PWM同步快采样；零点和滤波仍在2kHz监督拍处理128个样本，按快序号跨10拍判断连续性。
新快帧内电压、相电流和母线电流有效位相互独立。未校准电流是NAN，原码仍可用于上电标定。
AT32内部温度/VREF没有实测校准时只发布原码，不把典型值诊断当作保护资格。

以下是明确的异常路径增强，不能宣称与旧错误行为逐位等价：PWM错过提交窗口先关相并锁存、
部分启用失败由BSP收尾并读回；关断不完整进入致命路径；CAN报告溢出并保留幸存帧，已确认终态不被后续超时覆盖；
Flash双方均自行保护中断和采样、失败锁存；编码器失败配置槽明确无效，不复用旧值充当新读数。
CAN BSP识别合法经典/FD元数据；上层协议仍只使用原扩展FD+BRS及原过滤器，不扩大业务协议范围。

一次性功率窗口在`firmware/app/runtime/pwm_power_window.*`，母线电流零点/滤波在
`firmware/app/runtime/bus_current_feedback.*`。它们是应用状态与策略，不回放到BSP中。
没有驱动注册表、运行时板检测、虚表或为对称而制造的MCU空文件。

## 验证入口

[执行约定](bsp_api_v1/EXECUTION.md)、[覆盖矩阵](bsp_api_v1/COVERAGE.md)与
[实施计划](../plans/active/2026-10-26-bsp-peripheral-ownership-v1.md)共同记录验收要求。
旧评审记录仅解释变更来源，不作为当前仍存在缺陷的断言。
本轮输出位于`outputs/bsp-v1-implementation-20261026/`；最终实测报告尚待完成，不复用旧实测作为新实现的证据。
