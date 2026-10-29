# 双板BSP接口、能力配置与命名统一策略 v1

日期：2026-10-24。状态：评审草案；本轮只审查和制定策略，不实施固件迁移。
后续用户已授权实施第一批采样职责拆解，现行进展与结果见
[采样拆解实施](../completed/2026-10-24-sampling-responsibility-refactor-v1.md)；下方初始对照表保留为本策略起点。
后续编码器批次也已完成，见[编码器三层拆解及AT32实测](../completed/2026-10-24-encoder-responsibility-refactor-v1.md)。
后续PWM/功率批次已完成，见[PWM与功率输出职责及验证](../completed/2026-10-24-pwm-power-refactor-v1.md)：
两板硬件输出、初始化与公共许可已分开，9目标及PR/release通过；本批未烧录。
CAN的软件职责拆分和验收也已完成，见[CAN职责与关相实测](../completed/2026-10-24-can-responsibility-refactor-v1.md)；
9目标及最终PR/release通过，AT32获准烧录本批永久关相镜像，30秒只读CAN验证通过。
后续能力配置、功能使能与四类接口收口已完成，见[本批实施](../completed/2026-10-25-platform-capabilities-v1.md)，
9目标及PR/release通过；本批未烧录，不沿用此前实机结论。
同日补充：按用户指出的BSP/Port内容悬殊，调整为先统一职责和抽象层级，再迁移命名；
同名文件不能继续一边只转发、另一边承包全部生命周期与策略。

## 1. 目标、基线与范围

用户要求：电机控制使用固定的采样、PWM、编码器、电压和通信接口；
两板使用同一套命名和职责划分，通过明确的功能配置启用或禁用能力。
新增板卡应实现同一组功能端口，不再让共用软件依赖板名、芯片名或临时台架入口。

审查对象是the-system-refactor当前工作树，HEAD为
`8dd29950dd0030911cf5071aa697db50e97e29d0`，包含已有未提交及未跟踪文件。
HEAD不是当前源码快照；本轮静态接口清单及134个相关源码/构建文件哈希见
`outputs/review-bsp-strategy-20261024/inventory.json`。
清单扫描了17个API头、48个函数声明/内联定义候选；它不代替预处理、链接或实机证据。
实现前必须重新冻结当时工作树，不能直接沿用本轮清单作为后续构建证明。

本方案是BSP架构重构提议，后续确实需要调整公共API和直接调用点。
此前[纯适配范围](../../architecture/dual_board_adaptation_scope_v1.md)的“公共接口冻结”
继续约束现有纯适配工作；本草案不自动授权实施新的接口或共用行为变化。
本轮没有新增下载、复位、写参数或电机运动授权。

不变目标：FOC与控制数学、状态转换、STOP/故障优先级、参数/通信ABI、持久化含义、
20kHz快环、2kHz监督、CAN 1M/5M及现有保护限值。
接口重构引起的行为修正单独列项，不混入重命名或目录迁移。

## 2. 当前问题：哪些是混乱，哪些是必要差异

| 现状与源码落点 | 实际问题 | 处理方向 |
| --- | --- | --- |
| `api/motor_sensing.h` | 相电流、VBUS、MCU温度和换算常量混在一起；公共文字固定四次和、JEOS | 快速/辅助采样契约与温度契约分离；原始尺度显式定义 |
| `api/board_hw.h` | CubeMX、CH4、TIM7及旧板启动顺序被写成通用能力契约 | 通用契约只规定启动阶段、关相和失败语义，寄存器顺序留在本板实现 |
| AT32 `board_hw_start(true)`先启相，再调用要求关相的AdcStart/TimingStart | 公共接口存在未被当前false调用路径覆盖的分支不兼容 | 启动采样与启相分离；单独记录缺陷修正，不混称纯重命名 |
| `api/encoder_spi.h` | TLE5012B的半双工16位事务位于公共API；AT32无需且没有实现 | 移为TLE驱动的私有传输契约；公共接口统一为编码器采样 |
| `api/mcu_temperature.h` | 芯片专用出厂标定计算与通用保护常量混放 | STM32换算归私有驱动；通用温度输出不要求同一种标定方式 |
| `ports/motor/*_stm32g4.c`与AT32平铺的`*_at32f456.c` | 同一职责的层级、后缀和拆分方式不同 | 两板BSP门面同名、同层；私有实现允许按真实芯片结构不同 |
| `critical_hw_*`在STM32的`bsp/critical.c`，AT32的`ports/time_at32f456.c` | 临界区与时间职责混合，无法按能力定位 | 各自归统一的critical/time端口 |
| AT32的`r40_adc.h`、`R40_MotorTemperaturePoll`、`r40_sincos_diagnostic.h` | 新增能力绕过公共功能模型，APP装配需理解板级原码 | 纳入采样/温度/诊断的明确边界；不把原始sin/cos宣布为输出轴角度 |
| `application/bus_current_feedback.c`和`app_bus_current.h` | 独立母线电流测量、零点与滤波绕行APP扩展；无硬件时只保留估算 | 测量能力与APP来源选择分开，保持现有实测/估算及无效行为 |
| `R40_*`、`motor_hw_*`、`BootHw_*`、`AppPlatform_*`并存 | 历史板名、硬件能力、APP装配的命名层次不清 | 公共硬件统一Bsp前缀；AppPlatform保留为应用装配契约 |
| `config/boards/r40_profile.h`同时读取测试宏、保存采样输入和3A限值 | 硬件事实、功能使能和测试策略混合 | 分开supported/enabled/required/许可与限值来源 |
| STM32中断经CubeMX回调，AT32中断在platform_app中 | 原生接入不同是必要差异，但采样就绪与APP调度边界不统一 | 保留厂商入口，只转发到同名的板级事件适配函数 |
| `FAST_LOOP_STAGE_PROFILE`公共声明在两板都有，但当前实现只见于STM32中断文件 | 可选配置的可用范围未由公共能力模型约束；AT32当前显式关闭 | 诊断开关同样声明支持范围，未提供实现的目标不得开启，不能以默认关闭掩盖悬空符号 |

最新代码已经统一两板真实main模板，并把平台实现放到platform_app.c；
先前旁路Iq脉冲和大块角度捕获调试已不在当前平台入口中，不重复作为待删除项。
多数核心公共函数已经有双板实现，应沿用其有效契约和测试，避免把重构变成重写全部驱动。
TLE5012B与MT6835的线协议、DMA与PIO、STM32 HAL与AT32标准库仍然可以不同。
统一的是面向调用方的功能、命名、数据语义和生命周期，不要求寄存器实现逐字相同。

## 3. 目标分层与目录

```text
APP / Motor / Communication
    | 只依赖功能契约和明确的值类型
platform/api/bsp_*.h
    | 公共行为在语义相同时共用实现；静态链接，无运行时注册表
platform/common/               仅容纳已确认有相同语义的跨板行为
    | 与本板实现协作；薄能力可直接落到本板，不强加空转发
platform/<mcu>/bsp/bsp_*.c
    | 本板引脚/外设接线、驱动绑定、单位与状态适配
platform/<mcu>/drivers/ + 厂商生成代码/SDK

platform/config/boards/   硬件事实、采样尺度、固定资源及已确认限制
platform/config/targets/  产品或测试组合、功能使能、必需能力、测试许可
motor/profiles/           电机电气/机械参数，与MCU选择独立
```

目录收敛：

```text
platform/
  api/                         只有公共契约、项目自有类型；不含厂商帮助函数
  common/                      已确认相同的能力/质量/纯行为实现；不接触寄存器
  config/boards/               使用产品板名，保留旧板名迁移映射
  config/targets/              板卡 + 电机 + 功能/运行策略的编译期组合
  stm32g4/                     MCU家族名保持，避免无关路径改名
    application/               main.c、platform_app.c；APP生命周期与事件装配
    bsp/                       与AT32同名的功能门面及板内私有声明
    drivers/                   私有器件/传输实现，不能被APP/Motor直接包含
    cubemx/                    保留Core/Drivers/IOC/工程相对关系
  at32f456/
    application/               同样的入口和装配职责
    bsp/                       与STM32同名的功能门面及板内私有声明
    drivers/                   时钟、ADC/DMA、SPI/器件等私有实现
    foundation/                既有独立台架，移出正常APP依赖并明确退役条件
```

将当前bsp/ports两套混合用法收敛为“bsp功能门面 + drivers私有实现”。
不能把现有ports整文件机械改名后就宣布完成：先按下方职责表拆解，再迁移。
只有职责已经单一、边界正确的实现才可直接迁为门面；时间/临界区等薄能力不增加空转发。
寄存器初始化、器件协议与必要的传输部分进入drivers；确实同语义的跨板行为共用一份。
小型且纯硬件的模块可全部留在一个bsp文件，不为压平行数强制拆分。
不新增空的器件目录、不预建UART/USB框架、不强行把厂商生成目录改成对称结构。

### 3.1 固定公共模块与命名

公共函数使用仓库已有的Module_Action风格；文件用snake_case。
公共符号不带STM32、AT32、R40、TLE、MT型号；型号只出现在私有驱动、配置和诊断身份中。

| 公共头 / 两板同名实现文件 | 职责及建议符号族 | 当前来源 |
| --- | --- | --- |
| `bsp_sampling.h / bsp_sampling.c` | `BspSampling_ReadFast/ReadAux`：相电流、母线电压、可选相电压/母线电流的真实快照 | 两板motor_sensing、AT32 r40_adc及辅助采样 |
| `bsp_pwm.h / bsp_pwm.c` | `BspPwm_SetDuty/SetPhaseDuty/Enable/Disable/ReadOutputState`：比较值和功率使能分开 | motor_hw、power_stage_hw及对应ports |
| `bsp_encoder.h / bsp_encoder.c` | `BspEncoder_Begin/BeginReadback/Complete`：转子编码器分段采样与健康状态 | encoder_sensor及两板器件实现 |
| `bsp_can.h / bsp_can.c` | `BspCan_TrySend/TrySendTracked/TakeCompletion/Receive/Service`：帧及物理发送终态 | comm_hw及两板comm实现 |
| `bsp_temperature.h / bsp_temperature.c` | `BspTemperature_Read`：按MCU/电机/MOS等已定义来源报告温度、原码与质量 | motor_sensing中的MCU轮询及AT32 NTC端口 |
| `bsp_storage.h / bsp_storage.c` | `BspStorage_Read/Write`：停机前台字节记录访问 | param_store及两板Flash端口 |
| `bsp_board.h / bsp_board.c` | 身份、能力说明及板内硬件初始化支持，不接收隐式启相布尔参数 | board_hw、r40_platform及启动端口 |
| `bsp_time.h / bsp_time.c` | `BspTime_NowMs`：明确回绕规则的单调时基 | time_hw及两板time实现 |
| `bsp_critical.h / bsp_critical.c` | `BspCritical_Enter/Exit`：保持嵌套中断屏蔽语义 | STM32 critical及AT32 time中的临界区 |
| `bsp_indicator.h / bsp_indicator.c` | 统一逻辑灯状态到实际LED/RGB资源的映射 | indicator_hw及两板指示实现 |
| `bsp_boot.h / bsp_boot.c` | Loader交接/确认能力；未提供时明确不支持 | BootHw真实与unavailable实现 |

符号为拟定命名，不是已经实现的API。P0逐接口冻结完整签名、返回值、单位和上下文。
采样初始化/开始等由板级启动组合内部使用；普通Motor调用方只读已经就绪的样本。
不要求所有公共模块都提供同样的Init/Start/Process空函数。
Flash与CAN、编码器各自保留真实事务语义，不为外观一致统一成模糊的Read/Write设备接口。

### 3.2 内容对等优先：按完整职责链审查，不按文件名或行数判断

两板同名目录目前没有同样的抽象层级，以下为本次实际源码对照。
行数只是定位证据，不作为统一后应相等的指标；STM32使用的生成代码也属于其完整实现路径。

| 功能 | STM32G4实际落点 | AT32F456实际落点 | 需要统一的职责边界 |
| --- | --- | --- | --- |
| 采样 | ports/motor_sensing约56行：读JDR、温度轮询；初始化在cubemx/Core/Src/adc.c与board_startup，事件在stm32g4xx_it.c | ports/motor_sensing约622行：ADC/DMA初始化、触发、整帧缓存、IRQ、故障、存储暂停恢复、sin/cos诊断 | 硬件采集/本板错误检测、样本发布/质量、存储协作、诊断分别有明确所有者 |
| 编码器 | encoder_tle5012b约94行负责协议，encoder_spi约214行负责传输，SPI初始化另在生成代码 | encoder_mt6835约525行同时含引脚、SPI/DMA、CS、事务所有权、CRC/状态、诊断 | 公共编码器功能、型号协议、板内传输三种职责均可定位，不把全部内容留在一个公共端口 |
| 存储 | param_store约12行转发，bsp/flash约156行处理擦写/回读/忙状态 | param_store约135行直接处理Flash及采样暂停恢复 | 上层字节契约、Flash原语和本板资源协调分清，不能一边空转发另一边全事务 |
| 指示 | indicator约49行转发到LED/RGB，blink与breathing在bsp/led/rgb | indicator约144行含GPIO、闪烁、呼吸、软件PWM及诊断覆盖 | 闪烁/呼吸行为与颜色/引脚映射分开；相同行为复用，硬件输出保持私有 |
| CAN | comm_control/status约221行，初始化主要在cubemx/fdcan.c，另用HAL | comm约360行包含初始化、GPIO、位时序、过滤、票据、硬件终态及恢复 | 控制器配置/槽位事实留硬件层；帧规则/票据契约明确为同一功能，不盲目共用硬件状态机 |
| 时间/临界区 | time约6行转发HAL_GetTick，critical约18行独立实现；tick入口在生成中断代码 | time约35行同时提供SysTick、灯调度和临界区 | 时间读数、临界区原语、tick上的应用任务分别归位 |

统一后的分层必须满足以下内容约束：

| 层 | 可以拥有 | 不应拥有 |
| --- | --- | --- |
| common | 已核对等价的样本质量/可选能力处理、指示行为等公共逻辑；公共规则唯一实现 | HAL、寄存器、板引脚、厂商句柄、板名分支、另一个主状态机 |
| 本板bsp功能实现 | 板资源绑定、对私有驱动的适配、真实硬件结果到公共契约的转换、必要的原子快照 | 一份独立的业务滤波/保护策略、测试命令解释器、另一套公共票据或有效性规则 |
| drivers与厂商代码 | ADC/SPI/CAN/Flash/定时器配置和操作、器件帧协议、DMA/IRQ硬件事实及必要的即时关断 | APP命令准入、协议业务、型号运行参数、未授权的故障自动恢复 |
| application/目标策略 | APP入口接线、目标必需能力、验证许可与诊断装配 | 采样寄存器读取、器件CRC、Flash擦页与SPI事务 |

公共行为共用的具体准入规则：先比较单位、上下文、失败语义与时序，再决定提取。
指示器闪烁/呼吸有实际双板调用和相近逻辑，可先比较波形序列后共用；
采样序号/有效性可在统一快照契约后共用；原有电流换算/零偏/滤波仍沿用共享感测所有者。
CAN票据的100ms期限和64位身份有共同契约，但TXBRP/TXBTO与PTB/handle终态不同，
先用同一套契约测试验证两板，再判断可提取的纯部分，不强行共用完整控制器状态机。
芯片或数据通路专属的故障检测和即时关断必须留在本板有界路径，不能为提取公共层延迟关断。

以AT32 motor_sensing为第一份拆解样本：

1. adc_ready、AdcInit、触发配置和DMA设置归本板采集驱动；STM32对应生成配置由同层适配绑定。
2. DMA/TMR/ADC硬件事件、错误标志读取、停止触发与即时关相归本板驱动/事件适配。
3. fast/aux/temperature缓存和读取接口按统一快照职责组织；通用质量规则在契约对齐后共用。
4. PauseForStorage/ResumeAfterStorage归本板资源协调，仍严格保持停机、屏蔽中断及新鲜帧恢复条件。
5. sin/cos调试快照与发布分频归诊断实现，不继续嵌在控制必需采样模块中。

STM32沿同样的职责逐项对齐，不能保留“主要实现都在CubeMX，因此无须说明”的隐含路径。
生成代码不用复制出来；BSP明确绑定它并提供相同生命周期与事件契约。
新加STM32丢帧检测、CRC或更严故障检查属于行为增强，需独立评审；
AT32已有保护不因旧板尚无相同行为而删除或降级。

本节完成标准是每个功能都有“配置/初始化、I/O、样本/事务状态、错误/清理、调度、诊断”
六项所有权对照；适用项两板同层、同职责、同可观察契约，不适用项有明确理由。
不允许以同名文件加一层转发包装替代这一项验收。

## 4. 能力、使能、质量和运行许可必须分开

### 4.1 配置模型

1. `supported`：此板已接线且有可用驱动的能力；板级配置拥有，不由产品开关伪造。
2. `enabled`：此构建是否启用；目标配置拥有。enabled超出supported时编译失败。
3. `required`：此构建/功能的准入要求；控制所需电流、VBUS、转子编码器缺失时拒绝构建或准入。
4. `valid/fresh/status`：运行样本的质量，由端口及真实采样推进；与上面三个配置量不同。
5. `output permission`：永久关相或限时许可独立于PWM硬件能力；Off仍有PWM能力，启相始终被拒绝。

对应宏建议使用`BSP_SUPPORTED_*`、`BSP_ENABLED_*`、`BSP_REQUIRED_*`，
位图/描述表从这些唯一配置派生，不再手工维护另一份运行时能力真值。
第一版维持编译期选择，不添加在线热插拔、驱动工厂或可变函数指针表。
未知板卡必须报错，不默认退回STM32。

所有目标均可包含同一套公共头、链接同名功能门面。
未支持或禁用的可选功能通过明确实现返回状态；禁用后不配置对应GPIO/ADC/DMA/中断。
底层器件文件可根据目标裁剪，但不能使公共声明/调用方式随板卡消失。
共用控制代码不写`#if AT32`、`#if R40`或按板名选择实现。

### 4.2 必须区分的运行状态

| 状态 | 含义及处理 |
| --- | --- |
| `UNSUPPORTED` | 本板没有已实现的对应能力，不启动外设，不产生测量有效位 |
| `DISABLED` | 本板支持，但本目标关闭；不初始化资源，不静默代替其他传感器 |
| `NOT_READY` | 已启用，尚未取得首个合格样本或完成校零/标定 |
| `STALE` | 有历史值但超过有效时间，不能再次刷新成新测量 |
| `IO_ERROR`或具体传感器错误 | 已发生通信、DMA、CRC等异常；按既有故障语义处理 |
| `OK` | 样本及其质量条件合格；不等于电机可运行或命令已完成 |

CAN的BUSY、输入错误、发送完成/失败、PWM许可拒绝保持各自明确返回语义，
不把所有情况挤进一个bool或一个全设备通用状态机。
结构体有固定字段及质量标记，无效物理值用NaN或明确无效位；有效性只看契约，不能靠数值猜测。
诊断允许保留最后原码，但必须同时标出时间/序号及无效原因；不能用零值冒充真实电压、电流或温度。
状态OK与“取得新样本”分别表达；重复但仍新鲜的样本可以读取，但不能再次累计校零、滤波或刷新有效期。

### 4.3 当前两板能力矩阵

| 功能 | STM32G4 / Vector Mini ST | AT32F456 / YG-ESC_V1.0 | 统一策略 |
| --- | --- | --- | --- |
| 三相电流 | 已有快采样 | 已有ADC1抢占/DMA相关保护 | 控制目标必需；同一快帧和尺度契约 |
| 三相PWM与关相 | TIM1 | TMR1 | 同一API；启停不隐式改占空比，许可独立 |
| 转子编码器 | TLE5012B，半双工 | MT6835，全双工/DMA | 同一分段读取契约，器件差异私有 |
| 母线电压 | 已有 | 已有PA3 | 控制目标必需；保持相同物理单位与故障含义 |
| 独立母线电流 | 无现行独立采样，APP用估算 | PA4实测，需上电校零 | 同一可选采样通道；实测与估算标明来源 |
| 相电压 | 无现行端口 | 无现行端口 | 保留采样通道契约，supported=false，不假造驱动 |
| MCU温度 | 已有采样及芯片标定 | Off启用采样；Commissioning关闭，实板校准未完成 | 分开硬件能力、目标开关与校准有效性 |
| 电机NTC | 预留资源不等于已有可用端口 | PA7私有诊断已实现，未接完整APP保护 | 可选温度来源；不借统一命名新增保护行为 |
| 输出轴反馈 | SPI资源预留，不等于输出角度能力 | PA5/PA6原始诊断，不等于输出角度能力 | 原码诊断与已标定角度分开；闭环后置 |
| CAN FD | 已有1M/5M | 已有1M/5M | 同一帧、票据、完成和恢复契约 |
| 参数存储 | 旧板保留区 | 新板保留区 | 字节契约相同，地址/擦除粒度私有 |
| Loader交接 | 取决于普通APP/Loader配套目标 | 尚未实现 | supported由板与镜像组合确定；禁止假报成功 |

已有STM32估算母线电流属于共享算法结果，不属于BSP实测能力。
统一后APP明确选择measured/estimated来源；AT32测量失效时继续保持当前“不回退估算”行为。
MCU结温、电机绕组/MOS温度互不替代，增加统一温度接口不自动改变保护来源或阈值。
未接线、没有可用驱动或未定义标定的功能，不能仅改enabled就声称可用。

## 5. 核心契约必须统一到什么程度

### 5.1 采样：一拍一份证据，原码尺度明确

快环取一份`BspFastSample`，包含三相电流、VBUS、可选相电压的有效位与同拍序号。
辅助采样单独的`BspAuxSample`带采样序号、采样时间与通道有效位；慢通道不挤进20kHz必需路径。
同一快拍重复读取不会重新触发ADC或混合两拍，序号以完成的真实采样推进。
本板驱动先确认整帧、锁存，再发出FastSampleReady事件；错误帧不派发普通控制计算。
通道间不是同时转换时，记录其触发/顺序及偏移，快帧不宣称硬件同时采样。

第一阶段保留实际ADC原码和既有四次累加行为。固定配置说明每通道满量程、累加次数、
参考电压、增益、分流/分压与符号，不把“四次和”继续写成永久跨MCU限制。
不乘四伪造采样，不先平均/截断损失精度；缺少量程描述的启用通道拒绝编译。
已有共享感测模块继续拥有上电相电流零偏、A/V换算、滤波与保护，
FOC数学仍接收A/V，不让寄存器、ADC位数或通道号进入控制算法。
迁移时必须保持原表达式顺序和原码偏置口径；不在同一批改变schema或零偏存储单位。
是否进一步把纯换算抽成独立公共模块，以实际调用与验证需要决定，不预建空抽象。

### 5.2 PWM与功率许可

三相比较值更新、逐相诊断、功率Enable/Disable和实际使能状态分开。
明确占空比范围、NaN前置条件、预装载生效点、跨更新事件的三相一致性、ISR资格与嵌套临界区。
软件临界区不等于硬件原子更新，不能把禁止中断当作三相同步提交的证明。
Enable返回已使能/许可拒绝/硬件失败等可判断结果；Disable保持快速、有界、可重复调用。
实际关闭必须读取本板输出使能状态，不以“调用过Disable”替代证据。
Off与Commissioning的限制由目标选择的许可实现承担，不向Motor散落测试板宏。

### 5.3 编码器

保留Begin/BeginReadback/Complete分段和20kHz重叠计算，不强行退化成每拍同步阻塞Read。
公共角度第一阶段维持每圈65536计数的现行表示，明确跨圈、方向、CRC与状态有效性。
虽然历史名为q15，实际单圈范围为0..65535；本批不擅自改成32768计数或修改线协议。
物理帧、位宽、SPI模式、CS、DMA与超时收尾归私有驱动；失败不发布旧角度为新样本。
TLE当前CRC覆盖限制如实保留，不因新接口质量字段而宣称实现了未有的校验。
输出轴原始sin/cos与转子编码器不共用可混淆的angle字段；双反馈融合不在本批实现。

### 5.4 CAN、存储与调度

CAN统一帧格式和非阻塞入口，保留64位事务票据及真实物理发送终态；
不把提交成功、队列空或收到任意帧当作发送成功，不改变bus-off重试/恢复所有者。
Flash地址、擦除页与采样暂停恢复归本板；参数CRC、schema、保存事务和配置提交归参数/APP。
写失败仍可能部分生效，接口统一不新增断电原子保证；不隐式保存或改变看门狗时限。

各板保留真实main和AppPlatform_Init/Start/Process，严格保持已确认的APP调用顺序。
板内初始化顺序可以不同，但统一阶段为：安全GPIO/关相、时钟与外设准备、
APP和端点绑定、采样/监督/通信服务按既有顺序开放；任何阶段都不隐式启相。
实现前逐项标明App_Init内部启动采样的位置，不能在AppPlatform_Start再次启动同一时基。
公共启动能力不再提供“启动采样同时选择启相”的模糊bool分支。

板内事件适配统一命名为`BspIrq_OnFastSampleReady`、`BspIrq_OnSupervisorTick`、
`BspIrq_OnCanRxReady`等，声明放各板同名私有头；厂商向量名与HAL回调名保持。
事件适配完成本板有界校验，再由application调用App四入口或CAN入队入口。
BSP/drivers不直接包含APP运行对象或解析协议，应用装配不读取ADC寄存器完成换算。
CubeMX只在USER CODE回调点转发，生成区不手工重写。

## 6. 现有文件迁移清单

| 现有输入 | 目标归属与删除条件 |
| --- | --- |
| `api/motor_hw.h`、`api/power_stage_hw.h`，两板motor_pwm/power_stage实现 | 合为bsp_pwm契约/门面；旧声明与适配别名在全部调用点迁移后删除 |
| 两板motor_sensing、AT32 r40_adc | bsp_sampling门面；ADC/DMA配置按实际复杂度留私有drivers |
| STM32 motor_sensing中的温度部分、AT32 temperature/motor_temperature | bsp_temperature门面；STM32校准与NTC换算私有化 |
| `api/encoder_sensor.h`及两板encoder器件实现 | bsp_encoder门面 + 私有tle5012b/mt6835；同一工程只选择一套转子实现 |
| `api/encoder_spi.h`及STM32 encoder_spi | 从公共API移除，归TLE驱动私有；不为AT32补无意义的半双工占位 |
| `api/comm_hw.h`及STM32 comm_control/comm_status、AT32 comm | bsp_can门面；是否保留私有分文件取决于真实职责，不影响公共命名 |
| 两板board启动、r40_platform、r40_clock | bsp_board与私有clock/引脚驱动；APP阶段接线仍归platform_app |
| 两板time、STM32 critical、AT32 time中的critical | bsp_time/bsp_critical分开，保留回绕及嵌套语义 |
| param_store、两板Flash实现、sampling_storage私有协作 | bsp_storage门面；暂停采样与恢复协议仍是本板内部契约 |
| indicator、BootHw及unavailable实现 | bsp_indicator/bsp_boot门面；固定公共符号，具体能力由目标选择 |
| AT32 bus_current_feedback、APP来源扩展 | 原码取得走公共采样；校零/换算/滤波按原所有者显式迁移；APP只选择来源和发布 |
| app_telemetry、r40_sincos诊断及fast_loop_profile | 诊断边界；非控制必需API，不借命名统一开放输出闭环或新遥测字段 |
| boards中的r40_analog/r40_profile、targets中的r40_app_* | 产品板名与模式名统一；去除历史R40公共命名，旧名只留历史映射 |
| 两板Keil、scatter、tests和project_paths/harness | 每批同步路径、同名对象定位、源清单、SRAM执行段与验证目标 |

同名bsp对象只能来自所选板卡；工程源清单测试必须拒绝两板实现同时链接。
scatter中的对象名、函数节名随迁移更新，不能只修改Keil显示名。
禁止长期维持“旧API+新API”两套真值；兼容适配只允许在迁移批次内，明确移除条件。
诊断对象如需重命名，列出Watch/J-Scope/脚本消费者并同步迁移；不修改历史证据中的原符号。

## 7. 实施批次、证据与回退

| 阶段 | 改动 | 本阶段完成标准 |
| --- | --- | --- |
| P0 职责、契约与基线 | 按3.2节逐功能核对完整实现链，冻结六项所有权及接口单位/时序/错误，再记录能力矩阵与当前四目标基线 | 两板相同功能具有同层职责；禁止一边全流程、一边只转发而下层去向不明 |
| P1 内容归位与机械迁移 | 先拆解采样、编码器、存储/指示的混合职责，再逐模块迁移命名与工程；纯代码搬迁与公共逻辑提取分批 | 两板相同模块负责相同事项；硬件差异有明确落点；不引入新行为，加载/状态/输出差异逐项解释 |
| P2 能力配置 | 分开supported/enabled/required/许可；可选端口始终有契约，禁用不初始化资源 | 非法组合编译拒绝；未支持/禁用/未就绪/故障分别验证，无假成功 |
| P3 数据与启动契约 | 同拍快照、辅助质量、温度来源、初始化/事件接线统一；显式迁移调用点 | 数值、偏置、滤波、故障、事务、状态语义保持；变化单独评审，不称机械迁移 |
| P4 双板验收与清理 | 移除临时旧接口/名称，更新唯一现行文档；四目标构建、离线及获授权的实机回归 | 两板独立证据；未实测不宣布通过；新板未定版能力仍不标为产品支持 |

P1不能一次同时改目录、接口单位、初始化顺序和保护算法。
每个模块先通过职责对照，再做机械迁移；公共逻辑提取单独比较双板基线和真实调用。
P3先把readiness/sequence/scale契约冻结，再迁移使用者；不绕过现有STOP和配置所有者。
任何编译失败、时序超预算、旧板回归或输出差异不明，停止推进后续批次。

### 7.1 必须覆盖的验收

- 完整APP四目标：STM32普通、STM32 Loader配套、AT32永久关相、AT32限时。
  基础台架按本批受影响目标回归，不混入正式发布默认集合。
- 接口链接：公共头可独立包含；每目标公共符号恰好一份；不存在板名条件调用或头文件遮蔽。
- 内容对等：六项所有权逐项核对，端口无混入驱动初始化/调试策略；共同行为使用同一套契约测试，已提取部分有真实双板调用。
- 配置：supported/enabled/required非法组合、未知板型、禁用通道资源零初始化、必需采样缺失拒绝。
- 采样：同拍一致、重复/丢帧、序号回绕、过期、ADC/DMA错误、尺度与单位、零偏不伪造。
- 编码器：现行角度域、方向/跨圈、错误映射、CRC覆盖真实性、超时/CS/DMA收尾及分段预算。
- PWM：通道映射、预装载、启动拒绝、STOP抢占、真实关相、Off不可解锁、限时不能续期。
- CAN：全部现有帧约束、忙/超时、64位票据、迟到完成、槽复用、bus-off及协议队列所有权。
- 存储/APP：读写边界、部分失败、暂停恢复、新鲜度重建、状态/故障轨迹、命令进展、ABI黄金向量。
- 资源：Flash/RAM、静态栈、ER_FAST和UNINIT地址、最坏路径快环与中断嵌套；20kHz/2kHz保持。
- 文档：唯一现行接口说明和板卡能力表，历史日志保留；当前实现与未来能力明确区分。

文档批次跑quick；固件/构建/测试批次跑PR；工程、链接和发布输入批次跑release，
同时显式构建受影响的四目标，不把默认release构建范围当作全部目标证据。
每批在outputs下保存实际工作树哈希、工具链、配置、日志、summary、镜像和资源。
机械重命名若改变调试信息或链接布局，比较实际加载字节、反汇编/状态/数值及资源，
说明每项差异；不能只凭能编译或一份HEX哈希认定等价。
实机验证需具体镜像和后续明确授权；离线测试不证明引脚、栅极、噪声与真实时序。

### 7.2 回退与交付

实现前按批次保存当前未提交文件与新增文件清单，不reset/clean、不恢复他人历史状态。
每批迁移连同调用点、Keil/scatter、测试与文档成组回退；保持旧板独立可构建。
若独立行为修正不可与机械重命名分离，拆出独立变更先评审，不能偷偷借新BSP修控制行为。

最终交付要求：新增第三块板时，只需提供相同bsp门面、私有驱动和板/目标配置，
共用APP/Motor/Communication不增加板名判断；功能关闭有明确状态，控制必需能力缺失不能运行。
这项标准通过目标矩阵、链接与故障注入验证，不由目录对称或函数前缀相同单独证明。

## 8. 本轮审查与验证记录

已完成两板公共声明、实际实现、平台装配、配置入口及主要调用方的静态对照；
本轮只新增本策略和活动计划索引，不修改固件、工程、接口或既有适配范围。
文档链接检查PASS（0问题），本轮两个文档的空白检查通过。
quick检查PASS，包含doctor、format、lint、project-layout及Python单元/集成测试；
证据为`outputs/runs/bsp-strategy-review-20261024-venv/summary.json`，hardware_contacted=false。
首次用系统Python运行因缺少ruff及clang-format未通过doctor，原失败记录保留在
`outputs/runs/bsp-strategy-review-20261024/`；随后使用现有虚拟环境和已安装固定版本格式器重跑，
没有修改依赖、代码或门禁规则。
本轮未执行固件迁移、新一轮PR/release或实机验证，不把quick通过当作新架构已实现的证明。

同日职责补充复核：新增3.2节六项所有权对照与采样拆解，调整P0/P1为先内容归位，
补充公共逻辑实际复用和同层职责验收；文档检查与quick再次PASS。
证据为`outputs/runs/bsp-responsibility-review-20261024/summary.json`，hardware_contacted=false；
此次补充仍只修改策略及索引，不修改固件实现。


## 2026-10-26 精简收口审查

按用户要求梳理当前platform，目标是减少重复归类与应用/调试侵入，不继续叠加抽象层。
以当前工作树实际文件和9份Keil工程建立只读清单，排除CubeMX生成/厂商树后有57个C、62个头，
其中foundation独立台架为7个C、5个头。全部C至少在一个工程登记，不将此当作预处理或链接存活证明，
本轮不据此直接删除模块。清单及哈希：outputs/platform-architecture-review-20261026/inventory.json。

现行采样/PWM/CAN已直接实现统一接口，旧motor包装不再存在；剩余收敛点是ports/bsp交叉、
单文件common目录、application中的测量/遥测、API中的配置/芯片换算及双板型选择入口。
唯一精简目标与删并清单维护在[公共接口与实现对应表](../../architecture/platform_interface_implementations_v1.md)
的“精简目标”节，避免再创建平行架构文档。本节目标优先约束后续目录整理，不追改已完成批次的证据。

本轮只更新现有架构/策略/索引，不修改固件、工程或测试；不得将建议树称为已完成。
后续第一批只收拢目录，保持公共函数/类型、20kHz/2kHz、CAN 1M/5M、输出许可、故障优先级和存储语义。
验收要求是同一模块一个所有者、没有新增纯转发/动态注册层，双板工程和保护回归可对照。
物理迁移的回退按整批源清单/Keil/scatter/测试成组恢复，不清理用户其他工作。
本轮文档审查执行quick及文档链接检查；不产生新的固件或实机通过结论。

审查完成：当前结构图、实际接口落点和精简目标已在现有文档更新，没有新增架构文件。
quick的doctor/格式/lint/工程路径/Python测试通过，文档链接检查0问题。
证据outputs/platform-architecture-review-20261026/summary.json和
outputs/runs/platform-architecture-review-20261026/summary.json。
源码哈希复核见清单，本次没有固件编辑、编译/烧录或实机操作。


## 2026-10-26 精简实施批次

用户已批准执行上节删并清单。本批将ports归入两板bsp，common编码器入口归入现有drivers/encoder，
配置头归config、STM32温度换算归本板温度实现；RTT只读遥测移动到communication/telemetry。
不新增生产文件或转发层；main模板、BSP函数/类型、控制频率、CAN时序、参数/协议ABI、功率许可及故障门限保持。
母线电流测量移至drivers/measurement；原APP扩展的Init/Sample改名并用Read返回物理量/有效性，
0x40协议位归现有yg_protocol_status_adapter，旧板估算/新板实测失效语义保持。
这三项是非BSP测量装配接口的接线调整，不是更改冻结BSP契约；共享控制算法和状态机不改。

基线是执行前1195文件的冻结工作树，清单/哈希及改动映射存于outputs/platform-slim-20261026；
独立基线构建使用固定Loader打包令牌，仅用于镜像比较，不修改产品默认值。
验收为9个工程0错误/警告、PR/release离线回归、冻结BSP声明一致、比较加载字节和资源。
新位置与Keil、测试、文档作为一批回退；以baseline.json/moves.json对应的冻结源恢复本批文件，
不reset/clean或覆盖用户其他变更。实机为NOT_RUN，本轮不烧录。


本批构建与镜像对照：四个完整APP和五个台架工程全部0错误、0警告。
旧板普通/Loader配套APP及五个台架的实际HEX加载字节与执行前冻结基线一致；
新板Off/限时APP因母线测量/协议装配分工调整各减少16字节代码，RO/RW/ZI不变。
新板整体HEX因函数重排及重定位不同；对642/646个i.*反汇编节进行地址、字面量池、
分支宽度及历史测量符号的规范化后，仅BusCurrentFeedback_Read和YgProtocolStatusAdapter_Refresh有指令变化。
该比较只是辅助，不把规范化当作形式化等价或实机快环耗时证明；原始反汇编与差异均保留。
13个仍归API的契约头逐字节与基线一致；原有BSP公共函数、类型和含义没有新增或修改。

生产文件总数净减1（旧板critical并入time），没有新增生产文件；AT32 application只剩main/platform_app两个C。
参数schema、协议帧、20kHz/2kHz、CAN 1M/5M及3A测试保护保持；输出轴仍无新增闭环逻辑。
测量重点回归涵盖旧板估算、新板零偏/符号/滤波/缺帧/过期/ADC失效及CAN编码；
RTT回归涵盖2kHz整帧、只读输入、队列溢出、主机阻塞和恢复，未取消任何保护。
初次PR留下头迁移导致的测试编译失败记录；补齐显式契约包含后已做针对性复测。
最终完整验证以独立冻结源outputs/slim-final-1026中的
outputs/runs/platform-slim-release-1026/summary.json为准；证据汇总复制至
outputs/platform-slim-20261026/summary.json，关联源码提交与产物哈希。
构建对照见同目录baseline-fixed/builds.json、final/builds.json、image-comparison.json、
api-comparison.json及disassembly-comparison.json。hardware_contacted=false，HIL=NOT_RUN。
