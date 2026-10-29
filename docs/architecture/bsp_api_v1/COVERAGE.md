# 通道与功能覆盖复核（修订6）

日期：2026-10-26。范围：修订6接口及两板已接入实现；离线通过，统一实测未完成。
复核来源：当前两板代码、用户指出的相电流遗漏，以及任务
`codex://threads/01a17761-6789-7232-8fd9-b8992b0993ad` 中的独立审核建议。
该任务评审的是修订前版本；本表逐项对照当前修订，不把历史结论直接当成现存代码缺陷。

## 1. 完整ADC通道对应

ReadChannel返回物理量；ReadRawChannel返回支撑该逻辑量的原码。
三相电流、独立母线电流与母线电压共用PWM同步快帧；控制用ReadFast，电流原码用ReadRawCurrent。
逐通道连续调用不保证同拍，不能替代上述批量接口。

| 通道 | 物理量和参考点 | 原码来源 | 快帧字段 | 功能开关 | 当前两板情况 |
| --- | --- | --- | --- | --- | --- |
| `BSP_ADC_PHASE_CURRENT_A` | A相电流，A | A相ADC等效12位计数 | phase_current_a[0] | PHASE_CURRENT | 两板均有 |
| `BSP_ADC_PHASE_CURRENT_B` | B相电流，A | B相ADC等效12位计数 | phase_current_a[1] | PHASE_CURRENT | 两板均有 |
| `BSP_ADC_PHASE_CURRENT_C` | C相电流，A | C相ADC等效12位计数 | phase_current_a[2] | PHASE_CURRENT | 两板均有 |
| `BSP_ADC_BUS_VOLTAGE` | 母线正极相对负极，V | 母线分压ADC计数 | bus_voltage_v | BUS_VOLTAGE | 两板均有 |
| `BSP_ADC_BUS_CURRENT` | 独立传感器母线电流，A | 快原码帧bus_current_counts | bus_current_a | BUS_CURRENT | AT32已迁入ADC2的PWM同步注入采样；STM32未提供 |
| `BSP_ADC_MOTOR_TEMPERATURE` | 电机温度，摄氏度 | NTC ADC计数 | — | MOTOR_TEMPERATURE | 仅AT32有对应NTC |
| `BSP_ADC_MCU_TEMPERATURE` | MCU温度，摄氏度 | TS及同帧VREF计数 | — | MCU_TEMPERATURE | STM32沿用出厂校准；AT32缺实板校准时只提供原码 |
| `BSP_ADC_ANALOG_SUPPLY_VOLTAGE` | MCU模拟供电VDDA，V | 内部VREF计数，不是母线分压原码 | — | ANALOG_SUPPLY_VOLTAGE | STM32现有温度诊断已输出；AT32缺参考校准时不伪造物理值 |
| `BSP_ADC_HARDWARE_VERSION` | 版本采样脚相对ADC地，V | 电阻编码ADC计数 | — | HARDWARE_VERSION | 当前两板未绑定 |

BSP_ADC_CHANNEL_COUNT仅是边界哨兵，不能读取；负值和越界值亦返回INVALID_ARGUMENT。
枚举不是协议编号或ADC硬件rank；不得按其整数值访问任意ADC通道或作为持久ABI。

相电流正负沿用当前控制算法约定，板配置明确ADC极性；迁移不能顺手改变既有符号。
母线电流方向同样沿用现有测量与遥测约定，独立原码不能混用相电流分流电阻或增益。
本设计不包含相电压采样，不保留相关通道或占位能力。
VDDA由内部参考及其有效校准计算，缺校准返回NOT_CALIBRATED，但原始VREF仍可读。

HARDWARE_VERSION和ANALOG_SUPPLY_VOLTAGE的SUPPORTED/ENABLED/REQUIRED已按已有
配置规则接线；未绑定功能保持禁用。温度与VDDA可共享内部ADC序列资源，
资源需求取逻辑或，不因一个功能关闭就误停另一个；两路逻辑使能和有效性仍分别判断。

## 2. 同类遗漏复核

| 模块 | 已核对的实际用途 | 结论与边界 |
| --- | --- | --- |
| 生命周期 | 安全输出、BootHw早期交接、Init/Start、三个事件 | 已覆盖；轮询CAN不强制RX事件，失败不继续Start |
| 系统 | 时间、频率、临界区、UID、生产身份、复位、应用/继承看门狗 | 已覆盖；继承看门狗仍在BootHw，避免重复启动 |
| ADC | 三相、母线、NTC、内部温度/VREF/VDDA、版本采样 | 三相/母线电流共用快组；保留VDDA；统一原码/物理量映射 |
| PWM | 三相/单相占空比、许可、立即关相、全关判断、状态与关相序号 | 同事件生效、截止拒绝及BSP失败收尾已定义；实际时序仍需验证 |
| 编码器 | Begin/Readback/Complete、计数、型号、独立采样诊断及配置字 | 控制样本与器件字分开；有效位、CRC是否执行、配置六槽映射已明确 |
| LED | 模式/故障闪烁与颜色/灯带刷新 | 通道枚举、共享优先级、重复设置及5Hz/20Hz推进已定义 |
| CAN | 启动、单次提交、票据提交、接收、完成、状态、bus-off恢复 | 补溢出/坏帧结果、幸存帧保留和有界继续消费；不改变协议选择 |
| UART | 字节发送、完成、接收错误、运行状态 | 补发送期限、迟到事件隔离及失败锁存；当前无硬件，不生成空驱动 |
| Flash | 固定区读取、RAW区写入/回读、源重叠拒绝、失败锁存 | 已覆盖；完整硬件事务的中断状态归BSP |
| BootHw | 槽识别、进入Loader、继承看门狗、试运行确认与查询 | 原接口加已获准的IsAvailable只读查询，避免二次初始化 |
| 专用诊断 | FastLoopProfile及当前原码/电压诊断 | FastLoopProfile原合同单独保留，VDDA映射已补，不能以十个头文件数量删掉既有用途 |

不为将来可能使用的任意引脚、定时器通道或SPI设备预建原始外设公开API。
模块确有新的应用消费者时再扩展，不能把“完整接口”理解为公开所有寄存器操作。

## 3. 独立审核六项建议对照

| 独立审核建议 | 当前处理 | 验证约束 |
| --- | --- | --- |
| P1 母线电流没有标定原码 | 快原码帧bus_current_counts或ReadRawChannel(BUS_CURRENT)与Setter配对 | 保留128个连续稳定样本和轨限检查 |
| P1 零偏提交仅前台，与快ISR冲突 | Setter允许关相ISR原子提交；应用满窗收尾移至监督任务 | 成功才置complete；BSP同缓存可读，应用下一快拍使用，见EXECUTION第2节 |
| P1 Flash屏蔽责任不明 | Write拥有完整屏蔽、暂停、恢复与失败锁存；不保证墙钟上限 | 两板移植时对照所有提前退出及中断状态 |
| P1 窗口迁移后关断不消费许可 | GetStatus关相序号加唯一功率所有者；检查和Enable同一短临界区 | 正常停止、故障、部分启相、到期、重放均须覆盖 |
| P2 未标定电流屏蔽母线电压 | ReadFast分组有效位及BUS_VOLTAGE独立缓存读取 | 上电未标定可读电压，不能使用无效电流 |
| P2 CAN排除前台轮询 | Receive允许配置选择单一ISR或POLL消费者 | 不改变原台架调度，禁止双消费者 |

这些处理已接入双板实现并有离线回归；不能据此声称物理硬件验收完成。
本轮还补了上述独立审核未列出的两项完整性问题：三相电流漏列、VDDA物理诊断缺少映射。

## 4. 必检场景

- 枚举中的9个实际通道均有原码、物理单位、能力映射；COUNT不是有效通道。
- 三相单通道与批量读取使用同一缓存，序号与实际采样一致，读取不启动ADC。
- 未标定时原码可读、物理电流NOT_CALIBRATED，快帧母线电压仍有效。
- 缺参考校准时MCU温度/VDDA物理量不可伪造；原码和同帧VREF仍能用于诊断。
- 硬件版本和独立母线电流未提供时返回NOT_SUPPORTED；快帧对应字段为NAN且不置有效位。
- 整个接口包仍需C/C++独立及组合包含、Doxygen、函数签名与所有枚举覆盖检查。

## 修订4：母线电流快采样约束

目标快物理帧包含phase_current_a[3]、bus_current_a、bus_voltage_v及各组有效位。
目标快原码帧包含phase_counts[3]、bus_current_counts及两组原码有效位；某组无效不得使用。
物理组需要自己的零偏资格，原码组不需要；两种valid_mask表达不同层次，不能混用。

| 平台 | 当前实现 | 结果与边界 |
| --- | --- | --- |
| AT32F456 | 相电流/母线电压在PWM触发快序列，母线电流PA4在ADC2的PWM同步注入序列 | 母线电流按同一PWM周期收齐后发布，不复制旧辅助样本；实际精度及时限待实测 |
| STM32G4 | 无独立母线电流传感器 | BUS_CURRENT不支持；快物理/原码字段均NAN，有效位为0，不用估算值替代 |

正式实现还要验证ADC触发能力、序列/rank限制、采样建立时间、跨ADC同步及20kHz预算。
本轮已按此采集合同迁移硬件与消费者，离线验证覆盖时序配置和错误收尾；实际模拟精度及时限须由本轮AT32实测证据确认。

## 修订5：五类问题的关闭条件

| 问题 | 当前设计落点 | 不能以设计检查代替的验证 |
| --- | --- | --- |
| PWM三相跨更新事件、部分启相后依赖应用关断 | SetDuty/SetPhaseDuty明示截止；Enable失败先统一关断；EXECUTION第8节 | 两板有效更新边界、写入余量、正常延迟和异常物理关断 |
| 编码器控制样本混入器件诊断 | 新ReadDiagnostics返回型号/事务结果/有效位；第9节列配置映射 | 原错误行为、SPI字节数、遥测值及配对序号 |
| LED覆盖和重复设置含糊 | SetCode通道枚举，应用选码；第10节明确共享/独立资源及节拍 | 两板闪烁/呼吸波形、DMA资源和故障解除显示 |
| CAN丢帧被当空队列或不继续消费 | Receive先一次报告OVERFLOW，保留幸存帧；第11节有界消费 | 连续溢出、坏帧后好帧、IO活锁及原ISR预算 |
| UART传输无终止期限 | TX_TIMEOUT终态、TakeTxResult推进期限及TX静止确认；第12节 | 真实UART接线后故障注入、迟到IRQ和RX不受损 |

PWM、编码器、LED和CAN已实现并通过原生回归；UART按本契约保持未绑定，收发返回NOT_SUPPORTED。
接口编译和文档检查不能证明物理寄存器时序或安全关断，仍须实测。ADC九通道及快母线电流要求不变。
