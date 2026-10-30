# 当前未结计划

核对日期：2026-10-26。此目录包含当前事项及尚未归档的阶段记录；旧阶段状态不代表当前代码。
版本与工作树仍可能变化，执行前重新冻结源码；本索引不新增硬件操作授权。

| 计划 | 当前状态 |
| --- | --- |
| [BSP公共接口最简化](2026-10-30-bsp-public-minimal-v1.md) | 公开头10→8，闲置时基/复位/诊断删除，内部及台架声明收回；未编译或测试 |
| [ADC/PWM全部归MCU](2026-10-30-adc-pwm-mcu-local-v1.md) | 零偏及换算合入两板MCU，删除独立公共实现及辅助接口；未编译或测试 |
| [CAN长度换算就地实现](2026-10-30-can-local-dlc-v1.md) | 已删除独立换算文件和函数；转换直接放入两板收发实现，未编译或测试 |
| [BSP第一轮精简](2026-10-30-bsp-simplify-round1-v1.md) | 清理重复头、反向配置依赖及闲置初始化，统一启动归属；未测试或编译 |
| [删除LED和UART](2026-10-30-remove-led-uart-v1.md) | 删除两功能、调用及资源配置；保留四外设、Flash及必要系统支撑；未编译或测试 |
| [全BSP硬件边界核对](2026-10-30-bsp-hardware-boundary-audit-v1.md) | 共用层只保留逻辑，LED硬件初始化归MCU；接口和工程引用静态核对，未测试或编译 |
| [PWM硬件与逻辑分离v2](2026-10-30-pwm-hardware-boundary-v2.md) | 已迁移供审阅；BSP只换算占空比，MCU处理全部PWM硬件；未测试或编译 |
| [ADC硬件与逻辑分离v2](2026-10-30-adc-hardware-boundary-v2.md) | 已迁移供审阅；MCU处理ADC硬件，BSP只保留逻辑；未测试或编译 |
| [双板ADC/PWM统一最简接口](2026-10-30-dual-adc-pwm-minimal-v1.md) | 两板共用值接口和运行源，工程宏选板，上层无ADC/PWM板型分支；未测试或编译 |
| [Encoder硬件归MCU v3](2026-10-30-encoder-hardware-boundary-v3.md) | MCU直接实现统一接口及全部硬件操作，删除公共转发文件；未测试或编译 |
| [两板编码器统一最简接口v2](2026-10-30-encoder-unified-minimal-v2.md) | 统一接口阶段记录；当前硬件边界见v3 |
| [CAN硬件与逻辑分离v2](2026-10-30-can-hardware-boundary-v2.md) | BSP仅长度换算；MCU直接实现统一收发、启停和IRQ；未测试或编译 |
| [两板CAN统一最简接口](2026-10-30-can-unified-interface-v1.md) | 统一接口阶段记录；当前分工见CAN硬件与逻辑分离v2 |
| [AT32 CAN最简实现](2026-10-30-at32-can-minimal-v1.md) | 先行及codec合并阶段记录，当前实现见两板统一计划 |
| [AT32编码器最简实现](2026-10-30-at32-encoder-minimal-v1.md) | AT32先行阶段记录；当前接口与硬件边界见v3 |
| [AT32 PWM最简实现](2026-10-30-at32-pwm-minimal-v1.md) | 实现待审阅；MCU仅初始化，BSP直接写寄存器，删除保护/状态/结果码；按用户要求未测试或编译 |
| [AT32 ADC最简实现](2026-10-30-at32-adc-minimal-v1.md) | 实现待审阅；MCU仅初始化/寄存器与DMA暴露，删除ADC保护；按用户要求未运行测试或编译 |
| [采样快环删除软件保护](2026-10-30-remove-sensing-protection-v1.md) | 实施中：采样只搬运物理值并保留母线低通；离线验证，未烧录/未台架 |
| [FOC 测量边界方案](2026-10-29-foc-measurement-boundary-v1.md) | 测量对象分离及两板离线验证完成；目标时序未测 |
| [motor 归属与命名复核整改](2026-10-29-motor-followup-cleanup-v1.md) | 实施中；保持控制与保护行为，目标时序待验证 |
| [motor 分层与命名整理](2026-10-29-motor-layering-v1.md) | 结构与命名整理及离线验证完成；目标时序和实机行为待验证 |
| [BSP CAN状态精简](2026-10-29-bsp-can-simplification-v1.md) | 已完成：终态处理集中、冗余状态去除；新板普通APP编译与两模式离线对照通过，Code减少24字节，未烧录 |
| [新板sin/cos 24.5周期](2026-10-29-sincos-24p5-v1.md) | PA5/6采样24.5、NTC640.5，保留4次硬件平均；普通APP单工程与29个专项场景通过，未烧录 |
| [AT32三相ADC第二步优化](2026-10-29-adc-phase-12p5-v1.md) | 三相12.5/母线电压24.5；Off接入NTC/sin/cos四次硬件平均，冻结副本单工程与专项回归通过；未烧录，输出轴角度未接入 |
| [AT32 ADC 60MHz第一步优化](2026-10-29-adc-60mhz-v1.md) | 60MHz/24.5周期已实施，保留12位/4次过采样；九工程与PR全通过，旧板HEX不变，理论9.87µs待实测，未烧录 |
| [BSP整理后AT32标定与控制链路验收](2026-10-27-bsp-motor-control-hil-v1.md) | 台架范围已确认，准备当前候选和分阶段实测 |
| [BSP逻辑与文件架构收敛](2026-10-26-bsp-architecture-consolidation-v1.md) | 离线整理、九工程、PR及干净快照release通过；统一AT32实测后置，本阶段未操作硬件 |
| [BSP外设初始化与功能接口归位](2026-10-26-bsp-peripheral-ownership-v1.md) | 后续已实施；原评审阶段记录保留，当前状态见上项 |
| [双板ports目录对齐](2026-10-24-ports-directory-alignment-v1.md) | 五类目录及接口实现表完成；九目标重建/HEX/资源一致，109单元/14集成/36原生通过，正式release受未提交门禁阻止；未烧录 |
| [双板BSP接口、能力与命名统一](2026-10-24-bsp-unification-strategy-v1.md) | 10/26精简审查完成：ports归BSP、common归共享实现，测量/调试归位；仅文档与quick通过，目标目录尚未迁移 |
| [双板真实main与平台装配分离](2026-10-24-board-application-entry-v1.md) | AT32台架遗留清理300→264行；RTT配置合并，三目标HEX不变，冻结副本release通过；测量/遥测职责迁移未实施，未烧录 |
| [统一编译后新板关相实测](2026-10-24-armcc-off-hil-v1.md) | 统一main的Off完整APP已烧录并关相验收通过；ADC/MT6835/CAN及2kHz连续采集通过，快环40.42us，参数保留；转动及控制未复测 |
| [双板Keil工程对齐](2026-10-24-keil-project-alignment-v1.md) | 工作区YG-ESC及双板APP名称已统一；四工程重建/镜像字节一致，冻结副本release通过；Off关相验收见上项，限时版控制尚未复测 |
| [共用APP新板启动与速度实测](2026-10-24-shared-app-speed-hil-v1.md) | 关相启动及2kHz零缺口采集通过；±1/±2点达标，±3仍未达；release通过，参数未保存、已关相 |
| [双板release检查修复](2026-10-24-release-check-repair-v1.md) | 工作树PR与干净冻结副本release全通过；三个目标HEX不变，未接触硬件 |
| [双板共用配置收敛](2026-10-24-dual-board-config-v1.md) | 配置收敛、三个目标编译和定向离线验证完成；后续冻结副本release已通过 |
| [新板完整APP标定与闭环](2026-10-23-r40-app-commissioning-v1.md) | LUT/电零位、动态Id及双向Iq短脉冲通过；关相快照竞态已修复；速度±3rad/s可运行但精度未过，试验增益未落盘 |
| [新板MCU温度可选策略](2026-10-23-mcu-temperature-optional-v1.md) | 关相调试目标临时可选；默认限流修正为3A，实机待机/故障0及120次CAN读取通过 |
| [PA4母线电流APP接入](2026-10-23-pa4-bus-current-app-v1.md) | 按硬件能力选择反馈；关相偏置及120次CAN读取通过，带载绝对精度待验证 |
| [STM32G4 APP入口统一](2026-10-23-stm32-application-entry-v1.md) | 历史目录迁移完成；当前各板application/main.c为真实入口，平台实现归platform_app.c；本次入口分离仅离线验证 |
| [电机配置分离与1kHz默认值](2026-10-23-motor-profiles-1khz-v1.md) | HT有/无阻尼环、独立YQ型号配置；仅编译和离线验证 |
| [Rust GUI实机验收](2026-10-21-host-gui-bench-v1.md) | 当前host_app/rust候选；原计划未记录GUI实机终态，不沿用CLI结果 |
| [当前候选剩余验收](2026-10-21-current-candidate-acceptance-v1.md) | 合并旧计划中仍有效的当前候选/复位/时序证据缺口，未测保持NOT_RUN |
| [AT32F456 最小改动适配](2026-10-22-at32f456-port-v1.md) | 接口契约冻结，已分离旧板配置；先平台与MT6835，输出轴最后；驱动未实施 |

阶段完成和未测边界分别记录，详见[当前限制](../tech-debt-tracker.md)。
