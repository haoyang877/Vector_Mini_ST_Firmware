# 当前未结计划

核对日期：2026-10-26。此目录包含当前事项及尚未归档的阶段记录；旧阶段状态不代表当前代码。
版本与工作树仍可能变化，执行前重新冻结源码；本索引不新增硬件操作授权。

| 计划 | 当前状态 |
| --- | --- |
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
