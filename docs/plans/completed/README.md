# 已完成计划记录

结果只适用于各记录的原始范围和基线，未测/失败/限制仍按原文保留。

- [BSP整理后AT32烧录与关相验收](2026-10-27-bsp-cleanup-at32-flash-v1.md)：独立读回、运行、CAN及参数保留通过；未启相、未转动。

- [AT32 BSP职责收敛](2026-10-26-at32-bsp-responsibility-cleanup-v1.md)：统一命名、删除实现期测试代码，平铺11文件，九工程与完整回归通过；未烧录。


- [BSP私有接口收敛后的AT32关相实测](2026-10-26-bsp-private-at32-hil-v1.md)：修复时序问题，九工程、完整release、J-Link读回及CAN实测通过；未转动。

- [BSP私有头与直接查询收敛](2026-10-26-bsp-private-header-consolidation-v1.md)：统一私有声明、删除两处纯转发，九工程及完整离线验收通过；未操作硬件。

- [双板能力配置与接口收口v1](2026-10-25-platform-capabilities-v1.md)：统一能力/使能/必需条件，清理采样、编码器、PWM/CAN边界；9目标及PR/release通过，未烧录。

- [双板CAN职责与接口v1](2026-10-24-can-responsibility-refactor-v1.md)：统一CAN职责和共享DLC校验；9目标及PR/release通过，AT32关相通信实测通过。

- [双板PWM与功率输出职责v1](2026-10-24-pwm-power-refactor-v1.md)：统一PWM接口、输出硬件所有者、目标许可和初始化职责；9目标和PR/release通过，未烧录。

- [双板编码器职责拆解v1](2026-10-24-encoder-responsibility-refactor-v1.md)：统一编码器接口、器件协议与SPI职责；9目标和PR/release通过，AT32批准镜像关相实测通过。

- [双板采样职责拆解v1](2026-10-24-sampling-responsibility-refactor-v1.md)：共同采样契约、AT32职责拆分与STM32完整链路归位；9目标编译和PR/release通过，未烧录。

- [2kHz三段监督v1](2026-10-21-supervisor-staged-v1.md)：前处理、获准计算和复核提交已落地，离线与debug构建通过，实机/时序未测。
- [APP四入口收口v1](2026-10-21-app-entry-v1.md)：源码与离线构建已收口，正式release及实机限制见正文。

- [齿槽转矩补偿解耦与运行路径优化 v1.0](2026-09-19-cogging-compensation-decoupling.md)
- [参数只读适配器（yg_protocol P 任务）v0.1](2026-09-21-yg-parameter-read-adapter.md)
- [APP 业务边界接线 E4 v1](2026-09-28-app-boundaries-e4.md)
- [APP 收口与资源约束 E5 v1](2026-09-28-app-closeout-e5.md)
- [受控台架速度入口 E2 v1（已退役）](2026-09-28-bench-speed-owner-e2.md)
- [APP / Loader 容量适配 v1](2026-09-28-capacity-fit-v1.md)
- [运行所有者功能分析与局部快照优化 v1](2026-09-28-foc-run-state-analysis-v1.md)
- [内部所有权与异步结果解耦 E3 v2](2026-09-28-internal-decoupling-e3.md)
- [生命周期真实条件接线 E1 v1](2026-09-28-lifecycle-guards-e1.md)
- [Loader 独立产品重构 v1](2026-09-28-loader-product-refactor.md)
- [Loader 简化审查与修复计划 v1](2026-09-28-loader-simplification-review-v1.md)
- [Loader 修复合入 system-refactor v1](2026-09-28-loader-system-merge-v1.md)
- [功能任务驱动的主状态机重构 v1](2026-09-28-motor-state-machine-v1.md)
- [最简启动/升级方案（去 Boot/Record）v1](2026-09-28-simplified-boot-handoff.md)
- [通信、主状态机与 Loader 整体重构 v1](2026-09-28-system-refactor-e-h-d.md)
- [APP 模块与变量命名统一 v1](2026-09-29-app-naming-v1.md)
- [齿槽控制状态所有权收口 v1](2026-09-29-cogging-state-owner-v1.md)
- [取消 common 与通用堆分配器 v1](2026-09-29-common-heap-retirement-v1.md)
- [文件目录职责收口 v1](2026-09-29-directory-ownership-v1.md)
- [设备发现与当前契约收口 v1](2026-09-29-discovery-contract-closeout-v1.md)
- [APP/Loader 重复实现收敛 v1](2026-09-29-duplicate-removal-v1.md)
- [快环调度与控制执行边界收口 v1](2026-09-29-execution-boundary-v1.md)
- [框架边界与验证门禁收尾 v1](2026-09-29-framework-closure-v1.md)
- [Keil APP 职责分组 v1](2026-09-29-keil-app-groups-v1.md)
- [Keil APP 精简分组 v2](2026-09-29-keil-app-groups-v2.md)
- [APP 工程更名与导航修正 v3](2026-09-29-keil-app-name-v3.md)
- [主状态机冗余删减 v1](2026-09-29-motor-state-core-v1.md)
- [电机主状态机可读性优化 v1](2026-09-29-motor-state-readability-v1.md)
- [电机主状态机模块收敛 v1](2026-09-29-motor-state-unification-v1.md)
- [四类参数所有权解耦 v1](2026-09-29-parameter-ownership-v1.md)
- [基础自检与模式就绪 v1](2026-09-29-self-test-readiness-v1.md)
- [齿槽采样任务与运行适配解耦 v2](2026-10-08-cogging-core-decoupling-v2.md)
- [齿槽标定核心职责与等价简化 v1](2026-10-08-cogging-core-simplification-v1.md)
- [齿槽最小任务流程重写 v3](2026-10-08-cogging-minimal-task-v3.md)
- [MotorAxis 保留模式的最小实现 v3](2026-10-08-motor-axis-minimal-implementation-v3.md)
- [MotorAxis 参数事务所有权解耦 v2](2026-10-08-motor-axis-parameter-owner-v2.md)
- [MotorAxis 职责与等价简化 v1](2026-10-08-motor-axis-simplification-v1.md)
- [主状态机文档同步 v1](2026-10-09-motor-state-documentation-v1.md)
- [核心契约收口 v1](2026-10-16-core-contract-closeout-v1.md)
- [核心边界续收口 v1](2026-10-16-core-followup-v1.md)
- [独立上位机升级依赖收口 v1](2026-10-16-host-portable-upgrade-v1.md)
- [主状态机可读性整理 v1](2026-10-16-motor-state-clarity-v1.md)
- [配置与运行状态实体分离 v3](2026-10-20-configuration-runtime-separation-v3.md)
- [Loader 交接发送队列修复 v1](2026-10-20-loader-handoff-queue-v1.md)
- [Loader MCU 适配指南编写 v1](2026-10-20-loader-mcu-porting-guide-v1.md)
- [Loader 连续升级二十轮验收 v1](2026-10-20-loader-repeat20-v1.md)
- [紧凑参数配套固件实机验收 v1](2026-10-20-loader-schema14-acceptance-v1.md)
- [四类参数配置入口收口 v2](2026-10-20-parameter-entrypoints-v2.md)
- [参数来源与编译期派生收口 v1](2026-10-20-parameter-sources-and-constants-v1.md)
- [当前重构版本三项整机验收 v1](2026-10-21-core-full-bench-acceptance-v1.md)
- [核心执行路径精简 v1](2026-10-21-core-path-simplification-v1.md)
- [快环执行预算恢复 v1](2026-10-21-fast-loop-budget-recovery-v1.md)
- [APP文件命名与分层统一 v1](2026-10-21-file-naming-layering-v1.md)
- [Loader 多 MCU 改造复审 v1](2026-10-21-loader-boundary-review-v1.md)
- [Loader 多目标边界收口 v1](2026-10-21-loader-mcu-boundaries-v1.md)
- [Runtime职责收口 v1](2026-10-21-runtime-responsibilities-v1.md)
- [状态机临界区与运行对象写入边界收口 v1](2026-10-21-state-critical-budget-v1.md)
- [主状态机核心流程精简 v2](2026-10-21-state-machine-core-flow-v2.md)
- [Project harness implementation — 2026-09-18](project_harness_20260918.md)

- [文档体系清理与对齐v1](2026-10-21-docs-cleanup-v1.md)：归档、合并、源码核对及本轮检查结果。

- [Platform收敛为BSP、板级配置和原样SDK](2026-10-26-platform-bsp-sdk-v1.md)

- [共用电机BSP与实现分离](2026-10-26-common-motor-bsp-v1.md)：九目标/Loader及PR/release通过，AT32永久关相验证通过；剩余文件精简项见报告。

- [BSP最小直接实现](2026-10-26-minimal-bsp-v1.md)：删除17个文件，九工程及Loader、PR/release和AT32关相验证通过。
