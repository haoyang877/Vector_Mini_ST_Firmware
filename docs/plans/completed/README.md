# 已完成计划记录

- [PITCH 1024点 Hall LUT](2026-11-06-hall-lut-1024-v1.md)：配对采样、运行滤波及实机精度/延迟验证完成，发布门禁限制另记。
- [PITCH Hall LUT 实机验收](2026-11-06-hall-lut-bench-v1.md)：由1024点方案接续完成；保留128点未达标历史。

结果只适用于各记录的原始范围和基线，未测/失败/限制仍按原文保留。

- [线性 Hall 128 点 LUT](2026-11-06-linear-hall-lut-v1.md)：先方案与 144 组合成验证，再接入原码快照、前台建表、4 kHz 查表和 OEN2；双板编译零错误零警告，PR 仅保留两项既有失败，未上板。

- [观测器与无感启动规范整理](2026-11-06-observer-startup-style-v1.md)：启动类型归属、私有命名与初始化契约收敛；8 组 ARM 对象一致、原生回归及 PR 通过，未上板。

- [统一编程规范与 PI 标准样例](2026-11-06-programming-standard-v1.md)：三份规则职责收敛；PI ARM 对象一致，61,440 次对照通过；完整 PR 保留两个基线失败，未新增失败。

- [最近一条协议命令的全局观测](2026-11-04-protocol-last-command-v1.md)：一个48B全局符号覆盖APP各类命令，跨类型与坏帧测试、完整PR和三套Keil构建通过。

- [协议单调用点合并](2026-11-04-protocol-inline-v1.md)：合并六个辅助函数，通信实现净减74行，完整PR、前后行为对照和三个APP构建通过。

- [通信业务结构体聚合](2026-11-04-protocol-structs-v1.md)：请求回复、编码载荷和电机绑定聚合，净减71行；128,592组对照、完整PR与三个APP构建通过。

- [普通通信与Loader通信分离](2026-11-04-protocol-boot-split-v1.md)：两个实现加一个头文件，净减142行；删除重复检查，128,592组对照、完整PR与三个APP构建通过。

- [通信函数按用途命名](2026-11-03-protocol-function-names-v1.md)：46处函数命名统一，名称还原后代码一致；完整PR和三个APP构建通过，Flash/RAM总量不变。

- [通信静态初始化与直接调用](2026-11-03-protocol-direct-calls-v1.md)：删除注册和二次分发，通信净减202行；协议对照、原生回归、三工程构建通过，整仓三项并行变更检查未过。

- [通信实现可读性整理](2026-11-03-protocol-readability-v1.md)：入口前置、直接命令处理、具名结果码；128,592组前后对照、完整PR和三个APP构建通过。

- [协议按内容拆分](2026-11-03-protocol-three-units-v1.md)：yg_protocol、motor、device 三个同级实现及一个公共头；PR、四配置轨迹和三个 APP 构建通过。

- [协议说明合并](2026-11-03-single-protocol-doc-v1.md)：删除68份旧说明，保留一个现行入口；29份夹具字节不变，完整PR通过。

- [通信收敛为两个文件](2026-11-03-protocol-two-files-v1.md)：18→2文件，净减1038行；命名精简、无用遥测和参数后端删除，完整PR与三个APP构建通过。

- [通信调用链压平](2026-11-03-protocol-flatten-v1.md)：删除 Endpoint/Runtime 包装/adapter/provider，通信净减854行；PR与三个APP前后编译通过，RAM各减1944B。

- [协议消息宏名简化v2](2026-11-03-protocol-message-names-v2.md)：同步电机、参数、任务、Boot、升级消息名，19 个编号不变；完整 PR 和四个固件工程通过。
- [协议消息宏名同步](2026-11-03-protocol-message-macro-names-v1.md)：同步五个文件的 24 处引用，编号与报文不变；完整 PR、三个 APP 构建通过，HEX 相同。
- [APP启动分步表达](2026-11-03-app-start-readable-v1.md)：App_Start改为显式判断，原顺序及失败处理不变；四配置轨迹、PR和三个APP构建通过。
- [APP通信初始化分步表达](2026-11-03-app-init-stages-v1.md)：展开短路调用并说明各阶段职责；前后轨迹、完整PR和三个APP构建通过。
- [删除启动重复Vq清零](2026-11-03-remove-startup-vq-reset-v1.md)：移除四行重复分支，保留参数读取和默认回退；启动专项及最终PR通过。
- [编码器16/21位输出选择](2026-11-03-encoder-output-bits-v1.md)：读取参数选择位宽，控制环保留16位；全角度域、PR及八工程通过，未烧录。
- [MT6835直接字段v2](2026-11-03-mt6835-direct-fields-v2.md)：命令/地址、角度高低位、状态/CRC独立访问；ARMCC位布局、PR和AT32构建通过。
- [MT6835收发帧结构化](2026-11-03-mt6835-frame-struct-v1.md)：命名字段直接用于DMA，6字节布局固定；PR与AT32构建通过，HEX相同。
- [APP函数说明与可读性整理](2026-11-03-app-function-comments-v1.md)：18个函数补齐说明并整理命名；PR、双板构建及四配置对照通过，HEX相同。
- [删除APP调试命令入口](2026-11-03-app-debug-removal-v1.md)：调试邮箱、命令分派和专用测试已删除；PR、双板构建及四配置行为对照通过，未烧录。
- [APP组合入口可读性重构](2026-11-03-app-readability-v1.md)：职责分组、命令分派及健康监督整理；前后PR、双板构建和四配置行为对照通过，未烧录。
- [删除MotorBench及Commissioning工程](2026-11-03-motor-bench-removal-v1.md)：专用命令、工具和工程已移除；PR、冻结release与八目标通过，未烧录。
- [删除AT32启动状态](2026-11-03-at32-startup-state-removal-v1.md)：移除s_startup_state及全部依赖判断；PR与四个正式目标通过，未烧录。
- [AT32初始化默认成功](2026-11-03-at32-init-assumed-success-v1.md)：删除初始化异常返回，保留就绪等待和运行保护；PR、最终冻结release及九目标通过，未烧录。
- [Keil标定结果保存](2026-11-03-keil-calibration-save-v1.md)：命令5同步保存及回读，机械零位加载与AT32 Flash看门狗窗口修复；最终PR及九目标通过，未上板。
- [Keil全局标定控制](2026-11-03-keil-calibration-control-v1.md)：非static选择量与命令4接入三类标定；完整PR及四个正式目标编译通过，未烧录。
- [正式APP全局调试命令](2026-11-03-keil-global-control-v1.md)：Keil前台单次启停与目标提交，原准入保留；完整PR及四个正式目标构建通过，未烧录。
- [双板中断入口集中到main](2026-11-02-main-interrupt-routing-v1.md)：三类向量由各板映射，BSP保留硬件处理；9目标、PR及冻结release通过，未烧录。
- [AT32启动失败分支去除goto](2026-11-02-at32-startup-no-goto-v1.md)：指定system.c三处跳转改为直接返回；修改前后生命周期检查及完整PR档通过，未烧录。
- [FOC输出角度按1.5Ts补偿](2026-11-02-foc-phase-advance-1p5ts-v1.md)：共用逆Park按75μs预测；当前工作区PR、84组解析检查与双板构建通过，未烧录。
- [低电流短脉冲与角度速度复核](2026-11-02-current-short-pulse-v1.md)：12次正常结束，0.3A阶跃通过，0.1/0.2A未全达标；同步角度/速度复核完成，未修改固件。
- [D轴正弦扫频](2026-11-02-d-axis-frequency-sweep-v1.md)：低电流25点×2次完成，−3dB约2.96kHz；1.5A档记录前限速，高电流未完成。
- [Vd/Vq与1.2kHz实测](2026-11-02-vdq-current-loop-1200hz-v1.md)：3文件最小修改；电压、稳态及Id阶跃通过，Iq第2次速度截止并取消剩余脉冲。
- [电流阶跃等效带宽](2026-11-02-current-bandwidth-estimate-v1.md)：补测阶跃并给出条件化提升建议；Iq限速停止，未提高PI。
- [电流环稳定性与噪声](2026-11-02-current-loop-stability-v1.md)：实际PI整定约1kHz，关相噪声及0/0.3/0.75A Id工况通过；真实频响与非零Iq锁轴未测。

- [116与实时状态接通](2026-10-31-motor-live-protocol-v1.md)：原接口目标提交与实时反馈完成，正负电流实测和PR/release通过；低速跟踪仍未验收。

- [CAN FD协议实测v1](2026-10-31-protocol-validation-v1.md)：62项检查及1000次查询通过；原始主机误判保留，停止/续租/重连已独立重测。

- [历史测试收敛v2](2026-10-30-test-cleanup-v2.md)：16组失败迁移完成，40组原生/126项单元/16项集成通过，10项工程0错误0警告；未提交、未烧录。

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

- [输出轴绝对位置与启动恢复](2026-11-06-output-encoder-v1.md)：直驱/整数减速实现、OEN1 存储、双板编译及 PITCH 100 圈联合标定、9 点复位与实际断电恢复通过；release 两项既有测试失败保留。
