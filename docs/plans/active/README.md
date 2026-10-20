# Active Plans

- [Loader 多目标边界收口 v1](2026-10-21-loader-mcu-boundaries-v1.md)：分层门禁、目标配置贯通及板级/参数职责分离，仅离线验证。

- [schema14配套实机验收 v1](2026-10-20-loader-schema14-acceptance-v1.md)：连续正常升级、静默超时、下载中复位及不通信APP救援，结束恢复原机。

- [Loader与参数存储优化 v1](2026-10-20-loader-storage-optimize-v1.md)：保持单份参数，紧凑存储、显式迁移、分项耗时与运行库瘦身。

- [Loader 连续升级二十轮验收 v1](2026-10-20-loader-repeat20-v1.md)：固定配套固件，连续验证 APP 交接、CAN 下载、确认与镜像回读，最终恢复原固件和参数。

- [Loader 交接队列修复 v1](2026-10-20-loader-handoff-queue-v1.md)：修正 READY 入队、出队和超时删除的读写索引，回归 APP→Loader 实机升级链路。

- [核心边界续收口 v1](2026-10-16-core-followup-v1.md)：修复运行DEFAULTS设备地址一致性，统一维护释放门面，收口核心SPEED能力声明与主机准入。

- [核心契约收口 v1](2026-10-16-core-contract-closeout-v1.md)：修复标定采样失效，解开应用/节点参数策略，收口上位机会话身份与兼容升级；只保留核心功能。

- [主状态机可读性整理 v1](2026-10-16-motor-state-clarity-v1.md)：集中普通事件解析与 IDLE 请求，删除退役分支；保持八状态及动作顺序。

- [主状态机文档同步 v1](2026-10-09-motor-state-documentation-v1.md)：按核心 v1.3.0 核对八状态职责、全部迁移、命令输入和当前总图；仅文档同步。

- [核心功能删减 v3](2026-10-08-core-only-v3.md)：当前最高优先级，覆盖扩展收口与全部辅助模式保留要求；各模块只留下核心控制、标定、保存、保护和升级。

以下是历史参考，遇到范围冲突以核心删减v3为准。

- [多模块与整机流程收口 v2](2026-10-08-architecture-closeout-v2.md)：模块精简与v7已接入待组合验证；新协议、启动身份和真实只读端口在收口，未开放新功率线路。
- [可信启动身份 v1](2026-10-08-boot-epoch-provider-v1.md)：专属61/62页、未知或撕裂不擦、真实提交读回后发布；原本地开发入口保持。
- [主机只读端口 v1](2026-10-08-host-get-only-port-v1.md)：用户显式打开端口、仅查询和严格回帧，未进行设备联调，写入入口保持关闭。
- [整机主/子流程 v7](2026-10-08-motor-platform-lifecycle-v7.md)：已接已有标定串联最终SAVE、受控切换和显式租约；新线路与缺失算法边界单独登记。
- [编码器最小实现 v4](2026-10-08-encoder-calibration-minimal-v4.md)及[调用边界 v3](2026-10-08-encoder-calibration-call-boundaries-v3.md)：共享互斥窗口、归并有界构表/采样及真实退出。

- [架构收口与发布候选 v1](2026-10-08-architecture-release-v1.md)：四项核心冻结、批准的状态出口接入、干净 release 证据与真实发布边界；未进行新镜像实机验证。
- [主状态机八状态 v3](2026-10-08-motor-state-flow-simplification-v3.md)与[出口收敛 v4](2026-10-08-motor-state-flow-simplification-v4.md)：INIT/IDLE 原地 STOP，停止与显式健康 CLEAR 后只回 IDLE，FATAL 仅程序复位恢复。

- [电机模块核心职责解耦 v2](2026-10-08-module-core-decoupling-v2.md)：四项核心已选择性集成，工程输入已接线；组合检查与编译结果见本批报告，未实机验证。
- [位置核心实际解耦 v4](2026-10-08-position-core-implementation-v4.md)：核心/摩擦状态分离，保留运算顺序；独立资源代价如实登记。
- [位置串级解耦 v1](2026-10-08-position-cascade-decoupling-v1.md)及[尺寸梳理 v1](2026-10-08-position-cascade-size-v1.md)：本批继承的窄范围简化。
- [编码器核心提取 v2](2026-10-08-encoder-calibration-core-v2.md)及[任务简化 v1](2026-10-08-encoder-calibration-simplify-v1.md)：计算与任务分开，保持结果应用/退出语义。

- [电流 PI 空闲电压限幅刷新 v1](2026-09-29-current-pi-limit-refresh-v1.md)：有效母线采样后只刷新 d/q PI 电压额度；PR及normal编译通过，新候选未下载；主机三段监视布局已离线接入，守卫及STOP模拟通过，真实时延待独立授权验证，不放宽预算。

- [上电电流偏置与PI初始化修复 v1](2026-09-29-startup-current-offset-pi-fix-v1.md)：关相1秒采样校正与PI配置已修复，PR及normal编译通过；待授权实机验证，不自动下载或启相。

- [Loader P0/P1 v1](2026-09-29-loader-p0-p1-v1.md)：CAN FD 客户端、配套构建、生产身份和持久试运行门槛；本轮仅代码与离线验证，实机验收待执行。

- [取消 common 与通用堆分配器 v1](../completed/2026-09-29-common-heap-retirement-v1.md)：common/heap 已移除，数学归位，固定工作区收口；PR 与 normal 编译通过，新镜像未实机验证。

- [主状态机条件与职责精简 v1](2026-09-29-motor-state-conditions-v1.md)：已适配参数分组并集成；固件测试与 normal 编译通过，全仓库格式门禁待并行上位机源码收口。
- [参数所有权解耦 v1](../completed/2026-09-29-parameter-ownership-v1.md)：四类默认值和 RAM 分组已完成，删除参数节点通信桥；持久层角色/制造身份的完整分离尚未实施。

- [目录职责收口 v1](../completed/2026-09-29-directory-ownership-v1.md)：物理归属与工程路径已完成；参数解耦为下一阶段。
- [固件实机验证 v1](2026-09-29-firmware-machine-validation-v1.md)：专用测试会话独占硬件，保留备份并重新配置 schema12。

- [远程控制闭环收口 v2](2026-09-29-remote-control-closure-v2.md)：实际应用回执已接入；普通使能的线路语义待用户确认。首次未测记录是历史阶段边界，后续证据按匹配提交判定。
- [历史实现与过期入口清理 v1](2026-09-29-legacy-cleanup-v1.md)：源码清理已经完成；保留初次交付记录，不作为新的开发任务。

当前入口为[设备发现与支持范围](../../protocols/yg_protocol_device_discovery_v1.md)、
[主状态机契约](../../architecture/motor_state_machine_v1.md)和
[状态机最新精简记录](../completed/2026-09-29-motor-state-core-v1.md)。
参数遵循[schema12 装载政策](../../protocols/parameter_schema12_loading_policy_v1.md)，
设备发现本批收口见[独立交付记录](../completed/2026-09-29-discovery-contract-closeout-v1.md)。
下方保留各阶段原始计划索引，
其中阶段描述是历史登记状态，不作为当前待办或完成证明。实机验收与新增产品功能另行计划。

Create one dated Markdown file per significant active change. Include intent, acceptance
criteria, progress, decisions, and final evidence. Move the file to `../completed/` when done.

- [轮电机分阶段实机回归 v2](2026-09-29-wheel-bench-regression-v2.md)：先固定候选，再逐步进行备份、下载、通信、校准与启停；普通速度启动入口仍为阻塞项。
- [角度标定与速度闭环验收 v1](2026-09-30-angle-speed-acceptance-v1.md)：LUT 与电零位结果已核验保存，继续补齐普通启动入口并实测速度闭环。
- [轮电机维护标定通路验证 v1](2026-09-30-maintenance-path-bench-v1.md)：顺序验证齿槽、相电阻和摩擦任务，补齐已有所有者的 CAN 类型映射并保存实机证据。

- [故障保护解耦设计与实施计划 v1.0](2026-09-18-fault-protection-v1.md)：接口、检测逻辑、参数及标定归属；固件实施待启动。
- [公共接口中文契约补全](2026-09-18-interface-contracts.md)：建立机械门禁并优先补齐跨层接口。
- [齿槽补偿扩展到全部转矩闭环模式 v1.0](2026-09-19-cogging-compensation-torque-modes.md)：速度/位置/阻抗模式接线；待并行 `foc_task.c` 改动落定后实施。
- [FOC 快速环重构与遥测精简 v1.0](2026-09-19-foc-fast-loop-refactor.md)：foc_task 拆分、状态发布前台化、HIL 邮箱移除、RTT 4 通道帧；已实施待验收。
- [foc_run.c 拆解简化 v1.0](2026-09-19-foc-run-decomposition.md)：无感状态机与外环/位置适配按单一职责拆函数、重复逻辑收敛、风格债务随改动清理；已实施，验证证据见计划文档。
- [foc_run 归位与瘦身 v1.0](2026-09-19-foc-run-placement.md)：接口层级归位 + 硬件解耦；阶段 0/1.1–1.4、2.1/2.2 与 H1.1/H1.3/H1.4/H2.1 完成，H2.2 主体完成（新增 `motor_hardware_profile` 契约、`current_sense_profile` 迁 api、架构债 28→22，2026-09-19 后余 9 条）；H1.2、H5 余项（interface_can/服务协议）待办。
- [统一运行状态机设计 v1.0](2026-09-19-unified-run-state-machine.md)：显式 RunState（DISABLED/PREPARING/ENABLED/FAULT）、单一写者、worker 结果协议；阶段 1/2a 已实施、2b 进行中（差分等价 38,400 组 tick；电流零偏/相电阻/机械零位/friction/编码器观测器标定完成路径已收拢），阶段 3 待启动。
- [FOC 硬件解耦：PWM 契约与 MCU 温度归位 v1.0](2026-09-19-foc-hardware-decoupling.md)：`foc_algorithm.c`/`foc_phase_resistance.c`/`foc_mode_dispatch.c` 改走 `motor_hw` PWM 契约、`mcu_temperature.h` 归位 `platform/api`；阶段 1/2 完成，标定迁移与 shim 删除待 2b 收敛。
- [Encoder 解耦：传感器通道 / SPI 传输 / 角度输出 v2.0](2026-09-19-encoder-decoupling.md)：传感器功能通道 + 每型号 driver，SPI 传输与角度派生/校验分层归位；阶段 A 完成。
- [E 生命周期状态机差距分析与迁移方案 v1.0](2026-09-19-e-lifecycle-gap-and-migration.md)：对照 yg_esc E 框架 9 态核心（135 格转移表）与本仓库 4 态现状；提出移植纯核心→适配器→Operation 会话→FaultLatch 四阶段方案；阶段 A/B 已完成（核心逐 token 保真 + 适配器差分 38,400 组 tick 等价，最终合路 YG-ESC）；C 大部分实施（Save/Default/Zero + 齿槽/相电阻会话已接入，差分扩至 46,464 组并修复相电阻入口启相回归；friction 与 foc_calibration 拆解后任务待接入），D1 已实施（清除权限收归运行状态机 + 恢复矩阵 v1：编码器/温度/电压/过流/CAN 自动恢复，PR 全绿）。
- [对接 deepseek-e 节点：当前节点收尾与供料清单 v1.0](2026-09-19-e-framework-interface-prep.md)：对照 E 六件套 + schema11 对接面；四组 14 项（必须收口 / 供料 / 回移准备 / 冻结禁止）；待在途功能优化完成后按三批统一执行。
- [对接 deepseek-e：供料包（任务 4/5/6/7）v1.0](2026-09-19-e-handoff-pack.md)：E 核心逐 token 复验（2571/383 一致）与回灌包（待审，git apply --check 通过）；适配器语义对照表；Operation 会话规格；故障三口径待补。
- [硬件边界收口：仅 platform/stm32g4 为硬件域（AI 交接说明）v1.0](2026-09-19-hardware-boundary-closure.md)：services/communication/app 剩余硬件耦合的清零路线（T1–T5）+ AI 执行规则；完成后收紧 harness app 层 allows，把定则变成机械门禁。
- [编码器标定重建：Mode 13 / Mode 15 v1.0](2026-09-20-encoder-calibration-rebuild.md)：按裁决重建观测器 LUT 与电角度零位两条链路（结果协议接入、常量上移、Keil 登记）；其余三个标定任务仍待重建；实机验收待台架。**T1–T5 全部完成**（2026-09-19：board_hw/indicator_hw/param_store 契约、app 零硬件头、foc_param 清零与 `common_inc.h` 删除、CAN 契约化、app 层 allows 收紧；协议测试全绿）；**release 档已通过**（提交 `c39d0d25`，11/11）。
- [通信分层优化设计 v1.0](2026-09-19-communication-layering-optimization.md)：按分层说明落地"协议约定/协议实现/耦合"三层；S0–S4、S6 已实施并提交，通信层硬件债 0、隐藏 `extern CANMsg` 归零；S5 未执行（转下条）。
- [CAN 接收中断瘦身（S5）设计与验收计划 v1.0](2026-09-19-can-isr-slimming.md)：中断内完整派发改为有界队列 + 2 kHz 派发。**已按方案 A 实施；PR 档全绿 + 实机验收通过（突发应答 5/5、断连保护 0.404 s，2026-09-20）**。
- [HIL 退役：删除 HIL 工程与工装，仅保留 normal 工程 v1.0](2026-09-19-hil-retirement.md)：删除 HIL Keil 工程、`SERVO_HIL_ENABLE` 固件变体、HIL 台架工装与测试、`profiles.hil`/`layers.hil`，构建/发布链路改单目标；**已完成**（2026-09-19：PR 档全绿 + release 档通过，提交 `c39d0d25`）。
- [监督时基 2 kHz 与三层环路架构 v1.0](2026-09-20-supervisor-2khz.md)：TIM7 1 kHz→2 kHz 与内容重标定（分频、心跳、温度、总线恢复）；编码器慢估计、外环控制与主状态机迁入 2 kHz 慢拍，故障紧急关断保留 20 kHz 快车道。**阶段 1–4 已实施；PR 档全绿 + 实机台架验收通过（2026-09-20）**。
- [电机装配/启动参数归位 motor 层 v1.0](2026-09-20-motor-profile-relocation.md)：拆分 `control_config.h`——时基/采样率留 platform/api，阻尼环装配选择与无感启动/编码器标定默认值归 `motor`（`motor_hardware_profile.h`、`motor_startup_profile.h`）；宏名与取值不变。**已实施，PR 档全绿；Keil/release 待提交后执行**。
- [公司 CAN FD 通信协议框架重构 v1.1](2026-09-20-company-canfd-protocol-migration.md)：先搭建协议核心、CAN FD 传输、命令网关和服务边界，再分阶段接入电机控制、反馈、参数、升级和 5 轴 1 kHz 同步；当前框架和只读链路已实现，实时电机行为仍待接线与实机验收。
- [通信协议确认记录 2026-09-22](2026-09-22-communication-decision-record.md)：冻结公司帧、节点、1 Mbit/s / 5 Mbit/s、1 kHz、超时重试、参数/标定/升级范围和剩余六项输入。
- [yg_protocol 业务重构实施方案 v0.1](2026-09-23-yg-protocol-business-implementation.md)：基于候选 Type 表分阶段接入安全停机、单轴控制/反馈、参数/标定、五轴 1 kHz 和升级；当前待评估，未启动业务代码变更。
- [电机通信协议重构技术实现规范 v1.0](../../architecture/communication_protocol_refactor_spec_v1.md)：供 AI 和开发者执行重构的顶层规范，统一来源层级、目标架构、程序流、硬件解耦、文件边界、迁移阶段和验收规则。
- [yg_protocol 模块开发交接单 v0.1](2026-09-21-yg-protocol-module-handoff.md)：共同基线、业务所有者分工、文件所有权、可复制任务模板和集成验收；第一轮 DeepSeek v4.1-flash 并行开发已完成电机 STOP/DISABLE 适配器与参数只读适配器，分片可靠性任务待审。
- [电机 STOP/DISABLE 适配器第一步 v0.1](2026-09-21-yg-motor-adapter.md)：通过窄停机所有者接口实现幂等、异步确认和拒绝语义；尚未接入真实 FocRunState/CAN 入口。
- [参数描述符模型与 ID 分区设计 v1.0](2026-09-20-parameter-descriptor-model.md)：把逐 case 参数处理收敛为单一描述符表（分组 + `apply_policy` + ID 分区）；冻结"未实现不登记枚举"纪律，不改既有 106 个 ID；仅设计，P0 之外不实施。
- [编码器测速估计优化：低延迟估计器替换 8 ms 滑窗 v1.0](2026-09-23-encoder-speed-estimation-optimization.md)：Phase 0 只读确认 TLE5012B 内部更新率/滤波（延迟地板，前置阻塞）；Phase 1 用 20 kHz 角度样本的低延迟估计器（最小二乘斜率/α-β）替换 2 kHz 割线；Phase 2 可选角度跟踪观测器。原"2 阶 IIR 替换"经量化复核否决（等噪声下不降延迟）；Phase 0 只读采集已实施（PR 档全绿），寄存器台架读取待执行。
