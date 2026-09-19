# Active Plans

Create one dated Markdown file per significant active change. Include intent, acceptance
criteria, progress, decisions, and final evidence. Move the file to `../completed/` when done.

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
- [硬件边界收口：仅 platform/stm32g4 为硬件域（AI 交接说明）v1.0](2026-09-19-hardware-boundary-closure.md)：services/communication/app 剩余硬件耦合的清零路线（T1–T5）+ AI 执行规则；完成后收紧 harness app 层 allows，把定则变成机械门禁。**T1–T5 全部完成**（2026-09-19：board_hw/indicator_hw/param_store 契约、app 零硬件头、foc_param 清零与 `common_inc.h` 删除、CAN 契约化、app 层 allows 收紧；协议测试全绿）；release 档证据待干净工作树补。
- [通信分层优化设计 v1.0](2026-09-19-communication-layering-optimization.md)：按分层说明落地"协议约定/协议实现/耦合"三层（S0–S6，行为保持）；S1 复用硬件边界 T3，S4 消除隐藏 `extern CANMsg`；设计待评审。
- [HIL 退役：删除 HIL 工程与工装，仅保留 normal 工程 v1.0](2026-09-19-hil-retirement.md)：删除 HIL Keil 工程、`SERVO_HIL_ENABLE` 固件变体、HIL 台架工装与测试、`profiles.hil`/`layers.hil`，构建/发布链路改单目标；**已完成**（2026-09-19，PR 档全绿；release 档证据待干净工作树补）。
