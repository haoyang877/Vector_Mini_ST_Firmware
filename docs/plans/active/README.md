# Active Plans

Create one dated Markdown file per significant active change. Include intent, acceptance
criteria, progress, decisions, and final evidence. Move the file to `../completed/` when done.

- [故障保护解耦设计与实施计划 v1.0](2026-09-18-fault-protection-v1.md)：接口、检测逻辑、参数及标定归属；固件实施待启动。
- [公共接口中文契约补全](2026-09-18-interface-contracts.md)：建立机械门禁并优先补齐跨层接口。
- [齿槽补偿扩展到全部转矩闭环模式 v1.0](2026-09-19-cogging-compensation-torque-modes.md)：速度/位置/阻抗模式接线；待并行 `foc_task.c` 改动落定后实施。
- [FOC 快速环重构与遥测精简 v1.0](2026-09-19-foc-fast-loop-refactor.md)：foc_task 拆分、状态发布前台化、HIL 邮箱移除、RTT 4 通道帧；已实施待验收。
- [foc_run.c 拆解简化 v1.0](2026-09-19-foc-run-decomposition.md)：无感状态机与外环/位置适配按单一职责拆函数、重复逻辑收敛、风格债务随改动清理；已实施，验证证据见计划文档。
- [foc_run 归位与瘦身 v1.0](2026-09-19-foc-run-placement.md)：接口层级归位；阶段 0/1.1/1.2/1.3/1.4 已完成（无感运行模块独立成 `foc_sensorless_run`、控制时基下沉 `platform/api/control_config.h`、7 条接口债 + 1 条架构债还清、Keil 主工程 0/0、全量套件 17/17），2.2/2.3 待办。
- [统一运行状态机设计 v1.0](2026-09-19-unified-run-state-machine.md)：显式 RunState（DISABLED/PREPARING/ENABLED/FAULT）、单一写者、worker 结果协议；阶段 1/2a 已实施、2b 进行中（差分等价 38,400 组 tick；电流零偏/相电阻/机械零位/friction/编码器观测器标定完成路径已收拢），阶段 3 待启动。
- [E 生命周期状态机差距分析与迁移方案 v1.0](2026-09-19-e-lifecycle-gap-and-migration.md)：对照 yg_esc E 框架 9 态核心（135 格转移表）与本仓库 4 态现状；提出移植纯核心→适配器→Operation 会话→FaultLatch 四阶段方案；阶段 A/B 已完成（核心逐 token 保真 + 适配器差分 38,400 组 tick 等价，最终合路 YG-ESC）；C/D 设计就绪（Operation 全量会话 + 故障恢复矩阵草案，阈值待确认）。
