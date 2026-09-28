# 功能任务驱动的主状态机重构 v1

## 目标与基准

用户批准从功能函数端重构并删除旧状态体系。基准 fda84c2c，独立分支执行，
现有容量优化工作树和用户未提交文件不变。基准普通 APP ROM 89808 B / RAM
31600 B；Loader APP 诊断跨度 97552 B / RAM 31840 B，真实 78 KiB 仍超限。
基准构建和 PR 证据位于 outputs/state-machine/baseline/ 和
outputs/runs/state-machine-baseline/。诊断链接永不可部署。

## 最终职责与迁移子任务

1. 功能结果改为 RUNNING、PREPARED、COMPLETED、FINALIZE、STOP_REQUESTED、FAILED。
   删除 next_mode、power_off、SWITCH_MODE；功能内部阶段保留，功能不能选择下个任务。
2. 运行对象只保存 MotorControlMode；维护任务由唯一主状态机持有 kind/epoch/phase。
   删除 ModeNow/ModeLast/旧枚举、基于模式差分推断事件和旧分发准入层。
3. motor_state_machine 拥有命令准入、真实 guards、正常功率许可、任务衔接及取消。
   保留已测试 AppLifecycle 纯迁移规则作为唯一核心，不引入第二套迁移路径。
4. motor_execution 保留有界控制调度；motor_maintenance 只选择实际功能调用、取消
   与准入。删除 foc_run_state、foc_mode_dispatch、foc_run 旧入口，不添加兼容头或别名。
5. 正常启相仅由主状态机执行。齿槽功能先准备零矢量，再返回 PREPARED，由所有者
   按实际 guard/epoch 原子复核启相；快速故障优先关断，无硬件关闭证据不得 READY。
6. 保存策略归所有者：编码器、置零、默认参数和齿槽完成衔接独立 SAVE；摩擦与相阻
   保持现行仅 RAM 完成策略。标定票据和派生保存票据、UNCOMMITTED/COMMITTED 分开。
7. 迁移遥测、指示器、工程输入、测试和维护文档。CAN/参数 ABI 不变；线上模式映射
   只保留在协议边界。停止关闭输出；受控停止运动的模式设计不在本轮新增线上能力。

## 不变行为、验收和回退

保持 FOC 数学、参数校验和 Flash schema11、CAN Payload、标定过程、限流/限速、
STOP/FAULT/CLEAR 优先级、真实资源归还、原始票据及过期结果丢弃。运行私有结构
改变后 SWD 必须使用同版 ELF；不兼容旧源码和 SWD 字段。

逐任务结果与原基准刻画对比；真实所有者原生验证包含启动准备、拒绝优先级、
任务完成到保存、取消借用、故障与完成竞态、启相失败/关相失败和过期通知。
改动代码局部 format/lint 必须通过，执行 PR/release 及工程输入检查；构建普通 APP
和 Loader APP 诊断对比 ROM/RAM、ABI、产物哈希。现有基线门禁债务独立记录。

本轮纯离线，不烧录、不写设备参数、不控制电机。MCU 时序、栈、物理停止与标定
电流须后续明确台架验收，离线结果不构成发布许可。按阶段提交；回退本轮提交
恢复基准，保留其他会话与工作树未提交文件。

## 执行记录

七项迁移已完成，实现提交 aaff067cb956cc766e67edd091d2e0e09d903220。
旧状态协议与文件已删除；正常启相集中于唯一所有者；功能自行归还终态状态，
维护与保存票据分别确认，原 ABI/黄金向量通过。24 项原生、19 项集成通过，
43 个改动自有源码的格式/lint 0 问题；普通 APP ROM −592 B，RAM 不变。

当前正式 release summary 为 outputs/runs/state-machine-final-release/summary.json，
实现提交 clean。正式结果 FAIL：全局 format/lint/接口/历史报告链接诊断与基准逐字
相同，release 前置门禁阻塞其内置构建；没有扩大债务、添加豁免或声称发布通过。
另行对实现提交重建普通 APP 0 错误/警告；Loader APP 容量仍超限，诊断镜像不可部署。
实际台架时序、栈与关断证据在本轮授权范围外，未执行。

详见[验收报告](../../reports/2026-09/motor_state_machine_refactor_20260928.md)及
[最终设计](../../architecture/motor_state_machine_v1.md)。本计划完成的是上述离线
重构范围；全局发布门禁和实机验收作为明确未完成项记录，不静默豁免。
