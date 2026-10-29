# 状态机中断屏蔽与核心精简 v2

## 目标

逐项审核状态机的PRIMASK和BASEPRI区间，删除可证明多余的嵌套，缩短可安全缩短的区间，并精简相应状态处理；保留全部对外功能。不把仅移动代码当作消除复杂度。

## 不变项

保持八状态、故障、状态和任务所有权、采样、控制、功率启停、STOP/取消优先级、启动确认、维护/Flash真实效果、协议重试、参数和Loader ABI。ADC与更高优先级中断可抢占BASEPRI；共享快环数据不能仅依靠BASEPRI保护。先前目标提交的短PRIMASK不得无替代删除。不得接触硬件或改写其他工作。

## 基线、验收、回退

当前工作树源码、工程及测试快照为 `outputs/state_irq_simplification_v2/baseline_source/`；原生规则262144组合与状态所有者、R40变体基线见 `outputs/state_irq_simplification_v2/baseline/`。针对每项改动验证访问上下文与被保护不变量，补充必要的嵌套屏蔽/抢占反例，比较原生状态、故障与功率轨迹。运行格式、lint、PR验证及两板快照构建；评估资源和时序影响，未授权实机时记录未测。仅回退本轮代码，保留已有未提交修改和忽略数据。

## 审核记录

逐项记录保留、删除、缩短及其原因；不得将宿主夹具的单一互斥模型当作ADC抢占证明。

| 区间／入口 | 审核结论 |
| --- | --- |
| 快环 `PostOutcome` | 仅ADC最高优先级调用，低优先级不能抢占；移除BASEPRI，保留载荷先于标志的发布屏障。 |
| 监督取结果 `take_current_task_result` | 原BASEPRI挡不住ADC；仅邮箱非空时用短PRIMASK复制原代际和载荷并清标志，解锁后判断资格。空邮箱不屏蔽。 |
| STOP/取消撤销结果 `discard_pending_outcome` | 原BASEPRI挡不住ADC；非空时短PRIMASK复制并清空，故障报告移至区间外。空邮箱不屏蔽。 |
| `apply_pending_control` | 保留低优先级事务保护；先前的短PRIMASK保留，用于快故障复核和目标/进度原子提交。 |
| 状态事实读取及规则提交 | 保留对外入口的控制锁；拆分已持锁的事实读取及规则提交，消除受理、清错、启相、外环提交等嵌套锁。快环仍可抢占，不将BASEPRI当作ADC互斥。 |
| 自检/就绪 `ReadReadiness`、配置资格 | 多字段快照及可能的配置读取需要与前台/监督串行，保留；调用方本身已持锁时不再重复取得内部事实锁。 |
| 启动资格与进度 `prepare_start_configuration`、`RequestEnable`、`RequestIdentifiedEnable`、`ReadEnableProgress` | 身份、队列及跨监督提交保持控制锁；纯配置校验继续在可证明稳定时位于锁外。启动态最终发布改为仅在可能产生终态时的短PRIMASK，防止ADC故障与STARTED相互覆盖。 |
| `RequestClear` | 故障恢复事实、源恢复与清错提交保持同一低优先级事务；删除内部重复事实及提交锁。 |
| 维护请求/查询/取消 | `RequestMaintenance`、`RequestIdentifiedMaintenance`、`ReadMaintenance`、`CancelMaintenance` 保留会话、票据和资源检查锁；维护开始及取消的规则提交不再嵌套同一锁。 |
| 远程停止及Loader交接 | `RequestRemoteStop`、`RequestIdentifiedStop`、`RequestBootStop`、`BootHandoffReady`、`ReadBootHealth` 保留身份、票据及状态快照的低优先级保护；BootHandoff事实读取不再嵌套。 |
| 控制命令/进度 | `SubmitControl`、`ReadControlProgress` 保留前台与监督事务保护；BASEPRI不提供ADC互斥，不将其视为快故障保证。 |
| 自动SAVE与启相准备 | `dispatch_idle_work`、`enable_prepared_control` 保留对状态/任务/配置组合的现有提交保护；启相的真实硬件门仍由ADC快环单独复核。 |
| 监督准备与提交 | `PrepareSupervisor`、`CommitSupervisor` 保留控制快照及提交区间；带已核对事实的规则提交不再嵌套。ADC仍可抢占这些BASEPRI区间，不能声称其保护快环数据。 |
| 后台认领及完成 | `BeginBackground`、`FinishBackground` 保留票据、工作区和结果交接的低优先级事务；Flash及LUT发布仍使用其自身保护。 |

回归：两板原生状态和262144组规则用例通过；真实所有者输出、动作和状态轨迹与本轮前基线一致（仅忽略已删除的锁深度差异）。注入ADC时验证邮箱不会吞掉新失败、启动结果不产生STARTED/FAULT矛盾；验证空邮箱、空闲监督及无待处理启动不使用新增全局屏蔽。排查发现的两个竞态已在明确复现后修复，属于并发异常时的行为纠正。

## 资源与验证补充

- 当前工作区两板状态机定向测试通过，含取消与消费邮箱、启动提交的ADC抢占注入；结果轨迹和规则摘要与冻结基线一致，锁深度预期变化不计入行为差异。
- 在独立快照使用Keil 5.36分别重建当前STM32G431和AT32F456应用，均0错误、0警告：`workspace_final2_stm32.log`、`workspace_final2_at32.log` 位于 `outputs/state_irq_simplification_v2/`。不下载、不接线。
- 为隔离并行测量结构迁移，对本轮之前的源码快照及仅合入本轮修改的快照分别构建：两板Code+RO-data均增加120字节，RW/ZI不变。对比日志为 `baseline_source_stm32.log`、`baseline_source_at32.log`、`isolated_final2_stm32.log`、`isolated_final2_at32.log`。
- 初始完整PR在并行工作格式调整期间曾失败；后续代码和测试验证以最终检查为准。目标板上的PRIMASK最长持续时间、ADC抖动、闭环和保护行为未测，不能用离线验证推断通过。
- 最终工作区PR验证通过：`outputs/runs/20261029T111449336010Z-2dbb4003/summary.json`；两板最终快照构建与本轮关键状态机源码哈希一致。维护取消邮箱也覆盖了ADC在撤销点到达的反例。
