# 旧SWD台架邮箱退役记录

> 历史归档：以下为旧Watch指南的退役附录，原基线和删除范围保留。当前观测见[Watch指南](../../guides/watch_motor_commands.md)。



状态：2026-09-28 随容量适配批次退役。原 `foc_bench_speed.h` 定义的 `FocBenchSpeed`
调试邮箱（版本 3、92 B）、`RunState_ServiceBenchRequest` / `RunState_BenchFastGuard` /
`RunState_PublishBenchState` / `RunState_BenchCancel` 及速度台架会话已从固件删除，
用于回收 Flash。本节保留为退役记录，不再是操作入口。

### 退役范围

- 该邮箱是 SWD/J-Link 台架调试入口，不是线上 CAN ABI，也不参与产品控制路径。
- 删除内容：`firmware/app/foc_bench_speed.h`；运行所有者内的台架请求受理、序号
  回放、状态发布、SWD 输出区，以及快环期限/相电流/超速截止。
- 删除钩子：`FocRunState_CheckFastFaults`、`FocRunState_Tick`、
  `FocRunState_FastFaultStop`、`FocRunState_RequestRemoteStop`、`FocRunState_Init`
  中原有的台架调用与共享状态。
- 当前公开入口位于 `motor_state_machine.h`，统一使用 `MotorStateMachine_*`；
  `FocRunState_*` 旧门面已删除。生命周期纯核心、带身份维护请求和保存认领仍保留。
- 受影响：原 START/UPDATE/STOP 速度台架、command 4..12 维护直达、独立试验期限
  与采样式相电流截止不再可用。产品相电流/过流保护仍由既有保护路径负责，
  不依赖本邮箱。

### 替代入口

- 内部使能、目标、停机、清错：`MotorStateMachine_RequestEnable`、`SubmitControl`、
  `RequestRemoteStop`、`RequestClear`。CAN 117 走有身份启动候选入口，116 仅更新运行中的同模式目标。
- 维护、标定、保存：带来源身份的维护受理、ticket 查询/取消及前台保存认领。
- 升级交接：`MotorStateMachine_RequestBootStop` / `MotorStateMachine_BootHandoffReady`。
- 受理不是物理关断或资源释放证明，须按 [主状态机契约](../../architecture/motor_state_machine_v1.md) 确认。

容量批次记录见 [APP / Loader 容量适配 v1](../../plans/completed/2026-09-28-capacity-fit-v1.md)。

## 2026-10-08 状态机合并更新

主状态为 INIT、IDLE、STARTING、RUNNING、MAINTENANCE、STOPPING、FAULT、FATAL。
INIT／IDLE STOP 保持原状态；停止完成与 CLEAR 成功直接 IDLE；FATAL 没有软件恢复出口。
不再观察已删除的 tested 字段；INIT 的期限和连续健康窗口不因 STOP 重启。
Watch 内部状态数值不能当作 CAN 枚举，当前契约见[状态恢复 v1.2](../../protocols/yg_protocol_motor_state_v1_2.md)。
