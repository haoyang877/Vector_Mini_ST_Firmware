# 电机状态反馈与恢复契约 v1.2

日期：2026-10-08。用户已审全部跳转并批准本版。承接[八状态映射 v1.1](yg_protocol_motor_state_v1_1.md)，
本版收敛 STOP 与故障恢复行为，状态值、45B payload、CRC、统一帧版本 `0x01`、
参数 schema12 和请求／任务身份 ABI 保持。实施见[出口收敛 v4](../plans/active/2026-10-08-motor-state-flow-simplification-v4.md)。

| 事件 | v1.1 | v1.2 |
| --- | --- | --- |
| INIT／IDLE 收到 STOP | 资源条件不齐可能进入 STOPPING，原地处理也推进代际 | 保持原状态及代际，撤销未执行的请求；不复位控制器或重启 INIT |
| STOPPING 完成 | 按自检记录回 IDLE／INIT | 仅回 IDLE；故障仍进入 FAULT |
| FAULT 明确 CLEAR 成功 | 返回 INIT 再自检 | 在 FAULT 内确认源恢复和完整基础健康后直接 IDLE |
| FATAL | 图中曾以复位箭头连接 INIT | 普通软件事件没有出口；整程序复位后从程序入口重新初始化 |

只有 STARTING、RUNNING、MAINTENANCE 进入 STOPPING。
STOPPING 内执行关闭功率输出、撤销控制、取消任务及等待实际借用结束；
实际结束才确认停止。后台仍占用时可以等待，不能先开放新任务覆盖其内存。
INIT／IDLE 的后台占用不产生主状态迁移；各自原有健康或准入检查继续执行。

CLEAR 仍走现有明确请求入口。当前故障源恢复、八项基础健康、输出关闭、
任务／前台资源归还以及取消确认均满足后才成功；条件未齐继续 FAULT。
原参数未确认或标定不完整仍可 IDLE，启动条件独立判断。
CLEAR 不恢复旧目标，不自动启相。FATAL 的 CLEAR／STOP／连接恢复均不能解除锁存。
若 CLEAR 时内部工作代际已经耗尽，锁入 FATAL 并明确拒绝清错，保留原故障；不得报告成功。
没有增加自动清错、第二状态机或恢复等待标志。

状态值保持 INIT=3、IDLE=5、STARTING=2、RUNNING=6、STOPPING=7、FAULT=8、FATAL=9；
MAINTENANCE 沿任务 mode 投影为 4／5，0 继续退役。黄金 payload 仍见
[v1.1 向量](yg_protocol_motor_state_v1_1_vectors.json)；本版新增的是状态事件和恢复轨迹检查。
主机应接受 FAULT(8) 经 CLEAR 直接 IDLE(5)，不再等待 INIT(3)；
INIT／IDLE STOP 不再期待 STOPPING(7)。序号屏障和原请求的终态仍必须使用原身份查询。

STARTING→RUNNING 的“输出已开启”只指真实数字使能回读，不表示机械运动。
任务与 SAVE 的独立结果、实际 Flash 效果、异步取消和迟到结果契约保持。
源码验证不代替设备下载、物理输出和时序验收。

## 当前核心版本的阅读入口

2026-10-09 文档同步：上述 STOP／CLEAR／FATAL 行为继续适用于核心 APP v1.3.0，
本次没有修改线上值、payload 或黄金字节。主状态职责、全部迁移及命令输入分别见
[主说明和当前总图](../architecture/motor_state_machine_v1.md)、
[T1～T11 与逐状态输入表](../architecture/motor_state_flow_simplification_v1.md)。

| 主状态 | 固件内部值 | CAN 状态投影 |
| --- | ---: | --- |
| INIT | 1 | 3；线上 0 退役 |
| IDLE | 2 | 5 |
| STARTING | 3 | 2 |
| RUNNING | 4 | 6 |
| MAINTENANCE | 5 | LUT／电零位为 4；SAVE／机械零位／DEFAULTS 为 5 |
| STOPPING | 6 | 7 |
| FAULT | 7 | 8 |
| FATAL | 8 | 9 |

MAINTENANCE 的 5 与 IDLE 的 5 重合，不能只凭线上状态字节确认任务已结束；
应结合维护票据和结果。当前仅支持五项单项维护，整计划／租约／在线切模式已退役。
新 START、同模式目标、停止及查询的线路细节沿用现行各自契约，不直接发送内部枚举。
