# 电机状态跳转与命令处理 v1.2

更新日期：2026-10-21。对应当前主状态机；实际八状态、线路行为和事件优先级未改变。
职责及总图见[主状态机说明](motor_state_machine_v1.md)；本页逐状态列输入和出口。
原文件名保留以兼容链接，旧九状态及维护内串行 SAVE 候选属于历史方案。

## 全部实际迁移

| 编号 | 起点 → 终点 | 触发与必要条件 | 执行动作 |
| --- | --- | --- | --- |
| T1 | 程序入口 → INIT | 上电或整程序复位，装配初始化 | 建立上下文、关相，开始本次零点和自检；不是其他状态的软件出口 |
| T2 | INIT → IDLE | 八项基础健康连续通过，采样有效、输出关闭、资源释放、无活动故障 | 保持关相，等待明确请求 |
| T3 | IDLE → STARTING | 新 START 通过模式、来源／序号、健康、参数、标定、电压及资源准入 | 建立模式和安全初始参考，准备控制器；受理本身不启相 |
| T4 | STARTING → RUNNING | 控制器就绪；启相前复核 STOP、升级票据、故障及许可；实际输出开启确认 | 启相、确认原身份 STARTED；不表示机械运动或目标已达 |
| T5 | IDLE → MAINTENANCE | 支持的单项任务通过本任务准入和资源检查 | 建立独立票据，执行 LUT／电零位／机械零位／DEFAULTS／SAVE |
| T6 | STARTING／RUNNING → STOPPING | STOP；内部控制结果明确要求停机 | 撤销待启动及目标，关相并统一收尾 |
| T7 | MAINTENANCE → STOPPING | 当前身份的正常完成、STOP 或定向取消 | 保留实际结果，关相、撤销和取消；SAVE 成功须 COMMITTED |
| T8 | STOPPING → IDLE | 输出关闭、目标／结果清理、任务和前台释放、取消确认完成、无锁存故障 | 结束会话，保持关相；满足条件可同拍完成 |
| T9 | FAULT → IDLE | 明确 CLEAR；源恢复、八项健康、输出关闭、资源及取消确认全部满足 | 清锁存并直接待命，不重新 INIT、不恢复旧目标 |
| T10 | 任意非 FATAL → FAULT | 有效故障、当前任务失败、执行保护失效；STOPPING 确认时仍有锁存 | 锁存、关相、撤销和取消；已在 FAULT 时只继续累积 |
| T11 | 任意非 FATAL → FATAL | 致命事件、必要事实／事件缺失、内部代际耗尽、INIT 无法确认关相 | 不可恢复锁存、关闭输出并清理；FATAL 内保持锁存 |

没有 IDLE → STOPPING、INIT → STOPPING、STOPPING → INIT、FAULT → INIT、
FAULT → RUNNING 或 FATAL → INIT 的软件迁移。
无运行中直接切模式或 RUNNING → MAINTENANCE；切模式先 STOP → IDLE → 新 START。

## 每个状态收到外部输入时如何处理

下表针对新的普通请求，假定没有更高优先级故障。相同身份的合法重试只返回原进展，
不重建任务；终态票据的重复取消只确认旧身份，不停止当前其他任务。
“拒绝”表示不执行，结果可为状态不允许、忙、故障或参数／身份错误。

| 当前状态 | 新 START | SET_CONTROL 目标 | STOP | 明确 CLEAR | 新单项维护 |
| --- | --- | --- | --- | --- | --- |
| INIT | 拒绝，继续自检 | 拒绝 | 原地 INIT，撤销排队请求，不重启期限／窗口 | 公开入口拒绝，继续 INIT | 拒绝 |
| IDLE | 准入满足后 STARTING | 拒绝，不隐式启动 | 原地 IDLE，撤销待执行请求 | 无故障时无动作确认 | 准入满足后 MAINTENANCE |
| STARTING | 拒绝新启动，原请求继续准备 | 拒绝 | STOPPING | 拒绝 | 拒绝 |
| RUNNING | 拒绝新启动 | 同模式格式、范围及身份通过后更新目标，保持 RUNNING；异模式拒绝 | STOPPING | 拒绝 | 拒绝 |
| MAINTENANCE | 拒绝 | 拒绝 | STOPPING | 拒绝 | 拒绝并发任务 |
| STOPPING | 拒绝 | 拒绝 | 继续关相／收尾，不重新初始化 | 拒绝 | 拒绝 |
| FAULT | 拒绝 | 拒绝 | 保持 FAULT，补充关相和取消，不清锁存 | 全部恢复条件满足 → IDLE，否则 FAULT | 拒绝 |
| FATAL | 拒绝 | 拒绝 | 保持 FATAL | 拒绝，保持 FATAL | 拒绝 |

维护 CANCEL 必须匹配票据：活动 MAINTENANCE 可进入 STOPPING，已请求取消则重复确认；
未知或不匹配票据拒绝。没有公开的任意状态“取消控制”命令。
纯规则通用 CANCEL 与 STOP 同优先级，不能据此扩大外部票据取消能力。

## 内部事件、等待和自循环

| 当前状态 | 内部输入／事实 | 处理 |
| --- | --- | --- |
| INIT | 八项未齐或连续窗口中断 | 继续 INIT；不齐清连续计数，原期限不延长 |
| INIT | 当前期限到达 | 自检超时 → FAULT；期限判断先于完成 |
| INIT | SELF_TEST_DONE 身份及检查通过 | IDLE；提交完成时条件失效则 FAULT |
| IDLE | 无明确请求或准入不满足 | 保持 IDLE，不自动启动、标定或清错 |
| STARTING | 外环未就绪 | 继续 STARTING；持续许可失效则 FAULT |
| STARTING | START_DONE 身份匹配且输出已开 | RUNNING；输出确认或持续守卫失败则 FAULT |
| RUNNING | 位置到位、零速度、零电流、同模式更新 | RUNNING，不能用到位替代 STOP |
| MAINTENANCE | 任务 RUNNING／SAVE 前台未完成 | MAINTENANCE，只执行当前任务 |
| MAINTENANCE | 当前任务 OPERATION_DONE | 经 STOPPING 回 IDLE；SAVE 必须真实 COMMITTED |
| MAINTENANCE | 当前任务 OPERATION_FAILED | FAULT，即使同拍 STOP 也不能丢失失败 |
| STOPPING | 未关相、前台仍认领或取消待确认 | STOPPING，拒绝新工作 |
| STOPPING | 匹配 CANCEL_DONE 或空事件轮询，停止事实齐全 | 无锁存故障 IDLE，有锁存故障 FAULT |
| FAULT | 源自行恢复、LINK_RECOVERED 或无 CLEAR | FAULT，不自动清错 |
| FAULT | 匹配取消归还结果 | 释放原资源、继续锁存；不是 CLEAR |
| FAULT | 新故障 | 累积锁存，致命错误 FATAL |
| FATAL | 普通事件、恢复或迟到结果 | FATAL，无软件出口 |

陈旧完成不推进新会话，错误类别／效果不报告成功。
SAVE 的 UNKNOWN 或 UNCOMMITTED 不是成功，取消不回滚已提交效果。
兼容枚举 PREPARED／FINALIZE 已没有当前任务路径，所有者收到后丢弃，不启相或记完成。

## 判断顺序与四组事实

纯规则单次提交：FATAL → 有效故障／执行保护 → STOP／CANCEL → 完成 → 新请求。
完成和新请求同事件组合时不执行新请求。
IDLE／INIT 不套用 RUNNING 的持续执行守卫，健康／准入仍按各自职责检查。

| 条件组 | 包含什么 | 对应结果 |
| --- | --- | --- |
| 基础自检 | 新鲜采样、母线／电流／温度、数值配置、编码器在线、关相、资源释放 | INIT 连续通过才 IDLE；FAULT CLEAR 核对完整当前健康 |
| 启动准入 | 健康、参数确认、所需标定／反馈、电压、位置限制、资源及停机票据 | 新请求失败通常拒绝，受理与启相分别复核 |
| 停止确认 | 实际关相、邮箱／结果清理、任务和前台释放、取消确认 | STOPPING 完成直接 IDLE，否则等待 |
| 故障恢复 | 对应源恢复、完整健康、停止及归还完成、明确 CLEAR | FAULT 直接 IDLE，不恢复旧模式和目标 |

INIT／IDLE STOP 不推进规则代际，正常启动、任务创建及取消仍使用原身份规则。
CLEAR 前先确认代际可推进；耗尽 → FATAL，原锁存保留，不能报告 CLEAR 成功。
公开 `RequestClear` 在 INIT 拒绝；纯规则无故障 INIT CLEAR 是幂等空操作，
不能把内部行为解释成公开清错能力。

自检连续 4 个 2 kHz 拍；普通期限 500 ms，本次上电零点未完成时增加
1 秒，当前上电路径为 1.5 秒。FAULT CLEAR 不重新建立 INIT 或 500 ms 期限。
启动滤波母线窗口 25.6～33.8 V，原始值还须低于 34.5 V 硬过压线。
运行保护独立处理，详细诊断见[就绪说明](motor_readiness_v1.md)。

## 代码与历史边界

正常迁移见 `MotorStateRules_Step`／`handle_state_event`，故障优先级见
`handle_priority_events`，实际动作和事实见 `dispatch_state_event`、
`state_apply_actions`、`read_state_conditions`，均位于[主状态模块](../../firmware/app/motor_state_machine/)。
STARTING 实际放行见 `enable_prepared_control`，待机保存及启动派发见 `dispatch_idle_work`。
普通事件在 `handle_normal_event` 内只解析一次，完成事件仍优先于同拍新请求；
IDLE 分支分别校验控制与维护请求，再共用一次新会话代际推进；INIT/IDLE的CLEAR空操作
及STOPPING/FAULT的取消归还各只保留一处处理。
关断请求 `begin_shutdown` 与实际确认 `confirm_stopped` 保持分开，迁移表不变。

历史：[九状态总图](../assets/motor_state_machine/state-transition-map.svg)、
[早期分流程展示](../assets/motor_state_machine/state-flow-linear-review.html)、
[未实施的维护内串行 SAVE 候选](../assets/motor_state_machine/state-simplification-eight-v1.svg)。
它们保留设计过程，不覆盖本页 T1～T11。
本次核对记录见[文档更新计划 v1](../plans/active/2026-10-09-motor-state-documentation-v1.md)。
