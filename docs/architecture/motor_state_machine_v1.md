# 功能任务驱动的主状态机 v1

2026-09-28，基准 fda84c2c。当前实现删除旧模式体系，生命周期使用已验证的
AppLifecycle 纯转换核心，唯一实际所有者为 APP 的 motor_state_machine。
本轮不新增受控停止、双编码器或上位机功能，不改变 CAN Payload 和参数 schema11。

## 四类状态与代码归属

| 维度 | 状态与所有者 | 作用 |
| --- | --- | --- |
| 生命周期 | AppLifecycle：BOOT、SELF_TEST、READY、STARTING、RUNNING、STOPPING、MAINTENANCE、FAULT、FATAL | 纯规则计算迁移和动作，无硬件访问 |
| 控制模式 | motor_state_machine 选择 CURRENT、SPEED、POSITION、VQ、NONE | motor_execution 按已受理模式执行控制 |
| 维护任务 | motor_state_machine 持有 kind、原始 epoch、执行资格和后台认领 | SAVE、DEFAULTS、机械零位、编码器 LUT、电零位、相阻、摩擦、齿槽串行执行 |
| 功能内部阶段 | 各标定或辨识模块私有 | 对齐、采样、拟合等算法阶段，不决定主状态或下一任务 |

```text
firmware/app/
  motor_state.c             私有电机对象、快慢环组合、参数活动版本、前台服务
  motor_state_machine.c/h   请求、真实守卫、正常功率许可、结果消费和保存衔接
  motor_execution.c/h       电流/速度/位置/Vq 执行和有界外环
  motor_maintenance.c/h     实际维护功能调用、准入原因和定向取消
  yg_protocol_*_adapter.c   线上值与一致快照边界
firmware/services/lifecycle/
  app_lifecycle.c/h         唯一纯生命周期转换规则
  motor_control_command.h   SI 单位目标值命令
  motor_lifecycle_request.h 维护类型、来源序号、原始票据和进展
firmware/motor/
  motor_control_mode.h      四种控制与 NONE，不包含维护或清错命令
  motor_task_result.h       功能事实；不携带下一模式、功率动作或保存指令
  identification/           具体功能和私有内部阶段
```

已删除 foc_run、foc_run_state、foc_mode_dispatch、motor_work 和旧模式枚举，不保留
兼容头、别名或基于模式变化推断事件的路径。SWD 必须使用同版 ELF 字段。
正常启相只在 motor_state_machine；功能任务没有正常输出使能权。快速故障关断保留。

## 结果与调度契约

| MotorTaskResult | 功能报告 | 主状态机处理 |
| --- | --- | --- |
| RUNNING | 未终结；无错误 | 继续当前有界周期，同时登记快环采样进展 |
| PREPARED | 齿槽功能已配置零矢量 | 复核当前 epoch、未 STOP、健康、采样和输出守卫；成功启相，否则故障终止；重复就绪不重复启相 |
| COMPLETED | 功能完成，结果在 RAM | 撤销执行资格、关相、清控制参考；记录原任务完成，再按策略衔接独立 SAVE |
| FINALIZE | 齿槽采集结束，需前台整理 | 关相并保留 MAINTENANCE，等待同身份前台认领、整理、发布和归还 |
| STOP_REQUESTED | 控制保护要求关闭输出 | 进入统一停止路径，不自动重新启动 |
| FAILED | 失败和明确故障码 | 同快环锁存并紧急关相，监督拍对齐 FAULT、取消及资源归还 |

20 kHz 快环只运行一个获准控制或维护功能。2 kHz 监督拍消费结果、提供真实守卫、
执行迁移。Flash 和表整理在前台执行。各算法计算、抽点频率和外环准备策略保留。
结果邮箱绑定创建时 epoch，保留首个终态，FAILED 可覆盖尚未消费的成功。
消费时在同一临界区撤销维护执行资格，防止中断再次调用已结束算法。

STOP 先撤销目标和执行资格；陈旧、取消后的结果不能复活任务或请求 SAVE。
故障优先于完成与取消；CLEAR 不恢复旧目标。硬件数字关相、功能静默和前台实际归还
是独立证据，缺一不能 READY。电压保护在实际输出尚未关闭时持续工作。
数字关相回读不能代替物理栅极测量；停止运动仍需独立控制模式，本轮未添加。

## 保存、取消与反馈

| 完成任务 | 原任务效果 | 后续策略 |
| --- | --- | --- |
| 编码器 LUT、电零位、机械零位、默认参数 | COMPLETED + UNCOMMITTED | 主状态机创建新 SAVE 票据 |
| 齿槽 | 前台候选发布 RAM 后 COMPLETED + UNCOMMITTED | 主状态机创建新 SAVE 票据 |
| 相电阻、摩擦 | COMPLETED + UNCOMMITTED | 保持原 RAM-only 策略，显式 SAVE 才持久化 |
| SAVE | Flash 写入及回读验证后 COMMITTED | 不启动其他功能或运动 |

前台认领使用原始 `{epoch, kind}`，最终发布或写入前在同一短临界区复核。
取消只作用于当前功能；借用没有归还时保持 STOPPING/FAULT。迟到结果只能归还，
不能标为 COMMITTED。原始标定票据与派生 SAVE 票据分别可查；同来源序号重试
复用原票据，不再次执行。维护准入只返回原因，由所有者锁存原有拒绝故障。

线上状态仅在 yg_protocol_status_adapter 映射：CURRENT/SPEED/POSITION/VQ
为 1/2/3/6；编码器/电零位/齿槽为 7；相阻/摩擦为 8。其他维护为 0。
状态与测量、维护 kind 来自同一临界快照，内部枚举不直接作为线上数值。

## 验收入口

[执行计划与回退](../plans/completed/2026-09-28-motor-state-machine-v1.md)记录基准和证据。
原生真实所有者测试覆盖请求、功率失败、过期结果、取消借用、任务与保存身份；
结构检查禁止旧状态契约与功能侧正常启相；协议固定编号和原有黄金向量继续验证。
本轮只做离线构建和测试，未烧录、未修改设备参数、未控制电机。

[本批验收、资源和未验证项](../reports/2026-09/motor_state_machine_refactor_20260928.md)。
