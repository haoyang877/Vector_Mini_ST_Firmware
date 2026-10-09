# 整机流程业务接口与兼容 v1.3

状态：历史设计，2026-10-09 按核心 APP v1.3.0 核对为已退役。
以下六类视图、整计划、模式切换及租约入口均不属于当前固件能力；向量仅证明历史基线。
当前行为见[八状态主说明](../architecture/motor_state_machine_v1.md)、
[全部跳转](../architecture/motor_state_flow_simplification_v1.md)、
[普通启动 v0.2](yg_protocol_ordinary_start_wire_v0_2.md)及[单项维护 v1.2](yg_protocol_calibration_wire_v1_2.md)。

日期：2026-10-08。这是新增 APP 业务接口契约；新 CAN 命令／GUI 尚待独立适配，
不是已发布线路声明。承接[状态恢复 v1.2](yg_protocol_motor_state_v1_2.md)，
原统一帧版本 `0x01`、45B 反馈、schema12、现有请求和参数 ABI 均保持。

## 旧新视图映射

| 八状态执行边界 | 六类业务视图 | 旧线上反馈 |
| --- | --- | --- |
| INIT | INIT | 3；0 仍退役 |
| IDLE，无待启动／待切／标定计划 | IDLE | 5 |
| IDLE，有待启动／待切模式 | RUN 的准备 | 5；已有反馈不伪造输出已开 |
| STARTING／RUNNING | RUN | 2／6，准备与输出确认不能省掉 |
| STOPPING，运行或待切模式 | RUN 的收尾／切换 | 7，确认结束才普通 IDLE |
| MAINTENANCE／活动计划的中间边界／维护 STOPPING | CALIBRATION（含维护／保存） | 沿原任务 mode 投影，普通维护 4、保存等 5、收尾 7 |
| FAULT／FATAL | FAULT／FATAL，优先覆盖子流程 | 8／9；FATAL 无软件恢复 |

六类 enum 是只读业务值，不可直接编码到旧线上状态字节；没有第二份可变主状态。
已有八状态规则黄金摘要仍为 `9dd9307dcc7bb53f`，维护失败摘要仍为 `afc991123a3208fd`。
45B 向量沿[八状态黄金载荷](yg_protocol_motor_state_v1_1_vectors.json)，
新增业务轨迹见[流程向量](yg_protocol_motor_lifecycle_v1_3_vectors.json)，对应真实所有者原生检查。

## 新入口与结果

`RequestCalibration(steps, ticket)` 只接受已有叶算法位 `0x7b`，依赖无解或未知位即拒绝；
LUT 重做会使旧电零位／机械零位失效，不能凭旧标志跳过后续要求。
每个子步骤与最终 SAVE 使用独立票据；父进度 `COMPLETED + COMMITTED` 要求最终保存回读已验证。
STOP／取消撤销后续，借用尚未归还时仍为 CANCEL_REQUESTED；UNKNOWN 不得声称未提交或成功。
completed_steps 不表示电感／磁链已辨识。单项维护的原自动保存与查询身份保持。

`RequestModeSwitch(mode)` 是业务值入口，实际关断后复用准备／复核／输出确认；
同一待切模式重试不重复关断，STOP／故障撤销切换，无法确认关断进入 FATAL。
新启动／切换必须取得请求后真实快环完成的关相中性 PWM 预载证明；仅时间经过不放行。
下一标定步骤及受控切换还须经历关断后的两次快环完成，避免预载尚未生效便开启旧 PWM。
初次和异模式启动为零目标或当前位置；已运行同模式请求保持当前目标。
`RequestIdentifiedRun(request, timeout_ms)` 原子受理四模式及来源绑定租约，复用原半窗口序号、
冲突重试、拒绝记录和 STOP 屏障。原 `RequestIdentifiedEnable` SPEED 线路语义保持。

租约毫秒必须显式提供，范围 `1..INT32_MAX`。有效刷新不启动电机；过期不能用新命令掩盖，
报告 CAN_DISCONNECTED 并关断。FAULT 中正确原身份的刷新仅提供恢复证据，仍须 CLEAR；
成功回 IDLE 后必须新启动，不重放旧目标。未接新入口的旧线路不声称具备超时保护。
公共业务接口不绑定新 CAN Type／字节偏移，H 应独立版本化并保持明确能力协商。
