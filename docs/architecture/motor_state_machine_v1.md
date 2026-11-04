# 电机五状态核心

更新日期：2026-10-30。按用户要求删除旧八态框架并重写；自检当前只占位。

## 状态与路径

| 状态 | 行为与出口 |
| --- | --- |
| IDLE 空闲 | 上电直接进入；等待明确运行或标定请求 |
| SELF_TEST 自检 | 仅保留枚举，当前没有入口和自检实现 |
| CALIBRATION 标定 | 执行编码器线性化、电气零位或机械零位；完成后回空闲 |
| RUNNING 运行 | 实际启相后进入；停止后回空闲 |
| FAULT 故障 | 关闭输出；满足恢复条件并明确清错后回空闲 |

启动等待和停止收尾不是主状态。等待启相时仍为空闲，但请求占用会阻止第二次启动；
标定由明确的类型化接口请求；通信160的kind 1、2、6在APP适配层映射为三种类型。
保存、去重和通用任务查询仍不恢复。
仅在关相空闲且没有待启动时允许修改参数。

## 代码职责

- [motor_state_machine.c](../../firmware/app/motor_state_machine/motor_state_machine.c)：
  唯一五态纯迁移函数；不访问硬件、不拥有通信、任务或运行上下文。
- [motor_state_machine.h](../../firmware/app/motor_state_machine/motor_state_machine.h)：
  六个单一事件与纯函数接口，没有事件位图、动作位图或通用规则引擎。
- [motor_lifecycle.c](../../firmware/app/runtime/motor_lifecycle.c)：
  唯一运行上下文，直接实现启停、标定、状态监督及功率保护。
  内部函数仅文件可见，没有跨文件上下文接口或公开转发层。
- [motor_axis.c](../../firmware/app/runtime/motor_axis.c)：
  持有单轴对象并编排快慢环；快环进入BeginFastTick，结束时PostOutcome，
  两个入口各自负责所属阶段的故障检查和关相。
  标定分支复用已有算法；主循环调用ProcessCalibrationPublication发布已验证的LUT。

原motor_state_rules、motor_state_internal及同目录runtime/requests/maintenance实现均已删除。
外部APP调用改为MotorLifecycle_*；MotorStateMachine_*仅剩Step纯函数。

## 运行约束

自检占位不代表可以忽略启相条件：仍检查参数、母线、采样、编码器/模式、零偏和故障。
ADC只在当前请求代际和许可匹配时启相。STOP先撤销许可、关相，再清目标并撤销旧代计算。
故障不会自动恢复运行；清错仅回空闲，必须重新明确启动。

标定完成先关相，监督层再通过COMPLETE回到空闲。线性化任务先发布FINALIZE，
保持CALIBRATION但停止快环执行，前台发布候选后才允许完成。STOP和故障撤销原代际，
迟到的完成或发布结果不能影响新请求。三类标定只改变RAM配置，不自动写Flash。

调用示例见[标定请求接口](../guides/motor_calibration_request.md)，
状态在线路上的映射见[标定接入语义v1.5](../protocols/README.md#feedback)。
三类通信启动见[标定通信v1.6](../protocols/README.md#parameters)。

监督准备、外环计算和提交现由MotorLifecycle_SupervisorTick一次完成。目标命令、启动资格明细和测量快照已删除，状态接口仅保留模式/故障/主状态。

公共值契约现集中于interfaces/motor_state_types.h，ReadStatus一次返回状态/模式/故障；
没有状态邮箱、目标进度或启动资格接口。停机服务独立保留实际关相确认。
