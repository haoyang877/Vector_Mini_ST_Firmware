# 电机五态与功率交接

核对日期 2026-11-09。STM32G431 和 AT32F456 共用一套状态机，每根轴持有自己的 `MotorLifecycle`。上电进入 `IDLE`。`SELF_TEST` 保留枚举值，当前没有生产入口。

| 主线 | 谁提出 | 何时改变主状态 |
| --- | --- | --- |
| 空闲 → 运行 | 前台受理启动，登记模式、起始目标和会话 | 外环发布启相意图，ADC 拍首复核本会话、采样、模式、STOP 和故障。只有 PWM 启相且读回确认后，才提交 `RUN` |
| 运行或标定 → 空闲 | 前台 STOP 或快拍任务请求停机 | 请求立即撤销许可。ADC 遇到待停请求先关相；监督拍或外环提交入口消费 STOP、清目标、撤销会话、确认关相，再提交 `STOP`。关相失败时进入故障 |
| 空闲 → 标定 | 前台受理指定标定类型 | 从关相空闲且输入有效的轴发起，受理时提交 `CALIBRATE`。需要通电的标定由 ADC 再复核并启相；机械零位保持关相 |
| 标定 → 空闲 | 快拍报告算法完成 | 先关相。需要安装 LUT 时由 ADC 在读角前复核会话；前台复核并保存，成功后由监督拍提交 `COMPLETE`。联合标定在同一会话转入电零位 |
| 任意状态 → 故障 | 保护逻辑锁存故障，或启相关相确认失败 | 快环看到故障立即抑制功率，慢拍处理故障并提交 `FAULT`。故障锁存和主状态写入发生在各自的执行上下文 |
| 故障 → 空闲 | 前台显式 CLEAR | 必须关相、输入有效、故障源满足恢复条件。若原子清故障字失败则恢复 `FAULT`；成功只回空闲，启动需重新请求 |

## 从代码跟一条路径

- 从 [motor_state_machine.c](../../firmware/app/motor_state_machine.c) 的 `transition_motor_state` 开始读。它按 RUN、CALIBRATE、COMPLETE、STOP、FAULT、CLEAR 处理实际事件。初始化之后的主状态写入全部在这里。`MotorStateMachine_NextState` 给出五态之间允许的转换。
- 启动请求登记后，外环批准本次启相，ADC 在 `BeginFastTick` 处理 RUN。STOP、标定保存成功后的 COMPLETE、首次 FAULT 共用关相和会话收尾。CLEAR 在关相和故障源恢复后才回空闲，若同时发生新故障则仍锁存 FAULT。
- 每次 ADC 采样后，[MotorAxis_ControlFast](../../firmware/app/motor_axis.c) 先调用 `BeginFastTick` 检查故障和启相许可，再读取主状态。运行态执行 `MotorControl_Step`，可执行的标定态执行 `MotorCalibration_StepFast`。最后 `PostOutcome` 接收本拍结果并检查新故障。
- 前台在 [motor_lifecycle_exchange.c](../../firmware/app/motor_lifecycle_exchange.c) 校验目标并直接写本轴，也从本轴读取 CAN 反馈。外环在 `MotorLifecycle_CommitOuter` 直接写有效结果。ADC 单独复核启相条件。
- 标定的停稳、LUT 安装和 Flash 保存见 [motor_calibration.c](../../firmware/app/motor_calibration.c)，线路状态和命令编号见[协议说明](../protocols/README.md)。除 Flash 事务外，这些交接不暂缓 ADC。
