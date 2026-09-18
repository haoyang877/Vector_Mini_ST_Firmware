# FOC 快速环重构与遥测精简 v1.0

日期：2026-09-19。状态：实施完成，随本轮离线验证提交。

## 意图与验收

`foc_task.c` 曾同时承担全局状态定义、初始化、HIL 调试邮箱、模式分发、运行状态迁移、
状态发布与 RTT 遥测。本轮按指令完成：拆分职责、删除状态发布模块、移除 HIL 邮箱分发、
把 RTT 从双布局精简为固定 4 通道帧。验收条件：

1. `foc_task.c` 只保留 `FOC1kHzSupervisor` 与 `FOC20kHzIRQHandler` 的周期顺序；
2. CAN 状态流继续有数据，但采样改由前台打包时直接读取；
3. 迁移块逐 token 等价，20 kHz 调用顺序、故障提交点与功率级延迟使能语义不变；
4. 两个 Keil 目标可构建，普通/HIL 两种编译变体零错误，离线套件通过。

## 结果

| 模块 | 职责 |
| --- | --- |
| `foc_task.c` | 快速环骨架：采样序列、外环 tick、模式分发、后处理、RTT 调用 |
| `motor_state.{c,h}` | 全局状态实例 + `MotorControl_Init` + 配置校验 |
| `foc_mode_dispatch.{c,h}` | 模式 switch、禁用预载、齿槽标定退出清理、机械零位记录 |
| `foc_run_state.{c,h}` | 快速环故障检查、故障指示、功率级启停门控、模式/故障提交 |
| `rtt_telemetry.{c,h}` | 4 通道帧：位置 Q15 单圈 / 转速 0.1 rpm / Iq 指令 / Iq 反馈 |
| `interface_can.c` | `CAN_BuildMotorStatusSnapshot`：前台按需组装快照并发布 |

删除：`foc_status_publish.{c,h}`、`foc_hil_dispatch.{c,h}`、
`tests/unit/native/test_rtt_calibration.py`。RTT JScope 描述符改为 `JScope_i2i2i2i2`。

## 关键决策

- 状态快照由 `CAN_SendMessage`（前台主循环）在收到请求后直接读取
  `MotorControl/FOC/OnBoard_Encoder` 并发布；不再是采样时刻的一致快照。
- 移除 HIL 邮箱分发后，`tools/bench/servo_hil_run.py` 的命令无人应答：
  台架命令路径失效并以超时方式失败（不会产生未授权动作）。
- 旧 12 通道分析链（`tools/analysis/rtt_control_frame.py`、
  `tools/bench/run_mode3_validation.py` 及其分析脚本）保持原样，仅服务历史采集，
  不再与当前固件配套，是否退役待定。

## 验证证据

- 迁移等价：22/22 token 等价比对（函数体、模式 switch、HIL 块、后处理、ISR 调用顺序）
- `check_project_layout`：普通/HIL 两个 Keil 目标 0 错误
- 编译冒烟：普通与 `SERVO_HIL_ENABLE=1` 变体下 `foc_task`/`rtt_telemetry`/
  `foc_mode_dispatch`/`foc_run_state`/`motor_state`/`interface_can` 均 0 错误
- 离线套件：unit、integration 与 15 组原生测试通过，含重写后的
  `run_position_servo_tests` RTT 夹具、`run_can_status_tests` 与 RTT 集成测试
- 运行记录见 `outputs/runs/`；剩余 format/测试失败仅来自并行进行中的
  cogging WIP 文件（`cogging_calibration.c`、`foc_cogging_calibration.c`、
  `test_cogging_calibration.py`），与本轮无关

## 后续

- 旧的 12 通道分析链与主机回放待决定：退役删除或冻结为历史工具（当前冻结）。
- HIL 台架命令路径如需恢复，需要在 app 层重新引入邮箱轮询模块。
