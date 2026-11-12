# 标定执行与结果发布

两板 APP 的 160 请求支持编码器线性化、电气零位和机械零位。协议适配层把 kind 1、2、6 映射为内部标定类型；本轴状态机受理并建立会话后进入 CALIBRATION。受理回复中的 ticket 记录请求，持久化走以下路径。[标定命令](../protocols/README.md#parameters) · [命令映射](../../firmware/app/app_protocol.c)

| 上下文 | 所属模块 | 转交条件 |
| --- | --- | --- |
| ADC 快拍 | `MotorAxis_ControlFast → MotorCalibration_StepFast` | LUT 和电零位在许可启相后执行，机械零位保持关相。任务报完成后，状态机关闭输出并读回，设置待发布阶段。 |
| 前台与 ADC | `MotorCalibration_ProcessPublication / PublishLutFast` | 对 LUT，前台登记待安装的会话，带 Hall 的轴先等转子停稳；ADC 在本拍编码器读回前复核代际并安装。联合标定在原代际继续电零位。 |
| 停机前台 | `MotorCalibration_ProcessPublication` | 准备轴记录，在最终提交前检查原代际、标定类型、STOP、故障和实际关相，再写 Flash 并回读。写入失败锁存存储故障。 |
| 监督拍 | `MotorStateMachine_SupervisorTick` | 保存成功后消费完成事件并回 IDLE。中途停机或故障会使旧候选失效。 |

完整扫圈后的 Hall 候选不合格时，保留失败原因并保存诊断记录。中途取消走 STOP，不提交部分标定。在线参数 SAVE 与自动标定保存是两个独立入口，当前只有后者执行写入。[发布实现](../../firmware/app/motor_calibration.c) · [操作与判定](../guides/calibration_process.md) · [记录布局](../../firmware/parameters/README.md#storage)
