# 标定请求接口

日期：2026-10-30。自检仍只占位，标定通过APP运行接口明确请求；本文不执行硬件操作。

## 通信入口

主站0x02向设备单播发送0x00A0（160），载荷为小端kind:u16、零保留字:u16。
编码器线性化发送`01 00 00 00`，电气零位发送`02 00 00 00`，机械零位发送`06 00 00 00`。
APP协议适配层把三类请求映射到下述运行接口；状态机不解析报文。

回复ACCEPTED只表示受理，完成后状态回空闲。161/162不提供任务查询或按编号取消，
停止使用原110命令。回复中的非零编号只用于受理关联，不保存任务历史或重试记录。
完整字段、结果码和兼容范围见[标定通信v1.6](../protocols/yg_protocol_calibration_v1_6.md)。

## 接口与类型

包含motor_lifecycle.h，在前台调用：

```c
MotorControlResult result =
    MotorLifecycle_RequestCalibration(MOTOR_CALIBRATION_ELECTRICAL_ZERO);
```

| 类型 | 实际操作 | 前提 |
| --- | --- | --- |
| MOTOR_CALIBRATION_ENCODER_LINEARIZATION | 复用编码器LUT采样、构表和验证算法 | 空闲、有效配置、已确认参数、正常电压和编码器 |
| MOTOR_CALIBRATION_ELECTRICAL_ZERO | 对齐后确定电气零位 | 同上，且编码器线性化已完成 |
| MOTOR_CALIBRATION_MECHANICAL_ZERO | 将当前机械位置设为零位 | 空闲、有效采样和编码器，保持关相 |

ACCEPTED表示请求受理并进入CALIBRATION，不表示标定已经完成。
无效类型返回INVALID；占用返回BUSY；状态或前提不满足返回DENIED；故障返回FAULT。

## 执行与结束

- ADC快环复核启相许可，MotorAxis_FastTick在标定与正常控制之间只选择一个分支。
- 电气/机械零位完成返回MOTOR_TASK_COMPLETED：先关相，再由监督层提交COMPLETE，返回IDLE。
- 线性化完成返回MOTOR_TASK_FINALIZE：先关相并等待主循环发布候选；发布成功后才提交COMPLETE。
- 失败进入FAULT；STOP取消当前标定并使旧代际失效。清错仍回IDLE，不进入自检。

MotorLifecycle_GetCalibrationType返回当前类型；完成或取消后为NONE。
标定期间参数写入和普通运行请求被拒绝。成功结果保留在RAM，配置修订只增加一次；
不恢复Flash保存、旧维护票据、通用任务查询或自动重试。

反馈状态仍使用既有协议编号：标定期间为4，完成回空闲后为5，故障为8。
内部状态枚举不可直接作为线上编号；映射由状态适配器完成。

新增名称区分主状态MotorState与标定类型MotorCalibrationType。
辅助函数使用run_active_calibration_cycle、finish_calibration等完整动词短语，
候选发布的参数使用calibration_epoch和publication_succeeded，避免含糊的mode或done。
