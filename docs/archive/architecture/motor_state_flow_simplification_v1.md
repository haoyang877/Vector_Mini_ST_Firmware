# 五状态跳转与请求处理

归档于 2026-11-10。本文保留旧阶段的设计与行为；当前实现见[电机状态机](../../architecture/motor_state_machine_v1.md)和[文件职责](../../architecture/code_structure.md)。

更新日期：2026-10-30。旧八态方案由五态重写取代，自检暂不实现。

详见[五状态核心](../../architecture/motor_state_machine_v1.md)。实际迁移只有：

- 上电 → 空闲。
- 空闲 + 实际启相确认 → 运行。
- 空闲 + 标定受理 → 标定。
- 运行/标定 + 停止或完成 → 空闲。
- 任意状态 + 故障 → 故障。
- 故障 + 明确清错且恢复条件满足 → 空闲。

SELF_TEST仅保留枚举。未实现进入、执行或自动通过逻辑，诊断的自检完成位固定为零。
标定由MotorStateMachine_RequestCalibration请求，支持编码器线性化、电气零位和机械零位。
完成时先关相，再由监督层提交COMPLETE返回空闲；线性化还须等待前台发布已验证的LUT。
STOP和故障使原会话代际失效。结果仅更新RAM，不恢复SAVE或旧维护任务协议。
