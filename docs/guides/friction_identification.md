# 摩擦辨识：算法、维护任务与接入边界

日期：2026-09-29。本文描述当前源码。旧 USB 命令、标准 CAN 参数号和模式 19 入口已删除。

## 模型与运行条件

算法在空载、允许连续正反转的电机上辨识电流域等效摩擦：

```text
omega > 0: iq_a =  Ic_pos + B_pos * omega
omega < 0: iq_a = -Ic_neg + B_neg * omega
```

`omega` 为机械角速度 rad/s，`phase_c_current_a` 为 A，`B` 为 A/(rad/s)。辨识本身不执行摩擦补偿。
轴承、密封、铁耗、风阻及逆变器误差都会进入模型；机构上的重力、弹簧或外部负载
也可能进入结果，没有外部力矩测量时不能将它们完全分离。

维护请求先由主状态机检查 READY、无故障、轴配置、母线电压、输出与资源条件。
摩擦任务还要求编码器在线且 `ENC_CALIB_ALL` 有效，包含线性化、电角零位和机械零位。
worker 检查正电流限制、足够的速度限制及有效速度 PI；台架需具备空载正反转空间。

当前默认速度点为 `+0.10, -0.10, +0.20, -0.20, +0.40, -0.40, +0.50, -0.50 rev/s`。
每点先用 0.75 s 平均窗口确认稳态，再采集至少一机械圈且至少 0.5 s；每点结束先减速至零。
整圈平均用于降低齿槽等周期扰动。跟踪、采样或停车超时、电流持续饱和、无效样本、
电流方向不符合假设或拟合残差超限都会失败，既有电压、电流、温度及编码器保护继续有效。

## 所有权与调用关系

| 模块 | 职责 |
| --- | --- |
| `firmware/app/motor_state_machine.c` | 唯一受理者，维护 ticket、准入、输出许可、取消及资源释放确认 |
| `firmware/app/runtime/motor_maintenance.c` | 装配私有 `static const` 配置，按 `MOTOR_MAINTENANCE_FRICTION` 调度真实功能 |
| `firmware/motor/identification/foc_friction_identification.*` | 用显式电机、PI、编码器与配置输入执行任务，返回 `MotorTaskResult` |
| `firmware/motor/identification/friction_identification.*` | 确定性采样、状态推进及拟合，不访问硬件或参数服务 |

启动辅助函数与辨识实例均为模块私有，不提供绕过主状态机的公共启动入口。
配置由组合层传入，核心初始化时复制配置；任务不持有参数服务对象。
worker 归零控制参考并报告结果，主状态机负责功率关断及生命周期转换。
取消受理不代表资源已归还，应查询任务终态。

## 当前通信与保存范围

内部维护 API 已支持摩擦任务。CAN Type 160 的线路 kind=5 映射摩擦辨识；
不要发送旧模式号或将内部枚举直接作为线路值，详见
`docs/protocols/yg_protocol_calibration_wire_v1_1.md`。
原始八点导出及候选应用的产品通信入口尚未实现，不提供旧 USB 命令兼容。

辨识成功只产生候选，`FrictionIdentificationTask_ApplyCandidate()` 是内部停机应用函数，
当前没有产品调用方。应用 RAM 模型与写入 Flash 是不同步骤；不得把完成状态当成已保存。
当前只装载 schema 12，不自动迁移旧参数。

接入时依据 [主状态机契约](../architecture/motor_state_machine_v1.md)、
[标定线路契约](../protocols/yg_protocol_calibration_wire_v1.md) 和
[schema 12 装载策略](../protocols/parameter_schema12_loading_policy_v1.md)。
本轮离线重构没有开放新的运动或参数写入入口。
