# 电机驱动软件架构说明

日期：2026-10-08。当前开发按用户的核心功能删减决定收口。
代码是否已编译及实际容量以对应候选记录为准，不引用旧版本结果。

## 主调用链

上电 → 板级初始化 → 参数/LUT读取 → 电流偏置初始化 → IDLE。
IDLE 明确启动一种控制模式；RUN 更新同模式目标。
切模式先 STOP 回 IDLE，再启动新模式；STOP 和 FAULT 均撤销旧目标。
单项 LUT/零位标定由各自任务执行；参数保存以真实回读成功为完成。
CLEAR 只清除满足恢复条件的故障，不启动或恢复旧目标。

## 模块只做一件事

| 模块 | 留下的核心工作 |
| --- | --- |
| 平台/composition | 初始化、采样/功率/存储/CAN能力与对象装配 |
| motor_axis | 一个电机对象，快慢拍及前台调度 |
| motor_execution | 已批准的电流/速度/位置控制任务 |
| position_cascade | 位置→速度→电流，限幅、滤波、抗饱和、重置 |
| motor_maintenance | LUT/零位、参数默认值及保存分发 |
| motor_state_machine | 核心启停、维护准入、故障与资源归还 |
| 参数 | 单份活动配置、基本校验、schema12装载/保存 |
| CAN | 必要控制/反馈/参数和升级；有界帧校验与缓冲 |
| Loader | 镜像边界、读回校验、提交、试启动和失败恢复 |

删除租约、持久启动身份、泛化组合计划、新会话线路、复杂轨迹、摩擦/电阻辨识、
齿槽标定/补偿及自动RTT。必须使用原增益与限值，不能通过弱化保护获得容量。
LUT需要的无感启动/观测器仍保留；不等于开放无感扩展功能。

schema12占位仅用于旧数据字节兼容，不作为运行功能或可用参数发布。
新存储导出中的退役摩擦/齿槽字段清零；不会自动擦写任何现场参数。
独立APP与LoaderApp保持两个真实目标及原layout5，不扩大APP槽。

范围和回退见[核心删减决定](../plans/active/2026-10-08-core-only-v3.md)。

## 状态与参数归属

电机运行对象由motor_axis装配；状态机独占启停和维护准入；各控制器持有自己的积分与滤波。
其他模块使用明确的请求、结果与只读快照，不能通过公开对象绕过运行准入。
原Watch符号保留观察用途，不恢复到处写全局变量的调用方式。
参数仍区分hardware、model、application、device，只有所属模块更新活动配置。
CAN不直接操作PWM或Flash；平台层才接触HAL、句柄和寄存器。

## 源码入口

- 上电与装配：`firmware/app/composition/board_config.c`。
- 周期执行：`firmware/app/runtime/motor_axis.c`。
- 启停与保护：`firmware/app/motor_state_machine/motor_state_machine.c`。
- 控制与标定分发：`motor_execution.c`、`motor_maintenance.c`。
- 核心计算：`firmware/motor/foc/`、`position/`、`calibration/`、`feedback/`。
- 参数加载：`firmware/services/parameters/foc_param.c`；实际存储由平台提供。
- 通信接线：`firmware/app/composition/yg_protocol_owner_adapter.c`。

目录说明见[code_structure](code_structure.md)，接口与硬件隔离规则见[ARCHITECTURE](../../ARCHITECTURE.md)。
旧版扩展管理设计由历史报告保存，不再作为当前实现要求。
