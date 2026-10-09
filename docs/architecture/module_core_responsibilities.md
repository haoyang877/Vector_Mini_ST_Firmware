# 核心工程模块职责

更新：2026-10-09。描述 v1.3.0 核心工程当前源码；历史设计不代表仍存在的功能。

## 先看完整流程

```text
Loader 校验并跳转（LoaderApp） / 芯片入口（独立 APP）
  → 外设初始化 → Board_Init → 参数/LUT装载 → 关相电流零偏采样
  → INIT 基础自检 → IDLE
      ├─ 明确 START → STARTING → RUNNING → STOP → STOPPING → IDLE
      └─ 单项维护 → MAINTENANCE → STOPPING → IDLE
                                      └─ 成功标定/默认值 → 独立 SAVE → IDLE
故障：快环立即关相 → FAULT；恢复条件满足且明确 CLEAR → IDLE
```

没有“未标定就自动启动标定”的流程，也没有组合标定计划。
LUT与电零位分别请求，各任务只流转自己的内部阶段。

## 各模块的核心输入、处理和输出

| 模块及源码入口 | 输入 → 核心处理 → 输出 | 状态归属与边界 |
| --- | --- | --- |
| [composition](../../firmware/app/composition/board_config.c) | 平台能力、参数节点 → 初始化与业务绑定 → 可调度的电机及通信端点 | 只组装，不计算控制算法 |
| [motor_axis](../../firmware/app/runtime/motor_axis.c) | 采样与受理请求 → 快慢拍调度 → 控制输出及一致快照 | 持有单份电机、编码器、FOC、PI与观测器对象 |
| [主状态机](motor_state_machine_v1.md) | 请求、健康与任务结果 → 准入、启停、清理 → 许可及进展 | 独占生命周期和功率动作；不计算标定数学 |
| [控制执行](motor_execution_core_v3.md) | 已准入模式、目标与反馈 → 外环/电流环 → 电流参考及一拍结果 | 控制器拥有积分/滤波；不决定启动 |
| [维护与标定](motor_maintenance_core_v3.md) | 单项任务与显式对象 → 标定或配置处理 → RAM结果、完成/失败 | 任务拥有阶段；主状态机关相；前台保存 |
| [参数](parameter_ownership_v1.md) | 默认值、RAW记录、标量请求 → 装载/校验/事务 → 活动配置及保存结果 | Axis持有值，事务模块持有revision与重试记录 |
| [反馈与保护](../../firmware/motor/README.md) | 角度、电流、电压和温度 → 校正/估计/故障判定 → 测量及故障事实 | 不创建第二份生命周期 |
| [通信](communication_layering.md) | CAN FD帧 → 共享codec/业务映射 → 所有者请求与回复 | 不直接修改电机对象或访问PWM/Flash |
| [平台](../../firmware/platform/api/) | 能力调用 → BSP/ports执行I/O → 值、时间与失败结果 | HAL、寄存器和硬件变体只在平台 |
| [Loader](loader/loader_product_v3.md) | 匹配目标的镜像 → 边界/CRC/提交/试运行 → 启动或恢复升级 | 独立产品；升级会话不等于电机运行会话 |
| [shared](../../shared/README.md) | 跨产品字节和值 → 纯编解码/契约 → 统一数据 | 不依赖APP或HAL，不拥有运行电机 |
| [上位机](../../host_app/README.md) | 只读端口及离线模型 → 页面展示/包检查 → 信息与规划 | GUI尚未接入实际控制、标定及升级写操作 |

## 全局符号与唯一写入者

| 所有者 | 当前关键符号或对象 | 允许的使用方式 |
| --- | --- | --- |
| MotorAxis | `s_motor_state`、`s_foc_state`、`s_speed_pi`、`s_rotor_encoder`、`s_flux_observer`、`s_sensorless_startup`、`s_current_offset_calibration` | 显式传给功能任务；外部读状态快照 |
| 主状态机 | `s_context` | 请求入口串行化启停、目标、维护和清错 |
| 参数事务 | `s_config_revision`、请求缓存与SAVE记录 | 参数请求入口更新；成功写仅增一次revision |
| 位置控制器 | `position_cascade.c`中的私有`s_state` | Update/Reset接口；调用方不写内部积分 |
| 编码器标定 | 任务阶段、窗口和零位累积 | Step/Cancel接口；纯核心通过显式对象计算 |
| 维护工作区 | `maintenance_workspace.c`中的联合区 | 编码器采样与参数记录互斥借用、确认归还 |

部分运行符号因Watch需求是非静态全局，位置/标定内部仍有静态状态。
**全局可见不等于公共写接口，也不等于已消除所有全局状态。**
通信使用请求和快照，控制计算显式接收对象，硬件使用平台能力接口。

## 当前删减范围

删除电机运行租约、持久启动计数、组合计划、复杂轨迹、摩擦/电阻辨识、
齿槽标定/补偿、RTT及堆分配。齿槽历史文档和schema12占位仅作历史/字节兼容。
保留电流、速度、直接位置串级、q轴电压，LUT、电/机械零位，参数保存，保护和Loader升级恢复。
LUT依赖的无感启动/观测器保留，不表示新增独立无感控制产品功能。

当前范围见[删减决定](../plans/active/2026-10-08-core-only-v3.md)，
工程候选和未测项见[v1.3.0交付说明](../releases/architecture-app-v1.3.0-candidate.md)。
