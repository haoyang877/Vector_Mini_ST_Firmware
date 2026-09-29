# APP 模块、对象与变量命名 v1

本规范对应 2026-09-29 命名迁移，基线 `66606dd9`。它整理已有软件职责，
不新增电机、编码器、Loader 或上位机功能。根 [STYLE.md](../../STYLE.md) 管理统一格式，
[执行与验收记录](../plans/completed/2026-09-29-app-naming-v1.md) 记录行为保持证据。

## 模块按职责命名

| 模块 | 内容与所有权 |
| --- | --- |
| `app/motor_axis.c/.h` | 单轴组装、控制对象唯一实例、调度入口、参数请求身份与保存作业；公开入口 `MotorAxis_` |
| `app/motor_state_machine.c/.h` | 主生命周期、命令准入、故障与停止确认；显式借用控制对象 |
| `app/motor_execution.c/.h` | 控制执行、外环快照与确定性计算；不拥有主状态迁移 |
| `app/motor_maintenance.c/.h` | 维护任务选择和产品配置组装；功能返回结果事实 |
| `motor/motor_types.h` | 电机状态、故障码和跨模块值类型；不定义对象实例 |
| `motor/identification/encoder_calibration_task.c/.h` | 编码器标定任务；公开前缀 `EncoderCalibrationTask_` |
| `motor/identification/phase_resistance_task.c/.h` | 相电阻辨识的电机任务；纯计算仍在 `phase_resistance.c/.h` |
| `motor/identification/friction_identification_task.c/.h` | 摩擦辨识的电机任务；纯计算仍在 `friction_identification.c/.h` |
| `common/utils.c/.h` | 既有纯数学实现，统一 `Math_` 入口；没有增加通用杂项模块 |
| `common/heap.c/.h` | 固定工作区分配，统一 `Heap_` 入口；分配状态全部文件私有 |

旧文件名和 typedef 不保留转发头、别名或兼容包装。Keil 两个 APP 工程、生成工具、
测试源切片和现行文档使用实际新路径。线路 ID、参数 schema、保存记录与共享产品契约不改。

## 类型、函数与变量

| 类别 | 规则 | 当前示例 |
| --- | --- | --- |
| 类型 | PascalCase，省略 `_TypeDef` | `MotorControlState`、`FocState`、`PiController`、`EncoderState` |
| 公开函数 | 模块前缀加动作 | `MotorAxis_Init`、`PiController_Run`、`Math_SinCos`、`Heap_Allocate` |
| 私有函数 | snake_case | `run_fast_task`、`position_mode_update_configuration` |
| 文件私有对象 | `s_` 加职责名称 | `s_motor_state`、`s_foc_state`、`s_rotor_encoder` |
| 文件私有配置/表 | 同样 `s_`，由 const 限定只读 | `s_friction_config`、`s_sin_quarter_tab` |
| 必须导出的可写对象 | `g_` 加所有者，不作为普通模块通信方式 | APP 业务层没有此类公开对象 |
| 参数、局部变量和成员 | snake_case，无存储前缀 | `motor_state`、`reference`、`fault_code` |
| 故障枚举 | 大写模块前缀，原数值不变 | `MOTOR_FAULT_ENCODER`、`MOTOR_FAULT_OVERCURRENT` |

平台能力接口沿用明确的能力名称，如 `motor_hw_pwm_set_duty`；它们不携带 HAL 类型。
HAL、CubeMX 生成声明、第三方以及冻结共享契约沿用外部命名。
跨文件确有消费者的只读配置保留模块限定名称，例如
`SensorlessStartup_EncoderCalibConfig`；不使用可写全局传递状态。

## 物理量与角色

目标、测量、滤波、实际应用的值分别表明含义，而不是用 Ref、Fbk 或大小写区分。
单位按计算表达确定，不能只根据旧名字猜测。

| 含义 | 名称 |
| --- | --- |
| 电流目标与测量 | `iq_target_a`、`iq_a`、`iq_filtered_a` |
| 母线电压与滤波值 | `bus_voltage_v`、`bus_voltage_filtered_v` |
| 速度目标与斜坡结果 | `speed_target_rad_s`、`speed_ramped_rad_s` |
| 位置目标与轨迹参考 | `position_target_rad`、`position_reference_rad` |
| 电机物理参数 | `motor_phase_resistance_ohm`、`motor_d_inductance_h`、`motor_flux_wb` |
| 转子反馈 | `electrical_angle_rad`、`mechanical_position_rad`、`mechanical_velocity_rad_s` |
| 采样偏置与占空比 | `phase_a_offset_counts`、`duty_a` |
| PI 状态 | `proportional_gain`、`integral_gain`、`sample_period_s`、`integral_output` |

PI 是通用控制器，reference/feedback/output 的单位由调用契约决定，不固定为电流或速度。
Clarke/Park 的坐标分量和归一化调制量同理。纯计算内部的数学短名无需人为附加错误单位。
`s_rotor_encoder` 明确现有传感器负责转子反馈；输出轴角度仍是后续设计可能性，未新增实例。

## 对象组织与解耦

`motor_axis.c` 唯一定义以下六个文件私有对象：

```c
static MotorControlState s_motor_state;
static FocState s_foc_state;
static PiController s_speed_pi;
static EncoderState s_rotor_encoder;
static FluxObserverState s_flux_observer;
static SensorlessStartupState s_sensorless_startup;
```

组合初始化通过显式参数绑定实际计算调用方与主状态机；状态机保存私有借用指针，
不复制对象，也不允许其他模块通过 extern 直接写它们。其他模块交换命令、维护结果、
配置值或快照。函数内局部状态和函数内静态缓存保留原生命周期；改名不改变调度或初始化次序。
本轮不把六个对象合成大型上下文，结构成员顺序、RAM 布局和既有工作区预算保持。

## 历史测试与验证

[命名映射清单](../../tests/fixtures/app_naming_v1.json) 只供离线测试投影冻结源码，
不被 APP、Loader 或共享产品包含。测试保留历史算法和数值，只适配类型、字段及模块名称。
路径替换必须匹配完整文件名，不能把 `motor_state.control_mode` 误当作 `motor_state.c`。
有实际冻结消费者的调度、位置核心和配置缓存测试共同使用该映射，并测试字符串与成员边界。

ARMCC 的 330 项真实目标 sizeof/offsetof/故障枚举值与基线一致；源码 token、完整门禁、
位置逐拍差分、快环调度矩阵和三工程容量证据见执行记录。实机时序、栈峰值和物理关相
仍需独立台架验收，改名验收不代替硬件联调。
