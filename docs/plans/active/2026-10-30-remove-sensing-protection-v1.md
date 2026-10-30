# 采样快环删除软件保护方案 v1

## 目标与范围

按用户明确要求，删除 20kHz 采样快环中由采样模块自己承担的软件保护判断，采样模块只负责
把 BSP 快帧的物理值搬到 `MotorMeasurement` 并维持母线低通滤波。用户已确认删除；
本计划不授予硬件操作授权，实施后不得宣称实机通过。

范围仅限 `firmware/motor/sensing/` 的两个更新函数、其头文件契约、`motor_axis.c` 调用点
及对应离线测试。不改 BSP 换算、ADC 采集、PWM 许可、状态机、参数、协议或 Flash。

## 删除项（行为变化）

| 位置 | 删除内容 | 删除后行为 |
| --- | --- | --- |
| `motor_voltage.c` | 硬过压 3 拍旁路、滤波过压 2 拍确认、欠压 100ms 确认、NAN 立即过压 | 只写 `bus_voltage_v` 与 `bus_voltage_filtered_v` |
| `motor_current.c` | 三相过流连续 5 拍确认与 `MOTOR_FAULT_OVERCURRENT` | 只写三相电流 |
| `motor_current.c` | 快帧缺失/组无效时置 NAN 并报 `MOTOR_FAULT_CURRENT_OFFSET` | 缺失时置 NAN，不报故障 |
| `MotorVoltage_Update` | `motor_state`、`output_expected` 形参 | 签名只留测量对象与快帧 |
| `MotorCurrent_Update` | `configuration`、`motor_state` 形参及零偏合法性检查 | 签名只留测量对象与快帧 |

`MotorCurrentOffsets_IsValid` 函数本体保留（`MotorFault_SourceRecovered` 仍在用）；
上电零偏标定流程、`MotorCurrentOffsets_Sample/Finalize` 不变。

## 保留项（本次不动）

- BSP：ADC 采集、跨 ADC 收齐、原码到物理量换算、缺零偏填 NAN、有效位、超龄判定。
- 状态机：启动电压窗口 `state_vbus_in_enable_window`、转换故障诊断码、启停与故障锁存。
- 热保护、编码器在线判定、ADC 硬件健康检查与 AT32 快环超时停机。
- 看门狗：快拍无心跳仍会使前台在 10ms 后停机；STM32 与 AT32 均保留。
- 协议故障码枚举与状态映射：`MOTOR_FAULT_OVERCURRENT` 等仍存在，只是不再由采样产生。

## 已确认的后果

- 过流保护完全消失：固件内没有硬件比较器或刹车，删除后无任何过流关断。
- 过压/欠压不再在运行中触发停机，只在启动与维护准入时按窗口拒绝。
- 快帧失效时 FOC 会读到 NAN；`Foc_UpdateCurrentLimits` 与 `Foc_SetVoltageModulation`
  对无效母线电压的既有处理（限幅为 0、调制清零）是唯一兜底，不再是故障停机。
- `MotorFault_UpdateRecovery` 的过流恢复计数成为不可达路径，保留但不清理。

## 验收

1. `uv run python tools/run.py verify --profile pr` 通过，记录 `summary.json`。
2. 离线测试改为验证新契约：采值映射、缺帧置 NAN、母线低通收敛、无故障上报。
3. 不新增编译告警；`format --check` 与 `lint` 通过。
4. 明确记录未烧录、未台架、未验证真实过流/过压响应。

## 回退

单文件回退：`git checkout -- firmware/motor/sensing firmware/app/runtime/motor_axis.c`
并还原本计划涉及的测试文件；不改 BSP 即可恢复原保护，无需迁移或数据修复。
