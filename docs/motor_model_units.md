# 电机模型计算单位说明

本文档整理 Vector_Mini_ST_Firmware 工程中电流矢量控制与电机模型计算所使用的物理单位及换算关系。工程内部模型统一使用 **SI 单位（浮点）**，仅在 USB/CAN 上位机交互与调试打印时换算为 mΩ、µH、mWb、r/s 等显示单位。

## 1. 总览

| 物理量 | 内部单位 | 说明 |
| --- | --- | --- |
| 相电流 / dq 电流 | A（安培） | 由 ADC 原始值乘以电流传感系数得到 |
| 母线电压 | V（伏特） | 由 ADC 原始值乘以电压分压系数得到 |
| 调制量 mod_d/mod_q/mod_alpha/mod_beta | p.u.（标幺值） | 相对母线电压归一化，实际电压 = mod × Vbus / 1.5 |
| 电气角度 | rad（弧度） | 归一化到 [0, 2π) |
| 机械角度 | rad（弧度） | 圈数 × 2π |
| 电气/机械角速度 | rad/s | 电气 = 机械 × 极对数 |
| 时间 / 控制周期 | s（秒） | Current_Ts = 50 µs 等 |
| 相电阻 R | Ω（欧姆） | Product 设计值；辨识只做验收，显示时为 mΩ |
| dq 电感 Ld/Lq | H（亨利） | Product 设计值，显示时为 µH |
| 永磁磁链 ψ | Wb（韦伯，V·s） | Product 设计值，显示时为 mWb |
| 温度 | °C | 由具体温度 Adapter 换算；当前为 MCU 内部温度 |

## 2. 电流单位与换算

产品构造入口是 `Firmware/Core/Config/product_catalog.c` 中的 `ProductBoardDesign.current_sense`。配置为每个物理通道显式给出 role、polarity、endpoint、正的 `current_a_per_count`、默认 offset 和有效范围；通道顺序或 ADC Rank 不再隐含相位。

当前 VectorMiniSt 只启用低侧三分流：

| 项目 | 当前值 |
| --- | ---: |
| nominal shunt | 6 mΩ |
| 三通道 scale | 0.0134310134 A/count |
| 三通道 polarity | `INVERTED` |
| 默认 offset | 2048 / 2048 / 2048 |
| 有效 offset 范围 | 1948..2148 |
| 可靠量程 | 20 A |
| 命令/标定上限 | 10 A / 10 A |
| 软件过流阈值 | 18 A |
| 电机默认标定/运行限流 | 3 A / 6 A |
| 相电阻功率路径补偿 | 0.008 Ω |

12 位 ADC、3.3 V、增益 10、6 mΩ 的设计关系为：

```
SENSING_CURR_FACTOR = 3.3 / 4095 / 10 / 0.006 ≈ 0.01343 A/LSB
```

`Firmware/Core/Application/MotorControl/measurement_runtime.c` 和 `Firmware/Core/Services/Measurement/phase_current_strategy.c` 按显式极性换算。当前三相 polarity 都是 inverted，所以等价于：

```c
phase_a_current_a = -((int16_t)ADC值 - phase_a_offset_adc) * current_a_per_adc_count;  // A
```

因此 `calibration_current`、`current_limit`、`Id/Iq` 参考与反馈、`Ia/Ib/Ic`、`Ibus` 全部以 **A** 为单位。schema 10 中的 shunt 字段只作为已部署 ABI/兼容身份保留；Flash 不恢复电流比例、标定电流或运行限流，三相 offset 是唯一恢复的电流测量个体量。

## 3. 电压单位与换算

### 3.1 母线电压

```c
#define VBUS_R1  10.0f   /* kΩ */
#define VBUS_R2  1.0f    /* kΩ */
#define SENSING_VBUS_FACTOR (3.3f / 4095.0f * (VBUS_R1 + VBUS_R2) / VBUS_R2)
```

```
SENSING_VBUS_FACTOR = 3.3 / 4095 × 11 ≈ 0.008866 V/LSB
CurrentControl->Vbus = ADC值 × SENSING_VBUS_FACTOR;   // V
```

母线比例和保护阈值来自当前 ProductConfig：约 `0.0088644689 V/count`，Vbus > 30 V 报过压，< 10 V 报欠压。测量/保护编排位于 `Firmware/Core/Application/MotorControl/measurement_runtime.c`，纯测量模型位于 `Firmware/Core/Services/Measurement/measurement_model.c`。

### 3.2 调制量（p.u.）与实际电压的关系

`Firmware/Core/Application/MotorControl/current_control_runtime.c` 中，电压给定先归一化为标幺值调制量：

```c
V_to_mod = 1.5f / CurrentControl->Vbus_filt;   // V → p.u.
CurrentControl->mod_d = V_to_mod * Vd_set;
CurrentControl->mod_q = V_to_mod * Vq_set;
```

反向关系即：

```
V_phase = mod × Vbus / 1.5        // p.u. → V
```

1.5 是工程上的归一化基准，调制矢量被限幅在 `0.95 × √3/2 ≈ 0.823`。磁链观测器正是用该关系把调制量还原为真实相电压：

```c
// Firmware/Core/Application/MotorControl/sensorless_runtime.c
float mod_to_V = CurrentControl->Vbus_filt / 1.5f;
Fluxobserver->Ualpha = CurrentControl->mod_alpha * mod_to_V;  // V
Fluxobserver->Ubeta  = CurrentControl->mod_beta  * mod_to_V;  // V
```

## 4. 角度与角速度单位

编码器输出换算见 `Firmware/Core/Services/RotorFeedback/encoder.c`：

```c
Encoder->theta_elec = normalizeAngle((interpolated_enc * _2PI * pole_pairs) / cpr);  // rad，电气角 [0, 2π)
Encoder->vel_elec   = Encoder->vel * _2PI * pole_pairs;                              // rad/s，电气角速度
Encoder->theta_mech = Encoder->pos * _2PI;                                           // rad，机械角
Encoder->vel_mech   = Encoder->vel * _2PI;                                           // rad/s，机械角速度
```

换算关系：

```
θe = θm × 极对数
ωe = ωm × 极对数
1 圈 = 2π rad
```

当前电机设计极对数为 21。速度/位置相关参数内部均为 rad/s 或 rad；默认速度随构建变体变化：

```
motor speed design limit = 6.2 × 2π rad/s
default speed limit      = 6.2 × 2π（无阻尼）或 0.5 × 2π（阻尼）rad/s
speedAcc/Dec = 50  × 2π  rad/s²
pos_maxspeed = 0.125 × 2π（无阻尼）或 0.5 × 2π（阻尼）rad/s
posAcc/Dec   = 0.125 × 2π  rad/s²
```

位置模式的 `pos_Kp`、`pos_Kd`、`pos_Ki` 分别使用 A/rad、A/(rad/s)、A/(rad*s)，详细控制结构和整定方法见 [position_impedance_control.md](position_impedance_control.md)。

> 注意：USB/CAN 接口上速度使用 **rev/s（圈/秒）**，位置使用 **圈（r）**，进入固件时乘 2π 转为 rad/s 与 rad；调试打印 `spd=xx r/s`、`pos=xx r` 也是圈单位。

## 5. 电机模型参数单位（标定）

当前固件仅启用 `SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION`，由 `Firmware/Core/Application/MotorControl/phase_resistance_runtime.c` 驱动 `Firmware/Core/Services/Identification/phase_resistance.c`。协议动作号 4（完整 R/L/磁链辨识）明确返回不支持，不运行 L/磁链的一体化辨识。

### 5.1 相电阻 R（Ω）

```
R = (V_phase / I_phase) × 2/3 − PATH_COMPENSATION
```

`2/3` 用于从“单相通电 + 另外两相并联回流”的等效电阻折算到相电阻；当前 6 mΩ 硬件 entry 的 `PATH_COMPENSATION` 为 0.008 Ω。单位 **Ω**，打印为 mΩ。若新增硬件档位，应建立新的 BoardDesign/variant 并重新验证，不能在线切换这一设计值。

### 5.2 电感 Ld/Lq（H，配置模型）

```
L = (V − R·I) / (ωe·I) × 2.25
```

该公式仅说明历史参数的量纲关系；当前固件不会运行该辨识步骤。`d_axis_inductance_h`、`q_axis_inductance_h` 始终由活动 `ProductMotorDesign` 提供，单位 **H**，显示为 µH，Flash 不覆盖它们。

### 5.3 磁链 ψ（Wb，配置模型）

```
ψ = (|V| − R·|I|) / ωe − L·|I|
```

量纲推导：`(V − Ω·A) / (rad/s) − H·A = V·s = Wb`。当前固件不执行磁链辨识；`flux_weber` 来自已验证的配置，单位 **Wb**，显示为 mWb。

## 6. 磁链观测器量纲自洽性

`Firmware/Core/Application/MotorControl/sensorless_runtime.c` 的磁链观测器各中间量量纲如下：

```c
y1 = -Rs·Iα + Uα                        // V（电压）
η1 = x1_last − Ls·Iα                     // Wb（磁链误差）
φerr = ψ² − (η1² + η2²)                  // Wb²
x1 += Ts·(y1 + γ·η1·φerr)                // Wb（积分，Ts 单位 s）
cos = (x1 − Ls·Iα) / ψ                   // 无量纲
θe  = atan2(sin, cos) + π                // rad（电气角）
ωe  = Δθ / Ts                            // rad/s（电气角速度，经低通滤波）
```

当前 ProductConfig 的投影增益 γ = 800000，量纲为 `1/(Wb²·s)`，使 `γ·η·φerr` 与 y 同为 V。

## 7. 上位机接口单位对照

调试打印（`Firmware/Core/Communication/Interfaces/interface_usb.c`）确认的单位：

| 打印项 | 单位 | 内部存储 |
| --- | --- | --- |
| vbus | V | V |
| ia / ib / ic / id / iq / ibus | A | A |
| spd1/2_filt | r/s | 内部控制用 rad/s |
| pos1/2_filt | r | 内部控制用 rad |
| temp | °C | °C；当前为 MCU internal monitor-only |
| Rs | mΩ | Ω |
| Ld / Lq | µH | H |
| Flux | mWb | Wb |

## 8. 关键换算公式速查

```
I [A]     = polarity_sign × (ADC − Offset) × current_a_per_count
Vbus [V]  = ADC × 3.3 / 4095 × 11
V_phase   = mod × Vbus / 1.5
θe [rad]  = count / cpr × 2π × pole_pairs      （归一化到 [0, 2π)）
θm [rad]  = pos[turns] × 2π
ωe [rad/s] = ωm × pole_pairs
speed_ref(接口 rev/s) → 内部 × 2π → rad/s
pos_ref(接口 r)      → 内部 × 2π → rad

R  [Ω]  = (V/I) × 2/3 − PATH_COMPENSATION
L  [H]  = (V − R·I) / (ωe·I) × 2.25
ψ  [Wb] = (|V| − R·|I|) / ωe − L·|I|
```

## 9. 控制周期

| 周期 | 值 | 用途 |
| --- | --- | --- |
| Current_Ts | 50 µs（20 kHz） | 电流环 / 观测器积分 |
| Speed_Ts | 500 µs（2 kHz） | 速度环 / 编码器通用测速 |
| Position_Ts | 1 ms（1 kHz） | 位置阻抗环 / 轨迹发生器 |
