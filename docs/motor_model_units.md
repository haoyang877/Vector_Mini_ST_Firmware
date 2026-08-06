# 电机模型计算单位说明

本文档整理 Vector_Mini_ST_Firmware 工程中 FOC 与电机模型计算所使用的物理单位及换算关系。工程内部模型统一使用 **SI 单位（浮点）**，仅在 USB/CAN 上位机交互与调试打印时换算为 mΩ、µH、mWb、r/s 等显示单位。

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
| 相电阻 R | Ω（欧姆） | 标定得到，打印时显示 mΩ |
| dq 电感 Ld/Lq | H（亨利） | 标定得到，打印时显示 µH |
| 永磁磁链 ψ | Wb（韦伯，V·s） | 标定得到，打印时显示 mWb |
| 温度 | °C | NTC 换算 |

## 2. 电流单位与换算

定义见 `Bsp/hw_conf.h`：

```c
#define SENSING_RES        0.002f   /* 采样电阻 Ω */
#define CURRENT_AMP_GAIN   10.0f    /* 电流放大增益 V/A */
#define SENSING_CURR_FACTOR (3.3f / 4095.0f / CURRENT_AMP_GAIN / SENSING_RES)
```

12 位 ADC 满量程 3.3 V，换算系数：

```
SENSING_CURR_FACTOR = 3.3 / 4095 / 10 / 0.002 ≈ 0.0403 A/LSB
```

`Foc/foc_sensing.c` 中：

```c
FOC->Ia = -((int16_t)ADC值 - A_Offset) * SENSING_CURR_FACTOR;  // A
```

因此 `calib_current`、`current_limit`、`idRef/iqRef`、`Ia/Ib/Ic/Id/Iq`、`Ibus` 全部以 **A** 为单位。默认 `calib_current = 10 A`，`current_limit = 30 A`。

## 3. 电压单位与换算

### 3.1 母线电压

```c
#define VBUS_R1  10.0f   /* kΩ */
#define VBUS_R2  1.0f    /* kΩ */
#define SENSING_VBUS_FACTOR (3.3f / 4095.0f * (VBUS_R1 + VBUS_R2) / VBUS_R2)
```

```
SENSING_VBUS_FACTOR = 3.3 / 4095 × 11 ≈ 0.008866 V/LSB
FOC->Vbus = ADC值 × SENSING_VBUS_FACTOR;   // V
```

过压/欠压保护阈值（`Foc/foc_sensing.c`）同样以 V 为单位：Vbus > 30 V 报过压，< 10 V 报欠压。

### 3.2 调制量（p.u.）与实际电压的关系

`Foc/foc_algorithm.c` 中，电压给定先归一化为标幺值调制量：

```c
V_to_mod = 1.5f / FOC->Vbus_filt;   // V → p.u.
FOC->mod_d = V_to_mod * Vd_set;
FOC->mod_q = V_to_mod * Vq_set;
```

反向关系即：

```
V_phase = mod × Vbus / 1.5        // p.u. → V
```

1.5 是工程上的归一化基准，调制矢量被限幅在 `0.95 × √3/2 ≈ 0.823`。磁链观测器正是用该关系把调制量还原为真实相电压：

```c
// Foc/foc_sensorless.c
float mod_to_V = FOC->Vbus_filt / 1.5f;
Fluxobserver->Ualpha = FOC->mod_alpha * mod_to_V;  // V
Fluxobserver->Ubeta  = FOC->mod_beta  * mod_to_V;  // V
```

## 4. 角度与角速度单位

编码器输出换算见 `Bsp/encoder.c`：

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

默认极对数为 21。速度/位置相关参数内部均为 rad/s 或 rad：

```
speed_limit  = 200 × 2π  rad/s
speedAcc/Dec = 50  × 2π  rad/s²
pos_maxspeed = 5   × 2π  rad/s
posAcc/Dec   = 10  × 2π  rad/s²
```

> 注意：USB/CAN 接口上速度使用 **rev/s（圈/秒）**，位置使用 **圈（r）**，进入固件时乘 2π 转为 rad/s 与 rad；调试打印 `spd=xx r/s`、`pos=xx r` 也是圈单位。

## 5. 电机模型参数单位（标定）

标定流程 `Task_Calib_R_L_Flux`（`Foc/foc_calibration.c`）把电流、电压、角度三个量纲串成电机参数：

### 5.1 相电阻 R（Ω）

```
R = (V_phase / I_phase) × 2/3 − 0.004
```

`2/3` 用于从“单相通电 + 另外两相并联回流”的等效电阻折算到相电阻；`0.004 Ω` 是 MOSFET 导通电阻 + 采样电阻 + 线路电阻的补偿。单位 **Ω**，打印为 mΩ。

### 5.2 电感 Ld/Lq（H）

```
L = (V − R·I) / (ωe·I) × 2.25
```

其中注入电频率 `ωe = 2π × 1000 rad/s`（电气），`2.25` 为标定经验修正系数。单位 **H**，打印为 µH。

### 5.3 磁链 ψ（Wb）

```
ψ = (|V| − R·|I|) / ωe − L·|I|
```

量纲推导：`(V − Ω·A) / (rad/s) − H·A = V·s = Wb`。单位 **Wb**，打印为 mWb。

## 6. 磁链观测器量纲自洽性

`Foc/foc_sensorless.c` 的磁链观测器各中间量量纲如下：

```c
y1 = -Rs·Iα + Uα                        // V（电压）
η1 = x1_last − Ls·Iα                     // Wb（磁链误差）
φerr = ψ² − (η1² + η2²)                  // Wb²
x1 += Ts·(y1 + γ·η1·φerr)                // Wb（积分，Ts 单位 s）
cos = (x1 − Ls·Iα) / ψ                   // 无量纲
θe  = atan2(sin, cos) + π                // rad（电气角）
ωe  = Δθ / Ts                            // rad/s（电气角速度，经低通滤波）
```

投影增益 γ = 1e9，量纲为 `1/(Wb²·s)`，使 `γ·η·φerr` 与 y 同为 V。

## 7. 上位机接口单位对照

调试打印（`Communication/interface_usb.c`）确认的单位：

| 打印项 | 单位 | 内部存储 |
| --- | --- | --- |
| vbus | V | V |
| ia / ib / ic / id / iq / ibus | A | A |
| spd1/2_filt | r/s | 内部控制用 rad/s |
| pos1/2_filt | r | 内部控制用 rad |
| temp | °C | °C |
| Rs | mΩ | Ω |
| Ld / Lq | µH | H |
| Flux | mWb | Wb |

## 8. 关键换算公式速查

```
I [A]     = (ADC − Offset) × 3.3 / 4095 / 10 / 0.002
Vbus [V]  = ADC × 3.3 / 4095 × 11
V_phase   = mod × Vbus / 1.5
θe [rad]  = count / cpr × 2π × pole_pairs      （归一化到 [0, 2π)）
θm [rad]  = pos[turns] × 2π
ωe [rad/s] = ωm × pole_pairs
speed_ref(接口 rev/s) → 内部 × 2π → rad/s
pos_ref(接口 r)      → 内部 × 2π → rad

R  [Ω]  = (V/I) × 2/3 − 0.004
L  [H]  = (V − R·I) / (ωe·I) × 2.25
ψ  [Wb] = (|V| − R·|I|) / ωe − L·|I|
```

## 9. 控制周期

| 周期 | 值 | 用途 |
| --- | --- | --- |
| Current_Ts | 50 µs（20 kHz） | 电流环 / 观测器积分 |
| Speed_Ts | 100 µs | 速度环 |
| Position_Ts | 200 µs | 位置环 |
