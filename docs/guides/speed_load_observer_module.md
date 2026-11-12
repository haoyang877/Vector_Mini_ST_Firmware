# 连续负载观测与摩擦前馈接入指南

[负载观测器接口](../../firmware/motor/control/speed_load_observer.h)及[实现](../../firmware/motor/control/speed_load_observer.c)
根据实测电流和机械速度估计剩余负载，输出连续补偿电流。
[摩擦前馈接口](../../firmware/motor/control/speed_friction_feedforward.h)及[实现](../../firmware/motor/control/speed_friction_feedforward.c)
根据规划后的速度目标计算平滑摩擦补偿。两个模块仅依赖标准 C 库，不依赖测速器、PI、板级驱动或配置宏。
使用方将所需实现文件加入构建，分配状态，并接入自己的控制流程；只添加源文件不会自动启用补偿。

本文以当前方案 **2 kHz 速度环、20 Hz OBS、连续补偿与摩擦前馈同时工作** 为控制参考。
速度可以来自任意估计器，但必须和速度 PI 同源、同单位、同时间快照。该方案的反馈配置为
20 kHz KF600 后接 15 点整数均值，不再叠加测速陷波或额外速度低通；OBS 自身的低通保留。
惯性前馈关闭。本文的参数和调用顺序定义控制结构，不依赖额外记录或调试工具。

## 输入、状态及职责

| 数据 | 单位和要求 | 所有者 |
| --- | --- | --- |
| `speed_rad_s` | 同一控制请求的连续机械速度，rad/s，无额外零速死区 | 测速/快环发布，PI 与 OBS 共用 |
| `measured_iq_mean_a` | 实际测量 Iq 的快环窗口均值，A，不能使用 Iq 目标代替 | 快环累计并按实际样本数平均 |
| `applied_friction_a` | 发布请求时已经施加的摩擦前馈，A | 电流输出提交处维护 |
| `residual_iq_a` | `measured_iq_mean_a - applied_friction_a` | 调用 OBS 前构造 |
| `current_limit_a` | 当前热保护等约束后的可用电流上限，非负有限 A | 保护/控制层 |
| `SpeedLoadObserver` | 每台电机一份，先复位，单一控制任务更新 | 速度外环 |
| 摩擦上拍输出 | 每台电机一份，首次和停机时为零，单位 A | 速度外环计算、输出提交处确认 |

正 Iq 必须与正机械加速度同向。速度输入不是电角速度；若测量的是电角速度，应先除以极对数。
剩余电流只扣除已施加摩擦，不扣 OBS 自己的补偿，也不扣刚计算但尚未输出的新摩擦。
摩擦禁用时，其已施加值为零，剩余电流等于实际 Iq 均值。

## 一个控制周期的调用顺序

1. 20 kHz 快环累计实测 Iq，正常每 10 拍产生一次 2 kHz 请求。先求 Iq 均值，再与当时的滤波速度和已施加摩擦一起快照。
   电流均值代表一个窗口，速度代表窗口末端；保持该采样口径，不另加一整拍缓存，也不改为只取瞬时 Iq。
2. 控制任务消费完整请求，计算 `residual_iq_a`，先更新 OBS，得到本拍负载补偿。
3. 将速度命令按加减速限制推进到本拍规划目标，再用该目标更新摩擦前馈。摩擦模型输入不是实测速度。
4. 用本拍 OBS 与新摩擦之和收窄 PI 输出区间，运行 PI，再合成和限幅 Iq 目标。
5. 将 Iq 目标、PI 状态和新摩擦作为同一份结果提交，快环在确认仍处于同一有效运行状态后使用。
   此时才把新摩擦登记为已施加值，供后续请求使用。

慢环计算可在独立任务或适当的控制中断中执行，但每个请求只能推进一次算法状态，且必须满足 0.5 ms 周期。
若采用异步任务，启停或模式切换时给请求标记新的运行代次，并丢弃旧代次的结果；复位与更新不能并发。
电流窗口缺失、结果过期、非有限测量或算法返回失败时，走控制层故障处理，不把失败当作有效零补偿继续运行。

## OBS 参数与初始化

令 T 为速度环周期，ω=2πf；系数在初始化时预计算，不必每拍重复计算。

| 参数 | 参考值 | 含义 |
| --- | --- | --- |
| 速度环/OBS 频率 | 2000 Hz | T=0.0005 s |
| 低通频率 f | 20 Hz | 同时作用于速度和剩余电流 |
| J/Kt | 0.00068 A·s²/rad | 惯量与转矩常数之比；须与电机及传动的机械速度坐标一致 |
| `alpha` | 约 0.0591173974 | Tω/(1+Tω) |
| `derivative_gain` | 约 0.0854513202 A/(rad/s) | (J/Kt)ω |
| `gain` | 0.75 | 负载估计到补偿的比例 |
| 补偿变化率 | 200 A/s | 每拍最大变化 0.1 A |
| `maximum_step_a` | 0.1 A | 变化率乘 T |
| `limit_fraction` | 0.5 | 补偿幅值不超过可用电流上限的 50% |

J/Kt 是模型参数，不是单独的惯量 J。更换电机、传动或速度坐标时需要重新确认该比值，不能把参考值作为所有电机的常数。
所有配置字段必须有限且满足头文件范围；模块将系数视为可信配置，由调用方在启动前验证。

```c
#include "speed_load_observer.h"

static SpeedLoadObserver s_observer;
/* 以固定 2 kHz 时基预计算；也可在初始化函数中按公式生成。 */
static const SpeedLoadObserverCoefficients s_coefficients = {
    .alpha = 0.0591173974f,
    .derivative_gain = 0.0854513202f,
    .gain = 0.75f,
    .maximum_step_a = 0.1f,
    .limit_fraction = 0.5f
};

/* 初始化或停机复位时执行。 */
SpeedLoadObserver_Reset(&s_observer);

/* 每个速度环周期执行；输入来自同一请求快照。 */
float residual_iq_a = measured_iq_mean_a - applied_friction_a;
bool valid = SpeedLoadObserver_Update(&s_observer, &s_coefficients,
    feedback_rad_s, residual_iq_a, available_current_a);
/* valid 为 true 时消费 s_observer.compensation_a，否则处理控制故障。 */
```

模块按下式更新，其中 `v_f` 和 `i_f` 是滤波状态，`v_f[k]` 是本拍已更新的值：

```text
v_f[k] = v_f[k-1] + alpha × (v[k] - v_f[k-1])
i_f[k] = i_f[k-1] + alpha × (residual_iq[k] - i_f[k-1])
load[k] = i_f[k] - derivative_gain × (v[k] - v_f[k])
desired = clamp(gain × load[k], ±limit_fraction × available_current)
compensation += clamp(desired - compensation, ±maximum_step_a)
compensation = clamp(compensation, ±limit_fraction × available_current)
```

首拍速度滤波状态对齐输入，电流滤波状态从零开始；电流上限降低时立即重新限幅，优先于变化率约束。
20 Hz 的时间常数约 7.96 ms，另有补偿变化率约束；降低测速延迟不会消除这些动态。
改变调用频率时必须同时重算 `alpha` 和 `maximum_step_a`，并调整 PI 周期和电流窗口长度；不能只改变任务频率。

## 摩擦前馈参数与调用

| 参数 | 参考值 | 含义 |
| --- | --- | --- |
| `scale` | 0.5 | 总补偿比例 |
| `coulomb_a` | 0.3633 A | 库仑摩擦项，乘比例前 |
| `viscous_a_per_rad_s` | 0.000679 A/(rad/s) | 粘性摩擦项，乘比例前 |
| `zero_width_rad_s` | 0.5 rad/s | 零速过渡宽度 |
| `time_constant_s` | 0.08 s | 平滑时间常数 |
| `slew_a_per_s` | 2 A/s | 2 kHz 下每拍最大变化 0.001 A |
| `limit_a` | 0.5 A | 还受可用电流上限的 50% 约束 |

```c
#include "speed_friction_feedforward.h"

static const SpeedFrictionFeedforwardConfig s_friction = {
    .scale = 0.5f, .coulomb_a = 0.3633f,
    .viscous_a_per_rad_s = 0.000679f, .zero_width_rad_s = 0.5f,
    .time_constant_s = 0.08f, .slew_a_per_s = 2.0f, .limit_a = 0.5f
};

/* 每个速度环周期，在 OBS 之后调用；applied_friction_a 初始为零。 */
float next_friction_a = SpeedFrictionFeedforward_Update(&s_friction,
    applied_friction_a, ramped_target_rad_s, 0.0005f, available_current_a);
/* 先用 isfinite 检查，再合成输出；NaN 表示失败，不应写入下一拍状态。 */
```

原始补偿为 `scale × (coulomb × tanh(target/zero_width) + viscous × target)`。
实现用 17 点表线性插值近似 tanh，先限幅，再按 `T/(time_constant+0.5T)` 平滑并限制变化率，最后重新限幅。
接口周期范围为 (0,0.001] s。参数必须满足头文件约束；若禁用该前馈，调用方停止更新并将已施加状态清零。

## PI、斜坡和总电流合成

参考配置为加减速限制均 20 rad/s²、惯性前馈关闭、摩擦前馈开启。
归一化 PI 使用配置电流上限 Icfg=6 A、Kp=0.045 s/rad、Ki=1.0 rad⁻¹；
其等效电流单位增益为 Kp=0.27 A/(rad/s)、Ki=6 A/rad。这些参数只在此说明，模块不会修改现有 PI。

若 PI 直接输出 A，使用电流单位增益并直接与前馈相加；不要再乘 Icfg。
若 PI 输出归一化值，则保持下述结构。Icfg 必须为正，当前可用上限 Iavail 满足 0≤Iavail≤Icfg：

```text
ff = 本拍摩擦前馈 + 本拍OBS补偿
Umin = (-Iavail - ff) / Icfg
Umax = (+Iavail - ff) / Icfg
error = 规划后的目标速度 - 同请求的反馈速度
P = Kp × error
I_candidate = I + Ki × error × T
若 P+I_candidate 在 [Umin,Umax] 内，或误差使输出脱离饱和，则接受 I_candidate
u_pi = clamp(P + I, Umin, Umax)
Iq_target = clamp(u_pi × Icfg + ff, -Iavail, Iavail)
u_applied = (Iq_target - ff) / Icfg
I = clamp(I + u_applied - u_pi, Umin, Umax)
```

“误差使输出脱离饱和”指候选输出高于 Umax 且 error<0，或低于 Umin 且 error>0。
以上同时给出条件积分和输出跟踪；OBS 不直接清除或释放 PI 积分。
前馈必须参与 PI 的可用输出区间，不能在已经饱和的 PI 后简单追加补偿。
快环提交时若 Iavail 进一步降低，应重新限幅 Iq，并使用同一 ff 对 PI 执行输出跟踪，再更新输出状态。

Icfg 是增益的缩放基准，热降额只改变 Iavail；不能把 Iavail 用来替换乘数而意外改变 PI 增益。
若调整 Icfg 且希望电流单位增益不变，应设置 Kp=0.27/Icfg、Ki=6/Icfg，并重新检查可用电流余量。
转动惯量、摩擦或电流环动态不同的电机仍需按相同单位整定，参考参数并不保证所有电机响应相同。

## 启停、旁路与故障处理

初始化、停机、故障、控制模式改变或重新使能时，由单一控制上下文复位 OBS、摩擦状态和电流窗口，
同时按控制器启动策略复位 PI 与规划目标。未获得有效采样窗口前，不发布负载补偿。
若使用异步任务，停机前发出的结果不能在重新使能后被接受。

OBS 旁路时令补偿为零并复位其状态，速度 PI 与摩擦前馈仍可独立运行；重新启用从复位状态开始。
模块不包含在线开关或无扰切换逻辑，如需运行中切换，调用方必须另外实现输出过渡。
摩擦和 OBS 可以分别使用；不使用摩擦时，OBS 的输入就是实测 Iq 窗口均值。

复现上述控制响应需要同时保持反馈通路、时基、Iq 口径、模型参数、PI 增益单位及抗饱和顺序。
增加额外滤波、改变窗口对齐或在低优先级任务中延迟提交，都会改变补偿与 PI 的相位关系。
