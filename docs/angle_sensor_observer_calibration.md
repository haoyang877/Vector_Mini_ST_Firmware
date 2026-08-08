# 基于角度观测器的角度传感器线性化标定设计

## 1. 背景与目标

原线性化标定（`Calib_EncoderOffset`，mode=5）采用开环电压注入：

- 以固定电角速度（π rad/s，约 0.5 Hz 电气频率）旋转 d 轴电压矢量
- 以注入相角 `phase_set` 作为“真实角度”参考，采样编码器 raw 并建立误差表

问题：注入相角不是转子真实角度，二者相差负载相关的转矩角（slip / torque angle），这部分误差会进入 LUT。

目标：改用磁链观测器（反电动势法）估计的转子电角度作为参考，消除转矩角误差；电机参数（Rs/Ld/Lq/flux）已改为外部填入，观测器所需参数齐备。

## 2. 原理

- 磁链观测器（`Fluxobserver_Update`）基于占空比重建电压 + 实测电流，用 Rs、Ls=(Ld+Lq)/2、flux 估计转子磁链角度 `theta_e`。
- 标定时电机由开环电压注入驱动（与旧方法相同的驱动方式），但把注入频率提高到约 20 Hz 电气频率，使反电动势足够大、观测器可稳定跟踪。
- 采样参考从 `phase_set` 换成观测器累计电角度 `theta_cum`（对 `theta_e` 解卷绕累加）。
- 编码器计数参考：`count_ref = theta_cum * cpr / (2π * 极对数)`，与旧方法公式一致，只是参考来源不同。
- 误差 `error = raw - count_ref`，正反转各采一遍平均，FIR 平滑后生成 128 点 LUT，流程与旧方法一致（`Encoder_Calib_Finalize` 复用）。

## 3. 状态机（新增模式 Calib_EncoderObserver，mode=13）

| 状态 | 动作 |
| --- | --- |
| CS_DIR_START/LOOP/END | 开环低压判断编码器方向（与旧方法相同） |
| CS_OBS_ALIGN | d 轴电压对齐转子，观测器收敛（1.5 s），校验外部参数（极对数、Rs、flux） |
| CS_OBS_RAMP_CW | 开环电角速度 0 → 20 Hz 斜坡（3 s），电压叠加反电动势前馈保持同步 |
| CS_OBS_SAMPLE_CW | 观测器锁定后，按累计电角度等间隔采样一整圈机械角度 |
| CS_OBS_RAMP_CCW | 减速并反向至 -20 Hz |
| CS_OBS_SAMPLE_CCW | 反向采样，同一位置误差与正向平均 |
| CS_OBS_END | 计算 offset + FIR + 128 点 LUT，置标定标志，存 Flash |

采样按“角度触发”而非“时间触发”：观测器累计角度每越过一个 2π/128 电角度间隔就采一个点，对转速波动不敏感。

## 4. 关键参数

```c
#define OBS_CALIB_ALIGN_TIME  1.5f                  /* 对齐+观测器收敛时间 s */
#define OBS_CALIB_RAMP_TIME   3.0f                  /* 斜坡时间 s */
#define OBS_CALIB_SPEED       (2.0f * _PI * 20.0f)  /* 标定电角速度，约 20 Hz */
#define OBS_CALIB_STEP_ANGLE  (_2PI / 128.0f)       /* 采样电角度间隔 */
```

20 Hz 电气频率下 128×极对数 个采样点约 1 s 完成一圈；对 21 对极电机约 28 rpm 机械转速。

## 5. 误差来源与对策

| 误差来源 | 影响 | 对策 |
| --- | --- | --- |
| motor_flux 不准确 | 观测器角度系统性偏差 | 外部填入准确磁链；标定电流取小（1–2 A） |
| Ls 取 Ld/Lq 均值忽略凸极 | 与电流相关的角度误差 | 小电流标定；必要时扩展为 saliency 观测器 |
| 低速段死区/电压重建误差 | 低速时角度抖动 | 提高标定转速（20 Hz）；可加死区补偿 |
| 观测器初始相位偏移 | 常数偏移 | 被 offset 吸收，不影响 LUT 形状 |

## 6. 上板验证步骤

1. 外部填入极对数、Rs、Ld、Lq、flux（`w_pol`、`w_mrs`、`w_mld`、`w_mlq`、`w_mfx`）。
2. USB 发送 mode=13 进入观测器标定。
3. 验证观测器角度精度（三选一）：
   - 用调试器 watch `Fluxobserver.theta_e` 与 `External_Encoder.theta_elec`，转起来后比较稳态差值；
   - 或临时把 `Fluxobserver.theta_e`（×1000 转 int16）替换 RTT/J-Scope 的一个通道；
   - 或标定完成后直接进闭环，观察电流波动是否明显（LUT 不准会带来周期性电流纹波）。
   - 目标：观测器与编码器电角度稳态误差 < 1–2° 电角度；若偏大，优先校核 flux 准确性、检查标定转速是否足够、考虑死区补偿。
4. 标定完成后读 offset/LUT，观察 LUT 平滑度与幅值（正常应 < 1–2 个计数量级）。
5. 断电重启，确认闭环（mode=1/2/3）可直接进入（标定标志已从 Flash 读回）。

## 7. 回退方案

保留原 `Calib_EncoderOffset`（mode=5）开环低速标定作为回退：低速或观测器不可用场景使用。两种标定共用 LUT 生成与 Flash 存储逻辑。
