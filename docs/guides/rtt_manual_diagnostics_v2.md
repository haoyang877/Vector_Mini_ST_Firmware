# 手工位置调试 RTT 通道 v2

> 本页保留 v2 历史布局。当前固件和同名 Scope 使用
> [v3 布局](rtt_manual_diagnostics_v3.md)，以位置误差替换 Vq 并重新排序；
> v2 采集记录应按本页解码。

版本：v2，2026-10-10。普通固件仍使用 profile 2、RTT 上行通道 1、2 kHz，
每帧 10 个小端 int16、20 字节。配套
`tools/bench/scopes/pro_lks_manual_diagnostics.lksscope`。

按位置、速度、电流、电压整理顺序：

| data | 观测量 | 每计数单位 | 数据来源 |
|---:|---|---|---|
| 0 | 最终位置目标 | 0.01° | MotorControl.posRef |
| 1 | 机械位置反馈 | 0.01° | OnBoard_Encoder.theta_mech |
| 2 | 位置环速度参考 | 0.01°/s | 外环快照 speed_command |
| 3 | 控制滤波速度反馈 | 0.01°/s | 外环快照 speed_feedback |
| 4 | 编码器 PLL 速度 | 0.01°/s | OnBoard_Encoder.vel_mech |
| 5 | Iq 参考 | 1 mA | MotorControl.iqRef |
| 6 | Iq 反馈 | 1 mA | FOC.Iq |
| 7 | Id 反馈 | 1 mA | FOC.Id |
| 8 | 摩擦前馈电流 | 1 mA | 外环快照 friction_feedforward_current |
| 9 | Vq 实际输出 | 1 mV | FOC.Vq |

data2 是送入速度 PI 的实际参考，包含轨迹速度、位置误差及速度误差校正，
并经过速度指令限幅；不是外部 speedRef，也不是纯轨迹速度。
参考与反馈可直接用 data2/data3 对比，PLL 与滤波测速差异可用 data4/data3 对比。
只改变 RTT 观测，不改变控制计算、增益、滤波、Flash 或 CAN 协议。

mode 3 且外环快照有效时，data2、3、8 来自最近一次外环快照，不承诺与
当拍 ADC 完全同步。其他模式、停机或无有效快照时，data2/8 清零，data3
退回 `pos_vel_filtered`，可能保留旧值。Vd 仍参与控制，仅从本诊断档移除。

Scope 默认显示原始计数、公式为空。位置/速度除以 100 得 °/°每秒；电流/电压
除以 1000 得 A/V。原始范围为 −32768～32767，位置和速度约 ±327.67°/°每秒；
20 rad/s 的轮电机测速仍会饱和。非有限数输出 0，有限数向零截断并饱和。
仍只写一次完整帧，缓冲满不阻塞、不重试；累计丢帧读取
`rtt_manual_dropped_frames`，不占波形通道。

兼容性：帧长、描述符和 profile 编号未变，但 data2、4～9 的含义/顺序变更，
帧内没有布局版本标志，不能自动识别旧镜像。固件、AXF、Scope 和解码配置必须
配套使用 v2；旧记录按 [v1](rtt_manual_diagnostics_v1.md) 解码。
profile 1 的 11 路标定档与 HIL 布局保持原样。

黄金向量：`tests/vectors/rtt_manual_diagnostics_v2.json`；实际 C 编码器与 Scope
一致性检查：`tests/unit/native/test_rtt_manual_diagnostics.py`。本次没有下载到板，
板上固件不会自动切换到 v2 布局。
