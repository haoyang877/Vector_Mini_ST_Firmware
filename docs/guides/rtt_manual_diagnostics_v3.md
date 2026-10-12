# 手工位置调试 RTT 通道 v3

版本：v3，2026-10-10。profile 2、RTT 上行通道 1、2 kHz，10 个小端 int16，
20 字节/帧；配套 `tools/bench/scopes/pro_lks_manual_diagnostics.lksscope`。

| data | 观测量 | 每计数单位 | 数据来源 |
|---:|---|---|---|
| 0 | 最终位置目标 | 0.01° | MotorControl.posRef |
| 1 | 机械位置反馈 | 0.01° | OnBoard_Encoder.theta_mech |
| 2 | 位置误差 | 0.01° | posRef − theta_mech |
| 3 | 位置环速度参考 | 0.01°/s | 外环快照 speed_command |
| 4 | 控制滤波速度反馈 | 0.01°/s | 外环快照 speed_feedback |
| 5 | 编码器测速输出（含零速死区） | 0.01°/s | OnBoard_Encoder.vel_mech |
| 6 | Iq 参考 | 1 mA | MotorControl.iqRef |
| 7 | Iq 反馈 | 1 mA | FOC.Iq |
| 8 | Id 反馈 | 1 mA | FOC.Id |
| 9 | 摩擦前馈电流 | 1 mA | 外环快照 friction_feedforward_current |

位置误差采用最终目标减机械反馈，用于观察目标跟踪及到位偏差；正值表示反馈
小于目标。先对 rad 物理量作差，再按 0.01° 编码，不用已饱和的 data0/data1
计数相减，也不进行 ±180° 周期折返。data3 为含误差校正、限幅后的实际速度 PI
参考；data3含速度负反馈制动项，其与data4反相不能直接解释为被控对象延迟。
data5不是滤波器完整连续输入：当前默认恢复2 kHz、16点/8 ms速度均值，data5保留
8计数零速门限（约5.493°/s）；mode3使用同一均值的无死区连续输出再经控制低通。
历史PLL镜像的data5则有±0.05 rad/s零速死区。测速源是编译配置，帧内不携带来源；
应随采集记录固件版本，不能只按通道名称判定。此次帧布局、比例和黄金向量不变。

mode 3 且外环快照有效时，data3/4/9 来自最近一次外环快照，不保证与当拍 ADC
同刻。其他模式、停机或快照无效时，data3/9 清零，data4 退回
`pos_vel_filtered`，可能保留旧值。位置误差在这些状态仍正常计算。
Vq/Vd 仅从本诊断档移除，控制算法不变，CAN 和 Flash ABI 不变。

Scope 显示原始计数：位置/误差/速度除以 100 得 °/°每秒，电流除以 1000 得 A。
int16 范围 −32768～32767，角度/误差/速度量程约 ±327.67；超出饱和，
非有限数输出 0，有限数向零截断。10 路/20 字节、2 kHz、非阻塞和累计丢帧
`rtt_manual_dropped_frames` 保持不变。

兼容性：与 [v2](rtt_manual_diagnostics_v2.md) 相比，data2～9 全部重排，
但帧长/描述符/profile 编号未变，帧内无版本标志。固件、AXF、Scope 和解码配置
必须使用同一布局；旧采集按 v1/v2 文档及向量解码。标定档和 HIL 布局保持原样。

黄金向量：`tests/vectors/rtt_manual_diagnostics_v3.json`；实际编码器/Scope
检查：`tests/unit/native/test_rtt_manual_diagnostics.py`。本轮只改代码和离线构建，
未下载到板，不声称实机显示已更新。
