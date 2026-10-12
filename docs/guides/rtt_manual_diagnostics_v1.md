# 手工位置调试 RTT 通道 v1

> 本页保留旧版布局。2026-10-10 起普通诊断档使用
> [v3 通道布局](rtt_manual_diagnostics_v3.md)，删除电压并加入速度参考和位置误差；
> 当前同名 Scope 已更新为 v3，不能用于解码 v1 镜像或采集记录。

普通固件默认 `RTT_TELEMETRY_PROFILE = RTT_TELEMETRY_MANUAL_DIAGNOSTICS`（2）。
上行通道 1，2 kHz，10 个小端 int16/帧，20 字节，净载荷 40 kB/s。
配套 `tools/bench/scopes/pro_lks_manual_diagnostics.lksscope`，分位置、测速、电流、电压四组。
仅只读遥测，不改变控制参数、滤波、限流、标定或 Flash 参数。

| data | 观测量 | 每计数单位 |
|---:|---|---|
| 0 | 最终位置目标 | 0.01° |
| 1 | 机械位置反馈 | 0.01° |
| 2 | 编码器速度（当前配置为 PLL） | 0.01°/s |
| 3 | 实际控制速度反馈（含 HOLD 滤波） | 0.01°/s |
| 4 | Iq 参考 | 1 mA |
| 5 | Iq 反馈 | 1 mA |
| 6 | Id 反馈 | 1 mA |
| 7 | Vq 实际输出（限幅后） | 1 mV |
| 8 | Vd 实际输出（限幅后） | 1 mV |
| 9 | 实际摩擦前馈电流 | 1 mA |

Scope 显示原始计数。例如位置 500 表示 5°，Iq 5000 表示 5 A。
mode 3 的 3/9 来自最近一次外环快照，不承诺与本次 ADC 完全同刻。
停机、其他模式或快照无效时，3 取 `pos_vel_filtered`，9 清零；停机速度可能保留旧值。
位置/速度量程约 ±327.67° / ±327.67°/s；超范围饱和，非有限数输出 0，向零截断。
每次仅写一次完整帧，缓冲区满时不阻塞、不重试。

需要时通过 Scope 代码变量读取模式/故障、母线/利用率、三相电流：
`MotorControl.ModeNow/ErrorNow`、`FOC.Vbus_filt/duty/Ia/Ib/Ic`。
累计丢帧为 `rtt_manual_dropped_frames`，模 2^32，不占 RTT 波形通道。
代码变量为 SWD 独立读取，不与 RTT 同步。
AXF 路径相对于 Scope 文件；若未解析相对路径，在代码变量中选择本次 AXF。
不要沿用旧固件 RAM 地址。

兼容性：此 10 路布局与旧 11/20 路不同，必须匹配新版描述符与 Scope。
回退时编译指定 `RTT_TELEMETRY_PROFILE=1` 并使用 `pro_lks.lksscope`。
本次不处理既有 HIL 布局差异，HIL 不是验收目标。
黄金向量：`tests/vectors/rtt_manual_diagnostics_v1.json`。
实际编码器离线测试：`tests/unit/native/test_rtt_manual_diagnostics.py`。

当前阶段只配置及离线构建，不下载或启动电机；板上旧固件不会自动出现新增通道。
