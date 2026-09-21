# 8 路电机标定 RTT 输出

打开 `tools/bench/scopes/pro_lks_calibration.lksscope`，加载匹配板内固件的
`outputs/build/keil/Vector_Mini_ST/Vector_Mini_ST.axf`。
波形按转速、电流、状态分为三组，显示原始计数。

| 通道 | 信号 | 物理量换算 |
|---|---|---|
| data0 | 转速参考 `MotorControl.speedShadow` | raw / 10 rpm |
| data1 | 观测器闭环转速反馈 `SensorlessStartup.speed_feedback` | raw / 10 rpm |
| data2 | 编码器机械转速 `OnBoard_Encoder.vel_mech` | raw / 10 rpm |
| data3 | 观测器机械转速 `Fluxobserver.omega_e / motor_pole_pairs` | raw / 10 rpm |
| data4 | Iq 指令 `MotorControl.iqRef` | raw / 1000 A |
| data5 | Iq 反馈 `FOC.Iq` | raw / 1000 A |
| data6 | 标定阶段 `CalibStep` | 枚举原值 |
| data7 | 无感启动状态 `SensorlessStartup.state` | 枚举原值 |

这套通道按模式 13 的排查顺序选取：先看启动状态是否从 1 走到 5，再看观测器
闭环反馈能否追上参考（模式 13 强制 +20 rad/s，稳定判据取 20%），再看 Iq 是否
顶到限流（2.5 Nm 摩擦轴要持续约 4.5 A），最后看标定阶段停在哪一步。
无感启动运行在观测器估计上，因此 data0/data1 就是稳定判据比较的那一对量；
data2 与 data3 相互独立，可区分"观测器认为在转"和"编码器认为在转"。

RTT 上行通道 1，小端有符号 int16，每帧 8 项、16 字节，描述符为
`JScope_i2i2i2i2i2i2i2i2`。目标 2 kHz，带宽由原 64 kB/s 降为 32 kB/s。
继续使用 2048 字节缓冲、非阻塞 SKIP 和快环错峰发送，所有模式均输出实际字段。
本格式与旧 16 路标定帧、12 路位置伺服帧不兼容，必须重新打开配套波形工程。

模式、故障码、母线电压与方向保留在工程的“电机调试变量”面板，低频查看；
累计丢帧计数 `rtt_calibration_dropped_frames` 也在此面板。计数只由发送失败递增，
不会因读取而清零，复位后从 0 开始；8 路波形不再携带状态字或丢帧标志。
工程使用 4 MHz SWD、5 ms RTT 读取间隔。只能连接一个 RTT 读端。
后续重新构建若地址变化，应将工程 RTT 地址更新为新 AXF 中的 `_SEGGER_RTT`。

角度单圈显示已从本帧移除（模式 13 不需要角度曲线，残差门限已覆盖 LUT 质量）。
转速量程约 ±3276.7 rpm，电流量程约 ±32.767 A，超量程饱和而不回绕；
非有限值编码为 0，极对数无效时观测器转速编码为 0。量化分辨率不是测量精度；
帧不含时间戳，不能由帧序号证明没有丢样。

mode 13 阶段：22 对齐原点，23 等待闭环，24 等待速度稳定，25 寻找原点，
26 采集，27 构建 LUT，28 验证，29 减速，30 电流退出；0 为空闲。
mode 5（电压开环）阶段：31 对齐，32 对齐保持，33 升速，34 采集，35 收尾。
启动状态：0 空闲，1 对齐，2 开环，3 速度锁定，4 相位交接，5 闭环。

`Sensorless_Error` 的常见落点：data7 停在 1–2 且 data3 无转速 = 开环拖动没带动转子；
data7 停在 3 = 观测器 5 s 内没锁定；data6 停在 23 = 12 s 内没进闭环；
这些都优先怀疑启动电流不足（2.5 Nm 轴）或 `current_limit` 低于该档启动门槛。
模式 13 的启动门控要求 `current_limit ≥ 6.5 A`，而 Flash 里若已有该档位记录，
加载时不会重写 `current_limit`；面板里的"电流上限"必须是 8 A 才满足该门。

普通固件默认 `RTT_TELEMETRY_PROFILE=RTT_TELEMETRY_CALIBRATION`；HIL 默认仍使用
`RTT_TELEMETRY_SERVO` 对应的 12 路位置帧。配置在 `hw_conf.h`，可由编译参数覆盖。
帧布局编译时固定，不随运行模式切换。采样代码不修改方向、控制参数或运行模式。

离线验证：`python tests/unit/native/test_rtt_calibration.py --cc <zig.exe或C编译器>`。
