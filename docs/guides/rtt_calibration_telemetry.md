# 8 路电机标定 RTT 输出

打开 `tools/bench/scopes/pro_lks_calibration.lksscope`，加载匹配板内固件的
`outputs/build/keil/Vector_Mini_ST/Vector_Mini_ST.axf`。
波形按角度、速度、电流、状态分为四组，显示原始计数。

| 通道 | 信号 | 物理量换算 |
|---|---|---|
| data0 | 编码器电角度 `OnBoard_Encoder.theta_elec` | raw × 180/32768 ° |
| data1 | 观测器电角度 `Fluxobserver.theta_e` | raw × 180/32768 ° |
| data2 | 编码器机械转速 `OnBoard_Encoder.vel_mech` | raw / 10 rpm |
| data3 | 观测器机械转速 `Fluxobserver.omega_e / motor_pole_pairs` | raw / 10 rpm |
| data4 | Iq 指令 `MotorControl.iqRef` | raw / 1000 A |
| data5 | Iq 反馈 `FOC.Iq` | raw / 1000 A |
| data6 | 标定阶段 `CalibStep` | 枚举原值 |
| data7 | 无感启动状态 `SensorlessStartup.state` | 枚举原值 |

RTT 上行通道 1，小端有符号 int16，每帧 8 项、16 字节，描述符为
`JScope_i2i2i2i2i2i2i2i2`。目标 2 kHz，带宽由原 64 kB/s 降为 32 kB/s。
继续使用 2048 字节缓冲、非阻塞 SKIP 和快环错峰发送，所有模式均输出实际字段。
本格式与旧 16 路标定帧、12 路位置伺服帧不兼容，必须重新打开配套波形工程。

模式、故障码、母线电压与方向保留在工程的“电机调试变量”面板，低频查看；
累计丢帧计数 `rtt_calibration_dropped_frames` 也在此面板。计数只由发送失败递增，
不会因读取而清零，复位后从 0 开始；8 路波形不再携带状态字或丢帧标志。
工程使用 4 MHz SWD、5 ms RTT 读取间隔。只能连接一个 RTT 读端。
后续重新构建若地址变化，应将工程 RTT 地址更新为新 AXF 中的 `_SEGGER_RTT`。

角度单圈显示在 [-180°,180°)，跨界跳变正常；未完成电零位标定时，编码器电角度
不保证与观测器相位对齐。转速量程约 ±3276.7 rpm，电流量程约 ±32.767 A，
超量程饱和而不回绕；非有限值编码为 0，极对数无效时观测器转速编码为 0。
量化分辨率不是测量精度；帧不含时间戳，不能由帧序号证明没有丢样。

mode 13 常用阶段：22 对齐，23 等待闭环，24 等待速度稳定，25 寻找原点，
26 采集，27 构建 LUT，28 验证，29 减速，30 电流退出；0 为空闲。
启动状态：0 空闲，1 对齐，2 开环，3 速度锁定，4 相位交接，5 闭环。

普通固件默认 `RTT_TELEMETRY_PROFILE=RTT_TELEMETRY_CALIBRATION`；HIL 默认仍使用
`RTT_TELEMETRY_SERVO` 对应的 12 路位置帧。配置在 `hw_conf.h`，可由编译参数覆盖。
帧布局编译时固定，不随运行模式切换。采样代码不修改方向、控制参数或运行模式。

离线验证：`python tests/unit/native/test_rtt_calibration.py --cc <zig.exe或C编译器>`。
