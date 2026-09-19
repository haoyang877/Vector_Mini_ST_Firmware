# RTT 4 通道遥测

帧格式：上行 channel 1，标称 2 kHz，每帧 4 个有符号 `int16_t`
（8 字节），使用 `SEGGER_RTT_MODE_NO_BLOCK_SKIP` 非阻塞发送。调试器来不及读取时宁可丢帧，
也不阻塞 20 kHz 快环。帧不含时间戳；量化单位不代表实际测量精度。

## 通道布局

| 通道 | 信号 | 换算 | 来源 |
|---|---|---|---|
| data0 | 机械位置 | raw × 180/32768 °（Q15 单圈） | `OnBoard_Encoder.theta_mech`，折到 [-180°, 180°) |
| data1 | 机械转速 | raw / 10 rpm | `OnBoard_Encoder.vel_mech` |
| data2 | Iq 指令 | raw / 1000 A | 限幅后的 `MotorControl.iqRef` |
| data3 | Iq 反馈 | raw / 1000 A | FOC 实测 `FOC.Iq` |

- 位置只表示单圈角度，超出 ±180° 直接折返，不承诺多圈绝对位置。
- 转速量程 ±3276.7 rpm，电流量程 ±32.767 A；超量程饱和而不回绕。
- 非有限值（NaN/Inf）编码为 0；尺度无效时同样输出 0。
- JScope 描述符为 `JScope_i2i2i2i2`，定义在 `hw_conf.h` 的 `RTT_JSCOPE_DESCRIPTOR`，
  由 `main.c` 的 `SEGGER_RTT_ConfigUpBuffer(1, ...)` 应用。

## 分频与错峰

- `RTT_SAMPLE_RATE_HZ`（默认 2000）必须整除 `FOC_FREQ`；分频计数在 `rtt_telemetry.c` 内维护。
- 与 2 kHz 位置伺服同拍的帧延后一个 50 μs 快周期，随后恢复标称节拍，不延迟电流/PWM 更新。

## 配套工具与更新义务

- 波形工程：`pro_lks.lksscope`（通用）、
  `pro_lks_calibration.lksscope`（通用面板，保留 `rttFreq=2000`）。
  （原 HIL 专用 `pro_lks_servo_hil.lksscope` 与 `tools/bench/servo_hil_run.py` 采集链已随
  HIL 台架于 2026-09-19 退役删除。）
- 修改字段数量、顺序或类型时，必须同步：本文档、JScope 描述符、两个 `.lksscope` 工程、
  `tests/integration/test_rtt_control_telemetry.py` 以及
  `tests/unit/native/run_position_servo_tests.py` 的 RTT 夹具。

## 历史格式

2026-09 之前存在 12 通道模式 3 伺服帧与 8 路标定帧两种布局，携带状态字、前馈/保持电流
和标定阶段等字段。旧分析链（`tools/analysis/rtt_control_frame.py`、
`tools/bench/run_mode3_validation.py` 及其配套脚本）只适用于按旧格式采集的历史数据，
不再与当前固件配套，其中 HIL 采集与分析脚本已随 2026-09-19 HIL 退役删除；
历史结论与波形见 `docs/reports/`。
