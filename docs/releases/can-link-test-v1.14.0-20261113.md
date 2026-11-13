# CAN-Link-Test v1.14.0

新增顶部实际APP/Loader版本与状态摘要，以及位置/速度/Iq参考和反馈实时曲线。
速度模式无效的位置参考显示缺失，原始CSV保存全部有效点及有效位。
固件、控制和限值未改。运行/停止行为继续使用原运动会话。

最终Release：`outputs/runs/live-speed-display-final-v1.14.0-20261113/summary.json`，
干净源码365052f8，各检查通过。布局及数据回归见[报告](../reports/2026-11/live_speed_display_20261113.md)。
本次未启动电机或烧录；由用户实机测试。

交付目录：`outputs/deliveries/CAN-Link-Test-v1.14.0-win64-20261113`。
目录含可独立运行的EXE、使用说明、Release证据、文件清单和哈希。
EXE仍内置APP1.8.6 / Loader2.3.4，使用新界面无需为此刷固件。

- EXE SHA-256：`a8b0a3d08b21e622574c1f2c748b6116b8e353088428fe02f56a6663ab0462a2`。
- ZIP：`outputs/deliveries/CAN-Link-Test-v1.14.0-win64-20261113.zip`，43,091,239字节。
- ZIP SHA-256：`4b185305c059c1f7b78e507709e8af19d3ac81c547da2574fc517f2eb7dc0967`。

包内逐文件哈希及ZIP CRC通过。原用户固件、测试、Keil/示波/VS Code配置另存后原样恢复。
