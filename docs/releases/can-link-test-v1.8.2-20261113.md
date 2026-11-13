# CAN-Link-Test v1.8.2 交付

2026-11-13，源码`bdaf5d6d91326109a48db32bbc0ba3d39b33e046`，分支`the-system-refactor`。
新增可配置速度上限，默认30 rad/s；驱动拒绝先核对板端中止原因，保留原始帧。
固件仍为APP v1.8.2 / Loader v2.3.4，检测阈值不变。

交付：`outputs/deliveries/CAN-Link-Test-v1.8.2-win64-20261113.zip`，
15,547,049字节、19文件，SHA-256：
`8372fe87178a454cf33f0458d70fec9709813b7115c433084794977e21307811`。
EXE SHA-256：`e4fc44513972958f8010ec5080a953684449af1c29c473a5ede1c5f7233764d2`。
ZIP CRC与逐文件SHA-256均核对通过，不覆盖v1.8.1交付。

包内包含工具、使用指南、原FULL HEX/CAN升级资源及本次台架和Release证据。
当前PITCH保存的速度上限和轴包络已按要求设为30 rad/s，标定和PI保留；
GUI普通速度配置只修改RAM，不自动修改其他板的轴包络或保存Flash。

Release通过，实测30→12→30限值回读及±5 rad/s运行/停止通过；未测30 rad/s满速。
硬件检测复测九项正常、Hall动态仍异常，保留原诊断，不声明全部硬件通过。
完整范围见[验收报告](../reports/2026-11/canfd_abort_speed_limit_20261113.md)，
操作见[现行指南](../guides/canfd_motor_operations.md)。
