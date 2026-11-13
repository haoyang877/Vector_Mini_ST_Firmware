# CAN-Link-Test v1.8.1 交付

2026-11-13，分支`the-system-refactor`，代码基线
`02eb56e9e53005d864f323057d06e1e4f432c133`。
新增平铺角度标定按钮、独立速度控制页和500 ms标定请求期限。
固件仍为APP v1.8.2 / Loader v2.3.4 debug，不覆盖此前固件和GUI交付包。

产物：`outputs/deliveries/CAN-Link-Test-v1.8.1-win64-20261113.zip`，15,670,749字节、
16个文件；SHA-256：
`d46880b638a422b993ea3c3440777bdf9148c4565c363475c568a05ecc645968`。
EXE SHA-256：`5397e6f2c82409d90b6767f47a161c9dc44c04d15647ff1c4cd2b1f92da99c02`。

包含单文件Windows x64工具、独立使用指南、原v1.8.2 FULL HEX和CAN升级资源、
当前Release摘要、标定/速度/实际GUI工作进程摘要、原始帧、速度CSV和曲线。
manifest逐文件记录长度与SHA-256；ZIP CRC和逐文件SHA-256均复核通过。
内嵌FULL HEX SHA-256保持
`276e6ea7229f361a22da00c0c38410bc8bee2a2607c527088a740fb89163dcdc`。

当前干净源码Release档通过，节点6标定保存及低速正反向控制、关相通过；
保留最终布局未视觉复核、速度纹波及其他型号未测范围，详见
[验收报告](../reports/2026-11/canfd_calibration_speed_20261113.md)。
新页面的操作步骤见[现行指南](../guides/canfd_motor_operations.md)。
