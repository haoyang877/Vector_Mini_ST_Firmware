# YAW 标定减振 / APP1.8.4 与 CAN-Link-Test1.9.3

2026-11-13，`the-system-refactor`，冻结源码`2eb47246bda8fc4f771d9ddf7f152c77093c8c03`。
APP v1.8.4仅将YAW标定速度Kp由0.048降至0.012；Ki、电流PI、运行PI和阈值保持。
GUI v1.9.3内置新固件，并清理只读J-Link连接留下的复位捕获；Loader仍v2.3.4。

交付`outputs/deliveries/CAN-Link-Test-v1.9.3-APP-v1.8.4-win64-20261113.zip`，49,138,773字节。

- 整包SHA-256：`a79a958fde5d7759a088d28fdbcadb092fea47ed18bc83e075a71caf84fad7c7`。
- EXE SHA-256：`661e869edc793d4d684661cff4ba7b81ae34f9e5aeb40bd73ac27e0f4879ec21`。
- FULL HEX：`461eee1ac173c293ecb3f69d8a76652addf9a9cb1e2967b10e32126e8e1983aa`。
- CAN image：`d5668f423b4d62950b38b11c13730e3f71847cb5cea87cfef077f002f7c5534f`。
- CAN manifest：`6681fc8b4b30a66f28e7a673f909cab60c0cc2ae2b1627c2d973bbbeb5009b12`。
- 对照数据`outputs/deliveries/YAW-calibration-comparison-20261113.zip`：`4d94224d7dad1256335a1bf75d12f3e1979f387f82a063c4f2ea996edb5d1747`。

Release PASS：`outputs/runs/yaw-calibration-v1.8.4-final-20261113/summary.json`；ZIP和全部文件哈希复核通过。
节点7原版、低Kp和APP1.8.4三轮对照完成，正式版标定/保存/关相通过，速度标准差
9.867→1.103 rad/s、Iq标准差0.5910→0.0499 A。升级前后原参数与保护区相同，
重标定后再次升级仍保留最新参数。实测范围为当前空载YQ-5708，其他电机和负载未新增
实测，发布类别保持debug。完整原码与配置见数据包。

详见[诊断报告](../reports/2026-11/yaw_calibration_oscillation_20261113.md)和
[使用指南](../guides/canfd_motor_operations.md)。
