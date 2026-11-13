# CAN-Link-Test / APP v1.8.3边界修复包

2026-11-13，APP冻结`f9f87718a095fb08ee4621adbd9e0d0e4e827720`，工具清单
`34ff494c567e6d1d1adbb1d29839411523bd0962`，分支`the-system-refactor`。
APP v1.8.3修复30 rad/s精确边界误拒绝，GUI v1.8.3内置配套固件，Loader仍v2.3.4。
单独更新GUI不能修复旧APP的浮点换算。

`outputs/deliveries/CAN-Link-Test-v1.8.3-win64-20261113.zip`：15,739,150字节、24文件，
SHA-256：`914a5b1b6466472ac8f03c71346087bc3400fbc1f3da2407c38084e32b312580`。
EXE：`d416de41a81a1acaca026bdb0f3249c5477d4adcfe9d2b469b352b0eab9a5168`。
FULL HEX：`578caceb0d7b68e6449c660f4209380eabd093d0b2dfd58d1d2d44b32b73c602`。
CAN image：`3c2e99705aefc81dc9510c45befbe550a2b4871e9179fc4bc1ec59c9b509f4a5`；
CAN manifest：`a7d22d2a3a6615818500601e00a9c63a4656efab5e619d92a85dd36dee357069`。

包含工具、使用指南、FULL/CAN固件、当前Release摘要和实机原始帧/CSV/回归日志。
ZIP CRC及逐文件SHA-256均复核，原v1.8.2交付不覆盖。
当前节点6已升级并完成±30受理、±30.001拒绝及十项硬件检测；限值和标定保留。
其他型号和原Hall异常角区未作本轮验收，发布类别保持debug。
详见[实测报告](../reports/2026-11/speed_target_boundary_20261113.md)和
[使用指南](../guides/firmware_test_calibration.md)。
