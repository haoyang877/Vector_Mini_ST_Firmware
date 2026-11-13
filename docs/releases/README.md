# 历史交付与候选记录

- [CAN-Link-Test v1.11.0](can-link-test-v1.11.0-20261113.md)：APP/整片Flash显式擦除、备份及回读。

- [CAN-Link-Test v1.10.3](can-link-test-v1.10.3-20261113.md)：J-Link重复写入和板端版本展示。

- [CAN-Link-Test v1.10.2](can-link-test-v1.10.2-20261113.md)：硬件检测明确显示命令/编码器反向原因。

- [CAN-Link-Test v1.10.1](can-link-test-v1.10.1-20261113.md)：故障类型、清除按钮及状态确认，节点6只读诊断。

- [CAN-Link-Test v1.10.0](can-link-test-v1.10.0-20261113.md)：共用电机选择、六页重排，APP1.8.4未改。

- [APP1.8.4 / CAN-Link-Test1.9.3](can-link-test-v1.9.3-app-v1.8.4-20261113.md)：YAW标定减振、J-Link复位捕获清理及配套数据包。

- [CAN-Link-Test v1.9.2](can-link-test-v1.9.2-20261113.md)：标定曲线、Hall原始波形和双编号归档，配套APP1.8.3不变。

这些记录保留其原版本的发布/验证状态，本目录没有自动推断“最新正式发布”的机制。
当前目标版本从shared/targets读取，正式发布仍须匹配当前summary和产物哈希。

- [CAN-Link-Test / APP v1.8.3](can-link-test-v1.8.3-20261113.md)：30 rad/s精确边界修复，配套固件和工具包，当前节点6实测通过。

- [CAN-Link-Test v1.8.2](can-link-test-v1.8.2-20261113.md)：速度上限默认30且可配置，区分板端中止和续租拒绝；节点6限值回读与±5 rad/s通过，Hall异常保留。

- [CAN-Link-Test v1.8.1](can-link-test-v1.8.1-20261113.md)：平铺标定按钮、独立速度控制及500 ms标定期限，保留原APP1.8.2/Loader2.3.4和独立台架证据。

- [AT32F456 v1.8.2 / CAN FD v1.7.4 首答修复调试包](at32f456-v1.8.2-debug-20261113.md)：当前PITCH首答修复，10000查询、30静态及一次3 A双向零超时；默认全项与参数已填，Hall独立异常保留。

- [AT32F456 v1.8.1 / CAN FD v1.7.1 调试交付](at32f456-v1.8.1-debug-20261113.md)：电机命令/编码器方向通过，完整 GUI 验证；剩余霍尔动态与 CAN 首答可靠性异常保留。

- [AT32F456 v1.8.0 调试交付](at32f456-v1.8.0-debug-20261113.md)：固件与工具组合包、SHA-256；八项静态通过，旋转动态未通过，类别 debug。

- [APP1.1.0候选](architecture-app-v1.1.0-candidate.md)
- [APP1.2.0候选](architecture-app-v1.2.0-candidate.md)
- [APP1.3.0候选](architecture-app-v1.3.0-candidate.md)
- [轮电机2mΩ交付记录](wheel-motor-2mohm-no-damper-v1.0.0.md)

当前规则见[发布标准](../guides/firmware_release_standard.md)，当前证据缺口见[计划索引](../plans/active/README.md)。
