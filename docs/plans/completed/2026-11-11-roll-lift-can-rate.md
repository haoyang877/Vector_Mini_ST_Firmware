# ROLL、Lift 的 CAN FD 数据段

该 2 Mbit/s 试验版本已撤回，当前六轴固件统一使用 1 Mbit/s 仲裁、5 Mbit/s 数据。以下为试验记录。

原节点使用 1 Mbit/s 仲裁、5 Mbit/s 数据。APP v1.6.7、Loader v2.2.2 和工装 v1.4.2 对 ROLL（0x05）和 Lift（0x08）选择 2 Mbit/s 数据段，仲裁仍为 1 Mbit/s。参数版本为 v1.2.0。

按现场指定的 ROLL 验收。SWD 写入前备份 256 KiB Flash，写入后全片回读匹配，16 KiB 参数区逐字节一致。APP 运行后 CAN1 时序寄存器回读为 ACTIME 0x2C2C0085、FDTIME 0x15150042、分频 1。通过 CAN1 检查 UID、APP CRC 和空闲无故障状态，再以 50 Hz 连续发送 1,000 次状态查询，得到 1,000 次有效应答。此次仅实测 ROLL 通信，Lift 的新速率已通过寄存器模型验证。原片备份可用于恢复。

[六轴通信设置](../../guides/at32f456_five_axes.md) · [工装使用说明](../../guides/motor_commission.md)
