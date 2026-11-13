# CAN-Link-Test v1.10.1

2026-11-13，`the-system-refactor`，源码`93b82a4e1b308d92c35cd5c42e82f443d561ac5f`。
增加共享电机故障类型解析、显式清除按钮与状态回读，保留所有应答、拒绝和超时记录。
六页顺序及共享身份沿用v1.10.0。APP1.8.4 / Loader2.3.4未修改，固件仍为debug。

干净Release通过，当前summary位于`outputs/runs/canfd-fault-clear-v1.10.1-20261113/`。
交付`outputs/deliveries/CAN-Link-Test-v1.10.1-APP-v1.8.4-win64-20261113.zip`，
43,373,128字节，SHA-256为`84b4d055b898c725012be5e71c9446507c516cc53eed38ea819119de40194f40`。
ZIP CRC及文件哈希复核通过。工具已启动，用户Esc停止界面自动验证；实际清错和运动未执行。
节点6的J-Link只读诊断指向标定方向相反，完整证据和限制见
[报告](../reports/2026-11/canfd_fault_clear_encoder_20261113.md)，操作见
[指南](../guides/canfd_motor_operations.md)。
