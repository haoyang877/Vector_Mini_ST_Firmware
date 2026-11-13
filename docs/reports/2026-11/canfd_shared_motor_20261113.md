# CAN FD v1.10.0 共用电机与页面顺序

2026-11-13，`the-system-refactor`，实现`a6c6412a`，冻结源码
`0276346468370717e337853396794a07bb34d166`。APP1.8.4/Loader2.3.4不变。

六页依次为J-Link烧录、硬件测试、通信测试、角度标定、升级压测、速度控制。
各页共用顶部电机，识别已配板或烧录成功后继承节点/类型/UID；空板初装明确选择
一次类型，只接CAN可扫描。身份属于当前进程，重启或换板后需重新识别。
多轴查询、五轴验收和批量升级保留，样机号和测试号仍独立。

新增10项GUI回归通过；原升级回归按命名页面和显式批量模式更新后28项通过。
首次Release的旧测试假设失败记录保留在`outputs/runs/canfd-shared-motor-v1.10.0-20261113/`。
最终干净Release全部通过（256单元、21集成、原生、架构/文档、Keil及发布清单），证据为
`outputs/runs/canfd-shared-motor-v1.10.0-final-20261113/summary.json`和同目录
`release-manifest.json`。实际窗口确认六页顺序、用户扫描YAW节点7及标定页共用身份。
本轮未执行烧录或电机运动；烧录结果联动及多板批量操作经离线回归验证。

交付`outputs/deliveries/CAN-Link-Test-v1.10.0-APP-v1.8.4-win64-20261113.zip`，
43,366,905字节，ZIP CRC与内文件哈希通过。

| 产物 | SHA-256 |
| --- | --- |
| ZIP | `0a2940369f9b4b557a0c3a2d7d8a955c2fbc8f0052b114604b0e1442e0ac7ed5` |
| EXE | `b7f9fe0a0b40c71d5aa0b54018ea0577199951e49ec6117d8853061348eb62f4` |
| 原FULL HEX | `461eee1ac173c293ecb3f69d8a76652addf9a9cb1e2967b10e32126e8e1983aa` |
| 原CAN image | `d5668f423b4d62950b38b11c13730e3f71847cb5cea87cfef077f002f7c5534f` |
| 原CAN manifest | `6681fc8b4b30a66f28e7a673f909cab60c0cc2ae2b1627c2d973bbbeb5009b12` |

固件保持原debug类别及独立证据，不扩大上版台架结论。回退运行v1.9.3即可，无需重烧。
详见[操作指南](../../guides/canfd_motor_operations.md)。
