# 五台电机总体参数表

版本：v0.6，2026-10-09。本表记录用户确认的**目标配置**，并在下方单独记录实机应用状态；它不是五台板卡的同步 Flash 回读。运动范围统一用 °，速度统一用 °/s；速度限值按正反方向相同的绝对值列出。左右轮上限 20 rad/s ≈ 1145.9 °/s ≈ 190.99 RPM。

| 电机 | CAN ID | 极对数 / 采样电阻 | 阻尼环 / 摩擦前馈目标 | 正转 / 反转前馈电流 | 运行 / 标定电流目标 | 速度模式上限目标 | 位置运动范围 | 位置模式最高速度 | 速度环 Kp / Ki | 位置环 Kp / Kd | 位置环速度滤波目标 |
| --- | ---: | --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: |
| 左轮 | 1 | 21 / 6 mΩ | 无 / 关闭 | 0 / 0 A | 6 / 3 A | ±1145.9 °/s | 连续旋转 | — | 0.06 / 0.6 | — | — |
| 右轮 | 2 | 21 / 6 mΩ | 无 / 关闭 | 0 / 0 A | 6 / 3 A | ±1145.9 °/s | 连续旋转 | — | 0.06 / 0.6 | — | — |
| Roll | 3 | 21 / 6 mΩ | 有 / 开启 | 2.946 / 3.259 A | 8 / 5 A | ±180 °/s | −150°～+150° | 90 °/s | 0.5 / 1.0 | 8 / 2 | 5 Hz |
| Pitch | 4 | 21 / 6 mΩ | 有 / 开启 | 3.455 / 3.101 A | 8 / 5 A | ±180 °/s | −17.19°～+51.57° | 45 °/s | 0.5 / 1.0 | 8 / 2 | 5 Hz |
| Yaw | 5 | 21 / 6 mΩ | 有 / 开启 | 3.103 / 2.653 A | 8 / 5 A | ±180 °/s | −90°～+90° | 28.65 °/s | 0.5 / 1.0 | 8 / 2 | 5 Hz |

速度模式上限与位置模式最高速度是不同限制。三个关节的摩擦前馈值为各自的正、反方向库仑电流幅值；速度相关前馈项均未配置。编码器 LUT、电零位和机械零位属于每台电机的个体标定，不能从本表复制到另一台板卡。

## 已存值与待应用项

- 2026-10-09 当前左轮板 UID 前缀 `18004A` 原保存节点 2，已按用户要求将 Flash 节点改为 1；仅节点字段两字节改变，完整 Flash 回读与候选一致，复位后再次回读一致。保留原固件、LUT、电零位 Q15=15655、方向 0、标志 3，以及 21 极对、6 mΩ、运行/标定 6/3 A、速度上限 1145.9 °/s、速度 PI 0.06/0.6。轴记录仍为空，未复制其他板的标定；本轮未启动电机或测试 CAN。最终 CPU 运行、三相关闭、探针已释放。证据：`outputs/identity_check_20261009/write_left_node_result.json` 和 `finish_result.json`。此板与历史左轮的只读记录分别保留。
- 2026-10-09 当前右轮板 UID 前缀 `500051` 已下载普通 APP，保存节点 2、21 极对、6 mΩ、运行/标定 6/3 A、速度上限 1145.9 °/s、速度 PI 0.06/0.6，无阻尼/摩擦前馈。mode 13/15 完成，电零位 Q15=12605、方向 0、标志 3；完整 Flash 保存和复位回读一致，最终 mode 0、无故障、三相关闭。轮电机使用本版本兼容的空轴记录，由节点 2 表示右轮角色；AXS2 配置版本 0 不能使能，未当作可运行的轮轴记录保存。首轮 USB-CAN 通道 0 查询未收到回复；用户交换 CAN-H/CAN-L 后，一次 `0x209` 查询正确返回节点 2，往返 3.502 ms、错误计数 0，1M/1M 基本收发通过，未作长时通信验收。本轮未做速度闭环或机械零位验收。此板与以下 `2D0047` 历史右轮不是同一 UID。证据：`outputs/right_wheel_commission_20261009/summary.json`，执行说明见[本轮计划](../plans/2026-10-09-right-wheel-commissioning.md)。
- 历史左轮只读速度上限为 180 °/s，该记录不能作为上述 `18004A` 当前板的配置。右轮 UID `2D0047…` 已恢复到 20 rad/s，整片 Flash 回读一致；该次 J-Link 架空空载速度试验达到 +19.98/−21.32 rad/s（位置差测速），采样相电流峰值 0.484 A，结束时 mode 0、故障 0、三相关断。反向位置差测速偏差约 6.6%；USB-CAN 通道 0 尚无有效收发，不能据此认定 CAN 链路通过。两轮固件的参数加载上限均为 20 rad/s。
- Roll 与 Yaw 的 5 A 标定电流、阻尼环和摩擦前馈已有保存/回读记录。本表不把历史运动试验解释为全范围验收通过。
- 本表 Pitch 对应 UID 前缀 `4A004E`。该板最近只读标定电流为 3 A、阻尼环开启而摩擦前馈关闭；本机辨识得到 3.455/3.101 A，曾在 RAM 中开启摩擦前馈和 5 Hz 滤波测试，但未持久化。5 A 标定电流、前馈开启及 5 Hz 滤波均为待写入目标。此前另一个 UID 前缀 `2D0049` 的 Pitch 曾保存 2.946/3.259 A，后来同一板卡被配置为 Roll，不能作为 `4A004E` 的当前配置。
- 修改任何板卡 Flash 前，按该板 UID 和实际连接身份核对备份、编码器个体标定、目标字段及停机/关相状态；写后完整回读并复位核验。当前 J-Link 连接的右轮不代表其他四台已更新。

本机证据：`outputs/left_wheel_identification_20260930/parameter_inspection.json`、`outputs/right_wheel_identification_20261008/current_config_read_only.json`、`outputs/right_wheel_identification_20261008/right_wheel_20rad_flash_result.json`、`outputs/right_wheel_identification_20261008/speed_test_20rad_result.json`、`outputs/right_wheel_identification_20261008/can_after_line_swap_20261008.json`、`outputs/right_wheel_identification_20261008/right_wheel_forced_can_tx_20261008.json`、`outputs/roll_filter5_persistence_20260929/summary.json`、`outputs/new_damped_motor_calibration_20260928/friction_compensation_summary.json`、`outputs/pitch_position_recheck_20260929/friction5_position_test/result.json`、`outputs/pitch_parameters_flash_20260930/inspection.json`、`outputs/yaw_filter5_limit90_20260930/verify_result.json`。`outputs/` 为忽略目录，历史证据不代替连接板卡的现时回读。
