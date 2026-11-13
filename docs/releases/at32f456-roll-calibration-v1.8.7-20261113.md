# APP1.8.7 ROLL 标定速度环候选

适用 AT32F456 产品固件；沿用 Loader2.3.4，发布类别为 debug。
ROLL 的 LUT 标定速度 Kp/Ki 从 0.5/1.0 改为 0.05/0.5，与左右轮一致。
5 A 标定限流、5 A 对齐、4.9 A 启动 Iq、0.5 A 启动 Id、15 rad/s 和10圈保持原值。
其他轴、正常运行 PI、通信协议和 schema14/OEN3 参数布局不变。

标定速度 PI 来自固件产品预设，旧板无需为本项调整重写参数；已有较低标定电流仍保留。
整机 FULL HEX 只包含 Loader 与 APP，不包含参数和身份页；已有板更新应保留这两区。
本次不自动擦除、烧录或转动电机。带 1.5～2 N·m 阻尼轴的实际启动、稳速和标定精度待台架验证。

构建和验证证据归档于 `outputs/roll-calibration-v1.8.7-20261113/` 与
`outputs/runs/roll-calibration-release-v1.8.7-20261113/`，以实际 summary.json 为准。
