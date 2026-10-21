> 历史归档（2026-10-21）：依赖退役模块、旧工具或旧节点策略；历史测量与限制保留。 本文的“当前”指原始基线；现行入口见[当前说明](../../architecture/software_architecture.md)。归档不新增执行授权。

# RTT 标定遥测（已合并）

原 8 路标定帧已在 2026-09-19 与 12 路伺服帧合并为统一的
[4 通道 RTT 遥测](rtt_control_telemetry.md)。固件不再输出标定专用帧，
`rtt_calibration_dropped_frames` 计数器与配套波形通道已移除。

本文保留以维持历史链接；当前帧布局、换算与配套工程以
[4 通道 RTT 遥测](rtt_control_telemetry.md) 为准。
