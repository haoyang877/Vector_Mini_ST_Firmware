# MotorBench 台架功能退役记录

2026-11-03 按用户要求删除 `APP_ENABLE_MOTOR_BENCH`、MotorBench 模块、
私有 CAN `0x7F00` 命令、专用采集工具及 `ESC-APP-AT32F456-Commissioning` 工程。
原 LIMITS、RUN、CAPTURE、INFO、READ 和 KEEPALIVE 操作不再提供；
该模块对普通启动附加的电流、速度、行程、时限与租期监护也已移除。

当前使用普通 [Keil 工作区](keil_workspace.md)中的 APP 工程。
通信入口见 [实时控制协议](../protocols/README.md#control)，
标定入口见 [标定请求接口](motor_calibration_request.md)。Keil全局命令也已删除。
正常控制及核心故障保护、独立 BSP 底层测试工程继续保留。

以下仅为历史资料，不能作为当前固件操作指令：

- [原私有协议 v1](../protocols/README.md#implementation)及 [v1.1](../protocols/README.md#implementation)。
- [原测试记录](../reports/2026-10/unified_motor_validation_20261031.md)及 [续测记录](../reports/2026-10/unified_motor_continued_20261031.md)。
- [删除范围与验证](../plans/completed/2026-11-03-motor-bench-removal-v1.md)。
