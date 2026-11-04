# Keil全局命令已删除

2026-11-03按用户要求删除正式APP中的可写调试邮箱、启停/目标/标定/保存命令及其前台分派。
旧指南中的命令1至5不再提供，旧AXF符号不能作为当前固件的操作入口。

- 只读观察见[Keil Watch指南](watch_motor_commands.md)。
- 正式通信控制见[实时控制协议](../protocols/README.md#control)。
- 三类标定请求见[标定请求接口](motor_calibration_request.md)。

运行层标定保存实现仍保留，但当前APP未绑定对外保存请求；不会自动保存标定结果。
删除范围与验证见[计划记录](../plans/completed/2026-11-03-app-debug-removal-v1.md)。
