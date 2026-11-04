# 删除 MotorBench 专用台架功能 v1

按用户要求删除 APP_ENABLE_MOTOR_BENCH 及其对应实现。范围包括 MotorBench
模块、APP 绑定、快环采集与监护挂接、私有 CAN 0x7F00 命令、专用主机工具和测试、
Keil 工程宏与源文件引用及 SRAM 放置项。用户随后明确要求删除整个 Commissioning 工程，
同时移除两个工作区中的节点、专用工程文件、scatter 和命令行构建选择器。历史协议和实测记录保留并标记退役。

保留普通启停、目标提交、三类标定、Flash 保存、Keil 全局调试入口和核心故障保护。
独立 BSP 台架工程及通用遥测模块不属于本次删除范围。
私有 LIMITS/KEEPALIVE 监护随模块删除，普通协议帧及其结果码不变。

基线为当前工作树及 outputs/at32-startup-state-removal-20261103/builds。
修改前文件备份和后续证据存放于 outputs/motor-bench-removal-20261103。
验收：无生产代码/工程中的功能残留，普通协议回归通过，完整 PR 与冻结 release 通过，
剩余三个正式目标和五个独立台架目标均编译通过；对比正式目标资源和 HEX。
本轮不连接、烧录或驱动硬件，不声称验证实机时序。

回退只恢复本轮备份中的代码、工程项和专用测试/工具，不覆盖之前工作树的修改。

## 完成与验证

MotorBench 模块、专用主机工具和测试、私有 CAN 路由、两个工作区中的 Commissioning 节点、
工程文件、专用 scatter、构建选择器及目标宏均已删除。原协议和实测资料标记为历史记录。
保留普通启停、目标、标定、保存、Keil 全局量以及独立 BSP 测试工程。

最终 PR 通过：43 组原生、124 项单元、16 项集成，格式、lint、布局、架构、接口、文档和卫生检查通过。
证据：outputs/runs/motor-bench-removal-20261103-final-pr/summary.json。
三个正式目标和五个独立台架目标全部 0 错误、0 警告，见 outputs/motor-bench-removal-20261103/builds/
及 bench-builds/。正式目标 HEX 与上一轮基线逐字节一致，代码、常量和 RAM 占用均无变化，
见同目录 resource_comparison.json。复核构建中 AT32 曾因缺失 motor_axis.o 链接失败，
单独重新构建通过；失败摘录保存在 build_retry_record.json，重试日志保存在 at32-retry/。

冻结源码的 release 全部通过，包括 Keil 构建和产物清单校验，结果记录于 outputs/runs/motor-bench-removal-20261103-release/summary.json，
冻结清单位于 outputs/motor-bench-removal-20261103/release-source-manifest.json。
以上均为离线验证；未烧录、未操作电机，不代表实机时序或性能验收。
