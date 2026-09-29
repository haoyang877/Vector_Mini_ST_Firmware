# PC 上位机入口与实际实现范围

当前目录只有 `examples/company_canfd_position.c`：公司 CAN FD v1.2.3 帧编解码的
独立 C99 示例，配套[说明与黄金向量](../docs/protocols/canfd_application_example.md)。
它是现有离线夹具的真实输入，不是已完成的 GUI、设备服务或升级客户端。

现场主机工具位于 `tools/bench/`，离线分析工具位于 `tools/analysis/`。
`host_app/src/`、GUI、统一设备服务和升级客户端尚未实现，不以规划目录冒充现状。
设备 UART、USB CDC 与旧 YI 协议不属于当前产品入口。

后续主机产品应消费单份 [shared 契约](../shared/README.md)、
[当前参数线路契约](../docs/protocols/yg_protocol_parameter_wire_v1.md) 和
[升级线路 v2](../docs/protocols/yg_protocol_upgrade_wire_v2.md)，保持 CAN FD 传输与
业务状态分离。升级主机负责目标匹配、事务 nonce、镜像 CRC32、重试和 APP 身份核对；
这些职责当前仅是待实现边界，不能宣称已经接通。

虚拟环境、构建缓存和历史打包产物不是产品实现，生成结果归 `outputs/`。
