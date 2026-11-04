# 公共协议代码

本目录提供 APP、Loader 和主机共用的纯编解码实现；不拥有硬件或业务状态。
帧、CRC、CAN ID、分片和升级字段统一见[通信协议](../../docs/protocols/README.md)。
回归夹具在 `tests/fixtures/protocol/`。APP 实现位于 `firmware/communication/` 的
`yg_protocol.c` 和 `yg_protocol_boot.c`，接口统一在 `yg_protocol.h`。
