# 公司统一帧公共编解码 v1

APP 与独立 Loader 复用本目录的唯一实现。迁移说明见
[公共编解码迁移 v1](../../docs/protocols/yg_protocol_shared_codec_migration_v1.md)。

| 文件 | 职责 |
|---|---|
| `yg_protocol_wire_types.h` | 16B 头、46B 单帧载荷上限、结果码与消息视图 |
| `yg_protocol_crc.c/.h` | 公司 CRC8 与 CRC16 |
| `yg_protocol_frame_codec.c/.h` | 显式小端字段编码、长度与双 CRC 校验 |
| `yg_protocol_can_id.c/.h` | 29 位扩展 ID 编解码，PF 固定为 0xEF |
| `yg_protocol_fragment.c/.h` | 调用方提供缓冲区的单会话分片重组 |

无 HAL、APP 状态、Loader 状态、动态分配、队列或外设初始化。
消息视图引用输入缓冲区；只有在其生命周期内才可使用载荷指针。
头文件名称、函数签名和线上字节保持不变，不保留旧目录副本或转发包装。

业务路由、权限、Flash、物理发送及复位由各产品拥有。本目录返回编解码结果，
不意味着参数已应用、块已持久化、回复已发送或升级已成功。
