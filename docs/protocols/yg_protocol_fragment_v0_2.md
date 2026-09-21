# yg_protocol 分片重组 v0.2

本版本只定义完整业务消息在 CAN FD 数据帧之外的有界分片重组规则。它不改变承载链路，也不提供 ACK 或整消息重传。

## 固定规则

- 单帧 payload 直接交付，不影响在途分片会话。
- FIRST 和 MIDDLE 的 payload 必须恰为 46 字节；LAST 为 1–46 字节。
- FIRST 的 `sequence` 必须为 0，后续片按 1 递增。
- `version`、flags 的低 6 位、`reserved`、源节点、目的节点和消息类型必须保持一致。
- 首片开始后绝对 2000 ms 截止；调用方片间超时也生效，恰好到 deadline 即超时。
- 相同上一片的完整重复片忽略，不重复累加，也不刷新片间超时。
- 跳号、同通道头部冲突、容量不足或超时会终止会话；错误不会修改 `completed`。
- 不同源/目的/类型的分片不混入当前会话，也不破坏当前会话。

## 黄金向量

消息类型 `0x012C`、源 `1`、目的 `2`，payload 为 `A[46] || B[46] || C[3]`：

| 片 | flags | sequence | payload 长度 |
| --- | --- | ---: | ---: |
| FIRST | `0x40` | 0 | 46 |
| MIDDLE | `0x80` | 1 | 46 |
| LAST | `0xC0` | 2 | 3 |

重组后长度为 95 字节，输出 flags 清除分片位，payload 指向调用方提供的固定缓冲区。

## 接口边界

`yg_protocol_fragment_init/reset/accept/poll` 均为纯 C、有界、无动态分配接口。调用方负责提供缓冲区和单调毫秒时钟；重组层不访问 CAN、HAL 或电机状态。
