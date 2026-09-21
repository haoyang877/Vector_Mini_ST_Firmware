# yg_protocol CAN FD 适配契约 v0.1

日期：2026-09-21。状态：已接入 STM32G4 CAN FD 收发入口并完成离线闭环测试；项目候选承载规则，不代表已完成实机收发。

## 1. 实现边界

`firmware/communication/can/yg_protocol_canfd.{c,h}` 组合既有帧 codec、CAN ID codec、串行队列和
平台 `comm_hw`。没有新增电机 type、ACK 语义、自动重传、硬件初始化、滤波配置或调度入口。
`yg_protocol_link.c` 已在 `FDCAN1_Param_Init`、FDCAN RX 中断和 `CAN_Service` 中接线，首批只注册
GET_INFO/GET_CAPS，只读响应不使能电机。旧标准帧入口继续保留兼容路径；扩展 CAN FD 帧由新端点独占处理。
STM32G4 的工程输入已切到 1 Mbit/s 仲裁段、5 Mbit/s 数据段、FD+BRS；扩展帧通过全局滤波器进入 FIFO0。

## 2. 数据和所有权

```text
业务消息 → canfd_pack → transfer_queue_push(TX)
                            ↓
                  canfd_send_one → comm_hw_can_try_send_frame → HAL

唯一硬件 RX 入口 → comm_hw_can_receive → canfd_receive(RX queue)
                            ↓
                   transfer_queue_pop（调用方缓冲）
                            ↓
                  canfd_unpack → 分片处理/消息路由 → 服务
```

- `pack` 只做编码，不入队、不发硬件；`receive` 只复制调用方提供的硬件帧，不主动读取 FIFO。
- `unpack` 返回的 payload 借用输入帧存储，必须在覆写该帧之前完成同步处理或复制。
- `send_one` 先 peek，平台成功接收后才 pop。忙/失败保留队首，不等待、不循环重试。
- 入队、出队复制完整帧；队列满丢弃最新并计数。新链路 RX 队列采用单生产者/单消费者 head/tail，
  ISR 只提交已完整复制的帧，2 kHz 服务上下文消费；不得增加第二个生产者或消费者。
- `yg_protocol_transfer_frame_t` 实际包含 CAN 元数据，不能宣称是串口/以太网通用帧。
- `comm_hw` 不含公司帧语义：只检查 CAN ID 宽度、FD 数据帧和合法实际长度，并调用一次 HAL。

## 3. 候选承载规则

| 项目 | v0.1 实现 |
| --- | --- |
| CAN ID | 29 位，priority 0..7，R/DP 为 0，PF=0xEF，dst/src 各 8 位 |
| 帧形式 | IDE=1、FDF=1、BRS=1、非远程帧 |
| 路由 | ID 的 src/dst 必须等于公司头部地址；接受本机和 0xFF 广播目的地；源地址不可为广播 |
| 应用长度 | `18 + payload_len`，payload_len 为 0..46；加法之前检查上限 |
| DLC 对齐 | 取容纳应用帧的最小长度 20/24/32/48/64；接收也要求最小长度 |
| 填充 | CRC16 之后全为 0；接收遇非零填充拒绝，不把填充算进 CRC |
| CRC 基线 | CRC8 覆盖字节 0..14；CRC16 覆盖完整 16B 头及 payload，CRC16 小端；沿用既有离线示例 |
| 分片 | pack/unpack 仅处理一个物理片段，不做自动分片、ACK 或重传 |

公司 flags、重组并发和 type 登记的未决事项仍需独立定版。此实现不把 CRC 成功等同于业务准入，
也不把平台提交成功等同于电机执行成功。

## 4. 黄金向量与验证

沿用既有离线位置示例，type=105 是项目示例编号，没有开启真实电机处理器：

```text
CAN ID = 08EF0302, IDE=1 FDF=1 BRS=1
应用长度 = 26, DLC = 13, 物理长度 = 32
5A A5 01 00 02 03 69 00 02 00 08 00 00 00 00 D1
69 02 34 12 E8 03 00 00 D2 E1 00 00 00 00 00 00
```

`tests/unit/yg_protocol_canfd_test.c` 固定比较上述字节，并覆盖全部 47 种 payload 长度、DLC 对齐、
填充、CRC、地址匹配、广播、无效 CAN 元数据、越界长度、失败输出不变、RX 复制及满队列、TX 忙后
保留队首和 FIFO 顺序。链路最终进入测试服务函数；不表示已接入电机服务。

`tests/unit/native/run_can_status_tests.py` 编译真实 STM32 端口与 HAL 替身，覆盖所有 16 个 DLC、
FD/BRS 元数据、非法输入、HAL 失败及旧标准帧行为。G4 HAL 的 DataLength 是未移位的 DLC 值 0..15，
平台层将实际字节数转换为该值，协议层不包含厂商常量。

## 5. 兼容和下一步

`CommHwCanFrame` 末尾增加 `fd/bitrate_switch`，所有源码调用者需一起编译；这是内部 C 接口变化，
不是线上协议变化。既有发送接口保留；旧接收者忽略新字段。接收失败保持输出不变。

下一步仍需在明确的无动力台架上测量位时序、收发器能力、发送队列和中断负载；离线通过不能替代
CAN FD 实机收发，也不能宣称 5 轴 1 kHz 能力。硬件冒烟只允许 GET_INFO/GET_CAPS，需记录板卡固件
SHA-256、Node ID、适配器和原始收发帧。
