#ifndef YG_PROTOCOL_CANFD_H
#define YG_PROTOCOL_CANFD_H

#include "comm_hw.h"
#include "yg_protocol_transfer.h"
#include "yg_protocol_wire_types.h"

/**
 * @brief 将单个应用帧编码为最小 DLC 长度的扩展 CAN FD 帧。
 * @param message 输入消息片段，payload 最大 46 字节。
 * @param priority CAN 仲裁优先级，范围 0..7。
 * @param frame 输出完整帧；失败时保持不变。
 * @return 编码结果。
 * @note 不访问硬件；FD/BRS 置位，CRC 之后的 DLC 填充置零；不能在 ISR 中计算 CRC。
 */
yg_protocol_result_t yg_protocol_canfd_pack(const yg_protocol_message_t *message,
                                            uint8_t priority,
                                            yg_protocol_transfer_frame_t *frame);

/**
 * @brief 校验 CAN 元数据、地址与填充并解码一个应用帧。
 * @param frame 输入扩展 CAN FD 帧，length 为实际 DLC 字节数。
 * @param local_node 本机节点；不能为广播地址。
 * @param message 输出借用视图；失败时不变，成功后 payload 指向 frame 的数据。
 * @return 解码结果；CRC 校验通过不代表消息已被业务服务接受。
 * @note 接受本机及广播目的地；ID 地址必须匹配应用头。仅通信任务调用。
 */
yg_protocol_result_t yg_protocol_canfd_unpack(const yg_protocol_transfer_frame_t *frame,
                                              uint8_t local_node,
                                              yg_protocol_message_t *message);

/**
 * @brief 将平台收到的 CAN FD 帧复制到 RX 队列，不读取硬件 FIFO。
 * @param queue 调用方拥有的已初始化 RX 队列。
 * @param received 由唯一硬件接收入口提供的帧；函数返回后不保存其指针。
 * @return 入队结果；CRC、地址匹配与业务准入留给后续 unpack 和服务处理。
 * @note 只做有界复制和元数据检查；队列访问必须串行化，不能直接无锁跨 ISR/任务。
 */
yg_protocol_result_t yg_protocol_canfd_receive(yg_protocol_transfer_queue_t *queue,
                                               const CommHwCanFrame *received);

/**
 * @brief 尝试发送 TX 队首的一个 CAN FD 帧，成功才出队。
 * @param queue 已初始化 TX 队列，内容由 pack 生成。
 * @return 空队列、格式错误或硬件忙返回对应结果，失败时保留队首。
 * @note 单上下文调用；一次调用最多一次平台发送，无等待、无循环重试。
 */
yg_protocol_result_t yg_protocol_canfd_send_one(yg_protocol_transfer_queue_t *queue);

#endif
