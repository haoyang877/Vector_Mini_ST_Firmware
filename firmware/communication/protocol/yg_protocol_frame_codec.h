#ifndef YG_PROTOCOL_FRAME_CODEC_H
#define YG_PROTOCOL_FRAME_CODEC_H

#include <stddef.h>
#include <stdint.h>

#include "yg_protocol_wire_types.h"

/**
 * @brief 把已构造的协议消息编码为一个 CAN FD 应用帧。
 * @param message 待编码消息；payload 由调用方拥有。
 * @param buffer 输出缓冲区。
 * @param capacity 输出缓冲区容量，不能小于实际帧长度。
 * @param written 成功时写入实际帧长度，可为 NULL。
 * @return 编码结果。
 * @note 本接口只编码单帧；payload 最大为 46 字节，不执行分片和动态分配。
 */
yg_protocol_result_t yg_protocol_frame_encode(const yg_protocol_frame_view_t *message,
                                              uint8_t *buffer,
                                              size_t capacity,
                                              size_t *written);

/**
 * @brief 解码并校验一个完整的 CAN FD 应用帧。
 * @param buffer 输入帧缓冲区。
 * @param length 输入帧长度，必须等于 16+payload_len+2。
 * @param message 输出消息视图；payload 指向输入缓冲区。
 * @return 解码结果。
 * @note 不复制 payload；调用方必须保证输入缓冲区在消息使用期间保持有效。
 */
yg_protocol_result_t
yg_protocol_frame_decode(const uint8_t *buffer, size_t length, yg_protocol_frame_view_t *message);

#endif
