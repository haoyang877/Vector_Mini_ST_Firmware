#ifndef YG_PROTOCOL_WIRE_TYPES_H
#define YG_PROTOCOL_WIRE_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define YG_PROTOCOL_MAGIC 0xA55AU
#define YG_PROTOCOL_VERSION 0x01U
#define YG_PROTOCOL_HEADER_SIZE 16U
#define YG_PROTOCOL_TRAILER_SIZE 2U
#define YG_PROTOCOL_MAX_FRAME_SIZE 64U
#define YG_PROTOCOL_MAX_PAYLOAD_SIZE                                                               \
    (YG_PROTOCOL_MAX_FRAME_SIZE - YG_PROTOCOL_HEADER_SIZE - YG_PROTOCOL_TRAILER_SIZE)
#define YG_PROTOCOL_BROADCAST_NODE_ID 0xFFU
#define YG_PROTOCOL_FLAGS_FRAGMENT_MASK 0xC0U
#define YG_PROTOCOL_FLAGS_FRAGMENT_SINGLE 0x00U
#define YG_PROTOCOL_FLAGS_FRAGMENT_FIRST 0x40U
#define YG_PROTOCOL_FLAGS_FRAGMENT_MIDDLE 0x80U
#define YG_PROTOCOL_FLAGS_FRAGMENT_LAST 0xC0U
/* 项目评审稿的请求/响应位，尚非公司正式勘误。 */
#define YG_PROTOCOL_FLAGS_ACK_REQUEST 0x20U
#define YG_PROTOCOL_FLAGS_RESPONSE 0x10U
#define YG_PROTOCOL_FLAGS_RETRY 0x04U

/** @brief 协议核心操作结果。 */
typedef enum
{
    YG_PROTOCOL_OK = 0,
    YG_PROTOCOL_INVALID_ARGUMENT,
    YG_PROTOCOL_BUFFER_TOO_SMALL,
    YG_PROTOCOL_INVALID_MAGIC,
    YG_PROTOCOL_INVALID_VERSION,
    YG_PROTOCOL_INVALID_LENGTH,
    YG_PROTOCOL_INVALID_HEADER_CRC,
    YG_PROTOCOL_INVALID_FRAME_CRC,
    YG_PROTOCOL_INVALID_NODE_ID,
    YG_PROTOCOL_INVALID_MESSAGE_TYPE,
    YG_PROTOCOL_ROUTE_NOT_FOUND,
    YG_PROTOCOL_FRAGMENT_IN_PROGRESS,
    YG_PROTOCOL_FRAGMENT_TIMEOUT,
    YG_PROTOCOL_FRAGMENT_SEQUENCE_ERROR,
    YG_PROTOCOL_FRAGMENT_STATE_ERROR,
    YG_PROTOCOL_INVALID_CAN_FRAME,
    YG_PROTOCOL_INVALID_PADDING,
    YG_PROTOCOL_QUEUE_FULL,
    YG_PROTOCOL_QUEUE_EMPTY,
    YG_PROTOCOL_TRANSPORT_BUSY,
    YG_PROTOCOL_INVALID_FIELD,
    YG_PROTOCOL_DATA_UNAVAILABLE,
} yg_protocol_result_t;

/** @brief 已解码的公司统一帧视图；payload 仍由输入缓冲区拥有。 */
typedef struct
{
    uint8_t version;
    uint8_t flags;
    uint8_t source_node;
    uint8_t destination_node;
    uint16_t message_type;
    uint16_t sequence;
    uint32_t payload_length;
    uint8_t reserved;
    const uint8_t *payload;
} yg_protocol_frame_view_t;

/** @brief 协议消息名称，语义上表示已完成帧校验的业务消息。 */
typedef yg_protocol_frame_view_t yg_protocol_message_t;

/** @brief 服务执行结果；不表示物理帧已经被对端接收。 */
typedef struct
{
    yg_protocol_result_t result;
    uint16_t applied_sequence;
    uint16_t response_type;
    uint8_t response_flags;
    uint8_t response_source_node;
    uint8_t response_destination_node;
    uint8_t response_reserved;
    uint32_t response_payload_length;
    uint8_t response_payload[YG_PROTOCOL_MAX_PAYLOAD_SIZE];
} yg_protocol_service_result_t;

#endif
