#ifndef YG_PROTOCOL_COMMON_H
#define YG_PROTOCOL_COMMON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* APP、Loader 和主机共用的消息定义与字节操作；不包含产品权限或运行状态。 */

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

/** @brief 服务回复值；response_sequence 仅关联请求，不表示目标已应用或对端已收到。 */
typedef struct
{
    yg_protocol_result_t result;
    bool reply_ready; /* 内部控制字段，不在 CAN 数据中；false 表示本次不自动回复。 */
    uint16_t response_sequence;
    uint16_t response_type;
    uint8_t response_flags;
    uint8_t response_source_node;
    uint8_t response_destination_node;
    uint8_t response_reserved;
    uint32_t response_payload_length;
    uint8_t response_payload[YG_PROTOCOL_MAX_PAYLOAD_SIZE];
} yg_protocol_service_result_t;

/* 固定宽度字节转换无共享状态，允许未对齐地址；容量由编解码调用方验证。 */

/**
 * @brief 从小端字节读取无符号 16 位值。
 * @param data 非空只读缓冲区，至少 2 字节，允许未对齐地址。
 * @return 解码后的无符号整数。
 * @note 不检查容量，无硬件、分配或共享状态，可在 ISR 调用。
 */
static inline uint16_t YgProtocol_ReadU16Le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/**
 * @brief 从小端字节读取无符号 32 位值。
 * @param data 非空只读缓冲区，至少 4 字节，允许未对齐地址。
 * @return 解码后的无符号整数。
 * @note 不检查容量，无硬件、分配或共享状态，可在 ISR 调用。
 */
static inline uint32_t YgProtocol_ReadU32Le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) | ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/**
 * @brief 从小端字节读取无符号 64 位值。
 * @param data 非空只读缓冲区，至少 8 字节，允许未对齐地址。
 * @return 解码后的无符号整数。
 * @note 不检查容量，无硬件、分配或共享状态，可在 ISR 调用。
 */
static inline uint64_t YgProtocol_ReadU64Le(const uint8_t *data)
{
    return (uint64_t)YgProtocol_ReadU32Le(data) |
           ((uint64_t)YgProtocol_ReadU32Le(data + 4U) << 32U);
}

/**
 * @brief 将无符号 16 位值写为小端字节。
 * @param data 非空可写缓冲区，至少 2 字节，允许未对齐地址。
 * @param value 待写入的无符号整数。
 * @note 按地址递增写入，无容量检查、硬件、分配或共享状态，可在 ISR 调用。
 */
static inline void YgProtocol_WriteU16Le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value);
    data[1] = (uint8_t)(value >> 8U);
}

/**
 * @brief 将无符号 32 位值写为小端字节。
 * @param data 非空可写缓冲区，至少 4 字节，允许未对齐地址。
 * @param value 待写入的无符号整数。
 * @note 按地址递增写入，无容量检查、硬件、分配或共享状态，可在 ISR 调用。
 */
static inline void YgProtocol_WriteU32Le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value);
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

/**
 * @brief 将无符号 64 位值写为小端字节。
 * @param data 非空可写缓冲区，至少 8 字节，允许未对齐地址。
 * @param value 待写入的无符号整数。
 * @note 按地址递增写入，无容量检查、硬件、分配或共享状态，可在 ISR 调用。
 */
static inline void YgProtocol_WriteU64Le(uint8_t *data, uint64_t value)
{
    YgProtocol_WriteU32Le(data, (uint32_t)value);
    YgProtocol_WriteU32Le(data + 4U, (uint32_t)(value >> 32U));
}

/**
 * @brief 检查要求确认的单播请求的通用报文条件。
 * @param message 已完成帧校验的请求；允许空指针。
 * @return 版本、保留字段、地址和 ACK_REQUEST/RETRY 标志符合约定时返回 true。
 * @note 只判断请求外层条件；不检查载荷、消息类型或业务权限，不适用于响应或广播。
 */
static inline bool YgProtocol_IsAckUnicastRequest(const yg_protocol_message_t *message)
{
    return message != NULL && message->version == YG_PROTOCOL_VERSION && message->reserved == 0U &&
           message->source_node != YG_PROTOCOL_BROADCAST_NODE_ID &&
           message->destination_node != YG_PROTOCOL_BROADCAST_NODE_ID &&
           (message->flags == YG_PROTOCOL_FLAGS_ACK_REQUEST ||
            message->flags == (YG_PROTOCOL_FLAGS_ACK_REQUEST | YG_PROTOCOL_FLAGS_RETRY));
}

/**
 * @brief 检查业务上下文与需要回复的单播请求。
 * @param context 业务上下文。
 * @param request 请求。
 * @param result 回复缓冲。
 * @return 上下文及请求格式有效时返回 true。
 */
static inline bool yg_protocol_service_request_valid(const void *context,
                                                     const yg_protocol_message_t *request,
                                                     const yg_protocol_service_result_t *result)
{
    return context != NULL && request != NULL && result != NULL &&
           YgProtocol_IsAckUnicastRequest(request);
}

/**
 * @brief 写入公共的四字节结果回复。
 * @param response 已由调用方验证的回复缓冲。
 * @param code 线路结果码。
 * @param detail 业务细节码。
 */
static inline void yg_protocol_service_write_result(yg_protocol_service_result_t *response,
                                                    uint16_t code,
                                                    uint16_t detail)
{
    YgProtocol_WriteU16Le(response->response_payload, code);
    YgProtocol_WriteU16Le(response->response_payload + 2U, detail);
    response->response_payload_length = 4U;
}

#endif
