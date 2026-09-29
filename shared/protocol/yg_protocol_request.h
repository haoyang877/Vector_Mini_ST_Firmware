#ifndef YG_PROTOCOL_REQUEST_H
#define YG_PROTOCOL_REQUEST_H
#include "yg_protocol_wire_types.h"
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
#endif
