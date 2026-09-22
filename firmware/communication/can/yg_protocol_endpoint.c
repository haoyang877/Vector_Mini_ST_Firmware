/* 串行端点组装：帧队列到业务路由；不拥有硬件、不新增控制策略。 */
#include "yg_protocol_endpoint.h"

static bool queue_valid(const yg_protocol_transfer_queue_t *queue)
{
    return queue != NULL && queue->storage != NULL && queue->capacity > 0U &&
           yg_protocol_transfer_queue_count(queue) <= queue->capacity &&
           queue->head < queue->capacity && queue->tail < queue->capacity;
}

yg_protocol_result_t yg_protocol_endpoint_init(yg_protocol_endpoint_t *endpoint,
                                               const yg_protocol_endpoint_config_t *config)
{
    yg_protocol_result_t result;
    if (endpoint == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    endpoint->initialized = false;
    if (config == NULL || config->local_node == YG_PROTOCOL_BROADCAST_NODE_ID ||
        config->reply_priority > 7U || config->fragment_timeout_ms == 0U ||
        !queue_valid(config->rx) || !queue_valid(config->tx) || config->rx == config->tx ||
        config->rx->storage == config->tx->storage || config->router == NULL ||
        config->router->registry == NULL ||
        (config->router->route_count > 0U && config->router->routes == NULL) ||
        config->fragment_capacity > YG_PROTOCOL_MAX_MESSAGE_PAYLOAD)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    result = yg_protocol_fragment_init(
        &endpoint->fragment, config->fragment_storage, config->fragment_capacity);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    endpoint->config = *config;
    endpoint->response_pending = false;
    endpoint->initialized = true;
    return YG_PROTOCOL_OK;
}

static yg_protocol_message_t slice(const yg_protocol_message_t *message, size_t index, size_t count)
{
    yg_protocol_message_t part = *message;
    if (count > 1U)
    {
        size_t offset = index * YG_PROTOCOL_MAX_PAYLOAD_SIZE;
        part.payload = message->payload + offset;
        part.payload_length = message->payload_length - (uint32_t)offset;
        if (part.payload_length > YG_PROTOCOL_MAX_PAYLOAD_SIZE)
        {
            part.payload_length = YG_PROTOCOL_MAX_PAYLOAD_SIZE;
        }
        part.sequence = (uint16_t)index;
        if (index + 1U < count)
        {
            /* 分片请求只在尾片要求业务应答。 */
            part.flags &= (uint8_t)~YG_PROTOCOL_FLAGS_ACK_REQUEST;
        }
        part.flags |= index == 0U           ? YG_PROTOCOL_FLAGS_FRAGMENT_FIRST
                      : index + 1U == count ? YG_PROTOCOL_FLAGS_FRAGMENT_LAST
                                            : YG_PROTOCOL_FLAGS_FRAGMENT_MIDDLE;
    }
    return part;
}

yg_protocol_result_t yg_protocol_endpoint_send(yg_protocol_endpoint_t *endpoint,
                                               const yg_protocol_message_t *message,
                                               uint8_t priority)
{
    const yg_protocol_message_descriptor_t *descriptor;
    yg_protocol_transfer_frame_t frame;
    yg_protocol_message_t part;
    yg_protocol_result_t result;
    size_t count;
    if (endpoint == NULL || !endpoint->initialized || message == NULL ||
        (message->payload_length > 0U && message->payload == NULL))
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (message->source_node != endpoint->config.local_node)
    {
        return YG_PROTOCOL_INVALID_NODE_ID;
    }
    if ((message->flags & YG_PROTOCOL_FLAGS_FRAGMENT_MASK) != 0U ||
        message->payload_length > YG_PROTOCOL_MAX_MESSAGE_PAYLOAD)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    descriptor =
        yg_protocol_message_registry_find(endpoint->config.router->registry, message->message_type);
    if (descriptor == NULL)
    {
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    count = message->payload_length == 0U
                ? 1U
                : (message->payload_length + YG_PROTOCOL_MAX_PAYLOAD_SIZE - 1U) /
                      YG_PROTOCOL_MAX_PAYLOAD_SIZE;
    if (count > 1U && (!descriptor->allow_fragment || descriptor->periodic))
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    if (count > endpoint->config.tx->capacity ||
        count >
            endpoint->config.tx->capacity - yg_protocol_transfer_queue_count(endpoint->config.tx))
    {
        return YG_PROTOCOL_QUEUE_FULL;
    }
    /* 首片 pack 验证共享元数据；后续片长/地址/版本相同，串行容量已预留。 */
    part = slice(message, 0U, count);
    result = yg_protocol_canfd_pack(&part, priority, &frame);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    for (size_t index = 0U; index < count; ++index)
    {
        if (index > 0U)
        {
            part = slice(message, index, count);
            result = yg_protocol_canfd_pack(&part, priority, &frame);
            if (result != YG_PROTOCOL_OK)
            {
                return result;
            }
        }
        if (!yg_protocol_transfer_queue_push(endpoint->config.tx, &frame))
        {
            return YG_PROTOCOL_QUEUE_FULL;
        }
    }
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t send_pending(yg_protocol_endpoint_t *endpoint)
{
    yg_protocol_result_t result = yg_protocol_endpoint_send(
        endpoint, &endpoint->pending_response, endpoint->config.reply_priority);
    if (result != YG_PROTOCOL_QUEUE_FULL)
    {
        endpoint->response_pending = false;
    }
    return result;
}

yg_protocol_result_t yg_protocol_endpoint_process_one(yg_protocol_endpoint_t *endpoint,
                                                      uint32_t now_ms)
{
    yg_protocol_transfer_frame_t frame;
    if (endpoint == NULL || !endpoint->initialized)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (endpoint->response_pending || !yg_protocol_transfer_queue_pop(endpoint->config.rx, &frame))
    {
        return yg_protocol_endpoint_process_frame(endpoint, NULL, now_ms);
    }
    return yg_protocol_endpoint_process_frame(endpoint, &frame, now_ms);
}

yg_protocol_result_t yg_protocol_endpoint_process_frame(yg_protocol_endpoint_t *endpoint,
                                                        const yg_protocol_transfer_frame_t *frame,
                                                        uint32_t now_ms)
{
    yg_protocol_message_t part, message;
    const yg_protocol_message_descriptor_t *descriptor;
    yg_protocol_result_t result;
    if (endpoint == NULL || !endpoint->initialized)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (endpoint->response_pending)
    {
        /* 新帧不可越过待发应答；调用方保留输入并先以 NULL 推进应答。 */
        return frame == NULL ? send_pending(endpoint) : YG_PROTOCOL_QUEUE_FULL;
    }
    if (frame == NULL)
    {
        return YG_PROTOCOL_QUEUE_EMPTY;
    }
    result = yg_protocol_canfd_unpack(frame, endpoint->config.local_node, &part);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    descriptor =
        yg_protocol_message_registry_find(endpoint->config.router->registry, part.message_type);
    if (descriptor == NULL)
    {
        return YG_PROTOCOL_ROUTE_NOT_FOUND;
    }
    if ((part.flags & YG_PROTOCOL_FLAGS_FRAGMENT_MASK) != 0U &&
        (!descriptor->allow_fragment || descriptor->periodic))
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    result = yg_protocol_fragment_accept(
        &endpoint->fragment, &part, now_ms, endpoint->config.fragment_timeout_ms, &message);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    result =
        yg_protocol_router_handle(endpoint->config.router, &message, &endpoint->service_result);
    if (result != YG_PROTOCOL_OK ||
        (message.flags & (YG_PROTOCOL_FLAGS_ACK_REQUEST | YG_PROTOCOL_FLAGS_RESPONSE)) !=
            YG_PROTOCOL_FLAGS_ACK_REQUEST)
    {
        return result;
    }
    result = yg_protocol_router_build_response(
        &message, &endpoint->service_result, &endpoint->pending_response);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    endpoint->response_pending = true;
    return send_pending(endpoint);
}

yg_protocol_result_t yg_protocol_endpoint_reset(yg_protocol_endpoint_t *endpoint)
{
    if (endpoint == NULL || !endpoint->initialized)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    endpoint->config.rx->head = endpoint->config.rx->tail = endpoint->config.rx->count = 0U;
    endpoint->config.tx->head = endpoint->config.tx->tail = endpoint->config.tx->count = 0U;
    endpoint->response_pending = false;
    return yg_protocol_fragment_reset(&endpoint->fragment);
}
