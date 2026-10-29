#include "yg_protocol_endpoint.h"
#include <string.h>

/* 串行端点组装：帧队列到命令执行；不拥有硬件、不新增控制策略。 */

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
        config->rx->storage == config->tx->storage || config->commands == NULL ||
        (config->commands->count > 0U && config->commands->entries == NULL) ||
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
    const yg_protocol_command_t *descriptor;
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
    descriptor = yg_protocol_commands_find(endpoint->config.commands, message->message_type);
    if (descriptor == NULL)
    {
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    count = message->payload_length == 0U
                ? 1U
                : (message->payload_length + YG_PROTOCOL_MAX_PAYLOAD_SIZE - 1U) /
                      YG_PROTOCOL_MAX_PAYLOAD_SIZE;
    if (count > 1U && (!descriptor->allow_fragment))
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
    const yg_protocol_command_t *descriptor;
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
    descriptor = yg_protocol_commands_find(endpoint->config.commands, part.message_type);
    if (descriptor == NULL || descriptor->handler == NULL)
    {
        return YG_PROTOCOL_ROUTE_NOT_FOUND;
    }
    if ((part.flags & YG_PROTOCOL_FLAGS_FRAGMENT_MASK) != 0U && (!descriptor->allow_fragment))
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    result = yg_protocol_fragment_accept(
        &endpoint->fragment, &part, now_ms, endpoint->config.fragment_timeout_ms, &message);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    result = yg_protocol_commands_execute(descriptor, &message, &endpoint->service_result);
    if (result != YG_PROTOCOL_OK || !endpoint->service_result.reply_ready ||
        (message.flags & (YG_PROTOCOL_FLAGS_ACK_REQUEST | YG_PROTOCOL_FLAGS_RESPONSE)) !=
            YG_PROTOCOL_FLAGS_ACK_REQUEST)
    {
        return result;
    }
    result = yg_protocol_commands_build_response(
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
    endpoint->config.rx->head = endpoint->config.rx->tail = 0U;
    endpoint->config.rx->produced = endpoint->config.rx->consumed = 0U;
    endpoint->config.tx->head = endpoint->config.tx->tail = 0U;
    endpoint->config.tx->produced = endpoint->config.tx->consumed = 0U;
    endpoint->response_pending = false;
    return yg_protocol_fragment_reset(&endpoint->fragment);
}

/* 单一命令表：登记、查找、执行；不访问硬件或电机全局状态。 */

yg_protocol_result_t yg_protocol_commands_register(yg_protocol_commands_t *commands,
                                                   const yg_protocol_command_t *entries,
                                                   size_t count)
{
    if (commands == NULL || (count > 0U && entries == NULL))
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    for (size_t i = 0U; i < count; ++i)
    {
        for (size_t j = i + 1U; j < count; ++j)
        {
            if (entries[i].message_type == entries[j].message_type)
            {
                return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
            }
        }
    }
    commands->entries = entries;
    commands->count = count;
    return YG_PROTOCOL_OK;
}

const yg_protocol_command_t *yg_protocol_commands_find(const yg_protocol_commands_t *commands,
                                                       uint16_t message_type)
{
    if (commands == NULL || commands->entries == NULL)
    {
        return NULL;
    }
    for (size_t i = 0U; i < commands->count; ++i)
    {
        if (commands->entries[i].message_type == message_type)
        {
            return &commands->entries[i];
        }
    }
    return NULL;
}

yg_protocol_result_t yg_protocol_commands_execute(const yg_protocol_command_t *command,
                                                  const yg_protocol_message_t *request,
                                                  yg_protocol_service_result_t *result)
{
    if (request == NULL || result == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    memset(result, 0, sizeof(*result));
    result->result = YG_PROTOCOL_ROUTE_NOT_FOUND;
    if (command == NULL || command->handler == NULL ||
        command->message_type != request->message_type)
    {
        return result->result;
    }
    result->reply_ready = true;
    result->response_sequence = request->sequence;
    result->response_type = request->message_type;
    result->response_flags = YG_PROTOCOL_FLAGS_RESPONSE;
    result->response_source_node = request->destination_node;
    result->response_destination_node = request->source_node;
    result->result = command->handler(command->context, request, result);
    return result->result;
}

yg_protocol_result_t
yg_protocol_commands_build_response(const yg_protocol_message_t *request,
                                    const yg_protocol_service_result_t *service_result,
                                    yg_protocol_message_t *response)
{
    if (request == NULL || service_result == NULL || response == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (service_result->result != YG_PROTOCOL_OK)
    {
        return service_result->result;
    }
    if (service_result->response_source_node == YG_PROTOCOL_BROADCAST_NODE_ID ||
        service_result->response_destination_node == YG_PROTOCOL_BROADCAST_NODE_ID)
    {
        return YG_PROTOCOL_INVALID_NODE_ID;
    }
    if (service_result->response_payload_length > YG_PROTOCOL_MAX_PAYLOAD_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    response->version = request->version;
    response->flags = service_result->response_flags;
    response->source_node = service_result->response_source_node;
    response->destination_node = service_result->response_destination_node;
    response->message_type = service_result->response_type;
    response->sequence = service_result->response_sequence;
    response->payload_length = service_result->response_payload_length;
    response->reserved = service_result->response_reserved;
    response->payload = service_result->response_payload;
    return YG_PROTOCOL_OK;
}
