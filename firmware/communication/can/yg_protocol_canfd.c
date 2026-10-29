#include "bsp_system.h"
#include "yg_protocol_canfd.h"
#include <string.h>
#include "yg_protocol_can_id.h"
#include "yg_protocol_frame_codec.h"

/* CAN FD 承载适配：物理长度与应用长度分离，发送失败不消费队列。
 * 不启动外设、不配置速率/滤波，也不与旧入口竞争读取 RX FIFO。 */

static uint8_t padded_length(size_t length)
{
    static const uint8_t lengths[] = {20U, 24U, 32U, 48U, 64U};
    for (size_t i = 0U; i < sizeof(lengths); ++i)
    {
        if (length <= lengths[i])
        {
            return lengths[i];
        }
    }
    return 0U;
}

static bool valid_metadata(const yg_protocol_transfer_frame_t *frame)
{
    yg_protocol_can_id_fields_t fields;
    return frame->extended && !frame->remote && frame->fd && frame->bitrate_switch &&
           frame->length >= 20U && frame->length <= 64U &&
           padded_length(frame->length) == frame->length &&
           yg_protocol_can_id_decode(frame->identifier, &fields) == YG_PROTOCOL_OK &&
           fields.source_node != YG_PROTOCOL_BROADCAST_NODE_ID;
}

yg_protocol_result_t yg_protocol_canfd_pack(const yg_protocol_message_t *message,
                                            uint8_t priority,
                                            yg_protocol_transfer_frame_t *frame)
{
    yg_protocol_transfer_frame_t candidate = {0};
    yg_protocol_can_id_fields_t fields;
    size_t length;
    yg_protocol_result_t result;
    if (message == NULL || frame == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (message->source_node == YG_PROTOCOL_BROADCAST_NODE_ID || priority > 7U)
    {
        return YG_PROTOCOL_INVALID_CAN_FRAME;
    }
    fields.priority = priority;
    fields.source_node = message->source_node;
    fields.destination_node = message->destination_node;
    result = yg_protocol_can_id_encode(&fields, &candidate.identifier);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    result = yg_protocol_frame_encode(message, candidate.data, sizeof(candidate.data), &length);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    candidate.length = padded_length(length);
    candidate.extended = true;
    candidate.fd = true;
    candidate.bitrate_switch = true;
    *frame = candidate;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_canfd_unpack(const yg_protocol_transfer_frame_t *frame,
                                              uint8_t local_node,
                                              yg_protocol_message_t *message)
{
    yg_protocol_message_t candidate;
    yg_protocol_result_t result;
    uint32_t payload_length;
    size_t length;
    if (frame == NULL || message == NULL || local_node == YG_PROTOCOL_BROADCAST_NODE_ID)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (!valid_metadata(frame))
    {
        return YG_PROTOCOL_INVALID_CAN_FRAME;
    }
    /* 先限制不可信长度再求和；codec 随后检查完整头部和双 CRC。 */
    payload_length = (uint32_t)frame->data[10] | ((uint32_t)frame->data[11] << 8U) |
                     ((uint32_t)frame->data[12] << 16U) | ((uint32_t)frame->data[13] << 24U);
    if (payload_length > YG_PROTOCOL_MAX_PAYLOAD_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    length = YG_PROTOCOL_HEADER_SIZE + payload_length + YG_PROTOCOL_TRAILER_SIZE;
    if (padded_length(length) != frame->length)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    result = yg_protocol_frame_decode(frame->data, length, &candidate);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    if (candidate.source_node != (uint8_t)frame->identifier ||
        candidate.destination_node != (uint8_t)(frame->identifier >> 8U) ||
        (candidate.destination_node != local_node &&
         candidate.destination_node != YG_PROTOCOL_BROADCAST_NODE_ID))
    {
        return YG_PROTOCOL_INVALID_NODE_ID;
    }
    for (size_t i = length; i < frame->length; ++i)
    {
        if (frame->data[i] != 0U)
        {
            return YG_PROTOCOL_INVALID_PADDING;
        }
    }
    *message = candidate;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_canfd_receive(yg_protocol_transfer_queue_t *queue,
                                               const BspCanFrame *received)
{
    yg_protocol_transfer_frame_t frame = {0};
    if (queue == NULL || queue->storage == NULL || queue->capacity == 0U || received == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    frame.identifier = received->identifier;
    frame.length = received->length;
    frame.extended = received->extended;
    frame.remote = received->remote;
    frame.fd = received->fd;
    frame.bitrate_switch = received->bitrate_switch;
    if (!valid_metadata(&frame))
    {
        return YG_PROTOCOL_INVALID_CAN_FRAME;
    }
    memcpy(frame.data, received->data, frame.length);
    return yg_protocol_transfer_queue_push(queue, &frame) ? YG_PROTOCOL_OK : YG_PROTOCOL_QUEUE_FULL;
}

yg_protocol_result_t yg_protocol_canfd_send_one(yg_protocol_transfer_queue_t *queue)
{
    yg_protocol_transfer_frame_t frame;
    BspCanFrame hardware = {0};
    if (queue == NULL || queue->storage == NULL || queue->capacity == 0U)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (!yg_protocol_transfer_queue_peek(queue, &frame))
    {
        return YG_PROTOCOL_QUEUE_EMPTY;
    }
    if (!valid_metadata(&frame))
    {
        return YG_PROTOCOL_INVALID_CAN_FRAME;
    }
    hardware.identifier = frame.identifier;
    hardware.length = frame.length;
    hardware.extended = frame.extended;
    hardware.remote = frame.remote;
    hardware.fd = frame.fd;
    hardware.bitrate_switch = frame.bitrate_switch;
    memcpy(hardware.data, frame.data, frame.length);
    if (BspCan_TrySend(&hardware) != BSP_OK)
    {
        return YG_PROTOCOL_TRANSPORT_BUSY;
    }
    (void)yg_protocol_transfer_queue_pop(queue, &frame);
    return YG_PROTOCOL_OK;
}

/* 单生产者/单消费者分别发布累计序号；完整槽复制先于发布，不需全局锁。 */

bool yg_protocol_transfer_queue_init(yg_protocol_transfer_queue_t *queue,
                                     yg_protocol_transfer_frame_t *storage,
                                     size_t capacity)
{
    if (queue == NULL || storage == NULL || capacity == 0U || capacity > UINT32_MAX / 2U)
    {
        return false;
    }
    queue->storage = storage;
    queue->capacity = capacity;
    queue->head = 0U;
    queue->tail = 0U;
    queue->produced = 0U;
    queue->consumed = 0U;
    queue->dropped = 0U;
    return true;
}

bool yg_protocol_transfer_queue_push(yg_protocol_transfer_queue_t *queue,
                                     const yg_protocol_transfer_frame_t *frame)
{
    if (queue == NULL || frame == NULL || queue->storage == NULL || queue->capacity == 0U ||
        frame->length > YG_PROTOCOL_TRANSFER_DATA_SIZE)
    {
        return false;
    }
    if ((uint32_t)(queue->produced - queue->consumed) >= queue->capacity)
    {
        queue->dropped++;
        return false;
    }
    BspSystem_MemoryBarrier();
    queue->storage[queue->head % queue->capacity] = *frame;
    queue->head = (queue->head + 1U) % queue->capacity;
    BspSystem_MemoryBarrier();
    ++queue->produced;
    return true;
}

bool yg_protocol_transfer_queue_pop(yg_protocol_transfer_queue_t *queue,
                                    yg_protocol_transfer_frame_t *frame)
{
    if (queue == NULL || frame == NULL || queue->storage == NULL || queue->capacity == 0U)
    {
        return false;
    }
    if (queue->produced == queue->consumed)
    {
        return false;
    }
    BspSystem_MemoryBarrier();
    *frame = queue->storage[queue->tail % queue->capacity];
    queue->tail = (queue->tail + 1U) % queue->capacity;
    BspSystem_MemoryBarrier();
    ++queue->consumed;
    return true;
}

size_t yg_protocol_transfer_queue_count(const yg_protocol_transfer_queue_t *queue)
{
    if (queue == NULL)
    {
        return 0U;
    }
    return (uint32_t)(queue->produced - queue->consumed);
}

bool yg_protocol_transfer_queue_discard(yg_protocol_transfer_queue_t *queue)
{
    if (queue == NULL || queue->storage == NULL || queue->capacity == 0U ||
        queue->produced == queue->consumed)
    {
        return false;
    }
    queue->tail = (queue->tail + 1U) % queue->capacity;
    BspSystem_MemoryBarrier();
    ++queue->consumed;
    return true;
}

uint32_t yg_protocol_transfer_queue_dropped(const yg_protocol_transfer_queue_t *queue)
{
    return queue == NULL ? 0U : queue->dropped;
}

bool yg_protocol_transfer_queue_peek(const yg_protocol_transfer_queue_t *queue,
                                     yg_protocol_transfer_frame_t *frame)
{
    if (queue == NULL || frame == NULL || queue->storage == NULL || queue->capacity == 0U)
    {
        return false;
    }
    if (queue->produced == queue->consumed)
    {
        return false;
    }
    BspSystem_MemoryBarrier();
    *frame = queue->storage[queue->tail % queue->capacity];
    return true;
}
