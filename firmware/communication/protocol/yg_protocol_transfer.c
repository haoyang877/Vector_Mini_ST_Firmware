/* 固定容量环形队列；跨中断访问由通信组合层串行化。 */
#include "yg_protocol_transfer.h"

#include <string.h>

bool yg_protocol_transfer_queue_init(yg_protocol_transfer_queue_t *queue,
                                     yg_protocol_transfer_frame_t *storage,
                                     size_t capacity)
{
    if (queue == NULL || storage == NULL || capacity == 0U)
    {
        return false;
    }
    queue->storage = storage;
    queue->capacity = capacity;
    queue->head = 0U;
    queue->tail = 0U;
    queue->count = 0U;
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
    if (queue->count >= queue->capacity)
    {
        queue->dropped++;
        return false;
    }
    queue->storage[queue->head % queue->capacity] = *frame;
    queue->head = (queue->head + 1U) % queue->capacity;
    queue->count++;
    return true;
}

bool yg_protocol_transfer_queue_pop(yg_protocol_transfer_queue_t *queue,
                                    yg_protocol_transfer_frame_t *frame)
{
    if (queue == NULL || frame == NULL || queue->storage == NULL || queue->capacity == 0U)
    {
        return false;
    }
    if (queue->count == 0U)
    {
        return false;
    }
    *frame = queue->storage[queue->tail % queue->capacity];
    queue->tail = (queue->tail + 1U) % queue->capacity;
    queue->count--;
    return true;
}

size_t yg_protocol_transfer_queue_count(const yg_protocol_transfer_queue_t *queue)
{
    if (queue == NULL)
    {
        return 0U;
    }
    return queue->count;
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
    if (queue->count == 0U)
    {
        return false;
    }
    *frame = queue->storage[queue->tail % queue->capacity];
    return true;
}
