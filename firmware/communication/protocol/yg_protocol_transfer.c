/* CAN 帧的单生产者/单消费者内存队列；先写槽位，最后提交 head。 */
#include "yg_protocol_transfer.h"

#include <string.h>

static size_t queue_used(const yg_protocol_transfer_queue_t *queue)
{
    return queue->head - queue->tail;
}

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
    if (queue_used(queue) >= queue->capacity)
    {
        queue->dropped++;
        return false;
    }
    queue->storage[queue->head % queue->capacity] = *frame;
    queue->head++;
    queue->count = queue_used(queue);
    return true;
}

bool yg_protocol_transfer_queue_pop(yg_protocol_transfer_queue_t *queue,
                                    yg_protocol_transfer_frame_t *frame)
{
    if (queue == NULL || frame == NULL || queue->storage == NULL || queue->capacity == 0U ||
        queue->head == queue->tail)
    {
        return false;
    }
    *frame = queue->storage[queue->tail % queue->capacity];
    queue->tail++;
    queue->count = queue_used(queue);
    return true;
}

size_t yg_protocol_transfer_queue_count(const yg_protocol_transfer_queue_t *queue)
{
    return queue == NULL ? 0U : queue_used(queue);
}

uint32_t yg_protocol_transfer_queue_dropped(const yg_protocol_transfer_queue_t *queue)
{
    return queue == NULL ? 0U : queue->dropped;
}

bool yg_protocol_transfer_queue_peek(const yg_protocol_transfer_queue_t *queue,
                                     yg_protocol_transfer_frame_t *frame)
{
    if (queue == NULL || frame == NULL || queue->storage == NULL || queue->capacity == 0U ||
        queue->head == queue->tail)
    {
        return false;
    }
    *frame = queue->storage[queue->tail % queue->capacity];
    return true;
}
