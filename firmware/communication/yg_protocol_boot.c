#include "yg_protocol.h"
#include <string.h>

/* Loader 通信：命令、READY 票据、超时和物理发送确认集中于此。 */
#define BOOT_TX_TIMEOUT_MS 100U

typedef enum
{
    BOOT_TX_IDLE,
    BOOT_TX_QUEUED,
    BOOT_TX_PENDING
} BootTxState;

typedef struct
{
    uint8_t node_id;
    size_t tx_slot;
    BootTxState tx_state;
    uint64_t ticket;
    uint32_t ready_ms;
    uint16_t reply_sequence;
    uint8_t reply_destination;
} BootState;

static BootState s_boot;

static yg_protocol_result_t encode_loader_reply(const BootEntryReply *reply,
                                                ProtocolResponse *response);
static void discard_queued_boot_reply(CanQueue *queue);
static void finish_boot_reply(bool success);
static bool is_boot_ready_frame(const BspCanFrame *frame);

/* Loader 命令与回复 */

static yg_protocol_result_t encode_loader_reply(const BootEntryReply *reply,
                                                ProtocolResponse *response)
{
    ProtocolCode result;
    uint16_t detail = 0U;
    if (reply == NULL || response == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if ((reply->result == BOOT_ENTRY_READY && reply->ack_ticket == 0U) ||
        (reply->result != BOOT_ENTRY_READY && reply->ack_ticket != 0U))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    switch (reply->result)
    {
    case BOOT_ENTRY_UNAVAILABLE:
        result = PROTOCOL_RESULT_UNSUPPORTED;
        break;
    case BOOT_ENTRY_PENDING:
        result = PROTOCOL_RESULT_ACCEPTED;
        detail = 1U;
        break;
    case BOOT_ENTRY_READY:
        result = PROTOCOL_RESULT_ACCEPTED;
        break;
    case BOOT_ENTRY_DENIED:
        result = PROTOCOL_RESULT_DENIED;
        break;
    case BOOT_ENTRY_FAILED:
        result = PROTOCOL_RESULT_FAILED;
        break;
    default:
        return YG_PROTOCOL_INVALID_FIELD;
    }
    Protocol_WriteResult(response, result, detail);
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t ProtocolBoot_HandleCommand(ProtocolExchange *exchange)
{
    const yg_protocol_message_t *request = &exchange->request;
    ProtocolResponse *response = &exchange->reply;
    ProtocolCommandArgs args = {0};
    ProtocolCode code = PROTOCOL_RESULT_OK;
    if (request->source_node != PROTOCOL_MASTER_ID)
    {
        code = PROTOCOL_RESULT_FORBIDDEN;
    }
    else if (request->payload_length != PROTOCOL_BOOT_REQUEST_SIZE)
    {
        code = PROTOCOL_RESULT_BAD_LENGTH;
    }
    else if (request->payload == NULL)
    {
        code = PROTOCOL_RESULT_BAD_FIELD;
    }
    else if (request->message_type == PROTOCOL_BOOT_CONFIRM)
    {
        args.confirm.version = YgProtocol_ReadU32Le(request->payload);
        args.confirm.image_crc32 = YgProtocol_ReadU32Le(request->payload + 4U);
    }
    else
    {
        args.boot_epoch = YgProtocol_ReadU64Le(request->payload);
        if (args.boot_epoch == 0U)
        {
            code = PROTOCOL_RESULT_BAD_FIELD;
        }
    }
    if (code == PROTOCOL_RESULT_OK)
    {
        Protocol_RecordCommand(request, args);
    }
    if (Protocol_Boot.request == NULL)
    {
        return YG_PROTOCOL_ROUTE_NOT_FOUND;
    }
    if (code != PROTOCOL_RESULT_OK)
    {
        Protocol_WriteResult(response, code, 0U);
        return YG_PROTOCOL_OK;
    }
    if (request->message_type == PROTOCOL_BOOT_CONFIRM)
    {
        code = PROTOCOL_RESULT_UNSUPPORTED;
        if (Protocol_Boot.confirm != NULL)
        {
            bool confirmed = Protocol_Boot.confirm(
                Protocol_Boot.context, args.confirm.version, args.confirm.image_crc32);
            code = confirmed ? PROTOCOL_RESULT_OK : PROTOCOL_RESULT_DENIED;
        }
        Protocol_WriteResult(response, code, 0U);
        return YG_PROTOCOL_OK;
    }
    if (s_boot.ticket != 0U)
    {
        /* 一次只发送一个带票据的READY；重发不能替换原票据或续长其截止。 */
        Protocol_WriteResult(response, PROTOCOL_RESULT_BUSY, 0U);
        return YG_PROTOCOL_OK;
    }
    if (Protocol_Boot.tx_complete == NULL)
    {
        Protocol_WriteResult(response, PROTOCOL_RESULT_UNSUPPORTED, 0U);
        return YG_PROTOCOL_OK;
    }
    const BootEntryRequest command = {.boot_epoch = args.boot_epoch,
                                      .source_node = request->source_node,
                                      .sequence = request->sequence};
    BootEntryReply reply = {.result = BOOT_ENTRY_FAILED};
    Protocol_Boot.request(Protocol_Boot.context, &command, &reply);
    if (encode_loader_reply(&reply, response) != YG_PROTOCOL_OK)
    {
        if (reply.ack_ticket != 0U)
        {
            Protocol_Boot.tx_complete(Protocol_Boot.context, reply.ack_ticket, false);
        }
        Protocol_WriteResult(response, PROTOCOL_RESULT_FAILED, 0U);
        return YG_PROTOCOL_OK;
    }
    if (reply.result == BOOT_ENTRY_READY)
    {
        s_boot.ticket = reply.ack_ticket;
        s_boot.node_id = request->destination_node;
        s_boot.reply_sequence = request->sequence;
        s_boot.reply_destination = request->source_node;
    }
    return YG_PROTOCOL_OK;
}

/* 发送关联、超时与确认 */

static void finish_boot_reply(bool success)
{
    uint64_t ticket = s_boot.ticket;
    s_boot.tx_state = BOOT_TX_IDLE;
    s_boot.ticket = 0U;
    Protocol_Boot.tx_complete(Protocol_Boot.context, ticket, success);
}

static bool is_boot_ready_frame(const BspCanFrame *frame)
{
    /* TX 只接收已编码的帧；核对槽位中的请求身份，不再次解码或计算 CRC。 */
    return frame->length == 24U && frame->data[3] == YG_PROTOCOL_FLAGS_RESPONSE &&
           frame->data[4] == s_boot.node_id && frame->data[5] == s_boot.reply_destination &&
           YgProtocol_ReadU16Le(frame->data + 6U) == PROTOCOL_BOOT_ENTER &&
           YgProtocol_ReadU16Le(frame->data + 8U) == s_boot.reply_sequence &&
           YgProtocol_ReadU32Le(frame->data + 10U) == PROTOCOL_BOOT_REPLY_SIZE &&
           YgProtocol_ReadU32Le(frame->data + 16U) == PROTOCOL_RESULT_ACCEPTED;
}

static void discard_queued_boot_reply(CanQueue *queue)
{
    size_t slot = queue->tail;
    for (size_t i = 0U; i < queue_count(queue); ++i)
    {
        if (slot == s_boot.tx_slot)
        {
            if (!is_boot_ready_frame(&queue->storage[slot]))
            {
                return;
            }
            const size_t last = (queue->head + queue->capacity - 1U) % queue->capacity;
            /* 只有一条带票据回复；按当前队列容量有界移动，保持其余帧顺序。 */
            while (slot != last)
            {
                size_t next = (slot + 1U) % queue->capacity;
                queue->storage[slot] = queue->storage[next];
                slot = next;
            }
            queue->head = last;
            --queue->produced;
            return;
        }
        slot = (slot + 1U) % queue->capacity;
    }
}

void ProtocolBoot_TrackReply(const CanQueue *queue,
                             size_t previous_head,
                             yg_protocol_result_t result,
                             uint32_t now_ms)
{
    if (s_boot.tx_state != BOOT_TX_IDLE || s_boot.ticket == 0U)
    {
        return;
    }
    if (result != YG_PROTOCOL_OK || queue->head == previous_head)
    {
        finish_boot_reply(false);
        return;
    }
    s_boot.tx_slot = previous_head;
    s_boot.ready_ms = now_ms;
    s_boot.tx_state = BOOT_TX_QUEUED;
}

void ProtocolBoot_Process(CanQueue *queue, uint32_t now_ms)
{
    if (s_boot.tx_state == BOOT_TX_PENDING)
    {
        if ((uint32_t)(now_ms - s_boot.ready_ms) >= BOOT_TX_TIMEOUT_MS)
        {
            finish_boot_reply(false);
        }
        else if (BspCan_IsTransmitComplete())
        {
            finish_boot_reply(true);
        }
        else if (!BspCan_IsTransmitBusy())
        {
            finish_boot_reply(false);
        }
    }
    if (s_boot.tx_state == BOOT_TX_QUEUED &&
        (uint32_t)(now_ms - s_boot.ready_ms) >= BOOT_TX_TIMEOUT_MS)
    {
        discard_queued_boot_reply(queue);
        finish_boot_reply(false);
    }
}

yg_protocol_result_t ProtocolBoot_SendNext(CanQueue *queue)
{
    /* 交接回复确认前不提交其它帧，硬件完成位只对应本次回复。 */
    if (s_boot.tx_state == BOOT_TX_PENDING)
    {
        return YG_PROTOCOL_TRANSPORT_BUSY;
    }
    BspCanFrame frame;
    bool boot_reply = s_boot.tx_state == BOOT_TX_QUEUED && queue->tail == s_boot.tx_slot;
    if (!queue_peek(queue, &frame))
    {
        if (boot_reply)
        {
            finish_boot_reply(false);
        }
        return YG_PROTOCOL_QUEUE_EMPTY;
    }
    /* 只核对带票据的那一帧；普通回复已经在入队前编码完成。 */
    if (boot_reply && !is_boot_ready_frame(&frame))
    {
        (void)queue_drop(queue);
        finish_boot_reply(false);
        return YG_PROTOCOL_INVALID_CAN_FRAME;
    }
    if (!BspCan_TrySend(&frame))
    {
        return YG_PROTOCOL_TRANSPORT_BUSY;
    }
    (void)queue_drop(queue);
    if (boot_reply)
    {
        s_boot.tx_state = BOOT_TX_PENDING;
    }
    return YG_PROTOCOL_OK;
}
