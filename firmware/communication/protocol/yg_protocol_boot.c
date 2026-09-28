/* 56只转换共享值契约；APP负责静默证明、票据与ENTER邮箱/复位。 */
#include "yg_protocol_boot.h"

yg_protocol_result_t yg_protocol_boot_decode(const yg_protocol_message_t *message,
                                             BootEntryRequest *request)
{
    BootEntryRequest candidate = {0};
    if (message == NULL || request == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (message->message_type != YG_PROTOCOL_BOOT_TYPE_ENTER)
    {
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    if (message->payload_length != YG_PROTOCOL_BOOT_REQUEST_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    if (message->payload == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    for (uint8_t i = 0U; i < YG_PROTOCOL_BOOT_REQUEST_SIZE; ++i)
    {
        candidate.boot_epoch |= (uint64_t)message->payload[i] << (8U * i);
    }
    if (candidate.boot_epoch == 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    candidate.sequence = message->sequence;
    candidate.source_node = message->source_node;
    *request = candidate;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_boot_encode_reply(const BootEntryReply *reply,
                                                   uint8_t *payload,
                                                   size_t capacity,
                                                   size_t *written)
{
    uint16_t result;
    uint16_t detail = 0U;
    if (reply == NULL || payload == NULL)
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
        result = 2U;
        break;
    case BOOT_ENTRY_PENDING:
        result = 1U;
        detail = 1U;
        break;
    case BOOT_ENTRY_READY:
        result = 1U;
        break;
    case BOOT_ENTRY_DENIED:
        result = 5U;
        break;
    case BOOT_ENTRY_FAILED:
        result = 8U;
        break;
    default:
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (capacity < YG_PROTOCOL_BOOT_REPLY_SIZE)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    payload[0] = (uint8_t)result;
    payload[1] = (uint8_t)(result >> 8U);
    payload[2] = (uint8_t)detail;
    payload[3] = (uint8_t)(detail >> 8U);
    if (written != NULL)
    {
        *written = YG_PROTOCOL_BOOT_REPLY_SIZE;
    }
    return YG_PROTOCOL_OK;
}
