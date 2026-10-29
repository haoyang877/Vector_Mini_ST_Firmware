#include "yg_protocol_service.h"
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

yg_protocol_result_t yg_protocol_boot_handle(void *context,
                                             const yg_protocol_message_t *request,
                                             yg_protocol_service_result_t *result)
{
    yg_protocol_boot_context_t *boot = context;
    BootEntryRequest command;
    BootEntryReply reply = {.result = BOOT_ENTRY_FAILED};
    yg_protocol_result_t decoded;
    if (!yg_protocol_service_request_valid(boot, request, result))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_PROTOCOL_MASTER_NODE_ID)
    {
        yg_protocol_service_write_result(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (request->message_type == YG_PROTOCOL_BOOT_TYPE_CONFIRM)
    {
        if (request->payload_length != 8U)
        {
            yg_protocol_service_write_result(result, 3U, 0U);
        }
        else if (boot->boot_service.confirm == NULL)
        {
            yg_protocol_service_write_result(result, 2U, 0U);
        }
        else
        {
            bool confirmed =
                boot->boot_service.confirm(boot->boot_service.context,
                                           YgProtocol_ReadU32Le(request->payload),
                                           YgProtocol_ReadU32Le(request->payload + 4U));
            yg_protocol_service_write_result(result, confirmed ? 0U : 5U, 0U);
        }
        return YG_PROTOCOL_OK;
    }
    decoded = yg_protocol_boot_decode(request, &command);
    if (decoded != YG_PROTOCOL_OK)
    {
        yg_protocol_service_write_result(
            result, decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (boot->boot_ticket != 0U)
    {
        /* 一次只发送一个带票据的READY；重发不能替换原票据或续长其截止。 */
        yg_protocol_service_write_result(result, 6U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (boot->boot_service.request == NULL || boot->boot_service.tx_complete == NULL)
    {
        yg_protocol_service_write_result(result, 2U, 0U);
        return YG_PROTOCOL_OK;
    }
    boot->boot_service.request(boot->boot_service.context, &command, &reply);
    if (yg_protocol_boot_encode_reply(
            &reply, result->response_payload, sizeof(result->response_payload), NULL) !=
        YG_PROTOCOL_OK)
    {
        if (reply.ack_ticket != 0U)
        {
            boot->boot_service.tx_complete(boot->boot_service.context, reply.ack_ticket, false);
        }
        yg_protocol_service_write_result(result, 8U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (reply.result == BOOT_ENTRY_READY)
    {
        boot->boot_ticket = reply.ack_ticket;
        boot->boot_ready_ms = boot->service_now_ms;
        boot->boot_reply_sequence = request->sequence;
        boot->boot_reply_destination = request->source_node;
    }
    result->response_payload_length = YG_PROTOCOL_BOOT_REPLY_SIZE;
    return YG_PROTOCOL_OK;
}

void yg_protocol_boot_complete(yg_protocol_boot_context_t *boot, uint64_t ticket, bool success)
{
    if (boot == NULL || ticket == 0U || boot->boot_ticket != ticket)
    {
        return;
    }
    boot->boot_ticket = 0U;
    if (boot->boot_service.tx_complete != NULL)
    {
        boot->boot_service.tx_complete(boot->boot_service.context, ticket, success);
    }
}
