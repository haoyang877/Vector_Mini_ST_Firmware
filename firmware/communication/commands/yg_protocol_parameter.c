/* parameter 服务边界：显式注入后端，不读取全局状态或访问硬件。 */
#include "yg_protocol_parameter.h"
#include "yg_protocol_common.h"

#include <string.h>

yg_protocol_result_t yg_protocol_parameter_decode(const yg_protocol_message_t *message,
                                                  yg_protocol_parameter_request_t *request)
{
    yg_protocol_parameter_request_t decoded = {.operation = YG_PROTOCOL_PARAMETER_READ};
    uint32_t raw;
    if (message == NULL || request == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    decoded.source_node = message->source_node;
    decoded.sequence = message->sequence;
    switch (message->message_type)
    {
    case YG_PROTOCOL_PARAMETER_TYPE_READ:
        decoded.operation = YG_PROTOCOL_PARAMETER_READ;
        break;
    case YG_PROTOCOL_PARAMETER_TYPE_WRITE:
        decoded.operation = YG_PROTOCOL_PARAMETER_WRITE;
        break;
    case YG_PROTOCOL_PARAMETER_TYPE_SAVE:
        decoded.operation = YG_PROTOCOL_PARAMETER_SAVE;
        break;
    default:
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    if (message->payload == NULL ||
        message->payload_length != (decoded.operation == YG_PROTOCOL_PARAMETER_WRITE ? 12U : 4U))
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    if (decoded.operation == YG_PROTOCOL_PARAMETER_SAVE)
    {
        decoded.expected_revision = YgProtocol_ReadU32Le(message->payload);
    }
    else
    {
        decoded.parameter_id = message->payload[0];
        if (decoded.operation == YG_PROTOCOL_PARAMETER_READ)
        {
            if (message->payload[1] != 0U || message->payload[2] != 0U || message->payload[3] != 0U)
            {
                return YG_PROTOCOL_INVALID_FIELD;
            }
        }
        else
        {
            decoded.value_type = message->payload[1];
            if ((decoded.value_type != YG_PROTOCOL_PARAMETER_VALUE_I32 &&
                 decoded.value_type != YG_PROTOCOL_PARAMETER_VALUE_U32) ||
                message->payload[2] != 4U || message->payload[3] != 0U)
            {
                return YG_PROTOCOL_INVALID_FIELD;
            }
            decoded.expected_revision = YgProtocol_ReadU32Le(message->payload + 4U);
            raw = YgProtocol_ReadU32Le(message->payload + 8U);
            /* 保留u32高位，不做实现相关的无符号到有符号数值裁剪。 */
            memcpy(&decoded.value, &raw, sizeof(raw));
        }
    }
    *request = decoded;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t
yg_protocol_parameter_encode_reply(const yg_protocol_parameter_request_t *request,
                                   const yg_protocol_service_reply_t *reply,
                                   uint8_t *payload,
                                   size_t capacity,
                                   size_t *written)
{
    size_t length = 4U;
    bool completed;
    if (request == NULL || reply == NULL || payload == NULL || written == NULL ||
        request->parameter_id > UINT8_MAX ||
        (unsigned)request->operation > (unsigned)YG_PROTOCOL_PARAMETER_SAVE)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    completed = reply->status == YG_PROTOCOL_SERVICE_OK;
    if (completed || (request->operation == YG_PROTOCOL_PARAMETER_SAVE &&
                      reply->status == YG_PROTOCOL_SERVICE_ACCEPTED))
    {
        length = request->operation == YG_PROTOCOL_PARAMETER_SAVE ? 8U : 16U;
    }
    if (completed && request->operation != YG_PROTOCOL_PARAMETER_SAVE &&
        ((reply->value_type != YG_PROTOCOL_PARAMETER_VALUE_I32 &&
          reply->value_type != YG_PROTOCOL_PARAMETER_VALUE_U32) ||
         (request->operation == YG_PROTOCOL_PARAMETER_WRITE &&
          reply->value_type != request->value_type) ||
         (request->operation == YG_PROTOCOL_PARAMETER_READ &&
          ((reply->access & YG_PROTOCOL_PARAMETER_ACCESS_READ) == 0U ||
           (reply->access & (uint8_t)~0x07U) != 0U))))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->operation == YG_PROTOCOL_PARAMETER_READ &&
        reply->status == YG_PROTOCOL_SERVICE_ACCEPTED)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (capacity < length)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    memset(payload, 0, length);
    YgProtocol_WriteU16Le(payload, yg_protocol_service_result_code(reply->status));
    YgProtocol_WriteU16Le(payload + 2U, reply->detail);
    if (length == 8U)
    {
        YgProtocol_WriteU32Le(payload + 4U, reply->revision);
    }
    else if (length == 16U)
    {
        payload[4] = (uint8_t)request->parameter_id;
        payload[5] = reply->value_type;
        payload[6] = 4U;
        payload[7] = request->operation == YG_PROTOCOL_PARAMETER_READ ? reply->access : 0U;
        YgProtocol_WriteU32Le(payload + 8U, reply->revision);
        YgProtocol_WriteU32Le(payload + 12U, (uint32_t)reply->value);
    }
    *written = length;
    return YG_PROTOCOL_OK;
}

yg_protocol_service_status_t
yg_protocol_parameter_call(const yg_protocol_parameter_service_t *service,
                           const yg_protocol_parameter_request_t *request,
                           yg_protocol_service_reply_t *reply)
{
    if (reply == NULL)
    {
        return YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
    }
    *reply = (yg_protocol_service_reply_t){.status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT};
    if (service == NULL || request == NULL ||
        (unsigned)request->operation > (unsigned)YG_PROTOCOL_PARAMETER_RESTORE_DEFAULTS)
    {
        return reply->status;
    }
    reply->status = YG_PROTOCOL_SERVICE_UNSUPPORTED;
    if (service->handler != NULL)
    {
        /* 后端遗漏结果时不能被误认为成功。 */
        reply->status = YG_PROTOCOL_SERVICE_FAILED;
        service->handler(service->context, request, reply);
    }
    return reply->status;
}

yg_protocol_result_t yg_protocol_parameter_handle(void *context,
                                                  const yg_protocol_message_t *request,
                                                  yg_protocol_service_result_t *result)
{
    yg_protocol_parameter_service_t *service = context;
    yg_protocol_parameter_request_t command;
    yg_protocol_service_reply_t reply;
    yg_protocol_result_t decoded;
    size_t written;
    if (!yg_protocol_service_request_valid(service, request, result))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_PROTOCOL_MASTER_NODE_ID)
    {
        yg_protocol_service_write_result(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    decoded = yg_protocol_parameter_decode(request, &command);
    if (decoded != YG_PROTOCOL_OK)
    {
        yg_protocol_service_write_result(
            result,
            decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U,
            decoded == YG_PROTOCOL_INVALID_LENGTH ? 0U : YG_PROTOCOL_PARAMETER_DETAIL_BAD_FIELD);
        return YG_PROTOCOL_OK;
    }
    /* 幂等与持久作业由参数所有者掌握，通信不缓存一个尚未完成的“成功”。 */
    (void)yg_protocol_parameter_call(service, &command, &reply);
    if (yg_protocol_parameter_encode_reply(&command,
                                           &reply,
                                           result->response_payload,
                                           sizeof(result->response_payload),
                                           &written) != YG_PROTOCOL_OK)
    {
        yg_protocol_service_write_result(result, 8U, 0U);
        return YG_PROTOCOL_OK;
    }
    result->response_payload_length = (uint32_t)written;
    return YG_PROTOCOL_OK;
}
