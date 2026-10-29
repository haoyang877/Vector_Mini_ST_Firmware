/* job 服务边界：显式注入后端，不读取全局状态或访问硬件。 */
#include "yg_protocol_job.h"
#include <string.h>
#include "yg_protocol_common.h"

static bool active_job_kind(uint16_t kind)
{
    return kind == YG_PROTOCOL_JOB_KIND_ENCODER_LUT ||
           kind == YG_PROTOCOL_JOB_KIND_ELECTRICAL_ZERO ||
           kind == YG_PROTOCOL_JOB_KIND_MECHANICAL_ZERO || kind == YG_PROTOCOL_JOB_KIND_SAVE ||
           kind == YG_PROTOCOL_JOB_KIND_DEFAULTS;
}

yg_protocol_result_t yg_protocol_job_decode(const yg_protocol_message_t *message,
                                            yg_protocol_job_request_t *request)
{
    yg_protocol_job_request_t candidate = {.operation = YG_PROTOCOL_JOB_START};
    if (message == NULL || request == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (message->payload_length != YG_PROTOCOL_JOB_REQUEST_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    if (message->payload == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    switch (message->message_type)
    {
    case YG_PROTOCOL_JOB_TYPE_START:
        candidate.operation = YG_PROTOCOL_JOB_START;
        candidate.kind = YgProtocol_ReadU16Le(message->payload);
        if (candidate.kind == 0U || YgProtocol_ReadU16Le(message->payload + 2U) != 0U)
        {
            return YG_PROTOCOL_INVALID_FIELD;
        }
        break;
    case YG_PROTOCOL_JOB_TYPE_STATUS:
    case YG_PROTOCOL_JOB_TYPE_CANCEL:
        candidate.operation = message->message_type == YG_PROTOCOL_JOB_TYPE_STATUS
                                  ? YG_PROTOCOL_JOB_QUERY
                                  : YG_PROTOCOL_JOB_CANCEL;
        candidate.job_id = YgProtocol_ReadU32Le(message->payload);
        if (candidate.job_id == 0U)
        {
            return YG_PROTOCOL_INVALID_FIELD;
        }
        break;
    default:
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    candidate.source_node = message->source_node;
    candidate.sequence = message->sequence;
    *request = candidate;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_job_encode_reply(const yg_protocol_job_reply_t *reply,
                                                  uint8_t *payload,
                                                  size_t capacity,
                                                  size_t *written)
{
    if (reply == NULL || payload == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (reply->phase > YG_PROTOCOL_JOB_PHASE_FAILED ||
        reply->effects > YG_PROTOCOL_JOB_EFFECT_COMMITTED ||
        ((reply->status == YG_PROTOCOL_SERVICE_OK ||
          reply->status == YG_PROTOCOL_SERVICE_ACCEPTED) &&
         (reply->token == 0U || !active_job_kind(reply->kind) ||
          reply->phase == YG_PROTOCOL_JOB_PHASE_NONE)))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (capacity < YG_PROTOCOL_JOB_REPLY_SIZE)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    YgProtocol_WriteU16Le(payload, yg_protocol_service_result_code(reply->status));
    YgProtocol_WriteU16Le(payload + 2U, reply->detail);
    YgProtocol_WriteU32Le(payload + 4U, reply->token);
    YgProtocol_WriteU16Le(payload + 8U, reply->kind);
    payload[10] = reply->phase;
    payload[11] = reply->effects;
    if (written != NULL)
    {
        *written = YG_PROTOCOL_JOB_REPLY_SIZE;
    }
    return YG_PROTOCOL_OK;
}

yg_protocol_service_status_t yg_protocol_job_call(const yg_protocol_job_service_t *service,
                                                  const yg_protocol_job_request_t *request,
                                                  yg_protocol_job_reply_t *reply)
{
    if (reply == NULL)
    {
        return YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
    }
    *reply = (yg_protocol_job_reply_t){.status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT};
    if (service == NULL || request == NULL ||
        (unsigned)request->operation > (unsigned)YG_PROTOCOL_JOB_QUERY ||
        (request->operation == YG_PROTOCOL_JOB_START &&
         (request->kind == 0U || request->job_id != 0U)) ||
        (request->operation != YG_PROTOCOL_JOB_START &&
         (request->job_id == 0U || request->kind != 0U)))
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

static void write_job_error(yg_protocol_service_result_t *response, uint16_t code, uint16_t detail)
{
    yg_protocol_service_write_result(response, code, detail);
    memset(response->response_payload + 4U, 0, YG_PROTOCOL_JOB_REPLY_SIZE - 4U);
    response->response_payload_length = YG_PROTOCOL_JOB_REPLY_SIZE;
}

yg_protocol_result_t yg_protocol_job_handle(void *context,
                                            const yg_protocol_message_t *request,
                                            yg_protocol_service_result_t *result)
{
    yg_protocol_job_service_t *service = context;
    yg_protocol_job_request_t command;
    yg_protocol_job_reply_t reply;
    yg_protocol_result_t decoded;
    if (!yg_protocol_service_request_valid(service, request, result))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_PROTOCOL_MASTER_NODE_ID)
    {
        write_job_error(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    decoded = yg_protocol_job_decode(request, &command);
    if (decoded != YG_PROTOCOL_OK)
    {
        write_job_error(result,
                        decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U,
                        YG_PROTOCOL_JOB_DETAIL_BAD_FIELD);
        return YG_PROTOCOL_OK;
    }
    (void)yg_protocol_job_call(service, &command, &reply);
    if (yg_protocol_job_encode_reply(
            &reply, result->response_payload, sizeof(result->response_payload), NULL) !=
        YG_PROTOCOL_OK)
    {
        write_job_error(result, 8U, 0U);
        return YG_PROTOCOL_OK;
    }
    result->response_payload_length = YG_PROTOCOL_JOB_REPLY_SIZE;
    return YG_PROTOCOL_OK;
}
