/* job 服务边界：显式注入后端，不读取全局状态或访问硬件。 */
#include "yg_protocol_job.h"
#include "yg_protocol_byte_order.h"

yg_protocol_result_t yg_protocol_plan_decode(const yg_protocol_message_t *message,
                                             yg_protocol_plan_request_t *request)
{
    yg_protocol_plan_request_t decoded = {0};
    const uint8_t *payload;
    if (message == NULL || request == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (message->message_type != YG_PROTOCOL_JOB_TYPE_PLAN)
    {
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    if (message->payload == NULL || message->payload_length < YG_PROTOCOL_PLAN_QUERY_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    payload = message->payload;
    decoded.operation = payload[1];
    if ((decoded.operation == YG_PROTOCOL_PLAN_START &&
         message->payload_length != YG_PROTOCOL_PLAN_START_SIZE) ||
        (decoded.operation != YG_PROTOCOL_PLAN_START &&
         message->payload_length != YG_PROTOCOL_PLAN_QUERY_SIZE))
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    decoded.steps_or_ticket = YgProtocol_ReadU32Le(payload + 4U);
    if (decoded.operation == YG_PROTOCOL_PLAN_START)
    {
        decoded.boot_epoch = YgProtocol_ReadU64Le(payload + 12U);
    }
    if (payload[0] != 1U || YgProtocol_ReadU16Le(payload + 2U) != 0U ||
        YgProtocol_ReadU32Le(payload + 8U) != 0U || decoded.steps_or_ticket == 0U ||
        (decoded.operation != YG_PROTOCOL_PLAN_START &&
         decoded.operation != YG_PROTOCOL_PLAN_STATUS &&
         decoded.operation != YG_PROTOCOL_PLAN_CANCEL) ||
        (decoded.operation == YG_PROTOCOL_PLAN_START &&
         (decoded.boot_epoch == 0U || (decoded.steps_or_ticket & ~UINT32_C(0xFF)) != 0U)))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    decoded.source_node = message->source_node;
    decoded.sequence = message->sequence;
    *request = decoded;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_plan_encode_reply(const yg_protocol_plan_reply_t *reply,
                                                   uint8_t *payload,
                                                   size_t capacity,
                                                   size_t *written)
{
    if (reply == NULL || payload == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (capacity < YG_PROTOCOL_PLAN_REPLY_SIZE)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    YgProtocol_WriteU16Le(payload, yg_protocol_service_result_code(reply->status));
    YgProtocol_WriteU16Le(payload + 2U, reply->detail);
    YgProtocol_WriteU32Le(payload + 4U, reply->parent_ticket);
    YgProtocol_WriteU32Le(payload + 8U, reply->step_ticket);
    YgProtocol_WriteU32Le(payload + 12U, reply->requested_steps);
    YgProtocol_WriteU32Le(payload + 16U, reply->completed_steps);
    payload[20] = reply->step;
    payload[21] = reply->phase;
    payload[22] = reply->effects;
    payload[23] = 0U;
    if (written != NULL)
    {
        *written = YG_PROTOCOL_PLAN_REPLY_SIZE;
    }
    return YG_PROTOCOL_OK;
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
         (reply->token == 0U || reply->kind < YG_PROTOCOL_JOB_KIND_ENCODER_LUT ||
          reply->kind > YG_PROTOCOL_JOB_KIND_FRICTION ||
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
