/* motor 服务边界：显式注入后端，不读取全局状态或访问硬件。 */
#include "yg_protocol_motor.h"

static void put_u16(uint8_t *payload, uint16_t value)
{
    payload[0] = (uint8_t)value;
    payload[1] = (uint8_t)(value >> 8U);
}

static void put_u32(uint8_t *payload, uint32_t value)
{
    payload[0] = (uint8_t)value;
    payload[1] = (uint8_t)(value >> 8U);
    payload[2] = (uint8_t)(value >> 16U);
    payload[3] = (uint8_t)(value >> 24U);
}

yg_protocol_service_status_t yg_protocol_motor_call(const yg_protocol_motor_service_t *service,
                                                    const yg_protocol_motor_request_t *request,
                                                    yg_protocol_service_reply_t *reply)
{
    if (reply == NULL)
    {
        return YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
    }
    *reply = (yg_protocol_service_reply_t){.status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT};
    reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
    if (service == NULL || request == NULL || request->operation > YG_PROTOCOL_MOTOR_SET_TARGET)
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

yg_protocol_result_t yg_protocol_motor_encode_reply(const yg_protocol_service_reply_t *reply,
                                                    uint8_t *payload,
                                                    size_t capacity,
                                                    size_t *written)
{
    if (reply == NULL || payload == NULL || capacity < YG_PROTOCOL_MOTOR_REPLY_SIZE)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    payload[0] = (uint8_t)reply->status;
    payload[1] = 0U;
    put_u16(payload + 2U, reply->detail);
    put_u32(payload + 4U, reply->token);
    put_u32(payload + 8U, reply->revision);
    put_u32(payload + 12U, (uint32_t)reply->value);
    if (written != NULL)
    {
        *written = YG_PROTOCOL_MOTOR_REPLY_SIZE;
    }
    return YG_PROTOCOL_OK;
}
