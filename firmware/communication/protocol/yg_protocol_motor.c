/* motor 服务边界：显式注入后端，不读取全局状态或访问硬件。 */
#include "yg_protocol_motor.h"

#include <stddef.h>

static uint32_t read_u32(const uint8_t *payload)
{
    return (uint32_t)payload[0] | ((uint32_t)payload[1] << 8U) | ((uint32_t)payload[2] << 16U) |
           ((uint32_t)payload[3] << 24U);
}

yg_protocol_result_t yg_protocol_motor_decode_enable(const yg_protocol_message_t *message,
                                                     bool *enabled)
{
    if (message == NULL || enabled == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (message->message_type != YG_PROTOCOL_MOTOR_TYPE_ENABLE)
    {
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    if (message->payload_length != YG_PROTOCOL_MOTOR_ENABLE_REQUEST_SIZE ||
        message->payload == NULL)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    if (message->payload[0] > 1U || message->payload[1] != 0U || message->payload[2] != 0U ||
        message->payload[3] != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    *enabled = message->payload[0] != 0U;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_motor_decode_control(const yg_protocol_message_t *message,
                                                      yg_protocol_motor_request_t *request)
{
    yg_protocol_motor_request_t decoded = {.operation = YG_PROTOCOL_MOTOR_STOP};
    const uint8_t *payload;
    if (message == NULL || request == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (message->message_type != YG_PROTOCOL_MOTOR_TYPE_CONTROL)
    {
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    if (message->payload_length != YG_PROTOCOL_MOTOR_CONTROL_REQUEST_SIZE ||
        message->payload == NULL)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    payload = message->payload;
    decoded.operation = YG_PROTOCOL_MOTOR_SET_CONTROL;
    decoded.source_node = message->source_node;
    decoded.sequence = message->sequence;
    decoded.mode = payload[0];
    decoded.position_mrad = (int32_t)read_u32(payload + 4U);
    decoded.speed_mrad_s = (int32_t)read_u32(payload + 8U);
    decoded.iq_mA = (int32_t)read_u32(payload + 12U);
    decoded.v_q_mV = (int32_t)read_u32(payload + 16U);
    if (decoded.mode < 1U || decoded.mode > 6U || payload[1] != 0U || payload[2] != 0U ||
        payload[3] != 0U ||
        (decoded.mode != 3U && decoded.mode != 4U && decoded.position_mrad != 0) ||
        (decoded.mode != 2U && decoded.mode != 4U && decoded.speed_mrad_s != 0) ||
        (decoded.mode != 1U && decoded.mode != 4U && decoded.iq_mA != 0) ||
        (decoded.mode != 6U && decoded.v_q_mV != 0))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    *request = decoded;
    return YG_PROTOCOL_OK;
}

static void put_u16(uint8_t *payload, uint16_t value)
{
    payload[0] = (uint8_t)value;
    payload[1] = (uint8_t)(value >> 8U);
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
    if (service == NULL || request == NULL ||
        (unsigned)request->operation > (unsigned)YG_PROTOCOL_MOTOR_SET_CONTROL)
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
    if (reply == NULL || payload == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (capacity < YG_PROTOCOL_MOTOR_REPLY_SIZE)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    put_u16(payload, yg_protocol_motor_result_code(reply->status));
    put_u16(payload + 2U, reply->detail);
    if (written != NULL)
    {
        *written = YG_PROTOCOL_MOTOR_REPLY_SIZE;
    }
    return YG_PROTOCOL_OK;
}

uint16_t yg_protocol_motor_result_code(yg_protocol_service_status_t status)
{
    switch (status)
    {
    case YG_PROTOCOL_SERVICE_OK:
    case YG_PROTOCOL_SERVICE_ACCEPTED:
    case YG_PROTOCOL_SERVICE_UNSUPPORTED:
        return (uint16_t)status;
    case YG_PROTOCOL_SERVICE_INVALID_ARGUMENT:
        return 4U;
    case YG_PROTOCOL_SERVICE_BUSY:
        return 6U;
    case YG_PROTOCOL_SERVICE_DENIED:
        return 5U;
    case YG_PROTOCOL_SERVICE_FAILED:
    default:
        return 8U;
    }
}
