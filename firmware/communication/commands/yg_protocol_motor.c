#include "yg_protocol_motor.h"
#include "yg_protocol_common.h"
#include <stddef.h>
#include <limits.h>
#include <string.h>
#include "yg_protocol_readonly.h"

/* motor 服务边界：显式注入后端，不读取全局状态或访问硬件。 */

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
    if ((message->payload_length != YG_PROTOCOL_MOTOR_CONTROL_REQUEST_SIZE &&
         message->payload_length != YG_PROTOCOL_MOTOR_OPEN_LOOP_REQUEST_SIZE) ||
        message->payload == NULL)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    payload = message->payload;
    decoded.operation = YG_PROTOCOL_MOTOR_SET_CONTROL;
    decoded.source_node = message->source_node;
    decoded.sequence = message->sequence;
    decoded.mode = payload[0];
    decoded.position_mrad = (int32_t)YgProtocol_ReadU32Le(payload + 4U);
    decoded.speed_mrad_s = (int32_t)YgProtocol_ReadU32Le(payload + 8U);
    decoded.iq_mA = (int32_t)YgProtocol_ReadU32Le(payload + 12U);
    decoded.v_q_mV = (int32_t)YgProtocol_ReadU32Le(payload + 16U);
    bool open_loop = decoded.mode == 9U || decoded.mode == 10U;
    uint32_t expected_bytes = open_loop ? YG_PROTOCOL_MOTOR_OPEN_LOOP_REQUEST_SIZE
                                        : YG_PROTOCOL_MOTOR_CONTROL_REQUEST_SIZE;
    if (message->payload_length != expected_bytes)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    if (open_loop)
    {
        decoded.electrical_frequency_mHz = (int32_t)YgProtocol_ReadU32Le(payload + 20U);
        decoded.id_mA = (int32_t)YgProtocol_ReadU32Le(payload + 24U);
        decoded.vd_mV = (int32_t)YgProtocol_ReadU32Le(payload + 28U);
    }
    if (decoded.mode < 1U || decoded.mode > 10U || decoded.mode == 7U || decoded.mode == 8U ||
        payload[1] != 0U || payload[2] != 0U || payload[3] != 0U ||
        (decoded.mode != 3U && decoded.mode != 4U && decoded.position_mrad != 0) ||
        (decoded.mode != 2U && decoded.mode != 4U && decoded.speed_mrad_s != 0) ||
        (decoded.mode != 1U && decoded.mode != 4U && decoded.mode != 10U && decoded.iq_mA != 0) ||
        (decoded.mode != 6U && decoded.mode != 9U && decoded.v_q_mV != 0) ||
        (decoded.mode != 10U && decoded.id_mA != 0) || (decoded.mode != 9U && decoded.vd_mV != 0))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    *request = decoded;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_motor_decode_start(const yg_protocol_message_t *message,
                                                    yg_protocol_motor_request_t *request)
{
    yg_protocol_motor_request_t decoded = {.operation = YG_PROTOCOL_MOTOR_START_CONTROL};
    if (message == NULL || request == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (message->message_type != YG_PROTOCOL_MOTOR_TYPE_START)
    {
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    if (message->payload_length != YG_PROTOCOL_MOTOR_START_REQUEST_SIZE || message->payload == NULL)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    if (message->payload[1] != 0U || message->payload[2] != 0U || message->payload[3] != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    decoded.operation = YG_PROTOCOL_MOTOR_START_CONTROL;
    decoded.source_node = message->source_node;
    decoded.sequence = message->sequence;
    decoded.mode = message->payload[0];
    *request = decoded;
    return YG_PROTOCOL_OK;
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
        (unsigned)request->operation > (unsigned)YG_PROTOCOL_MOTOR_CLEAR_FAULT)
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
    YgProtocol_WriteU16Le(payload, yg_protocol_motor_result_code(reply->status));
    YgProtocol_WriteU16Le(payload + 2U, reply->detail);
    if (written != NULL)
    {
        *written = YG_PROTOCOL_MOTOR_REPLY_SIZE;
    }
    return YG_PROTOCOL_OK;
}

uint16_t yg_protocol_motor_result_code(yg_protocol_service_status_t status)
{
    return yg_protocol_service_result_code(status);
}

/* 电机快照到线路状态的单向适配；不消费旧邮箱，不改变控制状态或发送节拍。 */

yg_protocol_result_t yg_protocol_motor_feedback_encode(const yg_protocol_motor_feedback_t *feedback,
                                                       uint8_t *payload,
                                                       size_t capacity)
{
    uint8_t encoded[YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE];
    if (feedback == NULL || payload == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (capacity < YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    if ((feedback->valid_bits & ~YG_PROTOCOL_MOTOR_FEEDBACK_VALID_MASK) != 0U ||
        feedback->node_id == 0U || feedback->node_id == YG_PROTOCOL_BROADCAST_NODE_ID)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    YgProtocol_WriteU16Le(encoded, feedback->result);
    YgProtocol_WriteU16Le(encoded + 2U, feedback->correlated_seq);
    YgProtocol_WriteU32Le(encoded + 4U, feedback->fault_code);
    YgProtocol_WriteU32Le(encoded + 8U, (uint32_t)feedback->reference_position_mrad);
    YgProtocol_WriteU32Le(encoded + 12U, (uint32_t)feedback->actual_position_mrad);
    YgProtocol_WriteU32Le(encoded + 16U, (uint32_t)feedback->reference_speed_mrad_s);
    YgProtocol_WriteU32Le(encoded + 20U, (uint32_t)feedback->actual_speed_mrad_s);
    YgProtocol_WriteU16Le(encoded + 24U, feedback->valid_bits);
    YgProtocol_WriteU16Le(encoded + 26U, (uint16_t)feedback->bus_voltage_cV);
    YgProtocol_WriteU16Le(encoded + 28U, (uint16_t)feedback->bus_current_mA);
    YgProtocol_WriteU16Le(encoded + 30U, (uint16_t)feedback->reference_iq_mA);
    YgProtocol_WriteU16Le(encoded + 32U, (uint16_t)feedback->actual_iq_mA);
    YgProtocol_WriteU16Le(encoded + 34U, (uint16_t)feedback->mcu_temperature_centi_c);
    YgProtocol_WriteU16Le(encoded + 36U, (uint16_t)feedback->motor_temperature_centi_c);
    YgProtocol_WriteU16Le(encoded + 38U, (uint16_t)feedback->v_q_mV);
    YgProtocol_WriteU16Le(encoded + 40U, (uint16_t)feedback->v_d_mV);
    encoded[42] = feedback->node_id;
    encoded[43] = feedback->motor_state;
    encoded[44] = feedback->control_mode;
    memcpy(payload, encoded, sizeof(encoded));
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t
yg_protocol_motor_feedback_from_source(const yg_protocol_motor_status_source_t *source,
                                       uint8_t node_id,
                                       uint16_t result,
                                       uint16_t correlated_seq,
                                       yg_protocol_motor_feedback_t *feedback)
{
    yg_protocol_motor_feedback_t value = {0};
    if (feedback == NULL || node_id == 0U || node_id == YG_PROTOCOL_BROADCAST_NODE_ID)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    value.node_id = node_id;
    value.result = result;
    value.correlated_seq = correlated_seq;
    value.reference_position_mrad = INT32_MIN;
    value.actual_position_mrad = INT32_MIN;
    value.reference_speed_mrad_s = INT32_MIN;
    value.actual_speed_mrad_s = INT32_MIN;
    value.bus_voltage_cV = INT16_MIN;
    value.bus_current_mA = INT16_MIN;
    value.reference_iq_mA = INT16_MIN;
    value.actual_iq_mA = INT16_MIN;
    value.mcu_temperature_centi_c = INT16_MIN;
    value.motor_temperature_centi_c = INT16_MIN;
    value.v_q_mV = INT16_MIN;
    value.v_d_mV = INT16_MIN;
    if (source == NULL)
    {
        *feedback = value;
        return YG_PROTOCOL_OK;
    }
    if (source->state > 9U || source->mode > 10U || source->faults > 0x3FFFFU)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (source->sample_available && source->measurements_available)
    {
        value = source->measurements;
        value.node_id = node_id;
        value.result = result;
        value.correlated_seq = correlated_seq;
    }
    value.fault_code = source->faults;
    value.motor_state = source->state;
    value.control_mode = source->mode;
    *feedback = value;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_motor_status_provider(void *context,
                                                       const yg_protocol_message_t *request,
                                                       uint8_t *payload,
                                                       uint32_t *payload_length)
{
    const yg_protocol_motor_status_source_t *source = context;
    yg_protocol_motor_feedback_t feedback;
    yg_protocol_result_t result;
    if (source == NULL || request == NULL || payload == NULL || payload_length == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (request->message_type != YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE)
    {
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    if (*payload_length < YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    result = yg_protocol_motor_feedback_from_source(
        source,
        request->destination_node,
        request->payload_length != 0U ? 3U : (source->sample_available ? 0U : 6U),
        0U,
        &feedback);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    result = yg_protocol_motor_feedback_encode(&feedback, payload, *payload_length);
    if (result == YG_PROTOCOL_OK)
    {
        *payload_length = YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE;
    }
    return result;
}

static yg_protocol_result_t prepare_control_feedback(yg_protocol_motor_context_t *motor,
                                                     uint8_t destination,
                                                     uint16_t sequence,
                                                     uint16_t code)
{
    yg_protocol_motor_feedback_t feedback;
    if (yg_protocol_motor_feedback_from_source(
            motor->motor_status_source, motor->node_id, code, sequence, &feedback) !=
            YG_PROTOCOL_OK ||
        yg_protocol_motor_feedback_encode(
            &feedback, motor->feedback_payload, sizeof(motor->feedback_payload)) != YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    motor->feedback_destination = destination;
    motor->feedback_pending = true;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_motor_handle_action(void *context,
                                                     const yg_protocol_message_t *request,
                                                     yg_protocol_service_result_t *result)
{
    yg_protocol_motor_request_t command = {.operation = YG_PROTOCOL_MOTOR_STOP};
    yg_protocol_service_reply_t reply;
    yg_protocol_motor_context_t *motor = context;
    yg_protocol_result_t decoded;
    bool enabled;

    /* STOP 的非空请求在来源检查前拒绝，保留既有线路处理顺序。 */
    if (!yg_protocol_service_request_valid(motor, request, result) ||
        (request->message_type != YG_PROTOCOL_MOTOR_TYPE_ENABLE && request->payload_length != 0U))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_PROTOCOL_MASTER_NODE_ID)
    {
        yg_protocol_service_write_result(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (request->message_type == YG_PROTOCOL_MOTOR_TYPE_ENABLE)
    {
        decoded = yg_protocol_motor_decode_enable(request, &enabled);
        if (decoded != YG_PROTOCOL_OK)
        {
            yg_protocol_service_write_result(
                result, decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U, 0U);
            return YG_PROTOCOL_OK;
        }
        command.operation = enabled ? YG_PROTOCOL_MOTOR_ENABLE : YG_PROTOCOL_MOTOR_DISABLE;
    }
    else if (request->message_type == YG_PROTOCOL_MOTOR_TYPE_CLEAR_FAULT)
    {
        command.operation = YG_PROTOCOL_MOTOR_CLEAR_FAULT;
    }
    command.source_node = request->source_node;
    command.sequence = request->sequence;
    (void)yg_protocol_motor_call(&motor->motor_service, &command, &reply);
    if (yg_protocol_motor_encode_reply(
            &reply, result->response_payload, sizeof(result->response_payload), NULL) !=
        YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    result->response_payload_length = YG_PROTOCOL_MOTOR_REPLY_SIZE;
    result->response_flags = YG_PROTOCOL_FLAGS_RESPONSE;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_motor_handle_start(void *context,
                                                    const yg_protocol_message_t *request,
                                                    yg_protocol_service_result_t *result)
{
    yg_protocol_motor_context_t *motor = context;
    yg_protocol_motor_request_t command;
    yg_protocol_service_reply_t reply;
    yg_protocol_result_t decoded;
    if (!yg_protocol_service_request_valid(motor, request, result))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_PROTOCOL_MASTER_NODE_ID)
    {
        yg_protocol_service_write_result(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    decoded = yg_protocol_motor_decode_start(request, &command);
    if (decoded != YG_PROTOCOL_OK)
    {
        yg_protocol_service_write_result(
            result, decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (command.mode != 1U && command.mode != 2U && command.mode != 3U && command.mode != 6U &&
        command.mode != 9U && command.mode != 10U)
    {
        yg_protocol_service_write_result(result, 2U, 0U);
        return YG_PROTOCOL_OK;
    }
    (void)yg_protocol_motor_call(&motor->motor_service, &command, &reply);
    if (yg_protocol_motor_encode_reply(
            &reply, result->response_payload, sizeof(result->response_payload), NULL) !=
        YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    result->response_payload_length = YG_PROTOCOL_MOTOR_REPLY_SIZE;
    result->response_flags = YG_PROTOCOL_FLAGS_RESPONSE;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_motor_handle_control(void *context,
                                                      const yg_protocol_message_t *request,
                                                      yg_protocol_service_result_t *result)
{
    yg_protocol_motor_context_t *motor = context;
    yg_protocol_motor_request_t command;
    if (!yg_protocol_service_request_valid(motor, request, result))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    uint16_t code = 7U;
    if (request->source_node == YG_PROTOCOL_MASTER_NODE_ID)
    {
        yg_protocol_result_t decoded = yg_protocol_motor_decode_control(request, &command);
        if (decoded == YG_PROTOCOL_OK)
        {
            yg_protocol_service_reply_t reply;
            (void)yg_protocol_motor_call(&motor->motor_service, &command, &reply);
            code = yg_protocol_motor_result_code(reply.status);
        }
        else
        {
            code = decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U;
        }
    }
    if (prepare_control_feedback(motor, request->source_node, request->sequence, code) !=
        YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    yg_protocol_service_write_result(result, code, 0U);
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_motor_handle_status(void *context,
                                                     const yg_protocol_message_t *request,
                                                     yg_protocol_service_result_t *result)
{
    yg_protocol_motor_context_t *motor = context;
    if (motor == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    yg_protocol_readonly_service_t service = {(void *)motor->motor_status_source,
                                              yg_protocol_motor_status_provider};
    return yg_protocol_readonly_handle(&service, request, result);
}

yg_protocol_result_t yg_protocol_motor_flush_feedback(yg_protocol_motor_context_t *motor,
                                                      yg_protocol_send_t send,
                                                      void *send_context)
{
    yg_protocol_message_t feedback = {0};
    yg_protocol_result_t result;
    if (motor == NULL || send == NULL || motor->motor_status_source == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (!motor->feedback_pending)
    {
        return YG_PROTOCOL_OK;
    }
    feedback.version = YG_PROTOCOL_VERSION;
    feedback.source_node = motor->node_id;
    feedback.destination_node = motor->feedback_destination;
    feedback.message_type = YG_PROTOCOL_READONLY_TYPE_MOTION_FEEDBACK;
    feedback.sequence = motor->feedback_sequence;
    feedback.payload_length = YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE;
    feedback.payload = motor->feedback_payload;
    result = send(send_context, &feedback, 3U);
    if (result == YG_PROTOCOL_OK)
    {
        ++motor->feedback_sequence;
        motor->feedback_pending = false;
    }
    return result;
}
