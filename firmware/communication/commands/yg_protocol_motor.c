#include "yg_protocol_crc.h"
#include "yg_protocol_motor.h"
#include "yg_protocol_common.h"
#include <stddef.h>
#include <limits.h>
#include <math.h>
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

/* 当前线路缩放仅为 100/1000。拆解 binary32 的符号、尾数和指数，整数乘积
 * 精确表示缩放结果，以最高丢弃位实现半值远离零，避免引入软件双精度库。
 * 输入非有限或舍入后越界返回 INT32_MIN；状态 i16/i32 均保留最小值作无效哨兵。
 * 循环有界且无浮点近似；与参数编码的加半后边界检查语义不同，不合并入口。 */
static int32_t measurement(float value, uint32_t scale, int32_t minimum, int32_t maximum)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    unsigned exponent = (bits >> 23U) & 255U;
    if (exponent == 255U)
    {
        return INT32_MIN;
    }
    uint64_t product = (uint64_t)((bits & 0x7fffffU) | (exponent ? 0x800000U : 0U)) * scale;
    int shift = exponent ? 150 - (int)exponent : 149;
    uint64_t magnitude;
    if (shift > 63)
    {
        magnitude = 0U;
    }
    else if (shift > 0)
    {
        uint64_t midpoint = (uint64_t)1U << (shift - 1);
        magnitude = (product >> shift) + ((product & midpoint) != 0U);
    }
    else
    {
        if (shift < -31 || product > (UINT32_MAX >> -shift))
        {
            return INT32_MIN;
        }
        magnitude = product << -shift;
    }
    bool negative = (bits >> 31U) != 0U;

    uint32_t limit = negative ? 0U - (uint32_t)minimum : (uint32_t)maximum;
    if (magnitude > limit)
    {
        return INT32_MIN;
    }
    return negative ? -(int32_t)magnitude : (int32_t)magnitude;
}

static int32_t feedback_i32(float sample, bool available, uint16_t *valid_bits, uint8_t bit)
{
    int32_t value = available ? measurement(sample, 1000U, -INT32_MAX, INT32_MAX) : INT32_MIN;
    if (value != INT32_MIN)
    {
        *valid_bits |= (uint16_t)(1U << bit);
    }
    return value;
}

static int16_t
feedback_i16(float sample, uint32_t scale, bool available, uint16_t *valid_bits, uint8_t bit)
{
    int32_t value = available ? measurement(sample, scale, -INT16_MAX, INT16_MAX) : INT32_MIN;
    if (value == INT32_MIN)
    {
        return INT16_MIN;
    }
    *valid_bits |= (uint16_t)(1U << bit);
    return (int16_t)value;
}

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
    if (source->state > 9U || source->mode > 10U || source->faults > 0x3FFFFU ||
        (source->measurement_valid_bits & ~YG_PROTOCOL_MOTOR_MEASUREMENT_MASK) != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    value.fault_code = source->faults;
    value.motor_state = source->state;
    value.control_mode = source->mode;
    if (!source->sample_available)
    {
        *feedback = value;
        return YG_PROTOCOL_OK;
    }
    value.reference_position_mrad =
        feedback_i32(source->sample.position_target, true, &value.valid_bits, 0U);
    value.actual_position_mrad = feedback_i32(source->sample.position_feedback,
                                              (source->measurement_valid_bits & 0x01U) != 0U,
                                              &value.valid_bits,
                                              1U);
    value.reference_speed_mrad_s =
        feedback_i32(source->sample.speed_target, true, &value.valid_bits, 2U);
    value.actual_speed_mrad_s = feedback_i32(source->sample.speed_feedback_rad_s,
                                             (source->measurement_valid_bits & 0x02U) != 0U,
                                             &value.valid_bits,
                                             3U);
    value.bus_voltage_cV = feedback_i16(source->sample.bus_voltage,
                                        100U,
                                        (source->measurement_valid_bits & 0x10U) != 0U,
                                        &value.valid_bits,
                                        4U);
    value.bus_current_mA = feedback_i16(source->sample.bus_current,
                                        1000U,
                                        (source->measurement_valid_bits & 0x40U) != 0U,
                                        &value.valid_bits,
                                        5U);
    value.reference_iq_mA =
        feedback_i16(source->sample.current_reference, 1000U, true, &value.valid_bits, 6U);
    value.actual_iq_mA = feedback_i16(source->sample.current_feedback,
                                      1000U,
                                      (source->measurement_valid_bits & 0x04U) != 0U,
                                      &value.valid_bits,
                                      7U);
    value.mcu_temperature_centi_c = feedback_i16(source->sample.temperature,
                                                 100U,
                                                 (source->measurement_valid_bits & 0x20U) != 0U,
                                                 &value.valid_bits,
                                                 8U);
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

static void prepare_completed_control(yg_protocol_motor_context_t *motor)
{
    if (!motor->control_completion_pending || motor->feedback_pending)
    {
        return;
    }
    const MotorControlProgress *progress = &motor->motor_status_source->control_progress;
    if (progress->source_node != motor->control_completion_source ||
        progress->sequence != motor->control_completion_sequence ||
        (progress->state != MOTOR_CONTROL_PROGRESS_APPLIED &&
         progress->state != MOTOR_CONTROL_PROGRESS_REJECTED))
    {
        return;
    }
    uint16_t code = 8U;
    if (progress->state == MOTOR_CONTROL_PROGRESS_APPLIED)
    {
        code = 0U;
    }
    else
    {
        switch (progress->result)
        {
        case MOTOR_CONTROL_INVALID:
            code = 4U;
            break;
        case MOTOR_CONTROL_DENIED:
            code = 5U;
            break;
        case MOTOR_CONTROL_BUSY:
            code = 6U;
            break;
        case MOTOR_CONTROL_OUT_OF_RANGE:
            code = 11U;
            break;
        case MOTOR_CONTROL_FAULT:
            code = 17U;
            break;
        default:
            break;
        }
    }
    if (prepare_control_feedback(motor, progress->source_node, progress->sequence, code) !=
        YG_PROTOCOL_OK)
    {
        return;
    }
    yg_protocol_motor_history_t *entry =
        &motor->control_history[progress->sequence % YG_PROTOCOL_MOTOR_HISTORY_SIZE];
    if (entry->valid && entry->sequence == progress->sequence && entry->result == 1U)
    {
        entry->detail = code;
    }
    motor->control_completion_pending = false;
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
    yg_protocol_service_reply_t reply;
    yg_protocol_motor_history_t *entry;
    yg_protocol_result_t decoded;
    uint16_t code, feedback_code, detail = 0U, digest = 0U;
    bool defer_feedback = false;
    if (!yg_protocol_service_request_valid(motor, request, result))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    code = 7U;
    feedback_code = code;
    if (request->source_node == YG_PROTOCOL_MASTER_NODE_ID)
    {
        decoded = yg_protocol_motor_decode_control(request, &command);
        code = decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U;
        feedback_code = code;
        if (decoded == YG_PROTOCOL_OK)
        {
            digest = yg_protocol_crc16(request->payload, request->payload_length);
            entry = &motor->control_history[request->sequence % YG_PROTOCOL_MOTOR_HISTORY_SIZE];
            if (entry->valid && entry->sequence == request->sequence)
            {
                code = entry->request_crc == digest ? entry->result : 4U;
                detail = entry->request_crc == digest && code != 1U ? entry->detail : 0U;
                feedback_code = entry->request_crc == digest && code == 1U ? entry->detail : code;
            }
            else if (motor->control_sequence_seen &&
                     (request->sequence == motor->last_control_sequence ||
                      (uint16_t)(request->sequence - motor->last_control_sequence) >= 0x8000U))
            {
                code = 4U; /* 未缓存的相同或旧序号不得在终态发布后重新执行。 */
                feedback_code = code;
            }
            else
            {
                if (motor->control_completion_pending)
                {
                    code = 6U;
                }
                else
                {
                    (void)yg_protocol_motor_call(&motor->motor_service, &command, &reply);
                    code = yg_protocol_motor_result_code(reply.status);
                    detail = reply.detail;
                }
                feedback_code = code;
                /* 不能让拥塞拒绝覆盖仍待发布终态的历史槽。 */
                if (!motor->control_completion_pending ||
                    request->sequence % YG_PROTOCOL_MOTOR_HISTORY_SIZE !=
                        motor->control_completion_sequence % YG_PROTOCOL_MOTOR_HISTORY_SIZE)
                {
                    *entry = (yg_protocol_motor_history_t){
                        request->sequence, digest, code, code == 1U ? 1U : detail, true};
                }
                if (code == 1U)
                {
                    motor->control_completion_pending = true;
                    motor->control_completion_source = request->source_node;
                    motor->control_completion_sequence = request->sequence;
                    defer_feedback = true;
                    detail = 0U;
                }
                motor->last_control_sequence = request->sequence;
                motor->control_sequence_seen = true;
            }
        }
    }
    if (!defer_feedback &&
        prepare_control_feedback(motor, request->source_node, request->sequence, feedback_code) !=
            YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    yg_protocol_service_write_result(result, code, detail);
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
    prepare_completed_control(motor);
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
