/* 电机快照到线路状态的单向适配；不消费旧邮箱，不改变控制状态或发送节拍。 */
#include "yg_protocol_motor_status.h"

#include <limits.h>
#include <math.h>
#include <string.h>
#include "yg_protocol_readonly.h"

/* 使用 double 避免 float 的 INT32_MAX 表示误差导致越界强转；不复用旧协议的饱和转换。 */
static int32_t measurement(float value, double scale, int32_t minimum, int32_t maximum)
{
    double rounded;
    if (!isfinite(value))
    {
        return INT32_MIN;
    }
    rounded = round((double)value * scale);
    if (rounded < (double)minimum || rounded > (double)maximum)
    {
        return INT32_MIN;
    }
    return (int32_t)rounded;
}

static int32_t feedback_i32(float sample, bool available, uint16_t *valid_bits, uint8_t bit)
{
    int32_t value = available ? measurement(sample, 1000.0, -INT32_MAX, INT32_MAX) : INT32_MIN;
    if (value != INT32_MIN)
    {
        *valid_bits |= (uint16_t)(1U << bit);
    }
    return value;
}

static int16_t
feedback_i16(float sample, double scale, bool available, uint16_t *valid_bits, uint8_t bit)
{
    int32_t value = available ? measurement(sample, scale, -INT16_MAX, INT16_MAX) : INT32_MIN;
    if (value == INT32_MIN)
    {
        return INT16_MIN;
    }
    *valid_bits |= (uint16_t)(1U << bit);
    return (int16_t)value;
}

static void write_u16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8U);
}

static void write_u32(uint8_t *bytes, uint32_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8U);
    bytes[2] = (uint8_t)(value >> 16U);
    bytes[3] = (uint8_t)(value >> 24U);
}

yg_protocol_result_t
yg_protocol_motor_status_convert(const yg_protocol_motor_status_source_t *source,
                                 yg_protocol_readonly_motor_state_t *state)
{
    yg_protocol_readonly_motor_state_t candidate = {0};
    int32_t bus_mV, bus_mA, temperature_centi_c;
    if (source == NULL || state == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (source->state > 9U || source->mode > 8U || source->faults > 0x3FFFFU ||
        (source->measurement_valid_bits & ~YG_PROTOCOL_MOTOR_MEASUREMENT_MASK) != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (!source->sample_available)
    {
        return YG_PROTOCOL_DATA_UNAVAILABLE;
    }
    candidate.boot_id = source->boot_id;
    candidate.sample_counter = source->sample_counter;
    candidate.faults = source->faults;
    candidate.state = source->state;
    candidate.mode = source->mode;
    candidate.last_applied_sequence = source->target_applied ? source->last_applied_sequence : 0U;
    candidate.valid_bits = source->target_applied ? 0x08U : 0U;
    candidate.position_mrad =
        (source->measurement_valid_bits & 0x01U) != 0U
            ? measurement(source->sample.position_feedback, 1000.0, -INT32_MAX, INT32_MAX)
            : INT32_MIN;
    candidate.speed_mrad_s =
        (source->measurement_valid_bits & 0x02U) != 0U
            ? measurement(source->sample.speed_feedback, 1000.0, -INT32_MAX, INT32_MAX)
            : INT32_MIN;
    candidate.iq_mA =
        (source->measurement_valid_bits & 0x04U) != 0U
            ? measurement(source->sample.current_feedback, 1000.0, -INT32_MAX, INT32_MAX)
            : INT32_MIN;
    bus_mV = (source->measurement_valid_bits & 0x10U) != 0U
                 ? measurement(source->sample.bus_voltage, 1000.0, 0, UINT16_MAX - 1)
                 : INT32_MIN;
    bus_mA = (source->measurement_valid_bits & 0x40U) != 0U
                 ? measurement(source->sample.bus_current, 1000.0, -INT16_MAX, INT16_MAX)
                 : INT32_MIN;
    temperature_centi_c =
        (source->measurement_valid_bits & 0x20U) != 0U
            ? measurement(source->sample.temperature, 100.0, -INT16_MAX, INT16_MAX)
            : INT32_MIN;
    candidate.bus_mV = bus_mV == INT32_MIN ? UINT16_MAX : (uint16_t)bus_mV;
    candidate.bus_mA = bus_mA == INT32_MIN ? INT16_MIN : (int16_t)bus_mA;
    candidate.temperature_centi_c =
        temperature_centi_c == INT32_MIN ? INT16_MIN : (int16_t)temperature_centi_c;
    if (candidate.position_mrad != INT32_MIN)
    {
        candidate.valid_bits |= 0x01U;
    }
    if (candidate.speed_mrad_s != INT32_MIN)
    {
        candidate.valid_bits |= 0x02U;
    }
    if (candidate.iq_mA != INT32_MIN)
    {
        candidate.valid_bits |= 0x04U;
    }
    if (bus_mV != INT32_MIN)
    {
        candidate.valid_bits |= 0x10U;
    }
    if (temperature_centi_c != INT32_MIN)
    {
        candidate.valid_bits |= 0x20U;
    }
    if (bus_mA != INT32_MIN)
    {
        candidate.valid_bits |= 0x40U;
    }
    *state = candidate;
    return YG_PROTOCOL_OK;
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
    write_u16(encoded, feedback->result);
    write_u16(encoded + 2U, feedback->correlated_seq);
    write_u32(encoded + 4U, feedback->fault_code);
    write_u32(encoded + 8U, (uint32_t)feedback->reference_position_mrad);
    write_u32(encoded + 12U, (uint32_t)feedback->actual_position_mrad);
    write_u32(encoded + 16U, (uint32_t)feedback->reference_speed_mrad_s);
    write_u32(encoded + 20U, (uint32_t)feedback->actual_speed_mrad_s);
    write_u16(encoded + 24U, feedback->valid_bits);
    write_u16(encoded + 26U, (uint16_t)feedback->bus_voltage_cV);
    write_u16(encoded + 28U, (uint16_t)feedback->bus_current_mA);
    write_u16(encoded + 30U, (uint16_t)feedback->reference_iq_mA);
    write_u16(encoded + 32U, (uint16_t)feedback->actual_iq_mA);
    write_u16(encoded + 34U, (uint16_t)feedback->mcu_temperature_centi_c);
    write_u16(encoded + 36U, (uint16_t)feedback->motor_temperature_centi_c);
    write_u16(encoded + 38U, (uint16_t)feedback->v_q_mV);
    write_u16(encoded + 40U, (uint16_t)feedback->v_d_mV);
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
    value.actual_speed_mrad_s = feedback_i32(source->sample.speed_feedback,
                                             (source->measurement_valid_bits & 0x02U) != 0U,
                                             &value.valid_bits,
                                             3U);
    value.bus_voltage_cV = feedback_i16(source->sample.bus_voltage,
                                        100.0,
                                        (source->measurement_valid_bits & 0x10U) != 0U,
                                        &value.valid_bits,
                                        4U);
    value.bus_current_mA = feedback_i16(source->sample.bus_current,
                                        1000.0,
                                        (source->measurement_valid_bits & 0x40U) != 0U,
                                        &value.valid_bits,
                                        5U);
    value.reference_iq_mA =
        feedback_i16(source->sample.current_reference, 1000.0, true, &value.valid_bits, 6U);
    value.actual_iq_mA = feedback_i16(source->sample.current_feedback,
                                      1000.0,
                                      (source->measurement_valid_bits & 0x04U) != 0U,
                                      &value.valid_bits,
                                      7U);
    value.mcu_temperature_centi_c = feedback_i16(source->sample.temperature,
                                                 100.0,
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
    yg_protocol_readonly_motor_state_t state;
    yg_protocol_result_t result;
    uint8_t response[YG_PROTOCOL_MOTOR_RESPONSE_SIZE] = {0};
    uint16_t business_result = 0U, detail = 0U;
    uint32_t length = 12U;
    if (source == NULL || request == NULL || payload == NULL || payload_length == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (request->message_type != YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE)
    {
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    if (request->payload == NULL || request->payload_length < 8U)
    {
        /* 缺少完整 Q 时无法可靠关联事务，不制造应答。 */
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    memcpy(response, request->payload, 8U);
    if (request->payload_length != 8U)
    {
        business_result = 3U; /* BAD_LENGTH */
    }
    else if ((response[0] | response[1] | response[2] | response[3]) != 0U)
    {
        business_result = 4U; /* 此阶段只支持无会话查询。 */
    }
    else if ((response[4] | response[5] | response[6] | response[7]) == 0U)
    {
        business_result = 4U;
        detail = 4U;
    }
    else
    {
        result = yg_protocol_motor_status_convert(source, &state);
        if (result == YG_PROTOCOL_DATA_UNAVAILABLE)
        {
            business_result = 6U; /* BUSY */
        }
        else if (result != YG_PROTOCOL_OK)
        {
            return result;
        }
        else
        {
            result = yg_protocol_readonly_encode_motor_state(
                &state, response + 12U, YG_PROTOCOL_READONLY_MOTOR_STATE_SIZE, NULL);
            if (result != YG_PROTOCOL_OK)
            {
                return result;
            }
            length = YG_PROTOCOL_MOTOR_RESPONSE_SIZE;
        }
    }
    if (*payload_length < length)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    write_u16(response + 8U, business_result);
    write_u16(response + 10U, detail);
    memcpy(payload, response, length);
    *payload_length = length;
    return YG_PROTOCOL_OK;
}
