/* 电机快照到线路状态的单向适配；不消费旧邮箱，不改变控制状态或发送节拍。 */
#include "yg_protocol_motor_status.h"

#include <limits.h>
#include <math.h>
#include <string.h>
#include "yg_protocol_readonly.h"

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
    if (source->state > 9U || source->mode > 8U || source->faults > 0x3FFFFU ||
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
