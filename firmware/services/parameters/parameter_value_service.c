#include "parameter_value_service.h"
#include "motor_axis_profile.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "current_sense_profile.h"
#include "position_cascade_config.h"

/* 只登记有真实字段和已冻结缩放的项；单位缩放不等于允许的产品范围。 */
typedef struct
{
    uint8_t id, access;
    uint16_t offset;
    uint32_t scale;
} ScalarField;

#define READ_FIELD(id, field, scale)                                                               \
    {id, PARAMETER_ACCESS_READ, offsetof(MotorConfiguration, field), scale}
#define WRITE_FIELD(id, field, scale)                                                              \
    {id,                                                                                           \
     PARAMETER_ACCESS_READ | PARAMETER_ACCESS_WRITE_IDLE,                                          \
     offsetof(MotorConfiguration, field),                                                          \
     scale}
#define CAL_FIELD(id, field, scale)                                                                \
    {id,                                                                                           \
     PARAMETER_ACCESS_READ | PARAMETER_ACCESS_CALIBRATION,                                         \
     offsetof(MotorConfiguration, field),                                                          \
     scale}
static const ScalarField s_fields[] = {
    WRITE_FIELD(1U, application.speed_limit_rad_s, 1000U),
    WRITE_FIELD(2U, application.current_limit_a, 1000U),
    READ_FIELD(6U, model.motor_flux_wb, 1000000000U),
    READ_FIELD(7U, model.motor_phase_resistance_ohm, 1000000U),
    READ_FIELD(8U, model.motor_d_inductance_h, 1000000000U),
    READ_FIELD(9U, model.motor_q_inductance_h, 1000000000U),
    READ_FIELD(11U, model.id_kp, 1000U),
    READ_FIELD(12U, model.id_ki, 1000U),
    READ_FIELD(13U, model.iq_kp, 1000U),
    READ_FIELD(14U, model.iq_ki, 1000U),
    READ_FIELD(16U, application.speed_kp, 1000000U),
    READ_FIELD(17U, application.speed_ki, 1000000U),
    READ_FIELD(19U, application.speed_acceleration_rad_s2, 1000U),
    READ_FIELD(20U, application.speed_deceleration_rad_s2, 1000U),
    WRITE_FIELD(21U, application.cascade_position_kp, 1000000U),
    WRITE_FIELD(22U, application.cascade_position_kd, 1000000U),
    WRITE_FIELD(23U, application.position_maximum_speed_rad_s, 1000U),
    READ_FIELD(24U, application.position_acceleration_rad_s2, 1000U),
    READ_FIELD(25U, application.position_deceleration_rad_s2, 1000U),
    READ_FIELD(26U, application.position_error_window_rad, 1000U),
    CAL_FIELD(0x44U, device.phase_a_offset_counts, 1000U),
    CAL_FIELD(0x45U, device.phase_b_offset_counts, 1000U),
    CAL_FIELD(0x46U, device.phase_c_offset_counts, 1000U),
    WRITE_FIELD(0x60U, model.thermal.continuous_vector_a, 1000U),
    WRITE_FIELD(0x61U, model.thermal.peak_a, 1000U),
    WRITE_FIELD(0x62U, model.thermal.heating_tau_s, 1000U),
    WRITE_FIELD(0x63U, model.thermal.derating_enter_heat, 1000000U),
    WRITE_FIELD(0x64U, model.thermal.derating_exit_heat, 1000000U),
    WRITE_FIELD(0x65U, model.thermal.trip_heat, 1000000U),
    WRITE_FIELD(0x66U, model.thermal.clear_heat, 1000000U),
    WRITE_FIELD(0x69U, application.speed_feedforward.gain_a_per_rad_s2, 1000000U),
    WRITE_FIELD(0x6AU, application.speed_feedforward.acceleration_limit_rad_s2, 1000U),
    WRITE_FIELD(0x6BU, application.speed_feedforward.jerk_limit_rad_s3, 1000U),
    WRITE_FIELD(0x6CU, application.speed_feedforward.current_limit_a, 1000U),
};

bool ParameterValue_ConfigurationValid(const MotorConfiguration *configuration)
{
    return configuration != NULL && configuration->application.axis_profile_valid &&
           MotorWindingThermal_Valid(&configuration->model.thermal) &&
           SpeedFeedforward_IsValid(&configuration->application.speed_feedforward) &&
           configuration->model.motor_pole_pairs > 0 &&
           isfinite(configuration->model.motor_phase_resistance_ohm) &&
           configuration->model.motor_phase_resistance_ohm > 0.0f &&
           isfinite(configuration->model.motor_d_inductance_h) &&
           configuration->model.motor_d_inductance_h > 0.0f &&
           isfinite(configuration->model.motor_q_inductance_h) &&
           configuration->model.motor_q_inductance_h > 0.0f &&
           isfinite(configuration->model.motor_flux_wb) &&
           configuration->model.motor_flux_wb > 0.0f &&
           isfinite(configuration->application.current_limit_a) &&
           configuration->application.current_limit_a > 0.0f &&
           configuration->application.current_limit_a <=
               CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A &&
           isfinite(configuration->model.calibration_current_a) &&
           configuration->model.calibration_current_a > 0.0f &&
           configuration->model.calibration_current_a <=
               CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A &&
           isfinite(configuration->application.speed_limit_rad_s) &&
           configuration->application.speed_limit_rad_s > 0.0f &&
           isfinite(configuration->model.id_kp) && configuration->model.id_kp >= 0.0f &&
           isfinite(configuration->model.id_ki) && configuration->model.id_ki >= 0.0f &&
           isfinite(configuration->model.iq_kp) && configuration->model.iq_kp >= 0.0f &&
           isfinite(configuration->model.iq_ki) && configuration->model.iq_ki >= 0.0f &&
           isfinite(configuration->application.speed_acceleration_rad_s2) &&
           configuration->application.speed_acceleration_rad_s2 > 0.0f &&
           isfinite(configuration->application.speed_deceleration_rad_s2) &&
           configuration->application.speed_deceleration_rad_s2 > 0.0f &&
           isfinite(configuration->application.speed_kp) &&
           configuration->application.speed_kp >= 0.0f &&
           isfinite(configuration->application.speed_ki) &&
           configuration->application.speed_ki >= 0.0f;
}

/* 返回的描述符属于只读常量表，不含可变控制对象地址。 */
static const ScalarField *find_field(uint8_t id)
{
    for (unsigned index = 0U; index < sizeof(s_fields) / sizeof(s_fields[0]); ++index)
    {
        if (s_fields[index].id == id)
        {
            return &s_fields[index];
        }
    }
    return NULL;
}

/* 仅供本表的正整数缩放使用（1000/10^6/10^9），尾数乘积精确且不溢出 u64。
 * 保留原先 double 缩放后加正负半值、先检查 i32 边界再截断的顺序：
 * 舍入值到达端点时，只有精确半值允许通过；不可改成状态编码的舍入后检查。
 * binary32 非有限值直接拒绝，失败不修改输出；有界移位不依赖硬件或全局状态。 */
static ParameterStatus encode_float(float value, uint32_t scale, uint32_t *raw)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    unsigned exponent = (bits >> 23U) & 255U;
    if (exponent == 255U)
    {
        return PARAMETER_OUT_OF_RANGE;
    }
    uint64_t product = (uint64_t)((bits & 0x7fffffU) | (exponent ? 0x800000U : 0U)) * scale;
    int shift = exponent ? 150 - (int)exponent : 149;
    uint64_t magnitude;
    bool half = false;
    if (shift > 63)
    {
        magnitude = 0U;
    }
    else if (shift > 0)
    {
        uint64_t midpoint = (uint64_t)1U << (shift - 1);
        magnitude = (product >> shift) + ((product & midpoint) != 0U);
        half = (product & ((midpoint << 1) - 1U)) == midpoint;
    }
    else
    {
        if (shift < -31 || product > (UINT32_MAX >> -shift))
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        magnitude = product << -shift;
    }
    bool negative = (bits >> 31U) != 0U;

    uint32_t limit = negative ? 0x80000000U : 0x7fffffffU;
    if (magnitude > limit || (magnitude == limit && !half))
    {
        return PARAMETER_OUT_OF_RANGE;
    }
    *raw = negative ? 0U - (uint32_t)magnitude : (uint32_t)magnitude;
    return PARAMETER_OK;
}

ParameterStatus ParameterValue_Read(const MotorConfiguration *configuration,
                                    uint32_t revision,
                                    uint8_t parameter_id,
                                    ParameterValue *value)
{
    if (value == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    *value = (ParameterValue){0};
    if (configuration == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    ParameterValue candidate = {
        parameter_id, PARAMETER_VALUE_U32, PARAMETER_ACCESS_READ, 0U, revision, 0U};
    const ScalarField *field = find_field(parameter_id);
    if (field != NULL)
    {
        float si_value;
        memcpy(&si_value, (const uint8_t *)configuration + field->offset, sizeof(si_value));
        ParameterStatus status = encode_float(si_value, field->scale, &candidate.raw_value);
        if (status != PARAMETER_OK)
        {
            return status;
        }
        candidate.value_type = PARAMETER_VALUE_I32;
        candidate.access = field->access;
        if (configuration->application.axis_profile.magic == MOTOR_AXIS_PROFILE_NUMERIC_MAGIC &&
            (parameter_id == 21U || parameter_id == 22U || parameter_id == 23U))
        {
            /* 启动 joint profile 会覆盖这些字段，不能承诺单值改写的掉电持久语义。 */
            candidate.access &= (uint8_t)~PARAMETER_ACCESS_WRITE_IDLE;
        }
    }
    else
    {
        switch (parameter_id)
        {
        case 0U:
            candidate.raw_value = revision;
            break;
        case 3U:
            candidate.raw_value = configuration->device.node_id;
            break;
        case 4U:
            candidate.raw_value = 1U;
            break;
        case 5U:
            if (configuration->model.motor_pole_pairs <= 0)
            {
                return PARAMETER_OUT_OF_RANGE;
            }
            candidate.raw_value = (uint32_t)configuration->model.motor_pole_pairs;
            break;
        case 10U:
            candidate.value_type = PARAMETER_VALUE_I32;
            if (encode_float((configuration->model.motor_d_inductance_h +
                              configuration->model.motor_q_inductance_h) *
                                 0.5f,
                             1000000000.0f,
                             &candidate.raw_value) != PARAMETER_OK)
            {
                return PARAMETER_OUT_OF_RANGE;
            }
            break;
        case 0x40U:
            candidate.raw_value = configuration->device.encoder.electrical_zero_q15;
            break;
        case 0x41U:
            candidate.raw_value = configuration->device.encoder.mechanical_zero_q15;
            break;
        case 0x42U:
            candidate.raw_value = configuration->device.encoder.reverse;
            break;
        case 0x43U:
            /* 参数目录定义的是完整有效布尔值，不能把部分标志 0x01 解释为有效。 */
            candidate.raw_value =
                (configuration->device.encoder.calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL ? 1U
                                                                                            : 0U;
            break;
        case 0x4CU:
            candidate.raw_value = CURRENT_SENSE_SHUNT_MILLIOHM;
            break;
        case 0x4DU:
            candidate.raw_value = configuration->configuration_confirmed ? 1U : 0U;
            candidate.access |= PARAMETER_ACCESS_WRITE_IDLE;
            break;
        case 0x4EU:
            candidate.raw_value = (uint32_t)configuration->configuration_source;
            break;
        case 0x68U:
            candidate.raw_value = configuration->application.speed_feedforward.enabled ? 1U : 0U;
            candidate.access |= PARAMETER_ACCESS_WRITE_IDLE;
            break;
        default:
            return PARAMETER_UNSUPPORTED;
        }
        if (parameter_id >= 0x40U && parameter_id <= 0x4BU)
        {
            candidate.access |= PARAMETER_ACCESS_CALIBRATION;
        }
    }
    *value = candidate;
    return PARAMETER_OK;
}

static float speed_ceiling(const MotorConfiguration *configuration)
{
    float limit = configuration->application.speed_ceiling_rad_s;
    if (!isfinite(limit) || limit <= 0.0f)
    {
        return 0.0f;
    }
    if (configuration->application.axis_profile.magic != 0U)
    {
        limit = fminf(limit, configuration->application.axis_profile.maximum_speed_rad_s);
    }
    return limit;
}

ParameterStatus ParameterValue_Apply(MotorConfiguration *configuration,
                                     const ParameterWriteRequest *request)
{
    if (configuration == NULL || request == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    if (request->parameter_id == 0x4DU)
    {
        if (request->value_type != PARAMETER_VALUE_U32)
        {
            return PARAMETER_BAD_TYPE;
        }
        if (request->raw_value > 1U ||
            (request->raw_value == 1U && !ParameterValue_ConfigurationValid(configuration)))
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        configuration->configuration_confirmed = request->raw_value == 1U;
        configuration->configuration_source = MOTOR_CONFIGURATION_ACTIVE;
        return PARAMETER_OK;
    }
    if (request->parameter_id == 0x68U)
    {
        if (request->value_type != PARAMETER_VALUE_U32)
        {
            return PARAMETER_BAD_TYPE;
        }
        if (request->raw_value > 1U ||
            !SpeedFeedforward_IsValid(&configuration->application.speed_feedforward))
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        configuration->application.speed_feedforward.enabled = request->raw_value == 1U;
        return PARAMETER_OK;
    }
    const ScalarField *field = find_field(request->parameter_id);
    if (field == NULL)
    {
        switch (request->parameter_id)
        {
        case 0U:
        case 3U:
        case 4U:
        case 5U:
        case 10U:
        case 0x40U:
        case 0x41U:
        case 0x42U:
        case 0x43U:
        case 0x4BU:
        case 0x4CU:
        case 0x4EU:
        case 0x4FU:
        case 0x50U:
        case 0x51U:
            return PARAMETER_READ_ONLY;
        default:
            return PARAMETER_UNSUPPORTED;
        }
    }
    if ((field->access & PARAMETER_ACCESS_WRITE_IDLE) == 0U)
    {
        return PARAMETER_READ_ONLY;
    }
    if (configuration->application.axis_profile.magic == MOTOR_AXIS_PROFILE_NUMERIC_MAGIC &&
        (request->parameter_id == 21U || request->parameter_id == 22U ||
         request->parameter_id == 23U))
    {
        return PARAMETER_READ_ONLY;
    }
    if (request->value_type != PARAMETER_VALUE_I32)
    {
        return PARAMETER_BAD_TYPE;
    }
    int32_t signed_raw;
    memcpy(&signed_raw, &request->raw_value, sizeof(signed_raw));
    float si_value = (float)signed_raw / field->scale;
    if (request->parameter_id >= 0x60U && request->parameter_id <= 0x66U)
    {
        MotorWindingThermalConfig candidate = configuration->model.thermal;
        /* 只访问已登记字段；候选完整验证后才替换唯一活动配置。 */
        size_t offset = field->offset - offsetof(MotorConfiguration, model.thermal);
        memcpy((uint8_t *)&candidate + offset, &si_value, sizeof(si_value));
        if (!MotorWindingThermal_Valid(&candidate))
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        configuration->model.thermal = candidate;
        return PARAMETER_OK;
    }
    if (request->parameter_id >= 0x69U && request->parameter_id <= 0x6CU)
    {
        SpeedFeedforwardConfig candidate = configuration->application.speed_feedforward;
        size_t offset = field->offset - offsetof(MotorConfiguration, application.speed_feedforward);
        memcpy((uint8_t *)&candidate + offset, &si_value, sizeof(si_value));
        if (!SpeedFeedforward_IsValid(&candidate))
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        configuration->application.speed_feedforward = candidate;
        return PARAMETER_OK;
    }
    switch (request->parameter_id)
    {
    case 1U:
        if (si_value <= 0.0f || si_value > speed_ceiling(configuration))
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        configuration->application.speed_limit_rad_s = si_value;
        configuration->application.position_maximum_speed_rad_s =
            fminf(configuration->application.position_maximum_speed_rad_s, si_value);
        break;
    case 2U:
    {
        float limit = CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A;
        if (configuration->application.axis_profile.magic == MOTOR_AXIS_PROFILE_NUMERIC_MAGIC)
        {
            MotorJointControlConfig config;
            if (!MotorAxisProfile_Resolve(&configuration->application.axis_profile, &config))
            {
                return PARAMETER_OUT_OF_RANGE;
            }
            limit = fminf(limit, config.maximum_current_a);
        }
        if (si_value <= 0.0f || si_value > limit)
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        configuration->application.current_limit_a = si_value;
        break;
    }
    case 21U:
        if (si_value < 0.0f || si_value > CASCADE_POSITION_KP_MAX_PER_S)
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        configuration->application.cascade_position_kp = si_value;
        break;
    case 22U:
        if (si_value < 0.0f || si_value > CASCADE_POSITION_KD_MAX)
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        configuration->application.cascade_position_kd = si_value;
        break;
    case 23U:
        if (si_value <= 0.0f || si_value > configuration->application.speed_limit_rad_s ||
            si_value > speed_ceiling(configuration))
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        configuration->application.position_maximum_speed_rad_s = si_value;
        break;
    default:
        return PARAMETER_READ_ONLY;
    }
    return PARAMETER_OK;
}
