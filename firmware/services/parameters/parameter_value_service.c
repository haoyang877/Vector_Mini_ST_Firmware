#include "parameter_value_service.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "foc_param_profile.h"
#include "position_cascade_config.h"

/* 只登记有真实字段和已冻结缩放的项；单位缩放不等于允许的产品范围。 */
typedef struct
{
    uint8_t id, access;
    uint16_t offset;
    float scale;
} ScalarField;

#define READ_FIELD(id, field, scale)                                                               \
    {id, PARAMETER_ACCESS_READ, offsetof(MotorControl_TypeDef, field), scale}
#define WRITE_FIELD(id, field, scale)                                                              \
    {id,                                                                                           \
     PARAMETER_ACCESS_READ | PARAMETER_ACCESS_WRITE_IDLE,                                          \
     offsetof(MotorControl_TypeDef, field),                                                        \
     scale}
#define CAL_FIELD(id, field, scale)                                                                \
    {id,                                                                                           \
     PARAMETER_ACCESS_READ | PARAMETER_ACCESS_CALIBRATION,                                         \
     offsetof(MotorControl_TypeDef, field),                                                        \
     scale}
static const ScalarField fields[] = {
    WRITE_FIELD(1U, speed_limit, 1000.0f),
    WRITE_FIELD(2U, current_limit, 1000.0f),
    READ_FIELD(6U, motor_flux, 1000000000.0f),
    READ_FIELD(7U, motor_phase_resistance, 1000000.0f),
    READ_FIELD(8U, motor_d_inductance, 1000000000.0f),
    READ_FIELD(9U, motor_q_inductance, 1000000000.0f),
    READ_FIELD(11U, id_Kp, 1000.0f),
    READ_FIELD(12U, id_Ki, 1000.0f),
    READ_FIELD(13U, iq_Kp, 1000.0f),
    READ_FIELD(14U, iq_Ki, 1000.0f),
    READ_FIELD(16U, speed_Kp, 1000000.0f),
    READ_FIELD(17U, speed_Ki, 1000000.0f),
    READ_FIELD(19U, speedAcc, 1000.0f),
    READ_FIELD(20U, speedDec, 1000.0f),
    WRITE_FIELD(21U, cascade_pos_Kp, 1000000.0f),
    WRITE_FIELD(22U, cascade_pos_Kd, 1000000.0f),
    WRITE_FIELD(23U, pos_maxspeed, 1000.0f),
    READ_FIELD(24U, posAcc, 1000.0f),
    READ_FIELD(25U, posDec, 1000.0f),
    READ_FIELD(26U, pos_error_window, 1000.0f),
    CAL_FIELD(0x44U, A_Offset, 1000.0f),
    CAL_FIELD(0x45U, B_Offset, 1000.0f),
    CAL_FIELD(0x46U, C_Offset, 1000.0f),
    CAL_FIELD(0x47U, friction_coulomb_pos_a, 1000.0f),
    CAL_FIELD(0x48U, friction_coulomb_neg_a, 1000.0f),
    CAL_FIELD(0x49U, friction_viscous_pos_a_per_rad_s, 1000000.0f),
    CAL_FIELD(0x4AU, friction_viscous_neg_a_per_rad_s, 1000000.0f),
};

/* 返回的描述符属于只读常量表，不含可变控制对象地址。 */
static const ScalarField *find_field(uint8_t id)
{
    for (unsigned index = 0U; index < sizeof(fields) / sizeof(fields[0]); ++index)
    {
        if (fields[index].id == id)
        {
            return &fields[index];
        }
    }
    return NULL;
}

static ParameterStatus encode_float(float si_value, float scale, uint32_t *raw)
{
    if (!isfinite(si_value))
    {
        return PARAMETER_OUT_OF_RANGE;
    }
    double rounded = (double)si_value * scale;
    rounded += rounded >= 0.0 ? 0.5 : -0.5;
    /* 此界限只防整数编码溢出；写入另行校验真实产品范围。 */
    if (rounded > INT32_MAX || rounded < INT32_MIN)
    {
        return PARAMETER_OUT_OF_RANGE;
    }
    *raw = (uint32_t)(int32_t)rounded;
    return PARAMETER_OK;
}

ParameterStatus ParameterValue_Read(const MotorControl_TypeDef *motor,
                                    const Encoder_TypeDef *encoder,
                                    uint32_t revision,
                                    uint8_t node_id,
                                    uint8_t parameter_id,
                                    ParameterValue *value)
{
    if (value == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    *value = (ParameterValue){0};
    if (motor == NULL || encoder == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    ParameterValue candidate = {
        parameter_id, PARAMETER_VALUE_U32, PARAMETER_ACCESS_READ, 0U, revision, 0U};
    const ScalarField *field = find_field(parameter_id);
    if (field != NULL)
    {
        float si_value;
        memcpy(&si_value, (const uint8_t *)motor + field->offset, sizeof(si_value));
        ParameterStatus status = encode_float(si_value, field->scale, &candidate.raw_value);
        if (status != PARAMETER_OK)
        {
            return status;
        }
        candidate.value_type = PARAMETER_VALUE_I32;
        candidate.access = field->access;
        if (motor->axis_profile.magic == MOTOR_AXIS_PROFILE_NUMERIC_MAGIC &&
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
            candidate.raw_value = node_id;
            break;
        case 4U:
            candidate.raw_value = 1U;
            break;
        case 5U:
            if (motor->motor_pole_pairs <= 0)
            {
                return PARAMETER_OUT_OF_RANGE;
            }
            candidate.raw_value = (uint32_t)motor->motor_pole_pairs;
            break;
        case 10U:
            candidate.value_type = PARAMETER_VALUE_I32;
            if (encode_float((motor->motor_d_inductance + motor->motor_q_inductance) * 0.5f,
                             1000000000.0f,
                             &candidate.raw_value) != PARAMETER_OK)
            {
                return PARAMETER_OUT_OF_RANGE;
            }
            break;
        case 0x40U:
            candidate.raw_value = encoder->electrical_zero_q15;
            break;
        case 0x41U:
            candidate.raw_value = encoder->mechanical_zero_q15;
            break;
        case 0x42U:
            candidate.raw_value = encoder->reverse;
            break;
        case 0x43U:
            /* 参数目录定义的是完整有效布尔值，不能把部分标志 0x01 解释为有效。 */
            candidate.raw_value = (encoder->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL ? 1U : 0U;
            break;
        case 0x4BU:
            candidate.raw_value = motor->friction_model_valid ? 1U : 0U;
            break;
        case 0x4CU:
            candidate.raw_value = CURRENT_SENSE_SHUNT_MILLIOHM;
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

static float speed_ceiling(const MotorControl_TypeDef *motor, uint8_t node_id)
{
    float limit = Param_SpeedLimitRadS(node_id);
    if (motor->axis_profile.magic != 0U)
    {
        limit = fminf(limit, motor->axis_profile.maximum_speed_rad_s);
    }
    return limit;
}

ParameterStatus ParameterValue_Apply(MotorControl_TypeDef *motor,
                                     uint8_t node_id,
                                     const ParameterWriteRequest *request)
{
    if (motor == NULL || request == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
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
            return PARAMETER_READ_ONLY;
        default:
            return PARAMETER_UNSUPPORTED;
        }
    }
    if ((field->access & PARAMETER_ACCESS_WRITE_IDLE) == 0U)
    {
        return PARAMETER_READ_ONLY;
    }
    if (motor->axis_profile.magic == MOTOR_AXIS_PROFILE_NUMERIC_MAGIC &&
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
    switch (request->parameter_id)
    {
    case 1U:
        if (si_value <= 0.0f || si_value > speed_ceiling(motor, node_id))
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        motor->speed_limit = si_value;
        motor->pos_maxspeed = fminf(motor->pos_maxspeed, si_value);
        break;
    case 2U:
    {
        float limit = CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A;
        if (motor->axis_profile.magic == MOTOR_AXIS_PROFILE_NUMERIC_MAGIC)
        {
            MotorJointControlConfig config;
            if (!MotorAxisProfile_Resolve(&motor->axis_profile, &config))
            {
                return PARAMETER_OUT_OF_RANGE;
            }
            limit = fminf(limit, config.maximum_current_a);
        }
        if (si_value <= 0.0f || si_value > limit)
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        motor->current_limit = si_value;
        break;
    }
    case 21U:
        if (si_value < 0.0f || si_value > CASCADE_POSITION_KP_MAX_PER_S)
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        motor->cascade_pos_Kp = si_value;
        break;
    case 22U:
        if (si_value < 0.0f || si_value > CASCADE_POSITION_KD_MAX)
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        motor->cascade_pos_Kd = si_value;
        break;
    case 23U:
        if (si_value <= 0.0f || si_value > motor->speed_limit ||
            si_value > speed_ceiling(motor, node_id))
        {
            return PARAMETER_OUT_OF_RANGE;
        }
        motor->pos_maxspeed = si_value;
        break;
    default:
        return PARAMETER_READ_ONLY;
    }
    return PARAMETER_OK;
}
