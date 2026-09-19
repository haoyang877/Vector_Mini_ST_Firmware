#include "can_parameter_wire.h"

#include <limits.h>
#include <math.h>

/* 线路约定层实现：编码等级映射、定点转换与 ID/长度规则。
 * 全部为纯函数，不读时钟、不访问硬件、不持有状态。 */

CanValueEncoding CanParamWire_CommandEncoding(CAN_PARAM_ID param_id)
{
    switch (param_id)
    {
    case CAN_SET_CURRENT:
    case CAN_SET_CURRENT_CAL:
    case CAN_SET_CURRENT_LIMIT:
        return CAN_VALUE_MILLI_I16;
    case CAN_SET_POS:
        return CAN_VALUE_MILLI_I32;
    case CAN_SET_SPEED:
    case CAN_SET_SPEED_LIMIT:
    case CAN_SET_SPEED_ACC:
    case CAN_SET_SPEED_DEC:
    case CAN_SET_POS_ACC:
    case CAN_SET_POS_DEC:
    case CAN_SET_POS_MAXSPEED:
        return CAN_VALUE_CENTI_I32;
    default:
        return CAN_VALUE_FLOAT32;
    }
}

CanValueEncoding CanParamWire_ReplyEncoding(CAN_PARAM_ID param_id)
{
    switch (param_id)
    {
    case CAN_GET_CURRENT_SET:
    case CAN_GET_CURRENT_CAL:
    case CAN_GET_CURRENT_LIMIT:
    case CAN_GET_IBUS:
    case CAN_GET_IA:
    case CAN_GET_IB:
    case CAN_GET_IC:
    case CAN_GET_ID:
    case CAN_GET_IQ:
    case CAN_GET_FRICTION_COULOMB_POS:
    case CAN_GET_FRICTION_COULOMB_NEG:
    case CAN_GET_FRICTION_RMSE_POS:
    case CAN_GET_FRICTION_RMSE_NEG:
        return CAN_VALUE_MILLI_I16;
    case CAN_GET_POS_SET:
    case CAN_GET_POS2_FILT:
        return CAN_VALUE_MILLI_I32;
    case CAN_GET_SPEED_SET:
    case CAN_GET_SPEED_LIMIT:
    case CAN_GET_SPEED_ACC:
    case CAN_GET_SPEED_DEC:
    case CAN_GET_POS_ACC:
    case CAN_GET_POS_DEC:
    case CAN_GET_POS_MAXSPEED:
    case CAN_GET_SPEED2_FILT:
        return CAN_VALUE_CENTI_I32;
    default:
        return CAN_VALUE_FLOAT32;
    }
}

int32_t CanParamWire_Milli32(float value)
{
    float scaled;
    if (!isfinite(value))
        return INT32_MIN;
    scaled = value * 1000.0f;
    /* 2^31 可被 float 精确表示，而 INT32_MAX 不行，因此饱和边界写作 2^31。 */
    if (scaled >= 2147483648.0f)
        return INT32_MAX;
    if (scaled <= -2147483648.0f)
        return -INT32_MAX;
    return (int32_t)scaled;
}

int32_t CanParamWire_Centi32(float value)
{
    float scaled;
    if (!isfinite(value))
        return INT32_MIN;
    scaled = value * 100.0f;
    if (scaled >= 2147483648.0f)
        return INT32_MAX;
    if (scaled <= -2147483648.0f)
        return -INT32_MAX;
    return (int32_t)scaled;
}

int16_t CanParamWire_Milli16(float value)
{
    float scaled;
    if (!isfinite(value))
        return INT16_MIN;
    scaled = value * 1000.0f;
    if (scaled >= 32767.0f)
        return INT16_MAX;
    if (scaled <= -32767.0f)
        return -INT16_MAX;
    return (int16_t)scaled;
}

int16_t CanParamWire_Centi16(float value)
{
    float scaled;
    if (!isfinite(value))
        return INT16_MIN;
    scaled = value * 100.0f;
    if (scaled >= 32767.0f)
        return INT16_MAX;
    if (scaled <= -32767.0f)
        return -INT16_MAX;
    return (int16_t)scaled;
}

uint16_t CanParamWire_Identifier(uint8_t node, CAN_PARAM_ID param_id)
{
    return (uint16_t)(((uint16_t)node << 8) | (uint16_t)param_id);
}

uint8_t CanParamWire_Length(CanValueEncoding encoding)
{
    return encoding == CAN_VALUE_MILLI_I16 ? 2U : 4U;
}
