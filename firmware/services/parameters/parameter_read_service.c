#include "parameter_read_service.h"

#include <math.h>

/* 缩放上限留出浮点余量，保证 (int32_t) 转换不会越过 INT32_MAX 而变成未定义行为。 */
#define PARAMETER_READ_MAX_SCALED_MILLI 2000000000.0f
#define PARAMETER_READ_MILLI_PER_UNIT 1000.0f

/* 把正 SI 值转换为四舍五入后的毫单位整数；非有限、非正或越界一律拒绝。 */
static bool scale_positive_milli(float value_si, int32_t *scaled)
{
    float milli;

    if (!isfinite(value_si) || value_si <= 0.0f)
    {
        return false;
    }
    milli = value_si * PARAMETER_READ_MILLI_PER_UNIT;
    if (!isfinite(milli) || milli > PARAMETER_READ_MAX_SCALED_MILLI)
    {
        return false;
    }
    *scaled = (int32_t)(milli + 0.5f);
    return true;
}

/* 先判定 ID 是否登记，再检查快照；未知 ID 与未就绪必须可区分。 */
static bool parameter_id_is_registered(uint16_t parameter_id)
{
    switch (parameter_id)
    {
    case PARAMETER_READ_ID_ACTIVE_REVISION:
    case PARAMETER_READ_ID_MAX_SPEED_RAD_S:
    case PARAMETER_READ_ID_MAX_CURRENT_A:
        return true;
    default:
        return false;
    }
}

ParameterReadStatus ParameterRead_Get(const ParameterReadSource *source,
                                      uint16_t parameter_id,
                                      ParameterReadResult *result)
{
    ParameterReadResult value = {0};

    if (result == NULL)
    {
        return PARAMETER_READ_INVALID_ARGUMENT;
    }
    *result = value;
    if (source == NULL)
    {
        return PARAMETER_READ_INVALID_ARGUMENT;
    }
    if (!parameter_id_is_registered(parameter_id))
    {
        return PARAMETER_READ_UNSUPPORTED;
    }
    if (!source->snapshot_ready)
    {
        return PARAMETER_READ_NOT_READY;
    }
    switch (parameter_id)
    {
    case PARAMETER_READ_ID_ACTIVE_REVISION:
        value.value_type = PARAMETER_READ_VALUE_TYPE_U32;
        value.value = (int32_t)source->config_revision;
        break;
    case PARAMETER_READ_ID_MAX_SPEED_RAD_S:
        if (!scale_positive_milli(source->values.speed_limit, &value.value))
        {
            return PARAMETER_READ_INVALID_VALUE;
        }
        value.value_type = PARAMETER_READ_VALUE_TYPE_I32;
        break;
    case PARAMETER_READ_ID_MAX_CURRENT_A:
        if (!scale_positive_milli(source->values.current_limit, &value.value))
        {
            return PARAMETER_READ_INVALID_VALUE;
        }
        value.value_type = PARAMETER_READ_VALUE_TYPE_I32;
        break;
    default:
        return PARAMETER_READ_UNSUPPORTED;
    }
    *result = value;
    return PARAMETER_READ_OK;
}
