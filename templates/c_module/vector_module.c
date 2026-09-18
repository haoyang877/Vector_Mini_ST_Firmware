#include "vector_module.h"

#include <math.h>
#include <stddef.h>

/* 初始化之前集中验证配置，避免部分写入后留下可被误用的状态。 */
static bool config_is_valid(const VectorModuleConfig *config)
{
    if (config == NULL)
    {
        return false;
    }
    return isfinite(config->minimum_position_rad) && isfinite(config->maximum_position_rad) &&
           isfinite(config->maximum_speed_rad_s) &&
           config->minimum_position_rad < config->maximum_position_rad &&
           config->maximum_speed_rad_s > 0.0f;
}

VectorModuleStatus VectorModule_Init(VectorModule *module, const VectorModuleConfig *config)
{
    if (module == NULL || !config_is_valid(config))
    {
        return VECTOR_MODULE_STATUS_INVALID_ARGUMENT;
    }

    module->initialized = false;
    module->config = *config;
    module->target_position_rad = 0.0f;
    module->initialized = true;
    return VECTOR_MODULE_STATUS_OK;
}

VectorModuleStatus VectorModule_SetTarget(VectorModule *module, float target_position_rad)
{
    if (module == NULL || !isfinite(target_position_rad))
    {
        return VECTOR_MODULE_STATUS_INVALID_ARGUMENT;
    }
    if (!module->initialized)
    {
        return VECTOR_MODULE_STATUS_NOT_INITIALIZED;
    }
    if (target_position_rad < module->config.minimum_position_rad ||
        target_position_rad > module->config.maximum_position_rad)
    {
        return VECTOR_MODULE_STATUS_OUT_OF_RANGE;
    }

    module->target_position_rad = target_position_rad;
    return VECTOR_MODULE_STATUS_OK;
}
