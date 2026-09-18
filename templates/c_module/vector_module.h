#ifndef VECTOR_MODULE_H
#define VECTOR_MODULE_H

#include <stdbool.h>
#include <stdint.h>

/** 明确区分失败原因，调用方不得根据布尔值猜测恢复方式。 */
typedef enum
{
    VECTOR_MODULE_STATUS_OK = 0,
    VECTOR_MODULE_STATUS_INVALID_ARGUMENT = 1,
    VECTOR_MODULE_STATUS_NOT_INITIALIZED = 2,
    VECTOR_MODULE_STATUS_OUT_OF_RANGE = 3,
} VectorModuleStatus;

/** 配置只保存 SI 单位限值；产品默认值必须由组合层提供。 */
typedef struct
{
    float minimum_position_rad;
    float maximum_position_rad;
    float maximum_speed_rad_s;
} VectorModuleConfig;

/** 状态由调用方持有；本模块不分配内存、不阻塞，也不直接访问硬件。 */
typedef struct
{
    VectorModuleConfig config;
    float target_position_rad;
    bool initialized;
} VectorModule;

/** 校验配置并初始化状态；任何失败都保持模块未使能。 */
VectorModuleStatus VectorModule_Init(VectorModule *module, const VectorModuleConfig *config);

/** 更新经过限位检查的目标；不得在 ISR 中调用，也不会直接发出硬件命令。 */
VectorModuleStatus VectorModule_SetTarget(VectorModule *module, float target_position_rad);

#endif
