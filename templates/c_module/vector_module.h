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

/**
 * @brief 校验配置并初始化调用方持有的状态；任何失败都保持模块未使能。
 * @param module 待初始化状态；成功后由调用方继续持有，不允许传入 NULL。
 * @param config 只读 SI 单位配置；函数返回后不保存该指针。
 * @return 成功返回 OK；空指针、非有限值或范围非法返回 INVALID_ARGUMENT。
 * @note 不分配内存、不访问硬件，也不会使能功率输出。
 */
VectorModuleStatus VectorModule_Init(VectorModule *module, const VectorModuleConfig *config);

/**
 * @brief 更新经过限位检查的位置目标，但不直接发出硬件命令。
 * @param module 已成功初始化的调用方状态，不允许传入 NULL。
 * @param target_position_rad 目标机械位置，单位 rad，必须位于配置闭区间内。
 * @return 接受目标返回 OK；未初始化、参数非法或越界时返回对应错误。
 * @note 设计用于前台或慢速控制层，不得在硬实时 ISR 中调用。
 */
VectorModuleStatus VectorModule_SetTarget(VectorModule *module, float target_position_rad);

#endif
