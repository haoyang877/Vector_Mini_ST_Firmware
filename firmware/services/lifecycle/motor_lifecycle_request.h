#ifndef MOTOR_LIFECYCLE_REQUEST_H
#define MOTOR_LIFECYCLE_REQUEST_H

#include <stdint.h>
#include "app_lifecycle.h"

/** 明确业务任务，独立于 CAN 和控制模式编号。 */
typedef enum
{
    MOTOR_MAINTENANCE_ENCODER_LUT = 0,
    MOTOR_MAINTENANCE_ELECTRICAL_ZERO = 1,
    MOTOR_MAINTENANCE_SAVE = 2,
    MOTOR_MAINTENANCE_COGGING = 3,
    MOTOR_MAINTENANCE_PHASE_RESISTANCE = 4,
    MOTOR_MAINTENANCE_FRICTION = 5,
    MOTOR_MAINTENANCE_MECHANICAL_ZERO = 6,
    MOTOR_MAINTENANCE_DEFAULTS = 7,
    MOTOR_MAINTENANCE_NONE = 8
} MotorMaintenanceKind;

/** 原始来源与序号仅用于重试关联，任务枚举不是线上编号。 */
typedef struct
{
    MotorMaintenanceKind kind;
    uint16_t sequence;
    uint8_t source_node;
    uint8_t reserved;
} MotorMaintenanceRequest;

/** 原始作业身份及结果值；不包含工作区或控制对象地址。 */
typedef struct
{
    uint32_t ticket;
    MotorMaintenanceKind kind;
    AppOperationResult result;
    AppOperationEffect effects;
} MotorMaintenanceProgress;

#endif
