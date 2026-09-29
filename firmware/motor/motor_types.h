#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include "motor_config.h"

#include "motor_control_mode.h"

typedef enum
{
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_CURRENT_OFFSET,
    MOTOR_FAULT_ENCODER,
    MOTOR_FAULT_POLE_PAIRS,
    MOTOR_FAULT_CAN_DISCONNECTED,
    MOTOR_FAULT_PHASE_RESISTANCE,
    MOTOR_FAULT_PHASE_INDUCTANCE,
    MOTOR_FAULT_OVERCURRENT,
    MOTOR_FAULT_OVERVOLTAGE,
    MOTOR_FAULT_UNDERVOLTAGE,
    MOTOR_FAULT_OVERTEMPERATURE,
    MOTOR_FAULT_ENCODER_NOT_CALIBRATED,
    MOTOR_FAULT_INVALID_PARAMETERS,
    MOTOR_FAULT_SENSORLESS,
    MOTOR_FAULT_FRICTION_IDENTIFICATION,
    MOTOR_FAULT_CONTROL_OVERRUN,
    MOTOR_FAULT_COGGING_CALIBRATION,
    MOTOR_FAULT_TEMPERATURE_SENSOR,
    MOTOR_FAULT_SELF_TEST_TIMEOUT,
} MotorFaultCode;

/** @brief 活动配置来源；回退默认值不等于用户已确认电机配置。 */
typedef enum
{
    MOTOR_CONFIGURATION_DEFAULTS = 0,
    MOTOR_CONFIGURATION_STORED,
    MOTOR_CONFIGURATION_UNSUPPORTED,
    MOTOR_CONFIGURATION_CORRUPT,
    MOTOR_CONFIGURATION_ACTIVE
} MotorConfigurationSource;

/** @brief 运行对象聚合；配置分组单份持有，目标、故障和测量永不持久化。 */
typedef struct
{
    MotorModelConfig model;
    MotorApplicationConfig application;
    MotorDeviceConfig device;

    MotorControlMode control_mode;
    MotorFaultCode fault_code;
    /* 识别过程测量，不属于可保存的模型参数。 */
    float phase_resistance_vector_ohm[3];
    float phase_resistance_a_ohm, phase_resistance_b_ohm, phase_resistance_c_ohm;
    float phase_resistance_spread_pct;
    bool phase_resistance_valid, phase_resistance_warning, phase_resistance_balanced;
    bool uses_sensorless_feedback;

    float id_target_a, iq_target_a, vq_target_v;
    bool uses_speed_ramp;
    float speed_target_rad_s, speed_ramped_rad_s;
    bool position_target_reached;
    float position_target_rad, position_reference_rad;
    float position_velocity_filtered_rad_s;
    /* 模式 3 规划轨迹速度；遥测无效窗口为 NaN。 */
    float position_trajectory_speed_rad_s;

    /* 所有者的活动配置元数据；不是存储 schema 或控制目标。 */
    bool configuration_confirmed;
    MotorConfigurationSource configuration_source;
} MotorControlState;

#endif
