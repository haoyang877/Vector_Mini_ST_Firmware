#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdbool.h>
#include <stdint.h>

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
    MOTOR_FAULT_WINDING_THERMAL,
} MotorFaultCode;

/** @brief 单轴易失运行状态；不包含配置或配置指针，目标、故障和测量永不持久化。 */
typedef struct
{
    MotorControlMode control_mode;
    /* 原子故障锁存字；保存MotorFaultCode值，不属于线上或持久化结构。 */
    volatile uint32_t fault_code;
    bool uses_sensorless_feedback;
    /* 快环发布的动态热边界；配置值不被降额覆盖，保护层只读恢复事实。 */
    float thermal_current_limit_a;
    bool thermal_limit_active;
    bool thermal_clear_ready;

    float id_target_a, iq_target_a, vq_target_v;
    bool uses_speed_ramp;
    float speed_target_rad_s, speed_ramped_rad_s;
    bool position_target_reached;
    float position_target_rad, position_reference_rad;
    float position_velocity_filtered_rad_s;
    /* 模式 3 规划轨迹速度；遥测无效窗口为 NaN。 */
    float position_trajectory_speed_rad_s;

    /* 正式开环模式的易失参考；停止/启动清零，频率为电角Hz，相位为下一拍rad。 */
    float vd_target_v, open_loop_frequency_hz, open_loop_angle_rad;

} MotorControlState;

#endif
