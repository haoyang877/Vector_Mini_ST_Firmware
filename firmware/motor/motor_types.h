#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include "axis_config.h"

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

typedef struct
{
    MotorControlMode control_mode;
    MotorFaultCode fault_code;

    float phase_a_offset_counts; /* fractional 12-bit ADC counts, same units as stored parameters */
    float phase_b_offset_counts;
    float phase_c_offset_counts;

    int32_t motor_pole_pairs;
    float motor_phase_resistance_ohm;
    /* Raw resistance from the 0/120/240 degree test vectors. */
    float phase_resistance_vector_ohm[3];
    float phase_resistance_a_ohm;
    float phase_resistance_b_ohm;
    float phase_resistance_c_ohm;
    float phase_resistance_spread_pct;
    float motor_d_inductance_h;
    float motor_q_inductance_h;
    float motor_flux_wb;
    bool phase_resistance_valid;
    bool phase_resistance_warning;
    bool phase_resistance_balanced;

    bool uses_sensorless_feedback;

    float calibration_current_a;
    float current_limit_a;
    float speed_limit_rad_s;

    float id_target_a;
    float id_kp;
    float id_ki;
    float iq_target_a;
    float iq_kp;
    float iq_ki;
    /*q-axis voltage command (V), used by MOTOR_CONTROL_VQ*/
    float vq_target_v;

    bool uses_speed_ramp;
    float speed_acceleration_rad_s2;
    float speed_deceleration_rad_s2;
    float speed_target_rad_s;
    float speed_ramped_rad_s;
    float speed_kp;
    float speed_ki;

    bool position_target_reached;
    float position_error_window_rad;
    float position_acceleration_rad_s2;
    float position_deceleration_rad_s2;
    float position_maximum_speed_rad_s;
    float position_target_rad;
    float position_reference_rad;
    float cascade_position_kp;
    float cascade_position_kd;

    /* Applied friction model in current domain; identification does not enable compensation. */
    float friction_coulomb_pos_a;
    float friction_coulomb_neg_a;
    float friction_viscous_pos_a_per_rad_s;
    float friction_viscous_neg_a_per_rad_s;
    bool friction_model_valid;

    /* 位置级联遥测；控制器状态由所属模块私有持有。 */
    float position_velocity_filtered_rad_s;

    /* 私有运行配置；诊断成员偏移由当前 ELF 描述。 */
    MotorAxisProfile axis_profile;
    bool axis_profile_valid;
    bool configuration_confirmed;
    MotorConfigurationSource configuration_source;
    /* 模式 3 规划轨迹速度参考（rad/s）；由外环发布，遥测无效窗口为 NaN。 */
    float position_trajectory_speed_rad_s;
} MotorControlState;

#endif
