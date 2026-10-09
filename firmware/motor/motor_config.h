/* 活动参数的值对象；MotorAxis 独占实例，算法只借用，不拥有存储或通信地址。 */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include "axis_config.h"

/** @brief 电机物理模型与电流环整定；识别任务成功后由所有者提交。 */
typedef struct
{
    int32_t motor_pole_pairs;
    float motor_phase_resistance_ohm;
    float motor_d_inductance_h;
    float motor_q_inductance_h;
    float motor_flux_wb;
    float calibration_current_a;
    float id_kp, id_ki, iq_kp, iq_ki;
} MotorModelConfig;

/** @brief 应用运行包络、轨迹和外环整定；不含命令目标或 PI 积分状态。 */
typedef struct
{
    float current_limit_a;
    float speed_limit_rad_s;
    /* 装载边界解析的产品上限；不持久化、不随通信端点地址变化。 */
    float speed_ceiling_rad_s;
    float speed_acceleration_rad_s2, speed_deceleration_rad_s2;
    float speed_kp, speed_ki;
    float position_error_window_rad;
    float position_acceleration_rad_s2, position_deceleration_rad_s2;
    float position_maximum_speed_rad_s;
    float cascade_position_kp, cascade_position_kd;
    MotorAxisProfile axis_profile;
    bool axis_profile_valid;
} MotorApplicationConfig;

/** @brief 设备个体地址与电流域标定；编码器/LUT 和齿槽表保留各自单份所有权。 */
typedef struct
{
    float phase_a_offset_counts, phase_b_offset_counts, phase_c_offset_counts;
    uint8_t node_id;
} MotorDeviceConfig;

#endif
