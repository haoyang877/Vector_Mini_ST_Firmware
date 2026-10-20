/* 活动参数的值对象；MotorAxis 独占实例，算法只借用，不拥有存储或通信地址。 */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include "axis_config.h"
#include "motor_thermal_config.h"
#include "speed_feedforward.h"
#include "encoder_config.h"

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
    MotorWindingThermalConfig thermal;
} MotorModelConfig;

/** @brief 应用运行包络、轨迹和外环整定；不含命令目标或 PI 积分状态。 */
typedef struct
{
    float current_limit_a;
    float speed_limit_rad_s;
    /* 编译期应用包络经有效AXS收窄；不持久化、不按设备地址选择。 */
    float speed_ceiling_rad_s;
    float speed_acceleration_rad_s2, speed_deceleration_rad_s2;
    float speed_kp, speed_ki;
    SpeedFeedforwardConfig speed_feedforward;
    float position_error_window_rad;
    float position_acceleration_rad_s2, position_deceleration_rad_s2;
    float position_maximum_speed_rad_s;
    float cascade_position_kp, cascade_position_kd;
    MotorAxisProfile axis_profile;
    bool axis_profile_valid;
} MotorApplicationConfig;

/** @brief 设备个体地址与装配标定；电流偏置每次启动由关相采样重新取得。 */
typedef struct
{
    float phase_a_offset_counts, phase_b_offset_counts, phase_c_offset_counts;
    uint8_t node_id;
    EncoderCalibration encoder;
} MotorDeviceConfig;

/** @brief 活动配置来源；回退默认值不等于用户已确认电机配置。 */
typedef enum
{
    MOTOR_CONFIGURATION_DEFAULTS = 0,
    MOTOR_CONFIGURATION_STORED,
    MOTOR_CONFIGURATION_UNSUPPORTED,
    MOTOR_CONFIGURATION_CORRUPT,
    MOTOR_CONFIGURATION_ACTIVE
} MotorConfigurationSource;

/** @brief 单轴活动配置；Axis单份持有，算法仅借用const视图，禁止存放运行目标或积分。 */
typedef struct
{
    MotorModelConfig model;
    MotorApplicationConfig application;
    MotorDeviceConfig device;
    /* 参数所有者元数据，不写入运行状态；存储层显式编码确认位。 */
    bool configuration_confirmed;
    MotorConfigurationSource configuration_source;
} MotorConfiguration;

#endif
