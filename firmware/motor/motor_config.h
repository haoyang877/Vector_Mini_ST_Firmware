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

typedef struct
{
    float align_current_ramp_time_s;
    float align_hold_time_s;
    float align_current_a;
    float startup_iq_initial_a;
    float startup_iq_a;
    float startup_iq_ramp_time_s;
    float startup_id_a;
    float minimum_current_limit_a;
    float minimum_electrical_velocity_rad_s;
    float target_electrical_velocity_rad_s;
    float startup_ramp_time_s;
    float speed_lock_time_s;
    float speed_lock_filter_alpha;
    float observer_lock_ratio;
    float angle_handoff_time_s;
    float lock_timeout_s;
    float id_ramp_down_time_s;
    float observer_loss_time_s;
    float speed_kp, speed_ki; /* 标定专用归一化速度 PI 增益。 */
} SensorlessStartupConfig;

/* 由经过校验的轴身份在启动时绑定；算法只使用只读配置。 */
typedef struct
{
    int32_t pole_pairs;
    float resistance_ohm, inductance_h, flux_wb;
    float id_kp, id_ki, observer_gamma;
    float speed_pll_hz, gear_ratio;
} MotorElectricalPreset;

typedef struct
{
    uint8_t node_id;
    MotorJointType joint;
    const MotorElectricalPreset *electrical;
    const SensorlessStartupConfig *startup;
    float calibration_speed_rad_s, calibration_limit_a, speed_ceiling_rad_s;
    MotorJointControlConfig control;
    /* 装配标称阻尼力矩；用于标识工况，不直接作为电流前馈。 */
    float damper_torque_nm;
    /* 对称机械行程的单侧上限，输出轴 rad；连续旋转轮填零。 */
    float output_travel_limit_rad;
} MotorAxisPreset;

/**
 * @brief 查询该装配是否带有用于输出轴测角的正交双线性霍尔。
 * @param preset 启动时匹配的轴配置，未配置时为 NULL。
 * @return 配置为减速传动的 PITCH 和 YAW 返回 true；直驱返回 false。
 */
static inline bool MotorAxisPreset_HasLinearHall(const MotorAxisPreset *preset)
{
    return preset != NULL && preset->electrical != NULL && preset->electrical->gear_ratio > 1.0f &&
           (preset->joint == MOTOR_JOINT_PITCH || preset->joint == MOTOR_JOINT_YAW);
}

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
    const MotorAxisPreset *preset;
} MotorConfiguration;

/**
 * @brief 将产品轴活动机械速度上限转换为开环电频率上限。
 * @param configuration 当前活动配置，机械速度为电机轴 rad/s。
 * @param hardware_hz 公共电频率硬上限，Hz。
 * @return 受两种约束的电频率上限，旧配置沿用公共上限。
 */
static inline float MotorConfiguration_OpenLoopLimitHz(const MotorConfiguration *configuration,
                                                       float hardware_hz)
{
    return configuration->preset == NULL
               ? hardware_hz
               : fminf(hardware_hz,
                       configuration->application.speed_limit_rad_s *
                           configuration->model.motor_pole_pairs / 6.283185307f);
}

#endif
