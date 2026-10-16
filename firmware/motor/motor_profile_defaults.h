#ifndef MOTOR_PROFILE_DEFAULTS_H
#define MOTOR_PROFILE_DEFAULTS_H

/* 编译期选择有实际数据的电机型号；未知型号拒绝编译。 */
#define FOC_MOTOR_PROFILE_HT8115_4 1U
#ifndef FOC_ACTIVE_MOTOR_PROFILE
#define FOC_ACTIVE_MOTOR_PROFILE FOC_MOTOR_PROFILE_HT8115_4
#endif

/* 电机物理模型与电流环默认值；不随板卡采样尺度增大。 */
#if FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_HT8115_4

#define PARAM_MOTOR_POLE_PAIRS 21
#define PARAM_MOTOR_PHASE_RESISTANCE_OHM (3.81f * 0.5f)
#define PARAM_MOTOR_D_INDUCTANCE_H (3.27e-3f * 0.5f)
#define PARAM_MOTOR_Q_INDUCTANCE_H (3.27e-3f * 0.5f)
/* Kv = 15 rpm/V，磁链公式：60 / (sqrt(3) * 2π * 极对数 * Kv)。 */
#define PARAM_MOTOR_FLUX_WB 0.0175025f

/* 更换更低阻值分流器不能增大电机默认电流；对齐最小值由平台对齐契约定义。 */
#define PARAM_MOTOR_CALIB_CURRENT_A 3.0f
#define PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S (500.0f * 6.283185307f)

#else
#error "Unsupported FOC_ACTIVE_MOTOR_PROFILE"
#endif

#endif
