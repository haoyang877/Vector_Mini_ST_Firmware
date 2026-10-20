#ifndef MOTOR_PROFILE_DEFAULTS_H
#define MOTOR_PROFILE_DEFAULTS_H

/* 编译期选择有实际数据的电机型号；未知型号拒绝编译。 */
#define FOC_MOTOR_PROFILE_HT8115_4 1U
#ifndef FOC_ACTIVE_MOTOR_PROFILE
#define FOC_ACTIVE_MOTOR_PROFILE FOC_MOTOR_PROFILE_HT8115_4
#endif

/* 型号输入：内部统一为相模型的 SI 值，不随板卡采样尺度增大。 */
#if FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_HT8115_4

#define PARAM_MOTOR_POLE_PAIRS 21
/* 保留该型号既有折算；更换电机须确认绕组接法及线/相测量口径。 */
#define PARAM_MOTOR_PHASE_RESISTANCE_OHM (3.81f * 0.5f)
#define PARAM_MOTOR_D_INDUCTANCE_H (3.27e-3f * 0.5f)
#define PARAM_MOTOR_Q_INDUCTANCE_H (3.27e-3f * 0.5f)
/* 该型号既有磁链输入；固件不从未约定电压口径的 Kv 在线推导。 */
#define PARAM_MOTOR_FLUX_WB 0.0175025f

/* 设计输入：校准电流与电流环带宽；不是运行限流或实测标定结果。 */
#define PARAM_MOTOR_CALIB_CURRENT_A 3.0f
#define PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S (500.0f * 6.283185307f)

#else
#error "Unsupported FOC_ACTIVE_MOTOR_PROFILE"
#endif

/* 编译期派生默认 PI；float 转换保留原先相模型赋值的中间舍入。
 * Kp=L·带宽，单位 V/A；Ki=R·带宽，单位 V/(A·s)，执行器每拍另乘 Ts。
 * 有效 Flash 中的人工整定直接装载，不能用这些默认值重新覆盖。 */
#define PARAM_MOTOR_DEFAULT_ID_KP                                                                  \
    ((float)PARAM_MOTOR_D_INDUCTANCE_H * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S)
#define PARAM_MOTOR_DEFAULT_IQ_KP                                                                  \
    ((float)PARAM_MOTOR_Q_INDUCTANCE_H * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S)
#define PARAM_MOTOR_DEFAULT_ID_KI                                                                  \
    ((float)PARAM_MOTOR_PHASE_RESISTANCE_OHM * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S)
#define PARAM_MOTOR_DEFAULT_IQ_KI PARAM_MOTOR_DEFAULT_ID_KI

#endif
