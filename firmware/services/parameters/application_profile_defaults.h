#ifndef APPLICATION_PROFILE_DEFAULTS_H
#define APPLICATION_PROFILE_DEFAULTS_H

/* 几何与需求输入：独立于通信节点、板卡和电机物理模型。 */
#define PARAM_WHEEL_DIAMETER_M 0.10f
#define PARAM_WHEEL_MAX_LINEAR_SPEED_M_S 1.0f

/* 派生应用包络：角速度=线速度/半径，单位 rad/s。 */
#define PARAM_WHEEL_SPEED_LIMIT_RAD_S                                                              \
    (2.0f * PARAM_WHEEL_MAX_LINEAR_SPEED_M_S / PARAM_WHEEL_DIAMETER_M)
#define PARAM_APP_SPEED_CEILING_RAD_S PARAM_WHEEL_SPEED_LIMIT_RAD_S

/* 首次配置输入：低速默认值不等于应用包络，不按节点选取。 */
#define PARAM_APP_DEFAULT_SPEED_LIMIT_RPS 0.50f
#define PARAM_APP_DEFAULT_CURRENT_LIMIT_A 6.0f

/* 人工整定输入：rev/s² 在装配时乘 2π；速度 PI 为归一化电流输出口径。 */
#define PARAM_APP_SPEED_ACCEL_RPS2 50.0f
#define PARAM_APP_SPEED_DECEL_RPS2 50.0f
#define PARAM_APP_SPEED_KP 0.05f
#define PARAM_APP_SPEED_KI 0.5f

/* 人工整定输入：低速位置轨迹采用 rev/s²、rev/s，装配时转换为 SI。 */
#define PARAM_APP_POSITION_ACCEL_RPS2 0.125f
#define PARAM_APP_POSITION_DECEL_RPS2 0.125f
#define PARAM_APP_POSITION_MAX_SPEED_RPS 0.125f
/* 人工整定输入：位置 Kp 为 1/s，Kd 为无量纲，不由电机 R/L 自动求得。 */
#define PARAM_APP_CASCADE_POSITION_KP 0.05f
#define PARAM_APP_CASCADE_POSITION_KD 0.50f

#endif
