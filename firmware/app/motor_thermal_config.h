#ifndef MOTOR_THERMAL_CONFIG_H
#define MOTOR_THERMAL_CONFIG_H

/* 3 A暂按d/q向量幅值解释；8 A/10 s仅用于离线对照，尚未实测热安全边界。 */
#define MOTOR_THERMAL_CONTINUOUS_VECTOR_A 3.0f
#define MOTOR_THERMAL_CONTINUOUS_PHASE_RMS_A (MOTOR_THERMAL_CONTINUOUS_VECTOR_A * 0.707106781f)
#define MOTOR_THERMAL_PEAK_A 7.0f
#define MOTOR_THERMAL_HEATING_TAU_S 65.98486785f
/* 暂用较慢冷却，不代表已辨识；正式发布前需确认最坏安装及环境条件。 */
#define MOTOR_THERMAL_COOLING_MIN_TAU_S 120.0f
#define MOTOR_THERMAL_COOLING_TAU_S                                                                \
    ((MOTOR_THERMAL_HEATING_TAU_S > MOTOR_THERMAL_COOLING_MIN_TAU_S)                               \
         ? MOTOR_THERMAL_HEATING_TAU_S                                                             \
         : MOTOR_THERMAL_COOLING_MIN_TAU_S)
#define MOTOR_THERMAL_FAST_SAMPLES 20U
#define MOTOR_THERMAL_UPDATE_PERIOD_S 0.001f
#define MOTOR_THERMAL_DERATING_ENTER_HEAT 0.8f
#define MOTOR_THERMAL_DERATING_EXIT_HEAT 0.7f
#define MOTOR_THERMAL_TRIP_HEAT 1.0f
#define MOTOR_THERMAL_CLEAR_HEAT 0.5f

/* 用户授权默认放行；热降额、运行故障锁存和采样监督仍有效。 */
#ifndef MOTOR_THERMAL_PARAMETERS_CONFIRMED
#define MOTOR_THERMAL_PARAMETERS_CONFIRMED 1U
#endif

#endif
