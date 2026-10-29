#ifndef MOTOR_PROFILE_DEFAULTS_H
#define MOTOR_PROFILE_DEFAULTS_H

#include "motor_profile_selection.h"

/* 完整APP仍从此入口取得默认值；型号数据不全时，仅允许目标配置显式提供测试值。
 * 此处不识别板型或MCU，正式默认值仍必须完整。 */
#if !PARAM_MOTOR_APP_DEFAULTS_READY
#if !defined(PARAM_MOTOR_APP_TEST_DEFAULTS) || PARAM_MOTOR_APP_TEST_DEFAULTS != 1
#error "Motor APP defaults incomplete: confirm flux and thermal parameters before APP integration"
#endif
#if !defined(PARAM_MOTOR_FLUX_WB) || !defined(PARAM_MOTOR_THERMAL_CONTINUOUS_VECTOR_A) ||          \
    !defined(PARAM_MOTOR_THERMAL_PEAK_A) || !defined(PARAM_MOTOR_THERMAL_HEATING_TAU_S) ||         \
    !defined(PARAM_MOTOR_THERMAL_DERATING_ENTER_RATIO) ||                                          \
    !defined(PARAM_MOTOR_THERMAL_DERATING_EXIT_RATIO) ||                                           \
    !defined(PARAM_MOTOR_THERMAL_TRIP_RATIO) || !defined(PARAM_MOTOR_THERMAL_CLEAR_RATIO)
#error "Motor APP defaults incomplete: test target must supply flux and thermal parameters"
#endif
#endif

#endif
