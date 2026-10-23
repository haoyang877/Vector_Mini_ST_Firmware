#ifndef MOTOR_PROFILE_DEFAULTS_H
#define MOTOR_PROFILE_DEFAULTS_H

#include "motor_profile_selection.h"

/* 完整APP仍从此入口取得默认值；未知磁链/热参数不得从其他型号借用。
 * 独立台架仅需已知电气输入时使用motor_profile_selection.h。 */
#if !PARAM_MOTOR_APP_DEFAULTS_READY
#if defined(APP_TARGET_R40_F456) && APP_TARGET_R40_F456 &&                                         \
    ((defined(R40_APP_PHASE_LOCK) && R40_APP_PHASE_LOCK == 1) ||                                   \
     (defined(R40_APP_BOUNDED_TEST) && R40_APP_BOUNDED_TEST == 1))
/* 关相及获准限时目标采用用户指定的临时热时间常数；不作为正式热设计参数。
 * 已确认磁链从型号配置取得；其他缺磁链的型号仍保留无效占位。 */
#ifndef PARAM_MOTOR_FLUX_WB
#define PARAM_MOTOR_FLUX_WB 0.0f
#endif
#define PARAM_MOTOR_THERMAL_CONTINUOUS_VECTOR_A 3.0f
#define PARAM_MOTOR_THERMAL_PEAK_A 3.0f
#define PARAM_MOTOR_THERMAL_HEATING_TAU_S 3600.0f
#define PARAM_MOTOR_THERMAL_DERATING_ENTER_RATIO 0.8f
#define PARAM_MOTOR_THERMAL_DERATING_EXIT_RATIO 0.7f
#define PARAM_MOTOR_THERMAL_TRIP_RATIO 1.0f
#define PARAM_MOTOR_THERMAL_CLEAR_RATIO 0.5f
#else
#error "Motor APP defaults incomplete: confirm flux and thermal parameters before APP integration"
#endif
#endif

#endif
