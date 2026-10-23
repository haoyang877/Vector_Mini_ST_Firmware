#ifndef MOTOR_PROFILE_SELECTION_H
#define MOTOR_PROFILE_SELECTION_H

#include "motor_hardware_profile.h"

/* 只读电机选择，与板卡选择独立；生产型号标签不自动切换配置。 */
#define FOC_MOTOR_PROFILE_HT8115_4 1U
#define FOC_MOTOR_PROFILE_YQ_5708 2U
#define FOC_MOTOR_PROFILE_YQ_5740 3U
#define FOC_MOTOR_PROFILE_YQ_8116 4U
#ifndef FOC_ACTIVE_MOTOR_PROFILE
#define FOC_ACTIVE_MOTOR_PROFILE FOC_MOTOR_PROFILE_HT8115_4
#endif

#if MOTOR_HAS_DAMPING_RING != MOTOR_DAMPING_RING_DISABLED &&                                       \
    MOTOR_HAS_DAMPING_RING != MOTOR_DAMPING_RING_ENABLED
#error "Unsupported MOTOR_HAS_DAMPING_RING"
#endif

#if FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_HT8115_4
#include "profiles/ht8115_4.h"
#elif FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_YQ_5708
#include "profiles/yq_5708.h"
#elif FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_YQ_5740
#include "profiles/yq_5740.h"
#elif FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_YQ_8116
#include "profiles/yq_8116.h"
#else
#error "Unsupported FOC_ACTIVE_MOTOR_PROFILE"
#endif

/* 目前仅HT两种阻尼环装配有既有标定配置；其他电机不能自动套用。 */
#if FOC_ACTIVE_MOTOR_PROFILE != FOC_MOTOR_PROFILE_HT8115_4 && MOTOR_HAS_DAMPING_RING != 0U
#error "Damping-ring configuration is only defined for HT8115-4"
#endif

/* 所有电机统一1kHz带宽；快环仍为20kHz，Ki不预乘采样周期。 */
#define PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_HZ 1000.0f
#define PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S                                                   \
    (PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_HZ * 6.283185307f)
#define PARAM_MOTOR_DEFAULT_ID_KP                                                                  \
    ((float)PARAM_MOTOR_D_INDUCTANCE_H * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S)
#define PARAM_MOTOR_DEFAULT_IQ_KP                                                                  \
    ((float)PARAM_MOTOR_Q_INDUCTANCE_H * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S)
#define PARAM_MOTOR_DEFAULT_ID_KI                                                                  \
    ((float)PARAM_MOTOR_PHASE_RESISTANCE_OHM * PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S)
#define PARAM_MOTOR_DEFAULT_IQ_KI PARAM_MOTOR_DEFAULT_ID_KI

#endif
