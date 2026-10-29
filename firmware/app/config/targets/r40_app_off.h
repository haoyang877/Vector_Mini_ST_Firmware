#ifndef R40_APP_OFF_CONFIG_H
#define R40_APP_OFF_CONFIG_H

/* 永久关相目标：保留原有90Hz速度估计及默认速度PI。 */
#define R40_APP_PHASE_LOCK 1
#define BSP_PWM_HARD_LOCK 1
#define BSP_ENABLED_MCU_TEMPERATURE 1
#define BSP_ENABLED_OUTPUT_SINCOS 1
#include "r40_app_common.h"

#endif
