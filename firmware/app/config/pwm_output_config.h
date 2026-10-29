#ifndef PWM_OUTPUT_CONFIG_H
#define PWM_OUTPUT_CONFIG_H

#include "bsp_config.h"

/* 目标许可与PWM硬件能力分离；历史AT32目标宏仅在此转换，不进入共用控制层。 */
#define BSP_PWM_PERMIT_UNRESTRICTED 0
#define BSP_PWM_PERMIT_LOCKED 1
#define BSP_PWM_PERMIT_WINDOW 2

#ifndef BSP_PWM_PERMIT_MODE
#if defined(R40_APP_PHASE_LOCK) && R40_APP_PHASE_LOCK
#define BSP_PWM_PERMIT_MODE BSP_PWM_PERMIT_LOCKED
#elif defined(R40_APP_BOUNDED_TEST) && R40_APP_BOUNDED_TEST
#define BSP_PWM_PERMIT_MODE BSP_PWM_PERMIT_WINDOW
#else
#define BSP_PWM_PERMIT_MODE BSP_PWM_PERMIT_UNRESTRICTED
#endif
#endif

/* 硬件锁必须来自统一构建输入，不能按本编译单元的包含顺序推导。 */
#if BSP_PWM_HARD_LOCK != (BSP_PWM_PERMIT_MODE == BSP_PWM_PERMIT_LOCKED)
#error "Application PWM permit and BSP hard lock disagree"
#endif

#if BSP_PWM_PERMIT_MODE < BSP_PWM_PERMIT_UNRESTRICTED || BSP_PWM_PERMIT_MODE > BSP_PWM_PERMIT_WINDOW
#error "Invalid PWM output permit mode"
#endif
#if defined(R40_APP_PHASE_LOCK) && R40_APP_PHASE_LOCK &&                                           \
    BSP_PWM_PERMIT_MODE != BSP_PWM_PERMIT_LOCKED
#error "Phase-locked target cannot override its PWM permit"
#endif
#if defined(R40_APP_BOUNDED_TEST) && R40_APP_BOUNDED_TEST &&                                       \
    BSP_PWM_PERMIT_MODE != BSP_PWM_PERMIT_WINDOW
#error "Bounded target must retain its PWM time window"
#endif

#endif
