#ifndef R40_APP_COMMISSIONING_CONFIG_H
#define R40_APP_COMMISSIONING_CONFIG_H

/* 限时实机验证；没有启动请求或功率窗口时不能使能。
 * 测速带宽与速度PI须配对验收，不能直接降低带宽而沿用较高比例增益。 */
#define R40_APP_PHASE_LOCK 0
#define R40_APP_BOUNDED_TEST 1
#define BSP_WATCHDOG_RELOAD 2047
#define BSP_ENABLED_MCU_TEMPERATURE 0
#define BSP_ENABLED_FAST_LOOP_PROFILE 0
/* 标定和闭环走CAN验收；不在20kHz控制IRQ中生成后台RTT流。 */
#define APP_ENABLED_DIAGNOSTIC_TELEMETRY 0
/* 用户指定的临时耗时隔离：只关闭绕组热模型，仍保留固定限流和限时关相。 */
#define APP_ENABLED_WINDING_THERMAL_MODEL 0
#define ENCODER_PLL_OMEGA_N_HZ 800.0f
#define PARAM_APP_SPEED_KP 0.005f
#define PARAM_APP_SPEED_KI 0.05f
#include "r40_app_common.h"

#endif
