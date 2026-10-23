#ifndef R40_CLOCK_H
#define R40_CLOCK_H

#include <stdbool.h>

/* R40板内时钟计划；PLL输入8MHz、VCO720MHz，均满足手册范围。
 * 这些值不改变platform/api的控制时基及通信契约。 */
#define R40_HEXT_HZ 8000000U
#define R40_PLL_MULTIPLIER 90U
#define R40_PLL_PRE_DIVIDER 1U
#define R40_PLL_POST_DIVIDER 4U
#define R40_CORE_HZ 180000000U
#define R40_TIMER_HZ R40_CORE_HZ
#define R40_CAN_HZ R40_CORE_HZ
#define R40_CAN_NOMINAL_SEG1 135U
#define R40_CAN_NOMINAL_SEG2 45U
#define R40_CAN_DATA_SEG1 27U
#define R40_CAN_DATA_SEG2 9U

/**
 * @brief 从复位时钟配置到R40的180MHz计划并核对实际分频。
 * @return 晶振、PLL稳定且频率匹配返回true，超时或不匹配返回false。
 * @note 仅启动阶段调用；调用前必须令六路门极输入保持低电平。
 */
bool R40_ClockInit(void);

#endif
