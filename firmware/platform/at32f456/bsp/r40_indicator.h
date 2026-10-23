#ifndef R40_INDICATOR_H
#define R40_INDICATOR_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 初始化PC13/PC14为低有效红/蓝灯，保持PB5/PB6未配置。
 * @note 仅启动时调用，先熄灯后配置输出；不改变功率资源。
 */
void R40_IndicatorInit(void);
/** @brief 推进一次1ms软件调光，闪烁码优先于降级颜色。
 * @note 由SysTick独占调用；无忙等，不使用功率PWM定时器。
 */
void R40_IndicatorTick(void);
/** @brief 在数字关相时选择指示灯诊断状态。
 * @param pattern 0恢复应用指示，1红、2蓝、3双亮、4全灭。
 * @return 参数有效且当前关相返回true，否则不改变状态。
 * @note 仅前台诊断使用，不改变功率或应用状态。
 */
bool R40_IndicatorDiagnostic(uint32_t pattern);

#endif
