#ifndef STM32_INDICATOR_FAKE_TIM_H
#define STM32_INDICATOR_FAKE_TIM_H
#include <stdint.h>
extern unsigned htim2;
#define TIM_CHANNEL_3 3U
#define HAL_TIM_CHANNEL_STATE_READY 0U
/**
 * @brief 返回测试配置的DMA通道忙状态。
 * @param timer 测试句柄。
 * @param channel 被查询通道。
 * @return 就绪或繁忙的模拟HAL状态。
 */
unsigned HAL_TIM_GetChannelState(unsigned *timer, unsigned channel);
/**
 * @brief 记录真实指示器编码的DMA数据；不模拟灯带时序。
 * @param timer 模拟TIM2句柄。
 * @param channel 传输通道。
 * @param data 编码后的数据。
 * @param length 数据字数。
 * @return 固定成功。
 */
int HAL_TIM_PWM_Start_DMA(unsigned *timer, unsigned channel, uint32_t *data, unsigned length);
#endif
