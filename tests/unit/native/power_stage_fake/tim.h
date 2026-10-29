#ifndef POWER_STAGE_FAKE_TIM_H
#define POWER_STAGE_FAKE_TIM_H

#include <stdint.h>

#include "official_tim_defs.h"

/* C++寄存器单元仅用于离线逐次记录读写，不模拟定时器波形或测量耗时。 */
struct TimRegister
{
    uint32_t value;
    uint32_t identity;
    operator uint32_t() const;
    TimRegister &operator=(uint32_t next);
    TimRegister &operator&=(uint32_t mask);
    TimRegister &operator|=(uint32_t mask);
};

struct TIM_TypeDef
{
    TimRegister CR1, CR2, SMCR, CCER, BDTR, CNT, PSC, ARR, RCR;
    TimRegister CCR1, CCR2, CCR3, CCR4, DIER, SR, EGR, CCMR1, CCMR2;
};

extern TIM_TypeDef g_tim1;
extern TIM_TypeDef g_tim8;
#define TIM1 (&g_tim1)
#define TIM8 (&g_tim8)
#define RESET 0U
#define assert_param(expression) ((void)0U)
#define __STATIC_FORCEINLINE __attribute__((always_inline)) static inline

typedef struct
{
    TIM_TypeDef *Instance;
    uint32_t State, Lock;
    volatile HAL_TIM_ChannelStateTypeDef ChannelState[6];
    volatile HAL_TIM_ChannelStateTypeDef ChannelNState[4];
} TIM_HandleTypeDef;

extern TIM_HandleTypeDef htim1;
/** @brief 初始化路径的离线桩；运行期轨迹测试不调用本函数。 */
void MX_TIM1_Init(void);
/**
 * @brief 离线替身记录普通通道启动调用，并模拟HAL状态变更。
 * @param handle 测试拥有的定时器句柄，不访问实际硬件。
 * @param channel 待启动的HAL通道标识。
 * @return 模拟HAL执行状态。
 */
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *handle, uint32_t channel);
/**
 * @brief 离线替身记录互补通道启动调用，并模拟HAL状态变更。
 * @param handle 测试拥有的定时器句柄，不访问实际硬件。
 * @param channel 待启动的HAL通道标识。
 * @return 模拟HAL执行状态。
 */
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef *handle, uint32_t channel);
/**
 * @brief 离线替身记录普通通道停止调用，并模拟HAL状态变更。
 * @param handle 测试拥有的定时器句柄，不访问实际硬件。
 * @param channel 待停止的HAL通道标识。
 * @return 模拟HAL执行状态。
 */
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *handle, uint32_t channel);
/**
 * @brief 离线替身记录互补通道停止调用，并模拟HAL状态变更。
 * @param handle 测试拥有的定时器句柄，不访问实际硬件。
 * @param channel 待停止的HAL通道标识。
 * @return 模拟HAL执行状态。
 */
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef *handle, uint32_t channel);

/** @brief 捕获模拟致命路径的中断屏蔽，不访问硬件。 */
void __disable_irq(void);
/** @brief 模拟致命停留，测试通过终止捕获禁止返回的路径。 */
void __WFI(void);
#ifndef TIM_CR1_DIR
#define TIM_CR1_DIR (1U << 4U)
#endif
#endif
