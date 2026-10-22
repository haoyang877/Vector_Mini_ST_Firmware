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
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *handle, uint32_t channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef *handle, uint32_t channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *handle, uint32_t channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef *handle, uint32_t channel);

#endif
