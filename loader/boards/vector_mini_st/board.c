/* Loader独立板级初始化，仅冷复位入口，HSI16MHz/HSE8MHz，不启动PWM或ADC。 */
#include "stm32g4xx_hal.h"
#include "board.h"
#if HSE_VALUE != 8000000U
#error Narrow Loader clock requires the reviewed 8 MHz board crystal
#endif
void Error_Handler(void)
{
    __disable_irq();
    for (;;)
    {
    }
}
static bool await_bits(volatile uint32_t *reg, uint32_t mask, uint32_t value)
{
    uint32_t remaining = 4000000U;
    while ((*reg & mask) != value)
    {
        if (!--remaining)
        {
            return false;
        }
    }
    return true;
}
static void pins(GPIO_TypeDef *port, uint32_t first, uint32_t last, uint32_t mode)
{
    uint32_t pin, mask;
    for (pin = first; pin <= last; pin++)
    {
        mask = 3U << (pin * 2);
        port->OTYPER &= ~(1U << pin);
        port->OSPEEDR &= ~mask;
        port->PUPDR &= ~mask;
        port->MODER = (port->MODER & ~mask) | (mode << (pin * 2));
    }
}
void LoaderBoard_Safe(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;
    /* FD6288Q无独立使能；先置六路锁存低电平，再切推挽，避免门极毛刺。 */
    GPIOA->BSRR = 0x0780U << 16;
    GPIOB->BSRR = 3U << 16;
    pins(GPIOA, 7, 10, 1);
    pins(GPIOB, 0, 1, 1);
}
bool LoaderBoard_CanInit(void)
{
    MODIFY_REG(RCC->CCIPR, RCC_CCIPR_FDCANSEL, RCC_FDCANCLKSOURCE_PCLK1);
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    (void)RCC->APB1ENR1;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    (void)RCC->AHB2ENR;
    MODIFY_REG(GPIOB->AFR[1], 0xffU, 0x99U);
    pins(GPIOB, 8, 9, 2);
    return true;
}
void SystemClock_Config(void)
{
    /* 拒绝热切入的未知时钟；APP进入Loader必须使用系统复位。 */
    if ((RCC->CFGR & (RCC_CFGR_SWS | RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) !=
            RCC_CFGR_SWS_HSI ||
        (RCC->CR & (RCC_CR_PLLON | RCC_CR_HSEBYP)))
    {
        Error_Handler();
    }
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    (void)RCC->APB1ENR1;
    /* 保留ST电压等级切换与VOSF等待实现。 */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
    {
        Error_Handler();
    }
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_LATENCY_4);
    if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_LATENCY_4)
    {
        Error_Handler();
    }
    RCC->CR |= RCC_CR_HSEON;
    if (!await_bits(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY))
    {
        Error_Handler();
    }
    /* 8MHz/2*85/2=170MHz；不使用P/Q输出。 */
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | (1U << RCC_PLLCFGR_PLLM_Pos) |
                   (85U << RCC_PLLCFGR_PLLN_Pos) | RCC_PLLCFGR_PLLREN;
    RCC->CR |= RCC_CR_PLLON;
    if (!await_bits(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY))
    {
        Error_Handler();
    }
    /* 保留ST在超过80MHz切换时的HCLK二分频过渡。 */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_SYSCLK_DIV2);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_SYSCLKSOURCE_PLLCLK);
    if (!await_bits(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL))
    {
        Error_Handler();
    }
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2, RCC_HCLK_DIV16 | (RCC_HCLK_DIV16 << 3));
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_SYSCLK_DIV1);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2, 0);
    SystemCoreClock = 170000000U;
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
    {
        Error_Handler();
    }
}

bool LoaderBoard_OutputsOff(void)
{
    return !(GPIOA->ODR & 0x780U) && !(GPIOB->ODR & 3U) &&
           (GPIOA->MODER & 0x3FC000U) == 0x154000U && (GPIOB->MODER & 15U) == 5U &&
           !(GPIOA->OTYPER & 0x780U) && !(GPIOB->OTYPER & 3U) && !(TIM1->BDTR & TIM_BDTR_MOE);
}
