/* 实际板级关相与核对代码运行在寄存器模型上，不接触任何硬件。 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct
{
    uint32_t MODER, OTYPER, OSPEEDR, PUPDR, ODR, BSRR;
} GPIO_TypeDef;
static GPIO_TypeDef gpio_a, gpio_b;
static struct
{
    uint32_t AHB2ENR;
} rcc;
static struct
{
    uint32_t BDTR;
} timer;
#define GPIOA (&gpio_a)
#define GPIOB (&gpio_b)
#define RCC (&rcc)
#define TIM1 (&timer)
#define RCC_AHB2ENR_GPIOAEN 1U
#define RCC_AHB2ENR_GPIOBEN 2U
#define TIM_BDTR_MOE (1U << 15U)
void LoaderBoard_Safe(void);
bool LoaderBoard_OutputsOff(void);

int main(void)
{
    gpio_a.MODER = gpio_b.MODER = UINT32_MAX;
    gpio_a.OTYPER = gpio_b.OTYPER = UINT32_MAX;
    gpio_a.OSPEEDR = gpio_b.OSPEEDR = UINT32_MAX;
    gpio_a.PUPDR = gpio_b.PUPDR = UINT32_MAX;
    LoaderBoard_Safe();
    assert((rcc.AHB2ENR & 3U) == 3U);
    assert(gpio_a.BSRR == (0x780U << 16U) && gpio_b.BSRR == (3U << 16U));
    assert(LoaderBoard_OutputsOff());
    timer.BDTR = TIM_BDTR_MOE;
    assert(!LoaderBoard_OutputsOff());
    timer.BDTR = 0;
    gpio_a.ODR = 1U << 7U;
    assert(!LoaderBoard_OutputsOff());
    gpio_a.ODR = 0;
    gpio_b.ODR = 1U;
    assert(!LoaderBoard_OutputsOff());
    gpio_b.ODR = 0;
    gpio_a.MODER &= ~(3U << 14U);
    assert(!LoaderBoard_OutputsOff());
    LoaderBoard_Safe();
    gpio_b.OTYPER |= 1U;
    assert(!LoaderBoard_OutputsOff());
    LoaderBoard_Safe();
    assert(LoaderBoard_OutputsOff());
    puts("PASS board-owned safe pin configuration and output verification");
    return 0;
}
