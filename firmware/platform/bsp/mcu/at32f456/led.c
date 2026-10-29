/* 只拥有PC13/PC14低有效GPIO与1ms亮度调制，闪烁语义由共用BSP决定。 */
#include "bsp_internal.h"
#include "at32f45x.h"

static bool s_initialized;
/* 引脚和亮度作为一个对齐字发布，SysTick不会读到半个更新。 */
static volatile uint32_t s_output;
static uint8_t s_pwm_step;

void McuLed_Init(void)
{
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    gpio_bits_set(GPIOC, BSP_LED_RED_PIN | BSP_LED_BLUE_PIN);
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_pins = BSP_LED_RED_PIN | BSP_LED_BLUE_PIN;
    pins.gpio_mode = GPIO_MODE_OUTPUT;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_NONE;
    pins.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init(GPIOC, &pins);
    s_initialized = true;
}

void McuLed_WriteColor(BspLedColor color, uint8_t brightness)
{
    static const uint16_t s_pins[] = {0U,
                                      BSP_LED_RED_PIN,
                                      BSP_LED_BLUE_PIN,
                                      BSP_LED_BLUE_PIN,
                                      BSP_LED_RED_PIN,
                                      BSP_LED_RED_PIN | BSP_LED_BLUE_PIN,
                                      BSP_LED_BLUE_PIN,
                                      BSP_LED_RED_PIN | BSP_LED_BLUE_PIN};
    s_output = ((uint32_t)brightness << 16U) | s_pins[color];
}

void McuLed_Tick1ms(void)
{
    if (!s_initialized)
    {
        return;
    }
    uint32_t output = s_output;
    uint16_t active = (output >> 16U) > s_pwm_step ? (uint16_t)output : 0U;
    s_pwm_step = (uint8_t)((s_pwm_step + 1U) % 100U);
    gpio_bits_set(GPIOC, (BSP_LED_RED_PIN | BSP_LED_BLUE_PIN) & (uint16_t)~active);
    gpio_bits_reset(GPIOC, active);
}
