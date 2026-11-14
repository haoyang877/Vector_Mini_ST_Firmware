#include "../../bsp_internal.h"
#include "bsp_system.h"
#include "mcu_resources.h"
#include "at32f45x.h"

void BspSystem_WriteStatusLights(bool red, bool blue)
{
    uint32_t on = (red ? MCU_STATUS_LED_RED_PIN : 0U) | (blue ? MCU_STATUS_LED_BLUE_PIN : 0U);
    /* SCR 高半字拉低点亮，低半字拉高熄灭，两路在同一次写入中更新。 */
    MCU_STATUS_LED_PORT->scr =
        (on << 16U) | ((MCU_STATUS_LED_RED_PIN | MCU_STATUS_LED_BLUE_PIN) & ~on);
}

void McuStatusLights_Init(void)
{
    crm_periph_clock_enable(MCU_STATUS_LED_CLOCK, TRUE);
    BspSystem_WriteStatusLights(false, false);
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_pins = MCU_STATUS_LED_RED_PIN | MCU_STATUS_LED_BLUE_PIN;
    pins.gpio_mode = GPIO_MODE_OUTPUT;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_NONE;
    pins.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init(MCU_STATUS_LED_PORT, &pins);
}
