#ifndef R40_INDICATOR_FAKE_H
#define R40_INDICATOR_FAKE_H

/* 指示器专用GPIO替身，拒绝访问除PC13/PC14以外的板卡资源。 */
#include <stdint.h>
#include <stdbool.h>
typedef struct
{
    uint32_t odt;
} gpio_type;
typedef struct
{
    unsigned gpio_pins, gpio_mode, gpio_out_type, gpio_pull, gpio_drive_strength;
} gpio_init_type;
extern gpio_type fake_led_gpio;
#define GPIOC (&fake_led_gpio)
#define GPIO_PINS_13 (1U << 13)
#define GPIO_PINS_14 (1U << 14)
#define GPIO_MODE_OUTPUT 1U
#define GPIO_OUTPUT_PUSH_PULL 0U
#define GPIO_PULL_NONE 0U
#define GPIO_DRIVE_STRENGTH_MODERATE 1U
#define CRM_GPIOC_PERIPH_CLOCK 3U
#define TRUE 1U
/** @brief 记录GPIO时钟使能。
 * @param clock 时钟选择。
 * @param enabled 使能值。
 */
void crm_periph_clock_enable(unsigned clock, unsigned enabled);
/** @brief 清零引脚配置。
 * @param pins 配置输出。
 */
void gpio_default_para_init(gpio_init_type *pins);
/** @brief 验证引脚初始化及默认熄灯。
 * @param gpio 目标端口。
 * @param pins 引脚配置。
 */
void gpio_init(gpio_type *gpio, gpio_init_type *pins);
/** @brief 模拟置位。
 * @param gpio 目标端口。
 * @param pins 位掩码。
 */
void gpio_bits_set(gpio_type *gpio, uint16_t pins);
/** @brief 模拟清零。
 * @param gpio 目标端口。
 * @param pins 位掩码。
 */
void gpio_bits_reset(gpio_type *gpio, uint16_t pins);
#endif
