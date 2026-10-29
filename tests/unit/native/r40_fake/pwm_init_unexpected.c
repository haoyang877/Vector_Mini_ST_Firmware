/* 运行期功率夹具不应重新初始化硬件；基础启动夹具另行验证真实初始化顺序。 */
#include "at32f45x.h"
#include <stdlib.h>

gpio_type fake_gpioa, fake_gpiob;
void crm_periph_clock_enable(unsigned clock, unsigned enabled)
{
    (void)clock;
    (void)enabled;
    abort();
}
void gpio_bits_reset(gpio_type *gpio, unsigned pins)
{
    (void)gpio;
    (void)pins;
    abort();
}
void gpio_bits_set(gpio_type *gpio, unsigned pins)
{
    (void)gpio;
    (void)pins;
    abort();
}
void gpio_default_para_init(gpio_init_type *pins)
{
    (void)pins;
    abort();
}
void gpio_init(gpio_type *gpio, gpio_init_type *pins)
{
    (void)gpio;
    (void)pins;
    abort();
}
void gpio_pin_mux_config(gpio_type *gpio, unsigned pin, unsigned mux)
{
    (void)gpio;
    (void)pin;
    (void)mux;
    abort();
}
void tmr_base_init(tmr_type *timer, uint32_t period, uint32_t divider)
{
    (void)timer;
    (void)period;
    (void)divider;
    abort();
}
void tmr_cnt_dir_set(tmr_type *timer, unsigned direction)
{
    (void)timer;
    (void)direction;
    abort();
}
void tmr_clock_source_div_set(tmr_type *timer, unsigned divider)
{
    (void)timer;
    (void)divider;
    abort();
}
void tmr_period_buffer_enable(tmr_type *timer, unsigned enabled)
{
    (void)timer;
    (void)enabled;
    abort();
}
void tmr_output_default_para_init(tmr_output_config_type *config)
{
    (void)config;
    abort();
}
void tmr_output_channel_config(tmr_type *timer, unsigned channel, tmr_output_config_type *config)
{
    (void)timer;
    (void)channel;
    (void)config;
    abort();
}
void tmr_channel_value_set(tmr_type *timer, unsigned channel, uint32_t value)
{
    (void)timer;
    (void)channel;
    (void)value;
    abort();
}
void tmr_output_channel_buffer_enable(tmr_type *timer, unsigned channel, unsigned enabled)
{
    (void)timer;
    (void)channel;
    (void)enabled;
    abort();
}
void tmr_brkdt_default_para_init(tmr_brkdt_config_type *config)
{
    (void)config;
    abort();
}
void tmr_brkdt_config(tmr_type *timer, tmr_brkdt_config_type *config)
{
    (void)timer;
    (void)config;
    abort();
}

void __disable_irq(void)
{
    abort();
}
void __WFI(void)
{
    abort();
}
