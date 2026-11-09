/* YG_ESC_V1.0独立板级端口；芯片SDK边界，不运行APP外设和控制任务。 */
#include "board.h"
#include "at32f45x.h"

static bool clock_ready(uint32_t flag)
{
    for (uint32_t n = 0; n < 2000000U; ++n)
    {
        if (crm_flag_get(flag) != RESET)
        {
            return true;
        }
    }
    return false;
}
void LoaderBoard_Safe(void)
{
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    gpio_bits_reset(GPIOA, GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11);
    gpio_bits_reset(GPIOB, GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4);
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_mode = GPIO_MODE_OUTPUT;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_NONE;
    pins.gpio_pins = GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11;
    gpio_init(GPIOA, &pins);
    pins.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4;
    gpio_init(GPIOB, &pins);
}
bool LoaderBoard_OutputsOff(void)
{
    return !(GPIOA->idt & 0xE00U) && !(GPIOB->idt & 0x13U) && !(GPIOA->odt & 0xE00U) &&
           !(GPIOB->odt & 0x13U) && !(GPIOA->omode & 0xE00U) && !(GPIOB->omode & 0x13U) &&
           (GPIOA->cfgr & 0xFC0000U) == 0x540000U && (GPIOB->cfgr & 0x30FU) == 0x105U &&
           !(TMR1->brk & 0x8000U);
}
bool LoaderBoard_Init(void)
{
    LoaderBoard_Safe();
    flash_psr_set(FLASH_WAIT_CYCLE_5);
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    if (!clock_ready(CRM_HEXT_STABLE_FLAG))
    {
        return false;
    }
    crm_pll_config(CRM_PLL_SOURCE_HEXT, 90U, 1U, CRM_PLL_FP_4);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    if (!clock_ready(CRM_PLL_STABLE_FLAG))
    {
        return false;
    }
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb1_div_set(CRM_APB1_DIV_1);
    crm_apb2_div_set(CRM_APB2_DIV_1);
    crm_apb3_div_set(CRM_APB3_DIV_4);
    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(CRM_SCLK_PLL);
    for (uint32_t n = 0; n < 2000000U; ++n)
    {
        if (crm_sysclk_switch_status_get() == CRM_SCLK_PLL)
        {
            crm_auto_step_mode_enable(FALSE);
            system_core_clock_update();
            return system_core_clock == LOADER_AT32_CLOCK_HZ && LoaderBoard_OutputsOff();
        }
    }
    return false;
}
void LoaderBoard_CanInit(void)
{
    crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);
    crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_PCLK);
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_mode = GPIO_MODE_MUX;
    pins.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_NONE;
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE8, GPIO_MUX_9);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE9, GPIO_MUX_9);
    gpio_init(GPIOB, &pins);
}
