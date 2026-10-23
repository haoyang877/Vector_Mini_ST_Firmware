#include "r40_clock.h"

#include "at32f45x.h"

/* 等待上限是启动阶段的轮询次数，不冒充准确的毫秒超时。 */
#define R40_CLOCK_WAIT_ITERATIONS 2000000U

static bool wait_clock_flag(uint32_t flag)
{
    for (uint32_t remaining = R40_CLOCK_WAIT_ITERATIONS; remaining > 0U; --remaining)
    {
        if (crm_flag_get(flag) == SET)
        {
            return true;
        }
    }
    return false;
}

bool R40_ClockInit(void)
{
    /* 复用SDK在192MHz使用的5等待周期和1.3V内核档位，不降低访问裕量。 */
    flash_psr_set(FLASH_WAIT_CYCLE_5);
    crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);
    pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
    if (!wait_clock_flag(CRM_HEXT_STABLE_FLAG))
    {
        return false;
    }
    crm_pll_config(CRM_PLL_SOURCE_HEXT, R40_PLL_MULTIPLIER, R40_PLL_PRE_DIVIDER, CRM_PLL_FP_4);
    crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
    if (!wait_clock_flag(CRM_PLL_STABLE_FLAG))
    {
        return false;
    }
    crm_ahb_div_set(CRM_AHB_DIV_1);
    crm_apb1_div_set(CRM_APB1_DIV_1);
    crm_apb2_div_set(CRM_APB2_DIV_1);
    crm_apb3_div_set(CRM_APB3_DIV_4);
    crm_auto_step_mode_enable(TRUE);
    crm_sysclk_switch(CRM_SCLK_PLL);
    bool switched = false;
    for (uint32_t remaining = R40_CLOCK_WAIT_ITERATIONS; remaining > 0U; --remaining)
    {
        if (crm_sysclk_switch_status_get() == CRM_SCLK_PLL)
        {
            switched = true;
            break;
        }
    }
    crm_auto_step_mode_enable(FALSE);
    if (!switched)
    {
        return false;
    }
    system_core_clock_update();
    crm_clocks_freq_type clocks;
    crm_clocks_freq_get(&clocks);
    return system_core_clock == R40_CORE_HZ && clocks.ahb_freq == R40_CORE_HZ &&
           clocks.apb1_freq == R40_CORE_HZ && clocks.apb2_freq == R40_CORE_HZ &&
           CRM->misc2_bit.apb3div == CRM_APB3_DIV_4;
}
