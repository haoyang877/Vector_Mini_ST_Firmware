#include "bsp_pwm.h"
#include "at32f45x.h"
#include "bsp_system.h"
#include "control_config.h"
#include "bsp_config.h"
#include "bsp_internal.h"

#define PWM_TOP (R40_TIMER_HZ / (2U * FOC_FREQ))
#define PWM_DEADTIME_NS BSP_PWM_DEADTIME_NS

#if BSP_PWM_PHASE_A_CHANNEL == 1
#define PWM_COMPARE_A TMR1->c1dt
#elif BSP_PWM_PHASE_A_CHANNEL == 2
#define PWM_COMPARE_A TMR1->c2dt
#elif BSP_PWM_PHASE_A_CHANNEL == 3
#define PWM_COMPARE_A TMR1->c3dt
#elif BSP_PWM_PHASE_A_CHANNEL == 4
#define PWM_COMPARE_A TMR1->c4dt
#else
#error "Unsupported PWM compare channel"
#endif
#if BSP_PWM_PHASE_B_CHANNEL == 1
#define PWM_COMPARE_B TMR1->c1dt
#elif BSP_PWM_PHASE_B_CHANNEL == 2
#define PWM_COMPARE_B TMR1->c2dt
#elif BSP_PWM_PHASE_B_CHANNEL == 3
#define PWM_COMPARE_B TMR1->c3dt
#elif BSP_PWM_PHASE_B_CHANNEL == 4
#define PWM_COMPARE_B TMR1->c4dt
#else
#error "Unsupported PWM compare channel"
#endif
#if BSP_PWM_PHASE_C_CHANNEL == 1
#define PWM_COMPARE_C TMR1->c1dt
#elif BSP_PWM_PHASE_C_CHANNEL == 2
#define PWM_COMPARE_C TMR1->c2dt
#elif BSP_PWM_PHASE_C_CHANNEL == 3
#define PWM_COMPARE_C TMR1->c3dt
#elif BSP_PWM_PHASE_C_CHANNEL == 4
#define PWM_COMPARE_C TMR1->c4dt
#else
#error "Unsupported PWM compare channel"
#endif

/* 定时器原有寄存器顺序和HAL状态更新保持。 */
#define PWM_MAIN_OUTPUT_MASK ((1U << 4U) | (1U << 8U) | (1U << 12U))
#define PWM_COMPLEMENT_OUTPUT_MASK ((1U << 6U) | (1U << 10U) | (1U << 14U))
#define PWM_ALL_OUTPUT_MASK (PWM_MAIN_OUTPUT_MASK | PWM_COMPLEMENT_OUTPUT_MASK)

void McuPwm_DisableOutputs(void)
{
    tmr_output_enable(TMR1, FALSE);
    TMR1->cctrl &= ~PWM_MAIN_OUTPUT_MASK;
    TMR1->cctrl &= ~PWM_COMPLEMENT_OUTPUT_MASK;
}
bool BspPwm_ArePhasesDisabled(void)
{
    return (TMR1->cctrl & PWM_ALL_OUTPUT_MASK) == 0U;
}
float McuPwm_PeriodCounts(void)
{
    return (float)TMR1->pr;
}
/* 仅运行/部分输出路径检查期限；剩余计数复用原计数寄存器值，减少热路径活跃变量。 */
static inline bool pwm_can_commit(uint32_t control)
{
    if ((control & 1U) == 0U)
    {
        return true;
    }
    uint32_t remaining = TMR1->cval, period = TMR1->pr;
    if ((control & (1U << 4U)) == 0U)
    {
        remaining = period - remaining;
    }
    /* 上数时原count>period会无符号回绕为大于period；两方向共用同一范围检查。 */
    return remaining <= period && remaining > BSP_PWM_COMMIT_GUARD_COUNTS;
}

bool McuPwm_CommitThree(uint16_t a, uint16_t b, uint16_t c)
{
    if (BspPwm_ArePhasesDisabled() && (TMR1->brk & 0x8000U) == 0U)
    {
        /* 关相预置单独完成更新屏蔽/恢复，不把该状态带入正常提交路径。 */
        uint32_t control = TMR1->ctrl1;
        TMR1->ctrl1 = control | (1U << 1U);
        PWM_COMPARE_A = a;
        PWM_COMPARE_B = b;
        PWM_COMPARE_C = c;
        TMR1->ctrl1 = control;
        return true;
    }
    if (!pwm_can_commit(TMR1->ctrl1))
    {
        return false;
    }
    PWM_COMPARE_A = a;
    PWM_COMPARE_B = b;
    PWM_COMPARE_C = c;
    return true;
}

bool McuPwm_CommitPhase(BspPwmPhase phase, uint16_t counts)
{
    if (!(BspPwm_ArePhasesDisabled() && (TMR1->brk & 0x8000U) == 0U) &&
        !pwm_can_commit(TMR1->ctrl1))
    {
        return false;
    }
    switch (phase)
    {
    case BSP_PWM_PHASE_A:
        PWM_COMPARE_A = counts;
        break;
    case BSP_PWM_PHASE_B:
        PWM_COMPARE_B = counts;
        break;
    case BSP_PWM_PHASE_C:
        PWM_COMPARE_C = counts;
        break;
    default:
        return false;
    }
    return true;
}

void McuPwm_EnableOutputs(void)
{
    TMR1->cctrl |= PWM_COMPLEMENT_OUTPUT_MASK;
    TMR1->cctrl |= PWM_MAIN_OUTPUT_MASK;
    tmr_output_enable(TMR1, TRUE);
}
void McuPwm_ReadOutputs(bool *any_channel, bool *all_channels, bool *master)
{
    uint32_t channels = TMR1->cctrl & PWM_ALL_OUTPUT_MASK;
    *any_channel = channels != 0U;
    *all_channels = channels == PWM_ALL_OUTPUT_MASK;
    *master = (TMR1->brk & 0x8000U) != 0U;
}

/* 上电配置与运行操作同属本定时器实现，保持原校准/使能顺序。 */
void McuPwm_Init(void)
{
    tmr_base_init(TMR1, PWM_TOP, 0U);
    tmr_cnt_dir_set(TMR1, TMR_COUNT_TWO_WAY_2);
    tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);
    tmr_period_buffer_enable(TMR1, TRUE);
    tmr_output_config_type output;
    tmr_output_default_para_init(&output);
    /* 模式A使CCR增大对应上桥导通时间增大；不沿用SDK的反向模式B。 */
    output.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    output.oc_output_state = FALSE;
    output.occ_output_state = FALSE;
    output.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    output.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    output.oc_idle_state = FALSE;
    output.occ_idle_state = FALSE;
    for (uint32_t phase = 0U; phase < 3U; ++phase)
    {
        tmr_channel_select_type channel =
            (tmr_channel_select_type)(TMR_SELECT_CHANNEL_2 + 2U * phase);
        tmr_output_channel_config(TMR1, channel, &output);
        tmr_channel_value_set(TMR1, channel, PWM_TOP / 2U);
        tmr_output_channel_buffer_enable(TMR1, channel, TRUE);
    }
    tmr_brkdt_config_type protection;
    tmr_brkdt_default_para_init(&protection);
    protection.auto_output_enable = FALSE;
    protection.fcsoen_state = TRUE;
    protection.fcsodis_state = TRUE;
    protection.brk_enable = FALSE;
    protection.deadtime = (R40_TIMER_HZ / 1000000U) * PWM_DEADTIME_NS / 1000U;
    tmr_brkdt_config(TMR1, &protection);
    BspPwm_Disable();
}

void McuPwm_Fatal(void)
{
    __disable_irq();
    for (;;)
    {
        __WFI();
    }
}

void McuPwm_PreparePins(void)
{
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    gpio_bits_reset(GPIOA, GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11);
    gpio_bits_reset(GPIOB, GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4);
    gpio_bits_set(GPIOB, GPIO_PINS_12);
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_mode = GPIO_MODE_OUTPUT;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_DOWN;
    pins.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    pins.gpio_pins = GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11;
    gpio_init(GPIOA, &pins);
    pins.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4;
    gpio_init(GPIOB, &pins);
    pins.gpio_pins = GPIO_PINS_12;
    pins.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOB, &pins);
}

void McuPwm_ConnectPins(void)
{
    const uint16_t upper_pins = GPIO_PINS_9 | GPIO_PINS_10 | GPIO_PINS_11;
    const uint16_t lower_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_4;
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE11, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_1);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE4, GPIO_MUX_1);
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_mode = GPIO_MODE_MUX;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_DOWN;
    pins.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    pins.gpio_pins = upper_pins;
    gpio_init(GPIOA, &pins);
    pins.gpio_pins = lower_pins;
    gpio_init(GPIOB, &pins);
}
