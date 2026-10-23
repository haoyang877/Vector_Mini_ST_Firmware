#ifndef R40_FAKE_AT32_H
#define R40_FAKE_AT32_H

#include <stdbool.h>
#include <stdint.h>

/* 离线寄存器替身：仅实现底层启动的外部依赖，不替代被测端口逻辑。 */
typedef unsigned confirm_state;
typedef unsigned flag_status;
typedef unsigned tmr_channel_select_type;
typedef struct
{
    uint32_t cctrl, brk, pr, div, c2dt, c3dt, c4dt, cval;
    bool running;
} tmr_type;
typedef struct
{
    uint32_t output;
} gpio_type;
typedef struct
{
    unsigned gpio_mode, gpio_out_type, gpio_pull, gpio_drive_strength, gpio_pins;
} gpio_init_type;
typedef struct
{
    unsigned oc_mode, oc_output_state, occ_output_state, oc_polarity, occ_polarity;
    unsigned oc_idle_state, occ_idle_state;
} tmr_output_config_type;
typedef struct
{
    unsigned auto_output_enable, fcsoen_state, fcsodis_state, brk_enable, deadtime;
} tmr_brkdt_config_type;
typedef struct
{
    uint32_t ahb_freq, apb1_freq, apb2_freq;
} crm_clocks_freq_type;
typedef struct
{
    struct
    {
        unsigned apb3div;
    } misc2_bit;
} crm_type;

extern tmr_type fake_tmr1, fake_tmr6;
extern gpio_type fake_gpioa, fake_gpiob;
extern crm_type fake_crm;
extern unsigned fake_failure, fake_primask, fake_pll_n, fake_pll_m, fake_pll_p;
extern unsigned fake_timer_channels, fake_mux_count, fake_core_clock;
extern uint32_t system_core_clock;

#define TMR1 (&fake_tmr1)
#define TMR6 (&fake_tmr6)
#define GPIOA (&fake_gpioa)
#define GPIOB (&fake_gpiob)
#define CRM (&fake_crm)
#define TRUE 1U
#define FALSE 0U
#define SET 1U
#define RESET 0U
#define CRM_GPIOA_PERIPH_CLOCK 1U
#define CRM_GPIOB_PERIPH_CLOCK 2U
#define CRM_TMR1_PERIPH_CLOCK 3U
#define CRM_TMR6_PERIPH_CLOCK 4U
#define CRM_PWC_PERIPH_CLOCK 5U
#define CRM_CLOCK_SOURCE_HEXT 1U
#define CRM_CLOCK_SOURCE_PLL 2U
#define CRM_HEXT_STABLE_FLAG 1U
#define CRM_PLL_STABLE_FLAG 2U
#define CRM_PLL_SOURCE_HEXT 1U
#define CRM_PLL_FP_4 2U
#define CRM_AHB_DIV_1 0U
#define CRM_APB1_DIV_1 0U
#define CRM_APB2_DIV_1 0U
#define CRM_APB3_DIV_4 5U
#define CRM_SCLK_PLL 2U
#define FLASH_WAIT_CYCLE_5 5U
#define PWC_LDO_OUTPUT_1V3 3U
#define GPIO_PINS_0 (1U << 0)
#define GPIO_PINS_1 (1U << 1)
#define GPIO_PINS_4 (1U << 4)
#define GPIO_PINS_9 (1U << 9)
#define GPIO_PINS_10 (1U << 10)
#define GPIO_PINS_11 (1U << 11)
#define GPIO_PINS_12 (1U << 12)
#define GPIO_PINS_SOURCE0 0U
#define GPIO_PINS_SOURCE1 1U
#define GPIO_PINS_SOURCE4 4U
#define GPIO_PINS_SOURCE9 9U
#define GPIO_PINS_SOURCE10 10U
#define GPIO_PINS_SOURCE11 11U
#define GPIO_MUX_1 1U
#define GPIO_MODE_OUTPUT 1U
#define GPIO_MODE_MUX 2U
#define GPIO_OUTPUT_PUSH_PULL 0U
#define GPIO_PULL_DOWN 2U
#define GPIO_PULL_UP 1U
#define GPIO_DRIVE_STRENGTH_MODERATE 1U
#define TMR_COUNT_TWO_WAY_2 2U
#define TMR_CLOCK_DIV1 0U
#define TMR_OUTPUT_CONTROL_PWM_MODE_A 6U
#define TMR_OUTPUT_ACTIVE_HIGH 0U
#define TMR_SELECT_CHANNEL_2 2U
#define TMR_SELECT_CHANNEL_3 4U
#define TMR_SELECT_CHANNEL_4 6U
#define TMR_OVF_FLAG 1U
#define TMR_OVF_INT 1U
#define SysTick_IRQn 0U
#define TMR6_DAC_GLOBAL_IRQn 1U

/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param id 模拟外设编号。
 * @param enabled 模拟使能状态。
 */
void crm_periph_clock_enable(unsigned id, unsigned enabled);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param cycles 等待周期。
 */
void flash_psr_set(unsigned cycles);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param level 模拟电压档位。
 */
void pwc_ldo_output_voltage_set(unsigned level);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param source 模拟时钟源。
 * @param enabled 模拟使能状态。
 */
void crm_clock_source_enable(unsigned source, unsigned enabled);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param flag 模拟状态标志。
 * @return 对应替身状态或测试注入的结果。
 */
unsigned crm_flag_get(unsigned flag);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param source 模拟时钟源。
 * @param n PLL倍频。
 * @param m PLL预分频。
 * @param p PLL后分频枚举。
 */
void crm_pll_config(unsigned source, unsigned n, unsigned m, unsigned p);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param value 模拟寄存器值。
 */
void crm_ahb_div_set(unsigned value);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param value 模拟寄存器值。
 */
void crm_apb1_div_set(unsigned value);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param value 模拟寄存器值。
 */
void crm_apb2_div_set(unsigned value);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param value 模拟寄存器值。
 */
void crm_apb3_div_set(unsigned value);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param enabled 模拟使能状态。
 */
void crm_auto_step_mode_enable(unsigned enabled);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param source 模拟时钟源。
 */
void crm_sysclk_switch(unsigned source);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @return 对应替身状态或测试注入的结果。
 */
unsigned crm_sysclk_switch_status_get(void);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 */
void system_core_clock_update(void);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param clocks 输出频率快照。
 */
void crm_clocks_freq_get(crm_clocks_freq_type *clocks);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param gpio 模拟GPIO寄存器。
 * @param pins 引脚集合或配置。
 */
void gpio_bits_reset(gpio_type *gpio, unsigned pins);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param gpio 模拟GPIO寄存器。
 * @param pins 引脚集合或配置。
 */
void gpio_bits_set(gpio_type *gpio, unsigned pins);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param pins 引脚集合或配置。
 */
void gpio_default_para_init(gpio_init_type *pins);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param gpio 模拟GPIO寄存器。
 * @param pins 引脚集合或配置。
 */
void gpio_init(gpio_type *gpio, gpio_init_type *pins);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param gpio 模拟GPIO寄存器。
 * @param pin 引脚编号。
 * @param mux 复用功能编号。
 */
void gpio_pin_mux_config(gpio_type *gpio, unsigned pin, unsigned mux);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param timer 模拟定时器寄存器。
 */
void tmr_reset(tmr_type *timer);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param timer 模拟定时器寄存器。
 * @param period 自动重载计数。
 * @param divider 分频计数。
 */
void tmr_base_init(tmr_type *timer, unsigned period, unsigned divider);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param timer 模拟定时器寄存器。
 * @param direction 计数方向。
 */
void tmr_cnt_dir_set(tmr_type *timer, unsigned direction);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param timer 模拟定时器寄存器。
 * @param divider 分频计数。
 */
void tmr_clock_source_div_set(tmr_type *timer, unsigned divider);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param timer 模拟定时器寄存器。
 * @param enabled 模拟使能状态。
 */
void tmr_period_buffer_enable(tmr_type *timer, unsigned enabled);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param output 模拟输出配置。
 */
void tmr_output_default_para_init(tmr_output_config_type *output);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param timer 模拟定时器寄存器。
 * @param channel 通道枚举。
 * @param output 模拟输出配置。
 */
void tmr_output_channel_config(tmr_type *timer, unsigned channel, tmr_output_config_type *output);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param timer 模拟定时器寄存器。
 * @param channel 通道枚举。
 * @param value 模拟寄存器值。
 */
void tmr_channel_value_set(tmr_type *timer, unsigned channel, unsigned value);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param timer 模拟定时器寄存器。
 * @param channel 通道枚举。
 * @param enabled 模拟使能状态。
 */
void tmr_output_channel_buffer_enable(tmr_type *timer, unsigned channel, unsigned enabled);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param config 模拟死区配置。
 */
void tmr_brkdt_default_para_init(tmr_brkdt_config_type *config);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param timer 模拟定时器寄存器。
 * @param config 模拟死区配置。
 */
void tmr_brkdt_config(tmr_type *timer, tmr_brkdt_config_type *config);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param timer 模拟定时器寄存器。
 * @param enabled 模拟使能状态。
 */
void tmr_output_enable(tmr_type *timer, unsigned enabled);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param timer 模拟定时器寄存器。
 * @param flag 模拟状态标志。
 */
void tmr_flag_clear(tmr_type *timer, unsigned flag);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param timer 模拟定时器寄存器。
 * @param mask 中断掩码。
 * @param enabled 模拟使能状态。
 */
void tmr_interrupt_enable(tmr_type *timer, unsigned mask, unsigned enabled);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param timer 模拟定时器寄存器。
 * @param enabled 模拟使能状态。
 */
void tmr_counter_enable(tmr_type *timer, unsigned enabled);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param reload 系统时基重载计数。
 * @return 对应替身状态或测试注入的结果。
 */
unsigned SysTick_Config(unsigned reload);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param irq 模拟中断编号。
 * @param priority 中断优先级。
 */
void NVIC_SetPriority(unsigned irq, unsigned priority);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param irq 模拟中断编号。
 */
void NVIC_EnableIRQ(unsigned irq);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @return 对应替身状态或测试注入的结果。
 */
unsigned __get_PRIMASK(void);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 */
void __disable_irq(void);
/**
 * @brief 供离线测试记录或核对SDK调用，不访问硬件。
 * @param value 模拟寄存器值。
 */
void __set_PRIMASK(unsigned value);

#endif
