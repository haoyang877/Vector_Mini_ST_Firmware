#ifndef R40_IO_FAKE_H
#define R40_IO_FAKE_H

#include "../r40_fake/at32f45x.h"
#include <stddef.h>

/* 使用真实SDK类型和函数声明，模拟外设副作用，不缩写ADC/CAN接口签名。 */
#define __IO volatile
#define AT32F456xx 1
typedef enum
{
    ERROR = 0,
    SUCCESS = 1
} error_status;
#include "at32f45x_adc.h"
#include "at32f45x_dma.h"
#include "at32f45x_can.h"

#undef ADC1
#undef ADC2
#undef CAN1
#undef DMA1
#undef DMA1_CHANNEL2
#undef DMA1MUX_CHANNEL2
#undef DMA1_CHANNEL3
#undef DMA1MUX_CHANNEL3
#define ADC1 (&sim_adc_regs[0])
#define ADC2 (&sim_adc_regs[1])
#define CAN1 (&sim_can)
#define DMA1 (&sim_dma_regs)
#define DMA1_CHANNEL2 (&sim_dma_channel)
#define DMA1MUX_CHANNEL2 (&sim_mux)
#define DMA1_CHANNEL3 (&sim_temperature_dma_channel)
#define DMA1MUX_CHANNEL3 (&sim_temperature_mux)
#define GPIO_PINS_2 (1U << 2)
#define GPIO_PINS_3 (1U << 3)
#define GPIO_PINS_5 (1U << 5)
#define GPIO_PINS_6 (1U << 6)
#define __DMB() ((void)0)
typedef struct
{
    uint32_t CTRL, CYCCNT;
} SimCycleCounter;
typedef struct
{
    uint32_t DEMCR;
} SimCoreDebug;
extern SimCycleCounter sim_cycle_counter;
extern SimCoreDebug sim_core_debug;
#define DWT (&sim_cycle_counter)
#define CoreDebug (&sim_core_debug)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define CoreDebug_DEMCR_TRCENA_Msk (1U << 24)
#define GPIO_PINS_7 (1U << 7)
#define GPIO_PINS_8 (1U << 8)
#define GPIO_PINS_SOURCE8 8U
#define GPIO_MUX_9 9U
#define GPIO_PULL_NONE 0U
#define GPIO_MODE_ANALOG 3U
#define CRM_ADC1_PERIPH_CLOCK 6U
#define CRM_ADC2_PERIPH_CLOCK 7U
#define CRM_DMA1_PERIPH_CLOCK 8U
#define CRM_CAN1_PERIPH_CLOCK 9U
#define CRM_CAN1 0U
#define CRM_CAN_CLOCK_SOURCE_PCLK 2U
#define TMR_SELECT_CHANNEL_1 0U
#define TMR_OUTPUT_CONTROL_PWM_MODE_B 7U
#define TMR_PRIMARY_SEL_C1ORAW 4U
#define TMR_PRIMARY_SEL_OVERFLOW 2U
#define TMR_C1_FLAG 2U
#define TMR_C1_INT 2U
#define ADC1_2_IRQn 18U
#define TMR1_CH_IRQn 27U
#define DMA1_Channel2_IRQn 12U
#define DMA1_Channel3_IRQn 13U
#define CAN1_STAT_IRQn 21U

typedef struct
{
    bool enabled, calibrated, oversampled, dma_enabled;
    unsigned flags, length, channels[4], ordinary_channels[4], ordinary_length;
    unsigned ratio, shift, trigger, edge, ordinary_trigger, ordinary_edge, interrupts;
    uint16_t result[4];
} SimAdc;

extern adc_type sim_adc_regs[2];
extern SimAdc sim_adc[2];
extern can_type sim_can;
extern dma_type sim_dma_regs;
extern dma_channel_type sim_dma_channel;
extern dmamux_channel_type sim_mux;
extern dma_init_type sim_dma_config;
extern dma_channel_type sim_temperature_dma_channel;
extern dmamux_channel_type sim_temperature_mux;
extern dma_init_type sim_temperature_dma_config;
extern bool sim_temperature_dma_enabled;
extern adc_common_config_type sim_adc_common;
extern bool sim_dma_enabled, sim_power_off, sim_can_tx_fail, sim_rx_ready, sim_recovery_fail;
extern unsigned sim_dma_flags, sim_stop_calls, sim_adc_failure, sim_can_writes, sim_bit_writes;
extern unsigned sim_temperature_irq_enables;
extern uint32_t sim_ms, sim_analog_pins, sim_dma_interrupts;
extern can_txbuf_type sim_tx;
extern can_rxbuf_type sim_rx;
extern can_filter_config_type sim_filter;
extern can_bittime_type sim_timing;
extern bool sim_complete_during_status;
extern unsigned sim_can_pclk_hz;

/** @brief 记录离线CAN时钟选择。
 * @param instance 模拟CAN实例。
 * @param source 模拟时钟来源。
 */
void crm_can_clock_select(unsigned instance, unsigned source);
/** @brief 记录离线定时器触发源。
 * @param timer 模拟定时器。
 * @param source 模拟触发源。
 */
void tmr_primary_mode_select(tmr_type *timer, unsigned source);
/** @brief 读取离线定时器事件。
 * @param timer 模拟定时器。
 * @param flag 模拟标志。
 * @return 事件是否有效。
 */
unsigned tmr_flag_get(tmr_type *timer, unsigned flag);
/** @brief 捕获测试中不应进入的致命等待，不访问硬件。 */
void __WFI(void);
/** @brief 离线记录等待周期，不访问硬件。 */
void __NOP(void);

#endif
