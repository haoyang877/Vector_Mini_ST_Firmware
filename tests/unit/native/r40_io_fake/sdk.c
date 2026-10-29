/* ADC/CAN/DMA的确定性硬件模型，生产端口仍完整编译和执行。 */
#include "at32f45x.h"
#include <assert.h>
#include <string.h>

adc_type sim_adc_regs[2];
SimAdc sim_adc[2];
can_type sim_can;
dma_type sim_dma_regs;
dma_channel_type sim_dma_channel;
dmamux_channel_type sim_mux;
dma_init_type sim_dma_config;
dma_channel_type sim_temperature_dma_channel;
dmamux_channel_type sim_temperature_mux;
dma_init_type sim_temperature_dma_config;
bool sim_temperature_dma_enabled;
adc_common_config_type sim_adc_common;
bool sim_dma_enabled, sim_power_off = true, sim_can_tx_fail, sim_rx_ready, sim_recovery_fail;
unsigned sim_dma_flags, sim_stop_calls, sim_adc_failure, sim_can_writes, sim_bit_writes;
unsigned sim_temperature_irq_enables;
uint32_t sim_ms, sim_analog_pins, sim_dma_interrupts;
can_txbuf_type sim_tx;
can_rxbuf_type sim_rx;
can_filter_config_type sim_filter;
can_bittime_type sim_timing;
bool sim_complete_during_status;
unsigned sim_can_pclk_hz = 180000000U;
tmr_type fake_tmr1, fake_tmr6;
SimCycleCounter sim_cycle_counter;
SimCoreDebug sim_core_debug;
unsigned sim_adc_trigger_ticks;
gpio_type fake_gpioa, fake_gpiob;
crm_type fake_crm;
uint32_t system_core_clock = 180000000U;
static unsigned s_irq;
bool sim_storage_auto_samples, sim_storage_stop_stuck;
unsigned sim_storage_isr;
uint32_t sim_cleared_irqs;
uint32_t sim_nvic_pending, sim_nvic_enabled, sim_can_irq_mask;
unsigned sim_rx_remaining;
bool sim_can_reset_stuck, sim_can_release_stuck, sim_can_retlim_stuck;
bool sim_can_realim_stuck, sim_can_timing_bad;
void (*sim_fatal_hook)(void);
unsigned __get_PRIMASK(void)
{
    return s_irq;
}
uint32_t __get_IPSR(void)
{
    return sim_storage_isr;
}
void NVIC_ClearPendingIRQ(unsigned irq)
{
    sim_cleared_irqs |= 1U << irq;
}
void NVIC_SetPendingIRQ(unsigned irq)
{
    if (irq < 32U)
    {
        sim_nvic_pending |= 1U << irq;
    }
}
void (*sim_before_critical_enter)(void);
unsigned sim_critical_enters, sim_critical_exits;

uint32_t BspSystem_TimeUs(void)
{
    return sim_ms * 1000U;
}
uint32_t BspSystem_TimeMs(void)
{
    return sim_ms;
}
uint32_t BspSystem_EnterCritical(void)
{
    if (s_irq == 0U && sim_before_critical_enter != NULL)
    {
        void (*pending)(void) = sim_before_critical_enter;
        sim_before_critical_enter = NULL;
        pending();
    }
    unsigned old = s_irq;
    s_irq = 1U;
    ++sim_critical_enters;
    return old;
}
void BspSystem_ExitCritical(uint32_t old)
{
    s_irq = old;
    ++sim_critical_exits;
}
void BspPwm_Disable(void)
{
    sim_power_off = true;
    ++sim_stop_calls;
}
bool BspPwm_ArePhasesDisabled(void)
{
    return sim_power_off;
}
void __disable_irq(void)
{
    s_irq = 1U;
}
void __WFI(void)
{
    if (sim_fatal_hook != NULL)
    {
        sim_fatal_hook();
    }
    assert(!"unexpected fatal platform halt");
}
void __NOP(void)
{
}
void crm_periph_clock_enable(unsigned id, unsigned enabled)
{
    (void)id;
    assert(enabled);
}

void crm_clocks_freq_get(crm_clocks_freq_type *clocks)
{
    clocks->ahb_freq = system_core_clock;
    clocks->apb1_freq = sim_can_pclk_hz;
    clocks->apb2_freq = system_core_clock;
}
void crm_can_clock_select(unsigned instance, unsigned source)
{
    assert(instance == 0U && source == 2U);
}
void gpio_default_para_init(gpio_init_type *pins)
{
    memset(pins, 0, sizeof(*pins));
}
void gpio_init(gpio_type *gpio, gpio_init_type *pins)
{
    if (gpio == GPIOA && pins->gpio_mode == GPIO_MODE_ANALOG)
    {
        sim_analog_pins = pins->gpio_pins;
    }
}
void gpio_pin_mux_config(gpio_type *gpio, unsigned pin, unsigned mux)
{
    (void)gpio;
    (void)pin;
    assert(mux == 9U);
}
void NVIC_SetPriority(unsigned irq, unsigned priority)
{
    (void)irq;
    (void)priority;
}
void NVIC_EnableIRQ(unsigned irq)
{
    if (irq < 32U)
    {
        sim_nvic_enabled |= 1U << irq;
    }
    if (irq == DMA1_Channel3_IRQn)
    {
        ++sim_temperature_irq_enables;
    }
}
void tmr_output_default_para_init(tmr_output_config_type *v)
{
    memset(v, 0, sizeof(*v));
}
void tmr_output_channel_config(tmr_type *t, unsigned ch, tmr_output_config_type *v)
{
    assert(t == TMR1 && ch == 0U && v->oc_mode == 7U && !v->oc_output_state);
}
void tmr_channel_value_set(tmr_type *t, unsigned ch, unsigned v)
{
    assert(t == TMR1 && ch == 0U);
    sim_adc_trigger_ticks = v;
}
void tmr_primary_mode_select(tmr_type *t, unsigned source)
{
    assert((t == TMR1 && source == 4U) || (t == TMR6 && source == 2U));
}
void tmr_interrupt_enable(tmr_type *t, unsigned mask, unsigned enabled)
{
    (void)t;
    (void)mask;
    (void)enabled;
}
void tmr_flag_clear(tmr_type *t, unsigned flag)
{
    (void)t;
    (void)flag;
}
unsigned tmr_flag_get(tmr_type *t, unsigned flag)
{
    (void)t;
    (void)flag;
    return SET;
}

static unsigned index_of(adc_type *a)
{
    return a == ADC1 ? 0U : 1U;
}
void adc_reset(void)
{
    memset(sim_adc, 0, sizeof(sim_adc));
}
void adc_common_default_para_init(adc_common_config_type *v)
{
    memset(v, 0, sizeof(*v));
}
void adc_common_config(adc_common_config_type *v)
{
    sim_adc_common = *v;
}
void adc_base_default_para_init(adc_base_config_type *v)
{
    memset(v, 0, sizeof(*v));
}
void adc_base_config(adc_type *a, adc_base_config_type *v)
{
    assert(v->sequence_mode && !v->repeat_mode);
    sim_adc[index_of(a)].ordinary_length = v->ordinary_channel_length;
}
void adc_resolution_set(adc_type *a, adc_resolution_type resolution)
{
    (void)a;
    assert(resolution == ADC_RESOLUTION_12B);
}
void adc_preempt_channel_length_set(adc_type *a, uint8_t length)
{
    sim_adc[index_of(a)].length = length;
}
void adc_preempt_channel_set(adc_type *a,
                             adc_channel_select_type ch,
                             uint8_t rank,
                             adc_sampletime_select_type sample)
{
    assert((a == ADC1 && ch <= ADC_CHANNEL_3 &&
            sample == (ch == ADC_CHANNEL_3 ? ADC_SAMPLETIME_24_5 : ADC_SAMPLETIME_12_5)) ||
           (a == ADC2 && ch == ADC_CHANNEL_4 && sample == ADC_SAMPLETIME_92_5));
    sim_adc[index_of(a)].channels[rank - 1U] = ch;
    sim_adc[index_of(a)].sample_times[rank - 1U] = sample;
}
void adc_ordinary_channel_set(adc_type *a,
                              adc_channel_select_type ch,
                              uint8_t rank,
                              adc_sampletime_select_type sample)
{
    const bool hall = a == ADC2 && (ch == ADC_CHANNEL_5 || ch == ADC_CHANNEL_6);
    assert(sample == (hall ? ADC_SAMPLETIME_24_5 : ADC_SAMPLETIME_640_5));
    assert((a == ADC1 && (ch == 16U || ch == 17U)) || (a == ADC2 && ch >= 4U && ch <= 7U));
    assert(rank >= 1U && rank <= 4U);
    sim_adc[index_of(a)].ordinary_channels[rank - 1U] = ch;
    sim_adc[index_of(a)].ordinary_sample_times[rank - 1U] = sample;
}
void adc_oversample_ratio_shift_set(adc_type *a,
                                    adc_oversample_ratio_type ratio,
                                    adc_oversample_shift_type shift)
{
    sim_adc[index_of(a)].ratio = ratio;
    sim_adc[index_of(a)].shift = shift;
}
void adc_preempt_oversample_enable(adc_type *a, confirm_state enabled)
{
    sim_adc[index_of(a)].oversampled = enabled;
}
void adc_ordinary_oversample_enable(adc_type *a, confirm_state enabled)
{
    sim_adc[index_of(a)].ordinary_oversampled = enabled;
}
void adc_ordinary_oversample_trig_enable(adc_type *a, confirm_state enabled)
{
    sim_adc[index_of(a)].ordinary_oversample_per_trigger = enabled;
}
void adc_ordinary_oversample_restart_set(adc_type *a, adc_ordinary_oversample_restart_type recovery)
{
    sim_adc[index_of(a)].ordinary_oversample_recovery = recovery;
}
void adc_preempt_conversion_trigger_set(adc_type *a,
                                        adc_preempt_trig_select_type trigger,
                                        adc_preempt_trig_edge_type edge)
{
    sim_adc[index_of(a)].trigger = trigger;
    sim_adc[index_of(a)].edge = edge;
}
void adc_ordinary_conversion_trigger_set(adc_type *a,
                                         adc_ordinary_trig_select_type trigger,
                                         adc_ordinary_trig_edge_type edge)
{
    sim_adc[index_of(a)].ordinary_trigger = trigger;
    sim_adc[index_of(a)].ordinary_edge = edge;
}
void adc_convert_fail_auto_abort_enable(adc_type *a, confirm_state enabled)
{
    (void)a;
    assert(enabled);
}
void adc_occe_each_conversion_enable(adc_type *a, confirm_state enabled)
{
    assert((a == ADC1 || a == ADC2) && enabled);
}
void adc_dma_mode_enable(adc_type *a, confirm_state enabled)
{
    assert((a == ADC1 || a == ADC2) && enabled);
    sim_adc[index_of(a)].dma_enabled = enabled;
}
void adc_dma_request_repeat_enable(adc_type *a, confirm_state enabled)
{
    assert((a == ADC1 || a == ADC2) && enabled);
}
void adc_enable(adc_type *a, confirm_state enabled)
{
    sim_adc[index_of(a)].enabled = enabled;
}
flag_status adc_flag_get(adc_type *a, uint8_t flag)
{
    if (sim_storage_auto_samples && a == ADC1 && flag == ADC_PCCE_FLAG &&
        sim_adc[0].edge == ADC_PREEMPT_TRIG_EDGE_RISING)
    {
        sim_adc[0].flags |= ADC_PCCE_FLAG;
        sim_adc[1].flags |= ADC_PCCE_FLAG;
    }
    if (flag == ADC_RDY_FLAG)
    {
        return sim_adc_failure == 1U ? RESET : SET;
    }
    return (sim_adc[index_of(a)].flags & flag) != 0U;
}
void adc_conversion_stop(adc_type *a)
{
    (void)a;
}
flag_status adc_conversion_stop_status_get(adc_type *a)
{
    (void)a;
    return sim_storage_stop_stuck ? SET : RESET;
}
void adc_flag_clear(adc_type *a, uint32_t flag)
{
    sim_adc[index_of(a)].flags &= ~flag;
}
void adc_calibration_init(adc_type *a)
{
    (void)a;
}
flag_status adc_calibration_init_status_get(adc_type *a)
{
    (void)a;
    return sim_adc_failure == 2U;
}
void adc_calibration_start(adc_type *a)
{
    sim_adc[index_of(a)].calibrated = true;
}
flag_status adc_calibration_status_get(adc_type *a)
{
    (void)a;
    return sim_adc_failure == 3U;
}
void adc_interrupt_enable(adc_type *a, uint32_t mask, confirm_state enabled)
{
    if (enabled)
    {
        sim_adc[index_of(a)].interrupts |= mask;
    }
    else
    {
        sim_adc[index_of(a)].interrupts &= ~mask;
    }
}
uint16_t adc_preempt_conversion_data_get(adc_type *a, adc_preempt_channel_type rank)
{
    return sim_adc[index_of(a)].result[rank];
}
void dma_reset(dma_channel_type *c)
{
    assert(c == DMA1_CHANNEL2 || c == DMA1_CHANNEL3);
}
void dma_default_para_init(dma_init_type *v)
{
    memset(v, 0, sizeof(*v));
}
void dma_init(dma_channel_type *c, dma_init_type *v)
{
    assert(v->memory_base_addr && v->buffer_size >= 2U && v->buffer_size <= 8U &&
           v->buffer_size % 2U == 0U);
    if (c == DMA1_CHANNEL2)
    {
        assert(!sim_dma_enabled);
        sim_dma_config = *v;
    }
    else
    {
        assert(c == DMA1_CHANNEL3 && !sim_temperature_dma_enabled);
        sim_temperature_dma_config = *v;
    }
}
void dma_channel_enable(dma_channel_type *c, confirm_state enabled)
{
    if (c == DMA1_CHANNEL2)
    {
        sim_dma_enabled = enabled;
    }
    else
    {
        assert(c == DMA1_CHANNEL3);
        sim_temperature_dma_enabled = enabled;
    }
}
void dmamux_enable(dma_type *d, confirm_state enabled)
{
    assert(d == DMA1 && enabled);
}
void dmamux_init(dmamux_channel_type *m, dmamux_requst_id_sel_type req)
{
    assert((m == DMA1MUX_CHANNEL2 && req == DMAMUX_DMAREQ_ID_ADC2) ||
           (m == DMA1MUX_CHANNEL3 && req == DMAMUX_DMAREQ_ID_ADC1));
}
void dma_interrupt_enable(dma_channel_type *c, uint32_t mask, confirm_state enabled)
{
    (void)c;
    if (enabled)
    {
        sim_dma_interrupts |= mask;
    }
    else
    {
        sim_dma_interrupts &= ~mask;
    }
}
flag_status dma_flag_get(uint32_t flags)
{
    if (sim_storage_auto_samples && sim_dma_enabled &&
        sim_adc[1].ordinary_edge == ADC_ORDINARY_TRIG_EDGE_RISING)
    {
        sim_dma_flags |= DMA1_HDT2_FLAG;
    }
    return (sim_dma_flags & flags) != 0U;
}
void dma_data_number_set(dma_channel_type *channel, uint16_t number)
{
    channel->dtcnt = number;
}
void dma_flag_clear(uint32_t flags)
{
    sim_dma_flags &= flags == DMA1_GL2_FLAG   ? ~(DMA1_GL2_FLAG * 15U)
                     : flags == DMA1_GL3_FLAG ? ~(DMA1_GL3_FLAG * 15U)
                                              : ~flags;
}

void can_reset(can_type *c)
{
    memset(c, 0, sizeof(*c));
}
void can_software_reset(can_type *c, confirm_state enabled)
{
    c->ctrlstat_bit.reset = enabled ? !sim_can_reset_stuck : sim_can_release_stuck;
    /* RM软件复位表：重发/仲裁限制在RESET期间保持复位值。 */
    c->lbtcfg_bit.retlim = CAN_RE_TRANS_TIMES_UNLIMIT;
    c->lbtcfg_bit.realim = CAN_RE_ARBI_TIMES_UNLIMIT;
}
void can_bittime_default_para_init(can_bittime_type *v)
{
    memset(v, 0, sizeof(*v));
}
void can_bittime_set(can_type *c, can_bittime_type *v)
{
    assert(c->ctrlstat_bit.reset);
    sim_timing = *v;
    ++sim_bit_writes;
    c->lbtcfg_bit.presc = v->bittime_div - 1U;
    if (sim_can_timing_bad)
    {
        c->lbtcfg_bit.presc = 1U;
    }
    c->actime_bit.ac_seg_1 = v->ac_bts1_size - 2U;
    c->actime_bit.ac_seg_2 = v->ac_bts2_size - 1U;
    c->fdtime_bit.fd_seg_1 = v->fd_bts1_size - 2U;
    c->fdtime_bit.fd_seg_2 = v->fd_bts2_size - 1U;
    c->lbtcfg_bit.fd_sspoff = v->fd_ssp_offset;
}
void can_fd_iso_mode_enable(can_type *c, confirm_state enabled)
{
    c->ctrlstat_bit.fd_iso = enabled;
}
void can_mode_set(can_type *c, can_mode_type mode)
{
    (void)c;
    assert(mode == CAN_MODE_COMMUNICATE);
}
void can_retransmission_limit_set(can_type *c, can_retransmission_times_type limit)
{
    if (!c->ctrlstat_bit.reset)
    {
        c->lbtcfg_bit.retlim = sim_can_retlim_stuck ? CAN_RE_TRANS_TIMES_UNLIMIT : limit;
    }
}
void can_rearbitration_limit_set(can_type *c, can_rearbitration_times_type limit)
{
    if (!c->ctrlstat_bit.reset)
    {
        c->lbtcfg_bit.realim = sim_can_realim_stuck ? CAN_RE_ARBI_TIMES_1 : limit;
    }
}
void can_receive_all_enable(can_type *c, confirm_state enabled)
{
    (void)c;
    assert(!enabled);
}
void can_rxbuf_overflow_mode_set(can_type *c, can_rxbuf_overflow_mode_type mode)
{
    (void)c;
    assert(mode == CAN_RXBUF_OVERFLOW_BE_LOSE);
}
void can_error_warning_set(can_type *c, uint8_t value)
{
    (void)c;
    assert(value == 11U);
}
void can_filter_default_para_init(can_filter_config_type *v)
{
    memset(v, 0, sizeof(*v));
    v->mask_para.id = 0xFFFFFFFFU;
    v->mask_para.id_type = TRUE;
    v->mask_para.frame_type = TRUE;
    v->mask_para.data_length = 0xFU;
    v->mask_para.recv_frame = TRUE;
    v->mask_para.fd_format = TRUE;
    v->mask_para.fd_rate_switch = TRUE;
    v->mask_para.fd_error_state = TRUE;
}
void can_filter_enable(can_type *c, can_filter_type filter, confirm_state enabled)
{
    (void)c;
    (void)filter;
    (void)enabled;
}
void can_filter_set(can_type *c, can_filter_type filter, can_filter_config_type *v)
{
    (void)c;
    assert(filter == CAN_FILTER_NUM_0);
    sim_filter = *v;
}
void can_interrupt_enable(can_type *c, uint32_t mask, confirm_state enabled)
{
    (void)c;
    if (enabled)
    {
        sim_can_irq_mask |= mask;
    }
    else
    {
        sim_can_irq_mask &= ~mask;
    }
}
void can_flag_clear(can_type *c, uint32_t mask)
{
    c->sts &= ~mask;
}
flag_status can_flag_get(can_type *c, uint32_t mask)
{
    return (c->sts & mask) != 0U;
}
flag_status can_busoff_get(can_type *c)
{
    return c->ctrlstat_bit.busoff;
}
void can_busoff_reset(can_type *c)
{
    if (!sim_recovery_fail)
    {
        c->ctrlstat_bit.busoff = false;
    }
}
void can_transmit_status_get(can_type *c, can_transmit_status_type *v)
{
    if (sim_complete_during_status)
    {
        sim_complete_during_status = false;
        c->ctrlstat_bit.tpe = 0U;
        c->tstat_bit.handle_2 = sim_tx.handle;
        c->tstat_bit.tstat_2 = CAN_TSTAT_TRANSMITTED;
        c->sts |= CAN_TPIF_FLAG;
    }
    v->current_handle = c->tstat_bit.handle_1;
    v->current_tstat = c->tstat_bit.tstat_1;
    v->final_handle = c->tstat_bit.handle_2;
    v->final_tstat = c->tstat_bit.tstat_2;
}
error_status can_txbuf_write(can_type *c, can_txbuf_select_type buffer, can_txbuf_type *v)
{
    sim_can_submit_primask = __get_PRIMASK();
    sim_can_submit_basepri = __get_BASEPRI();
    assert(buffer == CAN_TXBUF_PTB && !c->ctrlstat_bit.tpe);
    if (sim_can_tx_fail)
    {
        return ERROR;
    }
    sim_tx = *v;
    ++sim_can_writes;
    return SUCCESS;
}
error_status can_txbuf_transmit(can_type *c, can_txbuf_transmit_type type)
{
    assert(type == CAN_TRANSMIT_PTB);
    c->ctrlstat_bit.tpe = 1U;
    return SUCCESS;
}
void can_transmit_abort(can_type *c, can_abort_transmit_type type)
{
    assert(type == CAN_ABORT_TRANSMIT_PTB);
    c->ctrlstat_bit.tpa = 1U;
}
error_status can_rxbuf_read(can_type *c, can_rxbuf_type *v)
{
    (void)c;
    if (!sim_rx_ready && sim_rx_remaining == 0U)
    {
        return ERROR;
    }
    *v = sim_rx;
    sim_rx_ready = false;
    if (sim_rx_remaining != 0U)
    {
        --sim_rx_remaining;
    }
    return SUCCESS;
}
can_rxbuf_status_type can_rxbuf_status_get(can_type *c)
{
    (void)c;
    return sim_rx_ready || sim_rx_remaining != 0U ? CAN_RXBUF_STATUS_FULL : CAN_RXBUF_STATUS_EMPTY;
}

static uint32_t s_basepri;
uint32_t sim_can_submit_primask, sim_can_submit_basepri;
uint32_t __get_BASEPRI(void)
{
    return s_basepri;
}
void __set_BASEPRI(uint32_t value)
{
    s_basepri = value;
}
void __set_BASEPRI_MAX(uint32_t value)
{
    if (value != 0U && (s_basepri == 0U || value < s_basepri))
        s_basepri = value;
}
