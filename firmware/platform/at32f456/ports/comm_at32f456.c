#include "comm_hw.h"

#include <stddef.h>
#include <string.h>

#include "at32f45x.h"
#include "critical_hw.h"
#include "power_stage_hw.h"
#include "r40_can.h"
#include "r40_clock.h"
#include "time_hw.h"

/* 唯一PTB发送槽由前台所有，64位票据绝不缩减为硬件8位handle。
 * 直到物理终态和撤销完成均确认之前不复用；不以队列空表示发送成功。 */
#define R40_TX_TIMEOUT_MS 100U
static const uint8_t s_dlc_lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
static bool s_configured;
static bool s_started;
static volatile bool s_busoff_latched;
static uint8_t s_next_handle;
static struct
{
    uint64_t ticket;
    uint32_t started_ms;
    uint8_t handle;
    bool active;
    bool terminal;
    bool success;
} s_tx;

static bool controller_available(void)
{
    return s_started && !s_busoff_latched && !CAN1->ctrlstat_bit.reset &&
           !CAN1->ctrlstat_bit.stby && !can_busoff_get(CAN1);
}

static bool slot_idle(void)
{
    return !CAN1->ctrlstat_bit.tpe && !CAN1->ctrlstat_bit.tpa;
}

static void refresh_tx(void)
{
    if (!s_tx.active)
    {
        return;
    }
    /* 票据消费前仍受超时/bus-off约束，不能由一次后台预读锁死成功结论。 */
    if (s_tx.ticket != 0U && (!controller_available() ||
                              (uint32_t)(time_hw_now_ms() - s_tx.started_ms) >= R40_TX_TIMEOUT_MS))
    {
        s_tx.terminal = true;
        s_tx.success = false;
        if (!slot_idle())
        {
            can_transmit_abort(CAN1, CAN_ABORT_TRANSMIT_PTB);
        }
    }
    if (!s_tx.terminal)
    {
        bool timeout = (uint32_t)(time_hw_now_ms() - s_tx.started_ms) >= R40_TX_TIMEOUT_MS;
        bool unavailable = !controller_available();
        bool cancelled = can_flag_get(CAN1, CAN_AIF_FLAG) != RESET;
        bool completed = can_flag_get(CAN1, CAN_TPIF_FLAG) != RESET;
        can_transmit_status_type status;
        can_transmit_status_get(CAN1, &status);
        /* 成功标志可能在本次取样中途到达，留到下一次轮询，不误报失败。 */
        bool matching_terminal = slot_idle() && status.final_handle == s_tx.handle &&
                                 (status.final_tstat == CAN_TSTAT_LOST_ARBITRATION ||
                                  status.final_tstat >= CAN_TSTAT_ABORTED);
        if (unavailable || timeout || cancelled || (completed && slot_idle()) || matching_terminal)
        {
            s_tx.terminal = true;
            s_tx.success = !unavailable && !timeout && !cancelled && completed && slot_idle() &&
                           status.final_handle == s_tx.handle &&
                           status.final_tstat == CAN_TSTAT_TRANSMITTED;
            if (!s_tx.success && !slot_idle())
            {
                can_transmit_abort(CAN1, CAN_ABORT_TRANSMIT_PTB);
            }
        }
    }
    if (s_tx.terminal && s_tx.ticket == 0U && slot_idle())
    {
        s_tx.active = false;
    }
}

static void start_failure(void)
{
    power_stage_hw_stop();
    __disable_irq();
    for (;;)
    {
        __WFI();
    }
}

bool R40_CanInit(void)
{
    crm_clocks_freq_type clocks;
    crm_clocks_freq_get(&clocks);
    if (s_configured || system_core_clock != R40_CORE_HZ || clocks.apb1_freq != R40_CAN_HZ)
    {
        return false;
    }
    crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK, TRUE);
    crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_PCLK);
    can_reset(CAN1);
    can_software_reset(CAN1, TRUE);
    if (!CAN1->ctrlstat_bit.reset)
    {
        return false;
    }
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_mode = GPIO_MODE_MUX;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_NONE;
    pins.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    pins.gpio_pins = GPIO_PINS_8 | GPIO_PINS_9;
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE8, GPIO_MUX_9);
    gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE9, GPIO_MUX_9);
    gpio_init(GPIOB, &pins);
    can_bittime_type timing;
    can_bittime_default_para_init(&timing);
    timing.bittime_div = 1U;
    timing.ac_bts1_size = R40_CAN_NOMINAL_SEG1;
    timing.ac_bts2_size = R40_CAN_NOMINAL_SEG2;
    timing.ac_rsaw_size = R40_CAN_NOMINAL_SEG2;
    timing.fd_bts1_size = R40_CAN_DATA_SEG1;
    timing.fd_bts2_size = R40_CAN_DATA_SEG2;
    timing.fd_rsaw_size = R40_CAN_DATA_SEG2;
    timing.fd_ssp_offset = R40_CAN_DATA_SEG1 + 1U;
    can_bittime_set(CAN1, &timing);
    can_fd_iso_mode_enable(CAN1, TRUE);
    can_mode_set(CAN1, CAN_MODE_COMMUNICATE);
    can_receive_all_enable(CAN1, FALSE);
    can_rxbuf_overflow_mode_set(CAN1, CAN_RXBUF_OVERFLOW_BE_LOSE);
    can_error_warning_set(CAN1, 11U);
    can_filter_config_type filter;
    can_filter_default_para_init(&filter);
    filter.code_para.id_type = CAN_ID_EXTENDED;
    filter.mask_para.id_type = FALSE;
    filter.code_para.frame_type = CAN_FRAME_DATA;
    filter.mask_para.frame_type = FALSE;
    filter.mask_para.id = 0x1FFFFFFFU;
    filter.mask_para.data_length = 0xFU;
    filter.mask_para.fd_format = TRUE;
    filter.mask_para.fd_rate_switch = TRUE;
    filter.mask_para.fd_error_state = TRUE;
    /* AT32掩码0比较、1忽略：只筛选扩展数据帧，FD/BRS由接收边界再校验。 */
    for (uint32_t i = 0U; i < 16U; ++i)
    {
        can_filter_enable(CAN1, (can_filter_type)i, FALSE);
    }
    can_filter_set(CAN1, CAN_FILTER_NUM_0, &filter);
    can_filter_enable(CAN1, CAN_FILTER_NUM_0, TRUE);
    can_interrupt_enable(CAN1, CAN_ALL_INT, FALSE);
    can_flag_clear(CAN1, CAN_TPIF_FLAG | CAN_AIF_FLAG | CAN_EIF_FLAG | CAN_EPIF_FLAG);
    s_configured =
        CAN1->lbtcfg_bit.presc == 0U && CAN1->actime_bit.ac_seg_1 == R40_CAN_NOMINAL_SEG1 - 2U &&
        CAN1->actime_bit.ac_seg_2 == R40_CAN_NOMINAL_SEG2 - 1U &&
        CAN1->fdtime_bit.fd_seg_1 == R40_CAN_DATA_SEG1 - 2U &&
        CAN1->fdtime_bit.fd_seg_2 == R40_CAN_DATA_SEG2 - 1U &&
        CAN1->lbtcfg_bit.fd_sspoff == R40_CAN_DATA_SEG1 + 1U && CAN1->ctrlstat_bit.fd_iso;
    return s_configured;
}

void comm_hw_can_start_fd(void)
{
    if (!s_configured || s_started)
    {
        start_failure();
        return;
    }
    can_software_reset(CAN1, FALSE);
    if (CAN1->ctrlstat_bit.reset)
    {
        start_failure();
        return;
    }
    /* RETLIM/REALIM被软件复位清回7，必须在退出RESET后设置；同寄存器跨域写入先读回。 */
    can_retransmission_limit_set(CAN1, CAN_RE_TRANS_TIMES_1);
    uint32_t remaining = 4096U;
    while (CAN1->lbtcfg_bit.retlim != CAN_RE_TRANS_TIMES_1 && remaining > 0U)
    {
        --remaining;
    }
    if (CAN1->lbtcfg_bit.retlim != CAN_RE_TRANS_TIMES_1)
    {
        start_failure();
        return;
    }
    can_rearbitration_limit_set(CAN1, CAN_RE_ARBI_TIMES_UNLIMIT);
    remaining = 4096U;
    while (CAN1->lbtcfg_bit.realim != CAN_RE_ARBI_TIMES_UNLIMIT && remaining > 0U)
    {
        --remaining;
    }
    if (CAN1->lbtcfg_bit.realim != CAN_RE_ARBI_TIMES_UNLIMIT)
    {
        start_failure();
        return;
    }
    s_started = true;
    can_interrupt_enable(CAN1, CAN_EIE_INT, TRUE);
    NVIC_SetPriority(CAN1_STAT_IRQn, 2U);
    NVIC_EnableIRQ(CAN1_STAT_IRQn);
}

static bool try_send(const CommHwCanFrame *frame, uint64_t ticket)
{
    if (frame == NULL || !frame->extended || !frame->fd || !frame->bitrate_switch ||
        frame->remote || frame->identifier > 0x1FFFFFFFU)
    {
        return false;
    }
    uint32_t dlc = 0U;
    while (dlc < 16U && s_dlc_lengths[dlc] != frame->length)
    {
        ++dlc;
    }
    if (dlc == 16U)
    {
        return false;
    }
    can_txbuf_type data = {0};
    data.id = frame->identifier;
    data.id_type = CAN_ID_EXTENDED;
    data.frame_type = CAN_FRAME_DATA;
    data.data_length = (can_data_length_type)dlc;
    data.fd_format = CAN_FORMAT_FD;
    data.fd_rate_switch = CAN_BRS_ON;
    memcpy(data.data, frame->data, frame->length);
    uint32_t state = critical_hw_enter();
    refresh_tx();
    if (!controller_available() || s_tx.active || !slot_idle())
    {
        critical_hw_exit(state);
        return false;
    }
    data.handle = ++s_next_handle;
    can_flag_clear(CAN1, CAN_TPIF_FLAG | CAN_AIF_FLAG);
    if (can_txbuf_write(CAN1, CAN_TXBUF_PTB, &data) != SUCCESS ||
        can_txbuf_transmit(CAN1, CAN_TRANSMIT_PTB) != SUCCESS)
    {
        critical_hw_exit(state);
        return false;
    }
    s_tx.ticket = ticket;
    s_tx.handle = data.handle;
    s_tx.started_ms = time_hw_now_ms();
    s_tx.active = true;
    s_tx.terminal = false;
    s_tx.success = false;
    critical_hw_exit(state);
    return true;
}

bool comm_hw_can_try_send_frame(const CommHwCanFrame *frame)
{
    return try_send(frame, 0U);
}

bool comm_hw_can_try_send_tracked_frame(const CommHwCanFrame *frame, uint64_t ticket)
{
    return ticket != 0U && try_send(frame, ticket);
}

bool comm_hw_can_take_tx_completion(CommHwCanTxCompletion *completion)
{
    if (completion == NULL)
    {
        return false;
    }
    uint32_t state = critical_hw_enter();
    refresh_tx();
    if (!s_tx.active || !s_tx.terminal || s_tx.ticket == 0U)
    {
        critical_hw_exit(state);
        return false;
    }
    completion->ticket = s_tx.ticket;
    completion->success = s_tx.success;
    s_tx.ticket = 0U;
    refresh_tx();
    critical_hw_exit(state);
    return true;
}

bool comm_hw_can_receive(CommHwCanFrame *frame)
{
    if (frame == NULL || !controller_available())
    {
        return false;
    }
    can_rxbuf_type data = {0};
    uint32_t state = critical_hw_enter();
    bool received = can_rxbuf_read(CAN1, &data) == SUCCESS;
    critical_hw_exit(state);
    if (!received || data.id_type != CAN_ID_EXTENDED || data.frame_type != CAN_FRAME_DATA ||
        data.fd_format != CAN_FORMAT_FD || data.fd_rate_switch != CAN_BRS_ON ||
        (uint32_t)data.data_length >= 16U || data.id > 0x1FFFFFFFU ||
        data.kind_error != CAN_KOER_NOERR)
    {
        return false;
    }
    CommHwCanFrame decoded = {0};
    decoded.identifier = data.id;
    decoded.length = s_dlc_lengths[data.data_length];
    decoded.extended = true;
    decoded.fd = true;
    decoded.bitrate_switch = true;
    memcpy(decoded.data, data.data, decoded.length);
    *frame = decoded;
    return true;
}

bool comm_hw_can_is_healthy(void)
{
    uint32_t state = critical_hw_enter();
    bool healthy = controller_available() && !can_flag_get(CAN1, CAN_EPASS_FLAG | CAN_EWARN_FLAG);
    critical_hw_exit(state);
    return healthy;
}

bool comm_hw_can_service_bus_off(void)
{
    uint32_t state = critical_hw_enter();
    if (!s_started || (!s_busoff_latched && !can_busoff_get(CAN1)))
    {
        critical_hw_exit(state);
        return false;
    }
    if (s_tx.active)
    {
        s_tx.terminal = true;
        s_tx.success = false;
        if (!slot_idle())
        {
            can_transmit_abort(CAN1, CAN_ABORT_TRANSMIT_PTB);
        }
    }
    /* 控制器复位会改变队列状态；使用专用bus-off复位，不改过滤和位时序。 */
    can_busoff_reset(CAN1);
    bool recovered = !can_busoff_get(CAN1) && !CAN1->ctrlstat_bit.reset;
    s_busoff_latched = !recovered;
    critical_hw_exit(state);
    return recovered;
}

void CAN1_STAT_IRQHandler(void)
{
    if (can_busoff_get(CAN1))
    {
        s_busoff_latched = true;
    }
    can_flag_clear(CAN1, CAN_EIF_FLAG);
}
