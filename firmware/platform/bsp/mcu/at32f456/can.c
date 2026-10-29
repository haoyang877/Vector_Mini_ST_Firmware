#include "bsp_internal.h"
#include "bsp_can.h"

#if BSP_ENABLED_CAN
#include "bsp_system.h"
#include "can_frame_codec.h"
#include "at32f45x.h"
#include "bsp_config.h"
#include "bsp_pwm.h"
#include <stddef.h>
#include <string.h>

/* 本模块独占控制器状态与发送票据；IRQ只锁存/分发，不解释协议。 */
static void irq_enable_receive(void);
static void irq_enable_error(void);

static bool s_configured;
static bool s_controller_started;
static volatile bool s_busoff_latched;

static void start_failure(void)
{
    BspPwm_Disable();
    __disable_irq();
    for (;;)
    {
        __WFI();
    }
}

bool McuCan_Init(void)
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

void McuCan_Start(void)
{
    if (!s_configured || s_controller_started)
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
    s_controller_started = true;
    irq_enable_error();
}

bool McuCan_IsAvailable(void)
{
    return s_controller_started && !s_busoff_latched && !CAN1->ctrlstat_bit.reset &&
           !CAN1->ctrlstat_bit.stby && !can_busoff_get(CAN1);
}

bool McuCan_NeedsRecovery(void)
{
    return s_controller_started && (s_busoff_latched || can_busoff_get(CAN1));
}
bool McuCan_Recover(void)
{
    /* 专用bus-off复位不改过滤、时序和前台发送票据。 */
    can_busoff_reset(CAN1);
    bool recovered = !can_busoff_get(CAN1) && !CAN1->ctrlstat_bit.reset;
    s_busoff_latched = !recovered;
    return recovered;
}
static void controller_on_error_interrupt(void)
{
    if (can_busoff_get(CAN1))
    {
        s_busoff_latched = true;
    }
    can_flag_clear(CAN1, CAN_EIF_FLAG);
}

/* CAN共享寄存器只需隔离CAN优先级；ADC和载波IRQ必须可抢占拷贝/发送。
 * RX为3、状态为2；保留调用者已有的更严格BASEPRI，不改PRIMASK。 */
#define CAN_IRQ_CEILING 2U
static uint32_t can_enter_critical(void)
{
    uint32_t previous = __get_BASEPRI();
    __set_BASEPRI_MAX(CAN_IRQ_CEILING << (8U - __NVIC_PRIO_BITS));
    __DSB();
    __ISB();
    return previous;
}
static void can_exit_critical(uint32_t previous)
{
    __set_BASEPRI(previous);
    __ISB();
}

/* 接收只执行有界回调并按硬件积压重挂；错误中断只锁存，不在此重试或解析协议。 */

static void irq_enable_receive(void)
{
    NVIC_SetPriority(CAN1_RX_IRQn, CAN_IRQ_CEILING + 1U);
    NVIC_ClearPendingIRQ(CAN1_RX_IRQn);
    can_interrupt_enable(CAN1, CAN_RIE_INT | CAN_ROIE_INT, TRUE);
    NVIC_EnableIRQ(CAN1_RX_IRQn);
}
static void irq_enable_error(void)
{
    can_interrupt_enable(CAN1, CAN_EIE_INT, TRUE);
    NVIC_SetPriority(CAN1_STAT_IRQn, CAN_IRQ_CEILING);
    NVIC_EnableIRQ(CAN1_STAT_IRQn);
}
void CAN1_STAT_IRQHandler(void)
{
    controller_on_error_interrupt();
}
void McuCan_OnReceiveInterrupt(BspCanReceiveHandler receive_handler, uint32_t events)
{
    (void)events;
    if (receive_handler == 0)
    {
        return;
    }
    can_flag_clear(CAN1, CAN_RIF_FLAG);
    receive_handler();
    if (can_rxbuf_status_get(CAN1) != CAN_RXBUF_STATUS_EMPTY)
    {
        NVIC_SetPendingIRQ(CAN1_RX_IRQn);
    }
}
void McuCan_EnableReceiveInterrupt(void)
{
    irq_enable_receive();
}

BspResult McuCan_Receive(BspCanFrame *frame)
{
    if (frame == NULL || !McuCan_IsAvailable())
    {
        return BSP_IO_ERROR;
    }
    if (can_rxbuf_status_get(CAN1) == CAN_RXBUF_STATUS_EMPTY)
    {
        return BSP_NOT_READY;
    }
    can_rxbuf_type data = {0};
    uint32_t state = can_enter_critical();
    bool received = can_rxbuf_read(CAN1, &data) == SUCCESS;
    can_exit_critical(state);
    if (!received)
    {
        return BSP_IO_ERROR;
    }
    BspCanFrame decoded = {0};
    decoded.identifier = data.id;
    decoded.extended = data.id_type == CAN_ID_EXTENDED;
    decoded.remote = data.frame_type == CAN_FRAME_REMOTE;
    decoded.fd = data.fd_format == CAN_FORMAT_FD;
    decoded.bitrate_switch = data.fd_rate_switch == CAN_BRS_ON;
    uint8_t dlc;
    if (data.kind_error != CAN_KOER_NOERR ||
        !CanFrame_DecodeLength((uint32_t)data.data_length, &decoded.length) ||
        !CanFrame_EncodeTx(&decoded, &dlc))
    {
        return BSP_FRAME_ERROR;
    }
    if (!decoded.remote)
    {
        memcpy(decoded.data, data.data, decoded.length);
    }
    *frame = decoded;
    return BSP_OK;
}
static uint8_t s_next_handle;
bool McuCan_SlotIdle(uint32_t token)
{
    (void)token;
    return !CAN1->ctrlstat_bit.tpe && !CAN1->ctrlstat_bit.tpa;
}
bool McuCan_TxCancelled(uint32_t token)
{
    (void)token;
    return can_flag_get(CAN1, CAN_AIF_FLAG) != RESET;
}
bool McuCan_TxCompleted(uint32_t token)
{
    (void)token;
    return can_flag_get(CAN1, CAN_TPIF_FLAG) != RESET;
}
void McuCan_ReadTerminal(uint32_t token, bool *matches, bool *sent)
{
    can_transmit_status_type status;
    can_transmit_status_get(CAN1, &status);
    *matches = status.final_handle == (uint8_t)token &&
               (status.final_tstat == CAN_TSTAT_LOST_ARBITRATION ||
                status.final_tstat >= CAN_TSTAT_ABORTED);
    *sent = status.final_handle == (uint8_t)token && status.final_tstat == CAN_TSTAT_TRANSMITTED;
}
void McuCan_Abort(uint32_t token)
{
    (void)token;
    can_transmit_abort(CAN1, CAN_ABORT_TRANSMIT_PTB);
}
bool McuCan_Submit(const BspCanFrame *frame, uint8_t dlc, uint32_t *token)
{
    can_txbuf_type data = {0};
    data.id = frame->identifier;
    data.id_type = frame->extended ? CAN_ID_EXTENDED : CAN_ID_STANDARD;
    data.frame_type = frame->remote ? CAN_FRAME_REMOTE : CAN_FRAME_DATA;
    data.data_length = (can_data_length_type)dlc;
    data.fd_format = frame->fd ? CAN_FORMAT_FD : CAN_FORMAT_CLASSIC;
    data.fd_rate_switch = frame->bitrate_switch ? CAN_BRS_ON : CAN_BRS_OFF;
    memcpy(data.data, frame->data, frame->length);
    uint32_t state = can_enter_critical();
    bool accepted = false;
    if (McuCan_IsAvailable())
    {
        data.handle = ++s_next_handle;
        can_flag_clear(CAN1, CAN_TPIF_FLAG | CAN_AIF_FLAG);
        accepted = can_txbuf_write(CAN1, CAN_TXBUF_PTB, &data) == SUCCESS &&
                   can_txbuf_transmit(CAN1, CAN_TRANSMIT_PTB) == SUCCESS;
        if (accepted)
        {
            *token = data.handle;
        }
    }
    can_exit_critical(state);
    return accepted;
}

bool McuCan_TakeRxOverflow(void)
{
    bool lost = can_flag_get(CAN1, CAN_ROIF_FLAG) != RESET;
    if (lost)
    {
        can_flag_clear(CAN1, CAN_ROIF_FLAG);
    }
    return lost;
}

BspCanState McuCan_GetState(void)
{
    if (s_busoff_latched || can_busoff_get(CAN1))
    {
        return BSP_CAN_BUS_OFF;
    }
    if (!McuCan_IsAvailable() || can_flag_get(CAN1, CAN_EPASS_FLAG))
    {
        return BSP_CAN_ERROR;
    }
    return can_flag_get(CAN1, CAN_EWARN_FLAG) ? BSP_CAN_WARNING : BSP_CAN_READY;
}
#endif
