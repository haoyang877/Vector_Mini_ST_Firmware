/**
 * @file can.c
 * @brief AT32 CAN初始化、寄存器收发、启动及接收中断。
 */
#include "../../bsp_internal.h"
#include "bsp_can.h"
#include "mcu_resources.h"
#include <string.h>
#include "at32f45x.h"

#if BSP_ENABLED_CAN
static const uint8_t s_dlc_lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

void McuCan_Init(void)
{
    crm_periph_clock_enable(MCU_CAN_CLOCK, TRUE);
    crm_can_clock_select(MCU_CAN_CLOCK_SELECT, CRM_CAN_CLOCK_SOURCE_PCLK);
    can_reset(MCU_CAN);
    can_software_reset(MCU_CAN, TRUE);
    crm_periph_clock_enable(MCU_CAN_GPIO_CLOCK, TRUE);
    gpio_init_type pins;
    gpio_default_para_init(&pins);
    pins.gpio_mode = GPIO_MODE_MUX;
    pins.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    pins.gpio_pull = GPIO_PULL_NONE;
    pins.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    pins.gpio_pins = MCU_CAN_RX_PIN | MCU_CAN_TX_PIN;
    gpio_pin_mux_config(MCU_CAN_GPIO_PORT, MCU_CAN_RX_SOURCE, MCU_CAN_GPIO_AF);
    gpio_pin_mux_config(MCU_CAN_GPIO_PORT, MCU_CAN_TX_SOURCE, MCU_CAN_GPIO_AF);
    gpio_init(MCU_CAN_GPIO_PORT, &pins);
    can_bittime_type timing;
    can_bittime_default_para_init(&timing);
    timing.bittime_div = 1U;
    timing.ac_bts1_size = AT32_CAN_NOMINAL_SEG1;
    timing.ac_bts2_size = AT32_CAN_NOMINAL_SEG2;
    timing.ac_rsaw_size = AT32_CAN_NOMINAL_SEG2;
    timing.fd_bts1_size = AT32_CAN_DATA_SEG1;
    timing.fd_bts2_size = AT32_CAN_DATA_SEG2;
    timing.fd_rsaw_size = AT32_CAN_DATA_SEG2;
    timing.fd_ssp_offset = AT32_CAN_DATA_SEG1 + 1U;
    can_bittime_set(MCU_CAN, &timing);
    can_fd_iso_mode_enable(MCU_CAN, TRUE);
    can_mode_set(MCU_CAN, CAN_MODE_COMMUNICATE);
    can_receive_all_enable(MCU_CAN, FALSE);
    can_rxbuf_overflow_mode_set(MCU_CAN, CAN_RXBUF_OVERFLOW_BE_LOSE);
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
    /* AT32掩码0比较、1忽略：只接收扩展数据帧，协议格式由上层处理。 */
    for (uint32_t i = 0U; i < 16U; ++i)
    {
        can_filter_enable(MCU_CAN, (can_filter_type)i, FALSE);
    }
    can_filter_set(MCU_CAN, CAN_FILTER_NUM_0, &filter);
    can_filter_enable(MCU_CAN, CAN_FILTER_NUM_0, TRUE);
    can_interrupt_enable(MCU_CAN, CAN_ALL_INT, FALSE);
    can_flag_clear(MCU_CAN, CAN_TPIF_FLAG | CAN_AIF_FLAG | CAN_EIF_FLAG | CAN_EPIF_FLAG);
    NVIC_SetPriority(MCU_CAN_IRQ, MCU_CAN_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(MCU_CAN_IRQ);
    NVIC_EnableIRQ(MCU_CAN_IRQ);
}

void BspCan_Start(void)
{
    MCU_CAN->ctrlstat_bit.reset = FALSE;
    /* 同寄存器跨时钟域，一次写入两项，避免两次位域写互相覆盖。 */
    MCU_CAN->lbtcfg = (MCU_CAN->lbtcfg & ~0x77000000U) | ((uint32_t)CAN_RE_TRANS_TIMES_1 << 28U) |
                      ((uint32_t)CAN_RE_ARBI_TIMES_UNLIMIT << 24U);
#if defined(BSP_APP_STATE_MACHINE) && BSP_APP_STATE_MACHINE
    MCU_CAN->inten |= CAN_RIE_INT;
#endif
}

bool BspCan_TrySend(const BspCanFrame *frame)
{
    if (BspCan_IsTransmitBusy())
    {
        return false;
    }
    uint8_t dlc = 0U;
    while (s_dlc_lengths[dlc] < frame->length)
    {
        ++dlc;
    }
    /* 命令所在字节单独写，不能重新提交旧请求位。 */
    volatile uint8_t *control = (volatile uint8_t *)&MCU_CAN->ctrlstat;
    control[1] &= 0x60U;
    MCU_CAN->tbid = frame->extended ? frame->identifier : frame->identifier << 18U;
    MCU_CAN->tbfmt = 0U;
    MCU_CAN->tbfmt_bit.ide = frame->extended;
    MCU_CAN->tbfmt_bit.rmf = frame->remote;
    MCU_CAN->tbfmt_bit.dlc = dlc;
    MCU_CAN->tbfmt_bit.fdf = frame->fd;
    MCU_CAN->tbfmt_bit.brs = frame->bitrate_switch;
    volatile uint32_t *data = MCU_CAN->tbdat;
    for (uint8_t offset = 0U; offset < frame->length; offset += 4U)
    {
        uint32_t word = 0U;
        uint8_t count = frame->length - offset;
        if (count > 4U)
        {
            count = 4U;
        }
        memcpy(&word, frame->data + offset, count);
        data[offset / 4U] = word;
    }
    MCU_CAN->tbtyp = 0U;
    MCU_CAN->reserved8 = 0U;
    MCU_CAN->sts = CAN_TPIF_FLAG | CAN_AIF_FLAG;
    control[1] = (control[1] & 0xE0U) | CAN_TRANSMIT_PTB;
    return true;
}

bool BspCan_Receive(BspCanFrame *frame)
{
    if (MCU_CAN->ctrlstat_bit.rstat == CAN_RXBUF_STATUS_EMPTY)
    {
        return false;
    }
    frame->extended = MCU_CAN->rbfmt_bit.ide;
    frame->identifier =
        frame->extended ? MCU_CAN->rbid & 0x1FFFFFFFU : (MCU_CAN->rbid >> 18U) & 0x7FFU;
    frame->remote = MCU_CAN->rbfmt_bit.rmf;
    frame->fd = MCU_CAN->rbfmt_bit.fdf;
    frame->bitrate_switch = MCU_CAN->rbfmt_bit.brs;
    uint8_t dlc = MCU_CAN->rbfmt_bit.dlc;
    volatile uint32_t *data = MCU_CAN->rbdat;
    frame->length = frame->fd ? s_dlc_lengths[dlc] : (dlc > 8U ? 8U : dlc);
    for (uint8_t offset = 0U; !frame->remote && offset < frame->length; offset += 4U)
    {
        uint32_t word = data[offset / 4U];
        uint8_t count = frame->length - offset;
        if (count > 4U)
        {
            count = 4U;
        }
        memcpy(frame->data + offset, &word, count);
    }
    volatile uint8_t *control = (volatile uint8_t *)&MCU_CAN->ctrlstat;
    control[3] |= 0x10U;
    return true;
}

bool BspCan_IsTransmitBusy(void)
{
    return MCU_CAN->ctrlstat_bit.tpe || MCU_CAN->ctrlstat_bit.tpa;
}

bool BspCan_IsTransmitComplete(void)
{
    return !MCU_CAN->ctrlstat_bit.tpe && !MCU_CAN->ctrlstat_bit.tpa &&
           (MCU_CAN->sts & CAN_TPIF_FLAG) != 0U &&
           MCU_CAN->tstat_bit.tstat_2 == CAN_TSTAT_TRANSMITTED;
}

#if defined(BSP_APP_STATE_MACHINE) && BSP_APP_STATE_MACHINE
void MCU_CAN_IRQ_HANDLER(void)
{
    MCU_CAN->sts = CAN_RIF_FLAG;
    BspApp_OnCanReceive();
    if (MCU_CAN->ctrlstat_bit.rstat != CAN_RXBUF_STATUS_EMPTY)
    {
        NVIC_SetPendingIRQ(MCU_CAN_IRQ);
    }
}
#endif
#endif
