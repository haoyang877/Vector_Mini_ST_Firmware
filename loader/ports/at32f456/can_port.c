/* AT32 CAN1轮询端口；只使用一个PTB槽，完整票据绑定一次真实发送终态。 */
#include "can_port.h"
#include "at32f456_can_timing.h"
#include "platform.h"
#include "board.h"
#include "at32f45x.h"
#include <string.h>

static const uint8_t s_lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
static uint8_t s_node;
static bool s_initialized, s_running, s_occupied, s_failed;
static uint64_t s_ticket;
static uint32_t s_started_ms;

bool LoaderCan_Init(uint8_t node, uint8_t profile)
{
    if (s_initialized || node == UINT8_MAX || profile >= CAN_BITRATE_PROFILE_COUNT)
    {
        return false;
    }
    s_initialized = true;
    s_node = node;
    LoaderBoard_CanInit();
    can_reset(CAN1);
    can_software_reset(CAN1, TRUE);
    At32CanTiming bits = At32Can_BitTiming(profile);
    can_bittime_type timing;
    can_bittime_default_para_init(&timing);
    timing.bittime_div = bits.divider;
    timing.ac_bts1_size = bits.nominal_seg1;
    timing.ac_bts2_size = timing.ac_rsaw_size = bits.nominal_seg2;
    timing.fd_bts1_size = bits.data_seg1;
    timing.fd_bts2_size = timing.fd_rsaw_size = bits.data_seg2;
    timing.fd_ssp_offset = bits.ssp_offset;
    can_bittime_set(CAN1, &timing);
    can_fd_iso_mode_enable(CAN1, TRUE);
    can_mode_set(CAN1, CAN_MODE_COMMUNICATE);
    can_receive_all_enable(CAN1, FALSE);
    can_rxbuf_overflow_mode_set(CAN1, CAN_RXBUF_OVERFLOW_BE_LOSE);
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
    for (uint32_t i = 0; i < 16U; ++i)
    {
        can_filter_enable(CAN1, (can_filter_type)i, FALSE);
    }
    can_filter_set(CAN1, CAN_FILTER_NUM_0, &filter);
    can_filter_enable(CAN1, CAN_FILTER_NUM_0, TRUE);
    can_interrupt_enable(CAN1, CAN_ALL_INT, FALSE);
    can_flag_clear(CAN1, CAN_TPIF_FLAG | CAN_AIF_FLAG | CAN_ROIF_FLAG);
    CAN1->lbtcfg = (CAN1->lbtcfg & ~0x77000000U) | ((uint32_t)CAN_RE_TRANS_TIMES_1 << 28U) |
                   ((uint32_t)CAN_RE_ARBI_TIMES_UNLIMIT << 24U);
    CAN1->ctrlstat_bit.reset = FALSE;
    s_running = true;
    return LoaderCan_Ready();
}
bool LoaderCan_Ready(void)
{
    return s_running && !CAN1->ctrlstat_bit.reset && !can_busoff_get(CAN1) &&
           !(CAN1->sts & CAN_ROIF_FLAG);
}
bool LoaderCan_Receive(LoaderCanFrame *frame)
{
    if (frame == NULL || !s_running)
    {
        return false;
    }
    if (can_busoff_get(CAN1) || (CAN1->sts & CAN_ROIF_FLAG))
    {
        /* 本轮Ready已使核心撤销会话；恢复物理链路不能恢复旧票据。 */
        can_transmit_abort(CAN1, CAN_ABORT_TRANSMIT_PTB);
        s_failed = s_occupied;
        CAN1->sts = CAN_ROIF_FLAG;
        if (can_busoff_get(CAN1))
        {
            can_busoff_reset(CAN1);
        }
        return false;
    }
    if (CAN1->ctrlstat_bit.rstat == CAN_RXBUF_STATUS_EMPTY)
    {
        return false;
    }
    uint32_t id = CAN1->rbid & 0x1FFFFFFFU;
    uint8_t bytes = s_lengths[CAN1->rbfmt_bit.dlc];
    bool accepted = CAN1->rbfmt_bit.ide && !CAN1->rbfmt_bit.rmf && CAN1->rbfmt_bit.fdf &&
                    CAN1->rbfmt_bit.brs &&
                    (((id >> 8U) & 255U) == s_node || ((id >> 8U) & 255U) == 255U);
    if (accepted)
    {
        frame->identifier = id;
        frame->bytes = bytes;
        for (uint32_t i = 0; i < bytes; i += 4U)
        {
            uint32_t word = CAN1->rbdat[i / 4U];
            uint32_t count = bytes - i < 4U ? bytes - i : 4U;
            memcpy(frame->data + i, &word, count);
        }
    }
    ((volatile uint8_t *)&CAN1->ctrlstat)[3] |= 0x10U;
    return accepted;
}
bool LoaderCan_Send(const LoaderCanFrame *frame, uint64_t ticket)
{
    if (frame == NULL || !LoaderCan_Ready() || s_occupied || CAN1->ctrlstat_bit.tpe ||
        CAN1->ctrlstat_bit.tpa || frame->identifier > 0x1FFFFFFFU)
    {
        return false;
    }
    uint8_t dlc = 0U;
    while (dlc < 16U && s_lengths[dlc] != frame->bytes)
    {
        ++dlc;
    }
    if (dlc == 16U || frame->bytes < 20U)
    {
        return false;
    }
    volatile uint8_t *control = (volatile uint8_t *)&CAN1->ctrlstat;
    control[1] &= 0x60U;
    CAN1->tbid = frame->identifier;
    CAN1->tbfmt = 0U;
    CAN1->tbfmt_bit.ide = TRUE;
    CAN1->tbfmt_bit.fdf = TRUE;
    CAN1->tbfmt_bit.brs = TRUE;
    CAN1->tbfmt_bit.dlc = dlc;
    for (uint32_t i = 0; i < frame->bytes; i += 4U)
    {
        uint32_t word;
        memcpy(&word, frame->data + i, 4U);
        CAN1->tbdat[i / 4U] = word;
    }
    CAN1->tbtyp = 0U;
    CAN1->reserved8 = 0U;
    can_flag_clear(CAN1, CAN_TPIF_FLAG | CAN_AIF_FLAG);
    s_ticket = ticket;
    s_started_ms = LoaderPlatform_NowMs();
    s_failed = false;
    s_occupied = true;
    control[1] = (control[1] & 0xE0U) | CAN_TRANSMIT_PTB;
    return true;
}
bool LoaderCan_TxEvent(LoaderCanTxEvent *event)
{
    if (event == NULL || !s_occupied)
    {
        return false;
    }
    bool busy = CAN1->ctrlstat_bit.tpe || CAN1->ctrlstat_bit.tpa;
    bool success = !s_failed && !busy && (uint32_t)(LoaderPlatform_NowMs() - s_started_ms) < 500U &&
                   (CAN1->sts & CAN_TPIF_FLAG) && !(CAN1->sts & CAN_AIF_FLAG) &&
                   CAN1->tstat_bit.tstat_2 == CAN_TSTAT_TRANSMITTED && LoaderCan_Ready();
    bool failure = s_failed || !LoaderCan_Ready() || (CAN1->sts & CAN_AIF_FLAG) ||
                   (!busy && (CAN1->sts & CAN_TPIF_FLAG) && !success) ||
                   (uint32_t)(LoaderPlatform_NowMs() - s_started_ms) >= 500U;
    if (!success && !failure)
    {
        return false;
    }
    if (!success)
    {
        can_transmit_abort(CAN1, CAN_ABORT_TRANSMIT_PTB);
    }
    *event = (LoaderCanTxEvent){s_ticket, success};
    s_occupied = false;
    s_failed = false;
    return true;
}
void LoaderCan_Shutdown(void)
{
    s_running = false;
    s_failed = s_occupied;
    can_interrupt_enable(CAN1, CAN_ALL_INT, FALSE);
    can_transmit_abort(CAN1, CAN_ABORT_TRANSMIT_PTB);
    can_software_reset(CAN1, TRUE);
}
