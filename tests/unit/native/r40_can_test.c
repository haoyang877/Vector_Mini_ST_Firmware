/* 以寄存器事件验证CAN生产端口的物理完成与失败语义。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../firmware/platform/at32f456/ports/comm_at32f456.c"

static bool
filter_accepts(uint32_t id, bool extended, bool remote, uint8_t dlc, bool fd, bool brs, bool esi)
{
    /* RM 21.3.1：掩码1忽略、0比较；独立模拟验收条件，防止沿用其他MCU相反语义。 */
    const can_filter_code_type *code = &sim_filter.code_para;
    const can_filter_mask_type *mask = &sim_filter.mask_para;
    return (((id ^ code->id) & ~mask->id & 0x1FFFFFFFU) == 0U) &&
           (mask->id_type || extended == (code->id_type == CAN_ID_EXTENDED)) &&
           (mask->frame_type || remote == (code->frame_type == CAN_FRAME_REMOTE)) &&
           (((dlc ^ code->data_length) & ~mask->data_length & 0xFU) == 0U) &&
           (mask->fd_format || fd == (code->fd_format == CAN_FORMAT_FD)) &&
           (mask->fd_rate_switch || brs == (code->fd_rate_switch == CAN_BRS_ON)) &&
           (mask->fd_error_state || esi == (code->fd_error_state == CAN_ESI_PASSIVE));
}

static void finish(bool success)
{
    sim_can.ctrlstat_bit.tpe = 0U;
    sim_can.ctrlstat_bit.tpa = 0U;
    sim_can.tstat_bit.handle_2 = sim_tx.handle;
    sim_can.tstat_bit.tstat_2 = success ? CAN_TSTAT_TRANSMITTED : CAN_TSTAT_DISTURBED;
    sim_can.sts = success ? CAN_TPIF_FLAG : 0U;
}

int main(void)
{
    CommHwCanFrame frame = {.identifier = 0x1234567U,
                            .length = 8U,
                            .extended = true,
                            .fd = true,
                            .bitrate_switch = true};
    assert(!comm_hw_can_try_send_frame(&frame) && !comm_hw_can_is_healthy());
    system_core_clock = 192000000U;
    assert(!R40_CanInit());
    system_core_clock = R40_CORE_HZ;
    sim_can_pclk_hz = 90000000U;
    assert(!R40_CanInit());
    sim_can_pclk_hz = R40_CAN_HZ;
    assert(R40_CanInit() && !R40_CanInit());
    assert(sim_timing.bittime_div == 1U);
    assert(R40_CAN_HZ / (sim_timing.ac_bts1_size + sim_timing.ac_bts2_size) == 1000000U);
    assert(R40_CAN_HZ / (sim_timing.fd_bts1_size + sim_timing.fd_bts2_size) == 5000000U);
    assert(sim_timing.fd_ssp_offset == 28U);
    for (unsigned bit = 0U; bit < 29U; ++bit)
    {
        for (unsigned dlc = 0U; dlc < 16U; ++dlc)
        {
            for (unsigned flags = 0U; flags < 8U; ++flags)
            {
                assert(filter_accepts(
                    1U << bit, true, false, dlc, flags & 1U, flags & 2U, flags & 4U));
            }
        }
    }
    assert(filter_accepts(0U, true, false, 8U, true, true, false));
    assert(filter_accepts(0x1FFFFFFFU, true, false, 15U, true, true, false));
    assert(filter_accepts(0x1FFF0001U, true, false, 8U, true, true, false));
    assert(!filter_accepts(0U, false, false, 8U, true, true, false));
    assert(!filter_accepts(0U, true, true, 8U, true, true, false));
    comm_hw_can_start_fd();
    assert(comm_hw_can_is_healthy());
    assert(sim_can.lbtcfg_bit.retlim == CAN_RE_TRANS_TIMES_1);
    assert(sim_can.lbtcfg_bit.realim == CAN_RE_ARBI_TIMES_UNLIMIT);
    for (unsigned length = 0U; length <= 65U; ++length)
    {
        bool legal = false;
        for (unsigned dlc = 0U; dlc < 16U; ++dlc)
        {
            legal |= s_dlc_lengths[dlc] == length;
        }
        frame.length = length;
        bool accepted = comm_hw_can_try_send_frame(&frame);
        assert(accepted == legal);
        if (accepted)
        {
            finish(true);
        }
    }
    frame.length = 8U;
    frame.remote = true;
    assert(!comm_hw_can_try_send_frame(&frame));
    frame.remote = false;
    frame.extended = false;
    assert(!comm_hw_can_try_send_frame(&frame));
    frame.extended = true;
    frame.fd = false;
    assert(!comm_hw_can_try_send_frame(&frame));
    frame.fd = true;
    frame.bitrate_switch = false;
    assert(!comm_hw_can_try_send_frame(&frame));
    frame.bitrate_switch = true;
    assert(!comm_hw_can_try_send_frame(NULL));
    assert(!comm_hw_can_try_send_tracked_frame(&frame, 0U));
    const uint64_t ticket = UINT64_C(0x1122334455667788);
    assert(comm_hw_can_try_send_tracked_frame(&frame, ticket));
    CommHwCanTxCompletion result = {99U, false};
    assert(!comm_hw_can_take_tx_completion(&result) && result.ticket == 99U);
    assert(!comm_hw_can_try_send_frame(&frame));
    finish(true);
    assert(!comm_hw_can_try_send_frame(&frame));
    assert(comm_hw_can_take_tx_completion(&result) && result.ticket == ticket && result.success);
    assert(!comm_hw_can_take_tx_completion(&result));
    assert(comm_hw_can_try_send_tracked_frame(&frame, ticket));
    sim_complete_during_status = true;
    assert(!comm_hw_can_take_tx_completion(&result));
    assert(comm_hw_can_take_tx_completion(&result) && result.success);
    assert(comm_hw_can_try_send_tracked_frame(&frame, ticket));
    finish(true);
    sim_can.ctrlstat_bit.tpe = 1U; /* 完成位先到，写锁尚未释放。 */
    assert(!comm_hw_can_take_tx_completion(&result));
    sim_can.ctrlstat_bit.tpe = 0U;
    assert(comm_hw_can_take_tx_completion(&result) && result.success);
    sim_ms = UINT32_MAX - 50U;
    assert(comm_hw_can_try_send_tracked_frame(&frame, ticket + 1U));
    sim_ms += 100U;
    assert(comm_hw_can_take_tx_completion(&result) && !result.success);
    assert(sim_can.ctrlstat_bit.tpa && !comm_hw_can_try_send_frame(&frame));
    finish(true); /* 取消后到达的迟到成功不得生成第二条成功记录。 */
    assert(!comm_hw_can_take_tx_completion(&result));
    assert(comm_hw_can_try_send_tracked_frame(&frame, ticket + 2U));
    sim_can.ctrlstat_bit.busoff = 1U;
    CAN1_STAT_IRQHandler();
    assert(!comm_hw_can_is_healthy());
    sim_recovery_fail = true;
    assert(!comm_hw_can_service_bus_off());
    sim_recovery_fail = false;
    assert(comm_hw_can_service_bus_off() && sim_bit_writes == 1U);
    assert(comm_hw_can_take_tx_completion(&result) && !result.success &&
           result.ticket == ticket + 2U);
    assert(!comm_hw_can_try_send_frame(&frame));
    finish(false);
    assert(comm_hw_can_try_send_tracked_frame(&frame, ticket + 3U));
    finish(true);
    sim_can.tstat_bit.handle_2 ^= 1U;
    assert(comm_hw_can_take_tx_completion(&result) && !result.success);
    assert(comm_hw_can_try_send_tracked_frame(&frame, ticket + 4U));
    finish(false);
    assert(comm_hw_can_take_tx_completion(&result) && !result.success);
    assert(comm_hw_can_try_send_tracked_frame(&frame, ticket + 5U));
    finish(true);
    assert(!comm_hw_can_try_send_frame(&frame));
    sim_can.ctrlstat_bit.busoff = 1U;
    assert(comm_hw_can_take_tx_completion(&result) && !result.success);
    assert(comm_hw_can_service_bus_off());
    sim_can.sts = CAN_EPASS_FLAG;
    assert(!comm_hw_can_is_healthy());
    sim_can.sts = CAN_EWARN_FLAG;
    assert(!comm_hw_can_is_healthy());
    sim_can.sts = 0U;
    sim_rx.id = frame.identifier;
    sim_rx.id_type = CAN_ID_EXTENDED;
    sim_rx.frame_type = CAN_FRAME_DATA;
    sim_rx.fd_format = CAN_FORMAT_FD;
    sim_rx.fd_rate_switch = CAN_BRS_ON;
    sim_rx.data_length = CAN_DLC_BYTES_64;
    sim_rx.data[63] = 42U;
    sim_rx_ready = true;
    assert(comm_hw_can_receive(&frame) && frame.length == 64U && frame.data[63] == 42U);
    CommHwCanFrame previous = frame;
    sim_rx.fd_rate_switch = CAN_BRS_OFF;
    sim_rx_ready = true;
    assert(!comm_hw_can_receive(&frame) && memcmp(&frame, &previous, sizeof(frame)) == 0);
    assert(!comm_hw_can_receive(NULL));
    puts("PASS 1M/5M, DLC, physical completion, 64-bit ticket, timeout, late ACK and bus-off");
    return 0;
}
