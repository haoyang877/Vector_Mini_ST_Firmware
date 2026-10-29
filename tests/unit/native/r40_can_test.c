/* 以寄存器事件验证CAN生产端口的物理完成与失败语义。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include "../../../firmware/platform/bsp/mcu/at32f456/can.c"
#include "../../../firmware/platform/bsp/bsp_can.c"
#include "../../../firmware/platform/bsp/can_frame_codec.c"
#include "can_legacy_assertions.h"

static jmp_buf fatal_exit;
/** @brief 记录真实致命停留前的关相和中断关闭，再返回测试环境。 */
static void catch_fatal(void)
{
    assert(sim_power_off && __get_PRIMASK() != 0U);
    longjmp(fatal_exit, 1);
}
/** @brief 模拟独立上电，不以重置生产状态替代单次运行验证。 */
static void boot_fixture(void)
{
    s_configured = s_started = s_controller_started = s_busoff_latched = s_initialized =
        s_recovery_seen = false;
    memset(&s_tx, 0, sizeof(s_tx));
    sim_can_reset_stuck = sim_can_release_stuck = sim_can_retlim_stuck = false;
    sim_can_realim_stuck = sim_can_timing_bad = false;
    sim_nvic_pending = sim_nvic_enabled = sim_can_irq_mask = 0U;
    sim_bit_writes = sim_can_writes = 0U;
    sim_fatal_hook = NULL;
    BspSystem_ExitCritical(0U);
    system_core_clock = R40_CORE_HZ;
    sim_can_pclk_hz = R40_CAN_HZ;
    memset(&sim_can, 0, sizeof(sim_can));
}
/** @brief 配置读回、重复启动与有界等待失败必须保持关相并进入原致命路径。 */
static void test_start_failures(void)
{
    boot_fixture();
    sim_can_reset_stuck = true;
    assert(!BspCan_Init());
    boot_fixture();
    sim_can_timing_bad = true;
    assert(!BspCan_Init());
    for (unsigned fault = 2U; fault < 5U; ++fault)
    {
        boot_fixture();
        if (fault != 0U)
        {
            assert(BspCan_Init());
        }
        if (fault == 1U)
        {
            BspCan_Start();
        }
        sim_can_release_stuck = fault == 2U;
        sim_can_retlim_stuck = fault == 3U;
        sim_can_realim_stuck = fault == 4U;
        sim_power_off = false;
        sim_fatal_hook = catch_fatal;
        if (setjmp(fatal_exit) == 0)
        {
            BspCan_Start();
            assert(!"expected original startup halt");
        }
    }
    boot_fixture();
}
static unsigned irq_frames;
/** @brief 三帧预算的接收消费者，用于确认硬件ACK和积压重挂的顺序。 */
static void receive_three(void)
{
    assert((sim_can.sts & CAN_RIF_FLAG) == 0U);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        BspCanFrame frame;
        if (BspCan_Receive(&frame))
        {
            ++irq_frames;
        }
    }
}
/** @brief 七帧积压必须分三次IRQ处理，接收中断仅在指定启动阶段开放。 */
static void test_receive_irq(void)
{
    assert((sim_can_irq_mask & CAN_RIE_INT) != 0U);
    assert((sim_can_irq_mask & CAN_RIE_INT) != 0U);
    assert((sim_nvic_enabled & (1U << CAN1_RX_IRQn)) != 0U);
    sim_rx = (can_rxbuf_type){.id = 0x1234567U,
                              .id_type = CAN_ID_EXTENDED,
                              .frame_type = CAN_FRAME_DATA,
                              .fd_format = CAN_FORMAT_FD,
                              .fd_rate_switch = CAN_BRS_ON,
                              .data_length = CAN_DLC_BYTES_8};
    sim_rx_remaining = 7U;
    irq_frames = 0U;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        sim_can.sts |= CAN_RIF_FLAG;
        sim_nvic_pending = 0U;
        BspCan_OnReceiveInterrupt(receive_three);
        assert(irq_frames == (i == 2U ? 7U : (i + 1U) * 3U));
        assert(((sim_nvic_pending & (1U << CAN1_RX_IRQn)) != 0U) == (i != 2U));
    }
    BspCan_OnReceiveInterrupt(NULL);
    assert(irq_frames == 7U);
}

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
    test_start_failures();
    BspCanFrame frame = {.identifier = 0x1234567U,
                         .length = 8U,
                         .extended = true,
                         .fd = true,
                         .bitrate_switch = true};
    assert(!BspCan_TrySendFrame(&frame) && !BspCan_IsHealthy());
    system_core_clock = 192000000U;
    assert(!BspCan_Init());
    system_core_clock = R40_CORE_HZ;
    sim_can_pclk_hz = 90000000U;
    assert(!BspCan_Init());
    sim_can_pclk_hz = R40_CAN_HZ;
    assert(BspCan_Init() && !BspCan_Init());
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
    BspCan_Start();
    assert(BspCan_IsHealthy());
    assert(sim_can.lbtcfg_bit.retlim == CAN_RE_TRANS_TIMES_1);
    assert(sim_can.lbtcfg_bit.realim == CAN_RE_ARBI_TIMES_UNLIMIT);
    test_receive_irq();
    for (unsigned length = 0U; length <= 65U; ++length)
    {
        bool legal = false;
        for (unsigned dlc = 0U; dlc < 16U; ++dlc)
        {
            const uint8_t lengths[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
            legal |= lengths[dlc] == length;
        }
        frame.length = length;
        uint32_t outer_priority = length == 16U ? 0x10U : 0U;
        __set_BASEPRI(outer_priority);
        bool accepted = BspCan_TrySendFrame(&frame);
        assert(accepted == legal);
        if (accepted)
        {
            assert(sim_can_submit_primask == 0U);
            assert(sim_can_submit_basepri == (outer_priority ? outer_priority : 0x20U));
            assert(__get_BASEPRI() == outer_priority);
            finish(true);
        }
        __set_BASEPRI(0U);
    }
    frame.length = 8U;
    frame.remote = true;
    assert(!BspCan_TrySendFrame(&frame));
    frame.remote = false;
    frame.extended = false;
    assert(!BspCan_TrySendFrame(&frame));
    frame.extended = true;
    frame.fd = false;
    assert(!BspCan_TrySendFrame(&frame));
    frame.fd = true;
    frame.bitrate_switch = false;
    assert(BspCan_TrySendFrame(&frame));
    finish(true);
    frame.bitrate_switch = true;
    assert(!BspCan_TrySendFrame(NULL));
    assert(!BspCan_TrySendTrackedFrame(&frame, 0U));
    const uint64_t ticket = UINT64_C(0x1122334455667788);
    assert(BspCan_TrySendTrackedFrame(&frame, ticket));
    BspCanTxCompletion result = {99U, false};
    assert(!BspCan_TakeTxCompletion(&result) && result.ticket == 99U);
    assert(!BspCan_TrySendFrame(&frame));
    finish(true);
    assert(!BspCan_TrySendFrame(&frame));
    assert(BspCan_TakeTxCompletion(&result) && result.ticket == ticket && result.success);
    assert(!BspCan_TakeTxCompletion(&result));
    assert(BspCan_TrySendTrackedFrame(&frame, ticket));
    sim_complete_during_status = true;
    assert(!BspCan_TakeTxCompletion(&result));
    assert(BspCan_TakeTxCompletion(&result) && result.success);
    assert(BspCan_TrySendTrackedFrame(&frame, ticket));
    finish(true);
    sim_can.ctrlstat_bit.tpe = 1U; /* 完成位先到，写锁尚未释放。 */
    assert(!BspCan_TakeTxCompletion(&result));
    sim_can.ctrlstat_bit.tpe = 0U;
    assert(BspCan_TakeTxCompletion(&result) && result.success);
    sim_ms = UINT32_MAX - 50U;
    assert(BspCan_TrySendTrackedFrame(&frame, ticket + 1U));
    sim_ms += 100U;
    assert(BspCan_TakeTxCompletion(&result) && !result.success);
    assert(sim_can.ctrlstat_bit.tpa && !BspCan_TrySendFrame(&frame));
    finish(true); /* 取消后到达的迟到成功不得生成第二条成功记录。 */
    assert(!BspCan_TakeTxCompletion(&result));
    assert(BspCan_TrySendTrackedFrame(&frame, ticket + 2U));
    sim_can.ctrlstat_bit.busoff = 1U;
    CAN1_STAT_IRQHandler();
    assert(!BspCan_IsHealthy());
    sim_recovery_fail = true;
    assert(!BspCan_ServiceBusOff());
    sim_recovery_fail = false;
    sim_ms += 1000U;
    assert(BspCan_ServiceBusOff() && sim_bit_writes == 1U);
    assert(BspCan_TakeTxCompletion(&result) && !result.success && result.ticket == ticket + 2U);
    assert(!BspCan_TrySendFrame(&frame));
    finish(false);
    assert(BspCan_TrySendTrackedFrame(&frame, ticket + 3U));
    finish(true);
    sim_can.tstat_bit.handle_2 ^= 1U;
    assert(BspCan_TakeTxCompletion(&result) && !result.success);
    assert(BspCan_TrySendTrackedFrame(&frame, ticket + 4U));
    finish(false);
    assert(BspCan_TakeTxCompletion(&result) && !result.success);
    assert(BspCan_TrySendTrackedFrame(&frame, ticket + 5U));
    finish(true);
    assert(!BspCan_TrySendFrame(&frame));
    sim_can.ctrlstat_bit.busoff = 1U;
    assert(BspCan_TakeTxCompletion(&result) && result.success);
    sim_ms += 1000U;
    assert(BspCan_ServiceBusOff());
    sim_can.sts = CAN_EPASS_FLAG;
    assert(!BspCan_IsHealthy());
    sim_can.sts = CAN_EWARN_FLAG;
    assert(!BspCan_IsHealthy());
    sim_can.sts = 0U;
    sim_rx.id = frame.identifier;
    sim_rx.id_type = CAN_ID_EXTENDED;
    sim_rx.frame_type = CAN_FRAME_DATA;
    sim_rx.fd_format = CAN_FORMAT_FD;
    sim_rx.fd_rate_switch = CAN_BRS_ON;
    sim_rx.data_length = CAN_DLC_BYTES_64;
    sim_rx.data[63] = 42U;
    sim_rx_ready = true;
    assert(BspCan_Receive(&frame) && frame.length == 64U && frame.data[63] == 42U);
    BspCanFrame previous = frame;
    sim_rx.fd_rate_switch = CAN_BRS_OFF;
    sim_rx_ready = true;
    assert(BspCan_Receive(&frame) && !frame.bitrate_switch);
    frame = previous;
    sim_rx.data_length = (can_data_length_type)16U;
    sim_rx_ready = true;
    assert(!BspCan_Receive(&frame) && memcmp(&frame, &previous, sizeof(frame)) == 0);
    assert(!BspCan_Receive(NULL));
    puts("PASS 1M/5M, DLC, physical completion, 64-bit ticket, timeout, late ACK and bus-off");
    return 0;
}
