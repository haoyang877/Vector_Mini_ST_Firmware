/* 真实AT32端口在寄存器模型上验证票据、终态、超时及过滤；不访问硬件。 */
#include <assert.h>
#include <stdio.h>
#include "at32f45x.h"
static can_type model_can;
static uint32_t model_time;
#undef CAN1
#define CAN1 (&model_can)
#include "../../loader/ports/at32f456/can_port.c"
#include "native/can_bitrate_register_test.h"
static can_bittime_type observed_timing;

uint32_t LoaderPlatform_NowMs(void)
{
    return model_time;
}
void LoaderBoard_CanInit(void)
{
}
void can_flag_clear(can_type *can, uint32_t flags)
{
    can->sts &= ~flags;
}
void can_reset(can_type *can)
{
    (void)can;
}
void can_bittime_default_para_init(can_bittime_type *timing)
{
    memset(timing, 0, sizeof(*timing));
}
void can_bittime_set(can_type *can, can_bittime_type *timing)
{
    assert(can->ctrlstat_bit.reset);
    observed_timing = *timing;
}
void can_fd_iso_mode_enable(can_type *can, confirm_state value)
{
    (void)can;
    (void)value;
}
void can_mode_set(can_type *can, can_mode_type mode)
{
    (void)can;
    (void)mode;
}
void can_receive_all_enable(can_type *can, confirm_state value)
{
    (void)can;
    (void)value;
}
void can_rxbuf_overflow_mode_set(can_type *can, can_rxbuf_overflow_mode_type value)
{
    (void)can;
    (void)value;
}
void can_filter_default_para_init(can_filter_config_type *filter)
{
    memset(filter, 0, sizeof(*filter));
}
void can_filter_enable(can_type *can, can_filter_type filter, confirm_state value)
{
    (void)can;
    (void)filter;
    (void)value;
}
void can_filter_set(can_type *can, can_filter_type index, can_filter_config_type *filter)
{
    (void)can;
    (void)index;
    (void)filter;
}
flag_status can_busoff_get(can_type *can)
{
    return can->ctrlstat_bit.busoff ? SET : RESET;
}
void can_busoff_reset(can_type *can)
{
    can->ctrlstat_bit.busoff = 0;
}
void can_transmit_abort(can_type *can, can_abort_transmit_type unused)
{
    (void)unused;
    can->ctrlstat_bit.tpe = 0;
    can->ctrlstat_bit.tpa = 0;
}
void can_software_reset(can_type *can, confirm_state enabled)
{
    can->ctrlstat_bit.reset = enabled;
}
void can_interrupt_enable(can_type *can, uint32_t mask, confirm_state enabled)
{
    (void)can;
    (void)mask;
    (void)enabled;
}
static void sent(void)
{
    model_can.ctrlstat_bit.tpe = 0;
    model_can.ctrlstat_bit.tpa = 0;
    model_can.sts = CAN_TPIF_FLAG;
    model_can.tstat_bit.tstat_2 = CAN_TSTAT_TRANSMITTED;
}
int main(int argc, char **argv)
{
    LoaderCanFrame frame = {0x00EF0203U, 20, {0}};
    LoaderCanTxEvent event;
    assert(argc == 7 && !LoaderCan_Init(3, 12U));
    assert(LoaderCan_Init(3, (uint8_t)strtoul(argv[1], NULL, 10)));
    check_can_timing(&observed_timing, argv);
    assert(LoaderCan_Send(&frame, 0x123456789ABCDEF0ULL));
    model_can.sts = 0; /* 寄存器模型显式模拟W1C及异步硬件进展。 */
    assert(!LoaderCan_Send(&frame, 2));
    assert(!LoaderCan_TxEvent(&event));
    sent();
    assert(LoaderCan_TxEvent(&event) && event.success && event.ticket == 0x123456789ABCDEF0ULL);
    assert(!LoaderCan_TxEvent(&event));
    assert(LoaderCan_Send(&frame, 2));
    model_can.sts = 0;
    model_time = 500;
    sent();
    assert(LoaderCan_TxEvent(&event) && !event.success && event.ticket == 2);
    assert(LoaderCan_Send(&frame, 3));
    model_can.sts = 0;
    model_can.ctrlstat_bit.busoff = 1;
    assert(!LoaderCan_Ready());
    assert(!LoaderCan_Receive(&frame));
    sent();
    assert(LoaderCan_TxEvent(&event) && !event.success && event.ticket == 3);
    model_can.ctrlstat_bit.rstat = CAN_RXBUF_STATUS_LESS;
    model_can.rbfmt_bit.ide = 1;
    model_can.rbfmt_bit.fdf = 1;
    model_can.rbfmt_bit.brs = 1;
    model_can.rbfmt_bit.dlc = 11;
    model_can.rbid = 0x00EF0902;
    assert(!LoaderCan_Receive(&frame));
    model_can.rbid = 0x00EF0302;
    assert(LoaderCan_Receive(&frame) && frame.identifier == 0x00EF0302 && frame.bytes == 20);
    puts("PASS real AT32 CAN port: exact ticket, late TX, bus-off and node filter");
    return 0;
}
