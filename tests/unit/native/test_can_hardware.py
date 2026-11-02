"""验证当前CAN硬件收发契约；历史自动bus-off恢复已退役。"""

from _bsp_test_support import ROOT, arguments, production, resources, run_c


def main() -> None:
    """两板执行实际报文编解码、忙/完成状态和单帧槽行为。"""
    args = arguments(__doc__)
    for board in (1, 2):
        source = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bsp_can.h"
static const uint8_t s_dlc_lengths[16]={0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
"""
        extra = []
        if board == 1:
            source += r"""
static struct {uint32_t CCCR,IE,ILE,TXFQS,TXBRP,TXBTO,TXBAR,RXF0S,RXF0A;} can;
static uint32_t txram[54],rxram[54];
static struct {struct {uintptr_t TxFIFOQSA,RxFIFO0SA;} msgRam;} hfdcan1;
#define FDCAN1 (&can)
#define FDCAN_CCCR_INIT 1U
#define FDCAN_IE_RF0NE 1U
#define FDCAN_ILE_EINT0 1U
#define FDCAN_TXFQS_TFQPI (31U<<16)
#define FDCAN_TXFQS_TFQPI_Pos 16U
#define FDCAN_RXF0S_F0FL 127U
#define FDCAN_RXF0S_F0GI (63U<<8)
#define FDCAN_RXF0S_F0GI_Pos 8U
"""
        else:
            source += '\n#include "at32f45x.h"\n#define BSP_APP_LIFECYCLE 1\ncan_type sim_can;\n'
            extra = [
                "-I",
                ROOT / "tests/unit/native/r40_io_fake",
                "-I",
                ROOT / "firmware/platform/sdk/at32f45x/drivers/inc",
            ]
        source += resources(board) + production(
            board,
            "can",
            "BspCan_Start",
            "BspCan_TrySend",
            "BspCan_Receive",
            "BspCan_IsTransmitBusy",
            "BspCan_IsTransmitComplete",
        )
        if board == 1:
            source += r"""
static void setup(void) {
    hfdcan1.msgRam.TxFIFOQSA=(uintptr_t)txram;hfdcan1.msgRam.RxFIFO0SA=(uintptr_t)rxram;
    can.CCCR=3;BspCan_Start();assert(can.CCCR==2 && can.IE==1 && can.ILE==1);
    for(unsigned next=0;next<3;next++) for(unsigned mask=0;mask<8;mask++) {
        can.TXFQS=next<<16;can.TXBTO=mask;can.TXBRP=0;
        bool expected=(mask&(1U<<(next==0 ? 2 : next-1)))!=0;
        assert(BspCan_IsTransmitComplete()==expected);
        assert(BspCan_IsTransmitComplete()==expected);
        can.TXBRP=1;assert(BspCan_IsTransmitBusy() && !BspCan_IsTransmitComplete());
    }
    can.TXBRP=0;
}
static void send_receive(const BspCanFrame *frame,unsigned dlc,unsigned slot) {
    memset(txram,0xA5,sizeof(txram));can.TXFQS=slot<<16;can.TXBRP=0;
    assert(BspCan_TrySend(frame));assert(can.TXBAR==(1U<<slot));
    uint32_t *message=txram+slot*18;
    assert(message[0]==((frame->extended ? 0x40000000U|frame->identifier : frame->identifier<<18)|(frame->remote ? 0x20000000U : 0)));
    assert(message[1]==((dlc<<16)|(frame->fd ? 1U<<21 : 0)|(frame->bitrate_switch ? 1U<<20 : 0)));
    assert(memcmp(message+2,frame->data,frame->length)==0);
    uint32_t before[54];memcpy(before,txram,sizeof(before));can.TXBRP=1;
    assert(!BspCan_TrySend(frame) && memcmp(before,txram,sizeof(before))==0);can.TXBRP=0;
    memcpy(rxram+slot*18,message,18*sizeof(uint32_t));can.RXF0S=1|(slot<<8);
}
"""
        else:
            source += r"""
static void setup(void) {
    sim_can.ctrlstat_bit.reset=1;BspCan_Start();assert(!sim_can.ctrlstat_bit.reset);
    assert(sim_can.lbtcfg_bit.retlim==CAN_RE_TRANS_TIMES_1 && sim_can.lbtcfg_bit.realim==CAN_RE_ARBI_TIMES_UNLIMIT);
    assert(sim_can.inten&CAN_RIE_INT);
    for(unsigned busy=0;busy<4;busy++) for(unsigned done=0;done<2;done++) for(unsigned status=0;status<8;status++) {
        sim_can.ctrlstat_bit.tpe=busy&1;sim_can.ctrlstat_bit.tpa=busy>>1;
        sim_can.sts=done ? CAN_TPIF_FLAG : 0;sim_can.tstat_bit.tstat_2=status;
        assert(BspCan_IsTransmitBusy()==(busy!=0));
        bool expected=!busy && done && status==CAN_TSTAT_TRANSMITTED;
        assert(BspCan_IsTransmitComplete()==expected && BspCan_IsTransmitComplete()==expected);
    }
    sim_can.ctrlstat_bit.tpe=sim_can.ctrlstat_bit.tpa=0;
}
static void send_receive(const BspCanFrame *frame,unsigned dlc,unsigned slot) {
    (void)slot;sim_can.ctrlstat_bit.tpe=sim_can.ctrlstat_bit.tpa=0;
    assert(BspCan_TrySend(frame));
    assert(sim_can.tbid==(frame->extended ? frame->identifier : frame->identifier<<18));
    assert(sim_can.tbfmt_bit.ide==frame->extended && sim_can.tbfmt_bit.rmf==frame->remote);
    assert(sim_can.tbfmt_bit.dlc==dlc && sim_can.tbfmt_bit.fdf==frame->fd && sim_can.tbfmt_bit.brs==frame->bitrate_switch);
    for(unsigned i=0;i<frame->length;i++) assert(((volatile uint8_t*)sim_can.tbdat)[i]==frame->data[i]);
    uint32_t id=sim_can.tbid;sim_can.ctrlstat_bit.tpe=1;
    assert(!BspCan_TrySend(frame) && sim_can.tbid==id);sim_can.ctrlstat_bit.tpe=0;
    sim_can.rbid=sim_can.tbid;sim_can.rbfmt=sim_can.tbfmt;
    for(unsigned i=0;i<16;i++) sim_can.rbdat[i]=sim_can.tbdat[i];
    sim_can.ctrlstat_bit.rstat=1;
}
"""
        source += r"""
int main(void) {
    setup();
    const uint8_t lengths[]={0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    for(unsigned slot=0;slot<3;slot++) for(unsigned ext=0;ext<2;ext++) for(unsigned index=0;index<16;index++) for(unsigned remote=0;remote<2;remote++) {
        if(remote && index>8) continue;
        BspCanFrame frame={.identifier=ext ? 0x1ABCDEFU : 0x5AAU,.extended=ext,.remote=remote,
                           .fd=!remote,.bitrate_switch=!remote,.length=lengths[index]};
        for(unsigned i=0;i<64;i++) frame.data[i]=(uint8_t)(17*i+index);
        send_receive(&frame,index,slot);
        BspCanFrame received;memset(&received,0xA5,sizeof(received));
        assert(BspCan_Receive(&received));
        assert(received.identifier==frame.identifier && received.length==frame.length);
        assert(received.extended==frame.extended && received.remote==frame.remote && received.fd==frame.fd && received.bitrate_switch==frame.bitrate_switch);
        for(unsigned i=0;i<64;i++) assert(received.data[i]==(!remote && i<frame.length ? frame.data[i] : 0xA5));
#if BSP_BOARD==1
        assert(can.RXF0A==slot);can.RXF0S=0;
#else
        sim_can.ctrlstat_bit.rstat=0;
#endif
        BspCanFrame before=received;assert(!BspCan_Receive(&received) && memcmp(&before,&received,sizeof(before))==0);
    }
    puts("PASS CAN hardware: all DLCs, IDs, data/remote frames, busy rejection and non-consuming completion");
    return 0;
}
"""
        run_c(args, f"can_{board}", source, board, extra)


if __name__ == "__main__":
    main()
