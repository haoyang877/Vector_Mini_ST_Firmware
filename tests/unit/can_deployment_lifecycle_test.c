/* 只用内存 HAL；运行真实 APP 端口初始化、重配置、发送及 BOOT 状态机。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "fdcan.h"
#include "comm_hw.h"
#include "boot_handoff.h"

Handle hfdcan1;
static unsigned configured_node, starts, inits, stops, adds;
static unsigned last_fd, last_brs, last_length;

void Error_Handler(void) { assert(!"unexpected HAL failure"); }
unsigned HAL_FDCAN_Init(Handle *h)
{
    assert(h == &hfdcan1);
    assert(h->Init.FrameFormat == (VECTOR_CAN_CLASSIC_ONLY ? FDCAN_FRAME_CLASSIC : FDCAN_FRAME_FD_BRS));
    ++inits;
    return HAL_OK;
}
unsigned HAL_FDCAN_Start(Handle *h) { (void)h; ++starts; return HAL_OK; }
unsigned HAL_FDCAN_Stop(Handle *h) { (void)h; ++stops; return HAL_OK; }
unsigned HAL_FDCAN_ConfigFilter(Handle *h,const FDCAN_FilterTypeDef *filter)
{
    (void)h;
    assert(filter->FilterID1 == (configured_node << 8));
    assert(filter->FilterID2 == ((configured_node << 8) | 255));
    return HAL_OK;
}
unsigned HAL_FDCAN_ConfigGlobalFilter(Handle *h,unsigned a,unsigned b,unsigned c,unsigned d)
{ (void)h; assert(a==FDCAN_REJECT && b==FDCAN_REJECT && c==FDCAN_FILTER_REMOTE && d==FDCAN_FILTER_REMOTE); return HAL_OK; }
unsigned HAL_FDCAN_ActivateNotification(Handle *h,unsigned a,unsigned b)
{ (void)h;assert(a==FDCAN_IT_RX_FIFO0_NEW_MESSAGE && b==0);return HAL_OK; }
unsigned HAL_FDCAN_IsTxBufferMessagePending(Handle *h,unsigned mask)
{ (void)h;(void)mask;return 0; }
unsigned HAL_FDCAN_AddMessageToTxFifoQ(Handle *h,const FDCAN_TxHeaderTypeDef *header,const uint8_t *data)
{
    (void)h;(void)data; ++adds;
    last_fd=header->FDFormat;last_brs=header->BitRateSwitch;last_length=header->DataLength;
    return HAL_OK;
}
unsigned HAL_FDCAN_GetRxMessage(Handle *h,unsigned q,FDCAN_RxHeaderTypeDef *header,uint8_t *data)
{ (void)h;(void)q;(void)header;(void)data;return 1; }

int main(void)
{
    uint8_t payload[48]={0};
    const unsigned rates[]={100,125,200,250,500,1000};
    hfdcan1.Init.TxFifoQueueMode=FDCAN_TX_QUEUE_OPERATION;
    for(configured_node=0;configured_node<8;++configured_node) {
        comm_hw_can_start(configured_node);
        for(unsigned i=0;i<sizeof(rates)/sizeof(rates[0]);++i) {
            unsigned count=starts;
            comm_hw_can_set_bitrate(rates[i]);
            assert(starts==count+1 && inits==starts);
            assert(hfdcan1.Init.NominalPrescaler==10000U/rates[i]);
        }
    }
    unsigned count=starts;
    configured_node=7;
    comm_hw_can_set_bitrate(0);assert(starts==count);
    comm_hw_can_set_bitrate(2000);
    assert(starts==count+(VECTOR_CAN_CLASSIC_ONLY?0U:1U));
    for(unsigned length=2;length<=8;length*=2) {
        assert(comm_hw_can_try_send_reply_format(0x309,payload,length,false,false));
        assert(last_fd==FDCAN_CLASSIC_CAN && last_brs==FDCAN_BRS_OFF && last_length==length);
    }
    count=adds;
    assert(!comm_hw_can_try_send_reply_format(0x309,payload,16,false,false));
    assert(!comm_hw_can_try_send_reply_format(0x309,payload,4,false,true));
    assert(adds==count);
    assert(comm_hw_can_try_send_status(0x7f3,payload,48)==!VECTOR_CAN_CLASSIC_ONLY);
    assert(comm_hw_can_try_send_reply(0x309,payload,4)==!VECTOR_CAN_CLASSIC_ONLY);
    assert(comm_hw_can_try_send_reply_format(0x309,payload,4,true,false)==!VECTOR_CAN_CLASSIC_ONLY);
    if(VECTOR_CAN_CLASSIC_ONLY) {
        assert(adds==count && !BootHandoff_Request(3,0) && !BootHandoff_Pending());
        BootHandoff_ReplyQueued(1);
        assert(BootHandoff_Poll(2,true,true,true)==BOOT_WAIT);
    } else {
        assert(BootHandoff_Request(3,0));
        assert(BootHandoff_Poll(0,false,false,false)==BOOT_STOP);
        assert(BootHandoff_Poll(1,true,true,false)==BOOT_REPLY_OK);
        BootHandoff_ReplyQueued(2);
        assert(BootHandoff_Poll(3,true,true,true)==BOOT_RESET);
    }
    assert(stops>0);
    printf("PASS real APP startup/reinit/bitrate/TX/BOOT: classic_only=%d\n",VECTOR_CAN_CLASSIC_ONLY);
    return 0;
}
