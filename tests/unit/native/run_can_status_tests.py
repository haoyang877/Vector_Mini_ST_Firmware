"""CAN 状态编解码、调度、电机状态源与适配器的原生测试。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cc", required=True)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    common = [
        "#include <assert.h>",
        "#include <stdint.h>",
        "#include <stdbool.h>",
        "#include <stddef.h>",
        "#include <string.h>",
        "#include <stdio.h>",
        "#include <math.h>",
    ]
    source = (ROOT / "firmware/communication/can/interface_can.c").read_text(encoding="utf-8")
    wire = (ROOT / "firmware/communication/protocol/can_parameter_wire.c").read_text(
        encoding="utf-8"
    )
    transport_source = (ROOT / "firmware/communication/can/can_transport.c").read_text(
        encoding="utf-8"
    )
    binding_source = (ROOT / "firmware/communication/can/can_command_binding.c").read_text(
        encoding="utf-8"
    )
    queries_source = (ROOT / "firmware/communication/can/can_binding_queries.c").read_text(
        encoding="utf-8"
    )
    status_source = (ROOT / "firmware/communication/can/can_status_source.c").read_text(
        encoding="utf-8"
    )
    sampling = (
        "\n".join(common)
        + r"""
#include "firmware/services/telemetry/motor_status.h"
#include <math.h>
#define Position_Mode 3
#define Position_Impedance_Mode 18
#define Speed_Mode 2
static struct { unsigned ErrorNow, ModeNow; float posRef,speedRef,iqRef,posShadow,speedShadow,pos_vel_filtered,pos_trajectory_speed_rad_s; } MotorControl;
typedef struct {float theta_mech,vel_mech;} EncoderTelemetry_TypeDef;
static EncoderTelemetry_TypeDef OnBoard_Encoder;
static float Encoder_GetMecPos(const EncoderTelemetry_TypeDef *e) { return e->theta_mech; }
static float Encoder_GetMecVel(const EncoderTelemetry_TypeDef *e) { return e->vel_mech; }
static struct {float Iq,temp,Vbus_filt,Ibus_filt;} FOC;
"""
        + function_source(status_source, "CanStatus_BuildSnapshot")
        + r"""
int main(void) {
 MotorStatus s;
 MotorControl.ErrorNow=7; MotorControl.ModeNow=3;
 MotorControl.posRef=1.25f; MotorControl.posShadow=.75f;
 MotorControl.pos_trajectory_speed_rad_s=-.5f;
 MotorControl.speedRef=2.5f; MotorControl.speedShadow=9.f;
 MotorControl.iqRef=1.5f; FOC.Iq=1.25f; FOC.temp=65.25f; FOC.Vbus_filt=48.75f; FOC.Ibus_filt=-.375f;
 OnBoard_Encoder.theta_mech=-1.5f; OnBoard_Encoder.vel_mech=-4.f;
 MotorControl.pos_vel_filtered=-2.f;
 CanStatus_BuildSnapshot(&s);
 assert(s.fault==7 && s.mode==3 && s.position_target==1.25f && s.position_planned==.75f);
 assert(s.speed_target==2.5f && s.speed_planned==-.5f && s.speed_feedback==-2.f);
 assert(s.position_feedback==-1.5f && s.current_reference==1.5f && s.current_feedback==1.25f);
 assert(s.temperature==65.25f && s.bus_voltage==48.75f);
 assert(s.bus_current==-.375f && s.bus_current!=s.current_feedback);
 MotorControl.ModeNow=2; CanStatus_BuildSnapshot(&s);
 assert(s.speed_feedback==-4.f && s.speed_planned==9.f);
 puts("PASS actual motor status source: target vs planned, filtered speed, current, temperature, voltage and fault");return 0;
}
"""
    )
    source = (ROOT / "firmware/communication/can/interface_can.c").read_text(encoding="utf-8")
    rx = (
        "\n".join(common)
        + r"""
#include "firmware/platform/api/comm_hw.h"
#include "firmware/communication/protocol/can_motor_status.h"
#include "can_transport.h"
#include "cogging_calibration.h"
typedef unsigned CAN_PARAM_ID;
typedef enum {CAN_VALUE_FLOAT32,CAN_VALUE_MILLI_I32,CAN_VALUE_CENTI_I32,CAN_VALUE_MILLI_I16} CanValueEncoding;
#define CAN_SET_CURRENT 0x02
#define CAN_SET_SPEED 0x04
#define CAN_SET_POS 0x06
#define CAN_SET_CURRENT_CAL 0x0e
#define CAN_SET_CURRENT_LIMIT 0x10
#define CAN_SET_SPEED_LIMIT 0x12
#define CAN_SET_SPEED_ACC 0x14
#define CAN_SET_SPEED_DEC 0x16
#define CAN_SET_POS_ACC 0x1c
#define CAN_SET_POS_DEC 0x1e
#define CAN_SET_POS_MAXSPEED 0x20
#define CAN_GET_COGGING_POINT 0x6B
static struct { unsigned node_id,can_hb_count; bool can_rx_en; unsigned rx_param_id; float rx_data; } CANMsg;
static CommHwCanFrame incoming;
static bool rx_ok=true;
static bool comm_hw_can_receive_stub(CommHwCanFrame *f) {*f=incoming;return rx_ok;}
#define comm_hw_can_receive comm_hw_can_receive_stub
static float IntBitToFloat(uint32_t i) {float f;memcpy(&f,&i,4);return f;}
"""
        + function_source(wire, "CanParamWire_CommandEncoding")
        + function_source(wire, "CanParamWire_Length")
        + function_source(transport_source, "CanTransport_ReceiveFrame")
        + "/* CAN_RING_BEGIN\n * ---- 定长环实现：RX 单生产者/单消费者，TX 单上下文 ---- */\n\ntypedef struct\n{\n    volatile uint16_t head;\n    volatile uint16_t tail;\n    CanQueuedCommand_TypeDef items[CAN_RX_RING_CAPACITY];\n} CanRxRing_TypeDef;\n\ntypedef struct\n{\n    volatile uint16_t head;\n    volatile uint16_t tail;\n    CanTxReply_TypeDef items[CAN_TX_RING_CAPACITY];\n} CanTxRing_TypeDef;\n\nstatic CanRxRing_TypeDef rx_ring;\nstatic CanTxRing_TypeDef tx_ring;\nstatic volatile uint32_t rx_drop_count;\nstatic volatile uint32_t tx_drop_count;\n\nbool CanTransport_PushRxCommand(uint8_t param_id, float data)\n{\n    if ((uint16_t)(rx_ring.head - rx_ring.tail) >= CAN_RX_RING_CAPACITY)\n    {\n        ++rx_drop_count;\n        return false;\n    }\n    /* 先写负载、后推进 head：消费者只读取 head 之下的槽位，中断抢占读取端安全。 */\n    rx_ring.items[rx_ring.head % CAN_RX_RING_CAPACITY].param_id = param_id;\n    rx_ring.items[rx_ring.head % CAN_RX_RING_CAPACITY].data = data;\n    ++rx_ring.head;\n    return true;\n}\n\nbool CanTransport_PopRxCommand(CanQueuedCommand_TypeDef *command)\n{\n    if (command == NULL || rx_ring.tail == rx_ring.head)\n    {\n        return false;\n    }\n    *command = rx_ring.items[rx_ring.tail % CAN_RX_RING_CAPACITY];\n    ++rx_ring.tail;\n    return true;\n}\n\nbool CanTransport_PushTxReply(const CanTxReply_TypeDef *reply)\n{\n    if (reply == NULL)\n    {\n        return false;\n    }\n    if ((uint16_t)(tx_ring.head - tx_ring.tail) >= CAN_TX_RING_CAPACITY)\n    {\n        ++tx_drop_count;\n        return false;\n    }\n    tx_ring.items[tx_ring.head % CAN_TX_RING_CAPACITY] = *reply;\n    ++tx_ring.head;\n    return true;\n}\n\nbool CanTransport_PopTxReply(CanTxReply_TypeDef *reply)\n{\n    if (reply == NULL || tx_ring.tail == tx_ring.head)\n    {\n        return false;\n    }\n    *reply = tx_ring.items[tx_ring.tail % CAN_TX_RING_CAPACITY];\n    ++tx_ring.tail;\n    return true;\n}\n\nbool CanTransport_TxPending(void)\n{\n    return tx_ring.tail != tx_ring.head;\n}\n\nuint32_t CanTransport_GetRxDropCount(void)\n{\n    return rx_drop_count;\n}\n\nuint32_t CanTransport_GetTxDropCount(void)\n{\n    return tx_drop_count;\n}\n/* CAN_RING_END */\n"
        + function_source(binding_source, "CanCommand_CoggingIndexInvalid")
        + function_source(binding_source, "CANRxIRQHandler")
        + r"""
static void expect_queued(unsigned param,float value) {
    CanQueuedCommand_TypeDef item;
    assert(CanTransport_PopRxCommand(&item));
    assert(item.param_id==param && item.data==value);
}
static void expect_empty(void) {
    CanQueuedCommand_TypeDef item;
    assert(!CanTransport_PopRxCommand(&item));
}
int main(void) {
 CANMsg.node_id=4;CANMsg.can_hb_count=123;
 incoming.identifier=0x464;incoming.length=4;incoming.data[0]=0x41;incoming.data[1]=0xa0;
 CANRxIRQHandler();assert(CANMsg.can_hb_count==0);expect_queued(0x64,20.f);
 incoming.identifier=0x402;incoming.length=2;incoming.data[0]=0xfb;incoming.data[1]=0x1e;
 CANRxIRQHandler();expect_queued(2,-1.25f);
 incoming.identifier=0x404;incoming.length=4;incoming.data[0]=0xff;incoming.data[1]=0xff;incoming.data[2]=0xfd;incoming.data[3]=0x8c;
 CANRxIRQHandler();expect_queued(4,-6.28f);
 /* Sentinels, lengths and frame classes stay rejected with heartbeat untouched. */
 incoming.data[0]=0x80;incoming.data[1]=incoming.data[2]=incoming.data[3]=0;CANMsg.can_hb_count=123;
 CANRxIRQHandler();assert(CANMsg.can_hb_count==123);expect_empty();
 incoming.identifier=0x402;incoming.length=2;incoming.data[0]=0x80;incoming.data[1]=0;CANMsg.can_hb_count=123;
 CANRxIRQHandler();assert(CANMsg.can_hb_count==123);expect_empty();
 incoming.identifier=0x464;
 for(unsigned n=0;n<=64;++n) if(n!=4) {
  incoming.length=n;CANMsg.can_hb_count=123;CANRxIRQHandler();
  assert(CANMsg.can_hb_count==123);
 }
 expect_empty();
 incoming.length=4;incoming.extended=true;CANRxIRQHandler();
 incoming.extended=false;incoming.remote=true;CANRxIRQHandler();
 incoming.remote=false;incoming.identifier=0x364;CANRxIRQHandler();
 CANMsg.node_id=7;incoming.identifier=0x7f4;CANRxIRQHandler();assert(CANMsg.can_hb_count==123);
 incoming.length=48;CANRxIRQHandler();
 expect_empty();
 /* Out-of-range cogging index is rejected before enqueue, so it never takes a slot. */
 CANMsg.node_id=4;incoming.identifier=0x46b;incoming.length=4;
 { float v=2048.f;uint32_t bits;memcpy(&bits,&v,4);
   incoming.data[0]=(uint8_t)(bits>>24);incoming.data[1]=(uint8_t)(bits>>16);
   incoming.data[2]=(uint8_t)(bits>>8);incoming.data[3]=(uint8_t)bits; }
 CANRxIRQHandler();expect_empty();
 /* Multiple frames keep FIFO order (0x02 is milliamp i16: 0x0100 reads 0.256 A). */
 for(unsigned n=0;n<3;++n) {
  incoming.identifier=0x402;incoming.length=2;incoming.data[0]=(uint8_t)(n+1);incoming.data[1]=0;
  CANRxIRQHandler();
 }
 for(unsigned n=0;n<3;++n) expect_queued(2,(float)((n+1)<<8)/1000.0f);
 expect_empty();
 /* Ring full: newest is dropped with a counter, queued items stay in order. */
 for(unsigned n=0;n<CAN_RX_RING_CAPACITY;++n)
  assert(CanTransport_PushRxCommand((uint8_t)(0x40U+n),(float)n));
 assert(!CanTransport_PushRxCommand(0x7f,1.f));
 assert(CanTransport_GetRxDropCount()==1U);
 for(unsigned n=0;n<CAN_RX_RING_CAPACITY;++n) expect_queued(0x40U+n,(float)n);
 expect_empty();
 puts("PASS actual CAN RX: heartbeat refresh, decode, guards, index rejection, FIFO order and ring-full drop count");return 0;
}
"""
    )
    header = r"""
#ifndef TEST_FDCAN_H
#define TEST_FDCAN_H
#include <stdint.h>
#include <stddef.h>
#define HAL_OK 0
#define FDCAN_STANDARD_ID 0
#define FDCAN_DATA_FRAME 0
#define FDCAN_FD_CAN 1
#define FDCAN_BRS_ON 1
#define FDCAN_NO_TX_EVENTS 0
#define FDCAN_RX_FIFO0 0
#define FDCAN_TX_QUEUE_OPERATION 1
#define FDCAN_TX_BUFFER0 1
#define FDCAN_TX_BUFFER1 2
#define FDCAN_TX_BUFFER2 4
#define FDCAN_DLC_BYTES_48 14
#define FDCAN_FILTER_RANGE 1
#define FDCAN_FILTER_TO_RXFIFO0 2
#define FDCAN_REJECT 2
#define FDCAN_FILTER_REMOTE 1
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 8U
/* Match all fields/order of the vendor header: omitted fields hid stack junk. */
typedef struct {unsigned Identifier,IdType,TxFrameType,DataLength,ErrorStateIndicator,BitRateSwitch,FDFormat,TxEventFifoControl,MessageMarker;} FDCAN_TxHeaderTypeDef;
typedef struct {unsigned Identifier,IdType,RxFrameType,DataLength;} FDCAN_RxHeaderTypeDef;
typedef struct {unsigned IdType,FilterIndex,FilterType,FilterConfig,FilterID1,FilterID2;} FDCAN_FilterTypeDef;
typedef struct {unsigned PSR,CCCR;} FDCAN_GlobalTypeDef;
#define FDCAN_PSR_BO 0x80U
#define FDCAN_CCCR_INIT 0x1U
typedef struct {struct {unsigned TxFifoQueueMode,NominalPrescaler,DataPrescaler;} Init; FDCAN_GlobalTypeDef *Instance;} Handle;
extern Handle hfdcan1;
unsigned HAL_FDCAN_IsTxBufferMessagePending(Handle *h,unsigned mask);
unsigned HAL_FDCAN_AddMessageToTxFifoQ(Handle *h,const FDCAN_TxHeaderTypeDef *hdr,const uint8_t *d);
unsigned HAL_FDCAN_GetRxMessage(Handle *h,unsigned fifo,FDCAN_RxHeaderTypeDef *hdr,uint8_t *d);
unsigned HAL_FDCAN_ConfigFilter(Handle *h,const FDCAN_FilterTypeDef *f);
unsigned HAL_FDCAN_ConfigGlobalFilter(Handle *h,unsigned a,unsigned b,unsigned c,unsigned d);
unsigned HAL_FDCAN_Start(Handle *h);
unsigned HAL_FDCAN_ActivateNotification(Handle *h,unsigned m,unsigned x);
unsigned HAL_FDCAN_Stop(Handle *h);
unsigned HAL_FDCAN_Init(Handle *h);
#endif
"""
    (out / "fdcan.h").write_text(header)
    (out / "main.h").write_text("void Error_Handler(void);\n")
    port = (
        "\n".join(common)
        + r"""
#include "fdcan.h"
#include "firmware/platform/api/comm_hw.h"
static FDCAN_GlobalTypeDef regs;
Handle hfdcan1 = {.Instance = &regs};
static unsigned pending,tx_calls,tx_result,rx_length=14;
static FDCAN_TxHeaderTypeDef last_tx_header;
static uint8_t last_tx_data0;
static FDCAN_FilterTypeDef last_filter;
static unsigned last_global[4];
static unsigned filter_calls,global_calls,start_calls,notify_calls,stop_calls,init_calls;
static unsigned last_notify_mask;
void Error_Handler(void) {assert(0);}
unsigned HAL_FDCAN_IsTxBufferMessagePending(Handle *h,unsigned mask) {(void)h;assert(mask==7);return (pending&mask)!=0;}
unsigned HAL_FDCAN_AddMessageToTxFifoQ(Handle *h,const FDCAN_TxHeaderTypeDef *hdr,const uint8_t *d) {
 (void)h;last_tx_header=*hdr;last_tx_data0=d[0];++tx_calls;return tx_result;
}
unsigned HAL_FDCAN_GetRxMessage(Handle *h,unsigned fifo,FDCAN_RxHeaderTypeDef *hdr,uint8_t *d) {
 (void)h;(void)fifo;memset(hdr,0,sizeof(*hdr));hdr->DataLength=rx_length;hdr->Identifier=0x7f4;
 memset(d,0xaa,64);return 0;
}
unsigned HAL_FDCAN_ConfigFilter(Handle *h,const FDCAN_FilterTypeDef *f) {(void)h;last_filter=*f;++filter_calls;return 0;}
unsigned HAL_FDCAN_ConfigGlobalFilter(Handle *h,unsigned a,unsigned b,unsigned c,unsigned d) {
 (void)h;last_global[0]=a;last_global[1]=b;last_global[2]=c;last_global[3]=d;++global_calls;return 0;
}
unsigned HAL_FDCAN_Start(Handle *h) {(void)h;++start_calls;return 0;}
unsigned HAL_FDCAN_ActivateNotification(Handle *h,unsigned m,unsigned x) {(void)h;(void)x;last_notify_mask=m;++notify_calls;return 0;}
unsigned HAL_FDCAN_Stop(Handle *h) {(void)h;++stop_calls;return 0;}
unsigned HAL_FDCAN_Init(Handle *h) {(void)h;++init_calls;return 0;}
int main(void) {
 uint8_t data[48]={0xab};struct {CommHwCanFrame f;uint32_t guard;} bounded;
 hfdcan1.Init.TxFifoQueueMode=1;
 for(pending=1;pending<=7;++pending) assert(!comm_hw_can_try_send_status(0x7f4,data,48));
 pending=0; /* Bench regression: empty queue has TFFL=0, TXBRP=0. */
 assert(tx_calls==0);assert(comm_hw_can_try_send_status(0x7f4,data,48));assert(tx_calls==1);
 assert(last_tx_header.Identifier==0x7f4 && last_tx_header.DataLength==14 && last_tx_header.FDFormat==1);
 assert(last_tx_header.BitRateSwitch==1 && last_tx_header.ErrorStateIndicator==0 && last_tx_header.MessageMarker==0);
 assert(last_tx_data0==0xab);
 tx_result=1;assert(!comm_hw_can_try_send_status(0x7f4,data,48));assert(tx_calls==2);
 assert(!comm_hw_can_try_send_status(0x7f4,data,47));assert(!comm_hw_can_try_send_status(0x800,data,48));
 hfdcan1.Init.TxFifoQueueMode=0;assert(!comm_hw_can_try_send_status(0x7f4,data,48));assert(tx_calls==2);
 bounded.guard=0x12345678;assert(comm_hw_can_receive(&bounded.f));
 assert(bounded.f.length==48 && bounded.guard==0x12345678);
 rx_length=15;assert(comm_hw_can_receive(&bounded.f));assert(bounded.f.length==64 && bounded.guard==0x12345678);
 tx_result=0;
 comm_hw_can_start(4);
 assert(filter_calls==1 && global_calls==1 && start_calls==1 && notify_calls==1);
 assert(last_filter.IdType==FDCAN_STANDARD_ID && last_filter.FilterIndex==0);
 assert(last_filter.FilterType==FDCAN_FILTER_RANGE && last_filter.FilterConfig==FDCAN_FILTER_TO_RXFIFO0);
 assert(last_filter.FilterID1==0x400 && last_filter.FilterID2==0x4FF);
 assert(last_global[0]==FDCAN_REJECT && last_global[1]==FDCAN_REJECT);
 assert(last_global[2]==FDCAN_FILTER_REMOTE && last_global[3]==FDCAN_FILTER_REMOTE);
 assert(last_notify_mask==FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
 comm_hw_can_set_baudrate(1000);
 assert(stop_calls==1 && init_calls==1 && start_calls==2);
 assert(hfdcan1.Init.DataPrescaler==10 && hfdcan1.Init.NominalPrescaler==10);
 comm_hw_can_set_baudrate(2000);
 assert(stop_calls==2 && init_calls==2 && start_calls==3);
 assert(hfdcan1.Init.DataPrescaler==5 && hfdcan1.Init.NominalPrescaler==10);
 {
  uint8_t reply[4]={0x11,0x22,0x33,0x44};
  assert(comm_hw_can_try_send_reply(0x465,reply,4));assert(tx_calls==3);
  assert(last_tx_header.Identifier==0x465 && last_tx_header.IdType==FDCAN_STANDARD_ID);
  assert(last_tx_header.DataLength==4 && last_tx_header.FDFormat==FDCAN_FD_CAN);
  assert(last_tx_header.BitRateSwitch==FDCAN_BRS_ON && last_tx_header.TxFrameType==FDCAN_DATA_FRAME);
  assert(last_tx_header.TxEventFifoControl==FDCAN_NO_TX_EVENTS && last_tx_data0==0x11);
  tx_result=1;assert(!comm_hw_can_try_send_reply(0x465,reply,2));assert(tx_calls==4);
 }
 /* bus-off 自恢复：健康不动作；PSR.BO 或 CCCR.INIT 置位时清 INIT 重回总线。 */
 regs.PSR=0;regs.CCCR=0;
 assert(!comm_hw_can_service_bus_off() && start_calls==3);
 regs.PSR=FDCAN_PSR_BO;
 assert(comm_hw_can_service_bus_off() && start_calls==4);
 regs.PSR=0;regs.CCCR=FDCAN_CCCR_INIT;
 assert(comm_hw_can_service_bus_off() && start_calls==5);
 puts("PASS actual HAL port: queue discipline, node filter start, baudrate switch and reply header");return 0;
}
"""
    )
    priority = (
        "\n".join(common)
        + r"""
#include "firmware/platform/api/comm_hw.h"
#include "firmware/communication/protocol/can_motor_status.h"
#include "can_transport.h"
static struct {unsigned node_id;} CANMsg;
static unsigned prepared,status_sent,replies;
static uint16_t last_reply_id;
static uint8_t last_reply_len;
static uint32_t time_hw_now_ms(void) {return 100;}
bool CanMotorStatus_Prepare(uint32_t t,uint8_t n,uint16_t *id,uint8_t *d,size_t cap) {
 assert(t==100 && n==4 && cap==48);*id=0x7f4;memset(d,0,48);++prepared;return true;
}
bool comm_hw_can_try_send_status(uint16_t id,const uint8_t *d,size_t len) {
 (void)d;assert(id==0x7f4 && len==48);++status_sent;return true;
}
bool comm_hw_can_try_send_reply(uint16_t id,const uint8_t *d,uint8_t len) {
 (void)d;++replies;last_reply_id=id;last_reply_len=len;return true;
}
static void CanStatus_BuildSnapshot(MotorStatus *sample) {(void)sample;}
static void CanCommand_ServiceRx(void) {}
typedef unsigned CAN_PARAM_ID;
"""
        + function_source(wire, "CanParamWire_Identifier")
        + function_source(transport_source, "CanTransport_SendReply")
        + function_source(transport_source, "CanTransport_TrySendStatus")
        + "/* CAN_RING_BEGIN\n * ---- 定长环实现：RX 单生产者/单消费者，TX 单上下文 ---- */\n\ntypedef struct\n{\n    volatile uint16_t head;\n    volatile uint16_t tail;\n    CanQueuedCommand_TypeDef items[CAN_RX_RING_CAPACITY];\n} CanRxRing_TypeDef;\n\ntypedef struct\n{\n    volatile uint16_t head;\n    volatile uint16_t tail;\n    CanTxReply_TypeDef items[CAN_TX_RING_CAPACITY];\n} CanTxRing_TypeDef;\n\nstatic CanRxRing_TypeDef rx_ring;\nstatic CanTxRing_TypeDef tx_ring;\nstatic volatile uint32_t rx_drop_count;\nstatic volatile uint32_t tx_drop_count;\n\nbool CanTransport_PushRxCommand(uint8_t param_id, float data)\n{\n    if ((uint16_t)(rx_ring.head - rx_ring.tail) >= CAN_RX_RING_CAPACITY)\n    {\n        ++rx_drop_count;\n        return false;\n    }\n    /* 先写负载、后推进 head：消费者只读取 head 之下的槽位，中断抢占读取端安全。 */\n    rx_ring.items[rx_ring.head % CAN_RX_RING_CAPACITY].param_id = param_id;\n    rx_ring.items[rx_ring.head % CAN_RX_RING_CAPACITY].data = data;\n    ++rx_ring.head;\n    return true;\n}\n\nbool CanTransport_PopRxCommand(CanQueuedCommand_TypeDef *command)\n{\n    if (command == NULL || rx_ring.tail == rx_ring.head)\n    {\n        return false;\n    }\n    *command = rx_ring.items[rx_ring.tail % CAN_RX_RING_CAPACITY];\n    ++rx_ring.tail;\n    return true;\n}\n\nbool CanTransport_PushTxReply(const CanTxReply_TypeDef *reply)\n{\n    if (reply == NULL)\n    {\n        return false;\n    }\n    if ((uint16_t)(tx_ring.head - tx_ring.tail) >= CAN_TX_RING_CAPACITY)\n    {\n        ++tx_drop_count;\n        return false;\n    }\n    tx_ring.items[tx_ring.head % CAN_TX_RING_CAPACITY] = *reply;\n    ++tx_ring.head;\n    return true;\n}\n\nbool CanTransport_PopTxReply(CanTxReply_TypeDef *reply)\n{\n    if (reply == NULL || tx_ring.tail == tx_ring.head)\n    {\n        return false;\n    }\n    *reply = tx_ring.items[tx_ring.tail % CAN_TX_RING_CAPACITY];\n    ++tx_ring.tail;\n    return true;\n}\n\nbool CanTransport_TxPending(void)\n{\n    return tx_ring.tail != tx_ring.head;\n}\n\nuint32_t CanTransport_GetRxDropCount(void)\n{\n    return rx_drop_count;\n}\n\nuint32_t CanTransport_GetTxDropCount(void)\n{\n    return tx_drop_count;\n}\n/* CAN_RING_END */\n"
        + function_source(source, "CAN_Service")
        + r"""
int main(void) {
 CanTxReply_TypeDef reply={0x65,{1,2,3,4},4};
 CANMsg.node_id=4;
 /* Queue idle: the status stream is submitted once. */
 CAN_Service();assert(prepared==1 && status_sent==1 && replies==0);
 /* Burst replies: both go out FIFO and the status stream is deferred for this tick. */
 assert(CanTransport_PushTxReply(&reply));
 reply.param_id=0x66;assert(CanTransport_PushTxReply(&reply));
 CAN_Service();
 assert(replies==2 && last_reply_id==0x466 && last_reply_len==4 && prepared==1);
 assert(!CanTransport_TxPending());
 /* A tick with a reply never submits the status stream; the next idle tick does. */
 assert(CanTransport_PushTxReply(&reply));
 CAN_Service();assert(replies==3 && prepared==1 && status_sent==1);
 CAN_Service();assert(prepared==2 && status_sent==2);
 /* TX ring full: newest dropped with a counter, queued items stay. */
 { uint32_t drops=CanTransport_GetTxDropCount();
   for(unsigned n=0;n<CAN_TX_RING_CAPACITY;++n) assert(CanTransport_PushTxReply(&reply));
   assert(!CanTransport_PushTxReply(&reply));
   assert(CanTransport_GetTxDropCount()==drops+1U);
   CanTxReply_TypeDef out;
   for(unsigned n=0;n<CAN_TX_RING_CAPACITY;++n) assert(CanTransport_PopTxReply(&out));
   assert(!CanTransport_TxPending());
 }
 puts("PASS send service: replies FIFO first, status deferred while pending, TX ring drop counter");return 0;
}
"""
    )
    reply = (
        "\n".join(common)
        + r"""
#include <limits.h>
#include <math.h>
#include "can_transport.h"
typedef unsigned CAN_PARAM_ID;
typedef enum {CAN_VALUE_FLOAT32,CAN_VALUE_MILLI_I32,CAN_VALUE_CENTI_I32,CAN_VALUE_MILLI_I16} CanValueEncoding;
#define CAN_GET_CURRENT_SET 0x03
#define CAN_GET_CURRENT_CAL 0x0f
#define CAN_GET_CURRENT_LIMIT 0x11
#define CAN_GET_IBUS 0x2f
#define CAN_GET_IA 0x31
#define CAN_GET_IB 0x33
#define CAN_GET_IC 0x35
#define CAN_GET_ID 0x37
#define CAN_GET_IQ 0x39
#define CAN_GET_FRICTION_COULOMB_POS 0x5b
#define CAN_GET_FRICTION_COULOMB_NEG 0x5c
#define CAN_GET_FRICTION_RMSE_POS 0x5f
#define CAN_GET_FRICTION_RMSE_NEG 0x60
#define CAN_GET_SPEED_SET 0x05
#define CAN_GET_SPEED_LIMIT 0x13
#define CAN_GET_SPEED_ACC 0x15
#define CAN_GET_SPEED_DEC 0x17
#define CAN_GET_POS_ACC 0x1d
#define CAN_GET_POS_DEC 0x1f
#define CAN_GET_POS_MAXSPEED 0x21
#define CAN_GET_SPEED2_FILT 0x3f
#define CAN_GET_POS_SET 0x07
#define CAN_GET_POS2_FILT 0x41
static uint32_t FloatToIntBit(float value) {uint32_t bits;memcpy(&bits,&value,4);return bits;}
"""
        + function_source(wire, "CanParamWire_ReplyEncoding")
        + function_source(wire, "CanParamWire_Milli32")
        + function_source(wire, "CanParamWire_Centi32")
        + function_source(wire, "CanParamWire_Milli16")
        + function_source(wire, "CanParamWire_Length")
        + "/* CAN_RING_BEGIN\n * ---- 定长环实现：RX 单生产者/单消费者，TX 单上下文 ---- */\n\ntypedef struct\n{\n    volatile uint16_t head;\n    volatile uint16_t tail;\n    CanQueuedCommand_TypeDef items[CAN_RX_RING_CAPACITY];\n} CanRxRing_TypeDef;\n\ntypedef struct\n{\n    volatile uint16_t head;\n    volatile uint16_t tail;\n    CanTxReply_TypeDef items[CAN_TX_RING_CAPACITY];\n} CanTxRing_TypeDef;\n\nstatic CanRxRing_TypeDef rx_ring;\nstatic CanTxRing_TypeDef tx_ring;\nstatic volatile uint32_t rx_drop_count;\nstatic volatile uint32_t tx_drop_count;\n\nbool CanTransport_PushRxCommand(uint8_t param_id, float data)\n{\n    if ((uint16_t)(rx_ring.head - rx_ring.tail) >= CAN_RX_RING_CAPACITY)\n    {\n        ++rx_drop_count;\n        return false;\n    }\n    /* 先写负载、后推进 head：消费者只读取 head 之下的槽位，中断抢占读取端安全。 */\n    rx_ring.items[rx_ring.head % CAN_RX_RING_CAPACITY].param_id = param_id;\n    rx_ring.items[rx_ring.head % CAN_RX_RING_CAPACITY].data = data;\n    ++rx_ring.head;\n    return true;\n}\n\nbool CanTransport_PopRxCommand(CanQueuedCommand_TypeDef *command)\n{\n    if (command == NULL || rx_ring.tail == rx_ring.head)\n    {\n        return false;\n    }\n    *command = rx_ring.items[rx_ring.tail % CAN_RX_RING_CAPACITY];\n    ++rx_ring.tail;\n    return true;\n}\n\nbool CanTransport_PushTxReply(const CanTxReply_TypeDef *reply)\n{\n    if (reply == NULL)\n    {\n        return false;\n    }\n    if ((uint16_t)(tx_ring.head - tx_ring.tail) >= CAN_TX_RING_CAPACITY)\n    {\n        ++tx_drop_count;\n        return false;\n    }\n    tx_ring.items[tx_ring.head % CAN_TX_RING_CAPACITY] = *reply;\n    ++tx_ring.head;\n    return true;\n}\n\nbool CanTransport_PopTxReply(CanTxReply_TypeDef *reply)\n{\n    if (reply == NULL || tx_ring.tail == tx_ring.head)\n    {\n        return false;\n    }\n    *reply = tx_ring.items[tx_ring.tail % CAN_TX_RING_CAPACITY];\n    ++tx_ring.tail;\n    return true;\n}\n\nbool CanTransport_TxPending(void)\n{\n    return tx_ring.tail != tx_ring.head;\n}\n\nuint32_t CanTransport_GetRxDropCount(void)\n{\n    return rx_drop_count;\n}\n\nuint32_t CanTransport_GetTxDropCount(void)\n{\n    return tx_drop_count;\n}\n/* CAN_RING_END */\n"
        + function_source(source, "CAN_SendMessage_Update")
        + r"""
static void expect_reply(unsigned id,unsigned len,unsigned b0,unsigned b1,unsigned b2,unsigned b3) {
 CanTxReply_TypeDef r;
 assert(CanTransport_PopTxReply(&r));
 assert(r.param_id==id && r.length==len);
 assert(r.data[0]==b0 && r.data[1]==b1 && r.data[2]==b2 && r.data[3]==b3);
}
int main(void) {
 CAN_SendMessage_Update(0x05,6.283f);
 expect_reply(0x05,4,0,0,2,0x74);
 CAN_SendMessage_Update(0x07,-1.25f);
 expect_reply(0x07,4,0xff,0xff,0xfb,0x1e);
 CAN_SendMessage_Update(0x03,-1.25f);
 expect_reply(0x03,2,0xfb,0x1e,0,0);
 CAN_SendMessage_Update(0x15,6.283f);
 expect_reply(0x15,4,0,0,2,0x74);
 { const unsigned ids[]={0x15,0x17,0x1d,0x1f,0x21};
   for(unsigned i=0;i<sizeof(ids)/sizeof(ids[0]);++i) {
    CAN_SendMessage_Update(ids[i],0.785398163f);
    expect_reply(ids[i],4,0,0,0,0x4e);
   }
 }
 CAN_SendMessage_Update(0x11,6.f);expect_reply(0x11,2,0x17,0x70,0,0);
 CAN_SendMessage_Update(0x67,2.f);expect_reply(0x67,4,0x40,0,0,0);
 assert(!CanTransport_TxPending());
 puts("PASS actual CAN reply encoding: centi speed/acceleration, milli position and milliamp current");return 0;
}
"""
    )
    command = (
        "\n".join(common)
        + r"""
#include <math.h>
#include "cogging_calibration.h"
#include "firmware/communication/protocol/can_motor_status.h"
typedef unsigned CAN_PARAM_ID;
#define CAN_SET_STATUS_STREAM 0x64
#define CAN_GET_STATUS_STREAM 0x65
#define CAN_GET_PROTOCOL_REVISION 0x67
#define CAN_GET_COGGING_POINT 0x6B
#include "firmware/communication/protocol/can_parameter_format.h"
static float reply;
static unsigned replies;
static unsigned reply_id;
static void CAN_SendMessage_Update(unsigned id,float value) {assert(id==0x65 || id==0x67);reply_id=id;reply=value;++replies;}
"""
        + function_source(binding_source, "CAN_ReceiveMessage_Update").split(
            "/* Validate table indexes"
        )[0]
        + "(void)data_int;(void)primask;}\n"
        + r"""
#include <math.h>
int main(void) {
 CanMotorStatus_Init();CAN_ReceiveMessage_Update(0x64,1);assert(reply==20);
 CAN_ReceiveMessage_Update(0x64,200);assert(reply==200);
 CAN_ReceiveMessage_Update(0x64,19.5f);assert(reply==-1 && CanMotorStatus_Rate()==200);
 CAN_ReceiveMessage_Update(0x64,NAN);assert(reply==-1 && CanMotorStatus_Rate()==200);
 CAN_ReceiveMessage_Update(0x64,0);assert(reply==0);
 CAN_ReceiveMessage_Update(0x65,0);assert(reply==0);
 CAN_ReceiveMessage_Update(0x64,1);assert(reply==200 && replies==7);
 CAN_ReceiveMessage_Update(0x67,0);assert(reply_id==0x67 && reply==2 && CanMotorStatus_Rate()==200);
 puts("PASS actual command adapter: start/rate/stop/query ACK and invalid input NAK");return 0;
}
"""
    )
    heartbeat = (
        "\n".join(common)
        + r"""
#define Current_Mode 1
#define Speed_Mode 2
#define Position_Mode 3
#define Position_Impedance_Mode 18
#define CAN_DisConnect 4
#define No_Error 0
static struct {bool can_hb_en,can_rx_en;uint32_t can_hb_set,can_hb_count;} CANMsg;
static struct {unsigned ModeNow,ErrorNow;} MotorControl;
static void Set_ErrorNow(unsigned error) {MotorControl.ErrorNow=error;}
/* 与 platform/api/control_config.h 的 SUPERVISOR_FREQ=2 kHz 契约一致。 */
#define SUPERVISOR_TICKS_PER_MS 2U
"""
        + function_source(status_source, "CanStatus_HeartbeatArmed")
        + function_source(source, "CAN_DisConnect_Handle")
        + r"""
int main(void) {
 const unsigned modes[]={1,2,3,18};
 for(unsigned i=0;i<4;++i) {
  MotorControl.ModeNow=modes[i];MotorControl.ErrorNow=0;
  CANMsg.can_rx_en=true;CANMsg.can_hb_set=500;CANMsg.can_hb_count=0;
  for(unsigned t=0;t<999;++t) CAN_DisConnect_Handle();
  assert(MotorControl.ErrorNow==0 && CANMsg.can_hb_count==999);
  CAN_DisConnect_Handle();assert(MotorControl.ErrorNow==4);
  CAN_DisConnect_Handle();assert(CANMsg.can_hb_count==1000);
 }
 MotorControl.ModeNow=0;MotorControl.ErrorNow=0;
 for(unsigned t=0;t<1000;++t) CAN_DisConnect_Handle();
 assert(!CANMsg.can_hb_en && CANMsg.can_hb_count==0 && MotorControl.ErrorNow==0);
 MotorControl.ModeNow=3;CAN_DisConnect_Handle();assert(CANMsg.can_hb_count==1);
 CANMsg.can_hb_count=1000;MotorControl.ErrorNow=7;
 CAN_DisConnect_Handle();assert(MotorControl.ErrorNow==7);
 CANMsg.can_hb_set=0;CAN_DisConnect_Handle();assert(!CANMsg.can_hb_en && CANMsg.can_hb_count==0);
 CANMsg.can_hb_set=UINT32_MAX;CANMsg.can_hb_count=UINT32_MAX;
 MotorControl.ErrorNow=0;CAN_DisConnect_Handle();
 assert(CANMsg.can_hb_count==UINT32_MAX && MotorControl.ErrorNow==4);
 MotorControl.ModeNow=0;CAN_DisConnect_Handle();assert(MotorControl.ErrorNow==4);
 puts("PASS heartbeat: timeout, STOP disarms, rearm, disabled, saturation, existing fault preserved");return 0;
}
"""
    )
    # Compile the real GET case bodies so a wrong reply ID cannot pass a codec-only test.
    cases = []
    for label in (
        "CAN_GET_CAN_BR",
        "CAN_GET_CAN_HB",
        "CAN_GET_TEMPERATURE_SOURCE",
        "CAN_GET_TEMPERATURE_VALID",
    ):
        case = queries_source.split("case " + label + ":", 1)[1].split("break;", 1)[0]
        cases.append("case " + label + ":" + case + "break;")
    config_get = (
        "\n".join(common)
        + r"""
#define CAN_GET_CAN_BR 0x29
#define CAN_GET_CAN_HB 0x2b
#define CAN_GET_TEMPERATURE_SOURCE 0x6e
#define CAN_GET_TEMPERATURE_VALID 0x6f
static struct {unsigned valid;} McuTemperature;
static struct {unsigned can_hb_set;} CANMsg={500};
static uint32_t CanTransport_Baudrate(void) {return 1000U;}
static unsigned reply_id,calls;
static float reply_value;
static void CAN_SendMessage_Update(unsigned id,float value)
{reply_id=id;reply_value=value;++calls;}
static void dispatch(unsigned param_id) {switch(param_id) {
"""
        + "\n".join(cases)
        + r"""
}}
int main(void) {
 for(unsigned n=0;n<5;++n) {
  dispatch(CAN_GET_CAN_BR);assert(reply_id==0x29 && reply_value==1000.f);
  dispatch(CAN_GET_CAN_HB);assert(reply_id==0x2b && reply_value==500.f);
 }
 assert(calls==10);
 dispatch(0x6e);assert(reply_id==0x6e && reply_value==1.f);
 dispatch(0x6f);assert(reply_id==0x6f && reply_value==0.f);
 McuTemperature.valid=1;dispatch(0x6f);assert(reply_id==0x6f && reply_value==1.f);
 puts("PASS production CAN baudrate/heartbeat IDs and MCU temperature source/validity");return 0;
}
"""
    )
    fixtures = [
        ("config_get", config_get, []),
        ("heartbeat", heartbeat, []),
        (
            "codec",
            ROOT / "tests/unit/can_motor_status_test.c",
            [
                ROOT / "firmware/communication/protocol/can_motor_status.c",
                ROOT / "firmware/communication/protocol/can_parameter_wire.c",
                ROOT / "firmware/services/telemetry/motor_status.c",
            ],
        ),
        ("sampling", sampling, []),
        ("rx", rx, []),
        (
            "port",
            port,
            [
                ROOT / "firmware/platform/stm32g4/ports/comm/comm_status_stm32g4.c",
                ROOT / "firmware/platform/stm32g4/ports/comm/comm_control_stm32g4.c",
            ],
        ),
        ("priority", priority, [ROOT / "firmware/services/telemetry/motor_status.c"]),
        ("reply", reply, []),
        (
            "command",
            command,
            [
                ROOT / "firmware/communication/protocol/can_motor_status.c",
                ROOT / "firmware/communication/protocol/can_parameter_wire.c",
                ROOT / "firmware/services/telemetry/motor_status.c",
            ],
        ),
    ]
    logs = []
    for name, fixture, extra in fixtures:
        if isinstance(fixture, str):
            path = out / (name + ".c")
            path.write_text(fixture)
        else:
            path = fixture
        exe = out / (name + ".exe")
        compiler = (
            [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else []) + NATIVE_INCLUDE_FLAGS
        )
        cmd = compiler + [
            "-std=c99",
            "-O2",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(out),
            "-I",
            str(ROOT),
            str(path),
            *map(str, extra),
            "-lm",
            "-o",
            str(exe),
        ]
        # Poison automatic variables to reproduce missing TX header initialization.
        # These host fixtures require a compiler supporting this Clang/GCC option.
        if name in ("priority", "port"):
            cmd.insert(len(compiler), "-ftrivial-auto-var-init=pattern")
        for command in (cmd, [str(exe)]):
            result = subprocess.run(command, capture_output=True, text=True)
            logs.append(result.stdout + result.stderr)
            print(logs[-1], end="")
            (out / "tests.log").write_text("".join(logs))
            result.check_returncode()


if __name__ == "__main__":
    main()
