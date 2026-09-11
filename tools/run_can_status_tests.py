"""Native tests of actual status codec, scheduler, motor source and CAN adapters."""
import argparse
from pathlib import Path
import subprocess
from run_position_servo_tests import function_source

ROOT=Path(__file__).resolve().parents[1]

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--cc',required=True)
    ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args();out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
    common=['#include <assert.h>','#include <stdint.h>','#include <stdbool.h>',
            '#include <stddef.h>','#include <string.h>','#include <stdio.h>']
    source=(ROOT/'Foc/foc_task.c').read_text(encoding='utf-8')
    sampling='\n'.join(common)+r'''
#include "software/services/telemetry/motor_status.h"
#include <math.h>
#define Position_Mode 3
#define Position_Impedance_Mode 18
#define Speed_Mode 2
typedef struct {float position_reference,trajectory_speed_reference;} PositionCascadeTelemetry_TypeDef;
static bool MotorOuterLoop_GetTelemetry(PositionCascadeTelemetry_TypeDef *out)
{out->position_reference=.75f;out->trajectory_speed_reference=-.5f;return true;}
static struct { unsigned ErrorNow, ModeNow; float posRef,speedRef,iqRef,posShadow,speedShadow,pos_vel_filtered; } MotorControl;
static struct {float theta_mech,vel_mech;} OnBoard_Encoder;
static struct {float Iq,temp,Vbus_filt;} FOC;
'''+function_source(source,'MotorStatus_Sampling')+r'''
int main(void) {
 MotorStatus s;
 MotorControl.ErrorNow=7; MotorControl.ModeNow=3;
 MotorControl.posRef=1.25f; MotorControl.posShadow=.75f;
 MotorControl.speedRef=2.5f; MotorControl.speedShadow=9.f;
 MotorControl.iqRef=1.5f; FOC.Iq=1.25f; FOC.temp=65.25f; FOC.Vbus_filt=48.75f;
 OnBoard_Encoder.theta_mech=-1.5f; OnBoard_Encoder.vel_mech=-4.f;
 MotorControl.pos_vel_filtered=-2.f;
 MotorStatus_Sampling(); assert(!MotorStatus_Take(&s));
 MotorStatus_Request(); MotorStatus_Sampling(); assert(MotorStatus_Take(&s));
 assert(s.fault==7 && s.mode==3 && s.position_target==1.25f && s.position_planned==.75f);
 assert(s.speed_target==2.5f && s.speed_planned==-.5f && s.speed_feedback==-2.f);
 assert(s.position_feedback==-1.5f && s.current_reference==1.5f && s.current_feedback==1.25f);
 assert(s.temperature==65.25f && s.bus_voltage==48.75f);
 MotorControl.ModeNow=2; MotorStatus_Request(); MotorStatus_Sampling();
 assert(MotorStatus_Take(&s) && s.speed_feedback==-4.f);
 puts("PASS actual motor status source: target vs planned, filtered speed, current, temperature, voltage and fault");return 0;
}
'''
    source=(ROOT/'Communication/interface_can.c').read_text(encoding='utf-8')
    rx='\n'.join(common)+r'''
#include "hal/api/comm_hw.h"
#include "software/communication/protocol/can_motor_status.h"
typedef unsigned CAN_PARAM_ID;
typedef enum {CAN_VALUE_FLOAT32,CAN_VALUE_MILLI_I32,CAN_VALUE_CENTI_I32,CAN_VALUE_MILLI_I16} CanValueEncoding;
static CanValueEncoding CAN_CommandEncoding(CAN_PARAM_ID p)
{if(p==2)return CAN_VALUE_MILLI_I16;if(p==4)return CAN_VALUE_CENTI_I32;if(p==6)return CAN_VALUE_MILLI_I32;return CAN_VALUE_FLOAT32;}
static struct { unsigned node_id,can_hb_count; bool can_rx_en; uint8_t rx_data_u8[4]; unsigned rx_param_id; float rx_data; } CANMsg;
static struct {unsigned ErrorNow;} MotorControl;
#define CAN_DisConnect 4
#define No_Error 0
static unsigned calls,last_param;
static float last_value;
static CommHwCanFrame incoming;
static bool rx_ok=true;
static bool comm_hw_can_receive_stub(CommHwCanFrame *f) {*f=incoming;return rx_ok;}
#define comm_hw_can_receive comm_hw_can_receive_stub
static void Set_ErrorNow(unsigned e) {MotorControl.ErrorNow=e;}
static float IntBitToFloat(uint32_t i) {float f;memcpy(&f,&i,4);return f;}
static void CAN_ReceiveMessage_Update(unsigned p,float f) {last_param=p;last_value=f;++calls;}
'''+function_source(source,'CANRxIRQHandler')+r'''
int main(void) {
 CANMsg.node_id=4;CANMsg.can_hb_count=123;
 incoming.identifier=0x464;incoming.length=4;incoming.data[0]=0x41;incoming.data[1]=0xa0;
 CANRxIRQHandler();assert(calls==1 && last_param==0x64 && last_value==20.f && CANMsg.can_hb_count==0);
 incoming.identifier=0x402;incoming.length=2;incoming.data[0]=0xfb;incoming.data[1]=0x1e;
 CANRxIRQHandler();assert(calls==2 && last_param==2 && last_value==-1.25f);
 incoming.identifier=0x404;incoming.length=4;incoming.data[0]=0xff;incoming.data[1]=0xff;incoming.data[2]=0xfd;incoming.data[3]=0x8c;
 CANRxIRQHandler();assert(calls==3 && last_param==4 && last_value==-6.28f);
 incoming.data[0]=0x80;incoming.data[1]=incoming.data[2]=incoming.data[3]=0;CANMsg.can_hb_count=123;
 CANRxIRQHandler();assert(calls==3 && CANMsg.can_hb_count==123);
 incoming.identifier=0x402;incoming.length=2;incoming.data[0]=0x80;incoming.data[1]=0;CANMsg.can_hb_count=123;
 CANRxIRQHandler();assert(calls==3 && CANMsg.can_hb_count==123);
 incoming.identifier=0x464;
 for(unsigned n=0;n<=64;++n) if(n!=4) {
  incoming.length=n;CANMsg.can_hb_count=123;CANRxIRQHandler();
  assert(calls==3 && CANMsg.can_hb_count==123);
 }
 incoming.length=4;incoming.extended=true;CANRxIRQHandler();assert(calls==3);
 incoming.extended=false;incoming.remote=true;CANRxIRQHandler();assert(calls==3);
 incoming.remote=false;incoming.identifier=0x364;CANRxIRQHandler();assert(calls==3);
 CANMsg.node_id=7;incoming.identifier=0x7f4;CANRxIRQHandler();assert(calls==3 && CANMsg.can_hb_count==123);
 incoming.length=48;CANRxIRQHandler();assert(calls==3);
 puts("PASS actual CAN RX: float config, milli-i16 current, centi-i32 speed, milli-i32 position, length and routing guards");return 0;
}
'''
    header=r'''
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
typedef struct {unsigned Identifier,IdType,TxFrameType,FDFormat,BitRateSwitch,DataLength,TxEventFifoControl;} FDCAN_TxHeaderTypeDef;
typedef struct {unsigned Identifier,IdType,RxFrameType,DataLength;} FDCAN_RxHeaderTypeDef;
typedef struct {struct {unsigned TxFifoQueueMode;} Init;} Handle;
extern Handle hfdcan1;
unsigned HAL_FDCAN_IsTxBufferMessagePending(Handle *h,unsigned mask);
unsigned HAL_FDCAN_AddMessageToTxFifoQ(Handle *h,const FDCAN_TxHeaderTypeDef *hdr,const uint8_t *d);
unsigned HAL_FDCAN_GetRxMessage(Handle *h,unsigned fifo,FDCAN_RxHeaderTypeDef *hdr,uint8_t *d);
#endif
'''
    (out/'fdcan.h').write_text(header)
    port='\n'.join(common)+r'''
#include "fdcan.h"
#include "hal/api/comm_hw.h"
Handle hfdcan1;
static unsigned pending,tx_calls,tx_result,rx_length=14;
unsigned HAL_FDCAN_IsTxBufferMessagePending(Handle *h,unsigned mask) {(void)h;assert(mask==7);return (pending&mask)!=0;}
unsigned HAL_FDCAN_AddMessageToTxFifoQ(Handle *h,const FDCAN_TxHeaderTypeDef *hdr,const uint8_t *d) {
 (void)h; assert(hdr->Identifier==0x7f4 && hdr->DataLength==14 && hdr->FDFormat==1 && hdr->BitRateSwitch==1);
 assert(d[0]==0xab);++tx_calls;return tx_result;
}
unsigned HAL_FDCAN_GetRxMessage(Handle *h,unsigned fifo,FDCAN_RxHeaderTypeDef *hdr,uint8_t *d) {
 (void)h;(void)fifo;memset(hdr,0,sizeof(*hdr));hdr->DataLength=rx_length;hdr->Identifier=0x7f4;
 memset(d,0xaa,64);return 0;
}
int main(void) {
 uint8_t data[48]={0xab};struct {CommHwCanFrame f;uint32_t guard;} bounded;
 hfdcan1.Init.TxFifoQueueMode=1;
 for(pending=1;pending<=7;++pending) assert(!comm_hw_can_try_send_status(0x7f4,data,48));
 pending=0; /* Bench regression: empty queue has TFFL=0, TXBRP=0. */
 assert(tx_calls==0);assert(comm_hw_can_try_send_status(0x7f4,data,48));assert(tx_calls==1);
 tx_result=1;assert(!comm_hw_can_try_send_status(0x7f4,data,48));assert(tx_calls==2);
 assert(!comm_hw_can_try_send_status(0x7f4,data,47));assert(!comm_hw_can_try_send_status(0x800,data,48));
 hfdcan1.Init.TxFifoQueueMode=0;assert(!comm_hw_can_try_send_status(0x7f4,data,48));assert(tx_calls==2);
 bounded.guard=0x12345678;assert(comm_hw_can_receive(&bounded.f));
 assert(bounded.f.length==48 && bounded.guard==0x12345678);
 rx_length=15;assert(comm_hw_can_receive(&bounded.f));assert(bounded.f.length==64 && bounded.guard==0x12345678);
 puts("PASS actual HAL port: low-priority queue, congestion/no retry, FD48 DLC and bounded FD64 receive");return 0;
}
'''
    priority='\n'.join(common)+r'''
#include "fdcan.h"
#include "hal/api/comm_hw.h"
#include "software/communication/protocol/can_motor_status.h"
Handle hfdcan1;
static struct {bool can_tx_en;unsigned node_id,tx_param_id;uint8_t tx_data_u8[4];uint8_t tx_data_len;} CANMsg;
static unsigned prepared,status_sent,replies;
static bool interrupt_reply;
static uint32_t time_hw_now_ms(void) {return 100;}
bool CanMotorStatus_Prepare(uint32_t t,uint8_t n,uint16_t *id,uint8_t *d,size_t cap) {
 assert(t==100 && n==4 && cap==48);*id=0x7f4;memset(d,0,48);++prepared;
 if(interrupt_reply) CANMsg.can_tx_en=true;
 return true;
}
bool comm_hw_can_try_send_status(uint16_t id,const uint8_t *d,size_t len) {
 (void)d;assert(id==0x7f4 && len==48);++status_sent;return true;
}
unsigned HAL_FDCAN_AddMessageToTxFifoQ(Handle *h,const FDCAN_TxHeaderTypeDef *hdr,const uint8_t *d) {
 (void)h;(void)d;assert(hdr->Identifier==0x465 && hdr->DataLength==4);++replies;return 0;
}
'''+function_source(source,'CAN_SendMessage')+r'''
int main(void) {
 CANMsg.node_id=4;CANMsg.tx_param_id=0x65;CANMsg.tx_data_len=4;CANMsg.can_tx_en=true;
 CAN_SendMessage();assert(replies==1 && prepared==0 && status_sent==0);
 CAN_SendMessage();assert(prepared==1 && status_sent==1);
 interrupt_reply=true;CAN_SendMessage();assert(prepared==2 && status_sent==1);
 CAN_SendMessage();assert(replies==2 && prepared==2);
 puts("PASS actual send dispatcher: reply first, recheck reply after snapshot preparation");return 0;
}
'''
    reply='\n'.join(common)+r'''
#include <limits.h>
#include <math.h>
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
static struct {unsigned tx_param_id;float tx_data;uint8_t tx_data_u8[4],tx_data_len;bool can_tx_en;} CANMsg;
static uint32_t FloatToIntBit(float value) {uint32_t bits;memcpy(&bits,&value,4);return bits;}
'''+function_source(source,'CAN_ReplyEncoding')+function_source(source,'CAN_Milli32')+function_source(source,'CAN_Centi32')+function_source(source,'CAN_Milli16')+function_source(source,'CAN_SendMessage_Update')+r'''
int main(void) {
 CAN_SendMessage_Update(0x05,6.283f);
 assert(CANMsg.tx_data_len==4 && CANMsg.tx_data_u8[0]==0 && CANMsg.tx_data_u8[1]==0 && CANMsg.tx_data_u8[2]==2 && CANMsg.tx_data_u8[3]==0x74);
 CAN_SendMessage_Update(0x07,-1.25f);
 assert(CANMsg.tx_data_len==4 && CANMsg.tx_data_u8[0]==0xff && CANMsg.tx_data_u8[1]==0xff && CANMsg.tx_data_u8[2]==0xfb && CANMsg.tx_data_u8[3]==0x1e);
 CAN_SendMessage_Update(0x03,-1.25f);
 assert(CANMsg.tx_data_len==2 && CANMsg.tx_data_u8[0]==0xfb && CANMsg.tx_data_u8[1]==0x1e);
 CAN_SendMessage_Update(0x15,6.283f);
 assert(CANMsg.tx_data_len==4 && CANMsg.tx_data_u8[0]==0 && CANMsg.tx_data_u8[1]==0 && CANMsg.tx_data_u8[2]==2 && CANMsg.tx_data_u8[3]==0x74);
 puts("PASS actual CAN reply encoding: centi speed/acceleration, milli position and milliamp current");return 0;
}
'''
    command='\n'.join(common)+r'''
#include "software/communication/protocol/can_motor_status.h"
typedef unsigned CAN_PARAM_ID;
#define CAN_SET_STATUS_STREAM 0x64
#define CAN_GET_STATUS_STREAM 0x65
static float reply;
static unsigned replies;
static void CAN_SendMessage_Update(unsigned id,float value) {assert(id==0x65);reply=value;++replies;}
'''+function_source(source,'CAN_ReceiveMessage_Update').split('\tif (!isfinite(data))')[0]+'}\n'+r'''
#include <math.h>
int main(void) {
 CanMotorStatus_Init();CAN_ReceiveMessage_Update(0x64,1);assert(reply==20);
 CAN_ReceiveMessage_Update(0x64,200);assert(reply==200);
 CAN_ReceiveMessage_Update(0x64,19.5f);assert(reply==-1 && CanMotorStatus_Rate()==200);
 CAN_ReceiveMessage_Update(0x64,NAN);assert(reply==-1 && CanMotorStatus_Rate()==200);
 CAN_ReceiveMessage_Update(0x64,0);assert(reply==0);
 CAN_ReceiveMessage_Update(0x65,0);assert(reply==0);
 CAN_ReceiveMessage_Update(0x64,1);assert(reply==200 && replies==7);
 puts("PASS actual command adapter: start/rate/stop/query ACK and invalid input NAK");return 0;
}
'''
    heartbeat='\n'.join(common)+r'''
#define Current_Mode 1
#define Speed_Mode 2
#define Position_Mode 3
#define Position_Impedance_Mode 18
#define CAN_DisConnect 4
#define No_Error 0
static struct {bool can_hb_en,can_rx_en;uint32_t can_hb_set,can_hb_count;} CANMsg;
static struct {unsigned ModeNow,ErrorNow;} MotorControl;
static void Set_ErrorNow(unsigned error) {MotorControl.ErrorNow=error;}
'''+function_source(source,'CAN_DisConnect_Handle')+r'''
int main(void) {
 const unsigned modes[]={1,2,3,18};
 for(unsigned i=0;i<4;++i) {
  MotorControl.ModeNow=modes[i];MotorControl.ErrorNow=0;
  CANMsg.can_rx_en=true;CANMsg.can_hb_set=500;CANMsg.can_hb_count=0;
  for(unsigned t=0;t<499;++t) CAN_DisConnect_Handle();
  assert(MotorControl.ErrorNow==0 && CANMsg.can_hb_count==499);
  CAN_DisConnect_Handle();assert(MotorControl.ErrorNow==4);
  CAN_DisConnect_Handle();assert(CANMsg.can_hb_count==500);
 }
 MotorControl.ModeNow=0;MotorControl.ErrorNow=0;
 for(unsigned t=0;t<1000;++t) CAN_DisConnect_Handle();
 assert(!CANMsg.can_hb_en && CANMsg.can_hb_count==0 && MotorControl.ErrorNow==0);
 MotorControl.ModeNow=3;CAN_DisConnect_Handle();assert(CANMsg.can_hb_count==1);
 CANMsg.can_hb_count=500;MotorControl.ErrorNow=7;
 CAN_DisConnect_Handle();assert(MotorControl.ErrorNow==7);
 CANMsg.can_hb_set=0;CAN_DisConnect_Handle();assert(!CANMsg.can_hb_en && CANMsg.can_hb_count==0);
 CANMsg.can_hb_set=UINT32_MAX;CANMsg.can_hb_count=UINT32_MAX;
 MotorControl.ErrorNow=0;CAN_DisConnect_Handle();
 assert(CANMsg.can_hb_count==UINT32_MAX && MotorControl.ErrorNow==4);
 MotorControl.ModeNow=0;CAN_DisConnect_Handle();assert(MotorControl.ErrorNow==4);
 puts("PASS heartbeat: timeout, STOP disarms, rearm, disabled, saturation, existing fault preserved");return 0;
}
'''
    fixtures=[('heartbeat',heartbeat,[]),('codec',ROOT/'tests/unit/can_motor_status_test.c',[
        ROOT/'software/communication/protocol/can_motor_status.c',ROOT/'software/services/telemetry/motor_status.c']),
        ('sampling',sampling,[ROOT/'software/services/telemetry/motor_status.c']),('rx',rx,[]),
        ('port',port,[ROOT/'hal/ports/stm32g4/comm/comm_status_stm32g4.c']),
        ('priority',priority,[]),('reply',reply,[]),
        ('command',command,[ROOT/'software/communication/protocol/can_motor_status.c',ROOT/'software/services/telemetry/motor_status.c'])]
    logs=[]
    for name,fixture,extra in fixtures:
        if isinstance(fixture,str):
            path=out/(name+'.c');path.write_text(fixture)
        else:path=fixture
        exe=out/(name+'.exe')
        cmd=[args.cc,'-std=c99','-O2','-Wall','-Wextra','-Werror','-I',str(out),'-I',str(ROOT),str(path),*map(str,extra),'-lm','-o',str(exe)]
        for command in (cmd,[str(exe)]):
            result=subprocess.run(command,capture_output=True,text=True)
            logs.append(result.stdout+result.stderr);print(logs[-1],end='')
            (out/'tests.log').write_text(''.join(logs))
            result.check_returncode()

if __name__=='__main__':main()
