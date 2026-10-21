"""Native tests of actual status codec, scheduler, motor source and CAN adapters."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS

import argparse
import json
import re
from pathlib import Path
import subprocess
from run_position_servo_tests import function_source

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--cc',required=True)
    ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args();out=args.out.resolve();out.mkdir(parents=True,exist_ok=True)
    common=['#include <assert.h>','#include <stdint.h>','#include <stdbool.h>',
            '#include <stddef.h>','#include <string.h>','#include <stdio.h>']
    # 复用生产版本常量定义，验证数字、字符串和线路黄金向量一致。
    version_source=(ROOT/'firmware/app/board_config.c').read_text(encoding='utf-8')
    version_definitions=version_source[version_source.index('#define VERSION_STRINGIFY_INNER'):version_source.index('/**')]
    can_header=(ROOT/'firmware/communication/can/interface_can.h').read_text(encoding='utf-8')
    diagnostic_types=re.search(r'typedef struct \{\s+uint32_t rx_rejected;.*?\} CAN_Diagnostics_TypeDef;',can_header,re.S).group(0)
    version_id=re.search(r'CAN_GET_SOFTWARE_VERSION\s*=\s*(0x[0-9A-Fa-f]+)',can_header).group(1)
    common += ['#include "firmware_version.h"', f'#define CAN_GET_SOFTWARE_VERSION {version_id}U', version_definitions]
    version_vector=json.loads((ROOT/'tests/vectors/software_version_v1.json').read_text(encoding='utf-8'))
    source=(ROOT/'firmware/app/foc_task.c').read_text(encoding='utf-8')
    sampling='\n'.join(common)+r'''
#include "firmware/services/telemetry/motor_status.h"
#include <math.h>
#define Position_Mode 3
#define Position_Impedance_Mode 18
#define Speed_Mode 2
typedef struct {float position_reference,trajectory_speed_reference;} PositionCascadeTelemetry_TypeDef;
static bool MotorOuterLoop_GetTelemetry(PositionCascadeTelemetry_TypeDef *out)
{out->position_reference=.75f;out->trajectory_speed_reference=-.5f;return true;}
static struct { unsigned ErrorNow, ModeNow; float posRef,speedRef,iqRef,posShadow,speedShadow,pos_vel_filtered; } MotorControl;
static struct {float theta_mech,vel_mech;} OnBoard_Encoder;
static struct {float Iq,temp,Vbus_filt,Ibus_filt;} FOC;
'''+function_source(source,'MotorStatus_Sampling')+r'''
int main(void) {
 MotorStatus s;
 MotorControl.ErrorNow=7; MotorControl.ModeNow=3;
 MotorControl.posRef=1.25f; MotorControl.posShadow=.75f;
 MotorControl.speedRef=2.5f; MotorControl.speedShadow=9.f;
 MotorControl.iqRef=1.5f; FOC.Iq=1.25f; FOC.temp=65.25f; FOC.Vbus_filt=48.75f; FOC.Ibus_filt=-.375f;
 OnBoard_Encoder.theta_mech=-1.5f; OnBoard_Encoder.vel_mech=-4.f;
 MotorControl.pos_vel_filtered=-2.f;
 MotorStatus_Sampling(); assert(!MotorStatus_Take(&s));
 MotorStatus_Request(); MotorStatus_Sampling(); assert(MotorStatus_Take(&s));
 assert(s.fault==7 && s.mode==3 && s.position_target==1.25f && s.position_planned==.75f);
 assert(s.speed_target==2.5f && s.speed_planned==-.5f && s.speed_feedback==-2.f);
 assert(s.position_feedback==-1.5f && s.current_reference==1.5f && s.current_feedback==1.25f);
 assert(s.temperature==65.25f && s.bus_voltage==48.75f);
 assert(s.bus_current==-.375f && s.bus_current!=s.current_feedback);
 MotorControl.ModeNow=2; MotorStatus_Request(); MotorStatus_Sampling();
 assert(MotorStatus_Take(&s) && s.speed_feedback==-4.f);
 puts("PASS actual motor status source: target vs planned, filtered speed, current, temperature, voltage and fault");return 0;
}
'''
    source=(ROOT/'firmware/communication/can/interface_can.c').read_text(encoding='utf-8')
    reply_runtime=source.split('/* CAN_REPLY_RUNTIME_BEGIN */')[1].split('/* CAN_REPLY_RUNTIME_END */')[0]
    reply_runtime=diagnostic_types+'\nstatic volatile CAN_Diagnostics_TypeDef CAN_Diagnostics;\n'+reply_runtime
    rx='\n'.join(common)+r'''
#include "firmware/platform/api/comm_hw.h"
#include "firmware/communication/protocol/can_motor_status.h"
typedef unsigned CAN_PARAM_ID;
typedef enum {CAN_VALUE_FLOAT32,CAN_VALUE_MILLI_I32,CAN_VALUE_CENTI_I32,CAN_VALUE_MILLI_I16} CanValueEncoding;
static CanValueEncoding CAN_CommandEncoding(CAN_PARAM_ID p)
{if(p==2)return CAN_VALUE_MILLI_I16;if(p==4)return CAN_VALUE_CENTI_I32;if(p==6)return CAN_VALUE_MILLI_I32;return CAN_VALUE_FLOAT32;}
static struct { unsigned node_id,can_hb_count; bool can_rx_en,can_tx_en; uint8_t rx_data_u8[4]; unsigned rx_param_id; float rx_data; } CANMsg;
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
static bool boot_pending;
#define CAN_GET_DEVICE_UID 0x63
static bool BootHandoff_Pending(void) { return boot_pending; }
static bool BootHandoff_Request(uint8_t n,uint32_t t) {(void)n;(void)t;boot_pending=true;return true;}
static void MotorControl_LockMaintenance(void) {}
static void motor_hw_outer_barrier(void) {}
static uint32_t time_hw_now_ms(void) { return 10; }
'''+reply_runtime+function_source(source,'CANRxIRQHandler')+r'''
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
 CANMsg.node_id=4;incoming.identifier=0x463;incoming.length=4;
 memcpy(incoming.data,"NONC",4);CANRxIRQHandler();assert(can_reply.state==CAN_REPLY_UID && can_reply.data[0]=='N' && calls==3);
 incoming.identifier=0x466;memcpy(incoming.data,"BOOT",4);CANRxIRQHandler();assert(boot_pending && calls==3);
 incoming.identifier=0x404;CANRxIRQHandler();assert(calls==3);
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
#define FDCAN_DLC_BYTES_8 8
#define FDCAN_DLC_BYTES_4 4
#define FDCAN_DLC_BYTES_2 2
#define FDCAN_DLC_BYTES_16 10
#define FDCAN_IR_TC 1
#define FDCAN_FILTER_RANGE 0
#define FDCAN_FILTER_TO_RXFIFO0 1
#define FDCAN_REJECT 2
#define FDCAN_FILTER_REMOTE 1
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 8
/* Match all fields/order of the vendor header: omitted fields hid stack junk. */
typedef struct {unsigned Identifier,IdType,TxFrameType,DataLength,ErrorStateIndicator,BitRateSwitch,FDFormat,TxEventFifoControl,MessageMarker;} FDCAN_TxHeaderTypeDef;
typedef struct {unsigned Identifier,IdType,RxFrameType,DataLength;} FDCAN_RxHeaderTypeDef;
typedef struct {unsigned IdType,FilterIndex,FilterType,FilterConfig,FilterID1,FilterID2;} FDCAN_FilterTypeDef;
typedef struct {struct {unsigned TxFifoQueueMode,DataPrescaler,NominalPrescaler;} Init;struct {unsigned IR,TXBTO;} *Instance;unsigned LatestTxFifoQRequest;} Handle;
extern Handle hfdcan1;
unsigned HAL_FDCAN_IsTxBufferMessagePending(Handle *h,unsigned mask);
unsigned HAL_FDCAN_AddMessageToTxFifoQ(Handle *h,const FDCAN_TxHeaderTypeDef *hdr,const uint8_t *d);
unsigned HAL_FDCAN_GetRxMessage(Handle *h,unsigned fifo,FDCAN_RxHeaderTypeDef *hdr,uint8_t *d);
unsigned HAL_FDCAN_ConfigFilter(Handle *h,const FDCAN_FilterTypeDef *filter);
unsigned HAL_FDCAN_ConfigGlobalFilter(Handle *h,unsigned standard,unsigned extended,unsigned std_remote,unsigned ext_remote);
unsigned HAL_FDCAN_Start(Handle *h);
unsigned HAL_FDCAN_Stop(Handle *h);
unsigned HAL_FDCAN_Init(Handle *h);
unsigned HAL_FDCAN_ActivateNotification(Handle *h,unsigned flags,unsigned buffers);
void Error_Handler(void);
#endif
'''
    (out/'fdcan.h').write_text(header)
    (out/'main.h').write_text('#include <stdint.h>\n#include <stdbool.h>\n')
    encoding_types=re.search(r'typedef enum \{\s*CAN_VALUE_FLOAT32,.*?\} CanValueEncoding;',source,re.S).group(0)
    actual=source.split('/* CAN_REPLY_RUNTIME_BEGIN */')[1].split('/* CAN_REPLY_RUNTIME_END */')[0]
    actual+='\n'+encoding_types+'\n'+'\n'.join(function_source(source,name) for name in (
        'CAN_SetNode','CAN_CommandEncoding','CAN_ReplyEncoding','CAN_Milli32','CAN_Centi32','CAN_Milli16',
        'CAN_SetEncoderState','CAN_GetEncoderState','CAN_ReceiveMessage_Update','CAN_SendMessage_Update',
        'CANRxIRQHandler','CAN_SendMessage'))
    # 交接夹具只观察收发抢占；温度开关的状态转换由独立热协议夹具运行真实实现。
    actual = r'''
bool MotorThermalControl_ProtectionEnabled(void) { return true; }
bool MotorThermalControl_RequestProtection(bool enabled, uint32_t mode, bool off)
{ (void)enabled; (void)mode; (void)off; return false; }
''' + actual
    (out/'can_reply_actual.inc').write_text(actual,encoding='utf-8')
    vectors=json.loads((ROOT/'tests/vectors/can_reply_handoff_v1.json').read_text(encoding='utf-8'))
    golden='static const struct {unsigned request_id,reply_id,length;uint8_t request[4],reply[16];} handoff_vectors[]={\n'
    for row in vectors['vectors']:
        request=bytes.fromhex(row['request_hex']);response=bytes.fromhex(row['reply_hex'])
        if len(request)!=4 or len(response) not in (2,4,8,16):
            raise ValueError('CAN交接黄金向量长度错误')
        golden+='{'+f"{row['request_id']},{row['reply_id']},{len(response)}"+',{'+','.join(map(str,request))+'},{'+','.join(map(str,response))+'}},\n'
    (out/'can_reply_vectors.h').write_text(golden+'};\n',encoding='utf-8')
    port='\n'.join(common)+r'''
#include "fdcan.h"
#include "firmware/platform/api/comm_hw.h"
#include <setjmp.h>
Handle hfdcan1;
static unsigned pending,tx_calls,tx_result,rx_length=14,expected_length=14;
static unsigned config_calls,fail_at,error_calls,filter_node;
static char config_trace[16];
static jmp_buf fatal_error;
static struct {uint8_t node_id;uint32_t baudrate;} CANMsg;
static unsigned config_step(char step) {
 assert(config_calls<sizeof(config_trace));config_trace[config_calls++]=step;
 return fail_at!=0 && config_calls==fail_at;
}
static void reset_config(unsigned failure) {
 config_calls=error_calls=0;fail_at=failure;memset(config_trace,0,sizeof(config_trace));
}
void Error_Handler(void) {++error_calls;longjmp(fatal_error,1);}
static void CanMotorStatus_Init(void) {assert(config_step('T')==0);}
unsigned HAL_FDCAN_ConfigGlobalFilter(Handle *h,unsigned standard,unsigned extended,unsigned std_remote,unsigned ext_remote) {
 assert(h==&hfdcan1 && standard==FDCAN_REJECT && extended==FDCAN_REJECT);
 assert(std_remote==FDCAN_FILTER_REMOTE && ext_remote==FDCAN_FILTER_REMOTE);return config_step('G');
}
unsigned HAL_FDCAN_Start(Handle *h) {assert(h==&hfdcan1);return config_step('S');}
unsigned HAL_FDCAN_Stop(Handle *h) {assert(h==&hfdcan1);return config_step('X');}
unsigned HAL_FDCAN_Init(Handle *h) {assert(h==&hfdcan1);return config_step('I');}
unsigned HAL_FDCAN_ActivateNotification(Handle *h,unsigned flags,unsigned buffers) {
 assert(h==&hfdcan1 && flags==FDCAN_IT_RX_FIFO0_NEW_MESSAGE && buffers==0);return config_step('N');
}
unsigned HAL_FDCAN_IsTxBufferMessagePending(Handle *h,unsigned mask) {(void)h;assert(mask==7 || mask==1 || mask==2 || mask==4);return (pending&mask)!=0;}
unsigned HAL_FDCAN_AddMessageToTxFifoQ(Handle *h,const FDCAN_TxHeaderTypeDef *hdr,const uint8_t *d) {
 (void)h; assert(hdr->Identifier==0x7f4 && hdr->DataLength==expected_length && hdr->FDFormat==1 && hdr->BitRateSwitch==1);
 assert(hdr->ErrorStateIndicator==0 && hdr->MessageMarker==0);
 assert(d[0]==0xab);++tx_calls;return tx_result;
}
unsigned HAL_FDCAN_GetRxMessage(Handle *h,unsigned fifo,FDCAN_RxHeaderTypeDef *hdr,uint8_t *d) {
 (void)h;(void)fifo;memset(hdr,0,sizeof(*hdr));hdr->DataLength=rx_length;hdr->Identifier=0x7f4;
 memset(d,0xaa,64);return 0;
}
unsigned HAL_FDCAN_ConfigFilter(Handle *h,const FDCAN_FilterTypeDef *filter) {
 assert(h==&hfdcan1 && filter->IdType==FDCAN_STANDARD_ID && filter->FilterIndex==0);
 assert(filter->FilterType==FDCAN_FILTER_RANGE && filter->FilterConfig==FDCAN_FILTER_TO_RXFIFO0);
 assert(filter->FilterID1==(filter_node<<8) && filter->FilterID2==((filter_node<<8)|0xff));
 return config_step('F');
}
'''+function_source(source,'FDCAN1_Param_Init')+function_source(source,'CAN_BaudRateSwitching')+r'''
static void check_configuration(void) {
 const unsigned rates[]={100,125,200,250,500,1000,2000,2500,5000};
 for(filter_node=0;filter_node<8;++filter_node) {
  reset_config(0);comm_hw_can_start((uint8_t)filter_node);
  assert(config_calls==4 && memcmp(config_trace,"FGSN",4)==0 && error_calls==0);
 }
 filter_node=CANMsg.node_id=4;CANMsg.baudrate=125;reset_config(0);
 FDCAN1_Param_Init();assert(config_calls==5 && memcmp(config_trace,"TFGSN",5)==0);
 assert(CANMsg.baudrate==1000);reset_config(0);
 CAN_BaudRateSwitching();assert(config_calls==0);
 for(unsigned i=0;i<sizeof(rates)/sizeof(rates[0]);++i) {
  CANMsg.baudrate=rates[i];reset_config(0);CAN_BaudRateSwitching();
  assert(config_calls==3 && memcmp(config_trace,"XIS",3)==0 && error_calls==0);
  assert(hfdcan1.Init.DataPrescaler==10000/rates[i]);
  assert(hfdcan1.Init.NominalPrescaler==(rates[i]<=1000 ? 10000/rates[i] : 10));
  CAN_BaudRateSwitching();assert(config_calls==3);
 }
 /* 致命处理用 longjmp 代替死循环，验证每个 HAL 失败都不会继续下一步。 */
 for(unsigned failure=1;failure<=4;++failure) {
  reset_config(failure);
  if(setjmp(fatal_error)==0) {comm_hw_can_start(4);assert(!"expected fatal initialization");}
  assert(error_calls==1 && config_calls==failure && memcmp(config_trace,"FGSN",failure)==0);
 }
 for(unsigned failure=1;failure<=3;++failure) {
  reset_config(failure);hfdcan1.Init.DataPrescaler=hfdcan1.Init.NominalPrescaler=123;
  if(setjmp(fatal_error)==0) {comm_hw_can_set_bitrate(2000);assert(!"expected fatal reconfiguration");}
  assert(error_calls==1 && config_calls==failure && memcmp(config_trace,"XIS",failure)==0);
  assert(hfdcan1.Init.DataPrescaler==(failure==1 ? 123 : 5));
  assert(hfdcan1.Init.NominalPrescaler==(failure==1 ? 123 : 10));
 }
 reset_config(0);
 puts("PASS CAN platform configuration: eight node ranges, nine bitrates, unchanged-value no-op, startup/reconfigure order, all seven HAL fatal paths");
}
int main(void) {
 uint8_t data[48]={0xab};struct {CommHwCanFrame f;uint32_t guard;} bounded;
 check_configuration();
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
 struct {unsigned IR,TXBTO;} fake_regs = {0};
 hfdcan1.Instance=(void *)&fake_regs; hfdcan1.LatestTxFifoQRequest=1;
 pending=0;assert(!comm_hw_can_reply_sent());
 fake_regs.TXBTO=1;assert(comm_hw_can_reply_sent());
 pending=1;assert(!comm_hw_can_reply_sent());pending=0;
 hfdcan1.LatestTxFifoQRequest=2;assert(!comm_hw_can_reply_sent());
 fake_regs.TXBTO=2;assert(comm_hw_can_reply_sent());
 tx_result=0;
 { const unsigned sizes[]={2,4,8,16},dlc[]={2,4,8,10};
   for(unsigned i=0;i<4;++i) {expected_length=dlc[i];assert(comm_hw_can_try_send_reply(0x7f4,data,sizes[i]));}
 }
 assert(!comm_hw_can_try_send_reply(0x7f4,data,3));
 puts("PASS actual HAL port: low-priority queue, congestion/no retry, FD48 DLC and bounded FD64 receive");return 0;
}
'''
    priority='\n'.join(common)+r'''
#include "fdcan.h"
#include "firmware/platform/api/comm_hw.h"
#include "firmware/communication/protocol/can_motor_status.h"
Handle hfdcan1;
static struct {bool can_tx_en;unsigned node_id,tx_param_id;uint8_t tx_data_u8[4];uint8_t tx_data_len;} CANMsg;
static unsigned prepared,status_sent,replies;
static bool interrupt_reply;
static uint32_t time_hw_now_ms(void) {return 100;}
static void motor_hw_outer_barrier(void) {}
'''+reply_runtime+r'''
bool CanMotorStatus_Prepare(uint32_t t,uint8_t n,uint16_t *id,uint8_t *d,size_t cap) {
 assert(t==100 && n==4 && cap==48);*id=0x7f4;memset(d,0,48);++prepared;
 if(interrupt_reply) {const uint8_t p[4]={0};assert(CAN_QueueReply(0x465,p,4,CAN_REPLY_READY));}
 return true;
}
bool comm_hw_can_try_send_status(uint16_t id,const uint8_t *d,size_t len) {
 (void)d;assert(id==0x7f4 && len==48);++status_sent;return true;
}
bool comm_hw_can_try_send_reply(uint16_t id,const uint8_t *d,size_t n) {
 (void)d;assert(id==0x465 && n==4);++replies;return true;
}
static bool BootHandoff_Pending(void) { return false; }
static void boot_hw_uid(uint8_t *p) { memset(p,0,12); }
static uint32_t boot_hw_parameters_crc(void) { return 0; }
'''+function_source(source,'CAN_SendMessage')+r'''
int main(void) {
 const uint8_t payload[4]={0};CANMsg.node_id=4;assert(CAN_QueueReply(0x465,payload,4,CAN_REPLY_READY));
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
static struct {unsigned node_id,tx_param_id;float tx_data;uint8_t tx_data_u8[4],tx_data_len;bool can_tx_en;} CANMsg;
static uint32_t FloatToIntBit(float value) {uint32_t bits;memcpy(&bits,&value,4);return bits;}
static void motor_hw_outer_barrier(void) {}
'''+reply_runtime+function_source(source,'CAN_ReplyEncoding')+function_source(source,'CAN_Milli32')+function_source(source,'CAN_Centi32')+function_source(source,'CAN_Milli16')+function_source(source,'CAN_SendMessage_Update')+r'''
static void encode_next(CAN_PARAM_ID id,float value) {
 can_reply.state=CAN_REPLY_FREE;CAN_SendMessage_Update(id,value);
}
#define CAN_SendMessage_Update encode_next
int main(void) {
 CAN_SendMessage_Update(0x05,6.283f);
 assert(CANMsg.tx_data_len==4 && CANMsg.tx_data_u8[0]==0 && CANMsg.tx_data_u8[1]==0 && CANMsg.tx_data_u8[2]==2 && CANMsg.tx_data_u8[3]==0x74);
 CAN_SendMessage_Update(0x07,-1.25f);
 assert(CANMsg.tx_data_len==4 && CANMsg.tx_data_u8[0]==0xff && CANMsg.tx_data_u8[1]==0xff && CANMsg.tx_data_u8[2]==0xfb && CANMsg.tx_data_u8[3]==0x1e);
 CAN_SendMessage_Update(0x03,-1.25f);
 assert(CANMsg.tx_data_len==2 && CANMsg.tx_data_u8[0]==0xfb && CANMsg.tx_data_u8[1]==0x1e);
 CAN_SendMessage_Update(0x15,6.283f);
 assert(CANMsg.tx_data_len==4 && CANMsg.tx_data_u8[0]==0 && CANMsg.tx_data_u8[1]==0 && CANMsg.tx_data_u8[2]==2 && CANMsg.tx_data_u8[3]==0x74);
 { const unsigned ids[]={0x15,0x17,0x1d,0x1f,0x21};
   for(unsigned i=0;i<sizeof(ids)/sizeof(ids[0]);++i) {
    CAN_SendMessage_Update(ids[i],0.785398163f);
    assert(CANMsg.tx_data_len==4 && CANMsg.tx_data_u8[0]==0 && CANMsg.tx_data_u8[1]==0 && CANMsg.tx_data_u8[2]==0 && CANMsg.tx_data_u8[3]==0x4e);
   }
 }
 CAN_SendMessage_Update(0x11,6.f);assert(CANMsg.tx_data_len==2 && CANMsg.tx_data_u8[0]==0x17 && CANMsg.tx_data_u8[1]==0x70);
 CAN_SendMessage_Update(0x67,2.f);assert(CANMsg.tx_data_len==4 && CANMsg.tx_data_u8[0]==0x40 && CANMsg.tx_data_u8[1]==0 && CANMsg.tx_data_u8[2]==0 && CANMsg.tx_data_u8[3]==0);
 {
  const uint8_t golden[]={VERSION_EXPECTED_BYTES};
  assert(g_firmware_software_version==VERSION_EXPECTED_CODE);
  assert(CAN_GET_SOFTWARE_VERSION==VERSION_EXPECTED_ID);
  assert(strcmp(g_firmware_software_version_string,"VERSION_EXPECTED_STRING")==0);
  CAN_SendMessage_Update(CAN_GET_SOFTWARE_VERSION,NAN);
  assert(CANMsg.tx_param_id==CAN_GET_SOFTWARE_VERSION && CANMsg.tx_data_len==4 && CANMsg.can_tx_en);
  assert(memcmp(CANMsg.tx_data_u8,golden,4)==0);
 }
 puts("PASS actual CAN reply encoding: centi speed/acceleration, milli position and milliamp current");return 0;
}
'''
    reply=reply.replace('VERSION_EXPECTED_BYTES',','.join(f'0x{b:02x}' for b in bytes.fromhex(version_vector['vectors'][0]['reply_hex'])))
    reply=reply.replace('VERSION_EXPECTED_CODE',str(version_vector['version_code'])+'U')
    reply=reply.replace('VERSION_EXPECTED_ID',str(version_vector['parameter_id'])+'U')
    reply=reply.replace('VERSION_EXPECTED_STRING',version_vector['software_version'])
    command='\n'.join(common)+r'''
#include "firmware/communication/protocol/can_motor_status.h"
#include <math.h>
typedef unsigned CAN_PARAM_ID;
/* 补齐当前命令入口已有的位置速度 PI 分支，避免旧夹具缺少声明。 */
#define CAN_SET_POSITION_SPEED_KP 0x6B
#define CAN_GET_POSITION_SPEED_KP 0x6C
#define CAN_SET_POSITION_SPEED_KI 0x6D
#define CAN_GET_POSITION_SPEED_KI 0x6E
#include "speed_feedforward.h"
#define CAN_SET_SPEED_FF_ENABLE 0x70
#define CAN_GET_SPEED_FF_ENABLE 0x71
#define CAN_SET_SPEED_FF_GAIN 0x72
#define CAN_GET_SPEED_FF_GAIN 0x73
#define CAN_SET_SPEED_FF_ACCELERATION 0x74
#define CAN_GET_SPEED_FF_ACCELERATION 0x75
#define CAN_SET_SPEED_FF_JERK 0x76
#define CAN_GET_SPEED_FF_JERK 0x77
#define CAN_SET_SPEED_FF_CURRENT_LIMIT 0x78
#define CAN_GET_SPEED_FF_CURRENT_LIMIT 0x79
#define CAN_GET_SPEED_FF_CURRENT 0x7A
#define CAN_GET_SPEED_FF_ACCELERATION_PLANNED 0x7B
#include "motor_thermal_control.h"
#define CAN_SET_THERMAL_CONTINUOUS 0x80
#define CAN_GET_THERMAL_CONTINUOUS 0x81
#define CAN_SET_THERMAL_PEAK 0x82
#define CAN_GET_THERMAL_PEAK 0x83
#define CAN_SET_THERMAL_HEATING_TAU 0x84
#define CAN_GET_THERMAL_HEATING_TAU 0x85
#define CAN_SET_THERMAL_DERATING_ENTER 0x86
#define CAN_GET_THERMAL_DERATING_ENTER 0x87
#define CAN_SET_THERMAL_DERATING_EXIT 0x88
#define CAN_GET_THERMAL_DERATING_EXIT 0x89
#define CAN_SET_THERMAL_TRIP 0x8A
#define CAN_GET_THERMAL_TRIP 0x8B
#define CAN_SET_THERMAL_CLEAR 0x8C
#define CAN_GET_THERMAL_CLEAR 0x8D
#define CAN_GET_THERMAL_REVISION 0x8E
#define CAN_SET_TEMPERATURE_PROTECTION 0x90
#define CAN_GET_TEMPERATURE_PROTECTION 0x91
/* 通信状态用例不修改热模型，热命令由独立 golden 用例覆盖。 */
bool MotorThermalControl_GetParameter(MotorThermalParameter p,float *v) { (void)p;(void)v;return false; }
bool MotorThermalControl_RequestParameter(MotorThermalParameter p,float v,uint32_t m,bool off) { (void)p;(void)v;(void)m;(void)off;return false; }
bool MotorThermalControl_ProtectionEnabled(void) { return true; }
bool MotorThermalControl_RequestProtection(bool enabled,uint32_t mode,bool off) { (void)enabled;(void)mode;(void)off;return false; }
bool motor_hw_phase_outputs_disabled(void) { return true; }
#define Motor_Disable 0U
static unsigned ModeLast;
static struct {float position_speed_Kp, position_speed_Ki;unsigned ModeNow,ErrorNow;
    SpeedFeedforwardConfig speed_ff_config;SpeedFeedforwardState speed_ff_state;} MotorControl;
static bool MotorControl_SetSpeedFeedforward(const SpeedFeedforwardConfig *config) {(void)config;return false;}
#define CAN_SET_STATUS_STREAM 0x64
#define CAN_GET_STATUS_STREAM 0x65
#define CAN_GET_PROTOCOL_REVISION 0x67
#define CAN_SET_LOAD_PROFILE 0x68
#define CAN_GET_LOAD_PROFILE 0x69
#define CAN_GET_LOAD_PROFILE_REVISION 0x6A
static bool MotorControl_RequestLoadProfile(uint32_t flags) { (void)flags; return false; }
static int32_t MotorControl_GetLoadProfile(void) { return 0; }
#include "firmware/communication/protocol/can_parameter_format.h"
static float reply;
static unsigned replies;
static unsigned reply_id;
static void CAN_SendMessage_Update(unsigned id,float value) {assert(id==0x65 || id==0x67 || id==0x69 || id==0x6A || id==CAN_GET_SOFTWARE_VERSION || id==CAN_GET_TEMPERATURE_PROTECTION);reply_id=id;reply=value;++replies;}
'''+function_source(source,'CAN_ReceiveMessage_Update').split('    /* 节点事务')[0]+'}\n'+r'''
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
 MotorControl.ModeNow=3;MotorControl.ErrorNow=7;
 MotorControl.position_speed_Kp=.5f;MotorControl.position_speed_Ki=1;
 CAN_ReceiveMessage_Update(CAN_GET_SOFTWARE_VERSION,0);
 assert(reply_id==CAN_GET_SOFTWARE_VERSION && reply==0 && replies==9);
 CAN_ReceiveMessage_Update(CAN_GET_SOFTWARE_VERSION,NAN);
 assert(reply_id==CAN_GET_SOFTWARE_VERSION && replies==10);
 assert(MotorControl.ModeNow==3 && MotorControl.ErrorNow==7);
 assert(MotorControl.position_speed_Kp==.5f && MotorControl.position_speed_Ki==1 && CanMotorStatus_Rate()==200);
 CAN_ReceiveMessage_Update(CAN_GET_TEMPERATURE_PROTECTION,NAN);
 assert(reply_id==CAN_GET_TEMPERATURE_PROTECTION && reply==1);
 CAN_ReceiveMessage_Update(CAN_SET_TEMPERATURE_PROTECTION,0);
 assert(reply_id==CAN_GET_TEMPERATURE_PROTECTION && reply==-1);
 assert(MotorControl.ModeNow==3 && MotorControl.ErrorNow==7);
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
        ROOT/'firmware/communication/protocol/can_motor_status.c',ROOT/'firmware/services/telemetry/motor_status.c']),
        ('sampling',sampling,[ROOT/'firmware/services/telemetry/motor_status.c']),('rx',rx,[]),
        ('port',port,[ROOT/'firmware/platform/stm32g4/ports/comm/comm_status_stm32g4.c']),
        ('priority',priority,[]),('reply',reply,[]),
        ('command',command,[ROOT/'firmware/communication/protocol/can_motor_status.c',ROOT/'firmware/services/telemetry/motor_status.c']),
        ('handoff',ROOT/'tests/unit/can_reply_handoff_test.c',[ROOT/'firmware/platform/stm32g4/ports/comm/comm_status_stm32g4.c'])]
    logs=[]
    for name,fixture,extra in fixtures:
        if isinstance(fixture,str):
            path=out/(name+'.c');path.write_text(fixture)
        else:path=fixture
        exe=out/(name+'.exe')
        compiler=[args.cc]+(['cc'] if Path(args.cc).stem=='zig' else []) + NATIVE_INCLUDE_FLAGS
        cmd=compiler+['-std=c99','-O2','-UNDEBUG','-Wall','-Wextra','-Werror','-I',str(out),'-I',str(ROOT),str(path),*map(str,extra),'-lm','-o',str(exe)]
        # Poison automatic variables to reproduce missing TX header initialization.
        # These host fixtures require a compiler supporting this Clang/GCC option.
        if name in ('priority','port'):cmd.insert(len(compiler),'-ftrivial-auto-var-init=pattern')
        for command in (cmd,[str(exe)]):
            result=subprocess.run(command,capture_output=True,text=True)
            logs.append(result.stdout+result.stderr);print(logs[-1],end='')
            (out/'tests.log').write_text(''.join(logs))
            result.check_returncode()

if __name__=='__main__':main()
