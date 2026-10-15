/* 运行真实 CAN 分派/回复和平台发送函数；替身在指定位置模拟 RX 抢占。 */
#include <assert.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "fdcan.h"
#include "interface_can.h"
#include "comm_hw.h"
#include "can_motor_status.h"
#include "can_parameter_format.h"
#include "firmware_version.h"
#include "foc_algorithm.h"
#include "foc_friction_identification.h"
#include "motor_thermal_control.h"
#include "position_impedance_config.h"
#include "position_cascade_config.h"
#include "utils.h"

CANMsg_TypeDef CANMsg;
volatile CAN_Diagnostics_TypeDef CAN_Diagnostics;
MotorControl_TypeDef MotorControl;
ModeNow_TypeDef ModeLast;
FOC_TypeDef FOC;
Encoder_TypeDef OnBoard_Encoder;
Handle hfdcan1;
const uint32_t g_firmware_software_version = FIRMWARE_VERSION_CODE;
static CommHwCanFrame incoming;
static bool phase_off = true, boot_pending, filter_failure;
static unsigned tx_result, tx_attempts, sent_count, pending, crc_calls, uid_calls;
static unsigned filter_calls, filter_node = 4, limit_updates, injected;
static uint8_t sent_data[16];
static unsigned sent_id, sent_length;
static void (*send_hook)(void), (*crc_hook)(void);

void motor_hw_outer_barrier(void) {}
bool motor_hw_phase_outputs_disabled(void) { return phase_off; }
static uint32_t time_hw_now_ms(void) { return 100U; }
static bool BootHandoff_Pending(void) { return boot_pending; }
static bool BootHandoff_Request(uint8_t n, uint32_t t)
{ (void)n; (void)t; boot_pending = true; return true; }
static void MotorControl_LockMaintenance(void) {}
static void boot_hw_uid(uint8_t *data)
{ ++uid_calls; for (unsigned i=0; i<12; ++i) { data[i]=(uint8_t)i; } }
static uint32_t boot_hw_parameters_crc(void)
{ ++crc_calls; if (crc_hook) { crc_hook(); } return 0x12345678U; }
static void Set_ErrorNow(ErrorNow_TypeDef e) { MotorControl.ErrorNow=e; }
static bool ModeSwitch_Handle(ModeNow_TypeDef m) { MotorControl.ModeNow=m; return true; }
static float Param_SpeedLimitRadS(uint8_t node) { return node == 3 ? 2.0f : 20.0f; }
static bool Param_SetSpeedLimit(float value)
{ ++limit_updates; assert(value <= Param_SpeedLimitRadS(CANMsg.node_id)); MotorControl.speed_limit=value; return true; }
static bool MotorControl_SetSpeedFeedforward(const SpeedFeedforwardConfig *c) { (void)c; return false; }
static bool MotorControl_RequestLoadProfile(uint32_t f) { (void)f; return false; }
static int32_t MotorControl_GetLoadProfile(void) { return 0; }
bool MotorThermalControl_GetParameter(MotorThermalParameter p,float *v) { (void)p;(void)v;return false; }
bool MotorThermalControl_RequestParameter(MotorThermalParameter p,float v,uint32_t m,bool off)
{ (void)p;(void)v;(void)m;(void)off;return false; }
bool CanMotorStatus_Configure(float rate) { (void)rate; return true; }
uint16_t CanMotorStatus_Rate(void) { return 20U; }
bool CanMotorStatus_Prepare(uint32_t t,uint8_t n,uint16_t *id,uint8_t *d,size_t size)
{ (void)t;(void)n;(void)id;(void)d;(void)size;return false; }
bool Encoder_IsOnline(const Encoder_TypeDef *e) { (void)e; return true; }
void Encoder_SetReverse(Encoder_TypeDef *e,bool reverse) { e->reverse=reverse; }
bool FocFrictionIdentification_ApplyCandidate(MotorControl_TypeDef *m) { (void)m;return false; }
FrictionIdentificationState_TypeDef FocFrictionIdentification_GetState(void) { return 0; }
FrictionIdentificationReason_TypeDef FocFrictionIdentification_GetReason(void) { return 0; }
const FrictionIdentificationResult_TypeDef *FocFrictionIdentification_GetResult(void)
{ static FrictionIdentificationResult_TypeDef result; return &result; }
float fast_abs(float v) { return fabsf(v); }
float constrain(float v,float low,float high) { return fminf(fmaxf(v,low),high); }
uint32_t FloatToIntBit(float v) { uint32_t bits;memcpy(&bits,&v,4);return bits; }
float IntBitToFloat(uint32_t bits) { float v;memcpy(&v,&bits,4);return v; }

unsigned HAL_FDCAN_IsTxBufferMessagePending(Handle *h,unsigned mask)
{ (void)h; return (pending&mask)!=0; }
unsigned HAL_FDCAN_AddMessageToTxFifoQ(Handle *h,const FDCAN_TxHeaderTypeDef *header,const uint8_t *data)
{
    (void)h; ++tx_attempts;
    assert(header->ErrorStateIndicator==0 && header->MessageMarker==0);
    assert(header->IdType==FDCAN_STANDARD_ID && header->TxFrameType==FDCAN_DATA_FRAME);
    assert(header->FDFormat==FDCAN_FD_CAN && header->BitRateSwitch==FDCAN_BRS_ON);
    if (send_hook) { send_hook(); }
    if (tx_result) { return tx_result; }
    sent_id=header->Identifier;
    sent_length=header->DataLength==10U ? 16U : header->DataLength;
    assert(sent_length<=sizeof(sent_data));
    memcpy(sent_data,data,sent_length); ++sent_count;
    return HAL_OK;
}
unsigned HAL_FDCAN_GetRxMessage(Handle *h,unsigned fifo,FDCAN_RxHeaderTypeDef *header,uint8_t *data)
{
    (void)h;(void)fifo;memset(header,0,sizeof(*header));
    header->Identifier=incoming.identifier;header->DataLength=incoming.length;
    header->IdType=incoming.extended;header->RxFrameType=incoming.remote;
    memcpy(data,incoming.data,64);return HAL_OK;
}
unsigned HAL_FDCAN_ConfigFilter(Handle *h,const FDCAN_FilterTypeDef *filter)
{
    (void)h;++filter_calls;
    assert(filter->IdType==FDCAN_STANDARD_ID && filter->FilterIndex==0);
    assert(filter->FilterType==FDCAN_FILTER_RANGE && filter->FilterConfig==FDCAN_FILTER_TO_RXFIFO0);
    assert(filter->FilterID2==(filter->FilterID1|255U));
    /* 平台写硬件之前，软件节点必须仍为旧值。 */
    assert(CANMsg.node_id==filter_node);
    if (filter_failure) { return 1; }
    filter_node=filter->FilterID1>>8;return HAL_OK;
}

/* 由现有 native 入口逐函数抽取生成，不复制生产实现。 */
#include "can_reply_actual.inc"
#include "can_reply_vectors.h"

static void reset_fixture(void)
{
    memset(&CANMsg,0,sizeof(CANMsg));memset(&MotorControl,0,sizeof(MotorControl));
    memset((void *)&CAN_Diagnostics,0,sizeof(CAN_Diagnostics));
    memset((void *)&can_reply,0,sizeof(can_reply));
    CANMsg.node_id=4;CANMsg.baudrate=1000;MotorControl.speed_limit=20;
    ModeLast=Motor_Disable;phase_off=true;boot_pending=false;filter_failure=false;
    tx_result=tx_attempts=sent_count=pending=crc_calls=uid_calls=0;
    filter_calls=limit_updates=injected=0;filter_node=4;send_hook=crc_hook=NULL;
}
static void request(unsigned node,unsigned op,const uint8_t data[4])
{
    memset(&incoming,0,sizeof(incoming));incoming.identifier=(node<<8)|op;
    incoming.length=4;memcpy(incoming.data,data,4);CANRxIRQHandler();
}
static void scalar(unsigned node,unsigned op,float v)
{
    uint32_t bits=FloatToIntBit(v);
    uint8_t data[4]={(uint8_t)(bits>>24),(uint8_t)(bits>>16),(uint8_t)(bits>>8),(uint8_t)bits};
    request(node,op,data);
}
static void check_sent(unsigned id,const uint8_t *data,unsigned length)
{ assert(sent_id==id && sent_length==length && memcmp(sent_data,data,length)==0); }
static void inject_query(void)
{ ++injected;scalar(CANMsg.node_id,CAN_GET_NODE_ID,0); }
static void inject_uid(void)
{ const uint8_t nonce[4]={9,8,7,6};++injected;request(CANMsg.node_id,CAN_GET_DEVICE_UID,nonce); }
static void inject_boot(void)
{ const uint8_t boot[4]={'B','O','O','T'};request(CANMsg.node_id,0x66,boot); }

int main(void)
{
    const uint8_t current[2]={0xfb,0x1e}, node4[4]={0x40,0x80,0,0};
    const uint8_t nonce[4]={1,2,3,4}, crc[8]={1,2,3,4,0x12,0x34,0x56,0x78};
    uint8_t uid[16]={1,2,3,4};
    for (unsigned i=0;i<12;++i) { uid[4+i]=(uint8_t)i; }
    for (unsigned i=0;i<sizeof(handoff_vectors)/sizeof(handoff_vectors[0]);++i) {
        reset_fixture();MotorControl.iqRef=-1.25f;
        request(handoff_vectors[i].request_id>>8,handoff_vectors[i].request_id&255U,handoff_vectors[i].request);
        CAN_SendMessage();
        check_sent(handoff_vectors[i].reply_id,handoff_vectors[i].reply,handoff_vectors[i].length);
    }
    reset_fixture();
    CAN_SendMessage_Update(CAN_GET_CURRENT_SET,-1.25f);
    CAN_SendMessage_Update(CAN_GET_NODE_ID,4);
    assert(CAN_Diagnostics.reply_busy==1 && CANMsg.tx_param_id==CAN_GET_CURRENT_SET);
    send_hook=inject_query;CAN_SendMessage();
    check_sent(0x403,current,2);assert(injected==1 && CAN_Diagnostics.reply_busy==2);
    assert(can_reply.state==CAN_REPLY_FREE && !CANMsg.can_tx_en);
    send_hook=NULL;CAN_SendMessage();assert(sent_count==1);
    scalar(4,CAN_GET_NODE_ID,0);CAN_SendMessage();check_sent(0x409,node4,4);

    reset_fixture();request(4,CAN_GET_PARAMETER_CRC,nonce);
    crc_hook=inject_uid;tx_result=1;CAN_SendMessage();
    assert(crc_calls==1 && injected==1 && CAN_Diagnostics.reply_busy==1);
    assert(can_reply.state==CAN_REPLY_READY && CANMsg.can_tx_en && tx_attempts==1);
    for (unsigned i=0;i<6;++i) { CAN_SendMessage(); }
    assert(crc_calls==1 && CAN_Diagnostics.tx_busy==7 && tx_attempts==7);
    pending=1;CAN_SendMessage();assert(tx_attempts==7 && CAN_Diagnostics.tx_busy==8);
    pending=0;tx_result=0;CAN_SendMessage();check_sent(0x47c,crc,8);
    assert(crc_calls==1 && can_reply.state==CAN_REPLY_FREE);
    request(4,CAN_GET_DEVICE_UID,nonce);send_hook=inject_query;CAN_SendMessage();
    check_sent(0x463,uid,16);assert(uid_calls==1 && CAN_Diagnostics.reply_busy==2);

    reset_fixture();request(4,CAN_GET_PARAMETER_CRC,nonce);crc_hook=inject_boot;CAN_SendMessage();
    assert(boot_pending && tx_attempts==0 && crc_calls==1);
    CAN_SendMessage();assert(tx_attempts==0 && crc_calls==1);
    scalar(4,CAN_SET_NODE_ID,3);assert(CANMsg.node_id==4 && filter_calls==0);

    reset_fixture();
    { const float invalid[]={NAN,INFINITY,-INFINITY,FLT_MAX,-1.0f,7.5f,8.0f};
      for (unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);++i) { scalar(4,CAN_SET_NODE_ID,invalid[i]); }
      assert(CAN_Diagnostics.node_rejected==7 && CANMsg.node_id==4 && filter_calls==0);
    }
    MotorControl.ModeNow=Speed_Mode;scalar(4,CAN_SET_NODE_ID,3);
    MotorControl.ModeNow=Motor_Disable;ModeLast=Speed_Mode;scalar(4,CAN_SET_NODE_ID,3);
    ModeLast=Motor_Disable;phase_off=false;scalar(4,CAN_SET_NODE_ID,3);
    assert(CAN_Diagnostics.node_rejected==10 && filter_calls==0 && CANMsg.node_id==4);
    phase_off=true;filter_failure=true;scalar(4,CAN_SET_NODE_ID,3);
    assert(CANMsg.node_id==4 && filter_node==4 && limit_updates==0 && CAN_Diagnostics.node_filter_failed==1);
    filter_failure=false;
    scalar(4,CAN_GET_NODE_ID,0);scalar(4,CAN_SET_NODE_ID,3);
    assert(CANMsg.node_id==3 && filter_node==3 && MotorControl.speed_limit==2 && limit_updates==1);
    CAN_SendMessage();check_sent(0x409,node4,4); /* 改号前的回复仍用原 ID。 */
    scalar(3,CAN_SET_NODE_ID,3);assert(filter_calls==2 && limit_updates==1);
    scalar(4,CAN_GET_NODE_ID,0);assert(can_reply.state==CAN_REPLY_FREE);
    scalar(3,CAN_GET_NODE_ID,0);CAN_SendMessage();
    { const uint8_t node3[4]={0x40,0x40,0,0};check_sent(0x309,node3,4); }

    reset_fixture();scalar(4,CAN_GET_CAN_BR,0);CAN_SendMessage();
    { const uint8_t baud[4]={0x44,0x7a,0,0};check_sent(0x429,baud,4); }
    scalar(4,0xfe,0);assert(CAN_Diagnostics.rx_rejected==1);
    scalar(4,CAN_GET_NODE_ID,FLT_MAX);assert(CAN_Diagnostics.rx_rejected==2);
    CAN_Diagnostics.rx_rejected=UINT32_MAX;scalar(4,0xfe,0);
    assert(CAN_Diagnostics.rx_rejected==UINT32_MAX);
    CAN_SendMessage_Update(CAN_GET_NODE_ID,4);CAN_Diagnostics.reply_busy=UINT32_MAX;
    CAN_SendMessage_Update(CAN_GET_NODE_ID,4);assert(CAN_Diagnostics.reply_busy==UINT32_MAX);
    CAN_Diagnostics.tx_busy=UINT32_MAX;tx_result=1;CAN_SendMessage();assert(CAN_Diagnostics.tx_busy==UINT32_MAX);
    puts("PASS CAN handoff: immutable reply/nonce/ID, one CRC per request, retry, BOOT, stopped node transaction, baud GET");
    return 0;
}
