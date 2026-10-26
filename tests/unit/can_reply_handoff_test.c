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
/* 此夹具只测试运行期收发；初始化/调速 HAL 由端口配置夹具覆盖。 */
void Error_Handler(void) { assert(!"unexpected platform fatal handler"); }
unsigned HAL_FDCAN_ConfigGlobalFilter(Handle *h,unsigned s,unsigned e,unsigned sr,unsigned er)
{ (void)h;(void)s;(void)e;(void)sr;(void)er;assert(!"unexpected CAN initialization");return 1; }
unsigned HAL_FDCAN_Start(Handle *h) { (void)h;assert(!"unexpected CAN start");return 1; }
unsigned HAL_FDCAN_Stop(Handle *h) { (void)h;assert(!"unexpected CAN stop");return 1; }
unsigned HAL_FDCAN_Init(Handle *h) { (void)h;assert(!"unexpected CAN initialization");return 1; }
unsigned HAL_FDCAN_ActivateNotification(Handle *h,unsigned f,unsigned b)
{ (void)h;(void)f;(void)b;assert(!"unexpected CAN notification configuration");return 1; }
const uint32_t g_firmware_software_version = FIRMWARE_VERSION_CODE;
static CommHwCanFrame incoming;
static bool phase_off = true, boot_pending, filter_failure, mode_allowed = true;
static unsigned tx_result, tx_attempts, sent_count, pending, crc_calls, uid_calls;
static unsigned filter_calls, filter_node = 4, limit_updates, injected;
static uint8_t sent_data[16];
static unsigned sent_id, sent_length;
static bool sent_fd, sent_brs; /**< 最近成功入队的帧格式，由 HAL 替身写入。 */
static uint16_t status_rate; /**< 可选状态流的模拟速率，0 表示关闭。 */
static unsigned continuous_speed_reads; /**< 连续速度getter调用次数，用于识别构建与模式门禁。 */
static void (*send_hook)(void), (*crc_hook)(void);

void motor_hw_outer_barrier(void) {}
bool motor_hw_phase_outputs_disabled(void) { return phase_off; }
static uint32_t time_hw_now_ms(void) { return 100U; }
static bool BootHandoff_Pending(void) { return boot_pending; }
static bool BootHandoff_Request(uint8_t n, uint32_t t)
{ (void)n; (void)t; boot_pending = true; return true; }
static void MotorControl_LockMaintenance(void) {}
void MotorOuterLoop_PublishPositionTarget(float position_rad,uint32_t received_ms)
{ (void)position_rad; (void)received_ms; }
static void boot_hw_uid(uint8_t *data)
{ ++uid_calls; for (unsigned i=0; i<12; ++i) { data[i]=(uint8_t)i; } }
static uint32_t boot_hw_parameters_crc(void)
{ ++crc_calls; if (crc_hook) { crc_hook(); } return 0x12345678U; }
static void Set_ErrorNow(ErrorNow_TypeDef e) { MotorControl.ErrorNow=e; }
static bool ModeSwitch_Handle(ModeNow_TypeDef m)
{ if (!mode_allowed) { return false; } MotorControl.ModeNow=m; return true; }
static float Param_SpeedLimitRadS(uint8_t node) { return node == 3 ? 2.0f : 20.0f; }
static bool Param_SetSpeedLimit(float value)
{ if (!isfinite(value) || value <= 0 || value > Param_SpeedLimitRadS(CANMsg.node_id)) { return false; }
  ++limit_updates; MotorControl.speed_limit=value; return true; }
static bool MotorControl_SetSpeedFeedforward(const SpeedFeedforwardConfig *c) { (void)c; return false; }
static bool MotorControl_RequestLoadProfile(uint32_t f) { (void)f; return false; }
static int32_t MotorControl_GetLoadProfile(void) { return 0; }
bool MotorThermalControl_GetParameter(MotorThermalParameter p,float *v) { (void)p;(void)v;return false; }
bool MotorThermalControl_RequestParameter(MotorThermalParameter p,float v,uint32_t m,bool off)
{ (void)p;(void)v;(void)m;(void)off;return false; }
/**
 * @brief 模拟状态流配置，供格式准入测试观察是否意外修改配置。
 * @param[in] rate 0 关闭，1 使用默认速率，或 10～200 的整数速率。
 * @return 参数有效返回 true 并记录速率，否则返回 false 且保留旧值。
 */
#ifndef CAN_REAL_STATUS
bool CanMotorStatus_Configure(float rate)
{
    if (!isfinite(rate) || (rate != 0.0f && rate != 1.0f &&
        (rate < 10.0f || rate > 200.0f || rate != (float)(uint16_t)rate))) return false;
    status_rate = rate == 1.0f ? 20U : (uint16_t)rate;
    return true;
}
/** @brief 返回测试状态流速率。
 * @return 当前模拟速率，0 为关闭。
 */
uint16_t CanMotorStatus_Rate(void) { return status_rate; }
bool CanMotorStatus_Prepare(uint32_t t,uint8_t n,uint16_t *id,uint8_t *d,size_t size)
{ (void)t;(void)n;(void)id;(void)d;(void)size;return false; }
#endif
bool Encoder_IsOnline(const Encoder_TypeDef *e) { (void)e; return true; }
/** @brief 返回模拟连续测速并记录读取次数，不更新编码器或访问设备。
 * @param[in] encoder 借用的编码器内存样本。
 * @return 样本的无门限机械速度，单位rad/s。
 */
float Encoder_GetMecVelContinuous(const Encoder_TypeDef *encoder)
{
    continuous_speed_reads++;
    return encoder->vel_mech_continuous;
}
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
/**
 * @brief 模拟 HAL 入队，记录有效载荷及格式并允许注入接收抢占。
 * @param[in] h 模拟外设句柄，不访问硬件。
 * @param[in] header 待入队帧头，仅在调用期读取。
 * @param[in] data 至少帧头长度的只读数据，成功时复制至测试结果。
 * @return tx_result；零表示成功，非零模拟 HAL 忙或错误。
 */
unsigned HAL_FDCAN_AddMessageToTxFifoQ(Handle *h,const FDCAN_TxHeaderTypeDef *header,const uint8_t *data)
{
    (void)h; ++tx_attempts;
    assert(header->ErrorStateIndicator==0 && header->MessageMarker==0);
    assert(header->IdType==FDCAN_STANDARD_ID && header->TxFrameType==FDCAN_DATA_FRAME);
    assert(header->FDFormat==FDCAN_FD_CAN || header->FDFormat==FDCAN_CLASSIC_CAN);
    assert(header->FDFormat==FDCAN_FD_CAN || header->BitRateSwitch==FDCAN_BRS_OFF);
    if (send_hook) { send_hook(); }
    if (tx_result) { return tx_result; }
    sent_id=header->Identifier;
    sent_fd=header->FDFormat==FDCAN_FD_CAN;
    sent_brs=header->BitRateSwitch==FDCAN_BRS_ON;
    sent_length=header->DataLength==10U ? 16U : header->DataLength;
    assert(sent_length<=sizeof(sent_data));
    memcpy(sent_data,data,sent_length); ++sent_count;
    return HAL_OK;
}
/**
 * @brief 将预设输入帧交给生产接收端口，保留其 FD/BRS 标志。
 * @param[in] h 模拟外设句柄，不访问硬件。
 * @param[in] fifo 模拟接收队列编号。
 * @param[out] header 完整模拟帧头。
 * @param[out] data 调用者提供的 64 字节缓冲区。
 * @return HAL_OK，表示测试帧已复制。
 */
unsigned HAL_FDCAN_GetRxMessage(Handle *h,unsigned fifo,FDCAN_RxHeaderTypeDef *header,uint8_t *data)
{
    (void)h;(void)fifo;memset(header,0,sizeof(*header));
    header->Identifier=incoming.identifier;header->DataLength=incoming.length;
    header->IdType=incoming.extended;header->RxFrameType=incoming.remote;
    header->FDFormat=incoming.fd?FDCAN_FD_CAN:FDCAN_CLASSIC_CAN;
    header->BitRateSwitch=incoming.brs?FDCAN_BRS_ON:FDCAN_BRS_OFF;
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
/** @brief 规划查询的测试发布源，实际序号由测试显式控制。 */
static uint16_t motion_sequence = 7;
/** @brief 发布源是否有效，用于验证失能及缺失快照。 */
static bool motion_valid = true;
/**
 * @brief 返回确定性内部规划替身，CAN编解码与锁存代码仍使用生产函数。
 * @param[out] values 三个SI参考，故意令规划速度与速度参考不同。
 * @param[out] sequence 当前设备发布序号。
 * @return 配置的有效性，失败时输出不变。
 */
static bool MotorOuterLoop_GetMotionTelemetry(float values[3], uint16_t *sequence)
{
    if (!motion_valid) { return false; }
    values[0] = .25f; values[1] = -.5f; values[2] = -.75f;
    *sequence = motion_sequence;
    return true;
}
#include "can_reply_actual.inc"
#include "can_reply_vectors.h"

/** @brief 清空模拟控制状态、回复槽及计数，确保每个用例相互独立。 */
static void reset_fixture(void)
{
    memset(&CANMsg,0,sizeof(CANMsg));memset(&MotorControl,0,sizeof(MotorControl));
    memset((void *)&CAN_Diagnostics,0,sizeof(CAN_Diagnostics));
    memset((void *)&can_reply,0,sizeof(can_reply));
    CANMsg.node_id=4;CANMsg.baudrate=1000;MotorControl.speed_limit=20;
    ModeLast=Motor_Disable;phase_off=true;boot_pending=false;filter_failure=false;mode_allowed=true;
    tx_result=tx_attempts=sent_count=pending=crc_calls=uid_calls=0;
    filter_calls=limit_updates=injected=0;filter_node=4;send_hook=crc_hook=NULL;
    status_rate=0;sent_fd=sent_brs=false;
    continuous_speed_reads=0;
    memset(&OnBoard_Encoder,0,sizeof(OnBoard_Encoder));
}
/**
 * @brief 构造指定格式的四字节输入帧并执行生产协议入口。
 * @param[in] node 三位节点号。
 * @param[in] op 参数号。
 * @param[in] data 调用期有效的四字节输入数据。
 * @param[in] fd 是否采用 CAN FD。
 * @param[in] brs 是否设置 FD 速率切换。
 * @note 仅操作内存夹具，可能改变模拟目标或占用回复槽。
 */
static void request_format(unsigned node,unsigned op,const uint8_t data[4],bool fd,bool brs)
{
    memset(&incoming,0,sizeof(incoming));incoming.identifier=(node<<8)|op;
    incoming.fd=fd;incoming.brs=brs;
    incoming.length=4;memcpy(incoming.data,data,4);CANRxIRQHandler();
}
/**
 * @brief 使用历史 FD/BRS 格式执行原维护协议用例。
 * @param[in] node 三位节点号。
 * @param[in] op 参数号。
 * @param[in] data 调用期有效的四字节输入数据。
 */
static void request(unsigned node,unsigned op,const uint8_t data[4])
{ request_format(node,op,data,true,true); }
/**
 * @brief 将 float32 数值编码为大端请求并选择帧格式。
 * @param[in] node 三位节点号。
 * @param[in] op 参数号。
 * @param[in] value 输入数值。
 * @param[in] fd 是否采用 CAN FD。
 * @param[in] brs 是否设置 FD 速率切换。
 */
static void scalar_format(unsigned node,unsigned op,float value,bool fd,bool brs)
{
    uint32_t bits=FloatToIntBit(value);
    uint8_t data[4]={(uint8_t)(bits>>24),(uint8_t)(bits>>16),(uint8_t)(bits>>8),(uint8_t)bits};
    request_format(node,op,data,fd,brs);
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

/**
 * @brief 验证请求格式跟随、忙槽冻结和经典帧维护请求的边界。
 * @note 控制结构仅设定测试哨兵值，合法只读查询不得改变这些值。
 */
static void check_reply_formats(void)
{
    const uint8_t node4[4]={0x40,0x80,0,0}, nonce[4]={1,2,3,4};
    const uint8_t crc[8]={1,2,3,4,0x12,0x34,0x56,0x78};
    for (unsigned format=0;format<3;++format) {
        bool fd=format!=0,brs=format==2;
        reset_fixture();
        scalar_format(4,CAN_GET_NODE_ID,0,fd,brs);
        assert(can_reply.state==CAN_REPLY_READY);
        tx_result=1;CAN_SendMessage();assert(sent_count==0);
        scalar_format(4,CAN_GET_NODE_ID,0,!fd,false);
        tx_result=0;CAN_SendMessage();check_sent(0x409,node4,4);
        assert(sent_fd==fd && sent_brs==brs && CAN_Diagnostics.reply_busy==1);
    }
    reset_fixture();
    request_format(4,CAN_GET_PARAMETER_CRC,nonce,false,false);
    CAN_SendMessage();check_sent(0x47c,crc,8);assert(!sent_fd && !sent_brs);
    reset_fixture();
    request_format(4,CAN_GET_DEVICE_UID,nonce,false,false);
    CAN_SendMessage();assert(sent_count==0 && uid_calls==0 && can_reply.state==CAN_REPLY_FREE);
    {const uint8_t boot[4]={'B','O','O','T'};
     request_format(4,CAN_ENTER_BOOT,boot,false,false);assert(!boot_pending);}
    assert(CAN_Diagnostics.rx_rejected==2);
    assert(CanMotorStatus_Configure(20));
    scalar_format(4,CAN_SET_STATUS_STREAM,10,false,false);
    assert(status_rate==20 && can_reply.state==CAN_REPLY_FREE);
    scalar_format(4,CAN_SET_STATUS_STREAM,0,false,false);
    CAN_SendMessage();assert(status_rate==0 && !sent_fd && !sent_brs);

    reset_fixture();
    MotorControl.ModeNow=Position_Mode;MotorControl.posRef=1.25f;
    MotorControl.speedRef=-2.0f;MotorControl.iqRef=0.5f;MotorControl.current_limit=3.0f;
    assert(CanMotorStatus_Configure(20));
    MotorControl_TypeDef motor_before=MotorControl;
    const CAN_PARAM_ID reads[]={CAN_GET_MODE,CAN_GET_NODE_ID,CAN_GET_POS2_FILT,
        CAN_GET_SPEED2_FILT,CAN_GET_ERROR,CAN_GET_VBUS,CAN_GET_TEMPERATURE_PROTECTION};
    for (unsigned i=0;i<sizeof(reads)/sizeof(reads[0]);++i) {
        scalar_format(4,reads[i],0,false,false);CAN_SendMessage();
        assert(!sent_fd && !sent_brs && status_rate==20);
        assert(memcmp(&motor_before,&MotorControl,sizeof(MotorControl))==0);
    }
    for (unsigned format=0;format<3;++format) {
        scalar_format(4,CAN_GET_SOFTWARE_VERSION,0,format!=0,format==2);CAN_SendMessage();
        const uint32_t expected=FIRMWARE_VERSION_CODE;
        const uint8_t bytes[4]={(uint8_t)(expected>>24),(uint8_t)(expected>>16),
            (uint8_t)(expected>>8),(uint8_t)expected};
        check_sent(0x46f,bytes,4);
        assert(sent_fd==(format!=0) && sent_brs==(format==2));
    }
    puts("PASS formats: classic/FD/BRS frozen through busy retry; classic UID/BOOT/stream gates; read-only targets and version preserved");
}

/**
 * @brief 用真实接收、参数选择及定点编码验证mode3连续低速反馈和默认兼容。
 * 覆盖正负2度/s、低于线路量化步长、正常高速度及非有限哨兵；不将0.01rad/s
 * 协议分辨率误称为精确2度/s。请求始终为经典帧，其他模式不得读取连续getter。
 */
static void check_position_speed_reply(void)
{
    const uint8_t query[4]={0};
    const float low_speeds[]={0.034906585f,-0.034906585f,0.009f,-0.009f};
    const int32_t quantized[]={3,-3,0,0};
    for (unsigned i=0;i<sizeof(low_speeds)/sizeof(low_speeds[0]);++i) {
        reset_fixture();CANMsg.node_id=3;MotorControl.ModeNow=Position_Mode;
        OnBoard_Encoder.vel_mech=0.0f;
        OnBoard_Encoder.vel_mech_continuous=low_speeds[i];
        MotorControl_TypeDef before=MotorControl;
        Encoder_TypeDef encoder_before=OnBoard_Encoder;
        request_format(3,CAN_GET_SPEED2_FILT,query,false,false);CAN_SendMessage();
        uint32_t expected=(uint32_t)(POSITION_EXTERNAL_REFERENCE_ENABLE?quantized[i]:0);
        const uint8_t wire[4]={(uint8_t)(expected>>24),(uint8_t)(expected>>16),
            (uint8_t)(expected>>8),(uint8_t)expected};
        check_sent(0x33f,wire,4);
        assert(!sent_fd && !sent_brs && sent_count==1);
        assert(continuous_speed_reads==(POSITION_EXTERNAL_REFERENCE_ENABLE?1U:0U));
        assert(memcmp(&before,&MotorControl,sizeof(before))==0);
        assert(memcmp(&encoder_before,&OnBoard_Encoder,sizeof(encoder_before))==0);
    }
    const unsigned modes[]={Motor_Disable,Current_Mode,Speed_Mode,Position_Mode,Position_Impedance_Mode};
    for (unsigned i=0;i<sizeof(modes)/sizeof(modes[0]);++i) {
        reset_fixture();CANMsg.node_id=3;MotorControl.ModeNow=(ModeNow_TypeDef)modes[i];
        OnBoard_Encoder.vel_mech=-0.25f;OnBoard_Encoder.vel_mech_continuous=0.4f;
        bool continuous=POSITION_EXTERNAL_REFERENCE_ENABLE && modes[i]==Position_Mode;
        uint32_t expected=(uint32_t)(continuous?40:-25);
        const uint8_t wire[4]={(uint8_t)(expected>>24),(uint8_t)(expected>>16),
            (uint8_t)(expected>>8),(uint8_t)expected};
        request_format(3,CAN_GET_SPEED2_FILT,query,false,false);CAN_SendMessage();
        check_sent(0x33f,wire,4);
        assert(continuous_speed_reads==(continuous?1U:0U));
    }
    const float common_speeds[]={0.4f,-0.4f,NAN,INFINITY};
    for (unsigned i=0;i<sizeof(common_speeds)/sizeof(common_speeds[0]);++i) {
        reset_fixture();CANMsg.node_id=3;MotorControl.ModeNow=Position_Mode;
        OnBoard_Encoder.vel_mech=OnBoard_Encoder.vel_mech_continuous=common_speeds[i];
        uint32_t expected=i==0?40U:(i==1?(uint32_t)-40:UINT32_C(0x80000000));
        const uint8_t wire[4]={(uint8_t)(expected>>24),(uint8_t)(expected>>16),
            (uint8_t)(expected>>8),(uint8_t)expected};
        request_format(3,CAN_GET_SPEED2_FILT,query,false,false);CAN_SendMessage();
        check_sent(0x33f,wire,4);
    }
    puts("PASS speed reply: external mode3 continuous +/-2deg/s, legacy modes, centi quantization and nonfinite sentinel");
}

/**
 * @brief 执行实际协议与端口回归，不访问任何硬件。
 * @return 全部断言通过返回 0，否则由断言终止进程。
 */
/** @brief 三个查询沿用经典读帧，回复锁存同序号；规划速度与参考不得混淆。 */
static void test_motion_queries(void)
{
    const uint8_t position[8] = {1,1,0,7,0x3e,0x80,0,0};
    const uint8_t velocity[8] = {1,1,0,7,0xbf,0,0,0};
    const uint8_t reference[8] = {1,1,0,7,0xbf,0x40,0,0};
    reset_fixture(); MotorControl.ModeNow=Position_Mode; motion_sequence=7;
    scalar_format(4,CAN_GET_POSITION_PLANNED,0,false,false); CAN_SendMessage();
    check_sent(0x493,position,8); assert(!sent_fd && !sent_brs);
    motion_sequence=8; /* 后两包仍必须使用首查询锁存的序号7。 */
    scalar_format(4,CAN_GET_VELOCITY_PLANNED,0,false,false); CAN_SendMessage();
    check_sent(0x495,velocity,8);
    scalar_format(4,CAN_GET_VELOCITY_REFERENCE,0,false,false); CAN_SendMessage();
    check_sent(0x497,reference,8);
    MotorControl.ModeNow=Motor_Disable;
    scalar_format(4,CAN_GET_POSITION_PLANNED,0,false,false); CAN_SendMessage();
    assert(sent_data[1]==0);
}

int main(void)
{
    test_motion_queries();
#if defined(TEST_SPEED_REPLY_ONLY)
    check_position_speed_reply();
    return 0;
#endif
    const uint8_t current[2]={0xfb,0x1e}, node4[4]={0x40,0x80,0,0};
    const uint8_t nonce[4]={1,2,3,4}, crc[8]={1,2,3,4,0x12,0x34,0x56,0x78};
    uint8_t uid[16]={1,2,3,4};
    check_reply_formats();
    check_position_speed_reply();
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
      assert(CAN_Diagnostics.node_rejected==4 && CANMsg.node_id==4 && filter_calls==0);
    }
    MotorControl.ModeNow=Speed_Mode;scalar(4,CAN_SET_NODE_ID,3);
    MotorControl.ModeNow=Motor_Disable;ModeLast=Speed_Mode;scalar(4,CAN_SET_NODE_ID,3);
    ModeLast=Motor_Disable;phase_off=false;scalar(4,CAN_SET_NODE_ID,3);
    assert(CAN_Diagnostics.node_rejected==7 && filter_calls==0 && CANMsg.node_id==4);
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
    scalar(4,CAN_GET_NODE_ID,FLT_MAX);assert(CAN_Diagnostics.rx_rejected==1);
    CAN_Diagnostics.rx_rejected=UINT32_MAX;scalar(4,0xfe,0);
    assert(CAN_Diagnostics.rx_rejected==UINT32_MAX);
    CAN_SendMessage_Update(CAN_GET_NODE_ID,4);CAN_Diagnostics.reply_busy=UINT32_MAX;
    CAN_SendMessage_Update(CAN_GET_NODE_ID,4);assert(CAN_Diagnostics.reply_busy==UINT32_MAX);
    CAN_Diagnostics.tx_busy=UINT32_MAX;tx_result=1;CAN_SendMessage();assert(CAN_Diagnostics.tx_busy==UINT32_MAX);
    puts("PASS CAN handoff: immutable reply/nonce/ID, one CRC per request, retry, BOOT, stopped node transaction, baud GET");
    return 0;
}
