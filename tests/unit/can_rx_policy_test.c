/* 运行真实解码、分派、回复队列、心跳和平台端口；只替换外部控制/硬件边界。 */
#define CAN_REAL_STATUS
#define main legacy_handoff_main
#include "can_reply_handoff_test.c"
#undef main
#include "firmware/communication/protocol/can_motor_status.c"
#include "can_admission_vectors.h"

static void prepare_test(void)
{
    reset_fixture();
    CanMotorStatus_Init();
    MotorControl.axis_profile_valid = true;
    MotorControl.current_limit = 7.0f;
    MotorControl.speed_limit = 20.0f;
    MotorControl.ModeNow = Position_Mode;
    MotorControl.posRef = 0.25f;
    MotorControl.iqRef = 0.5f;
    MotorControl.position_speed_Kp = 0.6f;
    CANMsg.node_id = 3;
    CANMsg.can_hb_count = 123;
}

static void check_golden_vectors(void)
{
    for(unsigned i=0;i<sizeof(admission_vectors)/sizeof(admission_vectors[0]);++i) {
        prepare_test();
        memset(&incoming,0,sizeof(incoming));
        incoming.identifier=admission_vectors[i].id;
        incoming.length=admission_vectors[i].length;
        memcpy(incoming.data,admission_vectors[i].request,incoming.length);
        CANRxIRQHandler();CAN_SendMessage();
        if(admission_vectors[i].reply_length) {
            assert(sent_count==1 && !sent_fd && !sent_brs);
            check_sent(admission_vectors[i].id,admission_vectors[i].reply,admission_vectors[i].reply_length);
            assert(CANMsg.can_hb_count==0);
        } else {
            assert(sent_count==0 && CANMsg.can_hb_count==123);
        }
    }
}

static void rejected_unchanged(unsigned op, float value)
{
    MotorControl_TypeDef before = MotorControl;
    CAN_PARAM_ID previous_id = CANMsg.rx_param_id;
    float previous_data = CANMsg.rx_data;
    CANMsg.can_hb_count = 123;
    CANMsg.can_rx_en = false;
    scalar_format(3, op, value, false, false);
    assert(CANMsg.can_hb_count == 123 && !CANMsg.can_rx_en);
    assert(CANMsg.rx_param_id == previous_id && memcmp(&CANMsg.rx_data,&previous_data,sizeof(previous_data)) == 0);
    assert(memcmp(&before, &MotorControl, sizeof(before)) == 0);
    CAN_SendMessage(); /* 拒绝时允许原协议 NAK，但不能喂狗。 */
}

static void check_validation(void)
{
    const unsigned unsupported[] = {0x0c,0x26,0x27,0x2c,0x2e,0x30,0x32,
        0x34,0x36,0x38,0x3a,0x3e,0x40,0x42,0x44,0x46,0x48,0x4a,0x4c,
        0x7d,0x7e,0xfd,0xfe,0xff};
    const unsigned reads[] = {0x01,0x09,0x5a,0x5c,0x60,0x62,0x6a,0x6c,0x6e,0x7a,0x7b,0x8e};
    prepare_test();
    MotorControl.ErrorNow = CAN_DisConnect;
    for (unsigned i = 0; i < sizeof(unsupported)/sizeof(unsupported[0]); ++i) {
        rejected_unchanged(unsupported[i], 0);
    }
    for (unsigned i = 0; i < sizeof(reads)/sizeof(reads[0]); ++i) {
        CANMsg.can_hb_count = 123;
        scalar_format(3, reads[i], NAN, false, false);
        assert(CANMsg.can_hb_count == 0 && CANMsg.can_rx_en);
        assert(MotorControl.ErrorNow == No_Error);
        CAN_SendMessage();
    }
    rejected_unchanged(CAN_SET_MODE, 2.5f);
    rejected_unchanged(CAN_SET_POLEPARIS, 21.5f);
    rejected_unchanged(CAN_SET_ENCODER_REVERSE, 0.5f);
    rejected_unchanged(CAN_SET_CAN_HB, 500.5f);
    rejected_unchanged(CAN_SET_SPEED_KP, NAN);
    rejected_unchanged(CAN_SET_SPEED_KP, INFINITY);
    rejected_unchanged(CAN_SET_SPEED_KP, -1);
    rejected_unchanged(CAN_SET_SPEED_LIMIT, 100);
    rejected_unchanged(CAN_SET_LOAD_PROFILE, 3); /* 真实返回值为拒绝的业务边界替身。 */
    rejected_unchanged(CAN_SET_TEMPERATURE_PROTECTION, 1);
    rejected_unchanged(CAN_SET_SPEED_FF_GAIN, 1);
    rejected_unchanged(CAN_APPLY_FRICTION_MODEL, 1);
    rejected_unchanged(CAN_SET_POSITION_SPEED_KP, -1);
    rejected_unchanged(CAN_SET_POSITION_SPEED_KI, -1);
    scalar_format(3, CAN_SET_POSITION_SPEED_KP, 0.6f, false, false);
    assert(MotorControl.position_speed_Kp == 0.6f && CANMsg.can_hb_count == 0);
    scalar_format(3, CAN_SET_POSITION_SPEED_KI, 1.2f, false, false);
    assert(MotorControl.position_speed_Ki == 1.2f);

    /* 完整节点、RTR/IDE/BRS、精确长度和无效哨兵。 */
    for (unsigned n=0; n<=64; ++n) {
        if (n == 4) { continue; }
        memset(&incoming,0,sizeof(incoming)); incoming.identifier=0x309;incoming.length=n;
        CANMsg.can_hb_count=123;CANRxIRQHandler();assert(CANMsg.can_hb_count==123);
    }
    incoming.length=4;
    const unsigned ids[]={0x209,0x409,0x709,0x8309,0x37d,0x37e,0x3fd,0x3fe};
    for (unsigned i=0;i<sizeof(ids)/sizeof(ids[0]);++i) {
        incoming.identifier=ids[i];CANRxIRQHandler();assert(CANMsg.can_hb_count==123);
    }
    incoming.identifier=0x309;incoming.extended=true;CANRxIRQHandler();
    incoming.extended=false;incoming.remote=true;CANRxIRQHandler();
    incoming.remote=false;incoming.brs=true;CANRxIRQHandler();
    assert(CANMsg.can_hb_count==123);

    /* 写入限值在模式切换前判定；业务拒绝后不写目标。 */
    prepare_test();MotorControl.ModeNow=Motor_Disable;mode_allowed=false;
    memset(&incoming,0,sizeof(incoming));incoming.identifier=0x302;incoming.length=2;
    incoming.data[0]=0x1f;incoming.data[1]=0x40; /* 8 A 超过 7 A。 */
    CANRxIRQHandler();assert(MotorControl.ModeNow==Motor_Disable && MotorControl.iqRef==0.5f);
    incoming.data[0]=0x03;incoming.data[1]=0xe8; /* 1 A，模式拒绝。 */
    CANRxIRQHandler();assert(MotorControl.ModeNow==Motor_Disable && MotorControl.iqRef==0.5f);
    incoming.data[0]=0x80;incoming.data[1]=0;CANRxIRQHandler();
    assert(CANMsg.can_hb_count==123);
    incoming.identifier=0x304;incoming.length=4;incoming.data[0]=0;incoming.data[1]=0;
    incoming.data[2]=0x0b;incoming.data[3]=0xb8; /* 30 rad/s，超限。 */
    CANRxIRQHandler();assert(MotorControl.ModeNow==Motor_Disable && MotorControl.speedRef==0);
    incoming.data[2]=0;incoming.data[3]=100;CANRxIRQHandler(); /* 1 rad/s，模式拒绝。 */
    assert(MotorControl.ModeNow==Motor_Disable && MotorControl.speedRef==0);
    incoming.identifier=0x306;incoming.data[2]=0;incoming.data[3]=100;
    CANRxIRQHandler();assert(MotorControl.ModeNow==Motor_Disable && MotorControl.posRef==0.25f);
    MotorControl.axis_profile.magic=1;MotorControl.axis_profile.minimum_position_rad=-0.05f;
    MotorControl.axis_profile.maximum_position_rad=0.05f;mode_allowed=true;
    CANRxIRQHandler();assert(MotorControl.ModeNow==Motor_Disable && MotorControl.posRef==0.25f);
    incoming.data[3]=20;CANRxIRQHandler();assert(MotorControl.ModeNow==Position_Mode);
    assert(fabsf(MotorControl.posRef-0.02f)<0.00001f && CANMsg.can_hb_count==0);
    MotorControl.ModeNow=Motor_Disable;
    incoming.identifier=0x302;incoming.length=2;incoming.data[0]=3;incoming.data[1]=0xe8;
    CANRxIRQHandler();assert(MotorControl.ModeNow==Current_Mode && MotorControl.iqRef==1.0f);
    MotorControl.ModeNow=Motor_Disable;
    incoming.identifier=0x304;incoming.length=4;incoming.data[0]=0;incoming.data[1]=0;
    incoming.data[2]=0;incoming.data[3]=100;
    CANRxIRQHandler();assert(MotorControl.ModeNow==Speed_Mode && MotorControl.speedRef==1.0f);
}

static void check_expiry(void)
{
    prepare_test();CANMsg.can_hb_set=500;CANMsg.can_rx_en=true;CANMsg.can_hb_count=0;
    for (unsigned i=0;i<500;++i) {
        scalar_format(3,0x7d,0,false,false);
        CAN_DisConnect_Handle();
    }
    assert(CANMsg.can_hb_count==500 && MotorControl.ErrorNow==CAN_DisConnect);
    rejected_unchanged(0x7e,0);
    assert(MotorControl.ErrorNow==CAN_DisConnect);
    scalar_format(3,CAN_GET_NODE_ID,0,false,false);
    assert(MotorControl.ErrorNow==No_Error && CANMsg.can_hb_count==0);
}

static void check_deployment(void)
{
    uint8_t data[48]={0},nonce[4]={1,2,3,4};
    prepare_test();
    for (unsigned fd=0;fd<2;++fd) {
        for (unsigned brs=0;brs<2;++brs) {
            can_reply_read=can_reply_write=0;CANMsg.can_hb_count=123;
            scalar_format(3,CAN_GET_NODE_ID,0,fd,brs);
            bool allowed=(!brs || fd) && (!VECTOR_CAN_CLASSIC_ONLY || !fd);
            assert((CANMsg.can_hb_count==0)==allowed);
            if (allowed) { CAN_SendMessage();assert(sent_fd==fd && sent_brs==brs); }
        }
    }
    if (VECTOR_CAN_CLASSIC_ONLY) {
        rejected_unchanged(CAN_SET_STATUS_STREAM,20);
        assert(!CanMotorStatus_Configure(1) && !CanMotorStatus_Configure(20));
        configuration=512U|200U; /* 模拟旧配置仍置位。 */
        assert(CanMotorStatus_Rate()==0);
        uint16_t id=0;
        assert(!CanMotorStatus_Prepare(10000,3,&id,data,sizeof(data)));
        assert(!comm_hw_can_try_send_status(0x7f3,data,48));
        assert(!comm_hw_can_try_send_reply(0x309,data,4));
        assert(!comm_hw_can_try_send_reply_format(0x309,data,4,true,false));
        assert(!comm_hw_can_try_send_reply_format(0x309,data,4,true,true));
        assert(!comm_hw_can_try_send_reply_format(0x309,data,16,false,false));
        request_format(3,CAN_GET_DEVICE_UID,nonce,false,false);assert(uid_calls==0);
        const uint8_t boot[4]={'B','O','O','T'};
        request_format(3,CAN_ENTER_BOOT,boot,false,false);assert(!boot_pending);
        request_format(3,CAN_ENTER_BOOT,boot,true,true);assert(!boot_pending);
        can_reply_write=can_reply_read+1U;can_replies[can_reply_read % CAN_REPLY_CAPACITY].state=CAN_REPLY_READY;can_replies[can_reply_read % CAN_REPLY_CAPACITY].fd=true;can_replies[can_reply_read % CAN_REPLY_CAPACITY].length=4;
        unsigned count=sent_count;
        CAN_SendMessage();
        assert(sent_count==count && can_reply_read==can_reply_write);
    } else {
        assert(CanMotorStatus_Configure(20) && CanMotorStatus_Rate()==20);
        request_format(3,CAN_GET_DEVICE_UID,nonce,true,false);
        CAN_SendMessage();assert(sent_length==16 && sent_fd && !sent_brs);
    }
    can_reply_read=can_reply_write=0;
    request_format(3,CAN_GET_PARAMETER_CRC,nonce,false,false);
    tx_result=1;CAN_SendMessage();assert(can_replies[can_reply_read % CAN_REPLY_CAPACITY].state==CAN_REPLY_READY);
    scalar_format(3,CAN_GET_NODE_ID,0,true,true); /* 冻结的经典重试不被替换。 */
    tx_result=0;CAN_SendMessage();assert(sent_length==8 && !sent_fd && !sent_brs);
    can_reply_read=can_reply_write=0;
    CAN_SendMessage_Update(CAN_GET_NODE_ID,3);CAN_SendMessage();
    assert(sent_fd==!VECTOR_CAN_CLASSIC_ONLY && sent_brs==!VECTOR_CAN_CLASSIC_ONLY);
}

/**
 * @brief 在离散毫秒调度中检查 50 Hz 轨迹写入和五类各 50 Hz 的串行查询。
 * @note 使用真实 RX/分派/队列/HAL 端口；总线与前台延迟是假设值，不能作为 MCU 时序测量。
 */
static void check_sustained_polling(void)
{
    const unsigned ids[5] = {0x41,0x3f,0x39,0x01,0x4d};
    const unsigned source_ids[6] = {0x1fd,0x1fe,0x2fd,0x2fe,0x27d,0x27e};
    const unsigned cycles = 6000U;
    for (unsigned busy_ms=0;busy_ms<=3;++busy_ms) {
        unsigned per_channel[5]={0},requests=0,writes=0,max_batch_ms=0;
        prepare_test();
        CANMsg.can_hb_set=500;
        CANMsg.can_hb_count=0;
        for (unsigned cycle=0;cycle<cycles;++cycle) {
            unsigned elapsed_ms=0;
            int32_t target_mrad=(int32_t)(cycle%601U)-300;
            uint32_t raw=(uint32_t)target_mrad;
            const uint8_t target[4]={(uint8_t)(raw>>24),(uint8_t)(raw>>16),
                (uint8_t)(raw>>8),(uint8_t)raw};
            request_format(3,CAN_SET_POS,target,false,false);
            ++writes;
            assert(MotorControl.ModeNow==Position_Mode && can_reply_read==can_reply_write);
            for (unsigned slot=0;slot<5;++slot) {
                uint8_t expected[4]={0};
                unsigned length=4;
                /* 二进制可精确表示的输入，独立按既有线单位构造期望值。 */
                OnBoard_Encoder.theta_mech=((int32_t)(cycle%129U)-64)/64.0f;
                OnBoard_Encoder.vel_mech=(cycle%2U)?-0.5f:0.25f;
                FOC.Iq=((int32_t)(cycle%65U)-32)/8.0f;
                int32_t value=0;
                if (slot==0) { value=(int32_t)(OnBoard_Encoder.theta_mech*1000.0f); }
                if (slot==1) { value=(int32_t)(OnBoard_Encoder.vel_mech*100.0f); }
                if (slot==2) { value=(int32_t)(FOC.Iq*1000.0f);length=2; }
                if (slot==3) { raw=FloatToIntBit((float)Position_Mode); }
                else if (slot==4) { raw=FloatToIntBit((float)No_Error); }
                else { raw=(uint32_t)value; }
                for(unsigned byte=0;byte<length;++byte) {
                    expected[byte]=(uint8_t)(raw>>(8U*(length-1U-byte)));
                }
                unsigned before=sent_count;
                scalar_format(3,ids[slot],0,false,false);
                ++requests;
                assert(can_replies[can_reply_read % CAN_REPLY_CAPACITY].state==CAN_REPLY_READY && CANMsg.can_hb_count==0);
                for(unsigned delay=0;delay<=busy_ms;++delay) {
                    /* 共线源擎 PDO 不应刷新本节点心跳或改写已排队回复。 */
                    unsigned old_count=CANMsg.can_hb_count;
                    memset(&incoming,0,sizeof(incoming));
                    incoming.identifier=source_ids[(cycle+slot+delay)%6U];incoming.length=8;
                    CANRxIRQHandler();assert(CANMsg.can_hb_count==old_count);
                    pending=delay<busy_ms?1U:0U;
                    CAN_SendMessage();
                    if(delay<busy_ms) {
                        assert(sent_count==before && can_replies[can_reply_read % CAN_REPLY_CAPACITY].state==CAN_REPLY_READY);
                    }
                    CAN_DisConnect_Handle();
                    ++elapsed_ms;
                }
                assert(sent_count==before+1U && can_reply_read==can_reply_write);
                assert(!sent_fd && !sent_brs);
                check_sent(0x300U|ids[slot],expected,length);
                ++per_channel[slot];
            }
            assert(elapsed_ms<=20U);
            if(elapsed_ms>max_batch_ms) { max_batch_ms=elapsed_ms; }
            while(elapsed_ms<20U) { CAN_DisConnect_Handle();++elapsed_ms; }
            assert(MotorControl.ErrorNow==No_Error);
        }
        assert(requests==cycles*5U && sent_count==requests && writes==cycles);
        assert(CAN_Diagnostics.reply_busy==0U);
        for(unsigned slot=0;slot<5;++slot) { assert(per_channel[slot]==cycles); }
        printf("PASS 120s discrete polling: busy=%ums, queries=%u, replies=%u, position_writes=%u, each_channel=%u, max_batch=%ums, lost=0\n",
            busy_ms,requests,sent_count,writes,per_channel[0],max_batch_ms);
    }
}

int main(void)
{
    check_validation();check_expiry();check_deployment();check_golden_vectors();
    check_sustained_polling();
    printf("PASS real RX/dispatch/watchdog/TX: classic_only=%d\n",VECTOR_CAN_CLASSIC_ONLY);
    return 0;
}
