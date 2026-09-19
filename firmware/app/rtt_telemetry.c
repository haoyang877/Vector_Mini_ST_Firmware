#include "rtt_telemetry.h"

#include "common_inc.h"
#include "SEGGER_RTT.h"
#include "motor_state.h"

/* 定标：物理量乘以尺度后饱和编码为 int16 计数，不是测量精度。 */
#define RTT_POSITION_SCALE_COUNTS_PER_RAD (32768.0f / _PI)
#define RTT_SPEED_SCALE_COUNTS_PER_RPM10 (300.0f / _PI)
#define RTT_CURRENT_SCALE_COUNTS_PER_A 1000.0f

/* 线上布局：4 项小端 int16，共 8 字节，字段顺序即 JScope 描述符顺序。 */
typedef struct
{
    int16_t position;     /* 机械位置，Q15 单圈：raw × 180/32768 度 */
    int16_t speed;        /* 机械转速：raw / 10 rpm */
    int16_t iq_reference; /* Iq 指令：raw / 1000 A */
    int16_t iq_feedback;  /* Iq 反馈：raw / 1000 A */
} RTT_TelemetryFrame_TypeDef;

typedef char
    RTT_TelemetryFrame_SizeMustBe8Bytes[(sizeof(RTT_TelemetryFrame_TypeDef) == 8U) ? 1 : -1];

/* 非有限值与超量程均收敛为可解码的饱和计数，避免回绕。 */
static int16_t RTT_EncodeInt16(float value, float scale)
{
    float scaled;

    if (!isfinite(value) || !isfinite(scale))
    {
        return 0;
    }
    scaled = value * scale;
    if (scaled > 32767.0f)
    {
        return 32767;
    }
    if (scaled < -32768.0f)
    {
        return -32768;
    }

    return (int16_t)scaled;
}

/* 机械位置折到单圈 [-π, π)：Q15 编码不承诺多圈绝对位置。 */
static float RTT_WrapSingleTurn(float angle)
{
    return angle - 2.0f * _PI * floorf(angle / (2.0f * _PI) + 0.5f);
}

/* 分频计数跨调用保持；本函数不修改任何控制状态。 */
void RTT_Sampling(void)
{
    static uint32_t rtt_divider_count;
    RTT_TelemetryFrame_TypeDef frame;

    if (++rtt_divider_count < RTT_SAMPLE_DIVIDER)
    {
        return;
    }
    if (Encoder_DidUpdateVelocity(&OnBoard_Encoder))
    {
        return;
    }
    /* 同一 IRQ 不叠加帧编码与 2 kHz 伺服：冲突帧延后一个快速周期，
     * 后续帧保持正常节拍，且不延迟电流/PWM 更新。 */
#if CASCADE_POSITION_LOOP_DIVIDER > 1U
    if (MotorControl.ModeNow == Position_Mode && !MotorOuterLoop_IsReady())
    {
        return;
    }
#endif
    rtt_divider_count = 0;

    frame.position = RTT_EncodeInt16(RTT_WrapSingleTurn(Encoder_GetMecPos(&OnBoard_Encoder)),
                                     RTT_POSITION_SCALE_COUNTS_PER_RAD);
    frame.speed =
        RTT_EncodeInt16(Encoder_GetMecVel(&OnBoard_Encoder), RTT_SPEED_SCALE_COUNTS_PER_RPM10);
    frame.iq_reference = RTT_EncodeInt16(MotorControl.iqRef, RTT_CURRENT_SCALE_COUNTS_PER_A);
    frame.iq_feedback = RTT_EncodeInt16(FOC.Iq, RTT_CURRENT_SCALE_COUNTS_PER_A);
    (void)SEGGER_RTT_Write(1, &frame, sizeof(frame));
}
