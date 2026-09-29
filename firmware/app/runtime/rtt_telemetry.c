#include "rtt_telemetry.h"

#include <math.h>

#include "SEGGER_RTT.h"
#include "angle_feedback.h"
#include "control_config.h"
#include "motor_execution.h"
#include "motor_types.h"
#include "utils.h"

/* 定标：物理量乘以尺度后饱和编码为 int16 计数，不是测量精度。 */
#define RTT_POSITION_SCALE_COUNTS_PER_RAD (32768.0f / _PI)
#define RTT_SPEED_SCALE_COUNTS_PER_RPM10 (300.0f / _PI)
#define RTT_CURRENT_SCALE_COUNTS_PER_A 1000.0f

/* 线上布局：4 项小端 int16，共 8 字节，字段顺序即 JScope 描述符顺序。 */
typedef struct
{
    int16_t position;     /* 机械位置，Q15 单圈：raw × 180/32768 度 */
    int16_t speed;        /* 机械转速：raw / 10 rpm */
    int16_t iq_reference; /* iq_a 指令：raw / 1000 A */
    int16_t iq_feedback;  /* iq_a 反馈：raw / 1000 A */
} RttTelemetryFrame;

typedef char RTT_TelemetryFrame_SizeMustBe8Bytes[(sizeof(RttTelemetryFrame) == 8U) ? 1 : -1];

/* 非有限值与超量程均收敛为可解码的饱和计数，避免回绕。 */
static int16_t rtt_encode_int16(float value, float scale)
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
static float rtt_wrap_single_turn(float angle)
{
    return angle - 2.0f * _PI * floorf(angle / (2.0f * _PI) + 0.5f);
}

/* 分频计数跨调用保持；本函数不修改任何控制状态。 */
void RTT_Sampling(MotorControlMode mode,
                  float position_rad,
                  float speed_rad_s,
                  float iq_reference_a,
                  float iq_feedback_a)
{
    static uint32_t rtt_divider_count;
    RttTelemetryFrame frame;

    if (++rtt_divider_count < RTT_SAMPLE_DIVIDER)
    {
        return;
    }
    /* 2 kHz 伺服已在 TIM7 监督上下文执行：本帧不再与慢估计叠加，
     * 仅保留"位置模式外环未就绪时延后一帧"的保护。 */
#if CASCADE_POSITION_LOOP_DIVIDER > 1U
    if (mode == MOTOR_CONTROL_POSITION && !MotorOuterLoop_IsReady())
    {
        return;
    }
#endif
    rtt_divider_count = 0;

    frame.position =
        rtt_encode_int16(rtt_wrap_single_turn(position_rad), RTT_POSITION_SCALE_COUNTS_PER_RAD);
    frame.speed = rtt_encode_int16(speed_rad_s, RTT_SPEED_SCALE_COUNTS_PER_RPM10);
    frame.iq_reference = rtt_encode_int16(iq_reference_a, RTT_CURRENT_SCALE_COUNTS_PER_A);
    frame.iq_feedback = rtt_encode_int16(iq_feedback_a, RTT_CURRENT_SCALE_COUNTS_PER_A);
    (void)SEGGER_RTT_Write(1, &frame, sizeof(frame));
}
