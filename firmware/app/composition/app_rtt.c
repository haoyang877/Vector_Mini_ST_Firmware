/**
 * @file app_rtt.c
 * @brief APP六路RTT波形；单一快环写入，缓冲满时丢整帧。
 */
#include "app.h"
#include "motor_axis.h"
#include "SEGGER_RTT.h"

/** @brief 六路波形：三相电流A、电角度rad、反馈转速rad/s、q轴电流A。 */
typedef struct
{
    float phase_a_a, phase_b_a, phase_c_a;
    float electrical_angle_rad;
    float speed_rad_s, iq_a;
} MotorAxisTrace;

static char s_rtt_buffer[4096];

/* 在快环末尾取得同一拍的只读快照。 */
static MotorAxisTrace read_trace(void)
{
    const FocState *foc = MotorAxis_Feedback.foc;
    const EncoderState *encoder = MotorAxis_Feedback.encoder;
    return (MotorAxisTrace){foc->phase_current_a.a,
                            foc->phase_current_a.b,
                            foc->phase_current_a.c,
                            Encoder_GetElecPhase(encoder),
                            Encoder_GetMechVelContinuous(encoder),
                            foc->current_dq_a.q};
}

void RTT_Init(void)
{
    (void)SEGGER_RTT_ConfigUpBuffer(1U,
                                    "JScope_f4f4f4f4f4f4",
                                    s_rtt_buffer,
                                    sizeof(s_rtt_buffer),
                                    SEGGER_RTT_MODE_NO_BLOCK_SKIP);
}

void RTT_Sampling(void)
{
    MotorAxisTrace trace = read_trace();
    (void)SEGGER_RTT_WriteSkipNoLock(1U, &trace, sizeof(trace));
}
