/**
 * @file app_rtt.c
 * @brief APP 电流和位置 RTT 波形；维护拍单写者，缓冲满时丢整帧。
 */
#include "app.h"
#include "motor_axis.h"
#include "SEGGER_RTT.h"

/* J-Scope按通道名解析数据类型：六路有符号int16，单帧12字节。 */
#define RTT_CHANNEL_NAME "JScope_i2i2i2i2i2i2"
/* 数据范围适配：三相电流±32.767A、电角度0..6.283rad、机械转速±327.67rad/s。 */
#define RTT_CURRENT_MA_PER_A 1000.0f
#define RTT_ANGLE_MRAD_PER_RAD 1000.0f
#define RTT_SPEED_CENTI_RAD_S_PER_RAD_S 100.0f

/** @brief 六路波形：三相电流mA、电角度mrad、机械转速0.01rad/s、q轴电流mA。 */
typedef struct
{
    int16_t data0; /* A相电流，mA */
    int16_t data1; /* B相电流，mA */
    int16_t data2; /* C相电流，mA */
    int16_t data3; /* 电角度，mrad */
    int16_t data4; /* 机械转速，0.01rad/s */
    int16_t data5; /* q轴电流，mA */
} RTT_Data_struct;

/* 供调试器/J-Scope代码变量观察；仅快环单写者写入。 */
RTT_Data_struct rtt_data;

/* 通道1上行缓冲；单写者，满时按RTT模式丢整帧。 */
static char s_rtt_buffer[4096];

/* 浮点换算后饱和到int16；截断取整，启动零偏未就绪的NaN记为0。 */
static int16_t scale_to_int16(float value, float counts_per_unit)
{
    float scaled = value * counts_per_unit;

    if (scaled != scaled)
    {
        return 0;
    }
    if (scaled >= 32767.0f)
    {
        return INT16_MAX;
    }
    if (scaled <= -32768.0f)
    {
        return INT16_MIN;
    }
    return (int16_t)scaled;
}

void RTT_Init(void)
{
    (void)SEGGER_RTT_ConfigUpBuffer(
        1U, RTT_CHANNEL_NAME, s_rtt_buffer, sizeof(s_rtt_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
}

void RTT_Sampling(void)
{
    const FocState *foc = MotorAxis_Feedback.foc;
    const EncoderState *encoder = MotorAxis_Feedback.encoder;
    const MotorControlState *motor = MotorAxis_Feedback.motor;
    float actual_rad = Encoder_GetMechPos(encoder);

    rtt_data.data0 = scale_to_int16(motor->position_reference_rad, 500.0f);
    rtt_data.data1 = scale_to_int16(actual_rad, 500.0f);
    rtt_data.data2 = scale_to_int16(motor->position_trajectory_speed_rad_s, 1000.0f);
    rtt_data.data3 = scale_to_int16(motor->position_velocity_filtered_rad_s, 1000.0f);
    rtt_data.data4 = scale_to_int16(motor->iq_target_a, 1000.0f);
    rtt_data.data5 = scale_to_int16(foc->current_dq_a.q, RTT_CURRENT_MA_PER_A);

    SEGGER_RTT_Write(1U, &rtt_data, sizeof(rtt_data));
}
