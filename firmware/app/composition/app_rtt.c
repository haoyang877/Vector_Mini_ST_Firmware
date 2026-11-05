/**
 * @file app_rtt.c
 * @brief APP 外环 RTT 波形；通道 1 单写者，缓冲满时丢整帧。
 */
#include "app.h"
#include "bsp_adc.h"
#include "motor_axis.h"
#include "SEGGER_RTT.h"

/* J-Scope按通道名解析数据类型：八路有符号int16，单帧16字节。 */
#define RTT_CHANNEL_NAME "JScope_i2i2i2i2i2i2i2i2"
#define RTT_HALL_MRAD_PER_RAD 1000.0f

/** @brief 通道1保留前五个槽，后三个槽发送霍尔角度和两路原码。 */
typedef struct
{
    int16_t reserved[5];
    int16_t data5;              /* 输出轴单圈角，mrad */
    int16_t hall_sin, hall_cos; /* ADC原码，0..4095 */
} RTT_Data_struct;

/* 供调试器观察；仅由2 kHz监督回调写入。 */
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
    rtt_data.data5 =
        scale_to_int16(LinearHall_getAngle(MotorAxis_Feedback.linear_hall), RTT_HALL_MRAD_PER_RAD);
    rtt_data.hall_sin = (int16_t)BspAdc_ReadRawChannel(BSP_ADC_LINEAR_HALL_SIN);
    rtt_data.hall_cos = (int16_t)BspAdc_ReadRawChannel(BSP_ADC_LINEAR_HALL_COS);

    SEGGER_RTT_Write(1U, &rtt_data, sizeof(rtt_data));
}
