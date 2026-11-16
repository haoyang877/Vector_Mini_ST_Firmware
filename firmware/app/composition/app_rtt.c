/**
 * @file app_rtt.c
 * @brief AT32 ADC 快拍的六槽 RTT 通道 1。
 */
#include "app.h"
#include "bsp_adc.h"
#include "SEGGER_RTT.h"

/* J-Scope 依次解析 Udc、Ibus、Ia、Ib、Ic 和采样序号。 */
#define RTT_CHANNEL_NAME "JScope_i2i2i2i2i2i2"

/* 仅 ADC 中断写入；环满丢弃整帧，接收端可从序号识别缺口。 */
static char s_rtt_buffer[1608];

void RTT_Init(void)
{
    (void)SEGGER_RTT_ConfigUpBuffer(
        1U, RTT_CHANNEL_NAME, s_rtt_buffer, sizeof(s_rtt_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
}

void RTT_Sampling(uint32_t sample_tick)
{
    /* AT32 三相原码为四次累加的平均，四舍五入到相同的 12 位 ADC 尺度。 */
    const int16_t frame[6] = {(int16_t)BspAdc_ReadRawChannel(BSP_ADC_BUS_VOLTAGE),
                              (int16_t)BspAdc_ReadRawChannel(BSP_ADC_BUS_CURRENT),
                              (int16_t)(BspAdc_ReadRawChannel(BSP_ADC_PHASE_CURRENT_A) + 0.5f),
                              (int16_t)(BspAdc_ReadRawChannel(BSP_ADC_PHASE_CURRENT_B) + 0.5f),
                              (int16_t)(BspAdc_ReadRawChannel(BSP_ADC_PHASE_CURRENT_C) + 0.5f),
                              (int16_t)(uint16_t)sample_tick};
    (void)SEGGER_RTT_WriteSkipNoLock(1U, frame, sizeof(frame));
}
