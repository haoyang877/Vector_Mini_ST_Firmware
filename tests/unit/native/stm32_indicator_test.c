/* 相同2kHz输入调度，对照冻结的原GPIO/GRB输出；另测非法输入和忙DMA。 */
#include "main.h"
#include "tim.h"
#include <assert.h>
#include <stdio.h>
#ifdef LEGACY_INDICATOR
#include "indicator_hw.h"
#define TestColor IndicatorHwColor
#else
#include "bsp_led.h"
#include "bsp_internal.h"
#define TestColor BspLedColor
#endif
IndicatorGpio g_indicator_red, g_indicator_green;
unsigned htim2;
static uint32_t s_signature = 2166136261U, s_mask;
static unsigned s_dma_calls;
static bool s_dma_busy;
uint32_t BspSystem_EnterCritical(void)
{
    uint32_t key = s_mask;
    s_mask = 1;
    return key;
}
void BspSystem_ExitCritical(uint32_t key)
{
    s_mask = key;
}
unsigned HAL_TIM_GetChannelState(unsigned *h, unsigned ch)
{
    assert(h == &htim2 && ch == TIM_CHANNEL_3);
    return s_dma_busy ? 1U : HAL_TIM_CHANNEL_STATE_READY;
}
static void record(uint32_t v)
{
    s_signature = (s_signature ^ v) * 16777619U;
}
int HAL_TIM_PWM_Start_DMA(unsigned *h, unsigned ch, uint32_t *data, unsigned length)
{
    assert(h == &htim2 && ch == TIM_CHANNEL_3 && length == 48U);
    for (unsigned i = 0; i < length; i++)
    {
        assert(i < 24 ? data[i] == 58U || data[i] == 136U : data[i] == 0U);
        record(data[i]);
    }
    ++s_dma_calls;
    return 0;
}
int main(void)
{
#ifndef LEGACY_INDICATOR
    assert(BspLed_SetCode(BSP_LED_MODE, 1U) == BSP_NOT_READY);
    Led_Init();
#endif
    for (unsigned error = 0; error < 2; error++)
        for (unsigned count = 0; count < 256; count++)
        {
#ifdef LEGACY_INDICATOR
            indicator_hw_set_led(error != 0U, (uint8_t)count);
#else
            assert(BspLed_SetCode(error ? BSP_LED_FAULT : BSP_LED_MODE, (uint8_t)count) == BSP_OK);
#endif
            for (unsigned period = 0; period < 56; period++)
                for (unsigned tick = 0; tick < 400; tick++)
                {
                    TestColor color = (TestColor)((period + count) % 8U);
#ifdef LEGACY_INDICATOR
                    if (tick == 399)
                    {
                        indicator_hw_led_task();
                    }
                    if (tick % 100 == 99)
                    {
                        indicator_hw_set_color(color);
                    }
#else
                    if (tick % 100 == 99)
                    {
                        assert(BspLed_SetColor(color) == BSP_OK);
                    }
                    Led_SupervisorTick();
#endif
                    if (tick == 399)
                    {
                        record(g_indicator_red.BSRR);
                        record(g_indicator_green.BSRR);
                    }
                }
        }
    printf("%08x %u\n", s_signature, s_dma_calls);
#ifndef LEGACY_INDICATOR
    unsigned previous = s_dma_calls;
    s_dma_busy = true;
    for (unsigned i = 0; i < 100; i++)
    {
        Led_SupervisorTick();
    }
    assert(s_dma_calls == previous);
    s_dma_busy = false;
    assert(BspLed_SetColor((BspLedColor)99) == BSP_INVALID_ARGUMENT);
    assert(BspLed_SetCode((BspLedCodeChannel)99, 1) == BSP_INVALID_ARGUMENT);
    assert(s_mask == 0U);
#endif
    return 0;
}
