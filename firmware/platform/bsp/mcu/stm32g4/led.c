/* 只拥有状态GPIO和灯带DMA缓冲，繁忙时不改写在传输的数据，节拍由共用BSP决定。 */
#include "bsp_internal.h"
#include "main.h"
#include "tim.h"

static uint32_t s_pixel_buffer[48];
volatile uint32_t g_bsp_led_dma_busy;

void McuLed_Init(void)
{
    LED_R_GPIO_Port->BSRR = LED_R_Pin;
    LED_G_GPIO_Port->BSRR = LED_G_Pin;
}

void McuLed_WriteCode(BspLedCodeChannel channel, bool on)
{
    LED_R_GPIO_Port->BSRR = on && channel == BSP_LED_FAULT ? (uint32_t)LED_R_Pin << 16U : LED_R_Pin;
    LED_G_GPIO_Port->BSRR = on && channel == BSP_LED_MODE ? (uint32_t)LED_G_Pin << 16U : LED_G_Pin;
}

void McuLed_WriteColor(BspLedColor color, uint8_t brightness)
{
    if (HAL_TIM_GetChannelState(&htim2, TIM_CHANNEL_3) != HAL_TIM_CHANNEL_STATE_READY)
    {
        ++g_bsp_led_dma_busy;
        return;
    }
    static const uint8_t s_color_channels[] = {0U, 1U, 2U, 4U, 3U, 5U, 6U, 7U};
    uint8_t channels = s_color_channels[color];
    const uint8_t grb[3] = {(channels & 2U) ? brightness : 0U,
                            (channels & 1U) ? brightness : 0U,
                            (channels & 4U) ? brightness : 0U};
    for (unsigned i = 0U; i < 24U; ++i)
    {
        s_pixel_buffer[i] = ((255U - grb[i / 8U]) & (1U << (7U - i % 8U))) ? 58U : 136U;
        s_pixel_buffer[24U + i] = 0U;
    }
    (void)HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, s_pixel_buffer, 48U);
}
