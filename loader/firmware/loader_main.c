/* Loader最小组成层；所有擦写在前台，整个运行期间保持六路功率关断。 */
#include "loader_core.h"
#include "loader_link.h"
#include "stm32g4xx_hal.h"
#include <string.h>

static LoaderCore core;

void SysTick_Handler(void) { HAL_IncTick(); }

static void power_safe(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_WritePin(GPIOA, gpio.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_WritePin(GPIOB, gpio.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOB, &gpio);
}

static bool clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = RCC_PLLM_DIV2;
    osc.PLL.PLLN = 85;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = RCC_PLLQ_DIV2;
    osc.PLL.PLLR = RCC_PLLR_DIV2;
    clk.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    return HAL_RCC_OscConfig(&osc) == HAL_OK &&
           HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) == HAL_OK;
}

static uint8_t parameter_node(void)
{
    /* 只识别APP支持的0～7精确float32编码，不引入浮点库或迁移参数。 */
    static const uint32_t encoded[8] = {
        0, 0x3F800000UL, 0x40000000UL, 0x40400000UL,
        0x40800000UL, 0x40A00000UL, 0x40C00000UL, 0x40E00000UL
    };
    uint32_t node, bits, schema, magic;
    memcpy(&bits, (const void *)BOOT_PARAM_BASE, 4);
    memcpy(&schema, (const void *)(BOOT_PARAM_BASE + 2160), 4);
    memcpy(&magic, (const void *)(BOOT_PARAM_BASE + 2164), 4);
    if (schema == 10 && magic == 0x454E4332UL)
    {
        for (node = 0; node < 8; ++node)
        {
            if (bits == encoded[node]) { return (uint8_t)node; }
        }
    }
    return 0;
}

// clang-format off
__asm static void jump_tail(uint32_t sp, uint32_t reset)
{
    MSR MSP, r0
    MOVS r2, #0
    MSR CONTROL, r2
    MSR BASEPRI, r2
    MSR FAULTMASK, r2
    ISB
    CPSIE i
    BX r1
}
// clang-format on

static void jump_app(void)
{
    unsigned i;
    if (!LoaderCore_BootValid())
    {
        return;
    }
    __disable_irq();
    loader_link_shutdown();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    for (i = 0; i < 8; ++i)
    {
        NVIC->ICER[i] = UINT32_MAX;
        NVIC->ICPR[i] = UINT32_MAX;
    }
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    SCB->VTOR = BOOT_APP_BASE;
    __DSB();
    __ISB();
    jump_tail(*(const uint32_t *)BOOT_APP_BASE, *(const uint32_t *)(BOOT_APP_BASE + 4));
}

int main(void)
{
    ldr_rx_frame_t frame;
    uint8_t reply[64], mailbox[BOOT_MAILBOX_BYTES];
    uint32_t mail_node, start_ms, length, pending_since = 0;
    uint8_t node = parameter_node();
    bool stay, tx_pending = false;
    power_safe();
    HAL_Init();
    memcpy(mailbox, (const void *)BOOT_MAILBOX_BASE, sizeof(mailbox));
    *(volatile uint32_t *)(BOOT_MAILBOX_BASE + 28) = 0;
    stay = __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != 0 && __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) == 0 &&
           Boot_MailboxDecode(mailbox, &mail_node);
    if (stay && mail_node != node)
    {
        /* 身份冲突保持恢复；不会按邮箱改写参数节点。 */
        stay = true;
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();
    if (*(const uint16_t *)FLASHSIZE_BASE != 128 || !clock_init() || !loader_link_init(node))
    {
        for (;;)
        {
            power_safe();
        }
    }
    LoaderCore_Init(&core, node);
    start_ms = HAL_GetTick();
    for (;;)
    {
        if (loader_link_rx_pop(&frame))
        {
            length = LoaderCore_Request(&core, frame.data, frame.len, HAL_GetTick(), reply);
            if (length != 0 && loader_link_send(reply, (uint8_t)length) && core.activate_pending &&
                frame.data[2] == LDR_OP_ACTIVATE && reply[12] == 0 && reply[13] == 0)
            {
                tx_pending = true;
                pending_since = HAL_GetTick();
            }
        }
        if (core.session != 0)
        {
            stay = true;
        }
        LoaderCore_Tick(&core, HAL_GetTick());
        if (!core.activate_pending)
        {
            tx_pending = false;
        }
        if (tx_pending && loader_link_sent())
        {
            *(volatile uint32_t *)(BOOT_MAILBOX_BASE + 28) = 0;
            jump_app();
        }
        if (tx_pending && (uint32_t)(HAL_GetTick() - pending_since) > 100UL)
        {
            tx_pending = false;
        }
        if (!stay && (uint32_t)(HAL_GetTick() - start_ms) >= LDR_BOOT_WINDOW_MS)
        {
            stay = true;
            jump_app();
        }
    }
}
