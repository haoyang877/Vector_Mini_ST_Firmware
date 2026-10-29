#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "bsp_encoder.h"
#include "bsp_internal.h"
#include "encoder_protocol.h"
uint32_t BspSystem_EnterCritical(void)
{
    return 0U;
}
void BspSystem_ExitCritical(uint32_t key)
{
    (void)key;
}
bool BspPwm_ArePhasesDisabled(void)
{
    return true;
}

/* 直接执行BSP全部函数，与独立编译的真实TLE协议组成完整三层链路。 */
#include "../../../firmware/platform/bsp/mcu/stm32g4/encoder_spi.c"

GPIO_TypeDef g_test_gpio;
static SPI_TypeDef s_spi;
SPI_HandleTypeDef hspi2 = {&s_spi, {0}};
static unsigned s_init_result;
static jmp_buf s_fatal;

unsigned HAL_SPI_Init(SPI_HandleTypeDef *spi)
{
    assert(spi == &hspi2);
    s_spi.CR1 |= SPI_CR1_SPE;
    return s_init_result;
}

void Error_Handler(void)
{
    longjmp(s_fatal, 1);
}

static void reset_bus(void)
{
    memset(&g_test_gpio, 0, sizeof(g_test_gpio));
    s_spi.CR1 = SPI_CR1_SPE;
    s_spi.SR = SPI_FLAG_TXE;
    s_spi.DR = 0xABCD0000U;
    g_test_gpio.MODER = 0xAAAAAAAAU;
    g_test_gpio.AFR[1] = 0x55555555U;
}

static void check_released(void)
{
    assert(g_test_gpio.BSRR == BRD_ENC_CS_Pin);
    assert((g_test_gpio.MODER >> 30) == 2U);
    assert((g_test_gpio.AFR[1] >> 28) == 5U);
}

int main(void)
{
    reset_bus();
    assert(EncoderSpi_Init());
    assert(hspi2.Init.DataSize == 16U && hspi2.Init.CLKPolarity == 0U);
    assert(hspi2.Init.CLKPhase == 2U && hspi2.Init.BaudRatePrescaler == 32U);
    s_init_result = 1U;
    if (setjmp(s_fatal) == 0)
    {
        (void)EncoderSpi_Init();
        assert(!"HAL failure must enter Error_Handler");
    }
    s_init_result = 0U;
    assert(Encoder_Init());
    for (unsigned enabled = 0U; enabled < 2U; ++enabled)
    {
        for (unsigned flags = 0U; flags < 256U; ++flags)
        {
            reset_bus();
            s_spi.CR1 = enabled ? SPI_CR1_SPE : 0U;
            s_spi.SR = flags;
            bool expected =
                enabled && (flags & (SPI_FLAG_TXE | SPI_FLAG_RXNE | SPI_FLAG_BSY | SPI_FLAG_OVR)) ==
                               SPI_FLAG_TXE;
            assert(EncoderSpi_Begin(&(EncoderSpiRequest){0x8021U, 2U}) == expected);
            assert(g_test_gpio.BSRR == (expected ? (BRD_ENC_CS_Pin << 16) : 0U));
        }
    }
    for (unsigned word = 0U; word < 65536U; ++word)
    {
        reset_bus();
        assert(BspEncoder_Begin() == BSP_OK);
        assert((s_spi.DR & 0xFFFFU) == 0x8021U);
        s_spi.SR = SPI_FLAG_RXNE | SPI_FLAG_TXE;
        BspResult stage = BspEncoder_BeginReadback();
        assert(stage == BSP_OK && (g_test_gpio.MODER >> 30) == 0U);
        assert((s_spi.DR & 0xFFFFU) == 0U);
        s_spi.DR = word;
        BspEncoderSample sample;
        assert(BspEncoder_Complete(&sample) == BSP_OK);
        assert(sample.angle_counts == (uint16_t)((word & 0x7FFFU) << 1));
        BspEncoderDiagnostics diagnostic;
        assert(BspEncoder_ReadDiagnostics(&diagnostic) == BSP_OK && diagnostic.frame_word == word &&
               diagnostic.valid_mask == BSP_ENCODER_DIAG_FRAME_VALID && !diagnostic.crc_checked);
        check_released();
    }
    for (unsigned failure = 0U; failure < 4U; ++failure)
    {
        reset_bus();
        assert(BspEncoder_Begin() == BSP_OK);
        s_spi.SR = failure == 0U   ? SPI_FLAG_TXE
                   : failure == 1U ? SPI_FLAG_RXNE
                                   : SPI_FLAG_RXNE | SPI_FLAG_TXE;
        (void)BspEncoder_BeginReadback();
        if (failure == 2U)
        {
            s_spi.SR = SPI_FLAG_TXE;
        }
        if (failure == 3U)
        {
            s_spi.SR |= SPI_FLAG_BSY;
        }
        BspEncoderSample sample = {0x1357U, 0x2468U};
        assert(BspEncoder_Complete(&sample) == BSP_TIMEOUT);
        assert(sample.angle_counts == 0x1357U && sample.sequence == 0x2468U);
        check_released();
    }
    reset_bus();
    assert(BspEncoder_Begin() == BSP_OK);
    s_spi.SR = SPI_FLAG_RXNE | SPI_FLAG_TXE;
    (void)BspEncoder_BeginReadback();
    assert(BspEncoder_Complete(NULL) == BSP_INVALID_ARGUMENT);
    check_released();
    reset_bus();
    s_spi.CR1 = 0U;
    s_spi.SR = SPI_FLAG_RXNE | SPI_FLAG_TXE | SPI_FLAG_OVR;
    EncoderSpiRequest request = {0x8061U, 2U};
    uint8_t bytes[2] = {0x13U, 0x57U};
    assert(EncoderSpi_Read(&request, bytes) == ENCODER_SPI_READ_OK);
    assert(s_spi.CR1 & SPI_CR1_SPE);
    assert(bytes[0] == 0U && bytes[1] == 0U);
    check_released();
    reset_bus();
    assert(EncoderSpi_Read(&request, bytes) == ENCODER_SPI_READ_ERROR);
    check_released();
    puts("PASS STM32 actual SPI + TLE chain: 65536 pipelined frames, 512 readiness states, "
         "timeout/MOSI/CS/init recovery");
    return 0;
}
