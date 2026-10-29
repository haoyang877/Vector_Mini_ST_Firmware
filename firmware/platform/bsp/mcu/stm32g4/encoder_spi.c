#include "encoder_spi.h"
#include "bsp_config.h"

#include "main.h"
#include "spi.h"

/* 编码器 SPI 端口：唯一访问 SPI2、片选与 MOSI 方向的实现。
 * 时序、超时常数与表达式与去耦前的 bsp/encoder.c 完全一致。 */

#define ENCODER_SPI_HANDLE hspi2
#define ENCODER_SPI_CS_ENABLE                                                                      \
    BSP_ENCODER_GPIO_PORT->BSRR = (uint32_t)(1U << BSP_ENCODER_CS_PIN) << 16U
#define ENCODER_SPI_CS_DISABLE BSP_ENCODER_GPIO_PORT->BSRR = (1U << BSP_ENCODER_CS_PIN)

#ifndef ENC_SPI_XFER_SPIN_MAX
#define ENC_SPI_XFER_SPIN_MAX 340U
#endif

static bool spi_wait_flag(SPI_TypeDef *SPIx, uint32_t flag, uint32_t timeout_spin)
{
    uint32_t spin = 0U;

    while ((SPIx->SR & flag) == 0U)
    {
        if (++spin > timeout_spin)
        {
            return false;
        }
    }
    return true;
}

static bool spi_wait_idle(SPI_TypeDef *SPIx, uint32_t timeout_spin)
{
    uint32_t spin = 0U;

    while ((SPIx->SR & SPI_FLAG_BSY) != 0U)
    {
        if (++spin > timeout_spin)
        {
            return false;
        }
    }
    return true;
}

static uint16_t spi_read_word(SPI_TypeDef *SPIx, bool *ok)
{
    uint16_t rx_data;

    if (!spi_wait_flag(SPIx, SPI_FLAG_RXNE, ENC_SPI_XFER_SPIN_MAX))
    {
        *ok = false;
        return 0U;
    }
    rx_data = *(__IO uint16_t *)&SPIx->DR;
    if (!spi_wait_idle(SPIx, ENC_SPI_XFER_SPIN_MAX))
    {
        *ok = false;
        return 0U;
    }
    *ok = true;
    return rx_data;
}

static bool spi_start_word(SPI_TypeDef *SPIx, uint16_t tx_data)
{
    if ((SPIx->CR1 & SPI_CR1_SPE) == 0U)
    {
        SPIx->CR1 |= SPI_CR1_SPE;
    }

    if ((SPIx->SR & SPI_FLAG_OVR) != 0U)
    {
        (void)*(__IO uint16_t *)&SPIx->DR;
        (void)SPIx->SR;
    }

    if (!spi_wait_flag(SPIx, SPI_FLAG_TXE, ENC_SPI_XFER_SPIN_MAX))
    {
        return false;
    }
    *(__IO uint16_t *)&SPIx->DR = tx_data;
    return true;
}

static uint16_t spi_transfer_word(SPI_TypeDef *SPIx, uint16_t tx_data, bool *ok)
{
    if (!spi_start_word(SPIx, tx_data))
    {
        *ok = false;
        return 0U;
    }
    return spi_read_word(SPIx, ok);
}

/* TLE/SSC 类传感器在读相位由传感器驱动数据线，主机须先释放 MOSI。 */
static void spi_mosi_release(void)
{
    BSP_ENCODER_GPIO_PORT->MODER &= ~(0x3UL << (2U * BSP_ENCODER_MOSI_PIN));
}

static void spi_mosi_restore(void)
{
    BSP_ENCODER_GPIO_PORT->MODER =
        (BSP_ENCODER_GPIO_PORT->MODER & ~(0x3UL << (2U * BSP_ENCODER_MOSI_PIN))) |
        (0x2UL << (2U * BSP_ENCODER_MOSI_PIN));
    BSP_ENCODER_GPIO_PORT->AFR[BSP_ENCODER_MOSI_PIN / 8U] =
        (BSP_ENCODER_GPIO_PORT->AFR[BSP_ENCODER_MOSI_PIN / 8U] &
         ~(0xFUL << (4U * (BSP_ENCODER_MOSI_PIN % 8U)))) |
        (BSP_ENCODER_GPIO_AF << (4U * (BSP_ENCODER_MOSI_PIN % 8U)));
}

bool EncoderSpi_Init(void)
{
    SPI_HandleTypeDef *encoder_spi = &ENCODER_SPI_HANDLE;

    encoder_spi->Init.DataSize = SPI_DATASIZE_16BIT;
    encoder_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
    encoder_spi->Init.CLKPhase = SPI_PHASE_2EDGE;
    encoder_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    if (HAL_SPI_Init(encoder_spi) != HAL_OK)
    {
        Error_Handler();
    }
    return true;
}

bool EncoderSpi_Begin(const EncoderSpiRequest *request)
{
    SPI_TypeDef *SPIx = ENCODER_SPI_HANDLE.Instance;

    /* 不抢在保护之前等待：总线状态异常时保留同步回退路径。 */
    if ((SPIx->CR1 & SPI_CR1_SPE) == 0U ||
        (SPIx->SR & (SPI_FLAG_TXE | SPI_FLAG_OVR | SPI_FLAG_BSY | SPI_FLAG_RXNE)) != SPI_FLAG_TXE)
    {
        return false;
    }
    ENCODER_SPI_CS_ENABLE;
    *(__IO uint16_t *)&SPIx->DR = request->command;
    return true;
}

uint8_t EncoderSpi_BeginReadback(bool begin_ok)
{
    SPI_TypeDef *SPIx = ENCODER_SPI_HANDLE.Instance;
    bool transfer_ok;

    if (!begin_ok)
    {
        return ENCODER_SPI_READ_SYNC;
    }
    (void)spi_read_word(SPIx, &transfer_ok);
    if (!transfer_ok)
    {
        return ENCODER_SPI_READ_FAILED;
    }
    spi_mosi_release();
    __NOP();
    __NOP();
    if (!spi_start_word(SPIx, 0U))
    {
        spi_mosi_restore();
        return ENCODER_SPI_READ_FAILED;
    }
    return ENCODER_SPI_READ_RESPONSE_PENDING;
}

bool EncoderSpi_Complete(uint8_t sample_stage, const EncoderSpiRequest *request, uint8_t *data)
{
    SPI_TypeDef *SPIx = ENCODER_SPI_HANDLE.Instance;
    uint16_t received = 0U;
    bool transfer_ok = true;

    if (sample_stage == ENCODER_SPI_READ_RESPONSE_PENDING)
    {
        received = spi_read_word(SPIx, &transfer_ok);
        spi_mosi_restore();
    }
    else if (sample_stage == ENCODER_SPI_READ_FAILED)
    {
        transfer_ok = false;
    }
    else
    {
        if (sample_stage == ENCODER_SPI_READ_COMMAND_PENDING)
        {
            (void)spi_read_word(SPIx, &transfer_ok);
        }
        else
        {
            ENCODER_SPI_CS_ENABLE;
            (void)spi_transfer_word(SPIx, request->command, &transfer_ok);
        }
        if (transfer_ok)
        {
            spi_mosi_release();
            __NOP();
            __NOP();
            received = spi_transfer_word(SPIx, 0U, &transfer_ok);
            spi_mosi_restore();
        }
    }
    (void)spi_wait_idle(SPIx, ENC_SPI_XFER_SPIN_MAX);
    ENCODER_SPI_CS_DISABLE;

    if (!transfer_ok)
    {
        return false;
    }
    data[0] = (uint8_t)(received >> 8);
    data[1] = (uint8_t)received;
    return true;
}

/* 同步寄存器只读：复用完整读回路径，起始 begin_ok=false 走同步补发。 */
EncoderSpiReadStatus EncoderSpi_Read(const EncoderSpiRequest *request, uint8_t *data)
{
    return EncoderSpi_Complete(ENCODER_SPI_READ_SYNC, request, data) ? ENCODER_SPI_READ_OK
                                                                     : ENCODER_SPI_READ_ERROR;
}
